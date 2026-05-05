#include "GameManage.h"
#include "tuning_config.h"
#include "ChassisControl.h"
#include "PoseEstimate.h"
#include "Vision.h"
#include "CoreScheduler.h"
#include "Tracker.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include "zf_common_headfile.h"


namespace App::GameEngine {

__attribute__((section(".dtcm_data"))) static DemoGameManager core_engine;

//===================================================================
// GameEngine 模块对外接口实现
// ==================================================================

// 初始化拨码开关并读取运行模式：赛段模式 + 演示模式
void init() {
    gpio_init(C27, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(C26, GPI, GPIO_HIGH, GPI_PULL_UP);
    system_delay_ms(10); 

    bool sw1_on = !gpio_get_level(C26); 
    bool sw2_on = !gpio_get_level(C27); 

    App::g_state.game.is_advanced_stage = sw1_on;  // 赛段模式：开-第二/三阶段（巡图+推箱），关-第一阶段（仅推箱）
    App::g_state.game.is_debug_mode     = sw2_on;  // 调试模式：开-直接注入地图数据，绕过视觉输入；关-正常模式，等待视觉输入

    if (App::g_state.game.is_debug_mode) {
        App::g_state.game.phase = GamePhase::NONE;  // 直接进入初始状态，UI 选完地图后再切到正常流程
    }

    App::g_state.debug.need_bg_redraw = true;
}

// 全局入口：基于状态机的静态多态派发
__attribute__((section(".ramfunc"))) void update() {
    auto& game = App::g_state.game;

    // 【机制挂起】：如果是 Debug 模式，在 UI 选完地图前，强制挂起状态机，不进入物理控制流
    if (game.is_debug_mode && game.phase == GamePhase::NONE) {
        return; 
    }

    if (game.is_demo_mode) {
        core_engine.update();                   // 纯动画推演 (Demo)
    } else if (game.is_debug_mode) {
        core_engine.MockGameManager::update();  // 实体车+脱机地图+注入语义 (Mock)
    } else {
        core_engine.GameManager::update();      // 正常比赛流程 (Prod)
    }
}

// 获取当前渲染上下文
RenderContext get_render_context() { 
    return core_engine.get_render_context();
}


//===================================================================
// GameManager 基类实现
// ==================================================================

// 全局业务状态机更新函数：根据当前阶段执行对应逻辑
__attribute__((section(".ramfunc"))) void GameManager::update() {
    auto& pos = App::g_state.physical.pose;
    auto& ctrl = App::g_state.control;       
    auto& game = App::g_state.game;          
    auto& vision_data = App::g_state.vision;
    
    switch (game.phase) {
        case GamePhase::INIT_CALIBRATE: {
            // 将里程计重置到已知入口位姿
            Subsystem::PoseEstimator::set_position(ENTRY_X, ENTRY_Y, ENTRY_YAW);
            
            // 直接下发出库目标点，切到手动目标模式执行离场
            ctrl.current_target = {OUT_TARGET_X, OUT_TARGET_Y, ENTRY_YAW};
            ctrl.mode = ControlMode::MANUAL_DEBUG;

            game.phase = GamePhase::EXIT_START_ZONE;
            break;
        }

        case GamePhase::EXIT_START_ZONE: {
            // 监控是否到达出库点
            bool arrived = Algorithm::Tracker::check_arrival({OUT_TARGET_X, OUT_TARGET_Y}, tune.tracker.reach_radius_min);

            // 到位后请求视觉模块 ART1 返回地图数据，进入等待状态
            if (arrived) {
                Subsystem::Vision::request_map_ART1();   // 请求 ART1 地图
                game.phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        case GamePhase::WAIT_FOR_VISION: {
            
            if (vision_data.art1_map_ready) {
                // 地图帧就绪，将视觉数据转换加载到逻辑地图结构中，供后续算法使用
                logical_level.map = vision_data.map;
                logical_level.player_start = {PLAN_START_X, PLAN_START_Y};
                logical_level.box_count = vision_data.box_count;
                logical_level.target_count = vision_data.box_count;
                logical_level.bomb_count = vision_data.bomb_count;
                
                for(int i=0; i<logical_level.box_count; ++i) {
                    logical_level.boxes[i] = vision_data.boxes[i];
                    logical_level.targets[i] = vision_data.targets[i];
                }
                for(int i=0; i<logical_level.bomb_count; ++i) {
                    logical_level.bombs[i] = vision_data.bombs[i];
                }

                // 异步请求位姿，用于后续全局定位校准
                Subsystem::Vision::request_pose_ART1(); 

                if (game.is_advanced_stage) {  
                    patrol_planner.load_level(logical_level);  // 将视觉数据加载到巡图规划引擎
                    game.phase = GamePhase::PLAN_PATROL;       // 进入巡图
                } else {
                    solver.load_from_vision(logical_level);    // 将视觉数据加载到推箱求解器
                    game.phase = GamePhase::PLAN_SOKOBAN;      // 直接进入推箱子阶段
                    // game.phase = GamePhase::NONE; 
                }
            } else {
                // 未接收到地图帧，超时重试请求地图数据
                static uint32_t last_request_tick = Core::Scheduler::get_sys_tick_ms();
                
                if (Core::Scheduler::get_sys_tick_ms() - last_request_tick > 1000) {
                    last_request_tick = Core::Scheduler::get_sys_tick_ms();
                    Subsystem::Vision::request_map_ART1();  
                }
            }
            break;
        }

        case GamePhase::PLAN_PATROL: {
            // 先解算炸弹任务，再做联合巡图规划（无炸弹时自动退化）
            auto& bombs = App::g_state.planning.bomb_tasks;
            bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE1_ANY>(logical_level);

            patrol_actions = patrol_planner.plan_optimal_patrol(logical_level.player_start, bombs);

            game.action_idx = 0;
            Subsystem::Vision::reset_semantic_labels();   // 清空语义池，准备重新填充
            game.phase = GamePhase::EXEC_ACTION_DISPATCH;
            break;
        }

        case GamePhase::EXEC_ACTION_DISPATCH: {

            if (game.action_idx >= patrol_actions.size()) {
                // 全部宏动作执行完毕，进入语义绑定阶段
                game.phase = GamePhase::BIND_SEMANTICS;
            } else {
                // 获取当前宏动作信息
                const auto& act = patrol_actions[game.action_idx];
                StaticArray<point, MAX_PATH_LENGTH> segment;

                // 按动作类型生成段路径：观测路径 / 推炸路径
                bool found = act.is_bomb_task ? 
                             patrol_planner.get_bomb_push_path(logical_level, logical_level.player_start, act.bomb, segment) :
                             patrol_planner.get_grid_path(logical_level, logical_level.player_start, act.obs.pos, segment);

                if (found) {
                    Algorithm::Tracker::load_path(segment);
                    ctrl.mode = ControlMode::AUTO_TRACKING; // 开启自动循迹执行该段路径
                    game.phase = act.is_bomb_task ? GamePhase::EXEC_BOMB_PUSH : GamePhase::EXEC_PATROL_MOVE;
                } else {
                    game.error_stage = 1; // 错误阶段1：寻图路径生成失败
                    game.phase = GamePhase::ERROR_OCCURRED; 
                }
            }
            break;
        }

        case GamePhase::EXEC_PATROL_MOVE: {
            // 等待到达观测点
            if (ctrl.tracker_state == TrackerState::FINISHED) {
                game.phase = GamePhase::EXEC_ALIGN_YAW;
            }
            break;
        }

        case GamePhase::EXEC_ALIGN_YAW: {
            // 到位后原地对齐朝向，准备触发 ART2 抓拍
            ctrl.current_target.yaw = patrol_actions[game.action_idx].obs.target_yaw;  
            ctrl.mode = ControlMode::MANUAL_DEBUG; // 停止循迹，仅执行角度对齐

            // 检查 Yaw 角度误差是否小于 2 度
            float current_yaw = App::g_state.physical.pose.yaw;
            float err_yaw = std::abs(ctrl.current_target.yaw - current_yaw);
            if (err_yaw > 180.0f) err_yaw = 360.0f - err_yaw;

            // 朝向收敛后触发 ART2 捕捉
            if (err_yaw < 2.0f) { 

                system_delay_ms(1000);   
                // 请求 ART2 捕捉该观测点对应实体的语义标签
                uint8_t current_entity = patrol_actions[game.action_idx].obs.entity_id;
                bool is_box = patrol_actions[game.action_idx].obs.is_box;
                Subsystem::Vision::request_capture_ART2(current_entity, is_box);
                game.phase = GamePhase::WAIT_ART2_CAPTURE_ACK;
            }
            break;
        }

        case GamePhase::WAIT_ART2_CAPTURE_ACK: {
            // Subsystem::Vision::test_loopback_art2_ack(); // ~~~调试阶段短接 ACK 测试~~~

            // 等待 ART2 捕捉 ACK；ACK 到达即进入下一个宏动作
            if (vision_data.capture_ack_received) {
                vision_data.capture_ack_received = false;
                // 更新逻辑位置为当前观测点
                logical_level.player_start = patrol_actions[game.action_idx].obs.pos; 
                game.action_idx++;
                game.phase = GamePhase::EXEC_ACTION_DISPATCH;
            }
            break;
        }

        case GamePhase::EXEC_BOMB_PUSH: {
            // 等待推炸弹宏动作执行完毕
            if (ctrl.tracker_state == TrackerState::FINISHED) {
                game.phase = GamePhase::UPDATE_MAP;
            }
            break;
        }

        case GamePhase::UPDATE_MAP: {
            point tw = patrol_actions[game.action_idx].bomb.target_wall;

            // 摧毁墙壁
            for(int dy=-1; dy<=1; dy++) for(int dx=-1; dx<=1; dx++) {
                if (tw.y+dy > 0 && tw.y+dy < MAP_MAX_HEIGHT-1 && tw.x+dx > 0 && tw.x+dx < MAP_MAX_WIDTH-1)
                    logical_level.map[tw.y+dy][tw.x+dx] = 0;
            }

            // 销该炸弹
            for (int i = 0; i < logical_level.bomb_count; ++i) {
                if (logical_level.bombs[i].x == patrol_actions[game.action_idx].bomb.bomb_start.x &&
                    logical_level.bombs[i].y == patrol_actions[game.action_idx].bomb.bomb_start.y) {
                    logical_level.bombs[i] = {-1, -1};
                    break;
                }
            }

            // 将当前物理位姿投影到网格坐标，刷新逻辑位置
            const auto& real_pos = App::g_state.physical.pose;
            int8_t grid_x = static_cast<int8_t>((real_pos.x - MAP_OFFSET_X) / GRID_SIZE_CM + 0.5f);  // static_cast会直接截断小数，这里的 +0.5 是为了四舍五入
            int8_t grid_y = static_cast<int8_t>((real_pos.y - MAP_OFFSET_Y) / GRID_SIZE_CM + 0.5f);
            if (grid_x < 0) grid_x = 0; else if (grid_x >= MAP_MAX_WIDTH) grid_x = MAP_MAX_WIDTH - 1;
            if (grid_y < 0) grid_y = 0; else if (grid_y >= MAP_MAX_HEIGHT) grid_y = MAP_MAX_HEIGHT - 1;
            logical_level.player_start = {grid_x, grid_y};

            game.action_idx++;
            game.phase = GamePhase::EXEC_ACTION_DISPATCH;
            break;
        }

        case GamePhase::BIND_SEMANTICS: {
            bool all_done = true;

            // 检查所有“观测动作”对应实体是否已写入语义标签
            for (int i = 0; i < patrol_actions.size(); i++) {
                if(patrol_actions[i].is_bomb_task) continue; // 跳过炸弹任务

                uint8_t visited_entity_id = patrol_actions[i].obs.entity_id; 

                // 有任一实体还未出语义结果，则继续等待
                if (vision_data.semantic_labels[visited_entity_id] == -1) {
                    all_done = false;
                    break;
                }
            }

            if (all_done) {
                // N-1 规则匹配箱子与目标点 ID
                uint8_t matched_ids[SystemConfig::MAX_BOXES];
                bool is_perfect = patrol_planner.match_semantics(vision_data.semantic_labels, matched_ids);
                
                if (!is_perfect) {
                    game.error_stage = 4; // 错误阶段4：语义匹配失败（不满足 N-1 规则）
                    game.phase = GamePhase::ERROR_OCCURRED;
                    break;
                }

                // 二次炸弹解算（附带语义信息）
                auto& bombs = App::g_state.planning.bomb_tasks;
                bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE2_SPECIFIC>(logical_level);

                solver.load_from_vision(logical_level);   // 导入当前地形（含爆炸改动与小车位置）
                solver.bind_semantics(matched_ids);       // 绑定语义映射与当前位置
                solver.load_bomb_tasks(bombs.data(), bombs.size()); // 加载炸弹任务（如果有的话）
                game.phase = GamePhase::PLAN_SOKOBAN;
            }
            break;
        }

        case GamePhase::PLAN_SOKOBAN: {
            // 按赛段调用不同求解模式
            bool success = game.is_advanced_stage ? solver.solve(GameMode::PHASE2_SPECIFIC) : solver.solve(GameMode::PHASE1_ANY);

            if (success) {
                // 求解成功：加载路径并启动自动执行
                Algorithm::Tracker::load_path(solver.get_result_path());
                ctrl.mode = ControlMode::AUTO_TRACKING;
                game.phase = GamePhase::EXEC_SOKOBAN;
            } else {
                game.error_stage = 2; // 错误阶段2：推箱子路径求解失败
                game.phase = GamePhase::ERROR_OCCURRED;
            }
            break;
        }

        case GamePhase::EXEC_SOKOBAN: {
            // 等待推箱路径执行完成
            if (ctrl.tracker_state == TrackerState::FINISHED) {

                // 同步逻辑终点
                if (!App::g_state.planning.grid_path.empty()) {
                    logical_level.player_start = App::g_state.planning.grid_path.back();  
                }

                game.phase = GamePhase::PLAN_RETURN_HOME;
            }
            break;
        }

        case GamePhase::PLAN_RETURN_HOME: {
            ctrl.current_target = {IN_TARGET_X, IN_TARGET_Y, ENTRY_YAW};
            ctrl.mode = ControlMode::MANUAL_DEBUG;
            break;
        }

        case GamePhase::EXEC_RETURN_HOME: {
            if (Algorithm::Tracker::check_arrival({IN_TARGET_X, IN_TARGET_Y}, tune.tracker.reach_radius_min)) {
                game.phase = GamePhase::FINISHED;
            }
            break;
        }

        case GamePhase::FINISHED: {
            // 完成态：保持原地
            ctrl.current_target = {pos.x, pos.y, SystemConfig::ENTRY_YAW};
            ctrl.mode = ControlMode::MANUAL_DEBUG;
            break;
        }

        case GamePhase::ERROR_OCCURRED: {
            // 错误态：立即停车并保持姿态
            ctrl.current_target = {pos.x, pos.y, SystemConfig::ENTRY_YAW};
            ctrl.mode = ControlMode::MANUAL_DEBUG;
            break;
        }

        default:
            break;
    }
}


} // namespace App::GameEngine