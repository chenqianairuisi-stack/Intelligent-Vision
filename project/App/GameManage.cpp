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

// 倒数计算宏区（利用编译期算好替代除法）
constexpr float INV_360 = 1.0f / 360.0f;
constexpr float INV_GRID_SIZE_CM = 1.0f / SystemConfig::GRID_SIZE_CM;

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
        // =============================================================================
        // ---- 阶段一：启动出库与建图 ----
        // =============================================================================
        case GamePhase::INIT_CALIBRATE: {
            Subsystem::PoseEstimator::set_position(ENTRY_X, ENTRY_Y, ENTRY_YAW);
            Algorithm::Tracker::track_point({OUT_TARGET_X, OUT_TARGET_Y, ENTRY_YAW});

            game.phase = GamePhase::EXIT_START_ZONE;
            break;
        }

        case GamePhase::EXIT_START_ZONE: {
            // 到位后请求视觉模块 ART1 返回地图数据，进入等待状态
            if (Algorithm::Tracker::check_arrival({OUT_TARGET_X, OUT_TARGET_Y}, tune.tracker.reach_radius_min)) {

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

        // =============================================================================
        // ---- 阶段二：巡图与任务派发 ----
        // =============================================================================
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
                // 获取当前宏动作信息并分发任务
                const auto& act = patrol_actions[game.action_idx];
                task_queue.clear();
                current_task_idx = 0;

                // 将宏任务转化为精锐极简的微指令流队列
                if (act.is_bomb_task) {
                    task_queue.push_back(RobotTask::make_path_bomb(act.bomb));
                    task_queue.push_back(RobotTask::make_wait_track());
                    task_queue.push_back(RobotTask::make_update_map(act.bomb.bomb_start, act.bomb.target_wall));
                } else {
                    task_queue.push_back(RobotTask::make_path_obs(act.obs.pos));
                    task_queue.push_back(RobotTask::make_wait_track());
                    task_queue.push_back(RobotTask::make_align(act.obs.target_yaw));
                    task_queue.push_back(RobotTask::make_capture(act.obs.entity_id, act.obs.is_box));
                }
                game.phase = GamePhase::EXEC_TASK_QUEUE; // 转移到微观流水线
            }
            break;
        }

        case GamePhase::EXEC_TASK_QUEUE: {
            if (current_task_idx >= task_queue.size()) {
                game.action_idx++;
                game.phase = GamePhase::EXEC_ACTION_DISPATCH; // 当前宏任务完成，切回 DISPATCH
                break;
            }

            auto& task = task_queue[current_task_idx];
            bool task_done = false;

            switch (task.type) {
                case TaskType::LOAD_PATH_BOMB: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    if (patrol_planner.get_bomb_push_path(logical_level, logical_level.player_start, task.param.bomb, segment)) {
                        Algorithm::Tracker::load_path(segment);
                        task_done = true;
                    } else {
                        game.error_stage = 1; game.phase = GamePhase::ERROR_OCCURRED;
                    }
                    break;
                }
                case TaskType::LOAD_PATH_OBS: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    if (patrol_planner.get_grid_path(logical_level, logical_level.player_start, task.param.target_grid, segment)) {
                        Algorithm::Tracker::load_path(segment);
                        task_done = true;
                    } else {
                        game.error_stage = 1; game.phase = GamePhase::ERROR_OCCURRED;
                    }
                    break;
                }
                case TaskType::WAIT_TRACKING_DONE: {
                    if (ctrl.tracker_state == TrackerState::FINISHED) task_done = true;
                    break;
                }
                case TaskType::ALIGN_YAW: {
                    ctrl.current_target.yaw = task.param.target_yaw;
                    ctrl.mode = ControlMode::POINT_TRACKING; // 停止循迹，仅执行角度对齐

                    // 无分支纯浮点 Yaw 包裹算法
                    float diff = ctrl.current_target.yaw - pos.yaw;
                    float err_yaw = std::abs(diff - 360.0f * std::roundf(diff * INV_360));

                    // 结合物理底盘是否停稳做决策
                    if (err_yaw < 1.0f && App::g_state.physical.is_stopped) {
                        Subsystem::PoseEstimator::calibrate_vision(4000);   // 视觉校准（重置物理位姿与视觉位姿的偏差，确保后续路径追踪的准确性）
                        task_done = true;
                    }
                    break;
                }
                case TaskType::WAIT_ART2_CAPTURE: {
                    static bool req_sent = false;
                    if (!req_sent) {
                        Subsystem::Vision::request_capture_ART2(task.param.capture.entity_id, task.param.capture.is_box);
                        req_sent = true;
                    }
                    if (vision_data.capture_ack_received) {
                        vision_data.capture_ack_received = false;
                        logical_level.player_start = patrol_actions[game.action_idx].obs.pos; 
                        req_sent = false;
                        task_done = true;
                    }
                    break;
                }
                case TaskType::UPDATE_MAP_LOGIC: {
                    point tw = task.param.map_update.target_wall;
                    // 摧毁墙壁：将目标墙及其周围八格标记为空地
                    for(int dy=-1; dy<=1; dy++) for(int dx=-1; dx<=1; dx++) {
                        if (tw.y+dy > 0 && tw.y+dy < MAP_MAX_HEIGHT-1 && tw.x+dx > 0 && tw.x+dx < MAP_MAX_WIDTH-1)
                            logical_level.map[tw.y+dy][tw.x+dx] = 0;
                    }
                    // 销该炸弹：将该炸弹坐标从逻辑地图的炸弹列表中移除
                    for (int i = 0; i < logical_level.bomb_count; ++i) {
                        if (logical_level.bombs[i].x == task.param.map_update.bomb_start.x &&
                            logical_level.bombs[i].y == task.param.map_update.bomb_start.y) {
                            logical_level.bombs[i] = {-1, -1}; break;
                        }
                    }

                    // 更新当前物理位姿对应的逻辑位置
                    int8_t grid_x = std::clamp<int8_t>(std::lroundf((pos.x - MAP_OFFSET_X) * INV_GRID_SIZE_CM), 0, MAP_MAX_WIDTH - 1);
                    int8_t grid_y = std::clamp<int8_t>(std::lroundf((pos.y - MAP_OFFSET_Y) * INV_GRID_SIZE_CM), 0, MAP_MAX_HEIGHT - 1);
                    logical_level.player_start = {grid_x, grid_y};
                    
                    task_done = true;
                    break;
                }
            }

            if (task_done) current_task_idx++;
            break;
        }

        // =============================================================================
        // ---- 阶段三：语义绑定与推箱执行 ----
        // =============================================================================
        case GamePhase::BIND_SEMANTICS: {
            bool all_done = true;

            // 检查所有“观测动作”对应实体是否已写入语义标签
            for (const auto& act : patrol_actions) {
                if(act.is_bomb_task) continue;
                if (vision_data.semantic_labels[act.obs.entity_id] == -1) {
                    all_done = false; break;
                }
            }
            // for (int i = 0; i < patrol_actions.size(); i++) {
            //     if(patrol_actions[i].is_bomb_task) continue; // 跳过炸弹任务

            //     uint8_t visited_entity_id = patrol_actions[i].obs.entity_id; 

            //     // 有任一实体还未出语义结果，则继续等待
            //     if (vision_data.semantic_labels[visited_entity_id] == -1) {
            //         all_done = false;
            //         break;
            //     }
            // }

            if (all_done) {
                // N-1 规则匹配箱子与目标点 ID
                uint8_t matched_ids[SystemConfig::MAX_BOXES];
                
                if (!patrol_planner.match_semantics(vision_data.semantic_labels, matched_ids)) {
                    // 错误阶段4：语义匹配失败（不满足 N-1 规则）
                    game.error_stage = 4; game.phase = GamePhase::ERROR_OCCURRED;  
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

        // =============================================================================
        // ---- 阶段四：返回起点 ----
        // =============================================================================
        case GamePhase::PLAN_RETURN_HOME: {
            Algorithm::Tracker::track_point({IN_TARGET_X, IN_TARGET_Y, ENTRY_YAW});
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
            Algorithm::Tracker::track_point({pos.x, pos.y, SystemConfig::ENTRY_YAW});
            break;
        }

        case GamePhase::ERROR_OCCURRED: {
            // 错误态：立即停车并保持姿态
            Algorithm::Tracker::track_point({pos.x, pos.y, SystemConfig::ENTRY_YAW});
            break;
        }

        default:
            break;
    }
}


} // namespace App::GameEngine