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

/// \brief 初始化比赛管理器入口状态
///
/// \details
/// 读取两路拨码开关，确定赛段模式和 Debug 模式
/// Debug 模式会停在 NONE，等待屏幕选图后再启动流程
///
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

/// \brief GameEngine 全局周期入口
///
/// \details
/// 根据运行模式把 update 分发给正式流程、Mock 流程或 Demo 动画流程
/// Debug 选图前保持挂起，避免真实控制流提前启动
///
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

// 获取当前渲染上下文 [Display 层只读取这个接口，不直接访问全局状态]
RenderContext get_render_context() { 
    return core_engine.get_render_context();
}


//===================================================================
// GameManager 基类实现
//===================================================================

/// \brief 正式比赛业务状态机
///
/// \details
/// 主状态机串起出库建图、巡图观测、语义绑定、推箱求解和返航
/// 底层移动由 RobotTask 队列拆分后交给 Tracker 和 Chassis 执行
///
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
                    game.phase = GamePhase::PLAN_PATROL;       // 进入巡图
                } else {
                    solver.load_from_vision(logical_level);    // 将视觉数据加载到推箱求解器
                    game.phase = GamePhase::PLAN_SOKOBAN;      // 直接进入推箱子阶段
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
            // 解算炸弹任务（无炸弹时自动退化）
            auto& bombs = App::g_state.planning.bomb_tasks;
            bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE1_ANY>(logical_level);

            // 规划巡图路径，得到宏观动作参考序列
            patrol_planner.load_level(logical_level);
            reference_patrol_actions = patrol_planner.plan_optimal_patrol(logical_level.player_start, bombs, ENTRY_YAW, 0);

            // 初始化实时规划器，加载参考序列，准备进入宏观动作调度阶段
            macro_planner.reset(logical_level);
            macro_planner.set_reference_plan(reference_patrol_actions);

            // 重置观测状态
            observed_mask = 0;
            current_observe_yaw = ENTRY_YAW;
            executed_patrol_actions.clear();
            game.action_idx = 0;

            Subsystem::Vision::reset_semantic_labels();   // 清空语义池，准备重新填充
            game.phase = GamePhase::EXEC_ACTION_DISPATCH;
            break;
        }

        case GamePhase::EXEC_ACTION_DISPATCH: {
            // 每次派发新动作前，先同步一次视觉模块已经识别的语义标签，刷新算法内部的配对状态
            macro_planner.sync_semantics(vision_data.semantic_labels);

            // 如果参考序列完成且语义信息足够，进入推箱子阶段；否则继续派发下一个宏动作
            if (macro_planner.ready_for_sokoban(logical_level)) {
                game.phase = GamePhase::BIND_SEMANTICS;
            } else {
                MacroAction act;
                if (!plan_next_macro_action(act)) {
                    game.error_stage = 1;
                    game.phase = GamePhase::ERROR_OCCURRED;
                    break;
                }

                // 将宏动作转换成底层任务序列并加载到调度器
                start_macro_action(act);
                game.phase = GamePhase::EXEC_TASK_QUEUE; // 转移到微观流水线
            }
            break;
        }

        case GamePhase::EXEC_TASK_QUEUE: {
            if (current_task_idx >= task_queue.size()) {
                game.phase = GamePhase::EXEC_ACTION_DISPATCH; // 当前宏任务完成，切回 DISPATCH
                break;
            }

            auto& task = task_queue[current_task_idx];
            bool task_done = false;

            switch (task.type) {
                case TaskType::LOAD_PATH_BOMB: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    BombTask bomb = make_bomb_task(task.param.bomb_push);
                    if (PlanningCommon::get_bomb_push_path(logical_level, logical_level.player_start, bomb, segment)) {
                        // 传入真实逻辑起点，避免路径首点缺失时 Tracker 误判第一段方向
                        Algorithm::Tracker::load_path(segment, logical_level.player_start);
                        task_done = true;
                    } else {
                        game.error_stage = 2; game.phase = GamePhase::ERROR_OCCURRED;
                    }
                    break;
                }
                case TaskType::LOAD_PATH_BOX: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    SokobanLevel probe = logical_level;
                    point probe_player = logical_level.player_start;
                    BoxPushTask box_push = make_box_push_task(task.param.box_push);

                    if (PlanningCommon::append_box_push_path(probe, probe_player, box_push, segment)) {
                        // 推箱路径可能从下一格开始，逻辑起点用于 Tracker 压缩首段
                        Algorithm::Tracker::load_path(segment, logical_level.player_start);
                        task_done = true;
                    } else {
                        game.error_stage = 3; game.phase = GamePhase::ERROR_OCCURRED;
                    }
                    break;
                }
                case TaskType::LOAD_PATH_OBS: {
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    
                    if (PlanningCommon::get_grid_time_path(logical_level, logical_level.player_start, task.param.target_grid, segment)) {
                        // 观察移动同样保留逻辑起点，保证第一段航向和视觉校正基准一致
                        Algorithm::Tracker::load_path(segment, logical_level.player_start);
                        task_done = true;
                    } else {
                        game.error_stage = 4; game.phase = GamePhase::ERROR_OCCURRED;
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
                        task_done = true;
                    }
                    break;
                }
                case TaskType::WAIT_ART2_CAPTURE: {
                    static bool req_sent = false;
                    static bool ack_seen = false;
                    uint8_t entity_id = task.param.capture.entity_id;
                    if (!req_sent) {
                        ack_seen = false;
                        Subsystem::Vision::request_capture_ART2(entity_id, task.param.capture.is_box);
                        req_sent = true;
                    }

                    if (vision_data.capture_ack_received) {
                        vision_data.capture_ack_received = false;
                        ack_seen = true;
                    }

                    // RESULT 和 ACK 都到齐后，才把该实体计入已观测
                    if (ack_seen && vision_data.semantic_labels[entity_id] != -1) {

                        logical_level.player_start = current_macro_action.observe.view.pos;
                        macro_planner.sync_semantics(vision_data.semantic_labels);
                        // 单个抓拍完成后立即更新观测位，便于 Macro 尽早触发完成式推箱
                        uint32_t entity_bit = (1UL << entity_id);
                        observed_mask |= entity_bit;
                        macro_planner.apply_observation(logical_level, entity_bit);
                        
                        req_sent = false;
                        ack_seen = false;
                        task_done = true;
                    }
                    break;
                }
                case TaskType::APPLY_BOMB_RESULT: {
                    // 真实执行完成后，同时结算地图状态和剩余炸弹任务
                    PlanningCommon::apply_executed_bomb_push_result(
                        logical_level,
                        App::g_state.planning.bomb_tasks,
                        task.param.bomb_push
                    );

                    // 更新当前物理位姿对应的逻辑位置
                    logical_level.player_start = current_grid_from_pose(pos);
                    
                    task_done = true;
                    break;
                }
                case TaskType::UPDATE_BOX_LOGIC: {
                    PlanningCommon::apply_box_push_action_effect(logical_level, task.param.box_push);
                    logical_level.player_start = current_grid_from_pose(pos);
                    task_done = true;
                    break;
                }
            }

            if (task_done) {
                current_task_idx++;
                if (current_task_idx >= task_queue.size()) {
                    if (current_macro_action.kind == MacroActionKind::OBSERVE) {
                        uint32_t mask = observe_mask_of(current_macro_action);
                        observed_mask |= mask;
                        current_observe_yaw = current_macro_action.observe.view.target_yaw;
                        macro_planner.sync_semantics(vision_data.semantic_labels);
                        macro_planner.apply_observation(logical_level, mask);
                    }
                }
            }
            break;
        }

        // =============================================================================
        // ---- 阶段三：语义绑定与推箱执行 ----
        // =============================================================================
        case GamePhase::BIND_SEMANTICS: {
            bool all_done = true;

            // 检查所有“观测动作”对应实体是否已写入语义标签
            for (uint8_t entity_id = 0; entity_id < SystemConfig::MAX_ENTITIES; ++entity_id) {
                if ((observed_mask & (1UL << entity_id)) && vision_data.semantic_labels[entity_id] == -1) {
                    all_done = false;
                    break;
                }
            }

            if (all_done) {
                // N-1 规则匹配箱子与目标点 ID
                uint8_t matched_ids[SystemConfig::MAX_BOXES];
                
                macro_planner.sync_semantics(vision_data.semantic_labels);
                if (!macro_planner.fill_matched_ids(matched_ids, logical_level.box_count)) {
                    // 错误阶段 5：语义匹配失败（不满足 N-1 规则）
                    game.error_stage = 5; game.phase = GamePhase::ERROR_OCCURRED;  
                    break;
                }

                // 将匹配结果写回逻辑地图，供后续求解器使用
                for (int i = 0; i < logical_level.box_count; ++i) {
                    logical_level.box_ids[i] = matched_ids[i];
                }

                // 二次炸弹解算（附带语义信息）
                auto& bombs = App::g_state.planning.bomb_tasks;
                bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE2_SPECIFIC>(logical_level);

                // 加载推箱求解器
                solver.load_from_vision(logical_level);              // 导入当前地形（含爆炸改动与小车位置）
                solver.bind_semantics(matched_ids);                  // 绑定语义映射与当前位置
                solver.load_bomb_tasks(bombs.data(), bombs.size());  // 加载炸弹任务
                game.phase = GamePhase::PLAN_SOKOBAN;
            }
            break;
        }
        
        case GamePhase::PLAN_SOKOBAN: {
            // 按赛段调用不同求解模式
            bool success = game.is_advanced_stage ? solver.solve(GameMode::PHASE2_SPECIFIC) : solver.solve(GameMode::PHASE1_ANY);
            if (!success && game.is_advanced_stage) {
                prepare_phase2_solver(true);
                success = solver.solve(GameMode::PHASE2_SPECIFIC);
            }

            if (success) {
                // 求解成功：加载路径并启动自动执行
                // Sokoban 结果路径由求解器生成，仍用当前逻辑起点修正首段压缩
                Algorithm::Tracker::load_path(solver.get_result_path(), logical_level.player_start);
                ctrl.mode = ControlMode::AUTO_TRACKING;
                game.phase = GamePhase::EXEC_SOKOBAN;
            } else {
                game.error_stage = 6; // 错误阶段 6：推箱子路径求解失败
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


//===================================================================
// GameManager 内部辅助函数
//===================================================================

/// \brief 准备第二阶段推箱求解器
/// \param dynamic_fallback 是否启用二阶段动态炸弹兜底策略
///
/// \details
/// 函数会根据当前 logical_level.box_ids 恢复语义绑定，
/// 重新计算第二阶段炸弹任务，并把当前逻辑地图、语义关系、
/// 炸弹任务一起载入 Sokoban 求解器。求解失败重试时会传入 true，
/// 让 Strategy 使用更激进的动态兜底策略重新选择炸弹任务。
void GameManager::prepare_phase2_solver(bool dynamic_fallback) {
    uint8_t matched_ids[SystemConfig::MAX_BOXES];
    for (int i = 0; i < logical_level.box_count; ++i) {
        matched_ids[i] = logical_level.box_ids[i];
    }

    auto& bombs = App::g_state.planning.bomb_tasks;
    strategic_planner.set_phase2_dynamic_fallback(dynamic_fallback);
    bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE2_SPECIFIC>(logical_level);
    strategic_planner.set_phase2_dynamic_fallback(false);

    solver.load_from_vision(logical_level);
    solver.bind_semantics(matched_ids);
    solver.load_bomb_tasks(bombs.empty() ? nullptr : bombs.data(), bombs.size());
}


/// \brief 规划下一条巡图宏动作
/// \param out_action 输出选中的宏动作
/// \return 成功得到宏动作时返回 true，MacroPlanner 和 fallback 都失败时返回 false
///
/// \details
/// 函数统一构造 MacroPlanContext，把 GameManager 持有的真实世界状态传给 MacroPlanner
bool GameManager::plan_next_macro_action(MacroAction& out_action) {
    MacroPlanContext ctx;
    ctx.level = logical_level;
    ctx.player = logical_level.player_start;
    ctx.yaw = current_observe_yaw;
    ctx.bomb_tasks = &App::g_state.planning.bomb_tasks;

    return macro_planner.plan_next_action(ctx, out_action);
}


/// \brief 启动一条宏动作
/// \param action 由 MacroPlanner 选出的宏动作
///
/// \details
/// 函数会记录当前宏动作、更新 UI 使用的动作序列索引，并将 PUSH_BOMB、PUSH_BOX、OBSERVE 三类宏动作转换为底层 RobotTask 队列。
/// 后续 EXEC_TASK_QUEUE 阶段只负责逐条执行这些微任务。
void GameManager::start_macro_action(const MacroAction& action) {
    // 更新当前宏动作
    current_macro_action = action;  

    // 将 action 记录到已执行动作序列 （仅供 UI 展示）
    executed_patrol_actions.push_back(action);
    App::g_state.game.action_idx = executed_patrol_actions.size() - 1;

    // 根据宏动作类型生成底层任务队列
    task_queue.clear();
    current_task_idx = 0;

    if (action.kind == MacroActionKind::PUSH_BOMB) {
        task_queue.push_back(RobotTask::make_path_bomb(action.bomb_push));
        task_queue.push_back(RobotTask::make_wait_track());
        task_queue.push_back(RobotTask::make_apply_bomb_result(action.bomb_push));
    } else if (action.kind == MacroActionKind::PUSH_BOX) {
        task_queue.push_back(RobotTask::make_path_box(action.box_push));
        task_queue.push_back(RobotTask::make_wait_track());
        task_queue.push_back(RobotTask::make_update_box(action.box_push));
    } else {
        const ViewPose& view = action.observe.view;
        uint32_t mask = action.observe.active_mask;

        task_queue.push_back(RobotTask::make_path_obs(view.pos));
        task_queue.push_back(RobotTask::make_wait_track());
        task_queue.push_back(RobotTask::make_align(view.target_yaw));

        // 根据激活掩码为每个需要观测的实体生成一个捕获任务
        for (uint8_t entity_id = 0; entity_id < SystemConfig::MAX_ENTITIES; ++entity_id) {
            if (mask & (1UL << entity_id)) {
                task_queue.push_back(RobotTask::make_capture(entity_id, entity_id < logical_level.box_count));
            }
        }
    }
}

/// \brief 根据物理坐标计算当前网格位置
/// \param pos 当前物理位姿
/// \return 合法网格坐标，越界时返回上一次逻辑位置
///
/// \details
/// 越界时直接进入错误态，而不是 clamp 成地图边界，避免后续规划基于假位置继续执行
///
point GameManager::current_grid_from_pose(const Pose2D& pos) const {
    int grid_x = static_cast<int>(std::lroundf((pos.x - MAP_OFFSET_X) * INV_GRID_SIZE_CM));
    int grid_y = static_cast<int>(std::lroundf((pos.y - MAP_OFFSET_Y) * INV_GRID_SIZE_CM));
    if (grid_x < 0 || grid_x >= MAP_MAX_WIDTH || grid_y < 0 || grid_y >= MAP_MAX_HEIGHT) {
        // 物理坐标已经越界时不再 clamp 成合法格点，避免规划层继续基于假位置行动
        App::g_state.game.error_stage = 9;
        App::g_state.game.phase = GamePhase::ERROR_OCCURRED;
        return logical_level.player_start;
    }
    return {static_cast<int8_t>(grid_x), static_cast<int8_t>(grid_y)};
}

/// \brief 获取观察动作的激活掩码
/// \param action 当前宏动作
/// \return OBSERVE 动作返回 active_mask，其他动作返回 0
///
uint32_t GameManager::observe_mask_of(const MacroAction& action) const {
    if (action.kind != MacroActionKind::OBSERVE) return 0;
    return action.observe.active_mask;
}

} // namespace App::GameEngine
