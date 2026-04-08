#include "task_manage.h"
#include "task_control.h"
#include "task_schedule.h"
#include "test_loadmap.h"
#include "odometry.h"
#include "display.h"
#include "imu_process.h"
#include <cmath>
#include <cstring>


__attribute__((section(".dtcm_data"))) GameManager game_manager;
GameManager::GameManager() : phase(GamePhase::INIT_CALIBRATE), logical_patrol_pos({PLAN_START_X, PLAN_START_Y}) {}

// 初始化拨码开关引脚，并读取当前的比赛阶段选择
void GameManager::init() {
    gpio_init(C27, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(C26, GPI, GPIO_HIGH, GPI_PULL_UP);

    system_delay_ms(10); 

    bool sw1_on = !gpio_get_level(C27); 
    bool sw2_on = !gpio_get_level(C26); 

    if (!sw1_on && !sw2_on) competition_stage = 1;       // 00 -> 阶段一
    else if ( sw1_on && !sw2_on) competition_stage = 2;  // 10 -> 阶段二
    else if (!sw1_on &&  sw2_on) competition_stage = 3;  // 01 -> 阶段三
    else competition_stage = 4;                          // 11 -> 调试阶段(本地导入地图数据)
}


// 全局业务状态机，放在 main 循环中高频调用
__attribute__((section(".ramfunc"))) void GameManager::update() {
    // 获取当前物理坐标，用于判断是否到位
    Point2D current_pos = chassis_odometry.get_position();
    
    switch (phase) {
        case GamePhase::INIT_CALIBRATE: {
            // 假设此时车停在发车区，重置底盘里程计坐标为入口位置
            chassis_odometry.set_position(ENTRY_X, ENTRY_Y);
            
            // 直接设置目标位姿为出库点，准备发车
            Pose2D target;
            target.x = OUT_TARGET_X;
            target.y = OUT_TARGET_Y;
            target.yaw = ENTRY_YAW;
            
            // 下发出库指令
            chassis_task.set_target_pose(target);

            phase = GamePhase::EXIT_START_ZONE;
            break;
        }

        case GamePhase::EXIT_START_ZONE: {
            // 计算到目标点的距离
            float dist = std::sqrt((OUT_TARGET_X - current_pos.x)*(OUT_TARGET_X - current_pos.x) + 
                (OUT_TARGET_Y - current_pos.y)*(OUT_TARGET_Y - current_pos.y));
            
            // 如果到达了地图的第一格
            if (dist < tune.tracker.reach_radius_min) {
                if (competition_stage == 4) {
                    TestMap::load_mock_map(0);      // ~~~ 调试用：直接导入本地地图 ~~~
                } else {
                    vision_manager.request_map_ART1();   // 请求视觉模块发送地图数据
                }
                
                phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        case GamePhase::WAIT_FOR_VISION: {
            // 轮询等待视觉模块解包完成
            if (vision_data.art1_map_ready) {

                // 将 VisionData 转化为纯粹的算法层 SokobanLevel
                SokobanLevel level_cache;
                level_cache.map = vision_data.map;
                level_cache.player_start = {PLAN_START_X, PLAN_START_Y};  // 注：规划起点与实际位置有一定偏差
                level_cache.box_count = vision_data.box_count;
                level_cache.target_count = vision_data.box_count;
                level_cache.bomb_count = vision_data.bomb_count;
                
                for(int i=0; i<level_cache.box_count; ++i) {
                    level_cache.boxes[i] = vision_data.boxes[i];
                    level_cache.targets[i] = vision_data.targets[i];
                }
                for(int i=0; i<level_cache.bomb_count; ++i) {
                    level_cache.bombs[i] = vision_data.bombs[i];
                }

                vision_manager.request_pose_ART1();     // 请求获取当前位姿，供路径跟踪使用
                vision_data.art1_map_ready = false;     // 重置标志，防止重复处理
                
                logical_level = level_cache;            // 同步当前真实地图到逻辑引擎

                if (competition_stage == 2 || competition_stage == 3) {  
                    patrol_planner.load_level(logical_level);  // 将视觉数据加载到巡图规划引擎
                    phase = GamePhase::PLAN_PATROL;            // 进入巡图
                } else {
                    solver.load_from_vision(logical_level);    // 将视觉数据加载到推箱求解器
                    phase = GamePhase::PLAN_SOKOBAN;           // 直接进入推箱子阶段
                }
            } else {
                static uint32_t last_request_tick = TaskScheduler::get_sys_tick_ms();
                if (TaskScheduler::get_sys_tick_ms() - last_request_tick > 1000) {
                    last_request_tick = TaskScheduler::get_sys_tick_ms();
                    vision_manager.request_map_ART1();  // 超时重试请求地图数据
                }
            }
            break;
        }

        case GamePhase::PLAN_PATROL: {
            if (competition_stage == 3) {
                // 【阶段三】先算炸弹，再融进 GTSP
                StaticArray<BombTask, MAX_BOMBS> bombs = strategic_planner.evaluate_and_assign_bombs(logical_level, {PLAN_START_X, PLAN_START_Y});
                patrol_actions = patrol_planner.plan_optimal_patrol({PLAN_START_X, PLAN_START_Y}, bombs);
            } else {
                // 【阶段二兼容】下发空炸弹数组
                StaticArray<BombTask, MAX_BOMBS> empty_bombs;
                patrol_actions = patrol_planner.plan_optimal_patrol({PLAN_START_X, PLAN_START_Y}, empty_bombs);
            }

            action_idx = 0;
            logical_patrol_pos = {PLAN_START_X, PLAN_START_Y};  // 逻辑位置从规划起点开始
            vision_manager.reset_semantic_labels();             // 清空语义缓存，准备接受新的识别结果
            phase = GamePhase::EXEC_ACTION_DISPATCH;
            break;
        }

        case GamePhase::EXEC_ACTION_DISPATCH: {
            if (action_idx >= patrol_actions.size()) {
                phase = GamePhase::BIND_SEMANTICS;
            } else {
                const auto& act = patrol_actions[action_idx];
                StaticArray<point, MAX_PATH_LENGTH> segment;

                if (act.is_bomb_task) {
                    // 计算当前逻辑位置推炸弹到目标墙壁的路径 (推炸弹过程中地图会发生改变，需更新 logical_level)
                    bool found = patrol_planner.get_bomb_push_path(logical_level, logical_patrol_pos, act.bomb, segment);
                    if (found) {
                        path_tracker.load_path(segment);
                        phase = GamePhase::EXEC_BOMB_PUSH;
                    } else {
                        phase = GamePhase::ERROR_OCCURRED; 
                    }

                } else {
                    // 计算当前逻辑位置到下一个观测点的路径 (观测过程不改变地图)
                    bool found = patrol_planner.get_grid_path(logical_level, logical_patrol_pos, act.obs.pos, segment);
                    if (found) {
                        path_tracker.load_path(segment);
                        phase = GamePhase::EXEC_PATROL_MOVE;
                    } else {
                        phase = GamePhase::ERROR_OCCURRED;
                    }
                }
            }
            break;
        }

        case GamePhase::EXEC_PATROL_MOVE: {
            // 监控 PathTracker 是否跑到了观测点
            if (path_tracker.get_state() == TrackerState::FINISHED) {
                phase = GamePhase::EXEC_ALIGN_YAW;
            }
            break;
        }

        case GamePhase::EXEC_ALIGN_YAW: {
            // 到底观测点了，闭环控制车头对准实体
            Pose2D target = chassis_task.get_target_pose();
            target.yaw = patrol_actions[action_idx].obs.target_yaw;  
            chassis_task.set_target_pose(target);

            // 检查 Yaw 角度误差是否小于 5 度
            float current_yaw = imu_sensor.get_yaw();
            float err_yaw = std::abs(target.yaw - current_yaw);
            if (err_yaw > 180.0f) err_yaw = 360.0f - err_yaw;

            // 已对准目标，触发 ART2 捕捉
            if (err_yaw < 5.0f) { 
                uint8_t current_entity = patrol_actions[action_idx].obs.entity_id;
                bool is_box = patrol_actions[action_idx].obs.is_box;
                vision_manager.request_capture_ART2(current_entity, is_box);
                phase = GamePhase::WAIT_ART2_CAPTURE_ACK;
            }
            break;
        }

        case GamePhase::WAIT_ART2_CAPTURE_ACK: {
            // 等待 ART2 的捕捉确认（非阻塞），确认收到后立刻去往下一个点（不必等识别结果）
            if (vision_data.capture_ack_received) {
                vision_data.capture_ack_received = false;
                // 确认到达并观测完毕后，更新小车逻辑位置为该观测点
                logical_patrol_pos = patrol_actions[action_idx].obs.pos; 
                action_idx++;
                phase = GamePhase::EXEC_ACTION_DISPATCH;
            }
            break;
        }

        case GamePhase::EXEC_BOMB_PUSH: {
            // 监控 PathTracker 是否完成推炸弹宏动作
            if (path_tracker.get_state() == TrackerState::FINISHED) {
                phase = GamePhase::UPDATE_MAP;
            }
            break;
        }

        case GamePhase::UPDATE_MAP: {
            point tw = patrol_actions[action_idx].bomb.target_wall;
            // 摧毁墙壁
            for(int dy=-1; dy<=1; dy++) for(int dx=-1; dx<=1; dx++) {
                if (tw.y+dy > 0 && tw.y+dy < MAP_MAX_HEIGHT-1 && tw.x+dx > 0 && tw.x+dx < MAP_MAX_WIDTH-1)
                    logical_level.map[tw.y+dy][tw.x+dx] = 0;
            }

            // 从 logical_level 中注销掉这颗炸弹
            for (int i = 0; i < logical_level.bomb_count; ++i) {
                if (logical_level.bombs[i].x == patrol_actions[action_idx].bomb.bomb_start.x &&
                    logical_level.bombs[i].y == patrol_actions[action_idx].bomb.bomb_start.y) {
                    logical_level.bombs[i] = {-1, -1};
                    break;
                }
            }

            // 直接获取小车当前的物理里程计位置，转换成网格坐标更新逻辑位置
            Point2D real_pos = chassis_odometry.get_position();
            int8_t grid_x = static_cast<int8_t>(std::round((real_pos.x - MAP_OFFSET_X) / GRID_SIZE_CM));
            int8_t grid_y = static_cast<int8_t>(std::round((real_pos.y - MAP_OFFSET_Y) / GRID_SIZE_CM));
            if (grid_x < 0) grid_x = 0; else if (grid_x >= MAP_MAX_WIDTH) grid_x = MAP_MAX_WIDTH - 1;
            if (grid_y < 0) grid_y = 0; else if (grid_y >= MAP_MAX_HEIGHT) grid_y = MAP_MAX_HEIGHT - 1;
            logical_patrol_pos = {grid_x, grid_y}; 

            action_idx++;
            phase = GamePhase::EXEC_ACTION_DISPATCH;
            break;
        }

        case GamePhase::BIND_SEMANTICS: {
            bool all_done = true;

            // 检查观测过的箱子和目标点是不是都出结果了
            for (int i = 0; i < patrol_actions.size(); i++) {
                if(patrol_actions[i].is_bomb_task) continue; // 跳过炸弹任务

                uint8_t visited_entity_id = patrol_actions[i].obs.entity_id; 
    
                // 用真正的实体 ID 去查表
                if (vision_data.semantic_labels[visited_entity_id] == -1) {
                    all_done = false;
                    break;
                }
            }

            if (all_done) {
                //  N-1 推理配对箱子和目标点的 ID
                uint8_t matched_ids[SystemConfig::MAX_BOXES];
                bool is_perfect = patrol_planner.match_semantics(vision_data.semantic_labels, matched_ids);
                
                solver.load_from_vision(logical_level);   // 将爆炸后改变了地形的 logical_level 导入求解器
                solver.bind_semantics(matched_ids, logical_patrol_pos);
                phase = GamePhase::PLAN_SOKOBAN;
            }
            break;
        }

        case GamePhase::PLAN_SOKOBAN: {
            bool success = false;

            // 根据赛段调用不同的底层 C++ 模板机器码
            if (competition_stage == 1) {
                success = solver.solve(GameMode::PHASE1_ANY);
            } else if (competition_stage == 2 || competition_stage == 3) {
                success = solver.solve(GameMode::PHASE2_SPECIFIC);
            }

            if (success) {
                // 求解成功，将生成的网格路径加载到追踪器
                path_tracker.load_path(solver.get_result_path());
                phase = GamePhase::EXEC_SOKOBAN;
            } else {
                // 求解失败，重新请求视觉的逻辑
                vision_manager.request_map_ART1();
                phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        case GamePhase::EXEC_SOKOBAN: {
            // 监控 PathTracker 是否跑完
            if (path_tracker.get_state() == TrackerState::FINISHED) {
                phase = GamePhase::FINISHED;
            }
            break;
        }

        case GamePhase::FINISHED: {
            Point2D current_pos = chassis_odometry.get_position();
            chassis_task.set_target_pose({current_pos.x, current_pos.y, ENTRY_YAW});
            break;
        }

        case GamePhase::ERROR_OCCURRED: {
            Point2D current_pos = chassis_odometry.get_position();
            chassis_task.set_target_pose({current_pos.x, current_pos.y, ENTRY_YAW});
            break;
        }

        default:
            break;
    }
}