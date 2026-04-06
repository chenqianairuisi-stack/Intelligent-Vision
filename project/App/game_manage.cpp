#include "game_manage.h"
#include "task_control.h"
#include "scheduler.h"
#include "odometry.h"
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
                    vision_manager.load_mock_map();      // ~~~ 调试用：直接导入本地地图 ~~~
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



//---------------------------------------------------------------------------------------------------------
// 下面是一个专门用于调试的派生类，增加了动画演示和规划耗时记录功能
//---------------------------------------------------------------------------------------------------------

__attribute__((section(".dtcm_data"))) DebugGameManager debug_manager;
DebugGameManager::DebugGameManager() : GameManager(), push_plan_time_ms(0), patrol_plan_time_ms(0),bomb_plan_time_ms(0) {}

// 多态拦截器
__attribute__((section(".ramfunc"))) void DebugGameManager::update() {
    
    switch (phase) {  

        // ====================================================================
        // 拦截 1: 测算 [炸弹战略] +[GTSP巡图] 耗时并开启巡图动画
        // ====================================================================
        case GamePhase::PLAN_PATROL: {
            uint32_t t0 = TaskScheduler::get_sys_tick_ms();

            if (competition_stage == 3) {
                // 先算炸弹任务，记录耗时
                cached_bomb_tasks = strategic_planner.evaluate_and_assign_bombs(logical_level, {PLAN_START_X, PLAN_START_Y});
                bomb_plan_time_ms = TaskScheduler::get_sys_tick_ms() - t0;
            } else {
                cached_bomb_tasks.clear();
                bomb_plan_time_ms = 0;
            }

            uint32_t t1 = TaskScheduler::get_sys_tick_ms();
            // 结合炸弹任务，生成含宏动作的 3D 动作流
            patrol_actions = patrol_planner.plan_optimal_patrol({PLAN_START_X, PLAN_START_Y}, cached_bomb_tasks);
            patrol_plan_time_ms = TaskScheduler::get_sys_tick_ms() - t1;

            vision_manager.reset_semantic_labels(); 
            
            // 初始化演示状态机
            demo.player = {PLAN_START_X, PLAN_START_Y};
            demo.patrol_target_idx = 0;
            demo.segment_path.clear();
            demo.segment_idx = 0;
            demo.last_tick = TaskScheduler::get_sys_tick_ms();
            demo.map_state = logical_level;
            
            // 深拷贝初始地图与炸弹数组，供动画过程随意破坏
            demo.map_state = logical_level;
            demo.bomb_count = logical_level.bomb_count;
            for (int i = 0; i < demo.bomb_count; ++i) demo.bombs[i] = logical_level.bombs[i];
            
            force_bg_redraw = true;
            phase = GamePhase::ANIMATE_PATROL_DEMO;
            break;
        }

        // ====================================================================
        // 新增状态: 巡图混合动画 (观测目标 + 强推炸弹破墙)
        // ====================================================================        
        case GamePhase::ANIMATE_PATROL_DEMO: {
            // 控制动画帧率：100ms 一步
            if (TaskScheduler::get_sys_tick_ms() - demo.last_tick > 100) {
                demo.last_tick = TaskScheduler::get_sys_tick_ms();
                
                // 动作流跑完，转入逻辑绑定
                if (demo.patrol_target_idx >= patrol_actions.size()) {
                    phase = GamePhase::BIND_SEMANTICS;
                    break;
                }

                const auto& act = patrol_actions[demo.patrol_target_idx];

                // 1. 如果没有路径，则生成路径
                if (demo.segment_path.empty()) {
                    bool success = false;
                    // 注意：这里必须传入 demo.map_state！因为之前的炸弹可能已经改变了地形
                    if (act.is_bomb_task) success = patrol_planner.get_bomb_push_path(demo.map_state, demo.player, act.bomb, demo.segment_path);
                    else success = patrol_planner.get_grid_path(demo.map_state, demo.player, act.obs.pos, demo.segment_path);
                    
                    if (!success) { phase = GamePhase::ERROR_OCCURRED; break; } // 死锁防御
                    demo.segment_idx = 0;
                }

                // 2. 物理推演：小车前进
                if (demo.segment_idx < demo.segment_path.size()) {
                    point next_pos = demo.segment_path[demo.segment_idx++];
                    
                    // 如果正在执行推炸弹宏动作，检查小车是不是踏入了炸弹的坐标
                    if (act.is_bomb_task) {
                        int b_idx = demo_find_bomb(next_pos);
                        if (b_idx != -1) {
                            // 发生了推移：炸弹沿着小车前进的方向被推一格
                            point push_dir = {static_cast<int8_t>(next_pos.x - demo.player.x), static_cast<int8_t>(next_pos.y - demo.player.y)};
                            demo.bombs[b_idx] = {static_cast<int8_t>(next_pos.x + push_dir.x), static_cast<int8_t>(next_pos.y + push_dir.y)};
                        }
                    }
                    demo.player = next_pos; // 小车走入新格子
                } 

                // 3. 终点结算
                if (demo.segment_idx >= demo.segment_path.size()){
                    if (act.is_bomb_task) {
                        // 炸弹已陷入废墟墙壁，触发爆炸摧毁周围 3x3 的格子
                        point tw = act.bomb.target_wall;
                        point bs = act.bomb.bomb_start;

                        for(int dy = -1; dy <= 1; dy++) {
                            for(int dx = -1; dx <= 1; dx++) {
                                int ny = tw.y + dy, nx = tw.x + dx;
                                if (ny > 0 && ny < MAP_MAX_HEIGHT-1 && nx > 0 && nx < MAP_MAX_WIDTH-1) {
                                    demo.map_state.map[ny][nx] = 0; 
                                    logical_level.map[ny][nx] = 0;   // 同步更新逻辑地图，保持动画和求解器的一致性
                                }
                            }
                        }

                        demo_remove_bomb(tw);    // 从 UI 中彻底抹除这颗炸弹
                        for (int i = 0; i < logical_level.bomb_count; ++i) {
                            if (logical_level.bombs[i].x == bs.x && logical_level.bombs[i].y == bs.y) {
                                logical_level.bombs[i] = {-1, -1};
                                demo.map_state.bombs[i] = {-1, -1}; // 同步更新 demo 的炸弹缓存，保持动画和求解器的一致性
                                break;
                            }
                        }

                        force_bg_redraw = true;  // 唤醒 UI 线程重绘地砖
                        
                    } else {
                        // 观测到达，上帝视角瞬间注入答案
                        uint8_t current_entity = act.obs.entity_id;
                        vision_data.semantic_labels[current_entity] = mock_truth_labels[current_entity];
                    }

                    // 真实逻辑基准点同步
                    logical_patrol_pos = demo.player;

                    // 准备进行下一个宏动作
                    demo.segment_path.clear();
                    demo.patrol_target_idx++;
                }
            }
            break;
        }

        // ====================================================================
        // 拦截 2: 测算 IDA* 推箱子耗时并开启推箱动画
        // ====================================================================
        case GamePhase::PLAN_SOKOBAN: {
            // 开始规划并记录时间
            uint32_t t0 = TaskScheduler::get_sys_tick_ms();
            bool success = false;
            if (competition_stage == 1) success = solver.solve(GameMode::PHASE1_ANY); 
            else if (competition_stage == 2 || competition_stage == 3) success = solver.solve(GameMode::PHASE2_SPECIFIC);
            push_plan_time_ms = TaskScheduler::get_sys_tick_ms() - t0;

            if (success) {
                // 拦截成功，初始化动画状态机
                demo.player = logical_patrol_pos;
                demo.map_state = logical_level;
                demo.box_count = logical_level.box_count;
                demo.target_count = logical_level.target_count;
                for (int i = 0; i < demo.box_count; i++) {
                    demo.boxes[i] = logical_level.boxes[i];
                    demo.targets[i] = logical_level.targets[i];
                }
                demo.path_idx = 0;
                demo.last_tick = TaskScheduler::get_sys_tick_ms();
                
                phase = GamePhase::ANIMATE_DEMO; 
            } else {
                // 求解失败，走原逻辑
                vision_manager.request_map_ART1();
                phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        // ====================================================================
        // 新增状态: 播放推箱子动画 
        // ====================================================================
        case GamePhase::ANIMATE_DEMO: {
            // 每 100ms 刷新一帧动画 (非阻塞)
            if (TaskScheduler::get_sys_tick_ms() - demo.last_tick > 100) {
                demo.last_tick = TaskScheduler::get_sys_tick_ms();
                
                const auto& path = solver.get_result_path(); 
                
                if (demo.path_idx >= path.size() - 1) {
                    
                    demo.player = logical_patrol_pos;           // 动画结束：将虚拟小车变回原点     
                    phase = GamePhase::FINISHED;                // 直接进入完成状态，等待重置
                    break;
                }

                // 物理引擎推箱子运算
                point next_pos = path[demo.path_idx + 1];
                point dir = {static_cast<int8_t>(next_pos.x - demo.player.x), static_cast<int8_t>(next_pos.y - demo.player.y)};

                int box1_idx = demo_find_box(next_pos);
                if (box1_idx != -1) {
                    point new_box1_pos = {static_cast<int8_t>(demo.boxes[box1_idx].x + dir.x), static_cast<int8_t>(demo.boxes[box1_idx].y + dir.y)};
                    int box2_idx = demo_find_box(new_box1_pos);
                    
                    if (box2_idx != -1) {
                        // 发生双推
                        point new_box2_pos = {static_cast<int8_t>(demo.boxes[box2_idx].x + dir.x), static_cast<int8_t>(demo.boxes[box2_idx].y + dir.y)};
                        bool tgt2_found = false;

                        // 检查第二个箱子的新位置上是否有目标
                        for(int i=0; i<demo.target_count; i++) if(demo.targets[i].x == new_box2_pos.x && demo.targets[i].y == new_box2_pos.y) tgt2_found = true;
                        
                        // 双推时，如果第二个箱子推到了目标点上，动画里就直接消失这个箱子和目标；如果没有推到目标点，则正常更新箱子位置
                        if (tgt2_found) { demo_remove_target(new_box2_pos); demo_remove_box(demo.boxes[box2_idx]); } 
                        else { demo.boxes[box2_idx] = new_box2_pos; }

                        box1_idx = demo_find_box(next_pos); 
                        if (box1_idx != -1) demo.boxes[box1_idx] = new_box1_pos;
                    } else {
                        // 发生单推
                        bool tgt1_found = false;
                        for(int i=0; i<demo.target_count; i++) if(demo.targets[i].x == new_box1_pos.x && demo.targets[i].y == new_box1_pos.y) tgt1_found = true;
                        if (tgt1_found) { demo_remove_target(new_box1_pos); demo_remove_box(demo.boxes[box1_idx]); } 
                        else { demo.boxes[box1_idx] = new_box1_pos; }
                    }
                }
                demo.player = next_pos;
                demo.path_idx++;
            } 
            break;
        }

        default:
            // 对于发车、等待视觉、底层追踪等所有逻辑，直接调用基类的 update()
            GameManager::update();
            break;
    }
}


// 注入测试数据 (建议在 init 后调用)
void DebugGameManager::inject_mock_semantics() {
    // 把数组全清为 -1，模拟真实开局
    std::memset(mock_truth_labels, -1, sizeof(mock_truth_labels));

    // 自适应配对虚拟数据：把前 N 个箱子和后 N 个目标点完美赋予相同的标签 1, 2, 3...
    for(int i = 0; i < 3; i++) {
        mock_truth_labels[i] = i + 1;         // 箱子 ID (0~7) 赋予标签 1~8
        mock_truth_labels[3 + i] = i + 1;     // 目标 ID (8~15) 赋予标签 1~8
    }

    // // 假设场上最多3个箱子，3个目标。
    // mock_truth_labels[0] = 7;  // 0号箱子写着7
    // mock_truth_labels[1] = 2;  // 1号箱子写着2
    // mock_truth_labels[2] = 9;  // 2号箱子写着9
    // // 假设 0~7 是箱子，8~15是目标点 (取决于你总实体数，为了安全我们可以把后面的也填了)
    // mock_truth_labels[3]  = 9; // 0号目标点写着9 -> 对应2号箱
    // mock_truth_labels[4]  = 7; // 1号目标点写着7 -> 对应0号箱
    // mock_truth_labels[5] = 2;  // 2号目标点写着2 -> 对应1号箱
}



// 辅助函数：数组操作
int DebugGameManager::demo_find_box(point p) {
    for (int i = 0; i < demo.box_count; ++i) {
        if (demo.boxes[i].x == p.x && demo.boxes[i].y == p.y) return i;
    }
    return -1;
}
void DebugGameManager::demo_remove_box(point p) {
    int idx = demo_find_box(p);
    if (idx != -1) {
        demo.boxes[idx] = demo.boxes[demo.box_count - 1]; 
        demo.box_count--;
    }
}
void DebugGameManager::demo_remove_target(point p) {
    for (int i = 0; i < demo.target_count; ++i) {
        if (demo.targets[i].x == p.x && demo.targets[i].y == p.y) {
            demo.targets[i] = demo.targets[demo.target_count - 1];
            demo.target_count--;
            return;
        }
    }
}
int DebugGameManager::demo_find_bomb(point p) {
    for (int i = 0; i < demo.bomb_count; ++i) 
        if (demo.bombs[i].x == p.x && demo.bombs[i].y == p.y) return i;
    return -1;
}
void DebugGameManager::demo_remove_bomb(point p) {
    int idx = demo_find_bomb(p);
    if (idx != -1) {
        demo.bombs[idx] = demo.bombs[demo.bomb_count - 1]; // 尾部置换法，极致性能
        demo.bomb_count--;
    }
}