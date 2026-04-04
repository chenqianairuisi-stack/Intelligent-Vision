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
    else competition_stage = 1;                          // 11 -> 默认回退到阶段一
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
            
            // 如果到达了地图的第一格 (误差 2cm)
            if (dist < tune.tracker.reach_radius_min) {
                // 请求视觉模块发送地图数据
                // vision_manager.request_map_ART1();
                vision_manager.load_mock_map();     // ~~~ 调试用：直接导入本地地图 ~~~
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
                level_cache.player_start = {PLAN_START_X, PLAN_START_Y};    // 注：规划起点与实际位置有一定偏差
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

                solver.load_from_vision(level_cache);   // 将视觉数据加载到推箱求解器
                
                vision_manager.request_pose_ART1();     // 请求获取当前位姿，供路径跟踪使用
                vision_data.art1_map_ready = false;     // 重置标志，防止重复处理

                
                if (competition_stage == 2 || competition_stage == 3) {  
                    patrol_planner.load_level(level_cache);        // 将视觉数据加载到巡图规划引擎(各自存缓存)
                    phase = GamePhase::PLAN_PATROL;     // 进入巡图
                } else {
                    phase = GamePhase::PLAN_SOKOBAN;    // 直接进入推箱子阶段
                }
            }
            break;
        }

        case GamePhase::PLAN_PATROL: {
            // 调用 GTSP 算出最优巡回序列
            patrol_path = patrol_planner.plan_optimal_patrol({PLAN_START_X, PLAN_START_Y}, ENTRY_YAW);
            patrol_idx = 0;
            logical_patrol_pos = {PLAN_START_X, PLAN_START_Y};  // 逻辑位置从规划起点开始，绝对准确
            vision_manager.reset_semantic_labels();   // 清空语义缓存，准备接受新的识别结果
            phase = GamePhase::EXEC_PATROL_MOVE;
            break;
        }

        case GamePhase::EXEC_PATROL_MOVE: {
            if (patrol_idx >= patrol_path.size()) {
                // 巡视全部完成，进入绑定阶段
                phase = GamePhase::BIND_SEMANTICS;
            } else {
                // 还没巡完，规划去下一个观测点的路径
                StaticArray<point, MAX_PATH_LENGTH> segment_path;
                patrol_planner.get_grid_path(logical_patrol_pos, patrol_path[patrol_idx].pos, segment_path);
                
                // 将网格路径加载到追踪器
                path_tracker.load_path(segment_path);

                phase = GamePhase::ALIGN_YAW;
            }
            break;
        }

        case GamePhase::ALIGN_YAW: {
            // 等待 Tracker 跑到观测点
            if (path_tracker.get_state() == TrackerState::FINISHED) {
                // 到底观测点了，闭环控制车头对准实体
                Pose2D target = chassis_task.get_target_pose();
                target.yaw = patrol_path[patrol_idx].target_yaw;
                chassis_task.set_target_pose(target);

                // 检查 Yaw 角度误差是否小于 5 度
                float current_yaw = imu_sensor.get_yaw();
                float err_yaw = std::abs(target.yaw - current_yaw);
                if (err_yaw > 180.0f) err_yaw = 360.0f - err_yaw;

                // 对准目标，触发 ART2 捕捉
                if (err_yaw < 5.0f) { 
                    uint8_t current_entity = patrol_path[patrol_idx].entity_id;
                    bool is_box = patrol_path[patrol_idx].is_box;
                    vision_manager.request_capture_ART2(current_entity, is_box);

                    phase = GamePhase::WAIT_ART2_CAPTURE_ACK;
                }
            }
            break;
        }

        case GamePhase::WAIT_ART2_CAPTURE_ACK: {
            // 等待 ART2 的捕捉确认（非阻塞），确认收到后立刻去往下一个点（不必等识别结果）
            if (vision_data.capture_ack_received) {
                vision_data.capture_ack_received = false;

                // 确认到达并观测完毕后，将逻辑位置更新为当前观测点！
                logical_patrol_pos = patrol_path[patrol_idx].pos; 

                patrol_idx++;

                phase = GamePhase::EXEC_PATROL_MOVE;
            }
            break;
        }

        case GamePhase::BIND_SEMANTICS: {
            bool all_done = true;

            // 检查观测过的箱子和目标点是不是都出结果了
            for (int i = 0; i < patrol_path.size(); i++) {
                uint8_t visited_entity_id = patrol_path[i].entity_id; 
    
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

                // 将正确的映射关系注入底层 IDA* 推箱子引擎
                // point current_player_pos = {(int8_t)((chassis_odometry.get_position().x - MAP_OFFSET_X) / GRID_SIZE_CM), 
                //                             (int8_t)((chassis_odometry.get_position().y - MAP_OFFSET_Y) / GRID_SIZE_CM)};
                // 注意：这里的 current_player_pos 是一个纯逻辑坐标，永远保持和巡图规划时一致的绝对准确坐标！
                point current_player_pos = logical_patrol_pos; 

                solver.bind_semantics(matched_ids, current_player_pos);
                phase = GamePhase::PLAN_SOKOBAN;
            }
            break;
        }

        case GamePhase::PLAN_SOKOBAN: {
            bool success = false;

            // 根据赛段调用不同的底层 C++ 模板机器码
            if (competition_stage == 1) {
                success = solver.solve(GameMode::PHASE1_ANY);
            } else if (competition_stage == 2) {
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

        default:
            break;
    }
}





//---------------------------------------------------------------------------------------------------------
// 下面是一个专门用于调试的派生类，增加了动画演示和规划耗时记录功能
//---------------------------------------------------------------------------------------------------------

__attribute__((section(".dtcm_data"))) DebugGameManager debug_manager;
DebugGameManager::DebugGameManager() : GameManager(), push_plan_time_ms(0), patrol_plan_time_ms(0) {}

// 多态拦截器：拦截机制：只拦截PLAN_PATROL 和 PLAN_SOKOBAN，其他原有状态甩给基类处理，并新增了两个动画状态来演示规划结果
__attribute__((section(".ramfunc"))) void DebugGameManager::update() {
    
    switch (phase) {  

        // 拦截 1: 测算 GTSP 巡图耗时并开启巡图动画
        case GamePhase::PLAN_PATROL: {
            // 计时开始
            uint32_t start_time = TaskScheduler::get_sys_tick_ms();
            patrol_path = patrol_planner.plan_optimal_patrol({PLAN_START_X, PLAN_START_Y}, ENTRY_YAW);
            patrol_plan_time_ms = TaskScheduler::get_sys_tick_ms() - start_time;

            vision_manager.reset_semantic_labels(); 
            
            // 初始化演示状态（将虚拟车放在起点，准备跑巡图）
            demo.player = {PLAN_START_X, PLAN_START_Y};
            demo.patrol_target_idx = 0;
            demo.segment_path.clear();
            demo.last_tick = TaskScheduler::get_sys_tick_ms();
            
            // 强行改变流向，不让底盘动，进入屏幕演示模式
            phase = GamePhase::ANIMATE_PATROL_DEMO;
            break;
        }

        // 新增状态: 巡图动画 (模拟小车跑图与拍照)
        case GamePhase::ANIMATE_PATROL_DEMO: {
            // 控制动画帧率：100ms 一步
            if (TaskScheduler::get_sys_tick_ms() - demo.last_tick > 100) {
                demo.last_tick = TaskScheduler::get_sys_tick_ms();
                
                // 如果已经跑完了所有观测点，进入 BIND_SEMANTICS，这会触发你的匹配和 N-1 推理
                if (demo.patrol_target_idx >= patrol_path.size()) {
                    phase = GamePhase::BIND_SEMANTICS;
                    break;
                }

                // 检查我们是否已经到了当前的观测点
                if (demo.player == patrol_path[demo.patrol_target_idx].pos) {
                    // 到达观测点，模拟 ART2 视觉触发并返回了结果
                    uint8_t current_entity = patrol_path[demo.patrol_target_idx].entity_id;
                    
                    // 瞬间注入上帝视角的答案
                    vision_data.semantic_labels[current_entity] = mock_truth_labels[current_entity];
                    
                    // 同步更新基类的逻辑坐标！防止 BINDING 时给求解器传入错误的起点！
                    logical_patrol_pos = demo.player;

                    // 准备去下一个观测点
                    demo.segment_path.clear();
                    demo.patrol_target_idx++;
                } else {
                    // 我们还没到观测点，需要往前开
                    if (demo.segment_path.empty() || demo.segment_idx >= demo.segment_path.size()) {
                        // 请求局部 BFS 算出绕开箱子前往观测点的网格路径
                        patrol_planner.get_grid_path(demo.player, patrol_path[demo.patrol_target_idx].pos, demo.segment_path);
                        demo.segment_idx = 0;
                    }
                    
                    // 让虚拟小车走一步
                    if (demo.segment_idx < demo.segment_path.size()) {
                        demo.player = demo.segment_path[demo.segment_idx++];
                    }
                }
            }
            break;
        }

        // 拦截 2: 测算推箱子耗时并开启推箱动画
        case GamePhase::PLAN_SOKOBAN: {
            // 开始规划并记录时间
            uint32_t start_time = TaskScheduler::get_sys_tick_ms();
            bool success = false;
            if (competition_stage == 1) success = solver.solve(GameMode::PHASE1_ANY); 
            else if (competition_stage == 2) success = solver.solve(GameMode::PHASE2_SPECIFIC);
            push_plan_time_ms = TaskScheduler::get_sys_tick_ms() - start_time;

            if (success) {
                // 拦截成功，不立刻发车，而是初始化动画状态机
                demo.player = logical_patrol_pos;
                demo.box_count = vision_data.box_count;
                demo.target_count = vision_data.box_count;
                for (int i = 0; i < demo.box_count; i++) {
                    demo.boxes[i] = vision_data.boxes[i];
                    demo.targets[i] = vision_data.targets[i];
                }
                demo.path_idx = 0;
                demo.last_tick = TaskScheduler::get_sys_tick_ms();
                
                // 将状态转入我们派生类独有的演示状态
                phase = GamePhase::ANIMATE_DEMO; 
            } else {
                // 求解失败，走原逻辑
                vision_manager.request_map_ART1();
                phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        // 新增状态: 播放推箱子动画
        case GamePhase::ANIMATE_DEMO: {
            // 每 100ms 刷新一帧动画 (非阻塞)
            if (TaskScheduler::get_sys_tick_ms() - demo.last_tick > 100) {
                demo.last_tick = TaskScheduler::get_sys_tick_ms();
                
                const auto& path = solver.get_result_path(); 
                
                if (demo.path_idx >= path.size() - 1) {
                    
                    demo.player = logical_patrol_pos;           // 动画结束：将虚拟小车变回原点     
                    path_tracker.load_path(path);               // 将计算好的路径真正加载给底层 Tracker
                    phase = GamePhase::EXEC_SOKOBAN;            // 状态机切入 EXEC_SOKOBAN，接下来基类就会接管真实的物理小车移动
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

    // 假设场上最多8个箱子，8个目标。
    // 这里故意给个乱序配对，用来验证你的 N-1 逻辑推理是否牛逼
    mock_truth_labels[0] = 7;  // 0号箱子写着7
    mock_truth_labels[1] = 2;  // 1号箱子写着2
    mock_truth_labels[2] = 9;  // 2号箱子写着9
    
    // 假设 0~7 是箱子，8~15是目标点 (取决于你总实体数，为了安全我们可以把后面的也填了)
    mock_truth_labels[3]  = 9; // 0号目标点写着9 -> 对应2号箱
    mock_truth_labels[4]  = 7; // 1号目标点写着7 -> 对应0号箱
    mock_truth_labels[5] = 2;  // 2号目标点写着2 -> 对应1号箱
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