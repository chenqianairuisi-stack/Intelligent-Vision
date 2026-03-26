#include "game_manage.h"
#include "task_control.h"
#include "odometry.h"
#include "imu.h"
#include <cmath>
#include <cstring>

__attribute__((section(".dtcm_data"))) GameManager game_manager;
GameManager::GameManager() : phase(GamePhase::INIT_CALIBRATE) {}


// 全局业务状态机，放在 main 循环中高频调用
__attribute__((section(".ramfunc"))) void GameManager::update() {
    
    // 获取当前物理坐标，用于判断是否到位
    Point2D current_pos = chassis_odometry.get_position();
    
    switch (phase) {
        case GamePhase::INIT_CALIBRATE: {
            // 假设此时车停在发车区，重置底盘里程计 (将此处定义为物理原点 0,0)
            // chassis_odometry.reset();
            
            // 计算进入地图第一格的物理坐标
            Pose2D target;
            target.x = ENTRY_GRID_X * GRID_SIZE_CM + MAP_OFFSET_X;
            target.y = ENTRY_GRID_Y * GRID_SIZE_CM + MAP_OFFSET_Y;
            target.yaw = 0.0f;
            
            // 下发第一步指令
            chassis_task.set_target_pose(target);
            phase = GamePhase::EXIT_START_ZONE;
            break;
        }

        case GamePhase::EXIT_START_ZONE: {
            // 计算到目标点的距离
            float tx = ENTRY_GRID_X * GRID_SIZE_CM + MAP_OFFSET_X;
            float ty = ENTRY_GRID_Y * GRID_SIZE_CM + MAP_OFFSET_Y;
            float dist = std::sqrt((tx - current_pos.x)*(tx - current_pos.x) + (ty - current_pos.y)*(ty - current_pos.y));
            
            // 如果到达了地图的第一格 (误差 2cm)
            if (dist < tune.tracker.reach_radius_min) {
                // 请求视觉模块发送地图数据
                vision_manager.request_map_ART1();
                phase = GamePhase::WAIT_FOR_VISION;
            }
            break;
        }

        case GamePhase::WAIT_FOR_VISION: {
            // 轮询等待视觉模块解包完成
            if (vision_data.art1_map_ready) {
                // 将 VisionData 转化为纯粹的算法层 SokobanLevel
                SokobanLevel level;
                level.map = vision_data.map;
                level.player_start = {ENTRY_GRID_X, ENTRY_GRID_Y + 1}; // 人物当前在入口格
                level.box_count = vision_data.box_count;
                level.target_count = vision_data.box_count; // 箱子与目标数量一致
                level.bomb_count = vision_data.bomb_count;
                
                for(int i=0; i<level.box_count; ++i) {
                    level.boxes[i] = vision_data.boxes[i];
                    level.targets[i] = vision_data.targets[i];
                }
                for(int i=0; i<level.bomb_count; ++i) {
                    level.bombs[i] = vision_data.bombs[i];
                }

                solver.load_from_vision(level);
                phase = GamePhase::PLAN_SOKOBAN;
                vision_data.art1_map_ready = false;   // 重置标志，防止重复处理
            }
            break;
        }

        case GamePhase::PLAN_SOKOBAN: {
            // 调用 IDA* 求解
            if (solver.solve()) {
                // 求解成功，将生成的网格路径加载到追踪器
                path_tracker.load_path(solver.get_result_path());
                phase = GamePhase::EXEC_SOKOBAN;
            } else {
                // 求解失败，重新请求视觉的逻辑
                vision_manager.request_map_ART1();
                phase = GamePhase::EXIT_START_ZONE;
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
            // chassis_task.set_target_pose({0.0f, 0.0f, 0.0f});
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
DebugGameManager::DebugGameManager() : GameManager(), plan_time_ms(0) {}

// 计时器初始化
void DebugGameManager::timer_init() {
    ::timer_init(GPT_TIM_1, TIMER_MS);
}


// 核心：多态拦截器
// 拦截机制：只拦截 PLAN_SOKOBAN 和 ANIMATE_DEMO，其他所有状态（如 INIT, EXIT, EXEC 等），全部甩给基类 GameManager 处理
__attribute__((section(".ramfunc"))) void DebugGameManager::update() {
    
    switch (phase) {  
        case GamePhase::PLAN_SOKOBAN: {
            // 开始规划并记录时间
            timer_clear(GPT_TIM_1);
            timer_start(GPT_TIM_1);
            bool success = solver.solve(); 
            plan_time_ms = timer_get(GPT_TIM_1);

            if (success) {
                // 拦截成功，不立刻发车，而是初始化动画状态机
                demo.player = {ENTRY_GRID_X, ENTRY_GRID_Y + 1};              //？？？？？
                demo.box_count = vision_data.box_count;
                demo.target_count = vision_data.box_count;
                for (int i = 0; i < demo.box_count; i++) {
                    demo.boxes[i] = vision_data.boxes[i];
                    demo.targets[i] = vision_data.targets[i];
                }
                demo.path_idx = 0;
                demo.last_tick = timer_get(GPT_TIM_1);
                
                // 将状态转入我们派生类独有的演示状态
                phase = GamePhase::ANIMATE_DEMO; 
            } else {
                // 求解失败，走原逻辑
                timer_stop(GPT_TIM_1); // 失败时停掉定时器
                vision_manager.request_map_ART1();
                phase = GamePhase::EXIT_START_ZONE;
            }
            break;
        }

        case GamePhase::ANIMATE_DEMO: {
            // 每 150ms 刷新一帧动画 (非阻塞)
            if (timer_get(GPT_TIM_1) - demo.last_tick > 150) {
                demo.last_tick = timer_get(GPT_TIM_1);
                
                const auto& path = solver.get_result_path(); 
                
                if (demo.path_idx >= path.size() - 1) {
                    
                    demo.player = {ENTRY_GRID_X, ENTRY_GRID_Y + 1};   // 动画结束：将虚拟小车变回原点                ？？？？
                    path_tracker.load_path(path);                 // 将计算好的路径真正加载给底层 Tracker
                    timer_stop(GPT_TIM_1);                        // 动画结束，关闭定时器
                    phase = GamePhase::EXEC_SOKOBAN;              // 状态机切入 EXEC_SOKOBAN，接下来基类就会接管真实的物理小车移动
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