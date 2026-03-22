#include "game_manage.h"
#include "task_control.h"
#include "odometry.h"
#include "imu.h"
#include <cmath>
#include <cstring>

__attribute__((section(".dtcm_data"))) GameManager game_manager;

GameManager::GameManager() : phase(GamePhase::INIT_CALIBRATE) {}

void GameManager::init() { phase = GamePhase::INIT_CALIBRATE; }


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
                level.player_start = {ENTRY_GRID_X, ENTRY_GRID_Y}; // 人物当前在入口格
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

        // case GamePhase::PLAN_RETURN: {
        //     StaticArray<point, MAX_PATH_LENGTH> return_path;
        //     // 获取小车推完箱子后停留的最终网格坐标
        //     point current_grid = solver.get_result_path().back();
        //     point goal_grid = {ENTRY_GRID_X, ENTRY_GRID_Y};

        //     // 规划返程 (避开墙壁、炸弹和已经停在目标点上的箱子)
        //     if (plan_return_path(current_grid, goal_grid, return_path)) {
        //         path_tracker.load_path(return_path);
        //         phase = GamePhase::EXEC_RETURN;
        //     }
        //     break;
        // }

        // case GamePhase::EXEC_RETURN: {
        //     if (path_tracker.get_state() == TrackerState::FINISHED) {
        //         phase = GamePhase::ENTER_START_ZONE;
                
        //         // 设定目标退回发车区 (原点)
        //         Pose2D start_zone_pose;
        //         start_zone_pose.x = 0.0f;
        //         start_zone_pose.y = 0.0f;
        //         start_zone_pose.yaw = 0.0f;
        //         chassis_task.set_target_pose(start_zone_pose);
        //     }
        //     break;
        // }

        // case GamePhase::ENTER_START_ZONE: {
        //     float dist = std::sqrt(current_pos.x * current_pos.x + current_pos.y * current_pos.y);
        //     if (dist < 3.0f) {
        //         phase = GamePhase::FINISHED;
        //     }
        //     break;
        // }

        case GamePhase::FINISHED: {
            // chassis_task.set_target_pose({0.0f, 0.0f, 0.0f});
            break;
        }
    }
}