#include "task_manage.h"
#include "task_control.h"
#include "task_schedule.h"
#include "test_loadmap.h"
#include "odometry.h"
#include "display.h"
#include "imu_process.h"
#include <cmath>
#include <cstring>


RenderContext dashboard_vm;
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


RenderContext DebugGameManager::get_render_context() const {
    RenderContext ctx = {0}; 
    
    bool is_anim = is_debug_mode && (phase == GamePhase::ANIMATE_PATROL_DEMO || phase == GamePhase::ANIMATE_DEMO);
    bool is_push = is_debug_mode && (phase == GamePhase::ANIMATE_DEMO);

    // ====================================================================
    // 0. HUD 文本投影 (由引擎负责排版)
    // ====================================================================
    if (is_debug_mode) {
        if (phase <= GamePhase::WAIT_FOR_VISION)           snprintf(ctx.hud_line0, 22, "Phase: WAITING MAP");
        else if (phase == GamePhase::PLAN_PATROL)          snprintf(ctx.hud_line0, 22, "Phase: PLAN PATROL");
        else if (phase == GamePhase::ANIMATE_PATROL_DEMO)  snprintf(ctx.hud_line0, 22, "Phase: DEMO PATROL");
        else if (phase == GamePhase::BIND_SEMANTICS)       snprintf(ctx.hud_line0, 22, "Phase: BINDING... ");
        else if (phase == GamePhase::PLAN_SOKOBAN)         snprintf(ctx.hud_line0, 22, "Phase: PLAN SOKO  ");
        else if (phase == GamePhase::ANIMATE_DEMO)         snprintf(ctx.hud_line0, 22, "Phase: DEMO PUSH  ");
        else if (phase >= GamePhase::FINISHED)             snprintf(ctx.hud_line0, 22, "Phase: FINISHED   ");

        if (phase == GamePhase::ANIMATE_PATROL_DEMO || phase == GamePhase::BIND_SEMANTICS || phase == GamePhase::PLAN_SOKOBAN) {
            snprintf(ctx.hud_line2, 22, "Bm:%3dms GT:%3dms", (int)bomb_plan_time_ms, (int)patrol_plan_time_ms);
        } else if (phase == GamePhase::ANIMATE_DEMO || phase == GamePhase::FINISHED) {
            snprintf(ctx.hud_line2, 22, "IDA* Time: %4dms", (int)push_plan_time_ms);
        } else {
            snprintf(ctx.hud_line2, 22, "Plan Time: --  ms");
        }
    } else {
        switch(phase) {
            case GamePhase::INIT_CALIBRATE:        snprintf(ctx.hud_line0, 22, "P: INIT      "); break;
            case GamePhase::EXIT_START_ZONE:       snprintf(ctx.hud_line0, 22, "P: EXIT_ZONE "); break;
            case GamePhase::WAIT_FOR_VISION:       snprintf(ctx.hud_line0, 22, "P: WAIT_ART1 "); break;
            case GamePhase::EXEC_ACTION_DISPATCH:  snprintf(ctx.hud_line0, 22, "P: ACT_DISP  "); break;
            case GamePhase::EXEC_PATROL_MOVE:      snprintf(ctx.hud_line0, 22, "P: MOVE_PTRL "); break;
            case GamePhase::EXEC_ALIGN_YAW:        snprintf(ctx.hud_line0, 22, "P: ALIGN_YAW "); break;
            case GamePhase::WAIT_ART2_CAPTURE_ACK: snprintf(ctx.hud_line0, 22, "P: WAIT_ART2 "); break;
            case GamePhase::EXEC_BOMB_PUSH:        snprintf(ctx.hud_line0, 22, "P: PUSH_BOMB "); break;
            case GamePhase::EXEC_SOKOBAN:          snprintf(ctx.hud_line0, 22, "P: TRACKING  "); break;
            case GamePhase::FINISHED:              snprintf(ctx.hud_line0, 22, "P: FINISHED  "); break;
            case GamePhase::ERROR_OCCURRED:        snprintf(ctx.hud_line0, 22, "P: ERROR     "); break;
            default:                               snprintf(ctx.hud_line0, 22, "P: COMPUTING "); break;
        }
        snprintf(ctx.hud_line2, 22, "Plan Time: --  ms");
    }
    snprintf(ctx.hud_line1, 22, "Stage: %d", get_stage());

    // ====================================================================
    // 1~3. 地图、实体、轨迹投影
    // ====================================================================
    const SokobanLevel& lvl = is_anim ? demo.map_state : logical_level;
    ctx.map          = &lvl.map;
    ctx.boxes        = is_push ? demo.boxes : lvl.boxes;
    ctx.box_count    = is_push ? demo.box_count : lvl.box_count;
    ctx.targets      = is_push ? demo.targets : lvl.targets;
    ctx.target_count = is_push ? demo.target_count : lvl.target_count;
    ctx.bombs        = is_anim ? demo.bombs : lvl.bombs;
    ctx.bomb_count   = is_anim ? demo.bomb_count : lvl.bomb_count;

    if (is_debug_mode && is_anim) {
        ctx.player_pos = demo.player;
    } else {
        Point2D rp = chassis_odometry.get_position();
        ctx.player_pos = {
            (int8_t)std::clamp((int)std::round((rp.x - MAP_OFFSET_X)/GRID_SIZE_CM), 0, MAP_MAX_WIDTH-1),
            (int8_t)std::clamp((int)std::round((rp.y - MAP_OFFSET_Y)/GRID_SIZE_CM), 0, MAP_MAX_HEIGHT-1)
        };
    }

    if (phase >= GamePhase::PLAN_PATROL) {
        ctx.actions_ptr = &patrol_actions;
        ctx.action_start_idx = is_anim ? demo.patrol_target_idx : action_idx;
        ctx.bomb_tasks_ptr = &cached_bomb_tasks;  // 投射炸弹目标框

        if (is_anim) {
            ctx.path_ptr = is_push ? &solver.get_result_path() : &demo.segment_path;
            ctx.path_start_idx = is_push ? demo.path_idx : demo.segment_idx;
        } else {
            ctx.path_ptr = (phase == GamePhase::EXEC_SOKOBAN) ? &solver.get_result_path() : &path_tracker.get_path();
            ctx.path_start_idx = 0;
        }
    }
    return ctx;
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

    // 手动配对虚拟数据
    // mock_truth_labels[0] = 7;  
    // mock_truth_labels[1] = 2;  
    // mock_truth_labels[2] = 9;  
    // mock_truth_labels[3]  = 9;
    // mock_truth_labels[4]  = 7; 
    // mock_truth_labels[5] = 2; 
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