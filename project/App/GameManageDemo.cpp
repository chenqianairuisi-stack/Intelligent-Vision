#include "GameManage.h"
#include "CoreScheduler.h"
#include "Vision.h"

namespace App::GameEngine {

// 多态拦截器
__attribute__((section(".ramfunc"))) void DemoGameManager::update() {
    auto& game = App::g_state.game;
    auto& phase = game.phase;

    switch (phase) {  

        // ====================================================================
        // 拦截 1：测算“炸弹规划 + 巡图规划”耗时，并切入巡图动画
        // ====================================================================
        case GamePhase::PLAN_PATROL: {
            // Step A：炸弹任务规划耗时
            auto& bomb_tasks = App::g_state.planning.bomb_tasks;
            uint32_t t0 = Core::Scheduler::get_sys_tick_ms();
            bomb_tasks = strategic_planner.evaluate_and_assign_bombs(logical_level);
            bomb_plan_time_ms = Core::Scheduler::get_sys_tick_ms() - t0;

            // Step B：联合巡图规划耗时
            uint32_t t1 = Core::Scheduler::get_sys_tick_ms();
            patrol_actions = patrol_planner.plan_optimal_patrol(logical_level.player_start, bomb_tasks);
            patrol_plan_time_ms = Core::Scheduler::get_sys_tick_ms() - t1;

            Subsystem::Vision::reset_semantic_labels(); 
            
            // 初始化演示状态机缓存
            init_demo_map_from_logical();
            demo.player = logical_level.player_start;
            demo.patrol_target_idx = 0;
            demo.segment_path.clear();
            demo.segment_idx = 0;
            demo.last_tick = Core::Scheduler::get_sys_tick_ms();
            
            App::g_state.debug.need_bg_redraw = true;
            phase = GamePhase::ANIMATE_PATROL_DEMO;
            break;
        }

        // ====================================================================
        // 巡图混合动画（观测动作 + 推炸动作）
        // ====================================================================        
        case GamePhase::ANIMATE_PATROL_DEMO: {
            // 帧率控制：150ms/步
            if (Core::Scheduler::get_sys_tick_ms() - demo.last_tick > 150) {
                demo.last_tick = Core::Scheduler::get_sys_tick_ms();
                
                // 动作流完成后进入语义绑定阶段
                if (demo.patrol_target_idx >= patrol_actions.size()) {
                    phase = GamePhase::BIND_SEMANTICS;
                    break;
                }

                const auto& act = patrol_actions[demo.patrol_target_idx];

                // 1) 当前动作无段路径时，先生成
                if (demo.segment_path.empty()) {
                    bool success = false;
                    if (act.is_bomb_task) success = patrol_planner.get_bomb_push_path(logical_level, logical_level.player_start, act.bomb, demo.segment_path);
                    else success = patrol_planner.get_grid_path(logical_level, logical_level.player_start, act.obs.pos, demo.segment_path);
                    
                    if (!success) { 
                        game.error_stage = 1; // 错误阶段1：寻图路径生成失败
                        phase = GamePhase::ERROR_OCCURRED; break; 
                    }
                    demo.segment_idx = 0;
                }

                // 2) 动画步进：推进小车与炸弹状态
                if (demo.segment_idx < demo.segment_path.size()) {
                    point next_pos = demo.segment_path[demo.segment_idx++];
                    
                    // 推炸动作下，车辆踏入炸弹格时触发炸弹前移
                    if (act.is_bomb_task && demo.map[next_pos.y][next_pos.x] == TILE_BOMB) {
                        point push_dir = {static_cast<int8_t>(next_pos.x - demo.player.x), static_cast<int8_t>(next_pos.y - demo.player.y)};
                        point new_bomb_pos = {static_cast<int8_t>(next_pos.x + push_dir.x), static_cast<int8_t>(next_pos.y + push_dir.y)};
                        
                        // 更新动画地图状态：炸弹前移
                        demo.map[next_pos.y][next_pos.x] = TILE_EMPTY;
                        demo.map[new_bomb_pos.y][new_bomb_pos.x] = TILE_BOMB;
                    }
                    demo.player = next_pos; // 小车移动到下一个位置
                } 

                // 3) 段路径结束后的状态结算
                if (demo.segment_idx >= demo.segment_path.size()){
                    if (act.is_bomb_task) {
                        // 推炸完成：执行 3x3 爆炸清图
                        point tw = act.bomb.target_wall;
                        point bs = act.bomb.bomb_start;

                        for(int dy = -1; dy <= 1; dy++) for(int dx = -1; dx <= 1; dx++) {
                            int ny = tw.y + dy, nx = tw.x + dx;
                            if (ny > 0 && ny < MAP_MAX_HEIGHT-1 && nx > 0 && nx < MAP_MAX_WIDTH-1 && (demo.map[ny][nx] == TILE_WALL || demo.map[ny][nx] == TILE_BOMB)) {
                                demo.map[ny][nx] = 0; 
                                logical_level.map[ny][nx] = 0;   // 同步逻辑地图，确保后续求解一致
                            }
                        }

                        for (int i = 0; i < logical_level.bomb_count; ++i) {
                            if (logical_level.bombs[i].x == bs.x && logical_level.bombs[i].y == bs.y) {
                                logical_level.bombs[i] = {-1, -1};
                                break;
                            }
                        }

                        App::g_state.debug.need_bg_redraw = true;  // 通知 UI 全量重绘地砖
                        
                    } else {
                        // 模拟观测动作：将 mock 语义写入视觉黑板
                        uint8_t current_entity = act.obs.entity_id;
                        App::g_state.vision.semantic_labels[current_entity] = mock_truth_labels[current_entity];
                    }

                    // 同步逻辑位置基准点
                    logical_level.player_start = demo.player;

                    // 进入下一个宏动作
                    demo.segment_path.clear();
                    demo.patrol_target_idx++;
                }
            }
            break;
        }

        // ====================================================================
        // 拦截 2：测算推箱求解耗时并切入推箱动画
        // ====================================================================
        case GamePhase::PLAN_SOKOBAN: {
            // 记录求解耗时
            uint32_t t0 = Core::Scheduler::get_sys_tick_ms();
            bool success = game.is_advanced_stage ? solver.solve(GameMode::PHASE2_SPECIFIC) : solver.solve(GameMode::PHASE1_ANY);
            push_plan_time_ms = Core::Scheduler::get_sys_tick_ms() - t0;

            if (success) {
                init_demo_map_from_logical(); 
                demo.player = logical_level.player_start;
                demo.segment_idx = 0;
                demo.segment_path.clear();
                demo.last_tick = Core::Scheduler::get_sys_tick_ms();
                phase = GamePhase::ANIMATE_DEMO; 
            } else {
                game.error_stage = 2; phase = GamePhase::ERROR_OCCURRED;  // 错误阶段2：推箱子路径求解失败
            }
            break;
        }

        // ====================================================================
        // 推箱动画播放
        // ====================================================================
        case GamePhase::ANIMATE_DEMO: {
            if (Core::Scheduler::get_sys_tick_ms() - demo.last_tick > 150) {
                demo.last_tick = Core::Scheduler::get_sys_tick_ms();
                
                demo.segment_path = solver.get_result_path(); 
                
                if (demo.segment_idx >= demo.segment_path.size() - 1) {
                    logical_level.player_start = demo.segment_path.back(); 
                    logical_level.box_count = 0;     // 强对齐：推箱完成后地图上不应有箱子
                    logical_level.target_count = 0;  // 强对齐：推箱完成后地图上不应有目标点
                    phase = GamePhase::PLAN_RETURN_HOME;               
                    break;
                }

                // 动画级“推箱物理”演算
                point next_pos = demo.segment_path[demo.segment_idx + 1];
                point dir = {static_cast<int8_t>(next_pos.x - demo.player.x), static_cast<int8_t>(next_pos.y - demo.player.y)};

                if (demo.map[next_pos.y][next_pos.x] == TILE_BOX) {
                    point new_box_pos = {static_cast<int8_t>(next_pos.x + dir.x), static_cast<int8_t>(next_pos.y + dir.y)};
                    
                    demo.map[next_pos.y][next_pos.x] = TILE_EMPTY; // 原位置箱子移走
                    
                    // 如果推到了目标点，箱子与目标点对消
                    if (demo.map[new_box_pos.y][new_box_pos.x] == TILE_TARGET) {
                        demo.map[new_box_pos.y][new_box_pos.x] = TILE_EMPTY; 
                    } else {
                        demo.map[new_box_pos.y][new_box_pos.x] = TILE_BOX; 
                    }
                }
                demo.player = next_pos;
                demo.segment_idx++;
            } 
            break;
        }

        // ====================================================================
        // 拦截 3：规划回程路径并开启回程动画
        // ====================================================================
        case GamePhase::PLAN_RETURN_HOME: {
            demo.segment_path.clear();
            point target_point = {SystemConfig::PLAN_END_X, SystemConfig::PLAN_END_Y};

            bool found = patrol_planner.get_grid_path(logical_level, logical_level.player_start, target_point, demo.segment_path);
            
            if (found) {
                demo.segment_idx = 0;
                demo.player = logical_level.player_start; // 回程动画起点 = 推箱终点
                demo.last_tick = Core::Scheduler::get_sys_tick_ms();
                phase = GamePhase::ANIMATE_RETURN_DEMO;
            } else {
                game.error_stage = 3; phase = GamePhase::ERROR_OCCURRED;  // 错误阶段3：回程路径生成失败
            }
            break;
        }

        // ====================================================================
        // 回程动画播放
        // ====================================================================
        case GamePhase::ANIMATE_RETURN_DEMO: {
            if (Core::Scheduler::get_sys_tick_ms() - demo.last_tick > 100) {
                demo.last_tick = Core::Scheduler::get_sys_tick_ms();
                
                // 终点判定：路径节点全部走完
                if (demo.segment_idx >= demo.segment_path.size()) {
                    demo.player = {SystemConfig::PLAN_END_X, SystemConfig::PLAN_END_Y}; // 强对齐入库点
                    phase = GamePhase::FINISHED;
                    logical_level.player_start = demo.player; // 同步逻辑位置
                    break;
                }

                demo.player = demo.segment_path[demo.segment_idx++]; // 沿路径推进一步
            }
            break;
        }

        default:
            // 对于发车、等待视觉、底层追踪等所有逻辑，直接调用基类的 update()
            MockGameManager::update();
            break;
    }
}

// 生成当前渲染上下文，供 Display 层读取
RenderContext DemoGameManager::get_render_context() const {
    RenderContext ctx = {0};
    auto& game = App::g_state.game;
    auto& phase = App::g_state.game.phase;
    auto& action_idx = App::g_state.game.action_idx;

    // Lambda 函数：将零散的数组强行压平到单层网格里
    auto flatten_to_map = [&](const auto& base_map, const point* boxes, int box_count, 
                              const point* targets, int tgt_count, 
                              const point* bombs, int bomb_count) {
        for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; ++x) {
                flattened_render_map[y][x] = (base_map[y][x] > 0) ? 1 : 0; // 墙壁
            }
        }
        if (boxes)   for(int i=0; i<box_count; i++)  if (boxes[i].x != -1)   flattened_render_map[boxes[i].y][boxes[i].x] = 2;
        if (targets) for(int i=0; i<tgt_count; i++)  if (targets[i].x != -1) flattened_render_map[targets[i].y][targets[i].x] = 3;
        if (bombs)   for(int i=0; i<bomb_count; i++) if (bombs[i].x != -1)   flattened_render_map[bombs[i].y][bombs[i].x] = 4;
    };


    if (game.is_demo_mode) {
        // ====================================================================
        // Demo 模式
        // ====================================================================
        bool is_anim = (phase == GamePhase::ANIMATE_PATROL_DEMO || phase == GamePhase::ANIMATE_DEMO || phase == GamePhase::ANIMATE_RETURN_DEMO);

        if (is_anim) {
            ctx.map = (const std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT>*)&demo.map;
        } else {
            // Demo 模式但不在动画中，把 logical_level 压平传给 UI
            flatten_to_map(logical_level.map, logical_level.boxes, logical_level.box_count,
                logical_level.targets, logical_level.target_count,logical_level.bombs, logical_level.bomb_count);

            ctx.map = &flattened_render_map;
        }

        ctx.player_pos = is_anim ? demo.player : logical_level.player_start;

        if (phase >= GamePhase::PLAN_PATROL) {
            ctx.actions_ptr = &patrol_actions;
            ctx.action_start_idx = is_anim ? demo.patrol_target_idx : action_idx;
            ctx.bomb_tasks_ptr = &App::g_state.planning.bomb_tasks;  

            if (is_anim) {
                ctx.path_ptr = &demo.segment_path;
                ctx.path_start_idx = demo.segment_idx;
            } else {
                ctx.path_ptr = (phase == GamePhase::EXEC_SOKOBAN) ? &solver.get_result_path() : &App::g_state.planning.grid_path;
                ctx.path_start_idx = 0;
            }
        }
    } else {
        // ====================================================================
        // 正赛实车模式 / Mock 注入模式
        // ====================================================================
        static std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT> first_render_map;
        static bool first_map_captured = false;

        // 1. 地图显示逻辑：隔离视觉噪音，使用 logical_level 替代 vision
        if (phase >= GamePhase::WAIT_FOR_VISION) {
            flatten_to_map(logical_level.map, 
                           logical_level.boxes, logical_level.box_count,
                           logical_level.targets, logical_level.target_count,
                           logical_level.bombs, logical_level.bomb_count);
            ctx.map = &flattened_render_map;

            if (!first_map_captured) {
                first_render_map = flattened_render_map; // 赛前第一张地图快照，作为异常时的回退显示
                first_map_captured = true;
            }
        } else if (phase == GamePhase::FINISHED || phase == GamePhase::ERROR_OCCURRED) {
            // 比赛结束或发生错误时，显示原始地图快照
            ctx.map = &first_render_map;
        } else {
            // 还没有到加载地图的阶段
            if (game.is_debug_mode) {
                // 如果是本地加载模式，直接黑屏，防止摄像头传来的乱码满屏墙壁
                ctx.map = nullptr;
            } else {
                // 如果是真正的实车比赛，可以把摄像头原始视觉实时投上去，作为赛前检查的雷达
                const auto& vision_data = App::g_state.vision;
                if (vision_data.art1_map_ready) {
                    flatten_to_map(vision_data.map, 
                                   vision_data.boxes, vision_data.box_count,
                                   vision_data.targets, vision_data.box_count,
                                   vision_data.bombs, vision_data.bomb_count);
                    ctx.map = &flattened_render_map;
                }
            }
        }

        // 2. 玩家位置（实车物理映射）
        const auto& rp = App::g_state.physical.pose;
        ctx.player_pos = {
            (int8_t)std::clamp((int)((rp.x - SystemConfig::MAP_OFFSET_X)/SystemConfig::GRID_SIZE_CM + 0.5), 0, SystemConfig::MAP_MAX_WIDTH-1),
            (int8_t)std::clamp((int)((rp.y - SystemConfig::MAP_OFFSET_Y)/SystemConfig::GRID_SIZE_CM + 0.5), 0, SystemConfig::MAP_MAX_HEIGHT-1)
        };

        // 3. 规划数据与动作绘制
        if (phase >= GamePhase::PLAN_PATROL) {
            // 将基类解算完的观测点一直显示
            ctx.actions_ptr = &patrol_actions;        
            ctx.action_start_idx = game.action_idx;        
            
            // 如果是高级阶段，将全局炸弹任务一直显示（画框）
            if (game.is_advanced_stage) {
                ctx.bomb_tasks_ptr = &App::g_state.planning.bomb_tasks;
            }
            
            // 循迹蓝线显示逻辑
            ctx.path_ptr = &App::g_state.planning.grid_path; 
            ctx.path_start_idx = 0;
        }
    }

    return ctx;
}


// 内部函数：从逻辑地图初始化 demo 地图，作为动画的基准状态
void DemoGameManager::init_demo_map_from_logical() {
    for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; ++x) {
            demo.map[y][x] = (logical_level.map[y][x] > 0) ? TILE_WALL : TILE_EMPTY;
        }
    }
    for (int i = 0; i < logical_level.target_count; ++i) {
        demo.map[logical_level.targets[i].y][logical_level.targets[i].x] = TILE_TARGET;
    }
    for (int i = 0; i < logical_level.box_count; ++i) {
        demo.map[logical_level.boxes[i].y][logical_level.boxes[i].x] = TILE_BOX;
    }
    for (int i = 0; i < logical_level.bomb_count; ++i) {
        if (logical_level.bombs[i].x != -1) {
            demo.map[logical_level.bombs[i].y][logical_level.bombs[i].x] = TILE_BOMB;
        }
    }
}


} // namespace App::GameEngine
