#include "GameManage.h"
#include "CoreScheduler.h"
#include "Vision.h"
#include <algorithm>

namespace App::GameEngine {

// 获取 Demo 观测动作的有效实体掩码
static uint32_t demo_observe_mask_of(const MacroAction& action) {
    if (action.kind != MacroActionKind::OBSERVE) return 0;
    return action.observe.active_mask;
}

// 多态拦截器
__attribute__((section(".ramfunc"))) void DemoGameManager::update() {
    auto& game = App::g_state.game;
    auto& phase = game.phase;
    auto& vision_data = App::g_state.vision;

    switch (phase) {  

        case GamePhase::WAIT_FOR_VISION: {
            // 新地图开始前清空上一轮动画的耗时显示
            patrol_bomb_plan_time_ms = 0;
            exploration_plan_time_ms = 0;
            push_bomb_plan_time_ms = 0;
            push_plan_time_ms = 0;
            MockGameManager::update();
            break;
        }

        // ====================================================================
        // 拦截 1：测算“炸弹规划 + 巡图规划”耗时，并切入巡图动画
        // ====================================================================
        case GamePhase::PLAN_PATROL: {
            // Step A：炸弹任务规划耗时
            auto& bomb_tasks = App::g_state.planning.bomb_tasks;
            uint32_t t0 = Core::Scheduler::get_sys_tick_ms();
            bomb_tasks = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE1_ANY>(logical_level);
            patrol_bomb_plan_time_ms = Core::Scheduler::get_sys_tick_ms() - t0;

            // Step B：只统计 Exploration 参考巡图规划耗时，不包含 MacroPlanner 在线调度
            uint32_t t1 = Core::Scheduler::get_sys_tick_ms();
            patrol_planner.load_level(logical_level);
            reference_patrol_actions = patrol_planner.plan_optimal_patrol(logical_level.player_start, bomb_tasks, SystemConfig::ENTRY_YAW, 0);
            exploration_plan_time_ms = Core::Scheduler::get_sys_tick_ms() - t1;

            // Demo 只在观测结算时注入语义，保持和 Mock / 实车一致的绑定流程
            macro_planner.reset(logical_level);
            macro_planner.set_reference_plan(reference_patrol_actions);
            executed_patrol_actions.clear();
            observed_mask = 0;
            current_observe_yaw = SystemConfig::ENTRY_YAW;

            Subsystem::Vision::reset_semantic_labels(); 
            
            // 初始化演示状态机缓存
            init_demo_map_from_logical();
            demo.player = logical_level.player_start;
            demo.patrol_target_idx = 0;
            demo.segment_path.clear();
            demo.segment_idx = 0;
            demo.has_active_macro = false;
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
                macro_planner.sync_semantics(App::g_state.vision.semantic_labels);
                if (macro_planner.ready_for_sokoban(logical_level)) {
                    phase = GamePhase::BIND_SEMANTICS;
                    break;
                }

                if (!demo.has_active_macro) {
                    MacroAction action;
                    if (!plan_next_macro_action(action)) {
                        game.error_stage = 1;
                        phase = GamePhase::ERROR_OCCURRED;
                        break;
                    }
                    current_macro_action = action;
                    executed_patrol_actions.push_back(action);
                    demo.patrol_target_idx = executed_patrol_actions.size() - 1;
                    demo.has_active_macro = true;
                    demo.segment_path.clear();
                    demo.segment_idx = 0;
                }

                const auto& act = current_macro_action;
                auto target_id_at = [&](point p) -> int {
                    for (int t = 0; t < logical_level.target_count; ++t) {
                        if (logical_level.targets[t] == p) return t;
                    }
                    return -1;
                };
                auto is_completed_box_push = [&](const MacroAction& action, point p) -> bool {
                    if (action.kind != MacroActionKind::PUSH_BOX) return false;
                    if (!(p == action.box_push.box_target)) return false;
                    if (action.box_push.box_id >= logical_level.box_count) return false;
                    int target_id = target_id_at(p);
                    if (target_id < 0) return false;
                    uint8_t bound_target = macro_planner.knowledge().bound_target[action.box_push.box_id];
                    return macro_planner.knowledge().is_bound[action.box_push.box_id] && bound_target == target_id;
                };

                // 1) 当前动作无段路径时，先生成
                if (demo.segment_path.empty()) {
                    bool success = false;
                    if (act.kind == MacroActionKind::PUSH_BOMB) {
                        BombTask bomb = macro_bomb_task(act);
                        success = PlanningCommon::get_bomb_push_path(logical_level, logical_level.player_start, bomb, demo.segment_path);
                    } else if (act.kind == MacroActionKind::PUSH_BOX) {
                        SokobanLevel probe = logical_level;
                        point probe_player = logical_level.player_start;
                        success = PlanningCommon::append_box_push_path(probe, probe_player, macro_box_task(act), demo.segment_path);
                    } else {
                        success = PlanningCommon::get_grid_time_path(logical_level, logical_level.player_start, act.observe.view.pos, demo.segment_path);
                    }
                    
                    if (!success) { 
                        game.error_stage = 1; // 错误阶段1：寻图路径生成失败
                        phase = GamePhase::ERROR_OCCURRED; break; 
                    }
                    demo.segment_idx = 0;
                }

                // 2) 动画步进：推进小车与炸弹状态
                if (demo.segment_idx < demo.segment_path.size()) {
                    point next_pos = demo.segment_path[demo.segment_idx++];
                    auto demo_base_tile_at = [&](point p) -> uint8_t {
                        for (int t = 0; t < logical_level.target_count; ++t) {
                            if (logical_level.targets[t] == p) return TILE_TARGET;
                        }
                        return logical_level.map[p.y][p.x] > 0 ? TILE_WALL : TILE_EMPTY;
                    };
                    
                    // 推物动作下，车辆踏入物体格时触发物体前移
                    if (demo.map[next_pos.y][next_pos.x] == TILE_BOX) {
                        point push_dir = {static_cast<int8_t>(next_pos.x - demo.player.x), static_cast<int8_t>(next_pos.y - demo.player.y)};
                        point new_box_pos = {static_cast<int8_t>(next_pos.x + push_dir.x), static_cast<int8_t>(next_pos.y + push_dir.y)};

                        demo.map[next_pos.y][next_pos.x] = demo_base_tile_at(next_pos);
                        demo.map[new_box_pos.y][new_box_pos.x] = is_completed_box_push(act, new_box_pos) ? TILE_EMPTY : TILE_BOX;
                    } else if (demo.map[next_pos.y][next_pos.x] == TILE_BOMB) {
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
                    if (act.kind == MacroActionKind::PUSH_BOMB) {
                        // 推炸完成：执行 3x3 爆炸清图
                        if (act.bomb_push.detonates) {
                            point tw = act.bomb_push.blast_wall;
                            demo.map[tw.y][tw.x] = 0; 

                            for(int dy = -1; dy <= 1; dy++) for(int dx = -1; dx <= 1; dx++) {
                                int ny = tw.y + dy, nx = tw.x + dx;
                                if (ny > 0 && ny < MAP_MAX_HEIGHT-1 && nx > 0 && nx < MAP_MAX_WIDTH-1 && demo.map[ny][nx] == TILE_WALL) {
                                    demo.map[ny][nx] = 0; 
                                }
                            }
                        }

                        PlanningCommon::apply_executed_bomb_push_result(
                            logical_level,
                            App::g_state.planning.bomb_tasks,
                            act.bomb_push
                        );

                        App::g_state.debug.need_bg_redraw = true;  // 通知 UI 全量重绘地砖
                        
                    } else if (act.kind == MacroActionKind::PUSH_BOX) {
                        PlanningCommon::apply_box_push_action_effect(logical_level, act.box_push);
                        App::g_state.debug.need_bg_redraw = true;
                    } else {
                        // 模拟观测动作：将 mock 语义写入视觉黑板
                        uint32_t mask = demo_observe_mask_of(act);
                        for (uint8_t entity_id = 0; entity_id < SystemConfig::MAX_ENTITIES; ++entity_id) {
                            if (mask & (1UL << entity_id)) {
                                App::g_state.vision.semantic_labels[entity_id] = mock_truth_labels[entity_id];
                            }
                        }
                        observed_mask |= mask;
                        current_observe_yaw = act.observe.view.target_yaw;
                        macro_planner.sync_semantics(App::g_state.vision.semantic_labels);
                        macro_planner.apply_observation(logical_level, mask);
                    }

                    // 同步逻辑位置基准点
                    logical_level.player_start = demo.player;

                    // 进入下一个宏动作
                    demo.segment_path.clear();
                    demo.segment_idx = 0;
                    demo.has_active_macro = false;
                    demo.patrol_target_idx = executed_patrol_actions.size();
                }
            }
            break;
        }

        case GamePhase::BIND_SEMANTICS: {
            bool all_done = true;

            // 检查所有“观测动作”对应实体是否已写入语义标签
            for (int i = 0; i < executed_patrol_actions.size(); i++) {
                if(executed_patrol_actions[i].kind != MacroActionKind::OBSERVE) continue; // 跳过非观测任务

                // 有任一实体还未出语义结果，则继续等待
                uint32_t mask = executed_patrol_actions[i].observe.active_mask;
                for (uint8_t entity_id = 0; entity_id < SystemConfig::MAX_ENTITIES; ++entity_id) {
                    if ((mask & (1UL << entity_id)) && vision_data.semantic_labels[entity_id] == -1) {
                        all_done = false;
                        break;
                    }
                }
                if (!all_done) break;
            }

            if (all_done) {
                // N-1 规则匹配箱子与目标点 ID
                uint8_t matched_ids[SystemConfig::MAX_BOXES];
                macro_planner.sync_semantics(vision_data.semantic_labels);
                
                if (!macro_planner.fill_matched_ids(matched_ids, logical_level.box_count)) {
                    game.error_stage = 4; // 错误阶段4：语义匹配失败（不满足 N-1 规则）
                    game.phase = GamePhase::ERROR_OCCURRED;
                    break;
                }
                for (int i = 0; i < logical_level.box_count; ++i) {
                    logical_level.box_ids[i] = matched_ids[i];
                }

                // 二次炸弹解算（附带语义信息）
                auto& bombs = App::g_state.planning.bomb_tasks;
                uint32_t t0 = Core::Scheduler::get_sys_tick_ms();
                bombs = strategic_planner.evaluate_and_assign_bombs<GameMode::PHASE2_SPECIFIC>(logical_level);
                push_bomb_plan_time_ms = Core::Scheduler::get_sys_tick_ms() - t0;

                App::g_state.debug.need_bg_redraw = true;

                solver.load_from_vision(logical_level);   // 导入当前地形（含爆炸改动与小车位置）
                solver.bind_semantics(matched_ids);       // 绑定语义映射与当前位置
                solver.load_bomb_tasks(bombs.data(), bombs.size()); // 加载炸弹任务（如果有的话）
                game.phase = GamePhase::PLAN_SOKOBAN;
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
            if (!success && game.is_advanced_stage) {
                prepare_phase2_solver(true);
                success = solver.solve(GameMode::PHASE2_SPECIFIC);
            }
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
                    phase = GamePhase::FINISHED;           
                    break;
                }

                // 动画级“推箱物理”演算
                point next_pos = demo.segment_path[demo.segment_idx + 1];
                point dir = {static_cast<int8_t>(next_pos.x - demo.player.x), static_cast<int8_t>(next_pos.y - demo.player.y)};
                auto target_id_at = [&](point p) -> int {
                    for (int t = 0; t < logical_level.target_count; ++t) {
                        if (logical_level.targets[t] == p) return t;
                    }
                    return -1;
                };
                auto find_box_at = [&](point p) -> int {
                    for (int b = 0; b < logical_level.box_count; ++b) {
                        if (logical_level.boxes[b] == p) return b;
                    }
                    return -1;
                };
                auto is_bound_target_for_box = [&](int box_idx, point p) -> bool {
                    if (box_idx < 0 || box_idx >= logical_level.box_count) return false;
                    uint8_t target_id = logical_level.box_ids[box_idx];
                    return target_id < logical_level.target_count && logical_level.targets[target_id] == p;
                };
                auto demo_base_tile_at = [&](point p) -> uint8_t {
                    return target_id_at(p) >= 0 ? TILE_TARGET : TILE_EMPTY;
                };

                if (demo.map[next_pos.y][next_pos.x] == TILE_BOX) {
                    point new_box_pos = {static_cast<int8_t>(next_pos.x + dir.x), static_cast<int8_t>(next_pos.y + dir.y)};
                    int box_idx = find_box_at(next_pos);
                    bool completed = is_bound_target_for_box(box_idx, new_box_pos);

                    demo.map[next_pos.y][next_pos.x] = demo_base_tile_at(next_pos); // 原位置箱子移走

                    // 只有推到自身绑定目标点时才对消，误入其他目标点仍按箱子显示
                    if (completed) {
                        demo.map[new_box_pos.y][new_box_pos.x] = TILE_EMPTY;
                        if (box_idx >= 0) logical_level.boxes[box_idx] = {-1, -1};
                    } else {
                        demo.map[new_box_pos.y][new_box_pos.x] = TILE_BOX;
                        if (box_idx >= 0) logical_level.boxes[box_idx] = new_box_pos;
                    }
                }
                else if (demo.map[next_pos.y][next_pos.x] == TILE_BOMB) {
                    point new_bomb_pos = {(int8_t)(next_pos.x + dir.x), (int8_t)(next_pos.y + dir.y)};
                    
                    // 检查是否撞墙引爆：如果 new_bomb_pos 是墙，则发生爆炸；否则仅推移炸弹
                    bool explode = false;
                    if (new_bomb_pos.x >= 0 && new_bomb_pos.x < MAP_MAX_WIDTH && new_bomb_pos.y >= 0 && new_bomb_pos.y < MAP_MAX_HEIGHT) {
                        if (demo.map[new_bomb_pos.y][new_bomb_pos.x] == TILE_WALL) explode = true;
                    }

                    if (explode) {
                        demo.map[next_pos.y][next_pos.x] = TILE_EMPTY; // 炸弹消失
                        // 执行 3x3 清除动画
                        for(int dy = -1; dy <= 1; dy++) for(int dx = -1; dx <= 1; dx++) {
                            int ny = new_bomb_pos.y + dy, nx = new_bomb_pos.x + dx;
                            if (ny > 0 && ny < MAP_MAX_HEIGHT-1 && nx > 0 && nx < MAP_MAX_WIDTH-1 && demo.map[ny][nx] == TILE_WALL) {
                                demo.map[ny][nx] = TILE_EMPTY; 
                                logical_level.map[ny][nx] = 0;   // 同步逻辑地图
                            }
                        }
                        App::g_state.debug.need_bg_redraw = true;
                    } else {
                        // 未爆炸，仅仅是推位
                        demo.map[next_pos.y][next_pos.x] = TILE_EMPTY;
                        demo.map[new_bomb_pos.y][new_bomb_pos.x] = TILE_BOMB;
                    }
                }
                demo.player = next_pos;
                demo.segment_idx++;
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

    ctx.patrol_bomb_plan_time_ms = this->patrol_bomb_plan_time_ms;
    ctx.exploration_plan_time_ms = this->exploration_plan_time_ms;
    ctx.push_bomb_plan_time_ms = this->push_bomb_plan_time_ms;
    ctx.push_plan_time_ms = this->push_plan_time_ms;

    auto box_bindings_available = [&]() -> bool {
        return phase == GamePhase::PLAN_SOKOBAN ||
               phase == GamePhase::EXEC_SOKOBAN ||
               phase == GamePhase::PLAN_RETURN_HOME ||
               phase == GamePhase::EXEC_RETURN_HOME ||
               phase == GamePhase::FINISHED ||
               phase == GamePhase::ANIMATE_DEMO;
    };

    // Lambda 函数：将零散的数组强行压平到单层网格里
    auto flatten_to_map = [&](const auto& base_map, const point* boxes, int box_count,
                            const point* targets, int tgt_count,
                            const uint8_t* box_ids,
                            const point* bombs, int bomb_count) {
        auto box_completed_on_target = [&](int box_idx) -> bool {
            if (!boxes || !targets || !box_ids) return false;
            if (box_idx < 0 || box_idx >= box_count) return false;
            if (boxes[box_idx].x == -1) return true;
            uint8_t target_id = box_ids[box_idx];
            return target_id < tgt_count && boxes[box_idx] == targets[target_id];
        };
        auto target_completed_by_box = [&](int target_idx) -> bool {
            if (!boxes || !targets || !box_ids) return false;
            for (int b = 0; b < box_count; ++b) {
                if (box_completed_on_target(b) && box_ids[b] == target_idx) return true;
            }
            return false;
        };

        for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; ++x) {
                flattened_render_map[y][x] = (base_map[y][x] > 0) ? 1 : 0; // 墙壁
            }
        }
        if (targets) for(int i=0; i<tgt_count; i++)  if (targets[i].x != -1 && !target_completed_by_box(i)) flattened_render_map[targets[i].y][targets[i].x] = 3;
        if (boxes)   for(int i=0; i<box_count; i++)  if (boxes[i].x != -1 && !box_completed_on_target(i))   flattened_render_map[boxes[i].y][boxes[i].x] = 2;
        if (bombs)   for(int i=0; i<bomb_count; i++) if (bombs[i].x != -1)   flattened_render_map[bombs[i].y][bombs[i].x] = 4;
    };


    if (game.is_demo_mode) {
        // ====================================================================
        // Demo 模式
        // ====================================================================
        bool is_anim = (phase == GamePhase::ANIMATE_PATROL_DEMO || phase == GamePhase::ANIMATE_DEMO);

        if (is_anim) {
            ctx.map = (const std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT>*)&demo.map;
        } else {
            // Demo 模式但不在动画中，把 logical_level 压平传给 UI
            flatten_to_map(logical_level.map, logical_level.boxes, logical_level.box_count,
                logical_level.targets, logical_level.target_count,
                box_bindings_available() ? logical_level.box_ids : nullptr,
                logical_level.bombs, logical_level.bomb_count);
            ctx.map = &flattened_render_map;
        }

        ctx.player_pos = is_anim ? demo.player : logical_level.player_start;

        if (phase >= GamePhase::PLAN_PATROL) {
            ctx.actions_ptr = &executed_patrol_actions;
            ctx.action_start_idx = is_anim ? demo.patrol_target_idx : action_idx;
            ctx.bomb_tasks_ptr = &App::g_state.planning.bomb_tasks;  

            if (is_anim) {
                ctx.path_ptr = &demo.segment_path;
                ctx.path_start_idx = demo.segment_idx;
            } else {
                ctx.path_ptr = nullptr; // 停止演示时不显示路径
                // ctx.path_ptr = (phase == GamePhase::EXEC_SOKOBAN) ? &solver.get_result_path() : &App::g_state.planning.grid_path;
                // ctx.path_start_idx = 0;
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
                        box_bindings_available() ? logical_level.box_ids : nullptr,
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
                                nullptr,
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
            ctx.actions_ptr = &executed_patrol_actions;        
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
    GamePhase phase = App::g_state.game.phase;
    bool use_box_bindings = phase == GamePhase::PLAN_SOKOBAN ||
                            phase == GamePhase::EXEC_SOKOBAN ||
                            phase == GamePhase::PLAN_RETURN_HOME ||
                            phase == GamePhase::EXEC_RETURN_HOME ||
                            phase == GamePhase::FINISHED ||
                            phase == GamePhase::ANIMATE_DEMO;
    auto box_completed_on_target = [&](int box_idx) -> bool {
        if (!use_box_bindings) return false;
        if (box_idx < 0 || box_idx >= logical_level.box_count) return false;
        if (logical_level.boxes[box_idx].x == -1) return true;
        uint8_t target_id = logical_level.box_ids[box_idx];
        return target_id < logical_level.target_count &&
               logical_level.boxes[box_idx] == logical_level.targets[target_id];
    };
    auto target_completed_by_box = [&](int target_idx) -> bool {
        for (int b = 0; b < logical_level.box_count; ++b) {
            if (box_completed_on_target(b) && logical_level.box_ids[b] == target_idx) return true;
        }
        return false;
    };

    for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; ++x) {
            demo.map[y][x] = (logical_level.map[y][x] > 0) ? TILE_WALL : TILE_EMPTY;
        }
    }
    for (int i = 0; i < logical_level.target_count; ++i) {
        if (target_completed_by_box(i)) continue;
        demo.map[logical_level.targets[i].y][logical_level.targets[i].x] = TILE_TARGET;
    }
    for (int i = 0; i < logical_level.box_count; ++i) {
        if (logical_level.boxes[i].x == -1 || box_completed_on_target(i)) continue;
        demo.map[logical_level.boxes[i].y][logical_level.boxes[i].x] = TILE_BOX;
    }
    for (int i = 0; i < logical_level.bomb_count; ++i) {
        if (logical_level.bombs[i].x != -1) {
            demo.map[logical_level.bombs[i].y][logical_level.bombs[i].x] = TILE_BOMB;
        }
    }
}


} // namespace App::GameEngine
