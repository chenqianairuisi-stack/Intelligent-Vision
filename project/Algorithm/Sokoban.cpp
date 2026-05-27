#include "Sokoban.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

// ============================================================================
// 编译器分支预测提示
// ============================================================================
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// ============================================================================
// 全局实例与哈希随机数生成
// ============================================================================
__attribute__((section(".dtcm_data"))) Sokoban solver;
__attribute__((section(".dtcm_data"))) TTEntry TT[TT_SIZE];  

static uint32_t xor_state = 123456789;

// 轻量级伪随机数生成器，用于初始化 Zobrist 哈希表
static uint32_t xorshift32() {
    xor_state ^= xor_state << 13; 
    xor_state ^= xor_state >> 17; 
    xor_state ^= xor_state << 5;
    return xor_state;
}

#if SOKOBAN_ENABLE_PROFILE
// 热路径计数统一走宏：MCU 版本用 -DSOKOBAN_ENABLE_PROFILE=0 可完全编译掉。
#define SOKOBAN_PROFILE_INC(field) do { ++profile.field; } while (0)
#define SOKOBAN_PROFILE_ADD(field, value) do { profile.field += (value); } while (0)
#define SOKOBAN_PROFILE_MAX(field, value) do { if ((value) > profile.field) profile.field = (value); } while (0)
#else
#define SOKOBAN_PROFILE_INC(field) do {} while (0)
#define SOKOBAN_PROFILE_ADD(field, value) do {} while (0)
#define SOKOBAN_PROFILE_MAX(field, value) do {} while (0)
#endif


// ============================================================================
// 模块 1：对外接口与求解器状态同步
// ============================================================================

/// \brief 从视觉或 PC 测试输入导入初始地图
/// \param level 输入地图快照
/// \return 加载成功时返回 true
///
/// \details
/// 该函数会缓存静态地图、玩家起点、箱子、目标点和炸弹初始位置，
/// 并立即初始化 Zobrist 表、目标距离场和静态死锁表
bool Sokoban::load_from_vision(const SokobanLevel& level) {
    player_start = level.player_start;
    initial_state.player = level.player_start;
    map = level.map;
    
    initial_state.num_boxes = level.box_count;
    initial_state.target_mask = (1 << level.box_count) - 1; 
    for (int i = 0; i < level.box_count; ++i) {
        initial_state.box_x[i] = level.boxes[i].x;
        initial_state.box_y[i] = level.boxes[i].y;
    }

    initial_targets.clear();
    for (int i = 0; i < level.target_count; ++i) { initial_targets.push_back(level.targets[i]); }
    
    initial_bombs.clear();
    for (int i = 0; i < level.bomb_count; ++i) {
        if (level.bombs[i].x != -1) initial_bombs.push_back(level.bombs[i]);
    }

    initial_state.num_bombs = 0; 
    initial_state.blown_mask = 0;
    num_bomb_tasks = 0;
    std::memset(wall_clear_mask, 0, sizeof(wall_clear_mask));

    init_zobrist();
    precompute_target_distances();
    precompute_walk_distances();
    precompute_deadlocks();

    return true;
}

void Sokoban::bind_semantics(const uint8_t* matched_ids) {
    int active_count = 0;
    uint8_t remaining_target_mask = initial_state.target_mask;

    for (int i = 0; i < initial_state.num_boxes; ++i) {
        uint8_t target_id = matched_ids[i];
        bool finished = target_id < initial_targets.size() &&
                        point{initial_state.box_x[i], initial_state.box_y[i]} == initial_targets[target_id];

        if (finished) {
            remaining_target_mask = static_cast<uint8_t>(remaining_target_mask & ~(1U << target_id));
            continue;
        }

        initial_state.box_x[active_count] = initial_state.box_x[i];
        initial_state.box_y[active_count] = initial_state.box_y[i];
        initial_state.box_ids[active_count] = target_id;
        ++active_count;
    }

    initial_state.num_boxes = active_count;
    initial_state.target_mask = remaining_target_mask;

    // 语义绑定后，初始哈希必须按第二阶段的一一匹配规则重新计算
    initial_state.hash = compute_hash<GameMode::PHASE2_SPECIFIC>(initial_state);
}

/// \brief 载入策略层给出的炸弹任务
/// \param tasks 炸弹任务数组，可为空
/// \param count 有效任务数量
///
/// \details
/// 函数会把任务绑定到初始炸弹编号，并建立 wall_clear_mask，
/// 使搜索中可通过 blown_mask 快速判断墙体是否已被炸平
void Sokoban::load_bomb_tasks(const BombTask* tasks, int count) {
    initial_state.num_bombs = initial_bombs.size();
    initial_state.blown_mask = 0;
    num_bomb_tasks = initial_bombs.size(); 
    
    std::memset(wall_clear_mask, 0, sizeof(wall_clear_mask));

    for (int b = 0; b < initial_state.num_bombs; ++b) {
        initial_state.bomb_x[b] = initial_bombs[b].x;
        initial_state.bomb_y[b] = initial_bombs[b].y;
        
        bool found = false;
        for (int t = 0; t < count; ++t) {
            if (tasks[t].bomb_start == initial_bombs[b]) {
                bomb_tasks[b] = tasks[t];
                bomb_tasks[b].box_pushes.clear();
                found = true;
                point tw = tasks[t].target_wall;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        int ny = tw.y + dy, nx = tw.x + dx;
                        if (ny >= 0 && ny < MAP_MAX_HEIGHT && nx >= 0 && nx < MAP_MAX_WIDTH) {
                            wall_clear_mask[ny][nx] |= (1 << b);
                        }
                    }
                }
                break;
            }
        }
        if (!found) {
            bomb_tasks[b].bomb_start = initial_bombs[b];
            bomb_tasks[b].target_wall = {-1, -1}; 
            bomb_tasks[b].box_pushes.clear();
        }
    }

    for (int b = 0; b < MAX_BOMBS; ++b) {
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                ZOBRIST_BOMB[b][y][x] = xorshift32();
            }
        }
    }
    for (int i = 0; i < (1 << MAX_BOMBS); ++i) ZOBRIST_BLOWN_MASK[i] = xorshift32();

    precompute_bomb_distances();
    precompute_target_distances();
    precompute_walk_distances();
    precompute_deadlocks();
}


/// \brief 统一求解入口
/// \param mode PHASE1_ANY 表示任意箱子到任意目标，PHASE2_SPECIFIC 表示语义绑定后的固定目标
/// \return 找到可行推箱路径时返回 true
bool Sokoban::solve(GameMode mode) {
    if (mode == GameMode::PHASE1_ANY) 
        return solve_internal<GameMode::PHASE1_ANY>();
    else 
        return solve_internal<GameMode::PHASE2_SPECIFIC>();
}

// ============================================================================
// 模块 2：IDA* 主流程与搜索热路径
// ============================================================================

static uint16_t bfs_visited_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];  
static uint16_t current_gen = 0;  
static point bfs_q[MAP_CELL_COUNT];
static int8_t bfs_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

/// \brief IDA* 外层迭代加深驱动
///
/// \details
/// 这里只负责初始化哈希、清空置换表、选择初始阈值并逐轮调用深度受限搜索。
/// 真正的热路径在 ida_star_search 内部，避免把频繁执行的逻辑拆散。
template <GameMode Mode> bool Sokoban::solve_internal() {
    if (initial_state.num_boxes > initial_targets.size()) return false;
    
    initial_state.hash = compute_hash<Mode>(initial_state);   
    std::memset(TT, 0, sizeof(TT)); // 清空置换表。
#if SOKOBAN_ENABLE_PROFILE
    profile = SokobanProfile{};
#endif

    int threshold = get_heuristic<Mode>(initial_state);
    StaticArray<point, MAX_PATH_LENGTH> rev_path;             

    // IDA* 迭代加深核心逻辑
    int root_active_bombs = 0;
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        for (int b = 0; b < initial_state.num_bombs; ++b) {
            if (!(initial_state.blown_mask & (1 << b)) && bomb_tasks[b].target_wall.x != -1) root_active_bombs++;
        }
    }
    int root_active_entities = initial_state.num_boxes + root_active_bombs;
    int root_W_num = heuristic_weight_num<Mode>(root_active_entities);
    threshold = (threshold * root_W_num + 9) / 10;
#if SOKOBAN_INITIAL_THRESHOLD_BOOST > 0
    threshold += SOKOBAN_INITIAL_THRESHOLD_BOOST;
#endif
    while (threshold <= MAX_PATH_LENGTH) {
        SOKOBAN_PROFILE_INC(threshold_iterations);
#if SOKOBAN_ENABLE_PROFILE
        profile.final_threshold = static_cast<uint16_t>(threshold);
#endif
        int res = ida_star_search<Mode>(initial_state, 0, 0, threshold, rev_path, -1);
        
        if (res == -1) {                                      
            rev_path.push_back(initial_state.player);         
            std::reverse(rev_path.begin(), rev_path.end());   
            final_path = rev_path;
#ifndef SOKOBAN_DISABLE_PATH_POSTOPT
            optimize_final_path_turns<Mode>();
#endif
            return true;
        }
        if (res >= 9999) break;                               
        threshold = res;
    }

    return false;
}


/// \brief 单次 IDA* 深度受限搜索
/// \tparam Mode 当前求解模式
/// \param state 当前搜索状态
/// \param g 已消耗代价
/// \param depth 当前搜索深度
/// \param threshold 当前 IDA* 阈值
/// \param path 成功时用于回溯输出路径
/// \param last_entity 上一步推动的实体编号，用于动作排序加权
/// \return -1 表示找到解；9999 表示无解分支；其他值表示下一轮建议阈值
///
/// \details
/// 函数内部完成置换表剪枝、启发式剪枝、玩家可达区域 BFS、可推动作生成、
/// 动作排序、状态转移和成功路径回溯，是 Sokoban 求解器最核心的递归热路径。

template <GameMode Mode>
int Sokoban::ida_star_search(const GameState& state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path, int last_entity, uint8_t last_push_dir) {
    if (unlikely(state.num_boxes == 0)) return -1;  // 搜索成功。
    if (unlikely(depth >= MAX_PATH_LENGTH)) return 9999; // 保护 path_hashes 和深度相关工作数组的索引。
    SOKOBAN_PROFILE_MAX(max_depth, static_cast<uint16_t>(depth));

    int tt_probe = probe_transposition(state.hash, g, threshold);
    if (tt_probe != 0) return tt_probe;

    int h = get_heuristic<Mode>(state);
    if (unlikely(h >= 9999)) {                
        SOKOBAN_PROFILE_INC(heuristic_dead_prunes);
        TT[state.hash & (TT_SIZE - 1)].sig = static_cast<uint16_t>(state.hash >> 16);
        TT[state.hash & (TT_SIZE - 1)].value = 9999;
        return 9999;
    }

    int active_bombs = count_active_bomb_tasks<Mode>(state);
    int active_entities = state.num_boxes + active_bombs;
    int box_push_lb_sum = phase2_box_push_lb_sum_if_needed<Mode>(state, active_bombs);
    int W_num = heuristic_weight_num<Mode>(active_entities);
    int W_den = 10;

    int f = g + (h * W_num) / W_den;
    if (f > threshold) {
        SOKOBAN_PROFILE_INC(threshold_prunes);
        return f;
    }

    NodeOccupancy occupancy;
    occupancy.build<Mode>(state);

    // 玩家可达区 BFS：后续 generate_moves 只需检查推位是否在本轮 visited 中。
    // canon_player 取可达区中坐标最小的玩家位置，用于把等价玩家位置归一化。
    current_gen++;  
    if (current_gen == 0) { 
        std::memset(bfs_visited_gen, 0, sizeof(bfs_visited_gen));
        current_gen = 1;
    }

    int head = 0, tail = 0;
    bfs_q[tail++] = state.player;
    bfs_visited_gen[state.player.y][state.player.x] = current_gen;
    bfs_dist[state.player.y][state.player.x] = 0;
    
    point canon_player = state.player;  

    while(head < tail) {
        point curr = bfs_q[head++];
        if (curr.y < canon_player.y || (curr.y == canon_player.y && curr.x < canon_player.x)) {
            canon_player = curr;
        }
        for(int dir = 0; dir < 4; ++dir) {
            point np = curr + MOVE[dir];
            if(!is_overstep(np)) {
                if(bfs_visited_gen[np.y][np.x] != current_gen && !is_solid(np, state.blown_mask)) {
                    if(occupancy.box_at(np) == -1 && occupancy.bomb_at<Mode>(np) == -1) {   
                        bfs_visited_gen[np.y][np.x] = current_gen;
                        bfs_dist[np.y][np.x] = bfs_dist[curr.y][curr.x] + 1;
                        bfs_q[tail++] = np;
                    }
                }
            }
        }
    }

    uint32_t canon_hash = state.hash ^ ZOBRIST_PLAYER[state.player.y][state.player.x] ^ ZOBRIST_PLAYER[canon_player.y][canon_player.x];
    for (int i = 0; i < depth; ++i) {
        if (path_hashes[i] == canon_hash) {
            SOKOBAN_PROFILE_INC(path_cycle_prunes);
            return 9999; // 剪掉当前路径上的环路。
        }
    }
    path_hashes[depth] = canon_hash;
    SOKOBAN_PROFILE_INC(expanded_nodes);

    // 每层约几百字节，32KB 栈足够承受当前 profile 中 40~50 层的典型深度。
    // 保持局部数组还能避免长期占用宝贵的 DTCM 全局空间。
    TinyMove moves[MAX_NODE_MOVES];
    int num_moves = generate_moves<Mode>(state, occupancy, active_bombs, moves);
    MacroMove macro_moves[MAX_NODE_MACROS];
    int num_macros = generate_bomb_macros<Mode>(state, h, active_entities, macro_moves);
    SOKOBAN_PROFILE_ADD(generated_moves, static_cast<uint32_t>(num_moves + num_macros));

    int min_next_threshold = 9999;

    EvalMove sorted_moves[MAX_NODE_ACTIONS];
    int action_count = 0;

    // 热路径性能优先：动作评分直接留在 IDA* 主循环所在函数内。
    // 这样编译器能同时看见 moves、state 和递归参数，减少寄存器溢出和重复取址。
    for (int m = 0; m < num_moves; ++m) {
        TinyMove& mv = moves[m];
        bool is_bomb_entity = (mv.entity_idx >= state.num_boxes);
        int b_idx = is_bomb_entity ? (mv.entity_idx - state.num_boxes) : -1;
        point pos = is_bomb_entity ? point{state.bomb_x[b_idx], state.bomb_y[b_idx]}
                                    : point{state.box_x[mv.entity_idx], state.box_y[mv.entity_idx]};
        point push_to = pos + MOVE[mv.dir];
        point eval_push_to = push_to;
        if (mv.slide_dist > 0) {
            eval_push_to.x += MOVE[mv.dir].x * mv.slide_dist;
            eval_push_to.y += MOVE[mv.dir].y * mv.slide_dist;
        }

        int delta_h = 0;
        if (!mv.triggers_explosion) {
            if (is_bomb_entity) {
                if (bomb_tasks[b_idx].target_wall.x != -1) {
                    delta_h = b_dist[b_idx][eval_push_to.y][eval_push_to.x] - b_dist[b_idx][pos.y][pos.x];
                }
            } else {
                if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                    int b_id = state.box_ids[mv.entity_idx];
                    delta_h = t_dist[b_id][eval_push_to.y][eval_push_to.x] - t_dist[b_id][pos.y][pos.x];
                } else {
                    int min_old = 9999, min_new = 9999;
                    for (size_t t = 0; t < initial_targets.size(); ++t) {
                        if (state.target_mask & (1 << t)) {
                            if (t_dist[t][pos.y][pos.x] < min_old) min_old = t_dist[t][pos.y][pos.x];
                            if (t_dist[t][eval_push_to.y][eval_push_to.x] < min_new) min_new = t_dist[t][eval_push_to.y][eval_push_to.x];
                        }
                    }
                    if (min_old != 9999 && min_new != 9999) delta_h = min_new - min_old;
                }
            }
        }

        int walk_weight = 1;
        int progress_weight = 100;
        int same_entity_bonus = 20;
        int same_dir_bonus = 0;
        int dir_change_penalty = 0;
        int switch_entity_penalty = 0;
        int explosion_bonus = 80;
        int slide_bonus = 5;
        if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
            walk_weight = (active_entities <= 4) ? 12 : 8;
            progress_weight = (active_entities <= 4) ? 32 : 42;
            same_entity_bonus = 28;
            same_dir_bonus = 12;
            dir_change_penalty = 16;
            switch_entity_penalty = 8;
            explosion_bonus = 45;
            slide_bonus = 4;
        }

        int sort_key = mv.walk_dist * walk_weight + 1 + delta_h * progress_weight;
        if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
            bool high_box_push_pressure = (state.num_boxes > 0 && box_push_lb_sum >= state.num_boxes * 10);
            if (is_bomb_entity && active_bombs > 0 && high_box_push_pressure) {
                // 箱子整体离目标很远时，先鼓励完成炸弹清墙，避免箱子在未开路区域里盲搜。
                sort_key -= 40;
                if (!mv.triggers_explosion && b_dist[b_idx][eval_push_to.y][eval_push_to.x] >= 0) {
                    int remaining = b_dist[b_idx][eval_push_to.y][eval_push_to.x];
                    if (remaining <= 2) sort_key -= (3 - remaining) * 40;
                }
                if (mv.triggers_explosion) sort_key -= 100;
            }
        }
        if (last_entity != -1 && mv.entity_idx == last_entity) sort_key -= same_entity_bonus;
        else if (last_entity != -1) sort_key += switch_entity_penalty;
        if (last_push_dir < 4) {
            if (mv.dir == last_push_dir) sort_key -= same_dir_bonus;
            else sort_key += dir_change_penalty;
        }
        if (mv.triggers_explosion) sort_key -= explosion_bonus;
        sort_key -= mv.slide_dist * slide_bonus;

        sorted_moves[action_count++] = {static_cast<uint8_t>(m), 0, static_cast<int16_t>(sort_key)};
    }

    for (int m = 0; m < num_macros; ++m) {
        sorted_moves[action_count++] = {static_cast<uint8_t>(m), 1, macro_moves[m].sort_key};
    }

    for (int i = 1; i < action_count; ++i) {
        EvalMove key = sorted_moves[i];
        int j = i - 1;
        while (j >= 0 && sorted_moves[j].sort_key > key.sort_key) {
            sorted_moves[j + 1] = sorted_moves[j];
            j = j - 1;
        }
        sorted_moves[j + 1] = key;
    }

    for (int m = 0; m < action_count; ++m) {
        if (sorted_moves[m].is_macro) {
            MacroMove& macro = macro_moves[sorted_moves[m].move_idx];
            int res = ida_star_search<Mode>(
                macro.next_state,
                g + macro.path_cost,
                depth + 1,
                threshold,
                path,
                macro.entity_idx,
                macro.final_push_dir);

            if (unlikely(res == -1)) {
                StaticArray<point, MAX_PATH_LENGTH> macro_path;
                if (!build_bomb_macro_path(state, macro.bomb_idx, macro_path)) return 9999;
                for (int i = macro_path.size() - 1; i >= 0; --i) {
                    path.push_back(macro_path[i]);
                }
                return -1;
            }

            if (res < min_next_threshold) min_next_threshold = res;
            continue;
        }

        TinyMove& mv = moves[sorted_moves[m].move_idx];

        bool is_bomb_entity = (mv.entity_idx >= state.num_boxes);
        int b_idx = is_bomb_entity ? (mv.entity_idx - state.num_boxes) : -1;
        point pos = is_bomb_entity ? point{state.bomb_x[b_idx], state.bomb_y[b_idx]}
                                : point{state.box_x[mv.entity_idx], state.box_y[mv.entity_idx]};
        point push_from = pos - MOVE[mv.dir];
        point push_to = pos + MOVE[mv.dir];
        if (mv.slide_dist > 0) {
            push_to.x += MOVE[mv.dir].x * mv.slide_dist;
            push_to.y += MOVE[mv.dir].y * mv.slide_dist;
        }

        // 状态转移是递归最热的一段，直接展开可避免 apply_move 造成的寄存器压力。
        GameState next_state;
        next_state = state;
        next_state.player = push_to - MOVE[mv.dir];

        uint32_t new_hash = state.hash;
        new_hash ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
        new_hash ^= ZOBRIST_PLAYER[next_state.player.y][next_state.player.x];

        if (mv.triggers_explosion) {
            next_state.blown_mask |= (1 << b_idx);
            new_hash ^= ZOBRIST_BLOWN_MASK[state.blown_mask];
            new_hash ^= ZOBRIST_BLOWN_MASK[next_state.blown_mask];
            new_hash ^= ZOBRIST_BOMB[b_idx][state.bomb_y[b_idx]][state.bomb_x[b_idx]];
        } else if (is_bomb_entity) {
            new_hash ^= ZOBRIST_BOMB[b_idx][state.bomb_y[b_idx]][state.bomb_x[b_idx]];
            new_hash ^= ZOBRIST_BOMB[b_idx][push_to.y][push_to.x];
            next_state.bomb_x[b_idx] = push_to.x;
            next_state.bomb_y[b_idx] = push_to.y;
        } else {
            int new_box_count = 0;
            int8_t new_bx[MAX_BOXES], new_by[MAX_BOXES];
            uint8_t new_ids[MAX_BOXES];
            uint8_t new_target_mask = state.target_mask;

            // 箱子进入目标后从活动列表中移除。这样后续节点只保留未完成箱子，
            // 降低状态体积、启发式维度和动作生成成本。
            for (int i = 0; i < state.num_boxes; ++i) {
                point old_p = {state.box_x[i], state.box_y[i]};
                point p = (i == mv.entity_idx) ? push_to : old_p;

                if constexpr (Mode == GameMode::PHASE1_ANY) {
                    new_hash ^= ZOBRIST_BOX[old_p.y][old_p.x];
                    int t_idx = -1;
                    for (size_t t = 0; t < initial_targets.size(); ++t) {
                        if ((new_target_mask & (1 << t)) && initial_targets[t] == p) {
                            t_idx = static_cast<int>(t);
                            break;
                        }
                    }
                    if (t_idx != -1) {
                        new_target_mask &= ~(1 << t_idx);
                        new_hash ^= ZOBRIST_TARGET[t_idx];
                    } else {
                        new_bx[new_box_count] = p.x;
                        new_by[new_box_count] = p.y;
                        new_hash ^= ZOBRIST_BOX[p.y][p.x];
                        new_box_count++;
                    }
                } else {
                    uint8_t b_id = state.box_ids[i];
                    new_hash ^= ZOBRIST_SPECIFIC_BOX[b_id][old_p.y][old_p.x];

                    if (p == initial_targets[b_id]) {
                        new_target_mask &= ~(1 << b_id);
                        new_hash ^= ZOBRIST_TARGET[b_id];
                    } else {
                        new_bx[new_box_count] = p.x;
                        new_by[new_box_count] = p.y;
                        new_ids[new_box_count] = b_id;
                        new_hash ^= ZOBRIST_SPECIFIC_BOX[b_id][p.y][p.x];
                        new_box_count++;
                    }
                }
            }

            next_state.num_boxes = new_box_count;
            next_state.target_mask = new_target_mask;
            for (int i = 0; i < new_box_count; ++i) {
                next_state.box_x[i] = new_bx[i];
                next_state.box_y[i] = new_by[i];
                if constexpr (Mode == GameMode::PHASE2_SPECIFIC) next_state.box_ids[i] = new_ids[i];
            }
        }

        next_state.hash = new_hash;
        int step_cost = mv.walk_dist + 1; 

        // 递归进入下一层 IDA* 搜索
        int res = ida_star_search<Mode>(next_state, g + step_cost, depth + 1, threshold, path, mv.entity_idx, mv.dir);
        
        if (unlikely(res == -1)) {
            // 只有找到解时才重建行走路径；放在这里避免每个节点额外穿过 helper 边界。
            current_gen++;
            if (current_gen == 0) {
                std::memset(bfs_visited_gen, 0, sizeof(bfs_visited_gen));
                current_gen = 1;
            }

            int h2 = 0, t2 = 0;
            static point temp_parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

            bfs_q[t2++] = state.player;
            bfs_visited_gen[state.player.y][state.player.x] = current_gen;

            while (h2 < t2) {
                point curr = bfs_q[h2++];
                if (curr == push_from) break;
                for (int d2 = 0; d2 < 4; ++d2) {
                    point np = curr + MOVE[d2];
                    if (!is_overstep(np)) {
                        if (bfs_visited_gen[np.y][np.x] != current_gen && !is_solid(np, state.blown_mask)) {
                            if (occupancy.box_at(np) == -1 && occupancy.bomb_at<Mode>(np) == -1) {
                                bfs_visited_gen[np.y][np.x] = current_gen;
                                temp_parent[np.y][np.x] = curr;
                                bfs_q[t2++] = np;
                            }
                        }
                    }
                }
            }

            for (int s = mv.slide_dist; s >= 0; --s) {
                point step_p;
                step_p.x = pos.x + MOVE[mv.dir].x * s;
                step_p.y = pos.y + MOVE[mv.dir].y * s;
                path.push_back(step_p);
            }

            point walk = push_from;
            while (walk != state.player) {
                path.push_back(walk);
                walk = temp_parent[walk.y][walk.x];
            }
            return -1;
        }
        
        if (res < min_next_threshold) min_next_threshold = res;
    }

    store_transposition(state.hash, g, min_next_threshold);

    return min_next_threshold;
}


template <GameMode Mode>
inline int Sokoban::count_active_bomb_tasks(const GameState& state) const {
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        (void)state;
        return 0;
    } else {
        int active_bombs = 0;
        for (int b = 0; b < state.num_bombs; ++b) {
            if (!(state.blown_mask & (1 << b)) && bomb_tasks[b].target_wall.x != -1) ++active_bombs;
        }
        return active_bombs;
    }
}

// 只有 Phase2 且仍有炸弹任务时，动作排序才需要估计箱子的剩余推动压力。
// 纯推箱阶段直接返回 0，减少每个节点不必要的 t_dist 访存。
template <GameMode Mode>
inline int Sokoban::phase2_box_push_lb_sum_if_needed(const GameState& state, int active_bombs) const {
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        (void)state;
        (void)active_bombs;
        return 0;
    } else {
        if (active_bombs == 0) return 0;

        // 只在“箱子压力很大时优先推炸弹”的排序分支里使用。
        // 无活动炸弹时跳过，可减少纯推箱阶段每个节点的无效 t_dist 访存。
        int sum = 0;
        for (int i = 0; i < state.num_boxes; ++i) {
            int d = t_dist[state.box_ids[i]][state.box_y[i]][state.box_x[i]];
            if (d > 0 && d < 9999) sum += d;
        }
        return sum;
    }
}

// 启发式权重用于 IDA* 的 f=g+w*h。实体多时提高权重，加快收敛；
// 实体少时权重更保守，避免过度贪心导致走进长路径。
template <GameMode Mode>
inline int Sokoban::heuristic_weight_num(int active_entities) const {
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        if (active_entities >= 8) return 30;
        if (active_entities >= 6) return 25;
        if (active_entities >= 5) return 20;
        if (active_entities >= 4) return 18;
        return 10;
    } else {
        if (active_entities >= 8) return 25;
        if (active_entities >= 6) return 22;
        if (active_entities >= 5) return 20;
        if (active_entities >= 4) return 15;
        return 10;
    }
}

// 置换表保存“该状态在某个剩余阈值下已经失败”的信息。
// 命中时可以直接返回下一轮建议阈值，跳过重复子树。
inline int Sokoban::probe_transposition(uint32_t hash, int g, int threshold) {
    int remaining_threshold = threshold - g;
    int tt_idx1 = hash & (TT_SIZE - 1);
    int tt_idx2 = (hash ^ 0x5BD1E995) & (TT_SIZE - 1);
    uint16_t sig = static_cast<uint16_t>(hash >> 16);

    if (TT[tt_idx1].sig == sig && TT[tt_idx1].value > remaining_threshold) {
        SOKOBAN_PROFILE_INC(tt_hits);
        return g + TT[tt_idx1].value;
    }
    if (TT[tt_idx2].sig == sig && TT[tt_idx2].value > remaining_threshold) {
        SOKOBAN_PROFILE_INC(tt_hits);
        return g + TT[tt_idx2].value;
    }
    return 0;
}

// 存储当前分支失败后得到的下一轮最小阈值。
// 两个候选槽位减少简单 hash 冲突，value 越大剪枝能力越强。
inline void Sokoban::store_transposition(uint32_t hash, int g, int min_next_threshold) {
    int tt_idx1 = hash & (TT_SIZE - 1);
    int tt_idx2 = (hash ^ 0x5BD1E995) & (TT_SIZE - 1);
    uint16_t sig = static_cast<uint16_t>(hash >> 16);

    int cost_to_go = min_next_threshold - g;
    uint16_t val_to_store = (cost_to_go > 65535) ? 65535 : cost_to_go;

    if (TT[tt_idx1].sig == sig) {
        if (val_to_store > TT[tt_idx1].value) TT[tt_idx1].value = val_to_store;
    } else if (TT[tt_idx2].sig == sig) {
        if (val_to_store > TT[tt_idx2].value) TT[tt_idx2].value = val_to_store;
    } else {
        if (TT[tt_idx1].value <= TT[tt_idx2].value) {
            TT[tt_idx1].sig = sig;
            TT[tt_idx1].value = val_to_store;
        } else {
            TT[tt_idx2].sig = sig;
            TT[tt_idx2].value = val_to_store;
        }
    }
}

// 判断 p 是否是当前箱子的有效目标格。
// Phase1 中任意未完成目标都可用；Phase2 中只能进入绑定目标。
template <GameMode Mode>
inline bool Sokoban::is_active_target_cell(const GameState& state, point p, int box_idx, int& out_idx) const {
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        for (size_t t = 0; t < initial_targets.size(); ++t) {
            if ((state.target_mask & (1 << t)) && initial_targets[t] == p) {
                out_idx = static_cast<int>(t);
                return true;
            }
        }
        return false;
    } else {
        int b_id = state.box_ids[box_idx];
        if ((state.target_mask & (1 << b_id)) && initial_targets[b_id] == p) {
            out_idx = b_id;
            return true;
        }
        return false;
    }
}

// 动态隧道检测：若推动方向两侧都是墙，则箱子/炸弹可沿隧道自动滑行。
// blown_mask 会让已炸开的墙不再被视作实体墙。
template <GameMode Mode>
inline bool Sokoban::is_tunnel_dynamic(point p, int dir, uint8_t blown_mask) const {
    (void)Mode;
    if (MOVE[dir].x == 0) {
        bool l = (p.x - 1 < 0) || is_solid({(int8_t)(p.x - 1), p.y}, blown_mask);
        bool r = (p.x + 1 > MAP_MAX_WIDTH - 1) || is_solid({(int8_t)(p.x + 1), p.y}, blown_mask);
        return l && r;
    } else {
        bool d_wall = (p.y - 1 < 0) || is_solid({p.x, (int8_t)(p.y - 1)}, blown_mask);
        bool u_wall = (p.y + 1 > MAP_MAX_HEIGHT - 1) || is_solid({p.x, (int8_t)(p.y + 1)}, blown_mask);
        return u_wall && d_wall;
    }
}

/// \brief 根据当前玩家可达区生成所有合法推动作
///
/// \details
/// 这是搜索热路径的一部分。箱子和炸弹分开枚举，目的是让常见的箱子推动路径
/// 避开 is_bomb_entity 分支；碰撞查询统一使用 NodeOccupancy，避免重复线性扫描。
template <GameMode Mode>
__attribute__((always_inline)) inline int Sokoban::generate_moves(
    const GameState& state,
    const NodeOccupancy& occupancy,
    int active_bombs,
    TinyMove moves[MAX_NODE_MOVES]) {
    int num_moves = 0;

    auto add_move = [&](uint8_t entity_idx, uint8_t dir, uint8_t walk_dist, uint8_t slide_dist, bool triggers_explosion) {
        if (num_moves < MAX_NODE_MOVES) {
            moves[num_moves++] = {entity_idx, dir, walk_dist, slide_dist, triggers_explosion};
        }
    };

    // 箱子是绝大多数节点的主路径。单独循环可去掉热路径里的 is_bomb_entity 分支。
    for (uint8_t i = 0; i < state.num_boxes; ++i) {
        point pos = {state.box_x[i], state.box_y[i]};

        for (uint8_t dir = 0; dir < 4; ++dir) {
            point push_from = pos - MOVE[dir];
            if (is_overstep(push_from) || bfs_visited_gen[push_from.y][push_from.x] != current_gen) continue;

            point push_to = pos + MOVE[dir];
            if (is_overstep(push_to) || is_solid(push_to, state.blown_mask)) continue;
            if (occupancy.box_at(push_to) != -1 || occupancy.bomb_at<Mode>(push_to) != -1) continue;

            if constexpr (Mode == GameMode::PHASE1_ANY) {
                int dummy_t;
                if (is_dead[push_to.y][push_to.x] && !is_active_target_cell<Mode>(state, push_to, i, dummy_t)) {
                    SOKOBAN_PROFILE_INC(static_deadlock_prunes);
                    continue;
                }
            } else {
                if (t_dist[state.box_ids[i]][push_to.y][push_to.x] == -1) {
                    SOKOBAN_PROFILE_INC(static_deadlock_prunes);
                    continue;
                }
            }

            // 2x2 局部死锁判定：若推入后形成不可解团块，直接剪枝。
            int dummy_t;
            if (!is_active_target_cell<Mode>(state, push_to, i, dummy_t)) {
                auto is_permanent_wall = [&](point cp) {
                    if (is_overstep(cp)) return true;
                    return map[cp.y][cp.x] == 1 && wall_clear_mask[cp.y][cp.x] == 0;
                };
                bool horizontal_lock = is_permanent_wall(push_to + MOVE[1]) || is_permanent_wall(push_to + MOVE[3]);
                bool vertical_lock = is_permanent_wall(push_to + MOVE[0]) || is_permanent_wall(push_to + MOVE[2]);
                if (horizontal_lock && vertical_lock) {
                    SOKOBAN_PROFILE_INC(static_deadlock_prunes);
                    continue;
                }

                auto get_next_solid_type = [&](point cp) -> int {
                    if (is_overstep(cp)) return 1;
                    if (map[cp.y][cp.x] == 1) {
                        if ((wall_clear_mask[cp.y][cp.x] & ((1 << num_bomb_tasks) - 1)) != 0) return 0;
                        return 1;
                    }
                    int bmb_id = occupancy.bomb_at<Mode>(cp);
                    if (bmb_id != -1) {
                        if (bomb_tasks[bmb_id].target_wall.x != -1) return 0;
                        return 1;
                    }
                    int box_id = -1;
                    if (cp == push_to) box_id = i;
                    else if (cp == pos) return 0;
                    else box_id = occupancy.box_at(cp);

                    if (box_id != -1) {
                        int out_idx;
                        if (is_active_target_cell<Mode>(state, cp, box_id, out_idx)) return 3;
                        return 2;
                    }
                    return 0;
                };

                bool is_2x2_deadlock = false;
                for (int dy = -1; dy <= 0; ++dy) {
                    for (int dx = -1; dx <= 0; ++dx) {
                        int solid_count = 0;
                        bool has_non_target_box = false;
                        for (int cy = 0; cy <= 1; ++cy) {
                            for (int cx = 0; cx <= 1; ++cx) {
                                point cp = {(int8_t)(push_to.x + dx + cx), (int8_t)(push_to.y + dy + cy)};
                                int st = get_next_solid_type(cp);
                                if (st > 0) solid_count++;
                                if (st == 2) has_non_target_box = true;
                            }
                        }
                        if (solid_count == 4 && has_non_target_box) {
                            is_2x2_deadlock = true;
                            break;
                        }
                    }
                    if (is_2x2_deadlock) break;
                }
                if (is_2x2_deadlock) {
                    SOKOBAN_PROFILE_INC(block_2x2_prunes);
                    continue;
                }
            }

            int slide_dist = 0;
            point final_push_to = push_to;
            while (is_tunnel_dynamic<Mode>(final_push_to, dir, state.blown_mask)) {
                if (is_active_target_cell<Mode>(state, final_push_to, i, dummy_t)) break;

                point next_p = final_push_to + MOVE[dir];
                if (is_overstep(next_p)) break;
                if (is_solid(next_p, state.blown_mask) || occupancy.box_at(next_p) != -1 || occupancy.bomb_at<Mode>(next_p) != -1) break;

                final_push_to = next_p;
                slide_dist++;
            }

            add_move(i, dir, static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x]), static_cast<uint8_t>(slide_dist), false);
        }
    }

    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        // 炸弹规则包含目标墙爆炸，和箱子死锁规则完全不同；拆开后箱子主路径更干净。
        if (active_bombs > 0) {
            for (uint8_t b_idx = 0; b_idx < state.num_bombs; ++b_idx) {
                if (state.blown_mask & (1 << b_idx)) continue;
                if (bomb_tasks[b_idx].target_wall.x == -1) continue;

                uint8_t entity_idx = static_cast<uint8_t>(state.num_boxes + b_idx);
                point pos = {state.bomb_x[b_idx], state.bomb_y[b_idx]};

                for (uint8_t dir = 0; dir < 4; ++dir) {
                    point push_from = pos - MOVE[dir];
                    if (is_overstep(push_from) || bfs_visited_gen[push_from.y][push_from.x] != current_gen) continue;

                    point push_to = pos + MOVE[dir];
                    if (is_overstep(push_to)) continue;

                    bool triggers_explosion = false;
                    if (is_solid(push_to, state.blown_mask)) {
                        if (push_to == bomb_tasks[b_idx].target_wall) triggers_explosion = true;
                        else continue;
                    }
                    if (!triggers_explosion && (occupancy.box_at(push_to) != -1 || occupancy.bomb_at<Mode>(push_to) != -1)) continue;

                    if (!triggers_explosion && b_dist[b_idx][push_to.y][push_to.x] == -1) {
                        SOKOBAN_PROFILE_INC(static_deadlock_prunes);
                        continue;
                    }

                    int slide_dist = 0;
                    if (!triggers_explosion) {
                        point final_push_to = push_to;
                        while (is_tunnel_dynamic<Mode>(final_push_to, dir, state.blown_mask)) {
                            point next_p = final_push_to + MOVE[dir];
                            if (is_overstep(next_p)) break;
                            if (next_p == bomb_tasks[b_idx].target_wall) break;
                            if (is_solid(next_p, state.blown_mask) || occupancy.box_at(next_p) != -1 || occupancy.bomb_at<Mode>(next_p) != -1) break;

                            final_push_to = next_p;
                            slide_dist++;
                        }
                    }

                    add_move(entity_idx, dir, static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x]), static_cast<uint8_t>(slide_dist), triggers_explosion);
                }
            }
        }
    }

    return num_moves;
}

void Sokoban::build_level_from_state(const GameState& state, SokobanLevel& out_level) const {
    out_level = SokobanLevel{};
    out_level.map = map;
    out_level.player_start = state.player;

    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            if ((wall_clear_mask[y][x] & state.blown_mask) != 0) {
                out_level.map[y][x] = 0;
            }
        }
    }

    out_level.box_count = state.num_boxes;
    for (int i = 0; i < state.num_boxes; ++i) {
        out_level.boxes[i] = {state.box_x[i], state.box_y[i]};
        out_level.box_ids[i] = state.box_ids[i];
    }

    out_level.target_count = initial_targets.size();
    for (int i = 0; i < initial_targets.size(); ++i) {
        out_level.targets[i] = initial_targets[i];
    }

    out_level.bomb_count = state.num_bombs;
    for (int b = 0; b < state.num_bombs; ++b) {
        if (state.blown_mask & (1 << b)) out_level.bombs[b] = {-1, -1};
        else out_level.bombs[b] = {state.bomb_x[b], state.bomb_y[b]};
    }
}

uint8_t Sokoban::infer_final_bomb_push_dir(point final_player, point target_wall) const {
    for (uint8_t d = 0; d < 4; ++d) {
        if (final_player + MOVE[d] == target_wall) return d;
    }
    return 4;
}

bool Sokoban::build_bomb_macro_path(
    const GameState& state,
    int bomb_idx,
    StaticArray<point, MAX_PATH_LENGTH>& out_path) const {
    out_path.clear();
    if (bomb_idx < 0 || bomb_idx >= state.num_bombs) return false;
    if (state.blown_mask & (1 << bomb_idx)) return false;
    if (bomb_tasks[bomb_idx].target_wall.x == -1) return false;
    if (b_dist[bomb_idx][state.bomb_y[bomb_idx]][state.bomb_x[bomb_idx]] == -1) return false;

    SokobanLevel macro_level;
    build_level_from_state(state, macro_level);

    BombTask task = bomb_tasks[bomb_idx];
    task.bomb_start = {state.bomb_x[bomb_idx], state.bomb_y[bomb_idx]};
    task.box_pushes.clear();

    if (!PlanningCommon::get_bomb_push_path(macro_level, state.player, task, out_path)) return false;
    if (out_path.empty() || out_path.size() > SOKOBAN_BOMB_MACRO_MAX_PATH) return false;

    point final_player = out_path.back();
    return infer_final_bomb_push_dir(final_player, task.target_wall) < 4;
}

template <GameMode Mode>
int Sokoban::generate_bomb_macros(
    const GameState& state,
    int h_before,
    int active_entities,
    MacroMove macros[MAX_NODE_MACROS]) const {
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        (void)state;
        (void)h_before;
        (void)active_entities;
        (void)macros;
        return 0;
    } else {
#if !SOKOBAN_ENABLE_BOMB_MACRO
        (void)state;
        (void)h_before;
        (void)active_entities;
        (void)macros;
        return 0;
#else
        int count = 0;
        for (uint8_t b = 0; b < state.num_bombs && count < MAX_NODE_MACROS; ++b) {
            if (state.blown_mask & (1 << b)) continue;
            if (bomb_tasks[b].target_wall.x == -1) continue;

            StaticArray<point, MAX_PATH_LENGTH> macro_path;
            if (!build_bomb_macro_path(state, b, macro_path)) continue;

            point final_player = macro_path.back();
            uint8_t final_dir = infer_final_bomb_push_dir(final_player, bomb_tasks[b].target_wall);
            if (final_dir >= 4) continue;

            MacroMove& mv = macros[count];
            mv.bomb_idx = b;
            mv.entity_idx = static_cast<uint8_t>(state.num_boxes + b);
            mv.final_push_dir = final_dir;
            mv.path_cost = static_cast<uint16_t>(macro_path.size());
            mv.next_state = state;
            mv.next_state.player = final_player;
            mv.next_state.blown_mask = static_cast<uint8_t>(state.blown_mask | (1 << b));

            uint32_t new_hash = state.hash;
            new_hash ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
            new_hash ^= ZOBRIST_PLAYER[final_player.y][final_player.x];
            new_hash ^= ZOBRIST_BLOWN_MASK[state.blown_mask];
            new_hash ^= ZOBRIST_BLOWN_MASK[mv.next_state.blown_mask];
            new_hash ^= ZOBRIST_BOMB[b][state.bomb_y[b]][state.bomb_x[b]];
            mv.next_state.hash = new_hash;

            int h_after = get_heuristic<Mode>(mv.next_state);
            if (h_after >= 9999) continue;

            int benefit = h_before - h_after;
            int sort_key = static_cast<int>(mv.path_cost) * 10 -
                           benefit * heuristic_weight_num<Mode>(active_entities) - 120;
            int direct_lb = b_dist[b][state.bomb_y[b]][state.bomb_x[b]];
            if (direct_lb >= 0 && direct_lb <= 3) sort_key -= (4 - direct_lb) * 35;
            if (bomb_tasks[b].is_essential) sort_key -= 50;
            if (sort_key < -32000) sort_key = -32000;
            if (sort_key > 32000) sort_key = 32000;
            mv.sort_key = static_cast<int16_t>(sort_key);
            ++count;
        }
        return count;
#endif
    }
}


// ============================================================================
// 模块 3：成功路径回放与后处理
// ============================================================================

static inline uint8_t dir_between(point a, point b) {
    for (uint8_t d = 0; d < 4; ++d) {
        if (a + MOVE[d] == b) return d;
    }
    return 4;
}

static int count_path_turns(const StaticArray<point, MAX_PATH_LENGTH>& path) {
    int turns = 0;
    uint8_t last_dir = 4;
    for (int i = 1; i < path.size(); ++i) {
        uint8_t d = dir_between(path[i - 1], path[i]);
        if (d == 4) continue;
        if (last_dir != 4 && d != last_dir) ++turns;
        last_dir = d;
    }
    return turns;
}

/// \brief 在找到推箱解后，重新拼接某段玩家行走路径
///
/// \details
/// IDA* 搜索阶段只记录推动作；成功后才回放玩家走位。这里在允许少量绕路的前提下
/// 优先减少转弯次数，方便下游小车执行。该函数不参与搜索节点扩展。
template <GameMode Mode>
bool Sokoban::append_optimized_walk_segment(
    const GameState& state,
    point start,
    point goal,
    uint8_t prev_dir,
    uint8_t next_dir,
    int max_steps,
    StaticArray<point, MAX_PATH_LENGTH>& out_path,
    uint8_t& out_dir) const {
    if (start == goal) return true;

    struct Node {
        int16_t cost;
        int16_t steps;
        point parent;
        uint8_t parent_dir;
        bool used;
    };

    static Node nodes[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            for (int d = 0; d < 4; ++d) {
                nodes[y][x][d].cost = 32767;
                nodes[y][x][d].steps = 32767;
                nodes[y][x][d].parent = {-1, -1};
                nodes[y][x][d].parent_dir = 4;
                nodes[y][x][d].used = false;
            }
        }
    }

    auto blocked = [&](point p) {
        if (is_overstep(p) || is_solid(p, state.blown_mask)) return true;
        if (find_box_id(state, p) != -1) return true;
        if (get_bomb_id(state, p, Mode) != -1) return true;
        return false;
    };

    for (uint8_t d = 0; d < 4; ++d) {
        nodes[start.y][start.x][d].cost = 0;
        nodes[start.y][start.x][d].steps = 0;
    }

    for (int iter = 0; iter < MAP_CELL_COUNT * 4; ++iter) {
        int best = 32767;
        point curr = {-1, -1};
        uint8_t curr_dir = 4;
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                for (uint8_t d = 0; d < 4; ++d) {
                    if (!nodes[y][x][d].used && nodes[y][x][d].cost < best) {
                        best = nodes[y][x][d].cost;
                        curr = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
                        curr_dir = d;
                    }
                }
            }
        }
        if (curr_dir == 4) break;
        nodes[curr.y][curr.x][curr_dir].used = true;

        for (uint8_t nd = 0; nd < 4; ++nd) {
            point np = curr + MOVE[nd];
            if (blocked(np) && np != goal) continue;
            int next_steps = nodes[curr.y][curr.x][curr_dir].steps + 1;
            if (next_steps > max_steps) continue;
            int turn_penalty = 0;
            if (!(curr == start && prev_dir == 4) && curr_dir != nd) turn_penalty += 1;
            if (np == goal && next_dir < 4 && nd != next_dir) turn_penalty += 1;
            int cand = best + 1 + turn_penalty * 32;
            if (cand < nodes[np.y][np.x][nd].cost) {
                nodes[np.y][np.x][nd].cost = cand;
                nodes[np.y][np.x][nd].steps = next_steps;
                nodes[np.y][np.x][nd].parent = curr;
                nodes[np.y][np.x][nd].parent_dir = curr_dir;
            }
        }
    }

    int best = 32767;
    uint8_t best_dir = 4;
    for (uint8_t d = 0; d < 4; ++d) {
        if (nodes[goal.y][goal.x][d].cost < best) {
            best = nodes[goal.y][goal.x][d].cost;
            best_dir = d;
        }
    }
    if (best_dir == 4) return false;

    StaticArray<point, MAX_PATH_LENGTH> temp;
    point curr = goal;
    uint8_t curr_dir = best_dir;
    while (curr != start) {
        temp.push_back(curr);
        Node n = nodes[curr.y][curr.x][curr_dir];
        curr = n.parent;
        curr_dir = n.parent_dir;
        if (curr.x == -1) return false;
    }
    for (int i = temp.size() - 1; i >= 0; --i) out_path.push_back(temp[i]);
    out_dir = best_dir;
    return true;
}

/// \brief 对最终路径做轻量后处理，尝试减少行走转弯
///
/// \details
/// 推箱动作本身保持不变，只替换相邻推动作之间的玩家行走段。
/// 若优化结果没有收益或路径变长过多，则保留原始路径。
template <GameMode Mode>
void Sokoban::optimize_final_path_turns() {
    if (final_path.size() <= 2) return;

    StaticArray<point, MAX_PATH_LENGTH> original = final_path;
    int original_turns = count_path_turns(original);
    GameState state = initial_state;
    StaticArray<point, MAX_PATH_LENGTH> optimized;
    optimized.push_back(final_path[0]);

    uint8_t prev_dir = 4;
    int i = 1;
    while (i < final_path.size()) {
        point curr = optimized.back();
        int movable_idx = -1;
        bool movable_is_bomb = false;
        uint8_t push_dir = 4;
        point push_to = {-1, -1};

        for (uint8_t d = 0; d < 4 && movable_idx == -1; ++d) {
            point obj = curr + MOVE[d];
            point to = obj + MOVE[d];
            int box_id = find_box_id(state, obj);
            if (box_id != -1 && obj == final_path[i]) {
                movable_idx = box_id;
                movable_is_bomb = false;
                push_dir = d;
                push_to = to;
                break;
            }
            int bomb_id = get_bomb_id(state, obj, Mode);
            if (bomb_id != -1 && obj == final_path[i]) {
                movable_idx = bomb_id;
                movable_is_bomb = true;
                push_dir = d;
                push_to = to;
                break;
            }
        }

        if (movable_idx != -1) {
            optimized.push_back(final_path[i]);
            prev_dir = push_dir;
            if (movable_is_bomb) {
                if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                    state.bomb_x[movable_idx] = push_to.x;
                    state.bomb_y[movable_idx] = push_to.y;
                }
            } else {
                bool box_finished = false;
                if constexpr (Mode == GameMode::PHASE1_ANY) {
                    for (size_t t = 0; t < initial_targets.size(); ++t) {
                        if ((state.target_mask & (1 << t)) && initial_targets[t] == push_to) {
                            state.target_mask &= ~(1 << t);
                            box_finished = true;
                            break;
                        }
                    }
                } else {
                    uint8_t b_id = state.box_ids[movable_idx];
                    if ((state.target_mask & (1 << b_id)) && initial_targets[b_id] == push_to) {
                        state.target_mask &= ~(1 << b_id);
                        box_finished = true;
                    }
                }

                if (box_finished) {
                    for (int b = movable_idx; b < state.num_boxes - 1; ++b) {
                        state.box_x[b] = state.box_x[b + 1];
                        state.box_y[b] = state.box_y[b + 1];
                        state.box_ids[b] = state.box_ids[b + 1];
                    }
                    state.num_boxes--;
                } else {
                    state.box_x[movable_idx] = push_to.x;
                    state.box_y[movable_idx] = push_to.y;
                }
            }
            state.player = push_to - MOVE[push_dir];
            ++i;
            continue;
        }

        int walk_start = i;
        int walk_end = i;
        while (walk_end + 1 < final_path.size()) {
            point p = final_path[walk_end];
            point next = final_path[walk_end + 1];
            bool next_is_push = false;
            for (uint8_t d = 0; d < 4 && !next_is_push; ++d) {
                point obj = p + MOVE[d];
                if (obj != next) continue;
                if (find_box_id(state, obj) != -1 || get_bomb_id(state, obj, Mode) != -1) next_is_push = true;
            }
            if (next_is_push) break;
            walk_end++;
        }

        uint8_t next_dir = 4;
        if (walk_end + 1 < final_path.size()) next_dir = dir_between(final_path[walk_end], final_path[walk_end + 1]);

        uint8_t out_dir = prev_dir;
        int original_walk_steps = walk_end - walk_start + 1;
        int max_walk_steps = original_walk_steps + 6;
        if (!append_optimized_walk_segment<Mode>(state, curr, final_path[walk_end], prev_dir, next_dir, max_walk_steps, optimized, out_dir)) {
            for (int k = walk_start; k <= walk_end; ++k) optimized.push_back(final_path[k]);
            out_dir = dir_between(curr, final_path[walk_end]);
        }
        if (optimized.size() > 1) prev_dir = out_dir;
        state.player = final_path[walk_end];
        i = walk_end + 1;
    }

    int optimized_turns = count_path_turns(optimized);
    if (optimized_turns < original_turns || optimized.size() <= original.size()) {
        final_path = optimized;
    } else {
        final_path = original;
    }
}


// ============================================================================
// 模块 4：哈希、匹配与启发式估价
// ============================================================================

/// \brief 按阶段规则计算搜索状态的 Zobrist 哈希
/// \tparam Mode 当前求解模式
/// \param state 搜索状态
/// \return 32 位 Zobrist 哈希值
template <GameMode Mode> 
uint32_t Sokoban::compute_hash(const GameState& state) const {
    uint32_t h = 0;
    for (int i = 0; i < state.num_boxes; ++i) {
        if constexpr (Mode == GameMode::PHASE1_ANY) 
            h ^= ZOBRIST_BOX[state.box_y[i]][state.box_x[i]];
        else 
            h ^= ZOBRIST_SPECIFIC_BOX[state.box_ids[i]][state.box_y[i]][state.box_x[i]];
    }
    h ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
    for (size_t i = 0; i < initial_targets.size(); ++i) {
        if (state.target_mask & (1 << i)) h ^= ZOBRIST_TARGET[i];
    }
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        h ^= ZOBRIST_BLOWN_MASK[state.blown_mask];
        for (int b = 0; b < state.num_bombs; ++b) {
            if (!(state.blown_mask & (1 << b))) {
                h ^= ZOBRIST_BOMB[b][state.bomb_y[b]][state.bomb_x[b]];
            }
        }
    }
    return h;
}

/// \brief 最小权二分匹配
/// \tparam N 矩阵最大维度
/// \param cost 代价矩阵
/// \param n 实际匹配规模
/// \return 最小匹配总代价；不可匹配时返回 9999
///
/// \details
/// 用于第一阶段启发式，估计“任意箱子到任意目标”的乐观下界
template<size_t N>
int Sokoban::min_weight_assignment(int cost[N][N], int n) const {
    if (n == 0) return 0;
    
    int u[N + 1] = {0}, v[N + 1] = {0}, p[N + 1] = {0}, way[N + 1] = {0};                 
    int minv[N + 1];                      
    bool used[N + 1];                     

    for (int i = 1; i <= n; ++i) {
        p[0] = i;  
        int j0 = 0;  
        for (int k = 0; k <= n; ++k) { minv[k] = 9999; used[k] = false; }

        do {
            used[j0] = true;  
            int i0 = p[j0], delta = 9999, j1 = 0;
            
            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    int cur = cost[i0 - 1][j - 1] - u[i0] - v[j];   
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            
            for (int j = 0; j <= n; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; } 
                else { minv[j] -= delta; }  
            }
            j0 = j1;  
        } while (p[j0] != 0);  

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    int total_cost = 0;
    for (int j = 1; j <= n; ++j) {
        int c = cost[p[j] - 1][j - 1];
        if (c >= 9999) return 9999; 
        total_cost += c;
    }
    return total_cost;
}

/// \brief 估计多任务之间的玩家行走下界
/// \param player 当前玩家位置
/// \param starts 每个任务的当前物体位置
/// \param ends 每个任务完成后的目标区域
/// \param task_count 有效任务数量
/// \return 串联所有任务至少需要的乐观行走代价
///
/// \details
/// 箱子/炸弹的推动代价由调用方单独累加。这里专门补足“完成一个任务后，
/// 再赶到下一个任务首推位”的行走代价。小规模任务用 bitmask DP 估计访问顺序；
/// 大规模任务退回 MST 下界，避免在热路径里产生过大的计算量。
int Sokoban::task_route_walk_lower_bound(
    point player,
    const point starts[MAX_BOXES + MAX_BOMBS],
    const point ends[MAX_BOXES + MAX_BOMBS],
    int task_count) const {
    if (task_count <= 0) return 0;

    auto walk_to_task = [&](point from, point obj) {
        int d = walk_to_push_stand_lower_bound(from, obj);
        if (d == 9999) d = std::abs(from.x - obj.x) + std::abs(from.y - obj.y) - 1;
        return (d < 0) ? 0 : d;
    };

    int entry[MAX_BOXES + MAX_BOMBS];
    int trans[MAX_BOXES + MAX_BOMBS][MAX_BOXES + MAX_BOMBS];

    for (int i = 0; i < task_count; ++i) {
        entry[i] = walk_to_task(player, starts[i]);
        for (int j = 0; j < task_count; ++j) {
            trans[i][j] = (i == j) ? 0 : walk_to_task(ends[i], starts[j]);
        }
    }

    constexpr int MAX_ROUTE_DP_TASKS = 6;
    if (task_count <= MAX_ROUTE_DP_TASKS) {
        constexpr int MAX_MASK = 1 << MAX_ROUTE_DP_TASKS;
        int dp[MAX_MASK][MAX_ROUTE_DP_TASKS];
        int full_mask = 1 << task_count;

        for (int mask = 0; mask < full_mask; ++mask) {
            for (int i = 0; i < task_count; ++i) dp[mask][i] = 9999;
        }
        for (int i = 0; i < task_count; ++i) dp[1 << i][i] = entry[i];

        for (int mask = 1; mask < full_mask; ++mask) {
            for (int last = 0; last < task_count; ++last) {
                int base = dp[mask][last];
                if (base >= 9999) continue;
                for (int next = 0; next < task_count; ++next) {
                    if (mask & (1 << next)) continue;
                    int next_mask = mask | (1 << next);
                    int cand = base + trans[last][next];
                    if (cand < dp[next_mask][next]) dp[next_mask][next] = cand;
                }
            }
        }

        int best = 9999;
        for (int last = 0; last < task_count; ++last) {
            if (dp[full_mask - 1][last] < best) best = dp[full_mask - 1][last];
        }
        return best;
    }

    int best_entry = entry[0];
    for (int i = 1; i < task_count; ++i) {
        if (entry[i] < best_entry) best_entry = entry[i];
    }

    bool used[MAX_BOXES + MAX_BOMBS] = {};
    int min_edge[MAX_BOXES + MAX_BOMBS];
    for (int i = 0; i < task_count; ++i) min_edge[i] = 9999;
    min_edge[0] = 0;

    int mst = 0;
    for (int iter = 0; iter < task_count; ++iter) {
        int v = -1;
        for (int i = 0; i < task_count; ++i) {
            if (!used[i] && (v == -1 || min_edge[i] < min_edge[v])) v = i;
        }
        if (v == -1) break;
        used[v] = true;
        mst += min_edge[v];

        for (int to = 0; to < task_count; ++to) {
            if (used[to]) continue;
            int e = (trans[v][to] < trans[to][v]) ? trans[v][to] : trans[to][v];
            if (e < min_edge[to]) min_edge[to] = e;
        }
    }

    return best_entry + mst;
}

/// \brief 计算 IDA* 启发式下界
/// \tparam Mode 当前求解模式
/// \param state 当前搜索状态
/// \return 乐观剩余代价；返回 9999 表示该状态不可解
///
/// \details
/// Phase1 使用最小权匹配估计箱子到目标的总推动代价
/// Phase2 使用语义绑定后的固定目标，并把炸弹任务也纳入估价
template <GameMode Mode>
int Sokoban::get_heuristic(const GameState& state) const {
    if (state.num_boxes == 0) return 0; 

    if constexpr (Mode == GameMode::PHASE1_ANY) {
        int min_p_dist = 9999;
        for (int i = 0; i < state.num_boxes; ++i) {
            point box_pos = {state.box_x[i], state.box_y[i]};
            int d = walk_to_push_stand_lower_bound(state.player, box_pos);
            if (d == 9999) d = std::abs(state.player.x - state.box_x[i]) + std::abs(state.player.y - state.box_y[i]) - 1;
            if (d < 0) d = 0;
            if (d < min_p_dist) min_p_dist = d;
        }
        int p_cost = (min_p_dist != 9999) ? min_p_dist : 0;

        int active_t_idx[MAX_BOXES]; 
        int t_count = 0;
        for (size_t i = 0; i < initial_targets.size(); ++i) {
            if (state.target_mask & (1 << i)) active_t_idx[t_count++] = i;
        }
        if (t_count < state.num_boxes) return 9999; 

        int cost_matrix[MAX_BOXES][MAX_BOXES];
        for (int i = 0; i < state.num_boxes; ++i) {
            for (int j = 0; j < state.num_boxes; ++j) {
                int dist = t_dist[active_t_idx[j]][state.box_y[i]][state.box_x[i]];
                cost_matrix[i][j] = (dist == -1) ? 9999 : dist;
            }
        }

        int min_h = min_weight_assignment<MAX_BOXES>(cost_matrix, state.num_boxes);
        if (min_h >= 9999) return 9999; 
        return min_h + p_cost;
    } 
    else {
        point starts[MAX_BOXES + MAX_BOMBS];
        point ends[MAX_BOXES + MAX_BOMBS];
        int task_count = 0;
        int sum_push = 0;

        for (int i = 0; i < state.num_boxes; ++i) {
            int id = state.box_ids[i];
            int dist = t_dist[id][state.box_y[i]][state.box_x[i]];
            if (dist == -1) return 9999; // 语义目标不可达，视为死锁。

            starts[task_count] = {state.box_x[i], state.box_y[i]};
            ends[task_count] = initial_targets[id];
            task_count++;
            sum_push += dist;
        }

        for (int b = 0; b < state.num_bombs; ++b) {
            if (!(state.blown_mask & (1 << b)) && bomb_tasks[b].target_wall.x != -1) {
                int d = b_dist[b][state.bomb_y[b]][state.bomb_x[b]];
                if (d == -1) return 9999;

                starts[task_count] = {state.bomb_x[b], state.bomb_y[b]};
                ends[task_count] = bomb_tasks[b].target_wall;
                task_count++;
                sum_push += d + 2;
            }
        }

        return sum_push + task_route_walk_lower_bound(state.player, starts, ends, task_count);
    }
}



// ============================================================================
// 模块 5：空间预计算
// ============================================================================

// 初始化所有 Zobrist 随机表，用于 O(1) 增量哈希
void Sokoban::init_zobrist() {
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            ZOBRIST_BOX[y][x] = xorshift32();
            ZOBRIST_PLAYER[y][x] = xorshift32();

            for (int i = 0; i < SystemConfig::MAX_BOXES; ++i) {
                ZOBRIST_SPECIFIC_BOX[i][y][x] = xorshift32();
            }
        }
    }
    for (int i = 0; i < MAX_BOXES; ++i) ZOBRIST_TARGET[i] = xorshift32();
}

/// \brief 预计算箱子到各目标点的反向推动距离场
///
/// \details
/// 从每个目标点反向 BFS，计算箱子从任意格被推到该目标点的最少推动步数
/// 结果写入 t_dist[target_id][y][x]，-1 表示不可达或死锁
void Sokoban::precompute_target_distances() {
    std::memset(t_dist, -1, sizeof(t_dist));
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1; 

    for (size_t i = 0; i < initial_targets.size(); ++i) {
            point start = initial_targets[i];
            point q[MAP_CELL_COUNT];
            int head = 0, tail = 0;
            
            q[tail++] = start;
            t_dist[i][start.y][start.x] = 0; 
            
            while (head < tail) {
                point curr = q[head++];
                int dist = t_dist[i][curr.y][curr.x];

            for (int dir = 0; dir < 4; ++dir) {
                point box_prev = curr - MOVE[dir];
                point player_prev = curr - MOVE[dir] - MOVE[dir];  // 反向推导时玩家需要站立的位置。
                
                if (is_overstep(box_prev) || is_overstep(player_prev)) continue;

                if (!is_solid(box_prev, all_blown_mask) && !is_solid(player_prev, all_blown_mask)) {
                    if (t_dist[i][box_prev.y][box_prev.x] == -1) {
                        t_dist[i][box_prev.y][box_prev.x] = dist + 1; 
                        q[tail++] = box_prev;        
                    }
                }
            }
        }
    }
}

/// \brief 预计算静态地图上的玩家行走距离
///
/// \details
/// 该距离忽略箱子和炸弹，只考虑墙体以及已清墙状态，用于启发式中的乐观行走下界。
/// 结果写入 relaxed_walk_dist[from_cell][to_cell]，255 表示不可达。
void Sokoban::precompute_walk_distances() {
    std::memset(relaxed_walk_dist, 255, sizeof(relaxed_walk_dist));
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1;

    for (int sy = 0; sy < MAP_MAX_HEIGHT; ++sy) {
        for (int sx = 0; sx < MAP_MAX_WIDTH; ++sx) {
            point start = {static_cast<int8_t>(sx), static_cast<int8_t>(sy)};
            int start_idx = sy * MAP_MAX_WIDTH + sx;
            if (is_solid(start, all_blown_mask)) continue;

            point q[MAP_CELL_COUNT];
            int head = 0, tail = 0;
            q[tail++] = start;
            relaxed_walk_dist[start_idx][start_idx] = 0;

            while (head < tail) {
                point curr = q[head++];
                int curr_idx = curr.y * MAP_MAX_WIDTH + curr.x;
                uint8_t curr_dist = relaxed_walk_dist[start_idx][curr_idx];

                for (int d = 0; d < 4; ++d) {
                    point np = curr + MOVE[d];
                    if (is_overstep(np) || is_solid(np, all_blown_mask)) continue;

                    int next_idx = np.y * MAP_MAX_WIDTH + np.x;
                    if (relaxed_walk_dist[start_idx][next_idx] == 255) {
                        relaxed_walk_dist[start_idx][next_idx] = curr_dist + 1;
                        q[tail++] = np;
                    }
                }
            }
        }
    }
}

/// \brief 预计算炸弹到目标墙体的反向推动距离场
///
/// \details
/// 只对已经绑定了 target_wall 的炸弹任务计算距离。
/// 结果写入 b_dist[bomb_id][y][x]，用于第二阶段启发式估价和剪枝。
void Sokoban::precompute_bomb_distances() {
    std::memset(b_dist, -1, sizeof(b_dist));
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1;

    for (int b = 0; b < num_bomb_tasks; ++b) {
        if (bomb_tasks[b].target_wall.x == -1) continue;
        point start = bomb_tasks[b].target_wall;

        point q[MAP_CELL_COUNT];
        int head = 0, tail = 0;
        
        q[tail++] = start;
        b_dist[b][start.y][start.x] = 0;

        while (head < tail) {
            point curr = q[head++];
            int dist = b_dist[b][curr.y][curr.x];

            for (int dir = 0; dir < 4; ++dir) {
                point bomb_prev = curr - MOVE[dir];
                point player_prev = curr - MOVE[dir] - MOVE[dir];
                
                if (is_overstep(bomb_prev) || is_overstep(player_prev)) continue;

                if (!is_solid(bomb_prev, all_blown_mask) && !is_solid(player_prev, all_blown_mask)) {
                    if (b_dist[b][bomb_prev.y][bomb_prev.x] == -1) {
                        b_dist[b][bomb_prev.y][bomb_prev.x] = dist + 1;
                        q[tail++] = bomb_prev;
                    }
                }
            }
        }
    }
}

/// \brief 根据目标距离场标记绝对死锁格
///
/// \details
/// 若某个格子无法把箱子推向任何目标点，则该格子为绝对死锁区
/// 搜索时箱子进入这类格子会被直接剪枝
void Sokoban::precompute_deadlocks() {
    std::memset(is_dead, true, sizeof(is_dead)); 
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1; 
    
    for (int8_t y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int8_t x = 0; x < MAP_MAX_WIDTH; ++x) {
            if (is_solid({x, y}, all_blown_mask)) continue;
            
            for (size_t i = 0; i < initial_targets.size(); ++i) {
                if (t_dist[i][y][x] != -1) {
                    is_dead[y][x] = false;
                    break;
                }
            }
        }
    }
}



// ============================================================================
// 模块 6：高频物理查询
// ============================================================================

inline bool Sokoban::is_overstep(point p) const {
    if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
    return false;
}

inline bool Sokoban::is_solid(point p, uint8_t blown_mask) const {
    if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
    if (map[p.y][p.x] != 1) return false; 
    return (wall_clear_mask[p.y][p.x] & blown_mask) == 0; 
}

inline bool Sokoban::is_bomb(point p) const {
    for (const auto& b : initial_bombs) if (b == p) return true;
    return false;
}

inline int Sokoban::find_box_id(const GameState& state, point p) const {
    for (int i = 0; i < state.num_boxes; ++i)
        if (state.box_x[i] == p.x && state.box_y[i] == p.y) return i;
    return -1;
}

inline int Sokoban::get_bomb_id(const GameState& state, point p, GameMode Mode) const {
    if (Mode == GameMode::PHASE1_ANY) return -1; 
    for (int b = 0; b < state.num_bombs; ++b) {
        if (!(state.blown_mask & (1 << b)) && state.bomb_x[b] == p.x && state.bomb_y[b] == p.y) return b;
    }
    return -1;
}

inline int Sokoban::walk_dist_between(point from, point to) const {
    if (is_overstep(from) || is_overstep(to)) return 9999;
    int from_idx = from.y * MAP_MAX_WIDTH + from.x;
    int to_idx = to.y * MAP_MAX_WIDTH + to.x;
    uint8_t d = relaxed_walk_dist[from_idx][to_idx];
    return (d == 255) ? 9999 : d;
}

inline int Sokoban::walk_to_push_stand_lower_bound(point from, point obj) const {
    int best = 9999;
    for (int d = 0; d < 4; ++d) {
        point stand = obj - MOVE[d];
        int w = walk_dist_between(from, stand);
        if (w < best) best = w;
    }
    return best;
}
