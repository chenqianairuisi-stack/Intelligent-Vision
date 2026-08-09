/// \file sokoban_planner.cpp
/// \brief C++ Sokoban 预计算、宏动作混合候选搜索和路径优化实现

#include "Sokoban.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

// ============================================================================
// 参数面板
// ============================================================================

namespace SokobanConfig {
    // 搜索功能开关
#if defined(SOKOBAN_ENABLE_PROFILE)
    inline constexpr bool ENABLE_PROFILE = true;        // 诊断构建记录 expanded/generated/TT 命中等 profile 数据
#else
    inline constexpr bool ENABLE_PROFILE = false;       // 正式构建关闭高频统计
#endif
    inline constexpr bool ENABLE_PATH_POSTOPT = true;   // 是否在成功后进行路径优化
    
    // IDA* 阈值与启发式权重
    inline constexpr int INITIAL_THRESHOLD_BOOST = 0;   // 初始 threshold 额外增量；默认交给启发式本身决定
    inline constexpr int HEURISTIC_WEIGHT_DEN = 10;     // f = g + h * weight / denominator
    inline constexpr int HEURISTIC_WEIGHT_BASE = 12;    // active_entities < 4
    inline constexpr int HEURISTIC_WEIGHT_GE_4 = 18;    // active_entities >= 4
    inline constexpr int HEURISTIC_WEIGHT_GE_5 = 20;    // active_entities >= 5
    inline constexpr int HEURISTIC_WEIGHT_GE_6 = 25;    // active_entities >= 6
    inline constexpr int HEURISTIC_WEIGHT_GE_8 = 35;    // active_entities >= 8

    // 多轮失败后的阈值推进
    inline constexpr bool ENABLE_LATE_THRESHOLD_ACCEL = true;      // 是否启用快速阈值推进
    inline constexpr int LATE_THRESHOLD_ACCEL_MIN_ITERATION = 32;  // 迭代次数达到该值后，开始加快 threshold 前进
    inline constexpr int LATE_THRESHOLD_ACCEL_STEP = 2;            // 加快时每轮至少推进的 threshold 步数

    // 等价玩家位置 TT
    inline constexpr bool ENABLE_CANONICAL_TT = true;              // 是否将归一化玩家位置写入 TT
    inline constexpr int CANONICAL_TT_MIN_THRESHOLD = 120;         // 迭代次数达到该值后启用，避免浅图 TT 命中结构被扰动

    // 单箱到目标总代价下界
    inline constexpr bool ENABLE_BOX_TARGET_COST_LB = true;        // 是否启用基于预计算 box_target_cost 的更精细排序
    inline constexpr int BOX_TARGET_COST_MIN_ITERATION = 40;       // 迭代次数达到该值后启用，避免前期预计算的误导

    // 小规模推箱宏层
    inline constexpr bool ENABLE_SMALL_BOX_MACRO_LAYER = true;     // 是否在根部尝试少箱宏层快解
    inline constexpr int SMALL_BOX_MACRO_MAX_BOXES = 4;            // 只处理极小规模纯推箱局面，避免入口开销扩散
    inline constexpr int SMALL_BOX_MACRO_MIN_COST_LB = 0;          // 静态下界会错杀长通道图，默认不用于预拒绝

    // 炸弹宏动作
    inline constexpr bool ENABLE_BOMB_MACRO = true;                // 是否将推炸弹宏动作加入搜索
    inline constexpr int BOMB_MACRO_MAX_PATH = 80;                 // 宏动作真实展开路径长度上限
    inline constexpr int BOMB_MACRO_THRESHOLD_MARGIN = 32;         // 过滤贴近阈值、通常会被真实路径立即剪掉的宏动作
    inline constexpr int BOMB_MACRO_SORT_BONUS = 120;              // 动作排序中给予宏动作的优先级补偿
    inline constexpr int BOMB_MACRO_MIN_BENEFIT = -9999;           // 宏动作启发式收益门槛；默认只做合法性过滤

}

// ============================================================================
// 编译器分支预测提示
// ============================================================================

#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// ============================================================================
// 全局实例与哈希随机数生成
// ============================================================================

DTCM_DATA Sokoban solver;
DTCM_DATA TTEntry TT[TT_SIZE];  
static uint8_t RELAXED_PUSH_STAND_DIST[MAP_CELL_COUNT][MAP_CELL_COUNT];
static uint16_t DYNAMIC_BOX_EVAL_SEEN[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
static uint16_t dynamic_box_eval_gen = 0;

namespace {
    inline constexpr bool ENABLE_STRICT_COST_REPAIR = true;
    inline constexpr uint32_t STRICT_REPAIR_NODE_BUDGET = 1000;
    inline constexpr uint32_t SMALL_BOX_MACRO_IDA_NODE_BUDGET = 6000;
    inline constexpr bool ENABLE_TURN_ACCESS_DEBT_SORT = true;
    inline constexpr bool ENABLE_DYNAMIC_BLOCK_SORT = true;

    struct PathQualityStats {
        int steps = 0;
        int reversals = 0;
        int repeated_cells = 0;
    };

    int path_step_count(const StaticArray<point, MAX_PATH_LENGTH>& path) {
        int steps = 0;
        for (int i = 1; i < path.size(); ++i) {
            int dx = std::abs(path[i].x - path[i - 1].x);
            int dy = std::abs(path[i].y - path[i - 1].y);
            if (dx + dy == 1) ++steps;
        }
        return steps;
    }

    // 与可视化面板“总代价”一致的路径代价：步数 + 拐点*4（拐点即行走/推动方向的 90° 变化）。
    // 仅用于在多条“已完成且合法”的候选路径之间择优，不参与搜索节点扩展或启发式估计，
    // 因此不涉及可采纳性问题。DISPLAY_TURN_COST 与面板权重保持一致。
    inline constexpr int DISPLAY_TURN_COST = 4;
    int path_display_cost(const StaticArray<point, MAX_PATH_LENGTH>& path) {
        int steps = 0;
        int turns = 0;
        uint8_t last_dir = 4;
        for (int i = 1; i < path.size(); ++i) {
            int dx = path[i].x - path[i - 1].x;
            int dy = path[i].y - path[i - 1].y;
            if (std::abs(dx) + std::abs(dy) != 1) continue;
            uint8_t dir = 4;
            if (dx == 1) dir = 1;
            else if (dx == -1) dir = 3;
            else if (dy == 1) dir = 0;
            else if (dy == -1) dir = 2;
            ++steps;
            if (last_dir < 4 && dir != last_dir) ++turns;
            last_dir = dir;
        }
        return steps + turns * DISPLAY_TURN_COST;
    }

    PathQualityStats analyze_path_quality(const StaticArray<point, MAX_PATH_LENGTH>& path) {
        PathQualityStats stats;
        uint8_t seen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
        uint8_t last_dir = 4;

        for (int i = 0; i < path.size(); ++i) {
            point p = path[i];
            if (p.x >= 0 && p.x < MAP_MAX_WIDTH && p.y >= 0 && p.y < MAP_MAX_HEIGHT) {
                if (seen[p.y][p.x]) ++stats.repeated_cells;
                else seen[p.y][p.x] = 1;
            }
            if (i == 0) continue;

            int dx = path[i].x - path[i - 1].x;
            int dy = path[i].y - path[i - 1].y;
            if (std::abs(dx) + std::abs(dy) != 1) continue;

            uint8_t dir = 4;
            if (dx == 1) dir = 1;
            else if (dx == -1) dir = 3;
            else if (dy == 1) dir = 0;
            else if (dy == -1) dir = 2;

            ++stats.steps;
            if (last_dir < 4 && dir == ((last_dir + 2) & 3)) ++stats.reversals;
            last_dir = dir;
        }
        return stats;
    }

    bool should_run_strict_repair(const StaticArray<point, MAX_PATH_LENGTH>& path) {
        PathQualityStats stats = analyze_path_quality(path);
        if (stats.steps < 55) return false;
        if (stats.steps >= 100 && stats.repeated_cells * 100 >= stats.steps * 32) return true;
        if (stats.reversals >= 4 && stats.repeated_cells * 100 >= stats.steps * 38) return true;
        if (stats.steps >= 90 && stats.reversals >= 3 && stats.repeated_cells * 100 >= stats.steps * 42) return true;
        if (stats.repeated_cells * 100 >= stats.steps * 46) return true;
        return false;
    }
}

static uint32_t xor_state = 123456789;

// 轻量级伪随机数生成器，用于初始化 Zobrist 哈希表
static uint32_t xorshift32() {
    xor_state ^= xor_state << 13; 
    xor_state ^= xor_state >> 17; 
    xor_state ^= xor_state << 5;
    return xor_state;
}

// 热路径计数统一走宏；ENABLE_PROFILE=false 时由 if constexpr 编译掉。
#define SOKOBAN_PROFILE_INC(field) do { if constexpr (SokobanConfig::ENABLE_PROFILE) { ++profile.field; } } while (0)
#define SOKOBAN_PROFILE_ADD(field, value) do { if constexpr (SokobanConfig::ENABLE_PROFILE) { profile.field += (value); } } while (0)
#define SOKOBAN_PROFILE_MAX(field, value) do { if constexpr (SokobanConfig::ENABLE_PROFILE) { if ((value) > profile.field) profile.field = (value); } } while (0)


// ============================================================================
// 模块 1：对外接口与求解器状态同步
// ============================================================================


/// \brief 统一语义求解入口
/// \return 找到可行推箱路径时返回 true
bool Sokoban::solve() {
    return solve_internal();
}

bool Sokoban::solve_macro_candidate() {
    if (initial_state.num_boxes > initial_targets.size()) return false;

    initial_state.hash = compute_hash(initial_state);
    std::memset(TT, 0, sizeof(TT));
    std::memset(macro_cost_cache, 0, sizeof(macro_cost_cache));
    current_threshold_iteration = 0;
    if constexpr (SokobanConfig::ENABLE_PROFILE) {
        profile = SokobanProfile{};
    }

    if (try_small_box_macro_solution()) return true;

    StaticArray<point, MAX_PATH_LENGTH> macro_candidate;
    if (try_bomb_then_small_box_macro_solution(macro_candidate)) {
        final_path = macro_candidate;
        try_strict_cost_repair(final_path);
        if constexpr (SokobanConfig::ENABLE_PATH_POSTOPT) {
            optimize_final_path_turns();
        }
        return true;
    }

    return false;
}

/// \brief 从视觉或 PC 测试输入导入初始地图和炸弹任务
/// \param level 输入地图快照
/// \param tasks 炸弹任务数组，可为空
/// \param count 有效任务数量
/// \return 加载成功时返回 true
///
/// \details
/// 该函数只缓存静态地图、玩家起点、箱子、目标点、炸弹和清墙任务
/// bind_semantics 完成语义裁剪后，再统一使用这些缓存做一次完整预计算
bool Sokoban::load_from_vision(const SokobanLevel& level, const BombTask* tasks, int count) {
    player_start = level.player_start;
    initial_state.player = level.player_start;
    map = level.map;
    
    initial_state.num_boxes = level.box_count;
    initial_state.target_mask = static_cast<uint16_t>((1U << level.target_count) - 1U);
    for (int i = 0; i < level.box_count; ++i) {
        initial_state.box_x[i] = level.boxes[i].x;
        initial_state.box_y[i] = level.boxes[i].y;
        initial_state.box_semantics[i] = level.box_semantics[i];
    }

    initial_targets.clear();
    for (int i = 0; i < level.target_count; ++i) {
        initial_targets.push_back(level.targets[i]);
        target_semantics[i] = level.target_semantics[i];
    }
    std::memset(target_semantic_mask, 0, sizeof(target_semantic_mask));
    std::memset(target_cell_mask, 0, sizeof(target_cell_mask));
    for (int i = 0; i < level.target_count; ++i) {
        uint8_t sem = level.target_semantics[i];
        if (sem < 10) {
            target_semantic_mask[sem] = static_cast<uint16_t>(target_semantic_mask[sem] | (1U << i));
            target_cell_mask[sem][level.targets[i].y][level.targets[i].x] =
                static_cast<uint16_t>(target_cell_mask[sem][level.targets[i].y][level.targets[i].x] | (1U << i));
        }
    }
    
    initial_bombs.clear();
    for (int i = 0; i < level.bomb_count; ++i) {
        if (level.bombs[i].x != -1) initial_bombs.push_back(level.bombs[i]);
    }

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
                point tw = tasks[t].target_wall;
                if (!PlanningCommon::is_blastable_wall(level, tw)) continue;

                bomb_tasks[b] = tasks[t];
                found = true;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        point p = {
                            static_cast<int8_t>(tw.x + dx),
                            static_cast<int8_t>(tw.y + dy)
                        };
                        if (PlanningCommon::is_inner_map_cell(p)) {
                            wall_clear_mask[p.y][p.x] |= (1 << b);
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

    return true;
}

/// \brief 校验并应用当前关卡中的语义信息
/// \return 语义数量合法时返回 true
///
/// \details
/// load_from_vision 已经把 SokobanLevel::box_semantics、target_semantics
/// 和炸弹任务写入求解器内部状态。本函数先剔除初始时已落位的同语义箱子，
/// 再基于裁剪后的初始状态统一重建全部距离场、死锁和哈希
bool Sokoban::bind_semantics() {
    const int original_box_count = initial_state.num_boxes;
    const int target_count = initial_targets.size();
    int box_sem_count[10] = {};
    int target_sem_count[10] = {};

    for (int i = 0; i < original_box_count; ++i) {
        int sem = initial_state.box_semantics[i];
        if (sem > 9) return false;
        box_sem_count[sem]++;
    }
    for (int t = 0; t < target_count; ++t) {
        int sem = target_semantics[t];
        if (sem > 9) return false;
        target_sem_count[sem]++;
    }
    for (int sem = 0; sem < 10; ++sem) {
        if (box_sem_count[sem] != target_sem_count[sem]) return false;
    }

    int active_count = 0;
    uint16_t remaining_target_mask = initial_state.target_mask;

    for (int i = 0; i < original_box_count; ++i) {
        uint8_t semantic_id = initial_state.box_semantics[i];
        point box_pos{initial_state.box_x[i], initial_state.box_y[i]};
        int finished_target = -1;
        for (int t = 0; t < target_count; ++t) {
            if ((remaining_target_mask & (1U << t)) == 0) continue;
            if (target_semantics[t] == semantic_id && box_pos == initial_targets[t]) {
                finished_target = t;
                break;
            }
        }

        if (finished_target != -1) {
            remaining_target_mask = static_cast<uint16_t>(remaining_target_mask & ~(1U << finished_target));
            continue;
        }

        initial_state.box_x[active_count] = initial_state.box_x[i];
        initial_state.box_y[active_count] = initial_state.box_y[i];
        initial_state.box_semantics[active_count] = semantic_id;
        ++active_count;
    }

    initial_state.num_boxes = active_count;
    initial_state.target_mask = remaining_target_mask;

    init_zobrist();
    precompute_solid_masks();
    precompute_target_distances();
    precompute_walk_distances();
    precompute_box_target_costs();
    precompute_bomb_distances();
    precompute_bomb_macro_costs();
    precompute_deadlocks();

    // 语义裁剪后，初始哈希必须按统一语义规则重新计算
    initial_state.hash = compute_hash(initial_state);
    return true;
}

bool Sokoban::profile_enabled() const {
    return SokobanConfig::ENABLE_PROFILE;
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
bool Sokoban::solve_internal() {
    if (initial_state.num_boxes > initial_targets.size()) return false;
    
    initial_state.hash = compute_hash(initial_state);
    std::memset(TT, 0, sizeof(TT)); // 清空置换表。
    std::memset(macro_cost_cache, 0, sizeof(macro_cost_cache));
    current_threshold_iteration = 0;
    if constexpr (SokobanConfig::ENABLE_PROFILE) {
        profile = SokobanProfile{};
    }

    if (try_small_box_macro_solution()) {
        return true;
    }

    StaticArray<point, MAX_PATH_LENGTH> macro_candidate;
    if (try_bomb_then_small_box_macro_solution(macro_candidate)) {
        // macro_candidate 是“炸弹全序枚举 + 少箱最优顺序”的最短候选，代价已被穷举最小化。
        // 加权 IDA* 只是快速首解（阈值被放大），可能反而更差；两者各自收尾后取更优，
        // 防止用更差的首解覆盖已最优的推箱顺序而引入无意义的中途推箱。
        StaticArray<point, MAX_PATH_LENGTH> ida_path;
        bool have_ida = run_ida_search(false, MAX_PATH_LENGTH, ida_path, SMALL_BOX_MACRO_IDA_NODE_BUDGET);
        select_cheaper_finalized(macro_candidate, have_ida ? &ida_path : nullptr);
        return true;
    }

    StaticArray<point, MAX_PATH_LENGTH> candidate_path;
    if (!run_ida_search(false, MAX_PATH_LENGTH, candidate_path)) {
        return false;
    }

    final_path = candidate_path;
    try_strict_cost_repair(candidate_path);
    if constexpr (SokobanConfig::ENABLE_PATH_POSTOPT) {
        optimize_final_path_turns();
    }
    return true;
}

bool Sokoban::run_ida_search(
    bool strict_cost,
    int max_threshold,
    StaticArray<point, MAX_PATH_LENGTH>& out_path,
    uint32_t node_budget) {
    std::memset(TT, 0, sizeof(TT));
    std::memset(macro_cost_cache, 0, sizeof(macro_cost_cache));
    current_threshold_iteration = 0;
    strict_cost_search = strict_cost;
    force_box_target_cost_lb = strict_cost;
    search_node_budget = node_budget;
    search_node_count = 0;
    search_aborted = false;

    int threshold = get_heuristic(initial_state);
    StaticArray<point, MAX_PATH_LENGTH> rev_path;             

    // IDA* 迭代加深核心逻辑
    int root_active_bombs = 0;
    for (int b = 0; b < initial_state.num_bombs; ++b) {
        if (!(initial_state.blown_mask & (1 << b)) && bomb_tasks[b].target_wall.x != -1) {
            root_active_bombs++;
        }
    }
    int root_active_entities = initial_state.num_boxes + root_active_bombs;
    if (!strict_cost_search && root_active_entities <= 5) {
        bool saved_force_lb = force_box_target_cost_lb;
        force_box_target_cost_lb = true;
        int strong_threshold = get_heuristic(initial_state);
        force_box_target_cost_lb = saved_force_lb;
        if (strong_threshold > threshold && strong_threshold < 9999) {
            force_box_target_cost_lb = true;
            threshold = strong_threshold;
        }
    }
    int effective_max_threshold = max_threshold;
    if (!strict_cost_search) {
        int root_W_num = heuristic_weight_num(root_active_entities);
        threshold = (threshold * root_W_num + SokobanConfig::HEURISTIC_WEIGHT_DEN - 1) /
                    SokobanConfig::HEURISTIC_WEIGHT_DEN;
        // max_threshold 是真实路径上限，加权 f 需要同步放大否则会把可行解误判无解
        effective_max_threshold =
            (max_threshold * root_W_num + SokobanConfig::HEURISTIC_WEIGHT_DEN - 1) /
            SokobanConfig::HEURISTIC_WEIGHT_DEN;
    }
    if constexpr (SokobanConfig::INITIAL_THRESHOLD_BOOST > 0) {
        if (!strict_cost_search) threshold += SokobanConfig::INITIAL_THRESHOLD_BOOST;
    }
    while (threshold <= effective_max_threshold) {
        ++current_threshold_iteration;
        SOKOBAN_PROFILE_INC(threshold_iterations);
        if constexpr (SokobanConfig::ENABLE_PROFILE) {
            profile.final_threshold = static_cast<uint16_t>(threshold);
        }
        int res = ida_star_search(initial_state, 0, 0, threshold, rev_path, -1);
        
        if (res == -1) {                                      
            rev_path.push_back(initial_state.player);         
            std::reverse(rev_path.begin(), rev_path.end());   
            out_path = rev_path;
            strict_cost_search = false;
            force_box_target_cost_lb = false;
            search_node_budget = 0;
            return true;
        }
        if (search_aborted) break;
        if (res >= 9999) break;                               
        int next_threshold = res;
        if constexpr (SokobanConfig::ENABLE_LATE_THRESHOLD_ACCEL) {
            if (!strict_cost_search &&
                current_threshold_iteration >= SokobanConfig::LATE_THRESHOLD_ACCEL_MIN_ITERATION &&
                next_threshold < threshold + SokobanConfig::LATE_THRESHOLD_ACCEL_STEP) {
                next_threshold = threshold + SokobanConfig::LATE_THRESHOLD_ACCEL_STEP;
            }
        }
        threshold = next_threshold;
    }

    strict_cost_search = false;
    force_box_target_cost_lb = false;
    search_node_budget = 0;
    return false;
}

bool Sokoban::run_bounded_strict_search(
    int max_cost,
    StaticArray<point, MAX_PATH_LENGTH>& out_path,
    uint32_t node_budget) {
    if (max_cost <= 0) return false;

    std::memset(TT, 0, sizeof(TT));
    std::memset(macro_cost_cache, 0, sizeof(macro_cost_cache));
    current_threshold_iteration = 0;
    strict_cost_search = true;
    force_box_target_cost_lb = true;
    search_node_budget = node_budget;
    search_node_count = 0;
    search_aborted = false;

    StaticArray<point, MAX_PATH_LENGTH> rev_path;
    int threshold = max_cost;

    // strict 修复阶段已有首解上界，直接从上界向下找改进解
    // 这比从 h 下界逐轮证明不可行更适合 MCU 上的小预算搜索
    while (threshold > 0) {
        ++current_threshold_iteration;
        SOKOBAN_PROFILE_INC(threshold_iterations);
        if constexpr (SokobanConfig::ENABLE_PROFILE) {
            profile.final_threshold = static_cast<uint16_t>(threshold);
        }

        rev_path.clear();
        int res = ida_star_search(initial_state, 0, 0, threshold, rev_path, -1);
        if (res == -1) {
            rev_path.push_back(initial_state.player);
            std::reverse(rev_path.begin(), rev_path.end());
            int improved_cost = path_step_count(rev_path);
            if (improved_cost <= 0 || improved_cost > threshold) break;
            out_path = rev_path;
            threshold = improved_cost - 1;

            std::memset(TT, 0, sizeof(TT));
            std::memset(macro_cost_cache, 0, sizeof(macro_cost_cache));
            continue;
        }

        if (search_aborted) break;
        break;
    }

    strict_cost_search = false;
    force_box_target_cost_lb = false;
    search_node_budget = 0;
    return out_path.size() > 0;
}

bool Sokoban::try_strict_cost_repair(const StaticArray<point, MAX_PATH_LENGTH>& candidate_path) {
    if constexpr (!ENABLE_STRICT_COST_REPAIR) {
        (void)candidate_path;
        return false;
    }
    int candidate_cost = path_step_count(candidate_path);
    if (candidate_cost <= 1) return false;
    if (!should_run_strict_repair(candidate_path)) return false;

    bool saved_strict = strict_cost_search;
    bool saved_force_lb = force_box_target_cost_lb;
    strict_cost_search = true;
    force_box_target_cost_lb = true;
    int lower_bound = get_heuristic(initial_state);
    strict_cost_search = saved_strict;
    force_box_target_cost_lb = saved_force_lb;
    if (lower_bound >= 9999 || lower_bound >= candidate_cost) return false;

    int strict_max = candidate_cost - 1;

    StaticArray<point, MAX_PATH_LENGTH> repaired_path;
    if (!run_bounded_strict_search(strict_max, repaired_path, STRICT_REPAIR_NODE_BUDGET)) {
        return false;
    }
    if (repaired_path.size() > 0 && path_step_count(repaired_path) < candidate_cost) {
        final_path = repaired_path;
        return true;
    }
    return false;
}

// 把一条候选路径做完整收尾：严格步数修复 + 行走转弯后处理，写入 final_path 并返回其面板总代价。
// 用于在多条已完成候选之间按“收尾后的真实代价”择优，避免用后处理前的粗略代价误判。
int Sokoban::finalize_path_candidate(const StaticArray<point, MAX_PATH_LENGTH>& candidate) {
    final_path = candidate;
    try_strict_cost_repair(candidate);
    if constexpr (SokobanConfig::ENABLE_PATH_POSTOPT) {
        optimize_final_path_turns();
    }
    return path_display_cost(final_path);
}

// 在“宏层候选”和“加权 IDA* 首解”之间择优：两者都各自收尾后比较总代价，取更优者写入 final_path。
// 由于取二者收尾代价的最小值，结果不会劣于“总是采用 IDA* 首解”的旧逻辑（后者恒等于收尾后的 IDA*）。
// ida_candidate == nullptr 表示本轮 IDA* 未出解，直接保留收尾后的宏层候选。
void Sokoban::select_cheaper_finalized(
    const StaticArray<point, MAX_PATH_LENGTH>& macro_candidate,
    const StaticArray<point, MAX_PATH_LENGTH>* ida_candidate) {
    int macro_cost = finalize_path_candidate(macro_candidate);
    if (ida_candidate == nullptr) return;

    StaticArray<point, MAX_PATH_LENGTH> macro_final = final_path;
    int ida_cost = finalize_path_candidate(*ida_candidate);
    // finalize_path_candidate 已把 final_path 设为收尾后的 IDA* 候选；仅当宏层严格更优时才回退。
    if (macro_cost < ida_cost) {
        final_path = macro_final;
    }
}


/// \brief 单次 IDA* 深度受限搜索
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

int Sokoban::ida_star_search(const GameState& state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path, int last_entity, uint8_t last_push_dir, int known_h, int known_active_bombs, bool tt_already_clear) {
    if (unlikely(state.num_boxes == 0)) {
        return (g <= threshold) ? -1 : g;
    }
    if (unlikely(depth >= MAX_PATH_LENGTH)) return 9999; // 保护 path_hashes 和深度相关工作数组的索引。
    SOKOBAN_PROFILE_MAX(max_depth, static_cast<uint16_t>(depth));

    if (!tt_already_clear) {
        int tt_probe = probe_transposition(state.hash, g, threshold);
        if (tt_probe != 0) return tt_probe;
    }

    int h = (known_h >= 0) ? known_h : get_heuristic(state);
    if (unlikely(h >= 9999)) {                
        SOKOBAN_PROFILE_INC(heuristic_dead_prunes);
        store_transposition(state.hash, g, g + 9999);
        return 9999;
    }

    int active_bombs = (known_active_bombs >= 0) ? known_active_bombs : count_active_bomb_tasks(state);
    int active_entities = state.num_boxes + active_bombs;
    int box_push_lb_sum = box_push_lb_sum_if_needed(state, active_bombs);
    int W_num = strict_cost_search ? SokobanConfig::HEURISTIC_WEIGHT_DEN
                                   : heuristic_weight_num(active_entities);
    bool use_box_target_cost_sort = false;
    if constexpr (SokobanConfig::ENABLE_BOX_TARGET_COST_LB) {
        use_box_target_cost_sort =
            strict_cost_search ||
            current_threshold_iteration >= SokobanConfig::BOX_TARGET_COST_MIN_ITERATION;
    }
    auto nearest_active_target_push_dist = [&](uint8_t semantic_id, point box_pos) {
        int best = 9999;
        uint16_t candidates = static_cast<uint16_t>(state.target_mask & target_semantic_mask[semantic_id]);
        for (uint16_t mask = candidates; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
            uint16_t bit = static_cast<uint16_t>(mask & -mask);
            int t = __builtin_ctz(static_cast<unsigned int>(bit));
            int d = t_dist[t][box_pos.y][box_pos.x];
            if (d >= 0 && d < best) best = d;
        }
        return best == 9999 ? -1 : best;
    };
    auto nearest_active_target_box_cost = [&](uint8_t semantic_id, point box_pos) {
        int best = 9999;
        uint16_t candidates = static_cast<uint16_t>(state.target_mask & target_semantic_mask[semantic_id]);
        for (uint16_t mask = candidates; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
            uint16_t bit = static_cast<uint16_t>(mask & -mask);
            int t = __builtin_ctz(static_cast<unsigned int>(bit));
            int d = static_cast<int>(box_target_cost[t][box_pos.y][box_pos.x]);
            if (d != 255 && d < best) best = d;
        }
        return best == 9999 ? -1 : best;
    };
    auto nearest_active_target_sort_cost = [&](uint8_t semantic_id, point box_pos) {
        int best = 9999;
        uint16_t candidates = static_cast<uint16_t>(state.target_mask & target_semantic_mask[semantic_id]);
        for (uint16_t mask = candidates; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
            uint16_t bit = static_cast<uint16_t>(mask & -mask);
            int t = __builtin_ctz(static_cast<unsigned int>(bit));
            int d = use_box_target_cost_sort
                        ? static_cast<int>(box_target_cost[t][box_pos.y][box_pos.x])
                        : t_dist[t][box_pos.y][box_pos.x];
            if (use_box_target_cost_sort && d == 255) d = -1;
            if (d >= 0 && d < best) best = d;
        }
        return best == 9999 ? -1 : best;
    };
    int f = g + (h * W_num) / SokobanConfig::HEURISTIC_WEIGHT_DEN;
    if (f > threshold) {
        SOKOBAN_PROFILE_INC(threshold_prunes);
        return f;
    }

    NodeOccupancy occupancy;
    occupancy.build(state);

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
                    if(occupancy.cell[np.y][np.x] == NodeOccupancy::EMPTY) {
                        bfs_visited_gen[np.y][np.x] = current_gen;
                        bfs_dist[np.y][np.x] = bfs_dist[curr.y][curr.x] + 1;
                        bfs_q[tail++] = np;
                    }
                }
            }
        }
    }

    uint32_t canon_hash = state.hash ^ ZOBRIST_PLAYER[state.player.y][state.player.x] ^ ZOBRIST_PLAYER[canon_player.y][canon_player.x];
    bool use_canonical_tt = false;
    if constexpr (SokobanConfig::ENABLE_CANONICAL_TT) {
        use_canonical_tt = !strict_cost_search &&
                           threshold >= SokobanConfig::CANONICAL_TT_MIN_THRESHOLD;
    }
    if (use_canonical_tt && canon_hash != state.hash) {
        int tt_probe = probe_transposition(canon_hash, g, threshold);
        if (tt_probe != 0) return tt_probe;
    }
    uint32_t cycle_hash = strict_cost_search ? state.hash : canon_hash;
    for (int i = 0; i < depth; ++i) {
        if (path_hashes[i] == cycle_hash) {
            SOKOBAN_PROFILE_INC(path_cycle_prunes);
            return 9999; // 剪掉当前路径上的环路。
        }
    }
    path_hashes[depth] = cycle_hash;
    SOKOBAN_PROFILE_INC(expanded_nodes);
    ++search_node_count;
    if (search_node_budget != 0 && search_node_count > search_node_budget) {
        search_aborted = true;
        return 9999;
    }

    // 每层约几百字节，32KB 栈足够承受当前 profile 中 40~50 层的典型深度。
    // 保持局部数组还能避免长期占用宝贵的 DTCM 全局空间。
    TinyMove moves[MAX_NODE_MOVES];
    int num_moves = generate_moves(state, occupancy, active_bombs, moves);
    MacroMove macro_moves[MAX_NODE_MACROS];
    int num_macros = strict_cost_search
                         ? 0
                         : generate_bomb_macros(state, occupancy, h, active_entities, g, threshold, macro_moves);
    SOKOBAN_PROFILE_ADD(generated_moves, static_cast<uint32_t>(num_moves + num_macros));

    int min_next_threshold = 9999;
    auto precheck_child = [&](const GameState& child, int child_g, int& out_h, int& out_active_bombs) {
        if (child.num_boxes == 0) {
            return (child_g <= threshold) ? 0 : child_g;
        }

        int tt_probe = probe_transposition(child.hash, child_g, threshold);
        if (tt_probe != 0) return tt_probe;

        out_h = get_heuristic(child);
        if (unlikely(out_h >= 9999)) {
            SOKOBAN_PROFILE_INC(heuristic_dead_prunes);
            store_transposition(child.hash, child_g, child_g + 9999);
            return 9999;
        }

        out_active_bombs = count_active_bomb_tasks(child);
        int child_active_entities = child.num_boxes + out_active_bombs;
        int child_weight = strict_cost_search ? SokobanConfig::HEURISTIC_WEIGHT_DEN
                                              : heuristic_weight_num(child_active_entities);
        int child_f = child_g +
                      (out_h * child_weight) / SokobanConfig::HEURISTIC_WEIGHT_DEN;
        if (child_f > threshold) {
            SOKOBAN_PROFILE_INC(threshold_prunes);
            return child_f;
        }
        return 0;
    };

    EvalMove sorted_moves[MAX_NODE_ACTIONS];
    int action_count = 0;

    auto cell_free_after_moving_box = [&](point p, uint8_t moving_box_idx) {
        if (is_overstep(p) || is_solid(p, state.blown_mask)) return false;
        int8_t occ = occupancy.cell[p.y][p.x];
        if (occ == NodeOccupancy::EMPTY) return true;
        return occ == static_cast<int8_t>(moving_box_idx);
    };

    auto static_push_improves = [&](uint8_t sem, point to, int from_push_dist, int from_box_cost) {
        int to_push_dist = nearest_active_target_push_dist(sem, to);
        if (from_push_dist >= 0 && to_push_dist >= 0 && to_push_dist < from_push_dist) return true;
        int to_box_cost = nearest_active_target_box_cost(sem, to);
        if (from_box_cost >= 0 && to_box_cost >= 0 && to_box_cost < from_box_cost) return true;
        return false;
    };

    // 只作为动作排序信号：其他箱子和炸弹是可移动障碍，不能写进 admissible h 里硬剪
    auto dynamic_box_push_distance = [&](uint8_t moving_box_idx, uint8_t sem, point start) {
        uint16_t candidates = static_cast<uint16_t>(state.target_mask & target_semantic_mask[sem]);
        if (candidates == 0) return 9999;

        ++dynamic_box_eval_gen;
        if (dynamic_box_eval_gen == 0) {
            std::memset(DYNAMIC_BOX_EVAL_SEEN, 0, sizeof(DYNAMIC_BOX_EVAL_SEEN));
            dynamic_box_eval_gen = 1;
        }

        point q[MAP_CELL_COUNT];
        uint8_t q_dist[MAP_CELL_COUNT];
        int head = 0;
        int tail = 0;
        q[tail] = start;
        q_dist[tail++] = 0;
        DYNAMIC_BOX_EVAL_SEEN[start.y][start.x] = dynamic_box_eval_gen;

        auto box_cell_free = [&](point p) {
            if (is_overstep(p) || is_solid(p, state.blown_mask)) return false;
            int8_t occ = occupancy.cell[p.y][p.x];
            return occ == NodeOccupancy::EMPTY || occ == static_cast<int8_t>(moving_box_idx);
        };

        auto player_stand_free = [&](point p, point box_pos) {
            if (is_overstep(p) || is_solid(p, state.blown_mask)) return false;
            if (p == box_pos) return false;
            int8_t occ = occupancy.cell[p.y][p.x];
            return occ == NodeOccupancy::EMPTY || occ == static_cast<int8_t>(moving_box_idx);
        };

        while (head < tail) {
            point curr = q[head];
            uint8_t curr_dist = q_dist[head++];
            if ((target_cell_mask[sem][curr.y][curr.x] & candidates) != 0) {
                return static_cast<int>(curr_dist);
            }

            for (uint8_t d = 0; d < 4; ++d) {
                point next = curr + MOVE[d];
                point stand = curr - MOVE[d];
                if (!box_cell_free(next) || !player_stand_free(stand, curr)) continue;
                if (DYNAMIC_BOX_EVAL_SEEN[next.y][next.x] == dynamic_box_eval_gen) continue;

                DYNAMIC_BOX_EVAL_SEEN[next.y][next.x] = dynamic_box_eval_gen;
                q[tail] = next;
                q_dist[tail++] = static_cast<uint8_t>(curr_dist + 1);
            }
        }
        return 9999;
    };

    int committed_bomb_idx = -1;
    int committed_bomb_dist = 9999;
    for (uint8_t b = 0; b < state.num_bombs; ++b) {
        if (state.blown_mask & (1 << b)) continue;
        if (bomb_tasks[b].target_wall.x == -1) continue;
        point bomb_pos = {state.bomb_x[b], state.bomb_y[b]};
        if (bomb_pos == bomb_tasks[b].bomb_start) continue;
        int d = b_dist[b][bomb_pos.y][bomb_pos.x];
        if (d > 4) continue;
        if (d >= 0 && d < committed_bomb_dist) {
            committed_bomb_dist = d;
            committed_bomb_idx = b;
        }
    }

    auto turn_access_debt_penalty = [&](uint8_t box_idx,
                                        uint8_t sem,
                                        uint8_t first_dir,
                                        point first_to,
                                        int first_push_dist,
                                        int first_box_cost) {
        if constexpr (!ENABLE_TURN_ACCESS_DEBT_SORT) {
            (void)box_idx;
            (void)sem;
            (void)first_dir;
            (void)first_to;
            (void)first_push_dist;
            (void)first_box_cost;
            return 0;
        }
        if (!strict_cost_search) return 0;
        if (first_push_dist == 0) return 0;
        if (first_push_dist < 0 && first_box_cost < 0) return 0;

        int turn_probe_horizon = 4;
        int free_turn_walk = 2;
        int turn_walk_penalty = 7;
        int no_turn_penalty = 42;
        int max_turn_penalty = 84;

        point probe_box = first_to;
        point probe_player = first_to - MOVE[first_dir];
        int probe_push_dist = first_push_dist;
        int probe_box_cost = first_box_cost;

        int straight_steps = 0;
        while (straight_steps < turn_probe_horizon) {
            point straight_to = probe_box + MOVE[first_dir];
            if (!cell_free_after_moving_box(straight_to, box_idx)) break;
            int straight_push_dist = nearest_active_target_push_dist(sem, straight_to);
            int straight_box_cost = nearest_active_target_box_cost(sem, straight_to);
            bool straight_improves =
                (probe_push_dist >= 0 && straight_push_dist >= 0 && straight_push_dist < probe_push_dist) ||
                (probe_box_cost >= 0 && straight_box_cost >= 0 && straight_box_cost < probe_box_cost);
            if (!straight_improves) break;

            probe_player = probe_box;
            probe_box = straight_to;
            probe_push_dist = straight_push_dist;
            probe_box_cost = straight_box_cost;
            ++straight_steps;
            if (probe_push_dist == 0) return 0;
        }

        if (straight_steps == turn_probe_horizon) {
            point straight_to = probe_box + MOVE[first_dir];
            if (cell_free_after_moving_box(straight_to, box_idx) &&
                static_push_improves(sem, straight_to, probe_push_dist, probe_box_cost)) {
                return 0;
            }
        }

        auto dynamic_walk_after_moving_box = [&](point start, point goal, point future_box) {
            if (start == goal) return 0;
            if (is_overstep(goal) || is_solid(goal, state.blown_mask)) return 9999;

            uint8_t seen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
            int8_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
            std::memset(seen, 0, sizeof(seen));

            int head = 0;
            int tail = 0;
            bfs_q[tail++] = start;
            seen[start.y][start.x] = 1;
            dist[start.y][start.x] = 0;

            while (head < tail) {
                point curr = bfs_q[head++];
                if (curr == goal) return static_cast<int>(dist[curr.y][curr.x]);

                for (uint8_t d = 0; d < 4; ++d) {
                    point np = curr + MOVE[d];
                    if (is_overstep(np) || seen[np.y][np.x]) continue;
                    if (is_solid(np, state.blown_mask)) continue;
                    if (np == future_box) continue;

                    int8_t occ = occupancy.cell[np.y][np.x];
                    if (occ != NodeOccupancy::EMPTY && occ != static_cast<int8_t>(box_idx)) continue;

                    seen[np.y][np.x] = 1;
                    dist[np.y][np.x] = static_cast<int8_t>(dist[curr.y][curr.x] + 1);
                    bfs_q[tail++] = np;
                }
            }
            return 9999;
        };

        int best_turn_walk = 9999;
        for (uint8_t turn_dir = 0; turn_dir < 4; ++turn_dir) {
            if (turn_dir == first_dir) continue;
            point turn_to = probe_box + MOVE[turn_dir];
            point turn_stand = probe_box - MOVE[turn_dir];
            if (!cell_free_after_moving_box(turn_to, box_idx)) continue;
            if (!cell_free_after_moving_box(turn_stand, box_idx)) continue;
            if (!static_push_improves(sem, turn_to, probe_push_dist, probe_box_cost)) continue;

            int walk = dynamic_walk_after_moving_box(probe_player, turn_stand, probe_box);
            if (walk < best_turn_walk) best_turn_walk = walk;
        }

        if (best_turn_walk == 9999) return no_turn_penalty;
        if (best_turn_walk <= free_turn_walk) return 0;

        int penalty = (best_turn_walk - free_turn_walk) * turn_walk_penalty;
        return penalty > max_turn_penalty ? max_turn_penalty : penalty;
    };

    // 热路径性能优先：动作评分直接留在 IDA* 主循环所在函数内。
    // 这样编译器能同时看见 moves、state 和递归参数，减少寄存器溢出和重复取址。
    int nearest_box_move_walk = 9999;
    for (int m = 0; m < num_moves; ++m) {
        const TinyMove& mv = moves[m];
        if (mv.entity_idx < state.num_boxes && mv.walk_dist < nearest_box_move_walk) {
            nearest_box_move_walk = mv.walk_dist;
        }
    }
    // 同一节点内每个箱子的移动前动态距离相同，缓存 old 值减少重复 BFS
    int16_t dynamic_box_start_dist[MAX_BOXES];
    for (uint8_t i = 0; i < MAX_BOXES; ++i) dynamic_box_start_dist[i] = -2;
    auto cached_dynamic_box_start_distance = [&](uint8_t box_idx, uint8_t sem, point pos) {
        int16_t cached = dynamic_box_start_dist[box_idx];
        if (cached == -2) {
            int d = dynamic_box_push_distance(box_idx, sem, pos);
            cached = static_cast<int16_t>(d >= 9999 ? 9999 : d);
            dynamic_box_start_dist[box_idx] = cached;
        }
        return static_cast<int>(cached);
    };
    auto static_descent_has_dynamic_pressure = [&](uint8_t box_idx, uint8_t sem, point start) {
        int moving_obstacles = static_cast<int>(state.num_boxes) - 1;
        for (uint8_t b = 0; b < state.num_bombs; ++b) {
            if (!(state.blown_mask & (1 << b))) ++moving_obstacles;
        }
        if (moving_obstacles <= 0) return false;

        int static_edges = 0;
        int blocked_edges = 0;
        int free_edges = 0;

        int curr_push_dist = nearest_active_target_push_dist(sem, start);
        int curr_box_cost = nearest_active_target_box_cost(sem, start);
        if (curr_push_dist == 0 || curr_box_cost == 0) return false;

        for (uint8_t d = 0; d < 4; ++d) {
            point next = start + MOVE[d];
            point stand = start - MOVE[d];
            if (is_overstep(next) || is_overstep(stand)) continue;
            if (is_solid(next, state.blown_mask) || is_solid(stand, state.blown_mask)) continue;

            int next_push_dist = nearest_active_target_push_dist(sem, next);
            int next_box_cost = nearest_active_target_box_cost(sem, next);
            bool static_descent =
                (curr_push_dist >= 0 && next_push_dist >= 0 && next_push_dist < curr_push_dist) ||
                (curr_box_cost >= 0 && next_box_cost >= 0 && next_box_cost < curr_box_cost);
            if (!static_descent) continue;

            ++static_edges;
            if (!cell_free_after_moving_box(next, box_idx) ||
                !cell_free_after_moving_box(stand, box_idx)) {
                ++blocked_edges;
            } else {
                ++free_edges;
            }
        }

        if (static_edges == 0) return false;
        return blocked_edges > 0 && (free_edges == 0 || blocked_edges >= 2);
    };
    auto no_task_bomb_move_saving = [&](point bomb_pos, point push_from, point push_to) {
        int best_saving = 0;
        if (bfs_dist[push_from.y][push_from.x] <= nearest_box_move_walk + 2) best_saving = 6;
        for (uint8_t dir = 0; dir < 4; ++dir) {
            point np = bomb_pos + MOVE[dir];
            if (np == push_from || np == push_to) continue;
            if (is_overstep(np) || is_solid(np, state.blown_mask)) continue;
            if (occupancy.cell[np.y][np.x] != NodeOccupancy::EMPTY) continue;
            if (bfs_visited_gen[np.y][np.x] != current_gen) best_saving = 10;
        }
        return best_saving;
    };

    for (int m = 0; m < num_moves; ++m) {
        TinyMove& mv = moves[m];
        bool is_bomb_entity = (mv.entity_idx >= state.num_boxes);
        int b_idx = is_bomb_entity ? (mv.entity_idx - state.num_boxes) : -1;
        point pos = is_bomb_entity ? point{state.bomb_x[b_idx], state.bomb_y[b_idx]}
                                    : point{state.box_x[mv.entity_idx], state.box_y[mv.entity_idx]};
        point push_to = pos + MOVE[mv.dir];
        point push_from = pos - MOVE[mv.dir];
        point eval_push_to = push_to;
        if (mv.slide_dist > 0) {
            eval_push_to.x += MOVE[mv.dir].x * mv.slide_dist;
            eval_push_to.y += MOVE[mv.dir].y * mv.slide_dist;
        }

        int delta_h = 0;
        int old_push_dist = -1;
        int new_push_dist = -1;
        int old_box_cost = -1;
        int new_box_cost = -1;
        if (!mv.triggers_explosion) {
            if (is_bomb_entity) {
                if (bomb_tasks[b_idx].target_wall.x != -1) {
                    delta_h = b_dist[b_idx][eval_push_to.y][eval_push_to.x] - b_dist[b_idx][pos.y][pos.x];
                }
            } else {
                uint8_t sem = state.box_semantics[mv.entity_idx];
                old_push_dist = nearest_active_target_push_dist(sem, pos);
                new_push_dist = nearest_active_target_push_dist(sem, eval_push_to);
                old_box_cost = nearest_active_target_box_cost(sem, pos);
                new_box_cost = nearest_active_target_box_cost(sem, eval_push_to);
                int old_dist = nearest_active_target_sort_cost(sem, pos);
                int new_dist = nearest_active_target_sort_cost(sem, eval_push_to);
                if (old_dist != -1 && new_dist != -1) delta_h = new_dist - old_dist;
            }
        }

        int walk_weight = (active_entities <= 4) ? 12 : 8;
        int progress_weight = (active_entities <= 4) ? 32 : 42;
        int same_entity_bonus = 28;
        int same_dir_bonus = 12;
        int dir_change_penalty = 16;
        int switch_entity_penalty = 8;
        int explosion_bonus = 45;
        int slide_bonus = 4;
        int finish_sort_max_dist = 2;
        int finish_sort_bonus = 48;
        int same_box_finish_bonus = 80;
        int false_progress_penalty = 36;
        int box_cost_tie_weight = 10;
        int max_box_cost_tie_bonus = 36;
        int dynamic_block_penalty = 56;
        int dynamic_block_delta_weight = 18;
        int max_dynamic_block_penalty = 160;

        int sort_key = mv.walk_dist * walk_weight + 1 + delta_h * progress_weight;
        bool high_box_push_pressure = (state.num_boxes > 0 && box_push_lb_sum >= state.num_boxes * 10);
        bool is_taskless_bomb = is_bomb_entity && bomb_tasks[b_idx].target_wall.x == -1;
        bool last_was_taskless_bomb = false;
        if (last_entity >= state.num_boxes) {
            int last_bomb_idx = last_entity - state.num_boxes;
            if (last_bomb_idx >= 0 &&
                last_bomb_idx < state.num_bombs &&
                bomb_tasks[last_bomb_idx].target_wall.x == -1) {
                last_was_taskless_bomb = true;
            }
        }
        if (is_taskless_bomb) {
            int no_task_bomb_penalty = 240;
            int shortcut_saving = no_task_bomb_move_saving(pos, push_from, push_to);
            int shortcut_bonus = shortcut_saving * 14;
            if (shortcut_bonus > 260) shortcut_bonus = 260;
            sort_key += no_task_bomb_penalty - shortcut_bonus;
            if (last_entity == mv.entity_idx) sort_key += 90;
        }
        if (is_bomb_entity &&
            active_bombs > 0 &&
            bomb_tasks[b_idx].target_wall.x != -1) {
            int remaining = b_dist[b_idx][eval_push_to.y][eval_push_to.x];
            int bomb_access_margin = active_entities <= 4 ? 5 : 3;
            int near_blast_horizon = 2;
            int near_blast_bonus = 36;
            int explosion_ready_bonus = 80;
            int essential_bomb_bonus = 28;
            // 只有当炸弹接近爆破且接近成本不明显劣于箱子任务时，才提升它的排序
            bool access_competitive =
                nearest_box_move_walk == 9999 ||
                mv.walk_dist <= nearest_box_move_walk + bomb_access_margin;
            if (access_competitive && remaining >= 0 && remaining <= near_blast_horizon) {
                sort_key -= (near_blast_horizon + 1 - remaining) * near_blast_bonus;
                if (bomb_tasks[b_idx].is_essential) sort_key -= essential_bomb_bonus;
            }
            if (access_competitive && mv.triggers_explosion) sort_key -= explosion_ready_bonus;
        }
        if (is_bomb_entity && active_bombs > 0 && high_box_push_pressure) {
            // 箱子整体离目标很远时，先鼓励完成炸弹清墙，避免箱子在未开路区域里盲搜。
            sort_key -= 40;
            if (!mv.triggers_explosion && b_dist[b_idx][eval_push_to.y][eval_push_to.x] >= 0) {
                int remaining = b_dist[b_idx][eval_push_to.y][eval_push_to.x];
                if (remaining <= 2) sort_key -= (3 - remaining) * 40;
            }
            if (mv.triggers_explosion) sort_key -= 100;
        }
        if (!is_bomb_entity && !mv.triggers_explosion) {
            uint8_t sem = state.box_semantics[mv.entity_idx];
            int finish_dist = nearest_active_target_sort_cost(sem, eval_push_to);
            int turn_penalty = 0;
            if (!use_box_target_cost_sort &&
                old_push_dist >= 0 &&
                new_push_dist >= 0 &&
                new_push_dist <= old_push_dist &&
                old_box_cost >= 0 &&
                new_box_cost >= 0) {
                int cost_progress_bonus = (old_box_cost - new_box_cost) * box_cost_tie_weight;
                if (cost_progress_bonus > max_box_cost_tie_bonus) cost_progress_bonus = max_box_cost_tie_bonus;
                if (cost_progress_bonus < -max_box_cost_tie_bonus) cost_progress_bonus = -max_box_cost_tie_bonus;
                sort_key -= cost_progress_bonus;
            }
            if (old_push_dist >= 0 &&
                new_push_dist >= 0 &&
                new_push_dist < old_push_dist &&
                old_box_cost >= 0 &&
                new_box_cost >= old_box_cost &&
                new_push_dist > finish_sort_max_dist) {
                sort_key += false_progress_penalty;
                if (new_box_cost > old_box_cost) sort_key += (new_box_cost - old_box_cost) * 4;
            }
            bool static_distance_non_worse =
                (old_push_dist >= 0 && new_push_dist >= 0 && new_push_dist <= old_push_dist) ||
                (old_box_cost >= 0 && new_box_cost >= 0 && new_box_cost <= old_box_cost);
            bool static_metric_conflict =
                old_push_dist >= 0 &&
                new_push_dist >= 0 &&
                new_push_dist < old_push_dist &&
                old_box_cost >= 0 &&
                new_box_cost >= old_box_cost &&
                new_push_dist > finish_sort_max_dist;
            bool needs_dynamic_block_eval = false;
            if constexpr (ENABLE_DYNAMIC_BLOCK_SORT) {
                needs_dynamic_block_eval =
                    strict_cost_search &&
                    static_distance_non_worse &&
                    static_descent_has_dynamic_pressure(mv.entity_idx, sem, eval_push_to);
            }
            if (needs_dynamic_block_eval) {
                int dynamic_old = cached_dynamic_box_start_distance(mv.entity_idx, sem, pos);
                int dynamic_new = dynamic_box_push_distance(mv.entity_idx, sem, eval_push_to);
                int penalty = 0;
                if (dynamic_old < 9999 && dynamic_new >= 9999) {
                    penalty = max_dynamic_block_penalty;
                } else if (dynamic_old < 9999 && dynamic_new > dynamic_old) {
                    penalty = dynamic_block_penalty +
                              (dynamic_new - dynamic_old) * dynamic_block_delta_weight;
                } else if (dynamic_old >= 9999 && dynamic_new >= 9999 &&
                           new_push_dist >= 0 && old_push_dist >= 0 &&
                           new_push_dist >= old_push_dist) {
                    penalty = dynamic_block_penalty;
                }
                if (penalty > max_dynamic_block_penalty) penalty = max_dynamic_block_penalty;
                sort_key += penalty;
            }
            if ((old_push_dist >= 0 && new_push_dist >= 0 && new_push_dist < old_push_dist) ||
                (old_box_cost >= 0 && new_box_cost >= 0 && new_box_cost < old_box_cost)) {
                turn_penalty = turn_access_debt_penalty(
                    mv.entity_idx,
                    sem,
                    mv.dir,
                    eval_push_to,
                    new_push_dist,
                    new_box_cost);
                sort_key += turn_penalty;
            }
            if (last_entity != -1 &&
                mv.entity_idx == last_entity &&
                finish_dist >= 0 &&
                finish_dist <= finish_sort_max_dist) {
                sort_key -= (finish_sort_max_dist + 1 - finish_dist) * finish_sort_bonus;
                sort_key -= same_box_finish_bonus;
            }
        }
        if (last_entity != -1 && mv.entity_idx == last_entity) {
            if (!is_taskless_bomb) sort_key -= same_entity_bonus;
        } else if (last_entity != -1 && !last_was_taskless_bomb) {
            sort_key += switch_entity_penalty;
        }
        if (last_push_dir < 4) {
            if (mv.dir == last_push_dir) {
                if (!is_taskless_bomb) sort_key -= same_dir_bonus;
            }
            else sort_key += dir_change_penalty;
        }
        if (mv.triggers_explosion) sort_key -= explosion_bonus;
        if (!strict_cost_search) {
            sort_key += mv.slide_dist * slide_bonus;
        }

        if (committed_bomb_idx >= 0) {
            int commit_bomb_bonus = 180;
            int commit_other_box_penalty = 170;
            int commit_other_bomb_penalty = 70;
            if (is_bomb_entity) {
                if (b_idx == committed_bomb_idx) sort_key -= commit_bomb_bonus;
                else sort_key += commit_other_bomb_penalty;
            } else {
                sort_key += commit_other_box_penalty;
            }
        }

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
            int b = macro.bomb_idx;
            point bomb_pos = {state.bomb_x[b], state.bomb_y[b]};

            uint16_t macro_path_cost = 0;
            point final_player;
            uint8_t final_dir = 4;
            GameState macro_next;

            // exact 宏路径规划比普通推一步贵得多；同一状态下相同炸弹的成功/失败结果
            // 可以复用，避免在排序后多次进入同一个 expensive 检查。
            uint32_t cache_mix = state.hash ^ (static_cast<uint32_t>(b) * 0x9E3779B9u);
            MacroCostCacheEntry& cache = macro_cost_cache[cache_mix & (MACRO_COST_CACHE_SIZE - 1)];
            bool use_macro_cache = bomb_tasks[b].box_pushes.empty();
            if (use_macro_cache && (cache.flags & 1) && cache.hash == state.hash && cache.bomb_idx == b) {
                if ((cache.flags & 2) == 0) continue;
                macro_path_cost = cache.cost;
                final_player = {cache.final_x, cache.final_y};
                final_dir = cache.final_dir;
                macro_next = state;
                macro_next.player = final_player;
                macro_next.blown_mask = static_cast<uint8_t>(state.blown_mask | (1 << b));

                uint32_t new_hash = state.hash;
                new_hash ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
                new_hash ^= ZOBRIST_PLAYER[final_player.y][final_player.x];
                new_hash ^= ZOBRIST_BLOWN_MASK[state.blown_mask];
                new_hash ^= ZOBRIST_BLOWN_MASK[macro_next.blown_mask];
                new_hash ^= ZOBRIST_BOMB[b][state.bomb_y[b]][state.bomb_x[b]];
                macro_next.hash = new_hash;
            } else {
                SokobanLevel macro_level;
                build_level_from_state(state, macro_level);

                BombTask task = bomb_tasks[b];
                task.bomb_start = bomb_pos;

                bool ok = false;
                if (task.box_pushes.empty()) {
                    ok = PlanningCommon::get_direct_bomb_push_path_cost(
                        macro_level,
                        state.player,
                        task,
                        macro_path_cost,
                        final_player);
                } else {
                    StaticArray<point, MAX_PATH_LENGTH> macro_path;
                    ok = PlanningCommon::get_bomb_push_path(macro_level, state.player, task, macro_path);
                    if (ok) {
                        macro_path_cost = static_cast<uint16_t>(
                            macro_path.size() > UINT16_MAX ? UINT16_MAX : macro_path.size());
                        final_player = macro_path.back();
                    }
                }
                if (ok && (macro_path_cost == 0 || macro_path_cost > SokobanConfig::BOMB_MACRO_MAX_PATH)) ok = false;
                if (ok) {
                    final_dir = infer_final_bomb_push_dir(final_player, task.target_wall);
                    if (final_dir >= 4) ok = false;
                }
                if (ok) {
                    ok = build_bomb_task_successor_state(state, b, task, final_player, macro_next);
                }

                // 失败结果也缓存：动态障碍不变时，这个宏动作下次仍然不会成功。
                if (use_macro_cache) {
                    cache.hash = state.hash;
                    cache.bomb_idx = b;
                    cache.flags = ok ? 3 : 1;
                    if (ok) {
                        cache.cost = macro_path_cost;
                        cache.final_x = final_player.x;
                        cache.final_y = final_player.y;
                        cache.final_dir = final_dir;
                    }
                }
                if (!ok) continue;
            }

            int child_g = g + macro_path_cost;
            int child_h = -1;
            int child_active_bombs = -1;
            int early_res = precheck_child(macro_next, child_g, child_h, child_active_bombs);
            if (early_res != 0) {
                if (early_res < min_next_threshold) min_next_threshold = early_res;
                continue;
            }

            int res = ida_star_search(
                macro_next,
                child_g,
                depth + 1,
                threshold,
                path,
                macro.entity_idx,
                final_dir,
                child_h,
                child_active_bombs,
                true);

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
            uint16_t new_target_mask = state.target_mask;

            // 箱子进入目标后从活动列表中移除。这样后续节点只保留未完成箱子，
            // 降低状态体积、启发式维度和动作生成成本。
            for (int i = 0; i < state.num_boxes; ++i) {
                point old_p = {state.box_x[i], state.box_y[i]};
                point p = (i == mv.entity_idx) ? push_to : old_p;

                uint8_t sem = state.box_semantics[i];
                new_hash ^= ZOBRIST_SPECIFIC_BOX[sem][old_p.y][old_p.x];

                int t_idx = find_active_target_index(state, new_target_mask, p, i);
                if (t_idx != -1) {
                    new_target_mask = static_cast<uint16_t>(new_target_mask & ~(1U << t_idx));
                    new_hash ^= ZOBRIST_TARGET[t_idx];
                } else {
                    new_bx[new_box_count] = p.x;
                    new_by[new_box_count] = p.y;
                    new_ids[new_box_count] = sem;
                    new_hash ^= ZOBRIST_SPECIFIC_BOX[sem][p.y][p.x];
                    new_box_count++;
                }
            }

            next_state.num_boxes = new_box_count;
            next_state.target_mask = new_target_mask;
            for (int i = 0; i < new_box_count; ++i) {
                next_state.box_x[i] = new_bx[i];
                next_state.box_y[i] = new_by[i];
                next_state.box_semantics[i] = new_ids[i];
            }
        }

        next_state.hash = new_hash;
        int step_cost = mv.walk_dist + 1;
        if (strict_cost_search) step_cost += mv.slide_dist;
        int child_g = g + step_cost;
        int child_h = -1;
        int child_active_bombs = -1;
        int early_res = precheck_child(next_state, child_g, child_h, child_active_bombs);
        if (early_res != 0) {
            if (early_res < min_next_threshold) min_next_threshold = early_res;
            continue;
        }

        // 递归进入下一层 IDA* 搜索
        int res = ida_star_search(
            next_state,
            child_g,
            depth + 1,
            threshold,
            path,
            mv.entity_idx,
            mv.dir,
            child_h,
            child_active_bombs,
            true);
        
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
                            if (occupancy.cell[np.y][np.x] == NodeOccupancy::EMPTY) {
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

    store_transposition(use_canonical_tt ? canon_hash : state.hash, g, min_next_threshold);

    return min_next_threshold;
}


inline int Sokoban::count_active_bomb_tasks(const GameState& state) const {
    int active_bombs = 0;
    for (int b = 0; b < state.num_bombs; ++b) {
        if (!(state.blown_mask & (1 << b)) && bomb_tasks[b].target_wall.x != -1) ++active_bombs;
    }
    return active_bombs;
}

// 只有仍有炸弹任务时，动作排序才需要估计箱子的剩余推动压力。
// 纯推箱阶段直接返回 0，减少每个节点不必要的 t_dist 访存。
inline int Sokoban::box_push_lb_sum_if_needed(const GameState& state, int active_bombs) const {
    if (active_bombs == 0) return 0;

    // 只在“箱子压力很大时优先推炸弹”的排序分支里使用。
    // 无活动炸弹时跳过，可减少纯推箱阶段每个节点的无效 t_dist 访存。
    int sum = 0;
    for (int i = 0; i < state.num_boxes; ++i) {
        int d = nearest_active_target_distance(
            state,
            state.box_semantics[i],
            {state.box_x[i], state.box_y[i]});
        if (d > 0 && d < 9999) sum += d;
    }
    return sum;
}

// 启发式权重用于 IDA* 的 f=g+w*h。实体多时提高权重，加快收敛；
// 实体少时权重更保守，避免过度贪心导致走进长路径。
inline int Sokoban::heuristic_weight_num(int active_entities) const {
    if (active_entities >= 8) return SokobanConfig::HEURISTIC_WEIGHT_GE_8;
    if (active_entities >= 6) return SokobanConfig::HEURISTIC_WEIGHT_GE_6;
    if (active_entities >= 5) return SokobanConfig::HEURISTIC_WEIGHT_GE_5;
    if (active_entities >= 4) return SokobanConfig::HEURISTIC_WEIGHT_GE_4;
    return SokobanConfig::HEURISTIC_WEIGHT_BASE;
}

// 置换表保存“该状态在某个剩余阈值下已经失败”的信息。
// 命中时可以直接返回下一轮建议阈值，跳过重复子树。
inline int Sokoban::probe_transposition(uint32_t hash, int g, int threshold) {
    int remaining_threshold = threshold - g;
    int tt_idx1 = hash & (TT_SIZE - 1);
    int tt_idx2 = (hash ^ (hash >> 16) ^ 0x5BD1E995) & (TT_SIZE - 1);
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
    int tt_idx2 = (hash ^ (hash >> 16) ^ 0x5BD1E995) & (TT_SIZE - 1);
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

// 查找 p 命中的同语义有效目标编号。
inline int Sokoban::find_active_target_index(const GameState& state, uint16_t target_mask, point p, int box_idx) const {
    uint8_t semantic_id = state.box_semantics[box_idx];
    uint16_t candidates = static_cast<uint16_t>(target_mask & target_cell_mask[semantic_id][p.y][p.x]);
    if (candidates != 0) return __builtin_ctz(static_cast<unsigned int>(candidates));
    return -1;
}

inline bool Sokoban::is_active_target_cell(const GameState& state, point p, int box_idx, int& out_idx) const {
    out_idx = find_active_target_index(state, state.target_mask, p, box_idx);
    return out_idx != -1;
}

// 动态隧道检测：若推动方向两侧都是墙，则箱子/炸弹可沿隧道自动滑行。
// blown_mask 会让已炸开的墙不再被视作实体墙。
inline bool Sokoban::is_tunnel_dynamic(point p, int dir, uint8_t blown_mask) const {
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

    auto no_task_bomb_push_relevant = [&](point bomb_pos, point push_from, point push_to) {
        int shortcut_margin = 8;
        int static_link_limit = 8;

        for (uint8_t i = 0; i < state.num_boxes; ++i) {
            point box_pos = {state.box_x[i], state.box_y[i]};
            for (uint8_t dir = 0; dir < 4; ++dir) {
                point stand = box_pos - MOVE[dir];
                point box_to = box_pos + MOVE[dir];
                if (is_overstep(stand) || is_solid(stand, state.blown_mask)) continue;
                if (is_overstep(box_to) || is_solid(box_to, state.blown_mask)) continue;
                if (nearest_active_target_distance(state, state.box_semantics[i], box_to) == -1) continue;

                int8_t stand_occ = occupancy.cell[stand.y][stand.x];
                if (stand_occ != NodeOccupancy::EMPTY && stand != bomb_pos) continue;
                if (stand == push_to) continue;
                int8_t to_occ = occupancy.cell[box_to.y][box_to.x];
                if (to_occ != NodeOccupancy::EMPTY && box_to != bomb_pos) continue;
                if (box_to == push_to) continue;

                int current_walk = 9999;
                if (bfs_visited_gen[stand.y][stand.x] == current_gen) {
                    current_walk = bfs_dist[stand.y][stand.x];
                }

                int opened_walk = walk_dist_between(bomb_pos, stand);
                if (opened_walk == 255 || opened_walk > static_link_limit) continue;
                int after_push_walk = bfs_dist[push_from.y][push_from.x] + 1 + opened_walk;

                // 无任务炸弹只作为可移动障碍参与搜索：生成条件必须连接到真实推箱前沿
                if (box_to == bomb_pos && current_walk != 9999) return true;
                if (active_bombs > 0 && current_walk != 9999) continue;
                if (current_walk == 9999 || after_push_walk + shortcut_margin < current_walk) return true;
            }
        }
        return false;
    };

    // 箱子是绝大多数节点的主路径。单独循环可去掉热路径里的 is_bomb_entity 分支。
    for (uint8_t i = 0; i < state.num_boxes; ++i) {
        point pos = {state.box_x[i], state.box_y[i]};

        for (uint8_t dir = 0; dir < 4; ++dir) {
            point push_from = pos - MOVE[dir];
            if (is_overstep(push_from) || bfs_visited_gen[push_from.y][push_from.x] != current_gen) continue;

            point push_to = pos + MOVE[dir];
            if (is_overstep(push_to) || is_solid(push_to, state.blown_mask)) continue;
            if (occupancy.cell[push_to.y][push_to.x] != NodeOccupancy::EMPTY) continue;

            if (nearest_active_target_distance(state, state.box_semantics[i], push_to) == -1) {
                SOKOBAN_PROFILE_INC(static_deadlock_prunes);
                continue;
            }

            // 2x2 局部死锁判定：若推入后形成不可解团块，直接剪枝。
            int dummy_t;
            if (!is_active_target_cell(state, push_to, i, dummy_t)) {
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
                    int8_t occ = occupancy.cell[cp.y][cp.x];
                    if (occ >= NodeOccupancy::BOMB_BASE) {
                        return 0;
                    }
                    int box_id = -1;
                    if (cp == push_to) box_id = i;
                    else if (cp == pos) return 0;
                    else if (occ >= 0 && occ < NodeOccupancy::BOMB_BASE) box_id = occ;

                    if (box_id != -1) {
                        int out_idx;
                        if (is_active_target_cell(state, cp, box_id, out_idx)) return 3;
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
            while (is_tunnel_dynamic(final_push_to, dir, state.blown_mask)) {
                if (is_active_target_cell(state, final_push_to, i, dummy_t)) break;

                point next_p = final_push_to + MOVE[dir];
                if (is_overstep(next_p)) break;
                if (is_solid(next_p, state.blown_mask) || occupancy.cell[next_p.y][next_p.x] != NodeOccupancy::EMPTY) break;

                final_push_to = next_p;
                slide_dist++;
            }

            add_move(i, dir, static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x]), static_cast<uint8_t>(slide_dist), false);
        }
    }

    // 炸弹即使没有炸墙任务也仍是可推动实体；只有引爆和 b_dist 剪枝依赖 target_wall
    for (uint8_t b_idx = 0; b_idx < state.num_bombs; ++b_idx) {
        if (state.blown_mask & (1 << b_idx)) continue;
        bool has_bomb_task = bomb_tasks[b_idx].target_wall.x != -1;

        uint8_t entity_idx = static_cast<uint8_t>(state.num_boxes + b_idx);
        point pos = {state.bomb_x[b_idx], state.bomb_y[b_idx]};

        for (uint8_t dir = 0; dir < 4; ++dir) {
            point push_from = pos - MOVE[dir];
            if (is_overstep(push_from) || bfs_visited_gen[push_from.y][push_from.x] != current_gen) continue;

            point push_to = pos + MOVE[dir];
            if (is_overstep(push_to)) continue;

            bool triggers_explosion = false;
            if (is_solid(push_to, state.blown_mask)) {
                if (has_bomb_task && push_to == bomb_tasks[b_idx].target_wall) triggers_explosion = true;
                else continue;
            }
            if (!triggers_explosion && occupancy.cell[push_to.y][push_to.x] != NodeOccupancy::EMPTY) continue;

            if (has_bomb_task && !triggers_explosion && b_dist[b_idx][push_to.y][push_to.x] == -1) {
                SOKOBAN_PROFILE_INC(static_deadlock_prunes);
                continue;
            }
            if (!has_bomb_task && !no_task_bomb_push_relevant(pos, push_from, push_to)) continue;

            int slide_dist = 0;
            if (!triggers_explosion) {
                point final_push_to = push_to;
                while (is_tunnel_dynamic(final_push_to, dir, state.blown_mask)) {
                    point next_p = final_push_to + MOVE[dir];
                    if (is_overstep(next_p)) break;
                    if (has_bomb_task && next_p == bomb_tasks[b_idx].target_wall) break;
                    if (is_solid(next_p, state.blown_mask) || occupancy.cell[next_p.y][next_p.x] != NodeOccupancy::EMPTY) break;

                    final_push_to = next_p;
                    slide_dist++;
                }
            }

            add_move(entity_idx, dir, static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x]), static_cast<uint8_t>(slide_dist), triggers_explosion);
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
        out_level.box_semantics[i] = state.box_semantics[i];
    }

    out_level.target_count = initial_targets.size();
    for (int i = 0; i < initial_targets.size(); ++i) {
        out_level.targets[i] = initial_targets[i];
        out_level.target_semantics[i] = target_semantics[i];
    }

    out_level.bomb_count = state.num_bombs;
    for (int b = 0; b < state.num_bombs; ++b) {
        if (state.blown_mask & (1 << b)) out_level.bombs[b] = {-1, -1};
        else out_level.bombs[b] = {state.bomb_x[b], state.bomb_y[b]};
    }
}

bool Sokoban::remove_macro_completed_box_at_index(SokobanLevel& level, int box_idx, point expected_target) const {
    if (box_idx < 0 || box_idx >= level.box_count) return false;
    if (level.boxes[box_idx] != expected_target) return false;
    for (int j = box_idx + 1; j < level.box_count; ++j) {
        level.boxes[j - 1] = level.boxes[j];
        level.box_semantics[j - 1] = level.box_semantics[j];
    }
    --level.box_count;
    return true;
}

bool Sokoban::validate_macro_solution_path(const StaticArray<point, MAX_PATH_LENGTH>& path) const {
    if (path.empty() || path[0] != initial_state.player) return false;

    point player = initial_state.player;
    int8_t box_x[MAX_BOXES];
    int8_t box_y[MAX_BOXES];
    uint8_t box_sem[MAX_BOXES];
    bool active[MAX_BOXES] = {};
    uint16_t remaining_targets = initial_state.target_mask;

    for (int i = 0; i < initial_state.num_boxes; ++i) {
        box_x[i] = initial_state.box_x[i];
        box_y[i] = initial_state.box_y[i];
        box_sem[i] = initial_state.box_semantics[i];
        active[i] = true;
    }

    auto active_box_at = [&](point p) {
        for (int i = 0; i < initial_state.num_boxes; ++i) {
            if (active[i] && box_x[i] == p.x && box_y[i] == p.y) return i;
        }
        return -1;
    };
    auto live_bomb_at = [&](point p) {
        for (int b = 0; b < initial_state.num_bombs; ++b) {
            if ((initial_state.blown_mask & (1 << b)) == 0 &&
                initial_state.bomb_x[b] == p.x &&
                initial_state.bomb_y[b] == p.y) {
                return true;
            }
        }
        return false;
    };

    for (int i = 1; i < path.size(); ++i) {
        point to = path[i];
        uint8_t dir = 4;
        for (uint8_t d = 0; d < 4; ++d) {
            if (player + MOVE[d] == to) {
                dir = d;
                break;
            }
        }
        if (dir >= 4) return false;
        if (is_overstep(to) || is_solid(to, initial_state.blown_mask)) return false;
        if (live_bomb_at(to)) return false;

        int box_idx = active_box_at(to);
        if (box_idx == -1) {
            player = to;
            continue;
        }

        point next_box = to + MOVE[dir];
        if (is_overstep(next_box) || is_solid(next_box, initial_state.blown_mask)) return false;
        if (live_bomb_at(next_box)) return false;
        if (active_box_at(next_box) != -1) return false;

        box_x[box_idx] = next_box.x;
        box_y[box_idx] = next_box.y;
        player = to;

        uint8_t sem = box_sem[box_idx];
        uint16_t candidates = static_cast<uint16_t>(remaining_targets & target_semantic_mask[sem]);
        for (uint16_t mask = candidates; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
            uint16_t bit = static_cast<uint16_t>(mask & -mask);
            int target_idx = __builtin_ctz(static_cast<unsigned int>(bit));
            if (initial_targets[target_idx] != next_box) continue;
            active[box_idx] = false;
            remaining_targets = static_cast<uint16_t>(remaining_targets & ~bit);
            break;
        }
    }

    if (remaining_targets != 0) return false;
    for (int i = 0; i < initial_state.num_boxes; ++i) {
        if (active[i]) return false;
    }
    return true;
}

bool Sokoban::search_small_box_macro_order(
    int depth,
    const GameState& root_state,
    int box_count,
    uint16_t remaining_target_mask,
    bool used[MAX_BOXES],
    const SokobanLevel& level,
    point player_pos,
    const StaticArray<point, MAX_PATH_LENGTH>& path,
    StaticArray<point, MAX_PATH_LENGTH>& best_path,
    int& best_len) const {
    if (depth == box_count) {
        if (path.size() < best_len) {
            best_path = path;
            best_len = path.size();
        }
        return true;
    }

    bool found = false;
    for (int i = 0; i < box_count; ++i) {
        if (used[i]) continue;

        uint8_t sem = root_state.box_semantics[i];
        point box_start = {root_state.box_x[i], root_state.box_y[i]};
        uint16_t candidates = static_cast<uint16_t>(remaining_target_mask & target_semantic_mask[sem]);
        for (uint16_t mask = candidates; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
            uint16_t bit = static_cast<uint16_t>(mask & -mask);
            uint8_t target_idx = static_cast<uint8_t>(__builtin_ctz(static_cast<unsigned int>(bit)));
            point box_target = initial_targets[target_idx];

            int y = root_state.box_y[i];
            int x = root_state.box_x[i];
            if (t_dist[target_idx][y][x] == -1) continue;
            if (box_target_cost[target_idx][y][x] == 255) continue;

            int moving_level_idx = -1;
            for (int b = 0; b < level.box_count; ++b) {
                if (level.boxes[b] == box_start && level.box_semantics[b] == sem) {
                    moving_level_idx = b;
                    break;
                }
            }
            if (moving_level_idx == -1) continue;

            SokobanLevel next_level = level;
            point next_player = player_pos;
            StaticArray<point, MAX_PATH_LENGTH> next_path = path;
            BoxPushTask task{box_start, box_target};

            if (!PlanningCommon::append_box_push_path(next_level, next_player, task, next_path)) continue;
            if (next_path.size() >= MAX_PATH_LENGTH - 1) continue;
            if (next_path.size() >= best_len) continue;

            if (!validate_box_macro_segment(
                    level,
                    player_pos,
                    moving_level_idx,
                    sem,
                    remaining_target_mask,
                    bit,
                    path,
                    next_path)) {
                continue;
            }

            if (!remove_macro_completed_box_at_index(next_level, moving_level_idx, box_target)) continue;

            used[i] = true;
            if (search_small_box_macro_order(
                    depth + 1,
                    root_state,
                    box_count,
                    static_cast<uint16_t>(remaining_target_mask & ~bit),
                    used,
                    next_level,
                    next_player,
                    next_path,
                    best_path,
                    best_len)) {
                found = true;
            }
            used[i] = false;
        }
    }
    return found;
}

bool Sokoban::validate_box_macro_segment(
    const SokobanLevel& level,
    point player_pos,
    int moving_level_idx,
    uint8_t sem,
    uint16_t remaining_target_mask,
    uint16_t target_bit,
    const StaticArray<point, MAX_PATH_LENGTH>& path,
    const StaticArray<point, MAX_PATH_LENGTH>& next_path) const {
    // 单箱完成宏不能推到其它箱子，也不能提前命中其它未完成目标
    point sim_player = player_pos;
    point sim_boxes[MAX_BOXES];
    bool sim_active[MAX_BOXES] = {};
    for (int b = 0; b < level.box_count; ++b) {
        sim_boxes[b] = level.boxes[b];
        sim_active[b] = true;
    }

    bool segment_ok = false;
    for (int p = path.size(); p < next_path.size(); ++p) {
        point to = next_path[p];
        uint8_t dir = 4;
        for (uint8_t d = 0; d < 4; ++d) {
            if (sim_player + MOVE[d] == to) {
                dir = d;
                break;
            }
        }
        if (dir >= 4) return false;

        int hit_box = -1;
        for (int b = 0; b < level.box_count; ++b) {
            if (sim_active[b] && sim_boxes[b] == to) {
                hit_box = b;
                break;
            }
        }
        if (hit_box == -1) {
            sim_player = to;
            continue;
        }
        if (hit_box != moving_level_idx) return false;

        point next_box = to + MOVE[dir];
        for (int b = 0; b < level.box_count; ++b) {
            if (sim_active[b] && sim_boxes[b] == next_box) return false;
        }
        sim_boxes[hit_box] = next_box;
        sim_player = to;

        uint16_t hit_targets = target_cell_mask[sem][next_box.y][next_box.x];
        hit_targets = static_cast<uint16_t>(hit_targets & remaining_target_mask);
        if (hit_targets != 0) {
            if ((hit_targets & target_bit) == 0 || p + 1 != next_path.size()) return false;
            segment_ok = true;
        }
    }
    return segment_ok;
}

bool Sokoban::try_small_box_macro_solution() {
    if constexpr (!SokobanConfig::ENABLE_SMALL_BOX_MACRO_LAYER) {
        return false;
    }

    StaticArray<point, MAX_PATH_LENGTH> macro_path;
    if (!try_small_box_macro_solution_from_state(initial_state, macro_path)) return false;

    // 少箱宏层的 macro_path 已在“完成顺序 × 目标分配”上穷举择优，是代价最小候选。
    // 加权 IDA* 首解可能更差；两者各自收尾后取更优，避免用更差的首解覆盖已最优的推箱顺序。
    StaticArray<point, MAX_PATH_LENGTH> ida_path;
    bool have_ida = run_ida_search(false, MAX_PATH_LENGTH, ida_path, SMALL_BOX_MACRO_IDA_NODE_BUDGET);
    select_cheaper_finalized(macro_path, have_ida ? &ida_path : nullptr);
    return true;
}

bool Sokoban::try_bomb_then_small_box_macro_solution(
    StaticArray<point, MAX_PATH_LENGTH>& out_path) const {
    out_path.clear();
    if constexpr (!SokobanConfig::ENABLE_SMALL_BOX_MACRO_LAYER) {
        return false;
    }
    int active_bombs = count_active_bomb_tasks(initial_state);
    if (active_bombs <= 0 || active_bombs > MAX_BOMBS) return false;
    if (initial_state.num_boxes > SokobanConfig::SMALL_BOX_MACRO_MAX_BOXES) return false;

    uint8_t bomb_ids[MAX_BOMBS];
    int bomb_count = 0;
    for (uint8_t b = 0; b < initial_state.num_bombs; ++b) {
        if ((initial_state.blown_mask & (1 << b)) == 0 &&
            bomb_tasks[b].target_wall.x != -1) {
            bomb_ids[bomb_count++] = b;
        }
    }
    if (bomb_count != active_bombs) return false;

    StaticArray<point, MAX_PATH_LENGTH> best_path;
    int best_len = MAX_PATH_LENGTH;
    bool used[MAX_BOMBS] = {};

    auto search_order = [&](auto&& self,
                            int depth,
                            const GameState& state,
                            const StaticArray<point, MAX_PATH_LENGTH>& path) -> void {
        if (path.size() >= best_len) return;

        if (depth == bomb_count) {
            StaticArray<point, MAX_PATH_LENGTH> suffix;
            if (!try_small_box_macro_solution_from_state(state, suffix)) return;

            StaticArray<point, MAX_PATH_LENGTH> full = path;
            for (int i = 1; i < suffix.size(); ++i) {
                if (full.size() >= MAX_PATH_LENGTH) return;
                full.push_back(suffix[i]);
            }
            if (full.size() < best_len) {
                best_path = full;
                best_len = full.size();
            }
            return;
        }

        for (int oi = 0; oi < bomb_count; ++oi) {
            if (used[oi]) continue;
            uint8_t b = bomb_ids[oi];
            if (state.blown_mask & (1 << b)) continue;
            if (bomb_tasks[b].target_wall.x == -1) continue;

            StaticArray<point, MAX_PATH_LENGTH> bomb_path;
            if (!build_bomb_macro_path(state, b, bomb_path)) continue;
            if (bomb_path.empty()) continue;

            StaticArray<point, MAX_PATH_LENGTH> next_path = path;
            for (int i = 0; i < bomb_path.size(); ++i) {
                if (next_path.size() >= MAX_PATH_LENGTH) return;
                next_path.push_back(bomb_path[i]);
            }
            if (next_path.size() >= best_len) continue;

            BombTask task = bomb_tasks[b];
            task.bomb_start = {state.bomb_x[b], state.bomb_y[b]};
            GameState next_state;
            if (!build_bomb_task_successor_state(state, b, task, bomb_path.back(), next_state)) continue;

            used[oi] = true;
            self(self, depth + 1, next_state, next_path);
            used[oi] = false;
        }
    };

    StaticArray<point, MAX_PATH_LENGTH> root_path;
    root_path.push_back(initial_state.player);
    search_order(search_order, 0, initial_state, root_path);

    if (best_path.empty()) return false;
    out_path = best_path;
    return true;
}

bool Sokoban::try_small_box_macro_solution_from_state(
    const GameState& state,
    StaticArray<point, MAX_PATH_LENGTH>& out_path) const {
    out_path.clear();

    if (state.num_boxes == 0) {
        out_path.push_back(state.player);
        return true;
    }
    if (state.num_boxes > SokobanConfig::SMALL_BOX_MACRO_MAX_BOXES) return false;
    if (count_active_bomb_tasks(state) != 0) return false;

    int macro_cost_lb = 0;
    for (int i = 0; i < state.num_boxes; ++i) {
        uint8_t sem = state.box_semantics[i];
        uint16_t candidates = static_cast<uint16_t>(state.target_mask & target_semantic_mask[sem]);
        if (candidates == 0) return false;

        int best_pair_cost = 9999;
        int y = state.box_y[i];
        int x = state.box_x[i];
        for (uint16_t mask = candidates; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
            uint16_t bit = static_cast<uint16_t>(mask & -mask);
            uint8_t target_idx = static_cast<uint8_t>(__builtin_ctz(static_cast<unsigned int>(bit)));
            if (t_dist[target_idx][y][x] == -1) continue;
            if (box_target_cost[target_idx][y][x] == 255) continue;
            if (box_target_cost[target_idx][y][x] < best_pair_cost) {
                best_pair_cost = box_target_cost[target_idx][y][x];
            }
        }
        if (best_pair_cost == 9999) return false;
        macro_cost_lb += best_pair_cost;
    }
    if (macro_cost_lb < SokobanConfig::SMALL_BOX_MACRO_MIN_COST_LB) return false;

    SokobanLevel root_level;
    build_level_from_state(state, root_level);

    bool used[MAX_BOXES] = {};
    StaticArray<point, MAX_PATH_LENGTH> candidate_path;
    StaticArray<point, MAX_PATH_LENGTH> best_path;
    int best_len = MAX_PATH_LENGTH;

    if (!search_small_box_macro_order(
            0,
            state,
            state.num_boxes,
            state.target_mask,
            used,
            root_level,
            state.player,
            candidate_path,
            best_path,
            best_len)) {
        return false;
    }

    out_path.push_back(state.player);
    for (int i = 0; i < best_path.size(); ++i) {
        out_path.push_back(best_path[i]);
    }
    if (&state == &initial_state) {
        if (!validate_macro_solution_path(out_path)) {
            out_path.clear();
            return false;
        }
    }
    return true;
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

    if (!PlanningCommon::get_bomb_push_path(macro_level, state.player, task, out_path)) return false;
    if (out_path.empty() || out_path.size() > SokobanConfig::BOMB_MACRO_MAX_PATH) return false;

    point final_player = out_path.back();
    return infer_final_bomb_push_dir(final_player, task.target_wall) < 4;
}

bool Sokoban::build_bomb_task_successor_state(
    const GameState& state,
    int bomb_idx,
    const BombTask& task,
    point final_player,
    GameState& out_state) const {
    out_state = state;

    int8_t box_x[MAX_BOXES];
    int8_t box_y[MAX_BOXES];
    uint8_t box_sem[MAX_BOXES];
    int box_count = state.num_boxes;
    uint16_t target_mask = state.target_mask;

    for (int i = 0; i < box_count; ++i) {
        box_x[i] = state.box_x[i];
        box_y[i] = state.box_y[i];
        box_sem[i] = state.box_semantics[i];
    }

    auto finish_target_for_box = [&](uint8_t sem, point p) {
        uint16_t candidates = static_cast<uint16_t>(target_mask & target_cell_mask[sem][p.y][p.x]);
        if (candidates == 0) return -1;
        return __builtin_ctz(static_cast<unsigned int>(candidates));
    };

    for (int i = 0; i < task.box_pushes.size(); ++i) {
        const BoxPushTask& bp = task.box_pushes[i];
        int moving = -1;
        for (int b = 0; b < box_count; ++b) {
            if (box_x[b] == bp.box_start.x && box_y[b] == bp.box_start.y) {
                moving = b;
                break;
            }
        }
        if (moving == -1) return false;

        box_x[moving] = bp.box_target.x;
        box_y[moving] = bp.box_target.y;

        int t_idx = finish_target_for_box(box_sem[moving], bp.box_target);
        if (t_idx != -1) {
            target_mask = static_cast<uint16_t>(target_mask & ~(1U << t_idx));
            for (int b = moving + 1; b < box_count; ++b) {
                box_x[b - 1] = box_x[b];
                box_y[b - 1] = box_y[b];
                box_sem[b - 1] = box_sem[b];
            }
            --box_count;
        }
    }

    out_state.num_boxes = static_cast<uint8_t>(box_count);
    out_state.target_mask = target_mask;
    for (int i = 0; i < box_count; ++i) {
        out_state.box_x[i] = box_x[i];
        out_state.box_y[i] = box_y[i];
        out_state.box_semantics[i] = box_sem[i];
    }
    out_state.player = final_player;
    out_state.blown_mask = static_cast<uint8_t>(state.blown_mask | (1 << bomb_idx));
    out_state.hash = compute_hash(out_state);
    return true;
}

/// \brief 生成“直接把炸弹推到目标墙并引爆”的宏动作候选
/// \param state 当前搜索状态
/// \param occupancy 当前节点的箱子/炸弹占用表，用于动态通道预筛
/// \param h_before 执行宏动作前的启发式值，用于动作排序
/// \param active_entities 当前仍需处理的箱子与炸弹任务总数
/// \param g 当前已消耗代价
/// \param threshold 当前 IDA* 阈值
/// \param macros 输出候选宏动作数组
/// \return 实际写入 macros 的候选数量
///
/// \details
/// 宏动作只作为额外候选参与排序；普通逐格推炸弹动作仍然保留，因此不会改变可达解空间。
/// 这里先用静态乐观下界和动态占用通道做预筛，避免大量明显会失败的 exact 宏路径规划。
int Sokoban::generate_bomb_macros(
    const GameState& state,
    const NodeOccupancy& occupancy,
    int h_before,
    int active_entities,
    int g,
    int threshold,
    MacroMove macros[MAX_NODE_MACROS]) const {
    if constexpr (!SokobanConfig::ENABLE_BOMB_MACRO) {
        (void)state;
        (void)occupancy;
        (void)h_before;
        (void)active_entities;
        (void)g;
        (void)threshold;
        (void)macros;
        return 0;
    }

    int count = 0;
    // 炸弹自身能经过的格子：允许走到目标墙触发爆炸，允许占用当前这颗炸弹的位置，
    // 但不能穿过箱子、其他炸弹和未炸开的实体墙。
    auto bomb_cell_passable = [&](uint8_t bomb_idx, point p) {
        if (is_overstep(p)) return false;
        if (p == bomb_tasks[bomb_idx].target_wall) return true;
        if (is_solid(p, state.blown_mask)) return false;
        int8_t occ = occupancy.cell[p.y][p.x];
        if (occ >= 0 && occ < NodeOccupancy::BOMB_BASE) return false;
        return occ == NodeOccupancy::EMPTY || occ == NodeOccupancy::BOMB_BASE + bomb_idx;
    };

    // 玩家换边站位必须是真正空格；同样允许忽略当前正在推动的那颗炸弹。
    auto stand_cell_passable = [&](uint8_t bomb_idx, point p) {
        if (is_overstep(p) || is_solid(p, state.blown_mask)) return false;
        int8_t occ = occupancy.cell[p.y][p.x];
        if (occ >= 0 && occ < NodeOccupancy::BOMB_BASE) return false;
        return occ == NodeOccupancy::EMPTY || occ == NodeOccupancy::BOMB_BASE + bomb_idx;
    };

    // 动态通道预筛：只证明“在当前箱子/炸弹占用下，炸弹状态图里是否还存在一条通路”。
    // 它不估计真实路径代价，也不替代 exact 路径规划；失败时才安全地跳过 expensive 检查。
    auto has_dynamic_bomb_corridor_path = [&](uint8_t bomb_idx, point bomb_pos, uint8_t start_dir_mask) {
        uint8_t visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
        struct BombDirNode { int8_t x; int8_t y; uint8_t dir; };
        BombDirNode q[MAP_CELL_COUNT * 4];
        int head = 0, tail = 0;

        for (uint8_t d = 0; d < 4; ++d) {
            if ((start_dir_mask & (1 << d)) == 0) continue;
            visited[bomb_pos.y][bomb_pos.x] |= static_cast<uint8_t>(1 << d);
            q[tail++] = {bomb_pos.x, bomb_pos.y, d};
        }

        while (head < tail) {
            BombDirNode curr = q[head++];
            point pos = {curr.x, curr.y};
            point next = pos + MOVE[curr.dir];
            if (next == bomb_tasks[bomb_idx].target_wall) return true;
            if (bomb_cell_passable(bomb_idx, next) && b_dist[bomb_idx][next.y][next.x] != -1) {
                if ((visited[next.y][next.x] & (1 << curr.dir)) == 0) {
                    visited[next.y][next.x] |= static_cast<uint8_t>(1 << curr.dir);
                    q[tail++] = {next.x, next.y, curr.dir};
                }
            }

            for (uint8_t nd = 0; nd < 4; ++nd) {
                if (nd == curr.dir) continue;
                if ((visited[pos.y][pos.x] & (1 << nd)) != 0) continue;
                point stand = pos - MOVE[nd];
                if (!stand_cell_passable(bomb_idx, stand)) continue;
                visited[pos.y][pos.x] |= static_cast<uint8_t>(1 << nd);
                q[tail++] = {pos.x, pos.y, nd};
            }
        }
        return false;
    };

    for (uint8_t b = 0; b < state.num_bombs && count < MAX_NODE_MACROS; ++b) {
        if (state.blown_mask & (1 << b)) continue;
        if (bomb_tasks[b].target_wall.x == -1) continue;

        point bomb_pos = {state.bomb_x[b], state.bomb_y[b]};
        bool has_clear_tasks = !bomb_tasks[b].box_pushes.empty();
        int direct_lb = b_dist[b][bomb_pos.y][bomb_pos.x];
        bool bomb_committed = bomb_pos != bomb_tasks[b].bomb_start && direct_lb >= 0 && direct_lb <= 4;
        int macro_path_lb = 9999;
        uint8_t start_dir_mask = 0;
        for (uint8_t d = 0; d < 4; ++d) {
            uint8_t cost_from_ready = b_macro_cost[b][bomb_pos.y][bomb_pos.x][d];
            if (cost_from_ready == 255) continue;
            point stand = bomb_pos - MOVE[d];
            int walk = 0;
            if (!has_clear_tasks) {
                if (is_overstep(stand) || bfs_visited_gen[stand.y][stand.x] != current_gen) continue;
                start_dir_mask = static_cast<uint8_t>(start_dir_mask | (1 << d));
                walk = bfs_dist[stand.y][stand.x];
            }
            int cand = walk + cost_from_ready + static_cast<int>(bomb_tasks[b].box_pushes.size()) * 2;
            if (cand < macro_path_lb) macro_path_lb = cand;
        }
        if (macro_path_lb == 9999) continue;

        int h_after_lb = 9999;
        for (uint8_t d = 0; d < 4; ++d) {
            point final_player = bomb_tasks[b].target_wall - MOVE[d];
            uint8_t after_mask = static_cast<uint8_t>(state.blown_mask | (1 << b));
            if (is_overstep(final_player) || is_solid(final_player, after_mask)) continue;
            GameState optimistic_after;
            if (!build_bomb_task_successor_state(
                    state,
                    b,
                    bomb_tasks[b],
                    final_player,
                    optimistic_after)) {
                continue;
            }
            optimistic_after.player = final_player;
            int h_candidate = get_heuristic(optimistic_after);
            if (h_candidate < h_after_lb) h_after_lb = h_candidate;
        }
        if (h_after_lb >= 9999) continue;

        int active_after = active_entities > 0 ? active_entities - 1 : active_entities;
        int optimistic_f = g + macro_path_lb +
                           (h_after_lb * heuristic_weight_num(active_after)) /
                                SokobanConfig::HEURISTIC_WEIGHT_DEN;
        if (bomb_committed) {
            if (g + macro_path_lb > threshold) continue;
        } else {
            optimistic_f += SokobanConfig::BOMB_MACRO_THRESHOLD_MARGIN;
            if (optimistic_f > threshold) continue;
        }
        if (!has_clear_tasks && !has_dynamic_bomb_corridor_path(b, bomb_pos, start_dir_mask)) continue;

        MacroMove& mv = macros[count];
        mv.bomb_idx = b;
        mv.entity_idx = static_cast<uint8_t>(state.num_boxes + b);
        mv.final_push_dir = 4;
        mv.path_cost = static_cast<uint16_t>(macro_path_lb > 65535 ? 65535 : macro_path_lb);
        mv.next_state = state;

        int benefit = h_before - h_after_lb;
        if (benefit < SokobanConfig::BOMB_MACRO_MIN_BENEFIT) continue;
        int sort_key = macro_path_lb * 10 -
                       benefit * heuristic_weight_num(active_entities) -
                       SokobanConfig::BOMB_MACRO_SORT_BONUS;
        if (direct_lb >= 0 && direct_lb <= 3) sort_key -= (4 - direct_lb) * 35;
        if (bomb_committed) sort_key -= 520;
        if (bomb_tasks[b].is_essential) sort_key -= 50;
        if (sort_key < -32000) sort_key = -32000;
        if (sort_key > 32000) sort_key = 32000;
        mv.sort_key = static_cast<int16_t>(sort_key);
        ++count;
    }
    return count;
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

/// \brief 在找到推箱解后，重新拼接某段玩家行走路径
///
/// \details
/// IDA* 搜索阶段只记录推动作；成功后才回放玩家走位。这里在允许少量绕路的前提下
/// 优先减少转弯次数，方便下游小车执行。该函数不参与搜索节点扩展。
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
        if (get_bomb_id(state, p) != -1) return true;
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

/// \brief 对同一个箱子的连续推动片段做局部代价重规划
///
/// \details
/// 替换片段必须从相同动态状态出发，并回到完全相同的箱子、炸弹、目标 mask 和玩家位置。
/// 这样后续原路径仍可直接拼接，局部优化不会改变全局箱子完成顺序。
void Sokoban::optimize_final_box_push_runs() {
    if (final_path.size() <= 2) return;

    auto replay_step = [&](GameState& state,
                           point next,
                           int& pushed_idx,
                           bool& pushed_bomb,
                           uint8_t& move_dir,
                           point& push_to) {
        pushed_idx = -1;
        pushed_bomb = false;
        move_dir = dir_between(state.player, next);
        push_to = {-1, -1};
        if (move_dir >= 4 || is_overstep(next) || is_solid(next, state.blown_mask)) return false;

        int box_idx = find_box_id(state, next);
        int bomb_idx = get_bomb_id(state, next);
        if (box_idx == -1 && bomb_idx == -1) {
            state.player = next;
            return true;
        }

        push_to = next + MOVE[move_dir];
        if (is_overstep(push_to)) return false;
        if (find_box_id(state, push_to) != -1 || get_bomb_id(state, push_to) != -1) return false;

        if (box_idx != -1) {
            if (is_solid(push_to, state.blown_mask)) return false;
            pushed_idx = box_idx;
            int target_idx = find_active_target_index(state, state.target_mask, push_to, box_idx);
            if (target_idx != -1) {
                state.target_mask = static_cast<uint16_t>(state.target_mask & ~(1U << target_idx));
                for (int b = box_idx; b + 1 < state.num_boxes; ++b) {
                    state.box_x[b] = state.box_x[b + 1];
                    state.box_y[b] = state.box_y[b + 1];
                    state.box_semantics[b] = state.box_semantics[b + 1];
                }
                --state.num_boxes;
            } else {
                state.box_x[box_idx] = push_to.x;
                state.box_y[box_idx] = push_to.y;
            }
        } else {
            pushed_idx = bomb_idx;
            pushed_bomb = true;
            bool explodes = is_solid(push_to, state.blown_mask) &&
                            bomb_tasks[bomb_idx].target_wall == push_to;
            if (is_solid(push_to, state.blown_mask) && !explodes) return false;
            if (explodes) {
                state.blown_mask = static_cast<uint8_t>(state.blown_mask | (1 << bomb_idx));
            } else {
                state.bomb_x[bomb_idx] = push_to.x;
                state.bomb_y[bomb_idx] = push_to.y;
            }
        }

        state.player = next;
        return true;
    };

    auto same_state = [](const GameState& a, const GameState& b) {
        if (a.player != b.player ||
            a.num_boxes != b.num_boxes ||
            a.num_bombs != b.num_bombs ||
            a.target_mask != b.target_mask ||
            a.blown_mask != b.blown_mask) {
            return false;
        }
        for (int i = 0; i < a.num_boxes; ++i) {
            if (a.box_x[i] != b.box_x[i] ||
                a.box_y[i] != b.box_y[i] ||
                a.box_semantics[i] != b.box_semantics[i]) {
                return false;
            }
        }
        for (int i = 0; i < a.num_bombs; ++i) {
            if (a.bomb_x[i] != b.bomb_x[i] || a.bomb_y[i] != b.bomb_y[i]) return false;
        }
        return true;
    };

    auto segment_cost = [](point start,
                           const StaticArray<point, MAX_PATH_LENGTH>& segment,
                           int initial_dir,
                           int next_dir) {
        int cost = 0;
        int last_dir = initial_dir;
        point curr = start;
        for (int i = 0; i < segment.size(); ++i) {
            uint8_t dir = dir_between(curr, segment[i]);
            if (dir >= 4) return 9999;
            ++cost;
            if (last_dir >= 0 && last_dir < 4 && last_dir != dir) cost += DISPLAY_TURN_COST;
            last_dir = dir;
            curr = segment[i];
        }
        if (next_dir >= 0 && next_dir < 4 && last_dir >= 0 && last_dir < 4 && last_dir != next_dir) {
            cost += DISPLAY_TURN_COST;
        }
        return cost;
    };

    StaticArray<point, MAX_PATH_LENGTH> original = final_path;
    StaticArray<point, MAX_PATH_LENGTH> rebuilt;
    rebuilt.push_back(original[0]);
    GameState state = initial_state;
    bool changed = false;

    int i = 1;
    while (i < original.size()) {
        GameState first_after = state;
        int first_idx = -1;
        bool first_is_bomb = false;
        uint8_t first_dir = 4;
        point first_to = {-1, -1};
        if (!replay_step(first_after, original[i], first_idx, first_is_bomb, first_dir, first_to)) return;

        if (first_idx == -1 || first_is_bomb) {
            rebuilt.push_back(original[i]);
            state = first_after;
            ++i;
            continue;
        }

        const GameState run_start_state = state;
        const int run_box_idx = first_idx;
        const uint8_t run_semantic = state.box_semantics[run_box_idx];
        const point box_start = {state.box_x[run_box_idx], state.box_y[run_box_idx]};
        GameState scan_state = state;
        GameState run_end_state = first_after;
        point box_end = first_to;
        uint8_t final_push_dir = first_dir;
        int run_end = i + 1;
        int push_count = 0;
        int direction_run_count = 0;
        uint8_t previous_push_dir = 4;

        for (int j = i; j < original.size(); ++j) {
            GameState next_state = scan_state;
            int pushed_idx = -1;
            bool pushed_bomb = false;
            uint8_t push_dir = 4;
            point push_target = {-1, -1};
            if (!replay_step(next_state, original[j], pushed_idx, pushed_bomb, push_dir, push_target)) return;

            if (pushed_idx != -1) {
                if (pushed_bomb || pushed_idx != run_box_idx ||
                    scan_state.box_semantics[pushed_idx] != run_semantic) {
                    break;
                }
                if (push_count == 0) {
                    direction_run_count = 1;
                } else if (push_dir != previous_push_dir) {
                    ++direction_run_count;
                }
                ++push_count;
                box_end = push_target;
                final_push_dir = push_dir;
                run_end = j + 1;
                run_end_state = next_state;
                previous_push_dir = push_dir;
                bool completed = next_state.num_boxes < scan_state.num_boxes;
                scan_state = next_state;
                if (completed) break;
                continue;
            }
            scan_state = next_state;
        }

        StaticArray<point, MAX_PATH_LENGTH> original_segment;
        for (int k = i; k < run_end; ++k) original_segment.push_back(original[k]);

        bool use_candidate = false;
        StaticArray<point, MAX_PATH_LENGTH> candidate;
        // 对完整连续推动段重规划，允许把换向点向后推迟，避免局部前缀最优锁死后续短路
        if (push_count >= 2 && direction_run_count >= 2) {
            SokobanLevel level;
            build_level_from_state(run_start_state, level);
            point candidate_player = run_start_state.player;
            BoxPushTask task{box_start, box_end};
            int initial_dir = rebuilt.size() >= 2
                ? dir_between(rebuilt[rebuilt.size() - 2], rebuilt.back())
                : -1;
            int next_dir = run_end < original.size()
                ? dir_between(original[run_end - 1], original[run_end])
                : -1;

            if (PlanningCommon::append_box_push_optimized_path(
                    level,
                    candidate_player,
                    task,
                    candidate,
                    initial_dir,
                    final_push_dir) &&
                candidate_player == run_end_state.player &&
                rebuilt.size() + candidate.size() + original.size() - run_end <= MAX_PATH_LENGTH) {
                GameState candidate_end = run_start_state;
                bool candidate_valid = true;
                for (int k = 0; k < candidate.size(); ++k) {
                    int pushed_idx = -1;
                    bool pushed_bomb = false;
                    uint8_t push_dir = 4;
                    point push_target = {-1, -1};
                    if (!replay_step(candidate_end, candidate[k], pushed_idx, pushed_bomb, push_dir, push_target)) {
                        candidate_valid = false;
                        break;
                    }
                }
                if (candidate_valid && same_state(candidate_end, run_end_state)) {
                    int original_cost = segment_cost(run_start_state.player, original_segment, initial_dir, next_dir);
                    int candidate_cost = segment_cost(run_start_state.player, candidate, initial_dir, next_dir);
                    use_candidate = candidate_cost < original_cost;
                }
            }
        }

        const StaticArray<point, MAX_PATH_LENGTH>& chosen = use_candidate ? candidate : original_segment;
        for (int k = 0; k < chosen.size(); ++k) rebuilt.push_back(chosen[k]);
        if (use_candidate) changed = true;
        state = run_end_state;
        i = run_end;
    }

    if (changed) final_path = rebuilt;
}

/// \brief 对最终路径做轻量后处理，尝试减少行走转弯
///
/// \details
/// 先重规划边界状态一致的同箱连续推动片段，再替换相邻推动作之间的玩家行走段。
/// 每一层都按面板总代价验收，没有收益时保留进入该层前的路径。
void Sokoban::optimize_final_path_turns() {
    if (final_path.size() <= 2) return;

    optimize_final_box_push_runs();

    StaticArray<point, MAX_PATH_LENGTH> original = final_path;
    int original_cost = path_display_cost(original);
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
            int bomb_id = get_bomb_id(state, obj);
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
                if (is_solid(push_to, state.blown_mask) &&
                    bomb_tasks[movable_idx].target_wall == push_to) {
                    // 后处理复盘必须同步爆破 mask，否则后续行走段仍按旧墙体规划
                    state.blown_mask = static_cast<uint8_t>(state.blown_mask | (1 << movable_idx));
                } else {
                    state.bomb_x[movable_idx] = push_to.x;
                    state.bomb_y[movable_idx] = push_to.y;
                }
            } else {
                int t_idx = find_active_target_index(state, state.target_mask, push_to, movable_idx);
                bool box_finished = t_idx != -1;
                if (box_finished) {
                    state.target_mask = static_cast<uint16_t>(state.target_mask & ~(1U << t_idx));
                }

                if (box_finished) {
                    for (int b = movable_idx; b < state.num_boxes - 1; ++b) {
                        state.box_x[b] = state.box_x[b + 1];
                        state.box_y[b] = state.box_y[b + 1];
                        state.box_semantics[b] = state.box_semantics[b + 1];
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
                if (find_box_id(state, obj) != -1 || get_bomb_id(state, obj) != -1) next_is_push = true;
            }
            if (next_is_push) break;
            walk_end++;
        }

        uint8_t next_dir = 4;
        if (walk_end + 1 < final_path.size()) next_dir = dir_between(final_path[walk_end], final_path[walk_end + 1]);

        uint8_t out_dir = prev_dir;
        int original_walk_steps = walk_end - walk_start + 1;
        int max_walk_steps = original_walk_steps + 6;
        if (!append_optimized_walk_segment(state, curr, final_path[walk_end], prev_dir, next_dir, max_walk_steps, optimized, out_dir)) {
            for (int k = walk_start; k <= walk_end; ++k) optimized.push_back(final_path[k]);
            out_dir = dir_between(curr, final_path[walk_end]);
        }
        if (optimized.size() > 1) prev_dir = out_dir;
        state.player = final_path[walk_end];
        i = walk_end + 1;
    }

    if (path_display_cost(optimized) <= original_cost) {
        final_path = optimized;
    } else {
        final_path = original;
    }
}


// ============================================================================
// 模块 4：哈希、匹配与启发式估价
// ============================================================================

/// \brief 按统一语义规则计算搜索状态的 Zobrist 哈希
/// \param state 搜索状态
/// \return 32 位 Zobrist 哈希值
uint32_t Sokoban::compute_hash(const GameState& state) const {
    uint32_t h = 0;
    for (int i = 0; i < state.num_boxes; ++i) {
        h ^= ZOBRIST_SPECIFIC_BOX[state.box_semantics[i]][state.box_y[i]][state.box_x[i]];
    }
    h ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
    for (size_t i = 0; i < initial_targets.size(); ++i) {
        if (state.target_mask & (1U << i)) h ^= ZOBRIST_TARGET[i];
    }
    h ^= ZOBRIST_BLOWN_MASK[state.blown_mask];
    for (int b = 0; b < state.num_bombs; ++b) {
        if (!(state.blown_mask & (1 << b))) {
            h ^= ZOBRIST_BOMB[b][state.bomb_y[b]][state.bomb_x[b]];
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
/// 用于同一语义组内部的乐观最小匹配下界
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

int Sokoban::nearest_active_target_distance(const GameState& state, uint8_t semantic_id, point box_pos) const {
    int best = 9999;
    uint16_t candidates = static_cast<uint16_t>(state.target_mask & target_semantic_mask[semantic_id]);
    for (uint16_t mask = candidates; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
        uint16_t bit = static_cast<uint16_t>(mask & -mask);
        int t = __builtin_ctz(static_cast<unsigned int>(bit));
        int d = t_dist[t][box_pos.y][box_pos.x];
        if (d >= 0 && d < best) best = d;
    }
    return best == 9999 ? -1 : best;
}

/// \brief 计算 IDA* 启发式下界
/// \param state 当前搜索状态
/// \return 乐观剩余代价；返回 9999 表示该状态不可解
///
/// \details
/// 同语义箱子和目标点可以互换；每个语义组内部做最小权匹配，避免重复语义时
/// 多个箱子“抢”同一个目标。炸弹任务如果存在，也一并纳入下界。
int Sokoban::get_heuristic(const GameState& state) const {
    if (state.num_boxes == 0) return 0; 

    bool use_box_target_cost_lb = false;
    if constexpr (SokobanConfig::ENABLE_BOX_TARGET_COST_LB) {
        use_box_target_cost_lb =
            force_box_target_cost_lb ||
            strict_cost_search ||
            current_threshold_iteration >= SokobanConfig::BOX_TARGET_COST_MIN_ITERATION;
    }
    auto target_cost = [&](int target_idx, int y, int x) {
        if (use_box_target_cost_lb) {
            int cost = box_target_cost[target_idx][y][x];
            return cost == 255 ? 9999 : cost;
        }
        int dist = t_dist[target_idx][y][x];
        return dist == -1 ? 9999 : dist;
    };

    point starts[MAX_BOXES + MAX_BOMBS];
    point ends[MAX_BOXES + MAX_BOMBS];
    int task_count = 0;
    int sum_push = 0;
    uint8_t box_count_by_sem[10] = {};
    uint8_t first_box_by_sem[10];
    bool processed_sem[10] = {};

    for (uint8_t sem = 0; sem < 10; ++sem) first_box_by_sem[sem] = 255;
    for (uint8_t i = 0; i < state.num_boxes; ++i) {
        uint8_t sem = state.box_semantics[i];
        if (first_box_by_sem[sem] == 255) first_box_by_sem[sem] = i;
        ++box_count_by_sem[sem];
    }

    for (uint8_t seed = 0; seed < state.num_boxes; ++seed) {
        uint8_t sem = state.box_semantics[seed];
        if (processed_sem[sem]) continue;
        processed_sem[sem] = true;

        int n = box_count_by_sem[sem];
        uint16_t sem_targets = static_cast<uint16_t>(state.target_mask & target_semantic_mask[sem]);
        int m = __builtin_popcount(static_cast<unsigned int>(sem_targets));
        if (m != n) return 9999;

        if (n == 1) {
            int b = first_box_by_sem[sem];
            int t = __builtin_ctz(static_cast<unsigned int>(sem_targets));
            int dist = target_cost(t, state.box_y[b], state.box_x[b]);
            if (dist >= 9999) return 9999;
            sum_push += dist;
            starts[task_count] = {state.box_x[b], state.box_y[b]};
            ends[task_count] = initial_targets[t];
            task_count++;
            continue;
        }

        int box_idx[MAX_BOXES];
        int target_idx[MAX_BOXES];
        int bi = 0;
        for (int i = 0; i < state.num_boxes; ++i) {
            if (state.box_semantics[i] == sem) box_idx[bi++] = i;
        }
        int ti = 0;
        for (uint16_t mask = sem_targets; mask != 0; mask = static_cast<uint16_t>(mask & (mask - 1))) {
            uint16_t bit = static_cast<uint16_t>(mask & -mask);
            target_idx[ti++] = __builtin_ctz(static_cast<unsigned int>(bit));
        }

        if (n == 2) {
            int b0 = box_idx[0];
            int b1 = box_idx[1];
            int t0 = target_idx[0];
            int t1 = target_idx[1];
            int d00 = target_cost(t0, state.box_y[b0], state.box_x[b0]);
            int d01 = target_cost(t1, state.box_y[b0], state.box_x[b0]);
            int d10 = target_cost(t0, state.box_y[b1], state.box_x[b1]);
            int d11 = target_cost(t1, state.box_y[b1], state.box_x[b1]);
            int c0 = (d00 >= 9999 || d11 >= 9999) ? 9999 : d00 + d11;
            int c1 = (d01 >= 9999 || d10 >= 9999) ? 9999 : d01 + d10;
            int min_h = (c0 < c1) ? c0 : c1;
            if (min_h >= 9999) return 9999;
            sum_push += min_h;

            for (int i = 0; i < 2; ++i) {
                int b = box_idx[i];
                int best_target = target_idx[0];
                int best_dist = target_cost(best_target, state.box_y[b], state.box_x[b]);
                int alt_target = target_idx[1];
                int alt_dist = target_cost(alt_target, state.box_y[b], state.box_x[b]);
                if (best_dist >= 9999 || (alt_dist < best_dist)) {
                    best_dist = alt_dist;
                    best_target = alt_target;
                }
                starts[task_count] = {state.box_x[b], state.box_y[b]};
                ends[task_count] = initial_targets[best_target];
                task_count++;
            }
            continue;
        }

        int cost_matrix[MAX_BOXES][MAX_BOXES];
        int best_target_for_box[MAX_BOXES];
        for (int i = 0; i < n; ++i) {
            int b = box_idx[i];
            int best_target = target_idx[0];
            int best_dist = 9999;
            for (int j = 0; j < n; ++j) {
                int t = target_idx[j];
                int dist = target_cost(t, state.box_y[b], state.box_x[b]);
                cost_matrix[i][j] = dist;
                if (dist < best_dist) {
                    best_dist = dist;
                    best_target = t;
                }
            }
            best_target_for_box[i] = best_target;
        }

        int min_h = min_weight_assignment<MAX_BOXES>(cost_matrix, n);
        if (min_h >= 9999) return 9999;
        sum_push += min_h;

        for (int i = 0; i < n; ++i) {
            int b = box_idx[i];
            starts[task_count] = {state.box_x[b], state.box_y[b]};
            ends[task_count] = initial_targets[best_target_for_box[i]];
            task_count++;
        }
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



// ============================================================================
// 模块 5：空间预计算
// ============================================================================

// 初始化所有 Zobrist 随机表，用于 O(1) 增量哈希
void Sokoban::init_zobrist() {
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            ZOBRIST_PLAYER[y][x] = xorshift32();

            for (int i = 0; i < SystemConfig::MAX_BOXES; ++i) {
                ZOBRIST_SPECIFIC_BOX[i][y][x] = xorshift32();
            }
        }
    }
    for (int i = 0; i < MAX_BOXES; ++i) ZOBRIST_TARGET[i] = xorshift32();
}

/// \brief 预计算不同炸墙状态下的实体墙占用表
///
/// \details
/// blown_mask 会让部分墙在搜索中变为空地。把“墙是否仍然阻挡”提前展开成小表，
/// 可让热路径里的 is_solid 只做边界检查和一次数组读取。
void Sokoban::precompute_solid_masks() {
    for (uint8_t mask = 0; mask < (1 << MAX_BOMBS); ++mask) {
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                solid_mask[mask][y][x] =
                    (map[y][x] == 1 && (wall_clear_mask[y][x] & mask) == 0) ? 1 : 0;
            }
        }
    }
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
                point player_prev = curr - MOVE[dir] - MOVE[dir];
                
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
    std::memset(RELAXED_PUSH_STAND_DIST, 255, sizeof(RELAXED_PUSH_STAND_DIST));
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

    for (int from = 0; from < MAP_CELL_COUNT; ++from) {
        for (int oy = 0; oy < MAP_MAX_HEIGHT; ++oy) {
            for (int ox = 0; ox < MAP_MAX_WIDTH; ++ox) {
                uint8_t best = 255;
                for (int d = 0; d < 4; ++d) {
                    point stand = {
                        static_cast<int8_t>(ox - MOVE[d].x),
                        static_cast<int8_t>(oy - MOVE[d].y)
                    };
                    if (is_overstep(stand)) continue;
                    int stand_idx = stand.y * MAP_MAX_WIDTH + stand.x;
                    uint8_t walk = relaxed_walk_dist[from][stand_idx];
                    if (walk < best) best = walk;
                }
                RELAXED_PUSH_STAND_DIST[from][oy * MAP_MAX_WIDTH + ox] = best;
            }
        }
    }
}

/// \brief 预计算单箱到目标的乐观总代价下界
///
/// \details
/// t_dist 只统计推动次数，容易低估需要多次转向的箱子通路。这里按“箱子位置 + 当前推动方向”
/// 做一次反向 Dijkstra，把推箱和玩家绕到下一发力点的静态行走下界合进去。
/// 运行时启发式只查 box_target_cost，不在 IDA* 热路径里跑 exact 单箱求解。
void Sokoban::precompute_box_target_costs() {
    std::memset(box_target_cost, 255, sizeof(box_target_cost));
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1;

    auto cell_free = [&](point p) {
        return !is_overstep(p) && !is_solid(p, all_blown_mask);
    };

    constexpr uint16_t INF = 65535;
    constexpr int NODE_COUNT = MAP_CELL_COUNT * 4;

    for (int t = 0; t < initial_targets.size(); ++t) {
        uint16_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
        bool used[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                for (int d = 0; d < 4; ++d) {
                    dist[y][x][d] = INF;
                    used[y][x][d] = false;
                }
            }
        }

        point target = initial_targets[t];
        for (uint8_t d = 0; d < 4; ++d) {
            point stand = target - MOVE[d];
            if (cell_free(stand)) dist[target.y][target.x][d] = 0;
        }
        box_target_cost[t][target.y][target.x] = 0;

        for (int iter = 0; iter < NODE_COUNT; ++iter) {
            int best_x = -1, best_y = -1, best_d = -1;
            uint16_t best = INF;
            for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
                for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                    for (int d = 0; d < 4; ++d) {
                        if (!used[y][x][d] && dist[y][x][d] < best) {
                            best = dist[y][x][d];
                            best_x = x;
                            best_y = y;
                            best_d = d;
                        }
                    }
                }
            }
            if (best == INF) break;
            used[best_y][best_x][best_d] = true;

            point curr = {static_cast<int8_t>(best_x), static_cast<int8_t>(best_y)};

            point prev_box = curr - MOVE[best_d];
            point prev_stand = prev_box - MOVE[best_d];
            if (cell_free(prev_box) && cell_free(prev_stand)) {
                uint16_t cand = static_cast<uint16_t>(best + 1);
                if (cand < dist[prev_box.y][prev_box.x][best_d]) {
                    dist[prev_box.y][prev_box.x][best_d] = cand;
                }
            }

            point curr_stand = curr - MOVE[best_d];
            for (uint8_t prev_d = 0; prev_d < 4; ++prev_d) {
                if (prev_d == best_d) continue;
                point prev_side = curr - MOVE[prev_d];
                if (!cell_free(prev_side) || !cell_free(curr_stand)) continue;
                int walk = walk_dist_between(prev_side, curr_stand);
                if (walk == 9999) continue;
                int cand_i = static_cast<int>(best) + walk;
                if (cand_i > 254) cand_i = 254;
                uint16_t cand = static_cast<uint16_t>(cand_i);
                if (cand < dist[curr.y][curr.x][prev_d]) {
                    dist[curr.y][curr.x][prev_d] = cand;
                }
            }
        }

        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                if (t_dist[t][y][x] == -1) continue;
                uint16_t best = INF;
                for (uint8_t d = 0; d < 4; ++d) {
                    if (dist[y][x][d] < best) best = dist[y][x][d];
                }
                if (best != INF) {
                    box_target_cost[t][y][x] =
                        static_cast<uint8_t>(best > 254 ? 254 : best);
                }
            }
        }
    }
}

/// \brief 预计算炸弹到目标墙体的反向推动距离场
///
/// \details
/// 只对已经绑定了 target_wall 的炸弹任务计算距离。
/// 结果写入 b_dist[bomb_id][y][x]，用于炸弹启发式估价和剪枝。
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

void Sokoban::precompute_bomb_macro_costs() {
    std::memset(b_macro_cost, 255, sizeof(b_macro_cost));
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1;

    auto cell_free = [&](point p) {
        return !is_overstep(p) && !is_solid(p, all_blown_mask);
    };

    constexpr uint16_t INF = 65535;
    constexpr int NODE_COUNT = MAP_CELL_COUNT * 4;

    for (int b = 0; b < num_bomb_tasks; ++b) {
        if (bomb_tasks[b].target_wall.x == -1) continue;

        uint16_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
        bool used[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                for (int d = 0; d < 4; ++d) {
                    dist[y][x][d] = INF;
                    used[y][x][d] = false;
                }
            }
        }

        point target = bomb_tasks[b].target_wall;
        for (uint8_t d = 0; d < 4; ++d) {
            point stand = target - MOVE[d];
            if (cell_free(stand)) dist[target.y][target.x][d] = 0;
        }

        auto bomb_cell_free = [&](point p) {
            if (is_overstep(p)) return false;
            if (p == target) return true;
            return !is_solid(p, all_blown_mask);
        };

        for (int iter = 0; iter < NODE_COUNT; ++iter) {
            int best_x = -1, best_y = -1, best_d = -1;
            uint16_t best = INF;
            for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
                for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                    for (int d = 0; d < 4; ++d) {
                        if (!used[y][x][d] && dist[y][x][d] < best) {
                            best = dist[y][x][d];
                            best_x = x;
                            best_y = y;
                            best_d = d;
                        }
                    }
                }
            }
            if (best == INF) break;
            used[best_y][best_x][best_d] = true;

            point curr = {static_cast<int8_t>(best_x), static_cast<int8_t>(best_y)};

            point prev_bomb = curr - MOVE[best_d];
            point prev_stand = prev_bomb - MOVE[best_d];
            if (bomb_cell_free(prev_bomb) && cell_free(prev_stand)) {
                uint16_t cand = static_cast<uint16_t>(best + 1);
                if (cand < dist[prev_bomb.y][prev_bomb.x][best_d]) {
                    dist[prev_bomb.y][prev_bomb.x][best_d] = cand;
                }
            }

            point curr_stand = curr - MOVE[best_d];
            for (uint8_t prev_d = 0; prev_d < 4; ++prev_d) {
                if (prev_d == best_d) continue;
                point prev_side = curr - MOVE[prev_d];
                if (!cell_free(prev_side) || !cell_free(curr_stand)) continue;
                int walk = walk_dist_between(prev_side, curr_stand);
                if (walk == 9999) continue;
                int cand_i = static_cast<int>(best) + walk;
                if (cand_i > 254) cand_i = 254;
                uint16_t cand = static_cast<uint16_t>(cand_i);
                if (cand < dist[curr.y][curr.x][prev_d]) {
                    dist[curr.y][curr.x][prev_d] = cand;
                }
            }
        }

        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                for (int d = 0; d < 4; ++d) {
                    if (dist[y][x][d] != INF) {
                        b_macro_cost[b][y][x][d] =
                            static_cast<uint8_t>(dist[y][x][d] > 254 ? 254 : dist[y][x][d]);
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
    return solid_mask[blown_mask][p.y][p.x] != 0;
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

inline int Sokoban::get_bomb_id(const GameState& state, point p) const {
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
    if (is_overstep(from) || is_overstep(obj)) return 9999;
    int from_idx = from.y * MAP_MAX_WIDTH + from.x;
    int obj_idx = obj.y * MAP_MAX_WIDTH + obj.x;
    uint8_t d = RELAXED_PUSH_STAND_DIST[from_idx][obj_idx];
    return (d == 255) ? 9999 : d;
}
