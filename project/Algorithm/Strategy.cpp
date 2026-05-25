#include "Strategy.h"
#include <cstring>
#include <algorithm>
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
#include <cstdio>
#endif
// ============================================================================
// Strategy.cpp 文件结构
// ============================================================================
// 1. 全局配置与 DFS 缓存区
// 2. Phase1 任意箱-任意目标匹配评估辅助函数
// 3. 对外入口：选择炸弹任务序列
// 4. DFS 策略搜索：枚举候选墙体并评估收益
// 5. 推物体距离场：fast_push_bfs / macro_soft_dijkstra
// 6. 软障碍宏观拓扑评估
// 7. 局部清障回退：生成真实推箱让路任务
// 8. 任务实体化与快速可执行性验证
// 9. 模板显式实例化
//
// 注意：
// - 本文件包含模板函数，函数顺序不要随意大规模移动
// - 静态缓存数组放在 DTCM 段，主要为 RT1064 上板性能服务
// ============================================================================

// ============================================================================
// 1. 全局配置与 DFS 缓存区
// ============================================================================
__attribute__((section(".dtcm_data"))) StrategicPlanner strategic_planner;

// DFS 每层最多保留的高分候选数，限制分支数量比盲目遍历全图墙体更适合单片机
constexpr uint8_t PHASE1_SELECTION_RESTRICTIONS = 4;
constexpr uint8_t PHASE2_SELECTION_RESTRICTIONS = 6;
constexpr int16_t INF_DIST = 9999;
constexpr int PHASE1_SOFT_REPLACE_PROFIT_MARGIN = 20;
constexpr int LOCAL_CLEAR_MAX_TASKS = 8;
constexpr int LOCAL_CLEAR_MAX_ITER = 5;
constexpr int LOCAL_CLEAR_CANDIDATE_LIMIT = 10;
constexpr int LOCAL_CLEAR_CHAIN_DEPTH = 1;

// Phase1 中“目标点是否已被匹配”的 bitmask 数量
constexpr int PHASE1_MATCH_MASKS = 1 << MAX_BOXES;

static int strategy_box_at(const SokobanLevel& lvl, point p) {
    for (int b = 0; b < lvl.box_count; ++b) {
        if (lvl.boxes[b] == p) return b;
    }
    return -1;
}

static bool strategy_target_at(const SokobanLevel& lvl, point p) {
    for (int t = 0; t < lvl.target_count; ++t) {
        if (lvl.targets[t] == p) return true;
    }
    return false;
}

static void mark_soft_deadlock_boxes(const SokobanLevel& lvl, bool out_hard[MAX_BOXES]) {
    std::memset(out_hard, 0, sizeof(bool) * MAX_BOXES);

    auto is_wall = [&](point p) {
        if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
        return lvl.map[p.y][p.x] == 1;
    };

    for (int b = 0; b < lvl.box_count; ++b) {
        point p = lvl.boxes[b];
        if (strategy_target_at(lvl, p)) continue;

        bool up = is_wall(p + MOVE[0]);
        bool right = is_wall(p + MOVE[1]);
        bool down = is_wall(p + MOVE[2]);
        bool left = is_wall(p + MOVE[3]);
        if ((up && right) || (right && down) || (down && left) || (left && up)) {
            out_hard[b] = true;
        }
    }

    for (int y = 0; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH - 1; ++x) {
            int box_id[2][2];
            bool wall[2][2];
            int box_count = 0;
            int wall_count = 0;

            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    point p = {static_cast<int8_t>(x + dx), static_cast<int8_t>(y + dy)};
                    box_id[dy][dx] = strategy_box_at(lvl, p);
                    wall[dy][dx] = (lvl.map[p.y][p.x] == 1);
                    if (box_id[dy][dx] >= 0) ++box_count;
                    if (wall[dy][dx]) ++wall_count;
                }
            }

            bool deadlock_block = false;
            if (box_count >= 3 && wall_count >= 1) {
                deadlock_block = true;
            } else if (box_count == 2 && wall_count == 2) {
                bool boxes_top = box_id[0][0] >= 0 && box_id[0][1] >= 0;
                bool boxes_bottom = box_id[1][0] >= 0 && box_id[1][1] >= 0;
                bool boxes_left = box_id[0][0] >= 0 && box_id[1][0] >= 0;
                bool boxes_right = box_id[0][1] >= 0 && box_id[1][1] >= 0;
                bool walls_top = wall[0][0] && wall[0][1];
                bool walls_bottom = wall[1][0] && wall[1][1];
                bool walls_left = wall[0][0] && wall[1][0];
                bool walls_right = wall[0][1] && wall[1][1];
                deadlock_block =
                    (boxes_top && walls_bottom) ||
                    (boxes_bottom && walls_top) ||
                    (boxes_left && walls_right) ||
                    (boxes_right && walls_left);
            }

            if (!deadlock_block) continue;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    if (box_id[dy][dx] >= 0) out_hard[box_id[dy][dx]] = true;
                }
            }
        }
    }
}

// dfs_dist_box[depth][box][y][x]：当前 DFS 深度下，某箱子被推到 (x,y) 的估计代价
__attribute__((section(".dtcm_data"))) static int16_t dfs_dist_box[MAX_BOMBS + 1][MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// dfs_dist_bomb[depth][bomb][y][x]：当前 DFS 深度下，某炸弹被推到 (x,y) 的估计代价
__attribute__((section(".dtcm_data"))) static int16_t dfs_dist_bomb[MAX_BOMBS + 1][MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// dfs_player_vis[depth][y][x]：当前 DFS 深度下玩家可达区域
__attribute__((section(".dtcm_data"))) static bool dfs_player_vis[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// Phase1 任意匹配 DP 缓存，避免递归过程中频繁申请大数组
__attribute__((section(".dtcm_bss"))) static int phase1_match_dp[PHASE1_MATCH_MASKS];
__attribute__((section(".dtcm_bss"))) static int phase1_match_next[PHASE1_MATCH_MASKS];

struct StrategyDfsScratch {
    StaticArray<BombCandidate, 256> candidates[MAX_BOMBS + 1];
};

static MCU_OCRAM_BSS StrategyDfsScratch strategy_dfs_ws;

// ============================================================================
// 2. Phase1 任意箱-任意目标匹配评估辅助函数
// ============================================================================

/// \brief 统计 bitmask 中置 1 的位数
/// \param mask 目标集合掩码
/// \return mask 中为 1 的 bit 数量
///
/// \details
/// Phase1 任意匹配 DP 需要知道某个目标集合已经匹配了多少目标
/// 这里不用标准库 bitset/popcount，保持对嵌入式编译环境的兼容
static inline int bit_count_u16(uint16_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 1);
        mask >>= 1;
    }
    return count;
}

/// \brief 评估 Phase1 任意箱子到任意目标的最佳匹配质量
/// \param lvl 当前炸弹序列执行后的地图状态
/// \param box_dist box_dist[b][y][x] 表示第 b 个箱子被推到 (x,y) 的估计代价
/// \param selected_task_count 当前已经选择的炸弹任务数量，用于调整惩罚权重
/// \param out_deadlocks 输出：仍无法匹配到目标的箱子数量
/// \param out_distance 输出：匹配距离、不可达惩罚和死锁惩罚的综合代价
///
/// \details
/// Phase1 尚未完成语义绑定，所以不能按固定箱-目标关系评估
/// 这里用目标集合 bitmask 做动态规划：依次处理每个箱子，尝试分配给任意未使用且可达的目标
/// 最终优先最大化匹配数量；匹配数量相同再最小化总推动距离
static void evaluate_phase1_any_matching(
    const SokobanLevel& lvl,
    int16_t box_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    int selected_task_count,
    int& out_deadlocks,
    int& out_distance)
{
    const int assign_inf = 999999;
    if (lvl.box_count == 0) {
        out_deadlocks = 0;
        out_distance = 0;
        return;
    }

    int mask_limit = 1 << lvl.target_count;
    int all_pair_distance = 0;
    int unreachable_pairs = 0;

    // 先统计所有箱-目标对的粗略距离，用作“整体通路质量”的辅助分
    for (int b = 0; b < lvl.box_count; ++b) {
        for (int t = 0; t < lvl.target_count; ++t) {
            point target = lvl.targets[t];
            int d = box_dist[b][target.y][target.x];
            if (d == INF_DIST) {
                ++unreachable_pairs;
            } else {
                all_pair_distance += d;
            }
        }
    }

    for (int mask = 0; mask < mask_limit; ++mask) phase1_match_dp[mask] = assign_inf;
    phase1_match_dp[0] = 0;

    int* cur = phase1_match_dp;
    int* next = phase1_match_next;

    // DP 状态：cur[mask] 表示已经使用 mask 中目标时的最小总推动距离
    for (int b = 0; b < lvl.box_count; ++b) {
        for (int mask = 0; mask < mask_limit; ++mask) next[mask] = cur[mask];

        for (int mask = 0; mask < mask_limit; ++mask) {
            if (cur[mask] >= assign_inf) continue;
            for (int t = 0; t < lvl.target_count; ++t) {
                if (mask & (1 << t)) continue;
                point target = lvl.targets[t];
                int d = box_dist[b][target.y][target.x];
                if (d == INF_DIST) continue;

                int next_mask = mask | (1 << t);
                int next_cost = cur[mask] + d;
                if (next_cost < next[next_mask]) next[next_mask] = next_cost;
            }
        }
        std::swap(cur, next);
    }

    int best_matched = -1;
    int best_distance = assign_inf;

    // 先最大化匹配数量，再最小化距离
    for (int mask = 0; mask < mask_limit; ++mask) {
        if (cur[mask] >= assign_inf) continue;
        int matched = bit_count_u16((uint16_t)mask);
        if (matched > best_matched || (matched == best_matched && cur[mask] < best_distance)) {
            best_matched = matched;
            best_distance = cur[mask];
        }
    }

    if (best_matched < 0) {
        best_matched = 0;
        best_distance = 0;
    }
    out_deadlocks = lvl.box_count - best_matched;

    // 选了多个炸弹后，策略更偏向真实打开通路，因此适当提高不可达惩罚的权重
    int pair_divisor = selected_task_count >= 2 ? 4 : 6;
    int unreachable_penalty = selected_task_count >= 2 ? 15 : 10;
    out_distance = best_distance + all_pair_distance / pair_divisor + unreachable_pairs * unreachable_penalty + out_deadlocks * 1000;
}


// ============================================================================
// 3. 对外入口：评估并生成炸弹任务序列
// ============================================================================
template <GameMode Mode>
/// \brief 对外入口：根据当前阶段评估并生成炸弹任务序列
/// \tparam Mode 当前比赛阶段对应的求解模式
/// \param level 当前地图快照
/// \return 建议执行的炸弹任务序列，可能为空
///
/// \details
/// 先运行快速静态 DFS；若结果仍有死锁或任务不可直接执行，再按阶段启用软障碍和局部清障回退
StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::evaluate_and_assign_bombs(const SokobanLevel& level) {
    if (level.bomb_count == 0) return StaticArray<BombTask, MAX_BOMBS>(); 

    this->cached_level = level;
    DFSResult best_res; 
    best_res.deadlocks_remaining = 9999; 
    best_res.net_profit = -999999;

    StaticArray<BombTask, MAX_BOMBS> empty_seq;
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
    auto debug_phase2_result = [](const char* label, const DFSResult& res) {
        if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
            std::fprintf(stderr, "%s dead=%d profit=%d tasks=%d\n",
                         label, res.deadlocks_remaining, res.net_profit, res.tasks.size());
            for (int i = 0; i < res.tasks.size(); ++i) {
                std::fprintf(stderr, "  #%d bomb=(%d,%d) wall=(%d,%d) pushes=%d\n",
                             i,
                             res.tasks[i].bomb_start.x, res.tasks[i].bomb_start.y,
                             res.tasks[i].target_wall.x, res.tasks[i].target_wall.y,
                             res.tasks[i].box_pushes.size());
            }
        }
    };
#endif
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        this->phase1_soft_bomb_eval = false;
    } else if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        this->phase2_soft_bomb_eval = false;
    }

    // =========================================================================
    // 阶段 1：极速静态推演（假定无需推箱子即可破局）
    // =========================================================================
    this->dfs_bomb_sequence<Mode, false>(level, level.player_start, empty_seq, 0, 0, best_res);

    // =========================================================================
    // 阶段 2：重型动态回退（如果极速推演宣告破产）
    // =========================================================================
    bool force_dynamic = false;
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        DFSResult hard_res = best_res;
        bool selected_soft_res = false;

        DFSResult soft_res;
        soft_res.deadlocks_remaining = 9999;
        soft_res.net_profit = -999999;
        this->phase1_soft_bomb_eval = true;
        this->dfs_bomb_sequence<Mode, false>(level, level.player_start, empty_seq, 0, 0, soft_res);
        this->phase1_soft_bomb_eval = false;

        bool soft_better =
            soft_res.deadlocks_remaining < hard_res.deadlocks_remaining ||
            (soft_res.deadlocks_remaining == hard_res.deadlocks_remaining &&
             soft_res.net_profit > hard_res.net_profit + PHASE1_SOFT_REPLACE_PROFIT_MARGIN);
        if (soft_better) {
            best_res = soft_res;
            selected_soft_res = true;
        } else {
            best_res = hard_res;
        }

        if (best_res.tasks.size() > 0 &&
            !this->are_fast_bomb_tasks_directly_executable(level, best_res.tasks)) {
            auto materialize_phase1_sequence = [&](StaticArray<BombTask, MAX_BOMBS>& seq, int* out_sequence_cost = nullptr) -> bool {
                SokobanLevel work = level;
                point player = level.player_start;
                int sequence_cost = 0;

                auto apply_executable = [&](const BombTask& task) -> bool {
                    StaticArray<point, MAX_PATH_LENGTH> path;
                    if (!PlanningCommon::get_bomb_push_path(work, player, task, path)) return false;
                    sequence_cost += PlanningCommon::path_time_cost(player, path);
                    if (!path.empty()) player = path.back();
                    PlanningCommon::apply_bomb_task_effect(work, task);
                    return true;
                };

                auto apply_or_materialize = [&](const BombTask& candidate, BombTask& applied_task) -> bool {
                    if (apply_executable(candidate)) {
                        applied_task = candidate;
                        return true;
                    }

                    BombTask materialized_task;
                    if (this->materialize_bomb_task(work, player, candidate, materialized_task) &&
                        apply_executable(materialized_task)) {
                        applied_task = materialized_task;
                        return true;
                    }
                    return false;
                };

                auto apply_same_bomb_hard_fallback = [&](const BombTask& soft_task, BombTask& applied_task) -> bool {
                    if (!selected_soft_res) return false;
                    for (int h = 0; h < hard_res.tasks.size(); ++h) {
                        const BombTask& hard_task = hard_res.tasks[h];
                        if (!(hard_task.bomb_start == soft_task.bomb_start)) continue;
                        if (hard_task.target_wall == soft_task.target_wall) continue;
                        if (apply_or_materialize(hard_task, applied_task)) return true;
                    }
                    return false;
                };

                for (int i = 0; i < seq.size(); ++i) {
                    BombTask task = seq[i];
                    BombTask applied_task;
                    if (apply_or_materialize(task, applied_task)) {
                        seq[i] = applied_task;
                        continue;
                    }

                    int bomb_idx = -1;
                    for (int b = 0; b < work.bomb_count; ++b) {
                        if (work.bombs[b].x != -1 && work.bombs[b] == task.bomb_start) {
                            bomb_idx = b;
                            break;
                        }
                    }
                    if (bomb_idx < 0) return false;

                    bool before_reach[MAX_BOXES][MAX_BOXES] = {};
                    bool origin_reach[MAX_BOXES][MAX_BOXES] = {};
                    bool candidate_reach[MAX_BOXES][MAX_BOXES] = {};
                    int origin_deadlocks = 0;
                    int origin_distance = 0;

                    auto eval_phase1_reach = [&](const SokobanLevel& eval_lvl, point eval_player,
                                                 bool reach[MAX_BOXES][MAX_BOXES],
                                                 int& out_deadlocks, int& out_distance) {
                        for (int b = 0; b < eval_lvl.box_count; ++b) {
                            this->fast_push_bfs(eval_lvl, eval_lvl.boxes[b], eval_player, false, dfs_dist_box[0][b], false);
                        }
                        for (int b = 0; b < eval_lvl.box_count; ++b) {
                            for (int t = 0; t < eval_lvl.target_count; ++t) {
                                point target = eval_lvl.targets[t];
                                reach[b][t] = dfs_dist_box[0][b][target.y][target.x] != INF_DIST;
                            }
                        }
                        evaluate_phase1_any_matching(eval_lvl, dfs_dist_box[0], i + 1, out_deadlocks, out_distance);
                    };

                    int before_deadlocks = 0;
                    int before_distance = 0;
                    eval_phase1_reach(work, player, before_reach, before_deadlocks, before_distance);

                    SokobanLevel origin_after = work;
                    PlanningCommon::apply_bomb_task_effect(origin_after, task);
                    eval_phase1_reach(origin_after, task.target_wall, origin_reach, origin_deadlocks, origin_distance);

                    bool found_nearby = false;
                    BombTask best_task = task;
                    int best_score = 999999;

                    for (int radius = 1; radius <= 2; ++radius) {
                        for (int dy = -radius; dy <= radius; ++dy) {
                            for (int dx = -radius; dx <= radius; ++dx) {
                                if (std::max(std::abs(dx), std::abs(dy)) != radius) continue;
                                point wall = {
                                    static_cast<int8_t>(task.target_wall.x + dx),
                                    static_cast<int8_t>(task.target_wall.y + dy)
                                };
                                if (wall.x <= 0 || wall.x >= MAP_MAX_WIDTH - 1 ||
                                    wall.y <= 0 || wall.y >= MAP_MAX_HEIGHT - 1) continue;
                                if (work.map[wall.y][wall.x] != 1) continue;

                                SokobanLevel next_lvl;
                                int real_cost = 0;
                                StaticArray<BoxPushTask, 8> pushes;
                                if (!this->local_clear_bomb_route(work, bomb_idx, wall, false, next_lvl, real_cost, pushes)) {
                                    continue;
                                }

                                BombTask candidate = task;
                                candidate.bomb_start = work.bombs[bomb_idx];
                                candidate.target_wall = wall;
                                candidate.box_pushes = pushes;

                                StaticArray<point, MAX_PATH_LENGTH> path;
                                if (!PlanningCommon::get_bomb_push_path(work, player, candidate, path)) continue;

                                SokobanLevel candidate_after = work;
                                PlanningCommon::apply_bomb_task_effect(candidate_after, candidate);
                                point candidate_player = path.empty() ? player : path.back();
                                int candidate_deadlocks = 0;
                                int candidate_distance = 0;
                                eval_phase1_reach(candidate_after, candidate_player, candidate_reach,
                                                  candidate_deadlocks, candidate_distance);

                                bool preserves_opened_pairs = true;
                                for (int b = 0; b < work.box_count; ++b) {
                                    for (int t = 0; t < work.target_count; ++t) {
                                        if (!before_reach[b][t] && origin_reach[b][t] && !candidate_reach[b][t]) {
                                            preserves_opened_pairs = false;
                                            break;
                                        }
                                    }
                                    if (!preserves_opened_pairs) break;
                                }
                                if (!preserves_opened_pairs) continue;
                                if (candidate_deadlocks > origin_deadlocks) continue;
                                if (candidate_distance > origin_distance + 20) continue;

                                int score = real_cost + radius * 100;
                                if (score < best_score) {
                                    best_score = score;
                                    best_task = candidate;
                                    found_nearby = true;
                                }
                            }
                        }
                        if (found_nearby) break;
                    }

                    if (found_nearby && apply_executable(best_task)) {
                        seq[i] = best_task;
                        continue;
                    }

                    if (apply_same_bomb_hard_fallback(task, applied_task)) {
                        seq[i] = applied_task;
                        continue;
                    }
                    return false;
                }
                if (out_sequence_cost) *out_sequence_cost = sequence_cost;
                return true;
            };

            StaticArray<BombTask, MAX_BOMBS> best_repaired_tasks;
            int best_repaired_score = 999999;
            int best_repaired_deadlocks = 9999;
            int best_repaired_distance = 999999;
            bool found_repaired_sequence = false;
            int task_count = best_res.tasks.size();
            bool used_order[MAX_BOMBS] = {false};
            StaticArray<BombTask, MAX_BOMBS> ordered_tasks;

            auto eval_phase1_sequence = [&](const StaticArray<BombTask, MAX_BOMBS>& seq,
                                            int& out_deadlocks,
                                            int& out_distance) -> bool {
                SokobanLevel work = level;
                point player = level.player_start;
                for (int task_idx = 0; task_idx < seq.size(); ++task_idx) {
                    StaticArray<point, MAX_PATH_LENGTH> path;
                    if (!PlanningCommon::get_bomb_push_path(work, player, seq[task_idx], path)) return false;
                    if (!path.empty()) player = path.back();
                    PlanningCommon::apply_bomb_task_effect(work, seq[task_idx]);
                }

                for (int b = 0; b < work.box_count; ++b) {
                    this->fast_push_bfs(work, work.boxes[b], player, false, dfs_dist_box[0][b], true);
                }
                evaluate_phase1_any_matching(work, dfs_dist_box[0], seq.size(), out_deadlocks, out_distance);
                return true;
            };

            int hard_actual_deadlocks = hard_res.deadlocks_remaining;
            int hard_actual_distance = 999999;
            eval_phase1_sequence(hard_res.tasks, hard_actual_deadlocks, hard_actual_distance);

            auto record_materialized_order = [&](const StaticArray<BombTask, MAX_BOMBS>& order) -> bool {
                StaticArray<BombTask, MAX_BOMBS> repaired = order;
                int repaired_cost = 0;
                if (!materialize_phase1_sequence(repaired, &repaired_cost)) return false;

                int repaired_deadlocks = 9999;
                int repaired_distance = 999999;
                if (!eval_phase1_sequence(repaired, repaired_deadlocks, repaired_distance)) return false;

                int push_count = 0;
                for (int i = 0; i < repaired.size(); ++i) {
                    push_count += repaired[i].box_pushes.size();
                }
                int score = repaired_cost + push_count * 20;
                if (!found_repaired_sequence ||
                    repaired_deadlocks < best_repaired_deadlocks ||
                    (repaired_deadlocks == best_repaired_deadlocks &&
                     repaired_distance < best_repaired_distance) ||
                    (repaired_deadlocks == best_repaired_deadlocks &&
                     repaired_distance == best_repaired_distance &&
                     score < best_repaired_score)) {
                    best_repaired_score = score;
                    best_repaired_deadlocks = repaired_deadlocks;
                    best_repaired_distance = repaired_distance;
                    best_repaired_tasks = repaired;
                    found_repaired_sequence = true;
                }
                return true;
            };

            auto repaired_beats_hard_now = [&]() -> bool {
                return found_repaired_sequence &&
                    (best_repaired_deadlocks < hard_actual_deadlocks ||
                     (best_repaired_deadlocks == hard_actual_deadlocks &&
                      best_repaired_distance + PHASE1_SOFT_REPLACE_PROFIT_MARGIN < hard_actual_distance));
            };

            record_materialized_order(best_res.tasks);

            auto try_materialize_order = [&](auto& self) -> void {
                if (ordered_tasks.size() == task_count) {
                    record_materialized_order(ordered_tasks);
                    return;
                }

                for (int i = 0; i < task_count; ++i) {
                    if (used_order[i]) continue;
                    used_order[i] = true;
                    ordered_tasks.push_back(best_res.tasks[i]);
                    self(self);
                    ordered_tasks.pop_back();
                    used_order[i] = false;
                }
            };

            // 软搜索输出本身已经带有收益排序。原顺序实体化后若已经把 Phase1 打通，
            // 继续枚举全排列只会重复做清障搜索，在复杂图上会把耗时放大数倍。
            if (!(repaired_beats_hard_now() && best_repaired_deadlocks == 0)) {
                try_materialize_order(try_materialize_order);
            }

            bool repaired_beats_hard =
                repaired_beats_hard_now();

            if (repaired_beats_hard) {
                best_res.tasks = best_repaired_tasks;
                best_res.deadlocks_remaining = best_repaired_deadlocks;
                best_res.net_profit = -best_repaired_distance - best_repaired_score;
            } else {
                bool found_partial_soft_sequence = false;
                StaticArray<BombTask, MAX_BOMBS> best_partial_tasks;
                int best_partial_deadlocks = 9999;
                int best_partial_distance = 999999;

                if (selected_soft_res && task_count > 1) {
                    for (int skip = 0; skip < task_count; ++skip) {
                        StaticArray<BombTask, MAX_BOMBS> partial;
                        for (int i = 0; i < task_count; ++i) {
                            if (i != skip) partial.push_back(best_res.tasks[i]);
                        }

                        int partial_cost = 0;
                        if (!materialize_phase1_sequence(partial, &partial_cost)) {
                            continue;
                        }

                        int partial_deadlocks = 9999;
                        int partial_distance = 999999;
                        if (!eval_phase1_sequence(partial, partial_deadlocks, partial_distance)) continue;

                        bool beats_hard =
                            partial_deadlocks < hard_actual_deadlocks ||
                            (partial_deadlocks == hard_actual_deadlocks &&
                             partial_distance + PHASE1_SOFT_REPLACE_PROFIT_MARGIN < hard_actual_distance);
                        if (!beats_hard) continue;

                        if (!found_partial_soft_sequence ||
                            partial_deadlocks < best_partial_deadlocks ||
                            (partial_deadlocks == best_partial_deadlocks && partial_distance < best_partial_distance)) {
                            best_partial_tasks = partial;
                            best_partial_deadlocks = partial_deadlocks;
                            best_partial_distance = partial_distance;
                            found_partial_soft_sequence = true;
                        }
                    }
                }

                if (found_partial_soft_sequence) {
                    best_res.tasks = best_partial_tasks;
                    best_res.deadlocks_remaining = best_partial_deadlocks;
                    best_res.net_profit = -best_partial_distance;
                } else {
                    if (selected_soft_res) best_res = hard_res;
                    bool need_dynamic_after_failed_materialize =
                        !selected_soft_res || hard_res.tasks.size() == 0;
                    if (need_dynamic_after_failed_materialize) {
                    DFSResult dynamic_res;
                    dynamic_res.deadlocks_remaining = 9999;
                    dynamic_res.net_profit = -999999;
                    this->dfs_bomb_sequence<Mode, true>(level, level.player_start, empty_seq, 0, 0, dynamic_res);

                    if (dynamic_res.tasks.size() > 0 &&
                        dynamic_res.deadlocks_remaining <= best_res.deadlocks_remaining) {
                        best_res = dynamic_res;
                    }
                    }
                }
            }
        }

        // 软搜索用于发现“箱子暂时挡路但理论上值得炸”的墙；但它是软评估，不能把
        // 硬搜索已经证明仍有收益、且仍可直接执行的炸弹整体吞掉。这里在软结果胜出后，
        // 只把硬搜索中尚未使用、并且按硬障碍重新评估确实改善匹配的任务补回去。
        if (selected_soft_res && best_res.tasks.size() < MAX_BOMBS) {
            auto eval_phase1_hard_pairs = [&](const SokobanLevel& eval_lvl, point eval_player,
                                             int selected_count,
                                             int& out_deadlocks, int& out_distance) {
                for (int b = 0; b < eval_lvl.box_count; ++b) {
                    this->fast_push_bfs(eval_lvl, eval_lvl.boxes[b], eval_player, false, dfs_dist_box[0][b], false);
                }
                evaluate_phase1_any_matching(eval_lvl, dfs_dist_box[0], selected_count, out_deadlocks, out_distance);
            };

            auto apply_if_executable = [&](SokobanLevel& work, point& player, const BombTask& task) -> bool {
                StaticArray<point, MAX_PATH_LENGTH> path;
                if (!PlanningCommon::get_bomb_push_path(work, player, task, path)) return false;
                if (!path.empty()) player = path.back();
                PlanningCommon::apply_bomb_task_effect(work, task);
                return true;
            };

            SokobanLevel work = level;
            point player = level.player_start;
            bool selected_sequence_ok = true;
            for (int i = 0; i < best_res.tasks.size(); ++i) {
                if (!apply_if_executable(work, player, best_res.tasks[i])) {
                    selected_sequence_ok = false;
                    break;
                }
            }

            if (selected_sequence_ok) {
                for (int i = 0; i < hard_res.tasks.size() && best_res.tasks.size() < MAX_BOMBS; ++i) {
                    const BombTask& hard_task = hard_res.tasks[i];

                    bool bomb_available = false;
                    for (int b = 0; b < work.bomb_count; ++b) {
                        if (work.bombs[b].x != -1 && work.bombs[b] == hard_task.bomb_start) {
                            bomb_available = true;
                            break;
                        }
                    }
                    if (!bomb_available) continue;

                    StaticArray<point, MAX_PATH_LENGTH> path;
                    if (!PlanningCommon::get_bomb_push_path(work, player, hard_task, path)) continue;

                    int before_deadlocks = 0, before_distance = 0;
                    int after_deadlocks = 0, after_distance = 0;
                    eval_phase1_hard_pairs(work, player, best_res.tasks.size(), before_deadlocks, before_distance);

                    SokobanLevel after = work;
                    PlanningCommon::apply_bomb_task_effect(after, hard_task);
                    point after_player = path.empty() ? player : path.back();
                    eval_phase1_hard_pairs(after, after_player, best_res.tasks.size() + 1, after_deadlocks, after_distance);

                    bool improves_hard_matching =
                        after_deadlocks < before_deadlocks ||
                        (after_deadlocks == before_deadlocks && after_distance + 40 < before_distance);
                    if (!improves_hard_matching) continue;

                    best_res.tasks.push_back(hard_task);
                    best_res.deadlocks_remaining = after_deadlocks;
                    work = after;
                    player = after_player;
                }
            }
        }
    } else if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        DFSResult hard_res = best_res;
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
        debug_phase2_result(force_phase2_dynamic ? "phase2 hard retry" : "phase2 hard", hard_res);
#endif

        DFSResult soft_res;
        soft_res.deadlocks_remaining = 9999;
        soft_res.net_profit = -999999;
        this->phase2_soft_bomb_eval = true;
        this->dfs_bomb_sequence<Mode, false>(level, level.player_start, empty_seq, 0, 0, soft_res);
        this->phase2_soft_bomb_eval = false;
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
        debug_phase2_result(force_phase2_dynamic ? "phase2 soft retry" : "phase2 soft", soft_res);
#endif

        bool soft_better =
            soft_res.deadlocks_remaining < hard_res.deadlocks_remaining ||
            (soft_res.deadlocks_remaining == hard_res.deadlocks_remaining &&
             soft_res.net_profit > hard_res.net_profit);

        if (soft_better && soft_res.tasks.size() > 0) {
            auto materialize_phase2_sequence = [&](StaticArray<BombTask, MAX_BOMBS>& seq) -> bool {
                SokobanLevel work = level;
                point player = level.player_start;

                auto apply_executable = [&](const BombTask& task) -> bool {
                    StaticArray<point, MAX_PATH_LENGTH> path;
                    if (!PlanningCommon::get_bomb_push_path(work, player, task, path)) return false;
                    if (!path.empty()) player = path.back();
                    PlanningCommon::apply_bomb_task_effect(work, task);
                    return true;
                };

                for (int i = 0; i < seq.size(); ++i) {
                    BombTask task = seq[i];
                    if (apply_executable(task)) {
                        seq[i] = task;
                        continue;
                    }

                    BombTask materialized_task;
                    if (!this->materialize_bomb_task(work, player, task, materialized_task, true)) {
                        return false;
                    }
                    if (!apply_executable(materialized_task)) return false;
                    seq[i] = materialized_task;
                }
                return true;
            };

            StaticArray<BombTask, MAX_BOMBS> materialized_tasks = soft_res.tasks;
            if (materialize_phase2_sequence(materialized_tasks)) {
                best_res = soft_res;
                best_res.tasks = materialized_tasks;
            } else {
                best_res = hard_res;
            }
        } else {
            best_res = hard_res;
        }
        force_dynamic = force_phase2_dynamic || best_res.deadlocks_remaining > 0;
    }
    if (force_dynamic) {
        // 重置最优记录
        best_res.deadlocks_remaining = 9999; 
        best_res.net_profit = -999999;
        // 启动重型推演引擎
        this->dfs_bomb_sequence<Mode, true>(level, level.player_start, empty_seq, 0, 0, best_res);
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
        if constexpr (Mode == GameMode::PHASE2_SPECIFIC) debug_phase2_result("phase2 dynamic", best_res);
#endif
    }

    for (int i = 0; i < best_res.tasks.size(); ++i) {
        best_res.tasks[i].is_essential = (best_res.deadlocks_remaining == 0);
        best_res.tasks[i].net_profit = best_res.net_profit;
    }
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        static __attribute__((section(".dtcm_bss"))) int16_t before_pair_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        static __attribute__((section(".dtcm_bss"))) int16_t origin_pair_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        static __attribute__((section(".dtcm_bss"))) int16_t candidate_pair_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

        auto eval_phase1_pairs = [&](const SokobanLevel& eval_lvl, point eval_player,
                                    int selected_count,
                                    int16_t out_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
                                    int& out_deadlocks, int& out_distance) {
            for (int b = 0; b < eval_lvl.box_count; ++b) {
                this->fast_push_bfs(eval_lvl, eval_lvl.boxes[b], eval_player, false, out_dist[b], false);
            }
            evaluate_phase1_any_matching(eval_lvl, out_dist, selected_count, out_deadlocks, out_distance);
        };

        SokobanLevel work = level;
        point player = level.player_start;
        for (int task_idx = 0; task_idx < best_res.tasks.size(); ++task_idx) {
            BombTask origin_task = best_res.tasks[task_idx];
            if (origin_task.box_pushes.size() > 0) {
                StaticArray<point, MAX_PATH_LENGTH> path;
                if (PlanningCommon::get_bomb_push_path(work, player, origin_task, path) && !path.empty()) {
                    player = path.back();
                }
                PlanningCommon::apply_bomb_task_effect(work, origin_task);
                continue;
            }

            int bomb_idx = -1;
            for (int b = 0; b < work.bomb_count; ++b) {
                if (work.bombs[b].x != -1 && work.bombs[b] == origin_task.bomb_start) {
                    bomb_idx = b;
                    break;
                }
            }
            if (bomb_idx < 0) continue;

            StaticArray<point, MAX_PATH_LENGTH> origin_path;
            if (!PlanningCommon::get_bomb_push_path(work, player, origin_task, origin_path)) {
                PlanningCommon::apply_bomb_task_effect(work, origin_task);
                player = origin_task.target_wall;
                continue;
            }

            int before_deadlocks = 0, before_distance = 0;
            int origin_deadlocks = 0, origin_distance = 0;
            eval_phase1_pairs(work, player, task_idx, before_pair_dist, before_deadlocks, before_distance);

            uint8_t before_box_degree[MAX_BOXES] = {0};
            uint8_t before_target_degree[MAX_BOXES] = {0};
            for (int b = 0; b < work.box_count; ++b) {
                for (int t = 0; t < work.target_count; ++t) {
                    point target = work.targets[t];
                    if (before_pair_dist[b][target.y][target.x] != INF_DIST) {
                        ++before_box_degree[b];
                        ++before_target_degree[t];
                    }
                }
            }
            auto unresolved_entity_pressure = [&](point wall) {
                int pressure = 0;
                for (int t = 0; t < work.target_count; ++t) {
                    int missing_boxes = work.box_count - before_target_degree[t];
                    if (missing_boxes <= 0) continue;
                    int dist = std::max(std::abs(work.targets[t].x - wall.x),
                                        std::abs(work.targets[t].y - wall.y));
                    pressure += (20 - dist) * missing_boxes * 3;
                }
                for (int b = 0; b < work.box_count; ++b) {
                    int missing_targets = work.target_count - before_box_degree[b];
                    if (missing_targets <= 0) continue;
                    int dist = std::max(std::abs(work.boxes[b].x - wall.x),
                                        std::abs(work.boxes[b].y - wall.y));
                    pressure += (20 - dist) * missing_targets;
                }
                return pressure;
            };
            int origin_pressure = unresolved_entity_pressure(origin_task.target_wall);

            SokobanLevel origin_after = work;
            PlanningCommon::apply_bomb_task_effect(origin_after, origin_task);
            point origin_player = origin_path.empty() ? player : origin_path.back();
            eval_phase1_pairs(origin_after, origin_player, task_idx + 1, origin_pair_dist, origin_deadlocks, origin_distance);

            BombTask best_task = origin_task;
            StaticArray<point, MAX_PATH_LENGTH> best_path = origin_path;
            int best_distance = origin_distance;

            int origin_wall_count = 0;
            for (int oy = origin_task.target_wall.y - 1; oy <= origin_task.target_wall.y + 1; ++oy) {
                for (int ox = origin_task.target_wall.x - 1; ox <= origin_task.target_wall.x + 1; ++ox) {
                    if (oy > 0 && oy < MAP_MAX_HEIGHT - 1 &&
                        ox > 0 && ox < MAP_MAX_WIDTH - 1 &&
                        work.map[oy][ox] == 1) {
                        ++origin_wall_count;
                    }
                }
            }

            for (int dy = -2; dy <= 2; ++dy) {
                for (int dx = -2; dx <= 2; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    point wall = {
                        static_cast<int8_t>(origin_task.target_wall.x + dx),
                        static_cast<int8_t>(origin_task.target_wall.y + dy)
                    };
                    if (wall.x <= 0 || wall.x >= MAP_MAX_WIDTH - 1 ||
                        wall.y <= 0 || wall.y >= MAP_MAX_HEIGHT - 1) {
                        continue;
                    }
                    if (work.map[wall.y][wall.x] != 1) continue;

                    BombTask candidate = origin_task;
                    candidate.target_wall = wall;
                    candidate.box_pushes.clear();

                    StaticArray<point, MAX_PATH_LENGTH> candidate_path;
                    if (!PlanningCommon::get_bomb_push_path(work, player, candidate, candidate_path)) continue;

                    SokobanLevel candidate_after = work;
                    PlanningCommon::apply_bomb_task_effect(candidate_after, candidate);
                    point candidate_player = candidate_path.empty() ? player : candidate_path.back();
                    int candidate_deadlocks = 0, candidate_distance = 0;
                    eval_phase1_pairs(candidate_after, candidate_player, task_idx + 1,
                                    candidate_pair_dist, candidate_deadlocks, candidate_distance);

                    bool preserves_opened_pairs = true;
                    int opened_pair_count = 0;
                    for (int b = 0; b < work.box_count; ++b) {
                        for (int t = 0; t < work.target_count; ++t) {
                            point target = work.targets[t];
                            bool was_blocked = before_pair_dist[b][target.y][target.x] == INF_DIST;
                            bool origin_opened = origin_pair_dist[b][target.y][target.x] != INF_DIST;
                            if (was_blocked && origin_opened) {
                                ++opened_pair_count;
                                if (candidate_pair_dist[b][target.y][target.x] == INF_DIST) {
                                    preserves_opened_pairs = false;
                                    break;
                                }
                            }
                        }
                        if (!preserves_opened_pairs) break;
                    }
                    if (!preserves_opened_pairs) continue;
                    if (unresolved_entity_pressure(wall) < origin_pressure) continue;
                    int shared_cleared_walls = 0;
                    for (int oy = origin_task.target_wall.y - 1; oy <= origin_task.target_wall.y + 1; ++oy) {
                        for (int ox = origin_task.target_wall.x - 1; ox <= origin_task.target_wall.x + 1; ++ox) {
                            if (oy <= 0 || oy >= MAP_MAX_HEIGHT - 1 ||
                                ox <= 0 || ox >= MAP_MAX_WIDTH - 1 ||
                                work.map[oy][ox] != 1) {
                                continue;
                            }
                            if (std::abs(oy - wall.y) <= 1 && std::abs(ox - wall.x) <= 1) {
                                ++shared_cleared_walls;
                            }
                        }
                    }
                    if (origin_wall_count > 0 && shared_cleared_walls * 2 < origin_wall_count) continue;
                    if (opened_pair_count == 0 && candidate_distance > origin_distance) continue;
                    if (candidate_deadlocks > origin_deadlocks) continue;
                    if (candidate_distance > origin_distance + 20) continue;

                    bool shorter_path = candidate_path.size() + 2 < best_path.size();
                    bool better_phase1_effect =
                        candidate_deadlocks < origin_deadlocks ||
                        (candidate_deadlocks == origin_deadlocks && candidate_distance + 20 < best_distance);
                    if (shorter_path || better_phase1_effect) {
                        best_task = candidate;
                        best_path = candidate_path;
                        best_distance = candidate_distance;
                    }
                }
            }

            best_task.is_essential = origin_task.is_essential;
            best_task.net_profit = origin_task.net_profit;
            best_res.tasks[task_idx] = best_task;
            if (!best_path.empty()) player = best_path.back();
            PlanningCommon::apply_bomb_task_effect(work, best_task);
        }
    }
    return best_res.tasks;
}


// ============================================================================
// 4. DFS 策略搜索：枚举候选墙体并评估收益
// ============================================================================
// 递归参数含义：
// - current_lvl：当前炸弹序列执行后的地图
// - player_start：当前玩家位置
// - current_seq：当前已选择的炸弹任务序列
// - cost_so_far：已选序列累计代价
// - depth：递归深度，同时用于复用 dfs_dist_* 缓存层
// - best_res：全局最优结果，递归过程中持续更新
template <GameMode Mode,bool Dynamic>
/// \brief DFS 枚举炸弹序列并更新全局最优结果
/// \tparam Mode 当前求解模式
/// \tparam Dynamic 是否启用动态回退评估
/// \param current_lvl 当前地图状态
/// \param player_start 当前玩家位置
/// \param current_seq 当前已经选择的炸弹任务序列
/// \param cost_so_far 当前序列累计代价
/// \param depth DFS 深度，同时用于复用距离场缓存层
/// \param best_res 全局最优结果，递归过程中被持续更新
///
/// \details
/// 该函数先评估当前地图死锁数量和箱子距离，再枚举所有可爆破墙体作为候选任务
/// 候选会按收益排序并限制分支数量，以适配 RT1064 上的时间预算
void StrategicPlanner::dfs_bomb_sequence(
    const SokobanLevel& current_lvl, point player_start,
    StaticArray<BombTask, MAX_BOMBS> current_seq, int cost_so_far, 
    int depth, DFSResult& best_res) 
{
    // =====================================================================
    // 1. 评估当前状态并更新全局最优结果
    // =====================================================================
    int current_deadlocks = 0;  // 当前状态死锁数量
    int current_distance = 0;   // 当前状态所有箱子到目标的总距离（作为成本评估的一部分）

    PlanningCommon::calc_player_reach(current_lvl, player_start, {-1,-1}, {-1,-1}, dfs_player_vis[depth]);

    // 计算每个箱子到目标的距离Phase1 后续按任意匹配评估，Phase2 仍按固定对应关系评估
    for (int b = 0; b < current_lvl.box_count; ++b) {
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, dfs_dist_box[depth][b], this->phase1_soft_bomb_eval);
        } else {
            this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, dfs_dist_box[depth][b], true);
        }

        if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
            // 第二阶段：精准评估，必须能走到专属的目标点才行！
            int t_id = current_lvl.box_ids[b]; // 获取它的专属目标
            point target = current_lvl.targets[t_id];
            if (dfs_dist_box[depth][b][target.y][target.x] == INF_DIST) {
                current_deadlocks += 10; // 定向死锁是致命的，加大惩罚
            } else {
                current_distance += dfs_dist_box[depth][b][target.y][target.x];
            }
        }
    }
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        evaluate_phase1_any_matching(current_lvl, dfs_dist_box[depth], current_seq.size(), current_deadlocks, current_distance);
    }

    if (current_seq.size() == current_lvl.bomb_count || depth >= MAX_BOMBS) {
        if (current_deadlocks > 0) return;
    }

    // 净收益评估：距离越短越好，已选炸弹越多（成本越高）越差
    // Phase1 先解决任意匹配的残局质量，避免短推炸弹压过真正打开箱-目标通路的墙
    int profit = -current_distance - cost_so_far;
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        profit = -current_distance - cost_so_far;
    }
    // 更新全局最优结果 [优先级：死锁数量（越少越好）> 净收益（越高越好）]
    bool allow_update = true;
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        allow_update = !(force_phase2_dynamic && current_seq.size() == 0 && current_lvl.bomb_count > 0);
    }
    if (allow_update &&
    (current_deadlocks < best_res.deadlocks_remaining || 
    (current_deadlocks == best_res.deadlocks_remaining && profit > best_res.net_profit))) {
        best_res.deadlocks_remaining = current_deadlocks;
        best_res.net_profit = profit;
        best_res.tasks = current_seq;
    }

    // 递归边界：如果已选炸弹数量达到上限或没有更多炸弹可选，则返回
    if (current_seq.size() == current_lvl.bomb_count || depth >= MAX_BOMBS) return;             


    // =====================================================================
    // 2. 识别 “孤岛” 实体
    // =====================================================================
    bool target_isolated[MAX_BOXES] = {false};
    bool box_isolated[MAX_BOXES] = {false};
    uint8_t phase1_box_target_degree[MAX_BOXES] = {0};
    uint8_t phase1_target_box_degree[MAX_BOXES] = {0};
    bool phase2_pair_dead[MAX_BOXES] = {false};
    int16_t phase2_pair_dist[MAX_BOXES] = {0};

    if constexpr (Mode == GameMode::PHASE1_ANY) {
        for (int b = 0; b < current_lvl.box_count; ++b) {
            for (int t = 0; t < current_lvl.target_count; ++t) {
                point target = current_lvl.targets[t];
                if (dfs_dist_box[depth][b][target.y][target.x] != INF_DIST) {
                    ++phase1_box_target_degree[b];
                    ++phase1_target_box_degree[t];
                }
            }
        }
    }

    // 抓出无解目标点
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        for (int b = 0; b < current_lvl.box_count; ++b) {
            int t_id = current_lvl.box_ids[b];
            point target = current_lvl.targets[t_id];
            phase2_pair_dist[b] = dfs_dist_box[depth][b][target.y][target.x];
            phase2_pair_dead[b] = (phase2_pair_dist[b] == INF_DIST);
        }
    }

    for (int t = 0; t < current_lvl.target_count; ++t) {
        bool can_be_reached = false;
        for (int b = 0; b < current_lvl.box_count; ++b) {
            if constexpr (Mode == GameMode::PHASE1_ANY) {
                if (dfs_dist_box[depth][b][current_lvl.targets[t].y][current_lvl.targets[t].x] != INF_DIST) { can_be_reached = true; break; }
            } else {
                if (current_lvl.box_ids[b] == t && dfs_dist_box[depth][b][current_lvl.targets[t].y][current_lvl.targets[t].x] != INF_DIST) { can_be_reached = true; break; }
            }
        }
        if (!can_be_reached) target_isolated[t] = true;
    }

    // 抓出无解箱子
    for (int b = 0; b < current_lvl.box_count; ++b) {
        bool can_reach_any = false;
        for (int t = 0; t < current_lvl.target_count; ++t) {
            if constexpr (Mode == GameMode::PHASE1_ANY) {
                if (dfs_dist_box[depth][b][current_lvl.targets[t].y][current_lvl.targets[t].x] != INF_DIST) { can_reach_any = true; break; }
            } else {
                if (current_lvl.box_ids[b] == t && dfs_dist_box[depth][b][current_lvl.targets[t].y][current_lvl.targets[t].x] != INF_DIST) { can_reach_any = true; break; }
            }
        }
        if (!can_reach_any) box_isolated[b] = true;
    }


    // =====================================================================
    // 3. 计算存活炸弹可达爆破点
    // =====================================================================
    for (int m = 0; m < current_lvl.bomb_count; ++m) {
        if (current_lvl.bombs[m].x != -1) {  
            if constexpr (!Dynamic) {
                // 【极速静态模式】：严格把箱子当死墙算距离
                if constexpr (Mode == GameMode::PHASE1_ANY) {
                    this->fast_push_bfs(current_lvl, current_lvl.bombs[m], player_start, true, dfs_dist_bomb[depth][m], this->phase1_soft_bomb_eval);
                } else {
                    this->fast_push_bfs(current_lvl, current_lvl.bombs[m], player_start, true, dfs_dist_bomb[depth][m], this->phase2_soft_bomb_eval);
                }
            } else {
                // 动态模式：先用软障碍拓扑寻找候选路线，再由局部清障函数做真实推演
                this->macro_soft_dijkstra(current_lvl, current_lvl.bombs[m], dfs_dist_bomb[depth][m]);
            }
        }
    }

    // 建立候选动作队列，避免无脑展开过多分支
    StaticArray<BombCandidate, 256>& candidates = strategy_dfs_ws.candidates[depth];
    candidates.clear();

    // =====================================================================
    // 4. 枚举可爆破墙体并评估打分
    // =====================================================================
    for (int m = 0; m < current_lvl.bomb_count; ++m) {
        if (current_lvl.bombs[m].x == -1) continue;

        // 扫描地图寻找当前炸弹可爆破的墙体
        // 备注：这些评估较为粗糙，主要是筛选可能有价值的墙壁，价值评估主要依靠上面对死锁和距离的综合评估函数
        for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
            for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                if (current_lvl.map[y][x] == 1 && dfs_dist_bomb[depth][m][y][x] != INF_DIST) {
                    
                    bool opens_new = false;  // 爆炸是否打通了玩家原本不可达的区域
                    bool touches_entity = false;  // 爆炸是否覆盖了箱子/目标/其他炸弹等实体
                    bool touches_box_or_target = false;
                    int min_shortcut = INF_DIST, max_shortcut = -1;  // 爆炸后玩家到箱子的距离变化范围（评估是否产生有价值的捷径）

                    // 扫描爆炸范围 3x3，评估 (A) 是否覆盖实体、(B) 是否打通新区域、(C) 是否产生明显捷径
                    int phase1_pair_pressure = 0;
                    int phase2_pair_pressure = 0;
                    int structural_score = 0;
                    int wall_cells_3x3 = 0;
                    bool left_open = false, right_open = false, up_open = false, down_open = false;

                    for (int dy = -2; dy <= 2; ++dy) {
                        for (int dx = -2; dx <= 2; ++dx) {
                            int ny = y + dy, nx = x + dx;
                            if (ny >= 0 && ny < MAP_MAX_HEIGHT && nx >= 0 && nx < MAP_MAX_WIDTH) {
                                if (std::abs(dy) <= 1 && std::abs(dx) <= 1 && current_lvl.map[ny][nx] == 1) ++wall_cells_3x3;
                                if (current_lvl.map[ny][nx] == 0 && std::abs(dy) <= 1 && std::abs(dx) == 2) {
                                    if (dx < 0) left_open = true;
                                    else right_open = true;
                                }
                                if (current_lvl.map[ny][nx] == 0 && std::abs(dx) <= 1 && std::abs(dy) == 2) {
                                    if (dy < 0) down_open = true;
                                    else up_open = true;
                                }
                                
                                // (A) 爆炸区是否覆盖实体
                                if (std::abs(dy) <= 1 && std::abs(dx) <= 1) {
                                    for (int b = 0; b < current_lvl.box_count; ++b) {
                                        if (current_lvl.boxes[b].x == nx && current_lvl.boxes[b].y == ny) touches_box_or_target = true;
                                    }
                                    for (int t = 0; t < current_lvl.target_count; ++t) {
                                        if (current_lvl.targets[t].x == nx && current_lvl.targets[t].y == ny) touches_box_or_target = true;
                                    }
                                    if (PlanningCommon::has_entity(current_lvl, nx, ny, m)) touches_entity = true;
                                }
                                // (B) 是否打通玩家原不可达区域
                                if (current_lvl.map[ny][nx] == 0 && !dfs_player_vis[depth][ny][nx]) {
                                    if (std::abs(dy) <= 1 && std::abs(dx) <= 1) opens_new = true;
                                    else if ((std::abs(dy) <= 1 && std::abs(dx) == 2) || (std::abs(dx) <= 1 && std::abs(dy) == 2)) opens_new = true;
                                }
                                // (C) 统计局部捷径差异
                                if (std::abs(dy) <= 1 && std::abs(dx) <= 1) {
                                    for (int b = 0; b < current_lvl.box_count; ++b) {
                                        int d = dfs_dist_box[depth][b][ny][nx];
                                        if (d != INF_DIST) {
                                            if (d < min_shortcut) min_shortcut = d;
                                            if (d > max_shortcut) max_shortcut = d;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // 找出这堵墙到最近“孤岛目标点”和“孤岛箱子”的切比雪夫距离 (因为炸弹是方形爆炸区)
                    int min_iso_target_dist = 999;
                    for (int t = 0; t < current_lvl.target_count; ++t) {
                        if (target_isolated[t]) {
                            int dist = std::max(std::abs(current_lvl.targets[t].x - x), std::abs(current_lvl.targets[t].y - y));
                            if (dist < min_iso_target_dist) min_iso_target_dist = dist;
                        }
                    }
                    int min_iso_box_dist = 999;
                    for (int b = 0; b < current_lvl.box_count; ++b) {
                        if (box_isolated[b]) {
                            int dist = std::max(std::abs(current_lvl.boxes[b].x - x), std::abs(current_lvl.boxes[b].y - y));
                            if (dist < min_iso_box_dist) min_iso_box_dist = dist;
                        }
                    }

                    int shortcut_span = 0;
                    if (min_shortcut != INF_DIST && max_shortcut >= 0) {
                        shortcut_span = max_shortcut - min_shortcut;
                    }

                    // 剪枝：无连通收益、无实体收益、无明显捷径收益
                    if constexpr (Mode == GameMode::PHASE1_ANY) {
                        structural_score += wall_cells_3x3 * 35;
                        if ((left_open && right_open) || (up_open && down_open)) structural_score += 220;
                        if ((left_open || right_open) && (up_open || down_open)) structural_score += 80;

                        int phase1_mobility_gate_score = 0;
                        auto in_blast = [&](int px, int py) {
                            return px > 0 && px < MAP_MAX_WIDTH - 1 &&
                                py > 0 && py < MAP_MAX_HEIGHT - 1 &&
                                std::abs(px - x) <= 1 && std::abs(py - y) <= 1;
                        };
                        auto floor_before = [&](point p) {
                            return p.x >= 0 && p.x < MAP_MAX_WIDTH &&
                                p.y >= 0 && p.y < MAP_MAX_HEIGHT &&
                                current_lvl.map[p.y][p.x] == 0;
                        };
                        auto floor_after = [&](point p) {
                            return floor_before(p) || in_blast(p.x, p.y);
                        };

                        // Phase1 is an any-box/any-target matching problem. A good wall often
                        // creates a new push lane or target entry, even before a full push BFS
                        // can prove the exact final pairing.
                        for (int b = 0; b < current_lvl.box_count; ++b) {
                            int missing_targets = current_lvl.target_count - phase1_box_target_degree[b];
                            if (missing_targets <= 0) continue;

                            int best_lane_score = 0;
                            for (int d = 0; d < 4; ++d) {
                                point push_from = current_lvl.boxes[b] - MOVE[d];
                                point push_to = current_lvl.boxes[b] + MOVE[d];
                                if (!floor_after(push_from) || !floor_after(push_to)) continue;
                                if (floor_before(push_from) && floor_before(push_to)) continue;

                                int lane_score = 240 + missing_targets * 80;
                                if (phase1_box_target_degree[b] == 0) lane_score += 520;
                                else if (phase1_box_target_degree[b] == 1) lane_score += 220;

                                int bdist = std::max(std::abs(current_lvl.boxes[b].x - x), std::abs(current_lvl.boxes[b].y - y));
                                if (bdist <= 2) lane_score += (3 - bdist) * 90;
                                if (lane_score > best_lane_score) best_lane_score = lane_score;
                            }
                            phase1_mobility_gate_score += best_lane_score;
                        }

                        for (int t = 0; t < current_lvl.target_count; ++t) {
                            int missing_boxes = current_lvl.box_count - phase1_target_box_degree[t];
                            if (missing_boxes <= 0) continue;

                            int best_entry_score = 0;
                            for (int d = 0; d < 4; ++d) {
                                point box_prev = current_lvl.targets[t] - MOVE[d];
                                point player_prev = current_lvl.targets[t] - MOVE[d] - MOVE[d];
                                if (!floor_after(box_prev) || !floor_after(player_prev)) continue;
                                if (floor_before(box_prev) && floor_before(player_prev)) continue;

                                int entry_score = 240 + missing_boxes * 80;
                                if (phase1_target_box_degree[t] == 0) entry_score += 560;
                                else if (phase1_target_box_degree[t] == 1) entry_score += 240;

                                int tdist = std::max(std::abs(current_lvl.targets[t].x - x), std::abs(current_lvl.targets[t].y - y));
                                if (tdist <= 2) entry_score += (3 - tdist) * 90;
                                if (entry_score > best_entry_score) best_entry_score = entry_score;
                            }
                            phase1_mobility_gate_score += best_entry_score;
                        }
                        phase1_pair_pressure += phase1_mobility_gate_score;

                        int phase1_entity_direction_score = 0;
                        point bomb_pos = current_lvl.bombs[m];
                        for (int b = 0; b < current_lvl.box_count; ++b) {
                            int missing_targets = current_lvl.target_count - phase1_box_target_degree[b];
                            if (missing_targets <= 0) continue;
                            int old_dist = std::abs(bomb_pos.x - current_lvl.boxes[b].x) +
                                        std::abs(bomb_pos.y - current_lvl.boxes[b].y);
                            int new_dist = std::abs(x - current_lvl.boxes[b].x) +
                                        std::abs(y - current_lvl.boxes[b].y);
                            int delta = old_dist - new_dist;
                            if (delta > 0) phase1_entity_direction_score += delta * missing_targets * 45;
                            else if (delta < 0) phase1_entity_direction_score += delta * 12;
                        }
                        for (int t = 0; t < current_lvl.target_count; ++t) {
                            int missing_boxes = current_lvl.box_count - phase1_target_box_degree[t];
                            if (missing_boxes <= 0) continue;
                            int old_dist = std::abs(bomb_pos.x - current_lvl.targets[t].x) +
                                        std::abs(bomb_pos.y - current_lvl.targets[t].y);
                            int new_dist = std::abs(x - current_lvl.targets[t].x) +
                                        std::abs(y - current_lvl.targets[t].y);
                            int delta = old_dist - new_dist;
                            if (delta > 0) phase1_entity_direction_score += delta * missing_boxes * 35;
                            else if (delta < 0) phase1_entity_direction_score += delta * 10;
                        }
                        phase1_pair_pressure += phase1_entity_direction_score / 6;

                        for (int t = 0; t < current_lvl.target_count; ++t) {
                            int missing_boxes = current_lvl.box_count - phase1_target_box_degree[t];
                            if (missing_boxes <= 0) continue;
                            int tdist = std::max(std::abs(current_lvl.targets[t].x - x), std::abs(current_lvl.targets[t].y - y));
                            if (tdist <= 5) {
                                phase1_pair_pressure += (6 - tdist) * missing_boxes * 45;
                                if (phase1_target_box_degree[t] == 0) phase1_pair_pressure += 260;
                                else if (phase1_target_box_degree[t] == 1) phase1_pair_pressure += 130;
                            }
                        }

                        for (int b = 0; b < current_lvl.box_count; ++b) {
                            int missing_targets = current_lvl.target_count - phase1_box_target_degree[b];
                            if (missing_targets <= 0) continue;

                            int min_blast_push = INF_DIST;
                            for (int by = y - 1; by <= y + 1; ++by) {
                                for (int bx = x - 1; bx <= x + 1; ++bx) {
                                    if (by < 0 || by >= MAP_MAX_HEIGHT || bx < 0 || bx >= MAP_MAX_WIDTH) continue;
                                    int d = dfs_dist_box[depth][b][by][bx];
                                    if (d < min_blast_push) min_blast_push = d;
                                }
                            }

                            int bdist = std::max(std::abs(current_lvl.boxes[b].x - x), std::abs(current_lvl.boxes[b].y - y));
                            if (min_blast_push != INF_DIST) phase1_pair_pressure += 80 + missing_targets * 45;
                            if (bdist <= 4) {
                                phase1_pair_pressure += (5 - bdist) * missing_targets * 35;
                                if (phase1_box_target_degree[b] == 0) phase1_pair_pressure += 260;
                                else if (phase1_box_target_degree[b] == 1) phase1_pair_pressure += 130;
                            }

                            if (min_blast_push == INF_DIST) continue;
                            for (int t = 0; t < current_lvl.target_count; ++t) {
                                point target = current_lvl.targets[t];
                                if (dfs_dist_box[depth][b][target.y][target.x] != INF_DIST) continue;
                                int tdist = std::max(std::abs(target.x - x), std::abs(target.y - y));
                                if (tdist <= 5) phase1_pair_pressure += (6 - tdist) * 90;
                            }
                        }
                    }
                    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                        structural_score += wall_cells_3x3 * 25;
                        if ((left_open && right_open) || (up_open && down_open)) structural_score += 160;

                        for (int b = 0; b < current_lvl.box_count; ++b) {
                            int t_id = current_lvl.box_ids[b];
                            point target = current_lvl.targets[t_id];
                            int tdist = std::max(std::abs(target.x - x), std::abs(target.y - y));
                            int bdist = std::max(std::abs(current_lvl.boxes[b].x - x), std::abs(current_lvl.boxes[b].y - y));

                            int min_blast_push = INF_DIST;
                            for (int by = y - 1; by <= y + 1; ++by) {
                                for (int bx = x - 1; bx <= x + 1; ++bx) {
                                    if (by < 0 || by >= MAP_MAX_HEIGHT || bx < 0 || bx >= MAP_MAX_WIDTH) continue;
                                    int d = dfs_dist_box[depth][b][by][bx];
                                    if (d < min_blast_push) min_blast_push = d;
                                }
                            }

                            int pair_score = 0;
                            if (phase2_pair_dead[b]) {
                                if (tdist <= 5) pair_score += 420 + (6 - tdist) * 130;
                                if (bdist <= 4) pair_score += 260 + (5 - bdist) * 80;
                                if (min_blast_push != INF_DIST) pair_score += 260;
                                if (min_blast_push != INF_DIST && tdist <= 5) pair_score += 650 + (6 - tdist) * 160;
                                if (target_isolated[t_id] && tdist <= 5) pair_score += 350;
                                if (box_isolated[b] && bdist <= 4) pair_score += 300;
                            } else {
                                int approx = (min_blast_push == INF_DIST) ? INF_DIST : min_blast_push + tdist;
                                if (approx + 2 < phase2_pair_dist[b]) {
                                    pair_score += (phase2_pair_dist[b] - approx) * 45;
                                }
                                if (tdist <= 3 && min_blast_push != INF_DIST) pair_score += 140;
                            }

                            bool between_x = (x >= std::min(current_lvl.boxes[b].x, target.x) - 1 &&
                                            x <= std::max(current_lvl.boxes[b].x, target.x) + 1);
                            bool between_y = (y >= std::min(current_lvl.boxes[b].y, target.y) - 1 &&
                                            y <= std::max(current_lvl.boxes[b].y, target.y) + 1);
                            if (between_x && between_y && (tdist <= 6 || bdist <= 6)) pair_score += phase2_pair_dead[b] ? 220 : 80;

                            phase2_pair_pressure += pair_score;
                        }
                    }

                    bool phase1_promising = false;
                    if constexpr (Mode == GameMode::PHASE1_ANY) {
                        phase1_promising = (phase1_pair_pressure > 0 || structural_score >= 220);
                    }
                    bool phase2_promising = false;
                    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                        phase2_promising = (phase2_pair_pressure > 0 || structural_score >= 160);
                    }
                    if (!opens_new && !touches_entity && !phase1_promising && !phase2_promising && shortcut_span <= 4 && min_iso_target_dist > 5 && min_iso_box_dist > 5) continue;

                    // 综合评估打分：优先级 = 实体覆盖 > 新区域 > 捷径提升 - 距离惩罚
                    int score = shortcut_span * 10;
                    if constexpr (Mode == GameMode::PHASE1_ANY) {
                        if (touches_box_or_target) score += 500;
                        else if (touches_entity) score += 80;
                    } else {
                        if (touches_entity) score += 500;
                    }
                    if (opens_new) score += 300;
                    if constexpr (Mode == GameMode::PHASE1_ANY) {
                        score += phase1_pair_pressure + structural_score;
                    }
                    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                        score += phase2_pair_pressure + structural_score;
                    }
                    score -= dfs_dist_bomb[depth][m][y][x] * 15; // 严厉惩罚远距离推行

                    // 孤岛解救加分：切比雪夫距离 <= 2 意味着爆炸(半径1)将直接接触该实体，或者贴在它脸上！
                    if (min_iso_target_dist <= 2) score += 600;
                    else if (min_iso_target_dist <= 4) score += 300;
                    else if (min_iso_target_dist <= 5) score += 100;
                    if (min_iso_target_dist <= 6) score += (6 - min_iso_target_dist) * 80;

                    if (min_iso_box_dist <= 2) score += 600; 
                    else if (min_iso_box_dist <= 4) score += 300;    
                    else if (min_iso_box_dist <= 5) score += 100;
                    if (min_iso_box_dist <= 6) score += (6 - min_iso_box_dist) * 60;

                    if constexpr (Mode == GameMode::PHASE1_ANY) {
                        int best_other_bomb_dist = INF_DIST;
                        for (int om = 0; om < current_lvl.bomb_count; ++om) {
                            if (om == m || current_lvl.bombs[om].x == -1) continue;
                            int od = dfs_dist_bomb[depth][om][y][x];
                            if (od < best_other_bomb_dist) best_other_bomb_dist = od;
                        }
                        int self_bomb_dist = dfs_dist_bomb[depth][m][y][x];
                        if (best_other_bomb_dist != INF_DIST && self_bomb_dist > best_other_bomb_dist + 1) {
                            score -= (self_bomb_dist - best_other_bomb_dist) * 220;
                        }
                    }

                    // 【安全防御】：防止溢出
                    if (candidates.size() < 255) {
                        candidates.push_back({(uint8_t)m, (int8_t)x, (int8_t)y, score});
                    }
                }
            }
        }
    }

    // =====================================================================
    // 5. 对候选动作进行排序并限制分支数量，进入下一层递归
    // =====================================================================

    // 对筛选出的候选墙壁进行排序
    std::sort(candidates.begin(), candidates.end());
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        for (int i = 0; i < candidates.size(); ++i) {
            int redundancy_penalty = 0;
            for (int j = 0; j < i; ++j) {
                int dx = std::abs(candidates[i].x - candidates[j].x);
                int dy = std::abs(candidates[i].y - candidates[j].y);
                if (dx == 0 && dy == 0) redundancy_penalty += 900;
                else if (dx <= 1 && dy <= 1) redundancy_penalty += 420;
            }
            candidates[i].score -= redundancy_penalty;
        }
        std::sort(candidates.begin(), candidates.end());
    }
    int selection_limit = 10;
    if constexpr (Mode == GameMode::PHASE1_ANY) selection_limit = PHASE1_SELECTION_RESTRICTIONS;
    else if constexpr (Mode == GameMode::PHASE2_SPECIFIC) selection_limit = PHASE2_SELECTION_RESTRICTIONS;
    int branch_limit = candidates.size() < selection_limit ? candidates.size() : selection_limit;

    if constexpr (!Dynamic) {
        // -----------------------------------------------------
        // 【极速模式执行路径】
        // -----------------------------------------------------
        for (int i = 0; i < branch_limit; ++i) {
            BombCandidate c = candidates[i];
            int m = c.bomb_idx;
            
            SokobanLevel next_lvl = current_lvl;
            next_lvl.bombs[m] = {-1, -1}; 
            
            // 炸墙
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int ny = c.y + dy, nx = c.x + dx;
                    if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 && nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                        next_lvl.map[ny][nx] = 0;
                    }
                }
            }

            StaticArray<BombTask, MAX_BOMBS> next_seq = current_seq;
            next_seq.push_back({current_lvl.bombs[m], { (int8_t)c.x, (int8_t)c.y }, false, 0}); // 注意：此时 box_pushes 自动为空
            
            int execution_cost = dfs_dist_bomb[depth][m][c.y][c.x] * 1.5f; 
            if constexpr (Mode == GameMode::PHASE1_ANY) {
                if (this->phase1_soft_bomb_eval && current_deadlocks > 0 && execution_cost > 10) {
                    execution_cost = 10 + (execution_cost - 10) / 4;
                }
            }
            
            // 下一层递归
            this->dfs_bomb_sequence<Mode, false>(next_lvl, { (int8_t)c.x, (int8_t)c.y }, next_seq, cost_so_far + execution_cost, depth + 1, best_res);
        }
    } 
    else {
        // -----------------------------------------------------
        // 动态执行路径：使用局部清障实际推演
        // -----------------------------------------------------
        int valid_branches = 0;
        for (int i = 0; i < candidates.size() && valid_branches < branch_limit; ++i) {
            BombCandidate c = candidates[i];
            
            SokobanLevel next_lvl;
            int real_execution_cost = 0;
            StaticArray<BoxPushTask, 8> extracted_pushes; 
            
            // 用局部清障函数提取真实推箱让路动作
            point target_wall = {(int8_t)c.x, (int8_t)c.y};
            bool is_physically_possible = this->local_clear_bomb_route(
                current_lvl, c.bomb_idx, target_wall,
                Mode == GameMode::PHASE2_SPECIFIC,
                next_lvl, real_execution_cost, extracted_pushes
            );
            
            if (!is_physically_possible) continue; 
            valid_branches++;

            next_lvl.bombs[c.bomb_idx] = {-1, -1};
            
            // 炸墙
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int ny = c.y + dy, nx = c.x + dx;
                    if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 && nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                        next_lvl.map[ny][nx] = 0; 
                    }
                }
            }

            StaticArray<BombTask, MAX_BOMBS> next_seq = current_seq;
            next_seq.push_back({
                current_lvl.bombs[c.bomb_idx], {(int8_t)c.x, (int8_t)c.y}, 
                false, 0, extracted_pushes // 将提取到的避让序列挂载到任务上
            });
            
            // 完美将残局带入下一层递归
            this->dfs_bomb_sequence<Mode, true>(next_lvl, next_lvl.player_start, next_seq, cost_so_far + real_execution_cost, depth + 1, best_res);
        }
    }
}



// ============================================================================
// 5. 推物体距离场：Fast Push-BFS
// ============================================================================
// 输入：
// - lvl：当前地图
// - start_obj：箱子或炸弹的当前位置
// - player_start：玩家当前位置，用于判断初始发力点是否可达
// - is_bomb：true 时允许把墙体作为炸弹爆破终点记录
// - soft_boxes：true 时把非目标箱子视为软障碍，给动态回退一个乐观估价
//
// 输出：
// - out_dist[y][x]：目标物体被推到 (x,y) 的估计代价
//
// 状态：
// - (物体坐标, 玩家相对物体的发力方向)
// - 直接前推、转向、掉头分别做空间可行性检查
// ============================================================================
/// \brief 计算单个箱子或炸弹被推动到各格子的估计代价
/// \param lvl 当前地图状态
/// \param start_obj 被推动物体的初始位置
/// \param player_start 玩家初始位置
/// \param is_bomb true 表示推动炸弹，允许墙体作为爆破终点被记录
/// \param out_dist 输出距离场，out_dist[y][x] 为物体到达 (x,y) 的代价
/// \param soft_boxes true 时把非目标箱子视作带惩罚的软障碍
///
/// \details
/// 搜索状态为“物体坐标 + 玩家相对物体的发力方向”
/// 函数会检查直推、转向和掉头空间，并用玩家可达性剪掉无法绕后的状态
void StrategicPlanner::fast_push_bfs(const SokobanLevel& lvl, point start_obj, point player_start, bool is_bomb, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH], bool soft_boxes) {
    
    struct QNode { int8_t x, y, dir; int16_t cost; };
    static QNode q[1024];
    int head = 0, tail = 0;
    
    // 初始化距离矩阵和状态成本矩阵
    static int16_t state_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    static uint16_t state_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    static uint16_t cur_state_gen = 0;

    cur_state_gen++;
    if (cur_state_gen == 0) { std::memset(state_gen, 0, sizeof(state_gen)); cur_state_gen = 1; }

    for(int y = 0; y < MAP_MAX_HEIGHT; y++) {
        for(int x = 0; x < MAP_MAX_WIDTH; x++) {
            out_dist[y][x] = INF_DIST;
            for(int d = 0; d < 4; d++) state_cost[y][x][d] = INF_DIST;
        }
    }

    bool soft_hard_box[MAX_BOXES] = {false};
    if (soft_boxes) mark_soft_deadlock_boxes(lvl, soft_hard_box);

    // 预先计算玩家可达性，剪枝不可达状态
    auto is_blocked = [&](point p, point ignored_obj) -> bool {
        if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
        if (lvl.map[p.y][p.x] == 1) return true;
        for (int i = 0; i < lvl.box_count; ++i) {
            if (lvl.boxes[i] == p) {
                if (p == ignored_obj) return false;
                if (soft_boxes && !(p == start_obj) && !soft_hard_box[i]) return false;
                return true;
            }
        }
        for (int i = 0; i < lvl.bomb_count; ++i) {
            if (lvl.bombs[i].x != -1 && lvl.bombs[i] == p && !(p == ignored_obj)) return true;
        }
        return false;
    };

    auto soft_penalty = [&](point p, point ignored_obj) -> int16_t {
        if (!soft_boxes || p == ignored_obj || p == start_obj) return 0;
        for (int i = 0; i < lvl.box_count; ++i) {
            if (lvl.boxes[i] == p && !soft_hard_box[i]) {
                return 10;
            }
        }
        return 0;
    };

    auto can_reach = [&](point start_pos, point target_pos, point ignored_obj, point extra_obs) -> bool {
        if (start_pos == target_pos) return true;
        static __attribute__((section(".dtcm_bss"))) uint16_t vis_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        static uint16_t cur_vis_gen = 0;
        cur_vis_gen++;
        if (cur_vis_gen == 0) { std::memset(vis_gen, 0, sizeof(vis_gen)); cur_vis_gen = 1; }

        static __attribute__((section(".dtcm_bss"))) point rq[256];
        int rh = 0, rt = 0;
        rq[rt++] = start_pos;
        vis_gen[start_pos.y][start_pos.x] = cur_vis_gen;

        while (rh < rt) {
            point curr = rq[rh++];
            for (int d = 0; d < 4; ++d) {
                point np = curr + MOVE[d];
                if (np == target_pos) return true;
                if (np == extra_obs) continue;
                if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                    if (vis_gen[np.y][np.x] != cur_vis_gen && !is_blocked(np, ignored_obj)) {
                        vis_gen[np.y][np.x] = cur_vis_gen;
                        rq[rt++] = np;
                    }
                }
            }
        }
        return false;
    };

    bool player_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    std::memset(player_vis, 0, sizeof(player_vis));
    static point pvis_q[MAP_CELL_COUNT];
    int pvis_h = 0, pvis_t = 0;
    pvis_q[pvis_t++] = player_start;
    player_vis[player_start.y][player_start.x] = true;
    while (pvis_h < pvis_t) {
        point curr = pvis_q[pvis_h++];
        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                if (!player_vis[np.y][np.x] && !is_blocked(np, {-1, -1})) {
                    player_vis[np.y][np.x] = true;
                    pvis_q[pvis_t++] = np;
                }
            }
        }
    }

    // 1) 初始化可发力站位
    for (int d = 0; d < 4; ++d) {
        point push_stand = start_obj - MOVE[d];
        if (push_stand.x >= 0 && push_stand.x < MAP_MAX_WIDTH && push_stand.y >= 0 && push_stand.y < MAP_MAX_HEIGHT) {
            if (player_vis[push_stand.y][push_stand.x]) {
                state_cost[start_obj.y][start_obj.x][d] = 0;
                q[tail++] = {start_obj.x, start_obj.y, (int8_t)d, 0};
                out_dist[start_obj.y][start_obj.x] = 0;
            }
        }
    }

    // 2) 状态扩展
    while(head < tail) {
        QNode curr = q[head++];
        point curr_p = {curr.x, curr.y};

        for (int nd = 0; nd < 4; ++nd) {
            point next_p = curr_p + MOVE[nd];

            // 先判定动力学可行性，再尝试推进
            bool can_push = false;

            // 同向推进自然可行
            if (nd == curr.dir) {
                can_push = true;
            // 转向时需要额外空间检测：内角和发力点
            } else if ((nd % 2) != (curr.dir % 2)) {
                point back = curr_p - MOVE[nd];  // 发力点坐标
                point corner = curr_p - MOVE[curr.dir] - MOVE[nd];  // 内角坐标
                
                // 内角和发力点都为空，可直接转向
                if (!is_blocked(back, start_obj) && !is_blocked(corner, start_obj)) {
                    can_push = true;
                } else if (!is_blocked(back, start_obj)) {
                    // 内角受阻时，回退到可达性检测
                    point player_current_pos = curr_p - MOVE[curr.dir]; 
                    if (back.x >= 0 && back.x < MAP_MAX_WIDTH && back.y >= 0 && back.y < MAP_MAX_HEIGHT) {
                        if (can_reach(player_current_pos, back, start_obj, curr_p)) {
                            can_push = true;
                        }
                    }
                }
            } else {
                // 横向掉头需要更严格的空间检测：发力点必须可站立
                point push_stand = curr_p - MOVE[nd]; 
                if (is_blocked(push_stand, start_obj)) {
                    continue; // 发力点是墙或箱子，绝对不可能掉头
                }

                // 掉头时要求任一侧 U 形三格通道可通行
                point side1_mid = curr_p + MOVE[(curr.dir+1)%4];
                point side1_back = side1_mid - MOVE[curr.dir]; 
                point side1_front = side1_mid + MOVE[curr.dir];
                
                point side2_mid = curr_p + MOVE[(curr.dir+3)%4];
                point side2_back = side2_mid - MOVE[curr.dir]; 
                point side2_front = side2_mid + MOVE[curr.dir];

                bool can_route1 = !is_blocked(side1_back, start_obj) && 
                                !is_blocked(side1_mid, start_obj) && 
                                !is_blocked(side1_front, start_obj);
                                
                bool can_route2 = !is_blocked(side2_back, start_obj) && 
                                !is_blocked(side2_mid, start_obj) && 
                                !is_blocked(side2_front, start_obj);
                                
                if (can_route1 || can_route2) {
                    can_push = true;
                }
            }

            // 发力位不可达，直接剪枝
            if (!can_push) continue; 

            // 可推进时再检测落点是否可用
            if (is_blocked(next_p, start_obj)) {
                // 仅炸弹撞墙时记录为可爆破墙体
                if (is_bomb && next_p.x >= 0 && next_p.x < MAP_MAX_WIDTH && next_p.y >= 0 && next_p.y < MAP_MAX_HEIGHT && lvl.map[next_p.y][next_p.x] == 1) {
                    if (curr.cost + 1 < out_dist[next_p.y][next_p.x]) {
                        out_dist[next_p.y][next_p.x] = curr.cost + 1;
                    }
                }
                continue;  // 炸弹使用后不再进入状态队列
            }

            // 前方为空，继续入队
            int16_t ncost = curr.cost + 1 + soft_penalty(next_p, start_obj);
            // 使用世代计数器判断该状态是否已走过
            if (state_gen[next_p.y][next_p.x][nd] != cur_state_gen || ncost < state_cost[next_p.y][next_p.x][nd]) {
                state_gen[next_p.y][next_p.x][nd] = cur_state_gen;
                state_cost[next_p.y][next_p.x][nd] = ncost;
                if (ncost < out_dist[next_p.y][next_p.x]) out_dist[next_p.y][next_p.x] = ncost;
                q[tail++] = {next_p.x, next_p.y, (int8_t)nd, ncost};
            }
        }
    }
}



// ============================================================================
// 6. 软障碍宏观拓扑评估
// ============================================================================
// 用途：
// - 在动态回退阶段快速估计炸弹穿过箱子区域的可能性
// - 只评估炸弹拓扑连通性，不精确模拟玩家站位
// - 箱子作为软障碍加入惩罚，后续再由 local_clear_bomb_route 验证真实可执行性
/// \brief 对炸弹移动做软障碍拓扑估价
/// \param lvl 当前地图状态
/// \param start_obj 炸弹初始位置
/// \param out_dist 输出距离场，墙体可作为终点但不可穿过
///
/// \details
/// 该函数不精确模拟玩家站位，只评估炸弹拓扑连通性
/// 箱子被视为软障碍并加入惩罚，用于判断是否值得进一步调用局部清障生成推箱让路任务
void StrategicPlanner::macro_soft_dijkstra(const SokobanLevel& lvl, point start_obj, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    // 使用 SPFA/Dijkstra 变体，放在极速区
    static __attribute__((section(".dtcm_bss"))) point q[2048];
    uint32_t head = 0, tail = 0;
    
    for(int y = 0; y < MAP_MAX_HEIGHT; y++) {
        for(int x = 0; x < MAP_MAX_WIDTH; x++) {
            out_dist[y][x] = INF_DIST;
        }
    }
    
    out_dist[start_obj.y][start_obj.x] = 0;
    q[tail & 2047] = start_obj; 
    tail++;

    bool soft_hard_box[MAX_BOXES] = {false};
    mark_soft_deadlock_boxes(lvl, soft_hard_box);
    
    while(head < tail) {
        point curr = q[head & 2047]; 
        head++;
        int16_t ccost = out_dist[curr.y][curr.x];
        
        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (np.x < 0 || np.x >= MAP_MAX_WIDTH || np.y < 0 || np.y >= MAP_MAX_HEIGHT) continue;
            
            // 墙壁：可以作为终点触碰，但不能穿过
            if (lvl.map[np.y][np.x] == 1) {
                if (ccost + 1 < out_dist[np.y][np.x]) out_dist[np.y][np.x] = ccost + 1;
                continue; 
            }
            
            // 其他未引爆的炸弹：视为绝对硬障碍
            bool is_other_bomb = false;
            for(int i = 0; i < lvl.bomb_count; i++) {
                if (lvl.bombs[i] == np && lvl.bombs[i] != start_obj && lvl.bombs[i].x != -1) is_other_bomb = true;
            }
            if (is_other_bomb) continue;
            
            // 箱子：视为软障碍（引发推箱子避让的惩罚）
            bool is_box = false;
            bool hard_box_cell = false;
            for(int i = 0; i < lvl.box_count; i++) {
                if (lvl.boxes[i] == np) {
                    if (soft_hard_box[i]) {
                        hard_box_cell = true;
                        break;
                    }
                    is_box = true;
                }
            }
            if (hard_box_cell) continue;
            
            // 箱子代价 +10，空地 +1；这是“可能需要推箱让路”的软惩罚
            int16_t ncost = ccost + (is_box ? 10 : 1);
            if (ncost < out_dist[np.y][np.x]) {
                out_dist[np.y][np.x] = ncost;
                q[tail & 2047] = np; 
                tail++;
            }
        }
    }
}

// ============================================================================
// 7. 局部清障可执行性推演
// ============================================================================

/// \brief 用局部清障补全炸弹任务的推箱让路序列
/// \param start_lvl 起始地图状态
/// \param bomb_idx 要推动的炸弹编号
/// \param target_wall 目标爆破墙体
/// \param phase2_specific true 表示按第二阶段固定箱-目标关系做安全检查
/// \param out_lvl 输出完成清障后的地图状态，尚未应用爆炸效果
/// \param out_cost 输出清障和推炸弹的估计代价
/// \param out_box_pushes 输出为了让路而生成的推箱子子任务
/// \return 找到可执行清障和推炸弹方案时返回 true
///
/// \details
/// 流程是软障碍路线找阻挡箱子，再把阻挡箱子推到离路线较近但不占路线的安全格
/// 每次推箱都调用 PlanningCommon::append_box_push_path 做真实可执行验证
/// 若推某个箱子前还需要挪开其他箱子，会做最多 1 层递归清障，避免退化为全局联合状态搜索
bool StrategicPlanner::local_clear_bomb_route(
    const SokobanLevel& start_lvl,
    int bomb_idx,
    point target_wall,
    bool phase2_specific,
    SokobanLevel& out_lvl,
    int& out_cost,
    StaticArray<BoxPushTask, 8>& out_box_pushes)
{
    if (bomb_idx < 0 || bomb_idx >= start_lvl.bomb_count) return false;
    if (start_lvl.bombs[bomb_idx].x == -1) return false;
    if (target_wall.x < 0 || target_wall.x >= MAP_MAX_WIDTH ||
        target_wall.y < 0 || target_wall.y >= MAP_MAX_HEIGHT) return false;
    SokobanLevel work = start_lvl;
    point player = start_lvl.player_start;
    out_box_pushes.clear();
    out_cost = 0;

    static int16_t soft_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static int16_t box_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static point route[MAP_CELL_COUNT];
    static bool route_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

    auto cheb = [](point a, point b) -> int {
        int dx = std::abs(a.x - b.x);
        int dy = std::abs(a.y - b.y);
        return dx > dy ? dx : dy;
    };

    auto is_target_for_box = [&](int box_id, point p) -> bool {
        if (phase2_specific) {
            int tid = work.box_ids[box_id];
            return tid >= 0 && tid < work.target_count && work.targets[tid] == p;
        }
        for (int t = 0; t < work.target_count; ++t) {
            if (work.targets[t] == p) return true;
        }
        return false;
    };

    auto is_any_target = [&](point p) -> bool {
        for (int t = 0; t < work.target_count; ++t) {
            if (work.targets[t] == p) return true;
        }
        return false;
    };

    auto is_cell_free_for_box_target = [&](point p, int moving_box) -> bool {
        if (!PlanningCommon::in_bounds(p)) return false;
        if (work.map[p.y][p.x] == 1) return false;
        if (p == target_wall) return false;

        for (int b = 0; b < work.box_count; ++b) {
            if (b != moving_box && work.boxes[b] == p) return false;
        }
        for (int b = 0; b < work.bomb_count; ++b) {
            if (work.bombs[b].x != -1 && work.bombs[b] == p) return false;
        }
        return true;
    };

    auto is_static_corner = [&](const SokobanLevel& lvl, point p) -> bool {
        auto solid = [&](point q) {
            if (!PlanningCommon::in_bounds(q)) return true;
            return lvl.map[q.y][q.x] == 1;
        };
        bool up = solid(p + MOVE[0]);
        bool right = solid(p + MOVE[1]);
        bool down = solid(p + MOVE[2]);
        bool left = solid(p + MOVE[3]);
        return (up && right) || (right && down) || (down && left) || (left && up);
    };

    auto bomb_can_rescue_box = [&](const SokobanLevel& lvl, int box_id, point box_pos) -> bool {
        point goal = {-1, -1};
        if (phase2_specific) {
            int tid = lvl.box_ids[box_id];
            if (tid >= 0 && tid < lvl.target_count) goal = lvl.targets[tid];
        }

        if (cheb(target_wall, box_pos) <= 2) return true;
        if (goal.x != -1 && cheb(target_wall, goal) <= 2) return true;

        for (int b = 0; b < lvl.bomb_count; ++b) {
            if (b == bomb_idx || lvl.bombs[b].x == -1) continue;
            this->macro_soft_dijkstra(lvl, lvl.bombs[b], soft_dist);
            for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
                for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                    if (lvl.map[y][x] != 1 || soft_dist[y][x] == INF_DIST) continue;
                    point wall = {(int8_t)x, (int8_t)y};
                    if (cheb(wall, box_pos) <= 2) return true;
                    if (goal.x != -1 && cheb(wall, goal) <= 2) return true;
                }
            }
        }
        return false;
    };

    auto box_position_is_safe = [&](const SokobanLevel& lvl, int box_id, point check_player) -> bool {
        point box_pos = lvl.boxes[box_id];
        bool on_valid_target = false;
        if (phase2_specific) {
            int tid = lvl.box_ids[box_id];
            on_valid_target = tid >= 0 && tid < lvl.target_count && lvl.targets[tid] == box_pos;
        } else {
            for (int t = 0; t < lvl.target_count; ++t) {
                if (lvl.targets[t] == box_pos) on_valid_target = true;
            }
        }

        this->fast_push_bfs(lvl, box_pos, check_player, false, box_dist, true);
        bool can_reach_goal = false;
        if (phase2_specific) {
            int tid = lvl.box_ids[box_id];
            if (tid >= 0 && tid < lvl.target_count) {
                point goal = lvl.targets[tid];
                can_reach_goal = box_dist[goal.y][goal.x] != INF_DIST;
            }
        } else {
            for (int t = 0; t < lvl.target_count; ++t) {
                point goal = lvl.targets[t];
                if (box_dist[goal.y][goal.x] != INF_DIST) {
                    can_reach_goal = true;
                    break;
                }
            }
        }

        if (can_reach_goal || on_valid_target) return true;
        if (!is_static_corner(lvl, box_pos)) return bomb_can_rescue_box(lvl, box_id, box_pos);
        return bomb_can_rescue_box(lvl, box_id, box_pos);
    };

    auto build_soft_route = [&]() -> int {
        std::memset(route_mask, 0, sizeof(route_mask));
        this->macro_soft_dijkstra(work, work.bombs[bomb_idx], soft_dist);
        if (soft_dist[target_wall.y][target_wall.x] == INF_DIST) return 0;

        point rev_route[MAP_CELL_COUNT];
        int rev_len = 0;
        point cur = target_wall;
        rev_route[rev_len++] = cur;

        while (!(cur == work.bombs[bomb_idx]) && rev_len < MAP_CELL_COUNT) {
            int best_d = soft_dist[cur.y][cur.x];
            point best_p = {-1, -1};
            for (int d = 0; d < 4; ++d) {
                point np = cur + MOVE[d];
                if (!PlanningCommon::in_bounds(np)) continue;
                int nd = soft_dist[np.y][np.x];
                if (nd < best_d) {
                    best_d = nd;
                    best_p = np;
                }
            }
            if (best_p.x == -1) return 0;
            cur = best_p;
            rev_route[rev_len++] = cur;
        }

        int route_len = 0;
        for (int i = rev_len - 1; i >= 0; --i) {
            route[route_len++] = rev_route[i];
            route_mask[rev_route[i].y][rev_route[i].x] = true;
        }
        return route_len;
    };

    auto find_box_at = [&](point p) -> int {
        for (int b = 0; b < work.box_count; ++b) {
            if (work.boxes[b] == p) return b;
        }
        return -1;
    };

    auto check_direct_bomb_path = [&](StaticArray<point, MAX_PATH_LENGTH>& path) -> bool {
        BombTask probe;
        probe.bomb_start = work.bombs[bomb_idx];
        probe.target_wall = target_wall;
        probe.is_essential = false;
        probe.net_profit = 0;
        probe.box_pushes.clear();
        return PlanningCommon::get_bomb_push_path(work, player, probe, path);
    };

    bool clearing_stack[MAX_BOXES] = {false};

    auto clear_box_recursive = [&](auto& self, int box_id, int depth) -> bool {
        if (box_id < 0 || box_id >= work.box_count) return false;
        if (clearing_stack[box_id]) return false;
        if (out_box_pushes.size() >= LOCAL_CLEAR_MAX_TASKS) return false;

        struct ClearCandidate {
            point p;
            int score;
            bool opens_bomb_path;
        };

        ClearCandidate candidates[MAP_CELL_COUNT];
        int candidate_count = 0;
        point box_start = work.boxes[box_id];
        clearing_stack[box_id] = true;

        auto nearest_goal_distance = [&](point p) -> int {
            int best = 99;
            if (phase2_specific) {
                int tid = work.box_ids[box_id];
                if (tid >= 0 && tid < work.target_count) {
                    return std::abs(p.x - work.targets[tid].x) + std::abs(p.y - work.targets[tid].y);
                }
            }
            for (int t = 0; t < work.target_count; ++t) {
                int d = std::abs(p.x - work.targets[t].x) + std::abs(p.y - work.targets[t].y);
                if (d < best) best = d;
            }
            return best;
        };

        auto first_push_access_cost = [&](point target) -> int {
            point push_dir = {0, 0};
            if (target.x == box_start.x && target.y != box_start.y) {
                push_dir.y = (target.y > box_start.y) ? 1 : -1;
            } else if (target.y == box_start.y && target.x != box_start.x) {
                push_dir.x = (target.x > box_start.x) ? 1 : -1;
            } else {
                return 8;
            }

            point push_from = {
                static_cast<int8_t>(box_start.x - push_dir.x),
                static_cast<int8_t>(box_start.y - push_dir.y)
            };
            if (!PlanningCommon::in_bounds(push_from)) return 80;
            if (work.map[push_from.y][push_from.x] == 1) return 80;

            bool occupied = false;
            for (int b = 0; b < work.box_count; ++b) {
                if (b != box_id && work.boxes[b] == push_from) occupied = true;
            }
            for (int b = 0; b < work.bomb_count; ++b) {
                if (work.bombs[b].x != -1 && work.bombs[b] == push_from) occupied = true;
            }
            if (occupied) return 80;

            uint16_t access = PlanningCommon::bfs_shortest_path(work, player, push_from);
            return access == 65535 ? 80 : access;
        };

        auto candidate_open_cost = [&](point target) -> int {
            SokobanLevel probe_lvl = work;
            point probe_player = player;
            StaticArray<point, MAX_PATH_LENGTH> push_path;
            BoxPushTask push_task{box_start, target};
            if (!PlanningCommon::append_box_push_path(probe_lvl, probe_player, push_task, push_path)) return 9999;

            BombTask probe_bomb;
            probe_bomb.bomb_start = probe_lvl.bombs[bomb_idx];
            probe_bomb.target_wall = target_wall;
            probe_bomb.is_essential = false;
            probe_bomb.net_profit = 0;
            probe_bomb.box_pushes.clear();

            StaticArray<point, MAX_PATH_LENGTH> bomb_path;
            if (!PlanningCommon::get_bomb_push_path(probe_lvl, probe_player, probe_bomb, bomb_path)) return 9999;
            return push_path.size() + bomb_path.size();
        };

        for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
            for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                point p = {(int8_t)x, (int8_t)y};
                if (!is_cell_free_for_box_target(p, box_id)) continue;

                bool any_target = is_any_target(p);
                bool valid_goal = is_target_for_box(box_id, p);
                if (any_target && !valid_goal) continue;

                int dist_box = std::abs(p.x - box_start.x) + std::abs(p.y - box_start.y);
                if (dist_box == 0) continue;
                if (p.x != box_start.x && p.y != box_start.y) continue;
                int min_route_dist = 99;
                for (int ry = 0; ry < MAP_MAX_HEIGHT; ++ry) {
                    for (int rx = 0; rx < MAP_MAX_WIDTH; ++rx) {
                        if (!route_mask[ry][rx]) continue;
                        int rd = std::abs(p.x - rx) + std::abs(p.y - ry);
                        if (rd < min_route_dist) min_route_dist = rd;
                    }
                }

                int open_cost = candidate_open_cost(p);
                bool opens_bomb_path = open_cost < 9999;
                int score = opens_bomb_path ? open_cost * 6 : 250;
                score += dist_box * 8;
                score += first_push_access_cost(p) * 6;
                if (phase2_specific) {
                    score += (nearest_goal_distance(p) - nearest_goal_distance(box_start)) * 80;
                }
                if (!opens_bomb_path) {
                    if (min_route_dist <= 1) score += 80;
                    else if (min_route_dist == 2) score += 20;
                }
                if (is_static_corner(work, p) && !valid_goal) score += 120;
                if (phase2_specific && valid_goal) score -= 60;

                candidates[candidate_count++] = {p, score, opens_bomb_path};
            }
        }

        for (int i = 0; i < candidate_count - 1; ++i) {
            for (int j = 0; j < candidate_count - 1 - i; ++j) {
                if (candidates[j].score > candidates[j + 1].score) {
                    ClearCandidate tmp = candidates[j];
                    candidates[j] = candidates[j + 1];
                    candidates[j + 1] = tmp;
                }
            }
        }

        int try_limit = candidate_count < LOCAL_CLEAR_CANDIDATE_LIMIT ? candidate_count : LOCAL_CLEAR_CANDIDATE_LIMIT;
        for (int i = 0; i < try_limit; ++i) {
            BoxPushTask task{box_start, candidates[i].p};

            SokobanLevel saved_level = work;
            point saved_player = player;
            StaticArray<BoxPushTask, 8> saved_pushes = out_box_pushes;
            int saved_cost = out_cost;

            StaticArray<point, MAX_PATH_LENGTH> segment;
            if (PlanningCommon::append_box_push_path(work, player, task, segment) &&
                (box_position_is_safe(work, box_id, player) || candidates[i].opens_bomb_path)) {
                out_box_pushes.push_back(task);
                out_cost += segment.size();
                clearing_stack[box_id] = false;
                return true;
            }

            work = saved_level;
            player = saved_player;
            out_box_pushes = saved_pushes;
            out_cost = saved_cost;

            if (depth >= LOCAL_CLEAR_CHAIN_DEPTH) continue;

            for (int other = 0; other < work.box_count; ++other) {
                if (other == box_id || clearing_stack[other]) continue;
                int near_start = std::abs(work.boxes[other].x - box_start.x) + std::abs(work.boxes[other].y - box_start.y);
                int near_target = std::abs(work.boxes[other].x - candidates[i].p.x) + std::abs(work.boxes[other].y - candidates[i].p.y);
                if (near_start > 3 && near_target > 3) continue;

                saved_level = work;
                saved_player = player;
                saved_pushes = out_box_pushes;
                saved_cost = out_cost;

                if (self(self, other, depth + 1)) {
                    segment.clear();
                    if (PlanningCommon::append_box_push_path(work, player, task, segment) &&
                        box_position_is_safe(work, box_id, player)) {
                        out_box_pushes.push_back(task);
                        out_cost += segment.size();
                        clearing_stack[box_id] = false;
                        return true;
                    }
                }

                work = saved_level;
                player = saved_player;
                out_box_pushes = saved_pushes;
                out_cost = saved_cost;
            }
        }

        clearing_stack[box_id] = false;
        return false;
    };

    for (int iter = 0; iter < LOCAL_CLEAR_MAX_ITER; ++iter) {
        StaticArray<point, MAX_PATH_LENGTH> direct_path;
        if (check_direct_bomb_path(direct_path)) {
            out_lvl = work;
            if (!direct_path.empty()) out_lvl.player_start = direct_path.back();
            else out_lvl.player_start = player;
            out_cost += direct_path.size();
            return true;
        }

        int route_len = build_soft_route();
        if (route_len <= 0) return false;

        int blockers[MAX_BOXES];
        int blocker_count = 0;
        for (int i = 0; i < route_len; ++i) {
            int bid = find_box_at(route[i]);
            if (bid < 0) continue;
            bool seen = false;
            for (int j = 0; j < blocker_count; ++j) {
                if (blockers[j] == bid) seen = true;
            }
            if (!seen && blocker_count < MAX_BOXES) blockers[blocker_count++] = bid;
        }

        if (blocker_count == 0) {
            for (int i = 0; i < route_len - 1; ++i) {
                point curr = route[i];
                point next = route[i + 1];
                point delta = {
                    static_cast<int8_t>(next.x - curr.x),
                    static_cast<int8_t>(next.y - curr.y)
                };
                point push_stand = {
                    static_cast<int8_t>(curr.x - delta.x),
                    static_cast<int8_t>(curr.y - delta.y)
                };
                for (int b = 0; b < work.box_count && blocker_count < MAX_BOXES; ++b) {
                    int md = std::abs(work.boxes[b].x - push_stand.x) + std::abs(work.boxes[b].y - push_stand.y);
                    if (md > 3) continue;
                    bool seen = false;
                    for (int j = 0; j < blocker_count; ++j) {
                        if (blockers[j] == b) seen = true;
                    }
                    if (!seen) blockers[blocker_count++] = b;
                }
            }
        }

        if (blocker_count == 0) {
            for (int b = 0; b < work.box_count && blocker_count < MAX_BOXES; ++b) {
                for (int i = 0; i < route_len; ++i) {
                    int md = std::abs(work.boxes[b].x - route[i].x) + std::abs(work.boxes[b].y - route[i].y);
                    if (md == 1) {
                        blockers[blocker_count++] = b;
                        break;
                    }
                }
            }
        }
        bool cleared = false;
        for (int i = 0; i < blocker_count; ++i) {
            if (clear_box_recursive(clear_box_recursive, blockers[i], 0)) {
                cleared = true;
                break;
            }
        }
        if (!cleared) {
            break;
        }
    }

    // 路线规则失败后，做一个小规模真实清障 DFS
    // 该分支不保存联合状态池，只枚举少量已经能真实执行的单箱推移
    struct RealClearCandidate {
        BoxPushTask task;
        int box_id;
        int path_len;
        int score;
    };

    auto real_clear_search = [&](auto& self,
                                SokobanLevel lvl,
                                point cur_player,
                                StaticArray<BoxPushTask, 8> pushes,
                                int cost,
                                int depth) -> bool {
        BombTask probe;
        probe.bomb_start = lvl.bombs[bomb_idx];
        probe.target_wall = target_wall;
        probe.is_essential = false;
        probe.net_profit = 0;
        probe.box_pushes.clear();

        StaticArray<point, MAX_PATH_LENGTH> bomb_path;
        if (PlanningCommon::get_bomb_push_path(lvl, cur_player, probe, bomb_path)) {
            out_lvl = lvl;
            out_lvl.player_start = bomb_path.empty() ? cur_player : bomb_path.back();
            out_box_pushes = pushes;
            out_cost = cost + bomb_path.size();
            return true;
        }
        if (depth >= 3 || pushes.size() >= LOCAL_CLEAR_MAX_TASKS) return false;

        bool support_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        std::memset(support_mask, 0, sizeof(support_mask));
        this->macro_soft_dijkstra(lvl, lvl.bombs[bomb_idx], soft_dist);
        if (soft_dist[target_wall.y][target_wall.x] != INF_DIST) {
            point cur = target_wall;
            for (int guard = 0; guard < MAP_CELL_COUNT && !(cur == lvl.bombs[bomb_idx]); ++guard) {
                support_mask[cur.y][cur.x] = true;
                int best_d = soft_dist[cur.y][cur.x];
                point best_p = {-1, -1};
                for (int d = 0; d < 4; ++d) {
                    point np = cur + MOVE[d];
                    if (!PlanningCommon::in_bounds(np)) continue;
                    int nd = soft_dist[np.y][np.x];
                    if (nd < best_d) {
                        best_d = nd;
                        best_p = np;
                    }
                }
                if (best_p.x == -1) break;
                point delta = {
                    static_cast<int8_t>(cur.x - best_p.x),
                    static_cast<int8_t>(cur.y - best_p.y)
                };
                point push_stand = {
                    static_cast<int8_t>(best_p.x - delta.x),
                    static_cast<int8_t>(best_p.y - delta.y)
                };
                if (PlanningCommon::in_bounds(push_stand)) support_mask[push_stand.y][push_stand.x] = true;
                cur = best_p;
            }
            support_mask[lvl.bombs[bomb_idx].y][lvl.bombs[bomb_idx].x] = true;
        }

        RealClearCandidate candidates[48];
        int candidate_count = 0;

        auto occupied_without_box = [&](point p, int moving_box) -> bool {
            for (int b = 0; b < lvl.box_count; ++b) {
                if (b != moving_box && lvl.boxes[b] == p) return true;
            }
            for (int b = 0; b < lvl.bomb_count; ++b) {
                if (lvl.bombs[b].x != -1 && lvl.bombs[b] == p) return true;
            }
            return false;
        };

        for (int b = 0; b < lvl.box_count; ++b) {
            int source_support_dist = 99;
            for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
                for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                    if (!support_mask[y][x]) continue;
                    int d = std::abs(lvl.boxes[b].x - x) + std::abs(lvl.boxes[b].y - y);
                    if (d < source_support_dist) source_support_dist = d;
                }
            }
            if (source_support_dist > 5) continue;

            for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
                for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                    point target = {(int8_t)x, (int8_t)y};
                    int move_dist = std::abs(target.x - lvl.boxes[b].x) + std::abs(target.y - lvl.boxes[b].y);
                    if (move_dist == 0 || move_dist > 4) continue;
                    if (target.x != lvl.boxes[b].x && target.y != lvl.boxes[b].y) continue;
                    if (lvl.map[y][x] == 1 || occupied_without_box(target, b)) continue;
                    if (target == target_wall) continue;

                    bool any_target = false;
                    for (int t = 0; t < lvl.target_count; ++t) {
                        if (lvl.targets[t] == target) any_target = true;
                    }
                    if (any_target && !is_target_for_box(b, target)) continue;

                    SokobanLevel next_lvl = lvl;
                    point next_player = cur_player;
                    StaticArray<point, MAX_PATH_LENGTH> segment;
                    BoxPushTask task{lvl.boxes[b], target};
                    if (!PlanningCommon::append_box_push_path(next_lvl, next_player, task, segment)) continue;

                    BombTask probe_after_push;
                    probe_after_push.bomb_start = next_lvl.bombs[bomb_idx];
                    probe_after_push.target_wall = target_wall;
                    probe_after_push.is_essential = false;
                    probe_after_push.net_profit = 0;
                    probe_after_push.box_pushes.clear();
                    StaticArray<point, MAX_PATH_LENGTH> bomb_after_push;
                    bool opens_bomb_path = PlanningCommon::get_bomb_push_path(
                        next_lvl, next_player, probe_after_push, bomb_after_push
                    );
                    if (!opens_bomb_path && !box_position_is_safe(next_lvl, b, next_player)) continue;

                    int target_support_dist = 99;
                    for (int sy = 0; sy < MAP_MAX_HEIGHT; ++sy) {
                        for (int sx = 0; sx < MAP_MAX_WIDTH; ++sx) {
                            if (!support_mask[sy][sx]) continue;
                            int d = std::abs(target.x - sx) + std::abs(target.y - sy);
                            if (d < target_support_dist) target_support_dist = d;
                        }
                    }

                    int score = source_support_dist * 30 + move_dist * 8;
                    if (opens_bomb_path) score += (segment.size() + bomb_after_push.size()) * 6;
                    else score += 250 + segment.size() * 4;
                    auto nearest_goal_dist_for_real = [&](point p) -> int {
                        int best = 99;
                        if (phase2_specific) {
                            int tid = lvl.box_ids[b];
                            if (tid >= 0 && tid < lvl.target_count) {
                                return std::abs(p.x - lvl.targets[tid].x) + std::abs(p.y - lvl.targets[tid].y);
                            }
                        }
                        for (int t = 0; t < lvl.target_count; ++t) {
                            int gd = std::abs(p.x - lvl.targets[t].x) + std::abs(p.y - lvl.targets[t].y);
                            if (gd < best) best = gd;
                        }
                        return best;
                    };
                    if (phase2_specific && (target.x == lvl.boxes[b].x || target.y == lvl.boxes[b].y)) {
                        score += (nearest_goal_dist_for_real(target) - nearest_goal_dist_for_real(lvl.boxes[b])) * 200;
                    } else if (target.x != lvl.boxes[b].x && target.y != lvl.boxes[b].y) {
                        score += 120;
                    }
                    if (!opens_bomb_path) {
                        if (target_support_dist <= 1) score += 120;
                        else if (target_support_dist == 2) score += 40;
                    }
                    if (is_static_corner(next_lvl, target) && !is_target_for_box(b, target)) score += 160;

                    if (candidate_count < 48) {
                        candidates[candidate_count++] = {task, b, segment.size(), score};
                    }
                }
            }
        }

        for (int i = 0; i < candidate_count - 1; ++i) {
            for (int j = 0; j < candidate_count - 1 - i; ++j) {
                if (candidates[j].score > candidates[j + 1].score) {
                    RealClearCandidate tmp = candidates[j];
                    candidates[j] = candidates[j + 1];
                    candidates[j + 1] = tmp;
                }
            }
        }

        int try_limit = candidate_count < 14 ? candidate_count : 14;
        for (int i = 0; i < try_limit; ++i) {
            SokobanLevel next_lvl = lvl;
            point next_player = cur_player;
            StaticArray<point, MAX_PATH_LENGTH> segment;
            if (!PlanningCommon::append_box_push_path(next_lvl, next_player, candidates[i].task, segment)) continue;

            StaticArray<BoxPushTask, 8> next_pushes = pushes;
            next_pushes.push_back(candidates[i].task);
            if (self(self, next_lvl, next_player, next_pushes, cost + segment.size(), depth + 1)) {
                return true;
            }
        }

        return false;
    };

    {
        StaticArray<BoxPushTask, 8> pushes;
        if (real_clear_search(real_clear_search, start_lvl, start_lvl.player_start, pushes, 0, 0)) {
            return true;
        }
    }

    return false;
}

// ============================================================================
// 9. 炸弹任务实体化与快速可执行性验证
// ============================================================================

// 将策略层的候选炸弹任务补全为可执行任务，必要时生成推箱让路序列
/// \brief 将候选炸弹任务补全为真实可执行任务
/// \param level 当前地图状态
/// \param player_start 当前玩家位置
/// \param task 策略层生成的候选炸弹任务
/// \param out_task 输出补全后的任务，可能包含 box_pushes
/// \return 成功生成真实可执行任务时返回 true
bool StrategicPlanner::materialize_bomb_task(const SokobanLevel& level, point player_start, const BombTask& task, BombTask& out_task, bool phase2_specific) {
    SokobanLevel temp = level;
    temp.player_start = player_start;

    int bomb_idx = -1;
    for (int b = 0; b < temp.bomb_count; ++b) {
        if (temp.bombs[b].x != -1 && temp.bombs[b] == task.bomb_start) {
            bomb_idx = b;
            break;
        }
    }
    if (bomb_idx < 0) return false;

    SokobanLevel next_lvl;
    int real_cost = 0;
    StaticArray<BoxPushTask, 8> pushes;
    bool ok = this->local_clear_bomb_route(
        temp, bomb_idx, task.target_wall, phase2_specific,
        next_lvl, real_cost, pushes
    );
    if (!ok) return false;

    out_task = task;
    out_task.box_pushes = pushes;
    return true;
}

// 快速验证一串炸弹任务是否能在不推箱让路的情况下直接执行
/// \brief 快速检查炸弹任务序列是否无需推箱让路即可直接执行
/// \param level 当前地图状态
/// \param tasks 待验证的炸弹任务序列
/// \return 全部任务都能直接执行时返回 true
///
/// \details
/// 该函数使用 fast_push_bfs 逐个验证炸弹到目标墙体的可达性，
/// 并在每个任务后应用爆炸效果，模拟任务序列的真实执行顺序
bool StrategicPlanner::are_fast_bomb_tasks_directly_executable(const SokobanLevel& level, const StaticArray<BombTask, MAX_BOMBS>& tasks) {
    SokobanLevel temp = level;
    point player = level.player_start;
    static __attribute__((section(".dtcm_bss"))) int16_t direct_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

    for (int i = 0; i < tasks.size(); ++i) {
        const BombTask& task = tasks[i];
        if (task.target_wall.x < 0 || task.target_wall.x >= MAP_MAX_WIDTH ||
            task.target_wall.y < 0 || task.target_wall.y >= MAP_MAX_HEIGHT) {
            return false;
        }
        if (temp.map[task.target_wall.y][task.target_wall.x] != 1) {
            return false;
        }

        int bomb_idx = -1;
        for (int b = 0; b < temp.bomb_count; ++b) {
            if (temp.bombs[b].x != -1 && temp.bombs[b] == task.bomb_start) {
                bomb_idx = b;
                break;
            }
        }
        if (bomb_idx < 0) return false;

        this->fast_push_bfs(temp, task.bomb_start, player, true, direct_dist, false);
        if (direct_dist[task.target_wall.y][task.target_wall.x] == INF_DIST) {
            return false;
        }

        PlanningCommon::apply_bomb_task_effect(temp, task);
        player = task.target_wall;
    }
    return true;
}

// ============================================================================
// 10. 模板显式实例化
// ============================================================================

template StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::evaluate_and_assign_bombs<GameMode::PHASE1_ANY>(const SokobanLevel&);
template StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::evaluate_and_assign_bombs<GameMode::PHASE2_SPECIFIC>(const SokobanLevel&);
template void StrategicPlanner::dfs_bomb_sequence<GameMode::PHASE1_ANY, false>(const SokobanLevel&, point, StaticArray<BombTask, MAX_BOMBS>, int, int, DFSResult&);
template void StrategicPlanner::dfs_bomb_sequence<GameMode::PHASE2_SPECIFIC, false>(const SokobanLevel&, point, StaticArray<BombTask, MAX_BOMBS>, int, int, DFSResult&);
template void StrategicPlanner::dfs_bomb_sequence<GameMode::PHASE1_ANY, true>(const SokobanLevel&, point, StaticArray<BombTask, MAX_BOMBS>, int, int, DFSResult&);
template void StrategicPlanner::dfs_bomb_sequence<GameMode::PHASE2_SPECIFIC, true>(const SokobanLevel&, point, StaticArray<BombTask, MAX_BOMBS>, int, int, DFSResult&);
