#include "Strategy.h"
#include <cstring>
#include <algorithm>

using namespace StrategyConfig;

static StrategySearchWorkspace& strategy_ws = strategy_search_workspace();
// ============================================================================
// 1. Phase2 logic-wall scoring helpers.
// ============================================================================


static bool strategy_is_wall(const SokobanLevel& lvl, point p) {
    if (!PlanningCommon::in_bounds(p)) return true;
    return lvl.map[p.y][p.x] == 1;
}

static bool strategy_blast_footprint_cell(point center, point p) {
    return std::abs(center.x - p.x) <= 1 && std::abs(center.y - p.y) <= 1;
}

static bool strategy_blast_covers(point center, point p) {
    return strategy_blast_footprint_cell(center, p);
}

static void logic_clear_scores(LogicBlastScores& scores) {
    std::memset(&scores, 0, sizeof(scores));
}

static void logic_add_score(LogicBlastScores& scores, point center, int add, uint8_t layer) {
    if (!PlanningCommon::in_bounds(center)) return;
    int v = scores.score[center.y][center.x] + add;
    if (v > 30000) v = 30000;
    if (v < -30000) v = -30000;
    scores.score[center.y][center.x] = static_cast<int16_t>(v);
    if (layer == 1 && scores.l1_hits[center.y][center.x] < 255) ++scores.l1_hits[center.y][center.x];
    if (layer == 2 && scores.l2_hits[center.y][center.x] < 255) ++scores.l2_hits[center.y][center.x];
    if (layer == 3 && scores.l3_hits[center.y][center.x] < 255) ++scores.l3_hits[center.y][center.x];
    if (layer == 4 && scores.bomb_unlock_hits[center.y][center.x] < 255) ++scores.bomb_unlock_hits[center.y][center.x];
}

static void logic_add_wall_requirement(
    const SokobanLevel& lvl,
    LogicBlastScores& scores,
    const point* required_walls,
    int required_count,
    int add,
    uint8_t layer) {
    if (required_count <= 0) return;
    for (int i = 0; i < required_count; ++i) {
        if (!PlanningCommon::is_blastable_wall(lvl, required_walls[i])) return;
    }
    point anchor = required_walls[0];
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int cx = anchor.x + dx;
            int cy = anchor.y + dy;
            if (cx <= 0 || cx >= MAP_MAX_WIDTH - 1 || cy <= 0 || cy >= MAP_MAX_HEIGHT - 1) continue;
            point center = {static_cast<int8_t>(cx), static_cast<int8_t>(cy)};
            if (!PlanningCommon::is_blastable_wall(lvl, center)) continue;
            bool covers_all = true;
            for (int i = 0; i < required_count; ++i) {
                if (!strategy_blast_covers(center, required_walls[i])) {
                    covers_all = false;
                    break;
                }
            }
            if (!covers_all) continue;
            logic_add_score(scores, center, add, layer);
        }
    }
}

static bool logic_edge_missing_walls(const SokobanLevel& lvl, point box_pos, int dir, point* out_walls, int& out_count) {
    out_count = 0;
    point box_to = box_pos + MOVE[dir];
    point push_from = box_pos - MOVE[dir];
    if (!PlanningCommon::in_bounds(box_to) || !PlanningCommon::in_bounds(push_from)) return false;
    if (strategy_is_wall(lvl, box_to)) out_walls[out_count++] = box_to;
    if (strategy_is_wall(lvl, push_from)) out_walls[out_count++] = push_from;
    return out_count > 0;
}

static void logic_build_reverse_push_reach(
    const SokobanLevel& lvl,
    point target,
    int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) out_dist[y][x] = INF_DIST;
    }
    if (!PlanningCommon::in_bounds(target) || strategy_is_wall(lvl, target)) return;

    OCRAM_BSS static point q[MAP_CELL_COUNT];
    int head = 0, tail = 0;
    out_dist[target.y][target.x] = 0;
    q[tail++] = target;

    while (head < tail) {
        point curr = q[head++];
        int16_t curr_dist = out_dist[curr.y][curr.x];
        for (int dir = 0; dir < 4; ++dir) {
            point box_prev = curr - MOVE[dir];
            point player_prev = curr - MOVE[dir] - MOVE[dir];
            if (!PlanningCommon::in_bounds(box_prev) || !PlanningCommon::in_bounds(player_prev)) continue;
            if (strategy_is_wall(lvl, box_prev) || strategy_is_wall(lvl, player_prev)) continue;
            if (out_dist[box_prev.y][box_prev.x] != INF_DIST) continue;
            out_dist[box_prev.y][box_prev.x] = curr_dist + 1;
            q[tail++] = box_prev;
        }
    }
}

static void build_logic_blast_scores(
    const SokobanLevel& lvl,
    bool player_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    int16_t box_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    LogicBlastScores& scores) {
    logic_clear_scores(scores);

    bool hard_box_deadlock[MAX_BOXES] = {false};
    mark_soft_deadlock_boxes(lvl, hard_box_deadlock);

    // L1: static box locks. Score walls that block the first useful push.
    for (int b = 0; b < lvl.box_count; ++b) {
        point box = lvl.boxes[b];
        bool on_target = strategy_is_goal_for_box(lvl, b, box, true);

        int static_legal_push_dirs = 0;
        for (int d = 0; d < 4; ++d) {
            point box_to = box + MOVE[d];
            point push_from = box - MOVE[d];
            if (!PlanningCommon::in_bounds(box_to) || !PlanningCommon::in_bounds(push_from)) continue;
            if (strategy_is_wall(lvl, box_to) || strategy_is_wall(lvl, push_from)) continue;
            ++static_legal_push_dirs;
        }

        bool wall_up = strategy_is_wall(lvl, box + MOVE[0]);
        bool wall_right = strategy_is_wall(lvl, box + MOVE[1]);
        bool wall_down = strategy_is_wall(lvl, box + MOVE[2]);
        bool wall_left = strategy_is_wall(lvl, box + MOVE[3]);
        bool static_corner = (wall_up && wall_right) || (wall_right && wall_down) ||
                            (wall_down && wall_left) || (wall_left && wall_up);
        bool hard_deadlock = hard_box_deadlock[b];
        if (on_target || (!hard_deadlock && !static_corner && static_legal_push_dirs > 0)) continue;

        for (int d = 0; d < 4; ++d) {
            point required[2];
            int required_count = 0;
            if (!logic_edge_missing_walls(lvl, box, d, required, required_count)) continue;
            logic_add_wall_requirement(lvl, scores, required, required_count,
                                    hard_deadlock ? 7200 : (static_corner ? 5200 : 3600), 1);
        }
    }

    // L1b: 2x2 or multi-box wall locks. Emit the long wall edge itself.
    for (int y = 0; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH - 1; ++x) {
            int box_id[2][2];
            bool wall[2][2];
            int box_count = 0;
            int wall_count = 0;
            point wall_cells[4];

            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    point p = {static_cast<int8_t>(x + dx), static_cast<int8_t>(y + dy)};
                    box_id[dy][dx] = strategy_box_at(lvl, p);
                    wall[dy][dx] = strategy_is_wall(lvl, p);
                    if (box_id[dy][dx] >= 0) ++box_count;
                    if (wall[dy][dx]) wall_cells[wall_count++] = p;
                }
            }

            bool boxes_top = box_id[0][0] >= 0 && box_id[0][1] >= 0;
            bool boxes_bottom = box_id[1][0] >= 0 && box_id[1][1] >= 0;
            bool boxes_left = box_id[0][0] >= 0 && box_id[1][0] >= 0;
            bool boxes_right = box_id[0][1] >= 0 && box_id[1][1] >= 0;
            bool walls_top = wall[0][0] && wall[0][1];
            bool walls_bottom = wall[1][0] && wall[1][1];
            bool walls_left = wall[0][0] && wall[1][0];
            bool walls_right = wall[0][1] && wall[1][1];

            bool long_edge_lock =
                (box_count >= 3 && wall_count >= 1) ||
                (boxes_top && walls_bottom) ||
                (boxes_bottom && walls_top) ||
                (boxes_left && walls_right) ||
                (boxes_right && walls_left);
            if (!long_edge_lock) continue;

            for (int i = 0; i < wall_count; ++i) {
                logic_add_wall_requirement(lvl, scores, &wall_cells[i], 1, 6800, 1);
            }
        }
    }

    // L2: player connectivity. Score walls between reachable and unreachable floor.
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (lvl.map[y][x] != 1) continue;
            point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
            bool touches_reachable_floor = false;
            bool touches_unreachable_floor = false;
            for (int d = 0; d < 4; ++d) {
                point np = wall + MOVE[d];
                if (!PlanningCommon::in_bounds(np) || lvl.map[np.y][np.x] == 1) continue;
                if (player_vis[np.y][np.x]) touches_reachable_floor = true;
                else touches_unreachable_floor = true;
            }
            if (!touches_reachable_floor || !touches_unreachable_floor) continue;

            int entity_bonus = 0;
            bool unlocks_bomb = false;
            for (int b = 0; b < lvl.box_count; ++b) {
                int dist = std::max(std::abs(lvl.boxes[b].x - x), std::abs(lvl.boxes[b].y - y));
                if (dist <= 3) entity_bonus += (4 - dist) * 180;
            }
            for (int t = 0; t < lvl.target_count; ++t) {
                int dist = std::max(std::abs(lvl.targets[t].x - x), std::abs(lvl.targets[t].y - y));
                if (dist <= 3) entity_bonus += (4 - dist) * 160;
            }
            for (int b = 0; b < strategy_bomb_count(lvl); ++b) {
                if (lvl.bombs[b].x == -1) continue;
                int dist = std::max(std::abs(lvl.bombs[b].x - x), std::abs(lvl.bombs[b].y - y));
                if (dist <= 3) {
                    entity_bonus += (4 - dist) * 220;
                    unlocks_bomb = true;
                }
            }

            point required[1] = {wall};
            logic_add_wall_requirement(lvl, scores, required, 1, 1300 + entity_bonus, 2);
            if (unlocks_bomb) logic_add_wall_requirement(lvl, scores, required, 1, 900, 4);
        }
    }

    // L3: bridge forward box reachability F to reverse target reachability R.
    // If one push edge from F to R is blocked only by walls, score those walls.
    OCRAM_BSS static int16_t reverse_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    bool target_needed[MAX_BOXES] = {false};
    for (int b = 0; b < lvl.box_count; ++b) {
        for (int t = 0; t < lvl.target_count; ++t) {
            if (!strategy_target_allowed_for_box(lvl, b, t, true)) continue;
            point target = lvl.targets[t];
            if (box_dist[b][target.y][target.x] == INF_DIST) target_needed[t] = true;
        }
    }
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!target_needed[t]) continue;
        logic_build_reverse_push_reach(lvl, lvl.targets[t], reverse_dist[t]);
    }

    // L3a: strongly sealed target region. Emit boundary walls of R.
    // This catches cases where simple one-edge bridge detection has no contact.
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!target_needed[t]) continue;

        bool target_has_box_contact = false;
        for (int b = 0; b < lvl.box_count && !target_has_box_contact; ++b) {
            if (!strategy_target_allowed_for_box(lvl, b, t, true)) continue;
            for (int y = 1; y < MAP_MAX_HEIGHT - 1 && !target_has_box_contact; ++y) {
                for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                    if (box_dist[b][y][x] != INF_DIST && reverse_dist[t][y][x] != INF_DIST) {
                        target_has_box_contact = true;
                        break;
                    }
                }
            }
        }
        if (target_has_box_contact) continue;

        int emitted = 0;
        point target = lvl.targets[t];
        for (int y = 1; y < MAP_MAX_HEIGHT - 1 && emitted < 16; ++y) {
            for (int x = 1; x < MAP_MAX_WIDTH - 1 && emitted < 16; ++x) {
                if (reverse_dist[t][y][x] == INF_DIST) continue;
                point cell = {static_cast<int8_t>(x), static_cast<int8_t>(y)};

                for (int d = 0; d < 4 && emitted < 16; ++d) {
                    point wall = cell + MOVE[d];
                    point outside = wall + MOVE[d];
                    if (!PlanningCommon::in_bounds(wall) || !PlanningCommon::in_bounds(outside)) continue;
                    if (!strategy_is_wall(lvl, wall) || strategy_is_wall(lvl, outside)) continue;

                    int dist = std::max(std::abs(target.x - wall.x), std::abs(target.y - wall.y));
                    int add = 6200;
                    if (dist <= 4) add += (5 - dist) * 420;
                    logic_add_wall_requirement(lvl, scores, &wall, 1, add, 3);
                    ++emitted;
                }
            }
        }
    }

    // This catches cases where simple one-edge bridge detection has no contact.
    // Opening F boundaries creates entrances for later F/R bridges.
    for (int b = 0; b < lvl.box_count; ++b) {
        bool box_has_reachable_target = false;
        bool box_has_applicable_target = false;
        for (int t = 0; t < lvl.target_count; ++t) {
            if (!strategy_target_allowed_for_box(lvl, b, t, true)) continue;
            box_has_applicable_target = true;
            point target = lvl.targets[t];
            if (box_dist[b][target.y][target.x] != INF_DIST) {
                box_has_reachable_target = true;
                break;
            }
        }
        if (!box_has_applicable_target || box_has_reachable_target) continue;

        bool box_has_target_contact = false;
        for (int t = 0; t < lvl.target_count && !box_has_target_contact; ++t) {
            if (!strategy_target_allowed_for_box(lvl, b, t, true)) continue;
            if (!target_needed[t]) continue;
            for (int y = 1; y < MAP_MAX_HEIGHT - 1 && !box_has_target_contact; ++y) {
                for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                    if (box_dist[b][y][x] != INF_DIST && reverse_dist[t][y][x] != INF_DIST) {
                        box_has_target_contact = true;
                        break;
                    }
                }
            }
        }
        if (box_has_target_contact) continue;

        int emitted = 0;
        point box = lvl.boxes[b];
        for (int y = 1; y < MAP_MAX_HEIGHT - 1 && emitted < 16; ++y) {
            for (int x = 1; x < MAP_MAX_WIDTH - 1 && emitted < 16; ++x) {
                if (box_dist[b][y][x] == INF_DIST) continue;
                point cell = {static_cast<int8_t>(x), static_cast<int8_t>(y)};

                for (int d = 0; d < 4 && emitted < 16; ++d) {
                    point wall = cell + MOVE[d];
                    point outside = wall + MOVE[d];
                    if (!PlanningCommon::in_bounds(wall) || !PlanningCommon::in_bounds(outside)) continue;
                    if (!strategy_is_wall(lvl, wall) || strategy_is_wall(lvl, outside)) continue;

                    int dist = std::max(std::abs(box.x - wall.x), std::abs(box.y - wall.y));
                    int add = 5600;
                    if (dist <= 4) add += (5 - dist) * 360;
                    logic_add_wall_requirement(lvl, scores, &wall, 1, add, 3);
                    ++emitted;
                }
            }
        }
    }

    for (int b = 0; b < lvl.box_count; ++b) {
        for (int t = 0; t < lvl.target_count; ++t) {
            if (!strategy_target_allowed_for_box(lvl, b, t, true)) continue;
            if (!target_needed[t]) continue;
            point target = lvl.targets[t];
            if (box_dist[b][target.y][target.x] != INF_DIST) continue;

            int emitted_for_pair = 0;
            for (int y = 1; y < MAP_MAX_HEIGHT - 1 && emitted_for_pair < 8; ++y) {
                for (int x = 1; x < MAP_MAX_WIDTH - 1 && emitted_for_pair < 8; ++x) {
                    if (box_dist[b][y][x] == INF_DIST) continue;
                    point from = {static_cast<int8_t>(x), static_cast<int8_t>(y)};

                    for (int d = 0; d < 4 && emitted_for_pair < 8; ++d) {
                        point to = from + MOVE[d];
                        point push_from = from - MOVE[d];
                        if (!PlanningCommon::in_bounds(to) || !PlanningCommon::in_bounds(push_from)) continue;
                        if (reverse_dist[t][to.y][to.x] == INF_DIST) continue;

                        point required[2];
                        int required_count = 0;
                        if (strategy_is_wall(lvl, to)) required[required_count++] = to;
                        if (strategy_is_wall(lvl, push_from)) required[required_count++] = push_from;
                        if (required_count == 0) continue;

                        int bridge_bonus = 4200;
                        int target_dist = std::max(std::abs(target.x - to.x), std::abs(target.y - to.y));
                        if (target_dist <= 5) bridge_bonus += (6 - target_dist) * 260;
                        logic_add_wall_requirement(lvl, scores, required, required_count, bridge_bonus, 3);
                        ++emitted_for_pair;
                    }
                }
            }
        }
    }

}

static inline int bit_count_u16(uint16_t mask) {
    int c = 0;
    while (mask) {
        mask = static_cast<uint16_t>(mask & (mask - 1));
        ++c;
    }
    return c;
}

template <typename MatchAllowed>
static void evaluate_matching_dp_core(
    const SokobanLevel& lvl,
    int16_t box_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    MatchAllowed match_allowed,
    int& out_best_matched,
    int& out_best_distance)
{
    const int assign_inf = 999999;
    int mask_limit = 1 << lvl.target_count;
    for (int mask = 0; mask < mask_limit; ++mask) strategy_ws.matching_dp[mask] = assign_inf;
    strategy_ws.matching_dp[0] = 0;

    int* cur = strategy_ws.matching_dp;
    int* next = strategy_ws.matching_next;

    // DP state: cur[mask] is the minimum push distance after using target mask.
    for (int b = 0; b < lvl.box_count; ++b) {
        for (int mask = 0; mask < mask_limit; ++mask) next[mask] = cur[mask];

        for (int mask = 0; mask < mask_limit; ++mask) {
            if (cur[mask] >= assign_inf) continue;
            for (int t = 0; t < lvl.target_count; ++t) {
                if (mask & (1 << t)) continue;
                if (!match_allowed(b, t)) continue;

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

    out_best_matched = -1;
    out_best_distance = assign_inf;
    for (int mask = 0; mask < mask_limit; ++mask) {
        if (cur[mask] >= assign_inf) continue;
        int matched = bit_count_u16(static_cast<uint16_t>(mask));
        if (matched > out_best_matched ||
            (matched == out_best_matched && cur[mask] < out_best_distance)) {
            out_best_matched = matched;
            out_best_distance = cur[mask];
        }
    }

    if (out_best_matched < 0) {
        out_best_matched = 0;
        out_best_distance = 0;
    }
}

// Evaluate Phase2 semantic matching quality.
// Repeated semantics cannot be checked by nearest-target only: two boxes may
// both prefer the same target. The shared bitmask DP assigns each box to one
// unused reachable target with the same semantic id.
static void evaluate_phase2_semantic_matching(
    const SokobanLevel& lvl,
    int16_t box_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    int& out_deadlocks,
    int& out_distance)
{
    if (lvl.box_count == 0) {
        out_deadlocks = 0;
        out_distance = 0;
        return;
    }

    int best_matched = 0;
    int best_distance = 0;
    evaluate_matching_dp_core(
        lvl,
        box_dist,
        [&](int box_id, int target_id) {
            return strategy_target_allowed_for_box(lvl, box_id, target_id, true);
        },
        best_matched,
        best_distance
    );
    out_deadlocks = (lvl.box_count - best_matched) * 10;
    out_distance = best_distance;
}


StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::plan_phase2_bombs(
    const SokobanLevel& level,
    const StaticArray<BombTask, MAX_BOMBS>& inherited_tasks)
{
    this->begin_profile_eval(1);
    this->cached_level = level;
    this->phase2_soft_bomb_eval = false;

    auto phase2_clear_push_count = [](const DFSResult& result) -> int {
        int pushes = 0;
        for (int i = 0; i < result.tasks.size(); ++i) {
            pushes += result.tasks[i].box_pushes.size();
        }
        return pushes;
    };

    auto phase2_push_distance_score = [](const DFSResult& result) -> int {
        int distance = -result.net_profit - result.bomb_supply_score;
        return distance < 0 ? 0 : distance;
    };

    auto phase2_result_better_than = [&](const DFSResult& candidate, const DFSResult& baseline) -> bool {
        if (candidate.deadlocks_remaining != baseline.deadlocks_remaining) {
            return candidate.deadlocks_remaining < baseline.deadlocks_remaining;
        }
        if (candidate.deadlocks_remaining == 0) {
            int candidate_clear_pushes = phase2_clear_push_count(candidate);
            int baseline_clear_pushes = phase2_clear_push_count(baseline);
            if (candidate_clear_pushes != baseline_clear_pushes) {
                // 语义已经全可达时，带推箱清障的炸弹只算路径优化候选
                // 不能为估计距离牺牲后续 Sokoban 搜索稳定性
                return candidate_clear_pushes < baseline_clear_pushes;
            }
            int candidate_distance = phase2_push_distance_score(candidate);
            int baseline_distance = phase2_push_distance_score(baseline);
            if (candidate_distance != baseline_distance) {
                // 可达关系已闭合后，Phase2 的剩余炸弹目标是缩短后续推箱距离
                return candidate_distance < baseline_distance;
            }
            if (candidate.bomb_supply_score != baseline.bomb_supply_score) {
                return candidate.bomb_supply_score < baseline.bomb_supply_score;
            }
        }
        if (candidate.net_profit != baseline.net_profit) {
            return candidate.net_profit > baseline.net_profit;
        }
        return candidate.tasks.size() < baseline.tasks.size();
    };

    auto build_phase2_empty_result = [&](const SokobanLevel& search_level) -> DFSResult {
        DFSResult empty_res;
        empty_res.tasks.clear();
        this->evaluate_phase2_level_matching(
            search_level,
            search_level.player_start,
            empty_res.deadlocks_remaining,
            empty_res.net_profit);
        empty_res.net_profit = -empty_res.net_profit;
        empty_res.unreachable_pairs_remaining = empty_res.deadlocks_remaining;
        empty_res.bomb_supply_score = 0;
        return empty_res;
    };

    auto materialize_and_score_phase2_result =
        [&](const SokobanLevel& search_level, DFSResult& result) -> bool {
            if (result.tasks.size() == 0) {
                result = build_phase2_empty_result(search_level);
                return true;
            }

            StaticArray<BombTask, MAX_BOMBS> materialized_tasks = result.tasks;
            int deadlocks = 9999;
            int distance = 999999;
            int sequence_cost = 0;
            if (!this->materialize_phase2_sequence(search_level, materialized_tasks) ||
                !this->evaluate_phase2_task_sequence(
                    search_level,
                    materialized_tasks,
                    deadlocks,
                    distance,
                    &sequence_cost)) {
                return false;
            }

            result.tasks = materialized_tasks;
            result.deadlocks_remaining = deadlocks;
            result.net_profit = -distance - sequence_cost;
            result.unreachable_pairs_remaining = deadlocks;
            result.bomb_supply_score = sequence_cost;
            return true;
        };

    auto run_phase2_search = [&](const SokobanLevel& search_level,
                                 DFSResult& best_res,
                                 uint8_t& selected_profile_pass) {
        selected_profile_pass = 0;
        this->phase2_soft_bomb_eval = false;
        best_res = build_phase2_empty_result(search_level);

        // hard pass 同时负责解除语义不可达，以及在已全可达时寻找缩短总推箱距离的炸弹
        DFSResult raw_hard_res;
        this->execute_phase2_search_pass(search_level, 0, raw_hard_res);

        DFSResult hard_res = raw_hard_res;
        bool hard_ok = materialize_and_score_phase2_result(search_level, hard_res);
        if (hard_ok && phase2_result_better_than(hard_res, best_res)) {
            best_res = hard_res;
            selected_profile_pass = 0;
        }

        DFSResult soft_res;
        soft_res.deadlocks_remaining = 9999;
        soft_res.net_profit = -999999;

        bool run_soft_pass = best_res.deadlocks_remaining > 0;
        if (run_soft_pass) {
            this->phase2_soft_bomb_eval = true;
            this->execute_phase2_search_pass(search_level, 1, soft_res);
            this->phase2_soft_bomb_eval = false;
        }

        if (run_soft_pass &&
            materialize_and_score_phase2_result(search_level, soft_res) &&
            phase2_result_better_than(soft_res, best_res)) {
            best_res = soft_res;
            selected_profile_pass = 1;
        }
    };

    StaticArray<BombTask, MAX_BOMBS> inherited = inherited_tasks;
    if (inherited.size() == 0 && this->phase1_phase2_inherited_candidates.size() > 0) {
        inherited = this->phase1_phase2_inherited_candidates;
    }
    SokobanLevel inherited_level = level;
    point inherited_player = level.player_start;
    int inherited_cost = 0;
    int inherited_deadlocks = 9999;
    int inherited_distance = 999999;
    bool inherited_valid = false;
    DFSResult inherited_res;

    if (inherited.size() > 0) {
        inherited_valid =
            this->materialize_phase2_sequence(level, inherited) &&
            this->apply_phase2_task_sequence(
                level,
                inherited,
                inherited_level,
                inherited_player,
                &inherited_cost);
        if (inherited_valid) {
            this->evaluate_phase2_level_matching(
                inherited_level,
                inherited_player,
                inherited_deadlocks,
                inherited_distance);

            inherited_res.tasks = inherited;
            inherited_res.deadlocks_remaining = inherited_deadlocks;
            inherited_res.net_profit = -inherited_distance - inherited_cost;
            inherited_res.unreachable_pairs_remaining = inherited_deadlocks;
            inherited_res.bomb_supply_score = inherited_cost;
            this->record_profile_result(2, inherited_res);

            // Phase2 继承 Phase1 剩余任务；后续 suffix 只在继承后的地图上追加
            // 只有继承任务已解除语义不可达时才固定为前缀
        }
    }

    DFSResult final_res;
    uint8_t selected_profile_pass = 0;
    run_phase2_search(level, final_res, selected_profile_pass);

    if (inherited_valid) {
        if (phase2_result_better_than(inherited_res, final_res)) {
            final_res = inherited_res;
            selected_profile_pass = 2;
        }

        SokobanLevel inherited_search_level = inherited_level;
        inherited_search_level.player_start = inherited_player;
        DFSResult suffix_res;
        uint8_t suffix_profile_pass = 0;
        run_phase2_search(inherited_search_level, suffix_res, suffix_profile_pass);

        bool used_original_bomb[MAX_BOMBS] = {false};
        for (int i = 0; i < inherited.size(); ++i) {
            for (int b = 0; b < level.bomb_count && b < MAX_BOMBS; ++b) {
                if (level.bombs[b] == inherited[i].bomb_start) {
                    used_original_bomb[b] = true;
                    break;
                }
            }
        }

        auto remap_suffix_task_to_original_bomb = [&](BombTask& task) -> bool {
            for (int b = 0; b < level.bomb_count && b < MAX_BOMBS; ++b) {
                if (used_original_bomb[b]) continue;
                if (inherited_level.bombs[b] == task.bomb_start) {
                    task.bomb_start = level.bombs[b];
                    used_original_bomb[b] = true;
                    return true;
                }
            }
            return false;
        };

        DFSResult combined_res = inherited_res;
        combined_res.tasks = inherited;
        bool suffix_remap_ok = true;
        for (int i = 0; i < suffix_res.tasks.size() && combined_res.tasks.size() < MAX_BOMBS; ++i) {
            BombTask suffix_task = suffix_res.tasks[i];
            if (!remap_suffix_task_to_original_bomb(suffix_task)) {
                suffix_remap_ok = false;
                break;
            }
            combined_res.tasks.push_back(suffix_task);
        }

        if (suffix_remap_ok && suffix_res.tasks.size() > 0) {
            int combined_deadlocks = inherited_deadlocks;
            int combined_distance = inherited_distance;
            int combined_cost = inherited_cost;
            bool combined_ok = this->evaluate_phase2_task_sequence(
                level,
                combined_res.tasks,
                combined_deadlocks,
                combined_distance,
                &combined_cost);
            if (combined_ok) {
                combined_res.deadlocks_remaining = combined_deadlocks;
                combined_res.net_profit = -combined_distance - combined_cost;
                combined_res.unreachable_pairs_remaining = combined_deadlocks;
                combined_res.bomb_supply_score = combined_cost;
            }
        } else if (!suffix_remap_ok) {
            combined_res = inherited_res;
        }

        if (phase2_result_better_than(combined_res, final_res)) {
            final_res = combined_res;
            selected_profile_pass = 2;
        }
    }

    this->stamp_selected_tasks(final_res);
    this->record_profile_selected(selected_profile_pass, final_res);
    return final_res.tasks;
}




void StrategicPlanner::dfs_phase2_bomb_sequence(
    const SokobanLevel& current_lvl, point player_start,
    StaticArray<BombTask, MAX_BOMBS> current_seq, int cost_so_far,
    int depth, DFSResult& best_res)
{
    this->record_profile_dfs_node();
    const int current_bomb_count = strategy_bomb_count(current_lvl);

    int current_deadlocks = 0;
    int current_distance = 0;
    PlanningCommon::calc_player_reach(current_lvl, player_start, {-1, -1}, {-1, -1}, strategy_ws.dfs_player_vis[depth]);

    // Phase2 只评估箱子到同语义目标的可达性
    for (int b = 0; b < current_lvl.box_count; ++b) {
        this->fast_push_bfs(
            current_lvl,
            current_lvl.boxes[b],
            player_start,
            false,
            strategy_ws.dfs_dist_box[depth][b],
            true,
            true
        );
    }
    evaluate_phase2_semantic_matching(current_lvl, strategy_ws.dfs_dist_box[depth], current_deadlocks, current_distance);

    bool terminal_node = current_seq.size() == current_bomb_count || depth >= MAX_BOMBS;
    if (terminal_node && current_deadlocks > 0) return;

    int profit = -current_distance - cost_so_far;
    if (current_deadlocks < best_res.deadlocks_remaining ||
        (current_deadlocks == best_res.deadlocks_remaining && profit > best_res.net_profit)) {
        best_res.deadlocks_remaining = current_deadlocks;
        best_res.net_profit = profit;
        best_res.tasks = current_seq;
    }
    if (terminal_node) return;

    for (int m = 0; m < current_bomb_count; ++m) {
        if (current_lvl.bombs[m].x == -1) continue;
        this->fast_push_bfs(
            current_lvl,
            current_lvl.bombs[m],
            player_start,
            true,
            strategy_ws.dfs_dist_bomb[depth][m],
            this->phase2_soft_bomb_eval
        );
    }

    const bool use_logic_scores = (depth <= 2);
    if (use_logic_scores) {
        this->record_profile_logic_build();
        build_logic_blast_scores(
            current_lvl,
            strategy_ws.dfs_player_vis[depth],
            strategy_ws.dfs_dist_box[depth],
            strategy_ws.logic
        );
    }

    StaticArray<BombCandidate, 256>& candidates = strategy_ws.dfs.candidates[depth];
    StaticArray<BombCandidate, 256>& preliminary = strategy_ws.dfs.preliminary[depth];
    candidates.clear();
    preliminary.clear();
    std::memset(strategy_ws.dfs.probe_valid[depth], 0, sizeof(strategy_ws.dfs.probe_valid[depth]));

    int selection_limit = PHASE2_SELECTION_RESTRICTIONS;
    int heavy_eval_limit = 255;
    if (depth == 1) heavy_eval_limit = 12;
    else if (depth > 1) heavy_eval_limit = selection_limit * 2;

    StrategyBoxDistances& probe_box_dist = strategy_ws.probe_box_dist;
    // Reuse shared probe cache for temporary Phase2 state evaluation.
    bool structural_defect_active = current_deadlocks > 0;

    auto apply_probe_bomb_transition = [](SokobanLevel& lvl, int bomb_idx, point wall) {
        if (bomb_idx >= 0 && bomb_idx < lvl.bomb_count) {
            lvl.bombs[bomb_idx] = {-1, -1};
        }
        PlanningCommon::apply_blast_effect(lvl, wall);
    };

    auto eval_probe_state = [&](const SokobanLevel& lvl,
                                point eval_player,
                                int& out_deadlocks,
                                int& out_distance) {
        out_deadlocks = 0;
        out_distance = 0;
        for (int b = 0; b < lvl.box_count; ++b) {
            this->fast_push_bfs(lvl, lvl.boxes[b], eval_player, false,
                                probe_box_dist[b], true, true);
        }
        evaluate_phase2_semantic_matching(lvl, probe_box_dist, out_deadlocks, out_distance);
    };

    auto keep_candidate = [&](BombCandidate candidate) {
        this->record_profile_candidate_kept();
        if (candidates.size() < 255) {
            candidates.push_back(candidate);
            return;
        }

        int worst = 0;
        for (int i = 1; i < candidates.size(); ++i) {
            if (candidates[i].score < candidates[worst].score) worst = i;
        }
        if (candidate.score > candidates[worst].score) {
            candidates[worst] = candidate;
        }
    };

    auto keep_preliminary_candidate = [&](BombCandidate candidate) {
        if (preliminary.size() < 255) {
            preliminary.push_back(candidate);
            return;
        }

        int worst = 0;
        for (int i = 1; i < preliminary.size(); ++i) {
            if (preliminary[i].score < preliminary[worst].score) worst = i;
        }
        if (candidate.score > preliminary[worst].score) {
            preliminary[worst] = candidate;
        }
    };

    auto heavy_evaluate_candidate = [&](const BombCandidate& pre) {
        int m = pre.bomb_idx;
        int x = pre.x;
        int y = pre.y;
        if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) return;
        point candidate_wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
        if (!PlanningCommon::is_blastable_wall(current_lvl, candidate_wall) ||
            strategy_ws.dfs_dist_bomb[depth][m][y][x] == INF_DIST) return;

        this->record_profile_candidate_eval();

        int logic_score = use_logic_scores ? strategy_ws.logic.score[y][x] : 0;
        int l1_hits = use_logic_scores ? strategy_ws.logic.l1_hits[y][x] : 0;
        int l2_hits = use_logic_scores ? strategy_ws.logic.l2_hits[y][x] : 0;
        int l3_hits = use_logic_scores ? strategy_ws.logic.l3_hits[y][x] : 0;
        int supply_hits = use_logic_scores ? strategy_ws.logic.bomb_unlock_hits[y][x] : 0;
        bool key_defect_wall = logic_score > 0 || l1_hits > 0 || l2_hits > 0 ||
                               l3_hits > 0 || supply_hits > 0;

        int weak_open_score = 0;
        int wall_mass = 0;
        int entity_touch_score = 0;
        bool opens_unreachable_floor = false;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int ny = y + dy;
                int nx = x + dx;
                if (ny <= 0 || ny >= MAP_MAX_HEIGHT - 1 ||
                    nx <= 0 || nx >= MAP_MAX_WIDTH - 1) {
                    continue;
                }
                if (current_lvl.map[ny][nx] == 1) ++wall_mass;
                if (current_lvl.map[ny][nx] == 0 && !strategy_ws.dfs_player_vis[depth][ny][nx]) {
                    opens_unreachable_floor = true;
                    weak_open_score += 700;
                }
                for (int b = 0; b < current_lvl.box_count; ++b) {
                    if (current_lvl.boxes[b].x == nx && current_lvl.boxes[b].y == ny) {
                        entity_touch_score += 900;
                    }
                }
                for (int t = 0; t < current_lvl.target_count; ++t) {
                    if (current_lvl.targets[t].x == nx && current_lvl.targets[t].y == ny) {
                        entity_touch_score += 700;
                    }
                }
                if (PlanningCommon::has_entity(current_lvl, nx, ny, m)) {
                    entity_touch_score += 200;
                }
            }
        }

        int after_deadlocks = 0;
        int after_distance = 0;
        if (strategy_ws.dfs.probe_valid[depth][y][x] &&
            strategy_ws.dfs.probe_bomb_idx[depth][y][x] == static_cast<uint8_t>(m)) {
            after_deadlocks = strategy_ws.dfs.probe_deadlocks[depth][y][x];
            after_distance = strategy_ws.dfs.probe_distance[depth][y][x];
        } else {
            SokobanLevel probe_lvl = current_lvl;
            point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
            apply_probe_bomb_transition(probe_lvl, m, wall);
            eval_probe_state(probe_lvl, wall, after_deadlocks, after_distance);
            strategy_ws.dfs.probe_valid[depth][y][x] = true;
            strategy_ws.dfs.probe_bomb_idx[depth][y][x] = static_cast<uint8_t>(m);
            strategy_ws.dfs.probe_deadlocks[depth][y][x] = after_deadlocks;
            strategy_ws.dfs.probe_unreachable[depth][y][x] = 9999;
            strategy_ws.dfs.probe_distance[depth][y][x] = after_distance;
        }

        int deadlock_gain = current_deadlocks - after_deadlocks;
        int distance_gain = current_distance - after_distance;

        bool direct_fix = deadlock_gain > 0;
        bool non_regressing_key_defect = key_defect_wall && deadlock_gain >= 0;
        bool keep = direct_fix || non_regressing_key_defect;
        if (!structural_defect_active) {
            keep = keep || distance_gain > 0 || opens_unreachable_floor || entity_touch_score > 0;
        }
        if (!keep) return;

        int score = 0;
        score += deadlock_gain * 900000;
        score += l1_hits * 50000;
        score += l3_hits * 36000;
        score += l2_hits * 24000;
        score += supply_hits * 18000;
        score += logic_score;

        if (opens_unreachable_floor) score += 9000 + weak_open_score;
        if (distance_gain > 0) score += distance_gain * (structural_defect_active ? 14 : 32);
        else score += distance_gain * (structural_defect_active ? 6 : 18);

        score += wall_mass * 260 + entity_touch_score;
        int route_cost_for_score = strategy_direct_bomb_cost_for_score(
            current_lvl,
            player_start,
            current_lvl.bombs[m],
            candidate_wall,
            strategy_ws.dfs_dist_bomb[depth][m][y][x]);
        score -= route_cost_for_score * (structural_defect_active ? 120 : 160);

        if (structural_defect_active && !direct_fix && logic_score <= 0) {
            score -= 60000;
        }

        keep_candidate({static_cast<uint8_t>(m),
                        static_cast<int8_t>(x),
                        static_cast<int8_t>(y),
                        score});
    };

    for (int m = 0; m < current_bomb_count; ++m) {
        if (current_lvl.bombs[m].x == -1) continue;

        for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
            for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
                if (!PlanningCommon::is_blastable_wall(current_lvl, wall) ||
                    strategy_ws.dfs_dist_bomb[depth][m][y][x] == INF_DIST) {
                    continue;
                }
                int logic_score = use_logic_scores ? strategy_ws.logic.score[y][x] : 0;
                int l1_hits = use_logic_scores ? strategy_ws.logic.l1_hits[y][x] : 0;
                int l2_hits = use_logic_scores ? strategy_ws.logic.l2_hits[y][x] : 0;
                int l3_hits = use_logic_scores ? strategy_ws.logic.l3_hits[y][x] : 0;
                int supply_hits = use_logic_scores ? strategy_ws.logic.bomb_unlock_hits[y][x] : 0;
                bool key_defect_wall = logic_score > 0 || l1_hits > 0 || l2_hits > 0 ||
                                       l3_hits > 0 || supply_hits > 0;

                int weak_open_score = 0;
                int wall_mass = 0;
                int entity_touch_score = 0;
                bool opens_unreachable_floor = false;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        int ny = y + dy;
                        int nx = x + dx;
                        if (ny <= 0 || ny >= MAP_MAX_HEIGHT - 1 ||
                            nx <= 0 || nx >= MAP_MAX_WIDTH - 1) {
                            continue;
                        }
                        if (current_lvl.map[ny][nx] == 1) ++wall_mass;
                        if (current_lvl.map[ny][nx] == 0 && !strategy_ws.dfs_player_vis[depth][ny][nx]) {
                            opens_unreachable_floor = true;
                            weak_open_score += 700;
                        }
                        for (int b = 0; b < current_lvl.box_count; ++b) {
                            if (current_lvl.boxes[b].x == nx && current_lvl.boxes[b].y == ny) {
                                entity_touch_score += 900;
                            }
                        }
                        for (int t = 0; t < current_lvl.target_count; ++t) {
                            if (current_lvl.targets[t].x == nx && current_lvl.targets[t].y == ny) {
                                entity_touch_score += 700;
                            }
                        }
                        if (PlanningCommon::has_entity(current_lvl, nx, ny, m)) {
                            entity_touch_score += 200;
                        }
                    }
                }

                if (structural_defect_active && !key_defect_wall && !opens_unreachable_floor) {
                    continue;
                }
                if (!structural_defect_active && !key_defect_wall &&
                    !opens_unreachable_floor && entity_touch_score == 0 && wall_mass < 3) {
                    continue;
                }

                int cheap_score = 0;
                cheap_score += l1_hits * 50000;
                cheap_score += l3_hits * 36000;
                cheap_score += l2_hits * 24000;
                cheap_score += supply_hits * 18000;
                cheap_score += logic_score;
                if (opens_unreachable_floor) cheap_score += 9000 + weak_open_score;
                cheap_score += wall_mass * 260 + entity_touch_score;
                cheap_score -= strategy_ws.dfs_dist_bomb[depth][m][y][x] * (structural_defect_active ? 12 : 18);
                if (structural_defect_active && !key_defect_wall) cheap_score -= 60000;

                keep_preliminary_candidate({
                    static_cast<uint8_t>(m),
                    static_cast<int8_t>(x),
                    static_cast<int8_t>(y),
                    cheap_score
                });
            }
        }
    }

    std::sort(preliminary.begin(), preliminary.end());
    int heavy_count = preliminary.size() < heavy_eval_limit ? preliminary.size() : heavy_eval_limit;
    for (int i = 0; i < heavy_count; ++i) {
        heavy_evaluate_candidate(preliminary[i]);
    }

    std::sort(candidates.begin(), candidates.end());
    int selected_wall_index[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) selected_wall_index[y][x] = -1;
    }
    int write_idx = 0;
    for (int i = 0; i < candidates.size(); ++i) {
        int x = candidates[i].x;
        int y = candidates[i].y;
        if (x < 0 || x >= MAP_MAX_WIDTH || y < 0 || y >= MAP_MAX_HEIGHT) continue;
        int existing_idx = selected_wall_index[y][x];
        if (existing_idx >= 0) {
            BombCandidate& existing = candidates[existing_idx];
            int existing_dist = INF_DIST;
            int candidate_dist = INF_DIST;
            if (existing.bomb_idx < current_bomb_count) {
                existing_dist = strategy_direct_bomb_cost_for_score(
                    current_lvl,
                    player_start,
                    current_lvl.bombs[existing.bomb_idx],
                    {static_cast<int8_t>(x), static_cast<int8_t>(y)},
                    strategy_ws.dfs_dist_bomb[depth][existing.bomb_idx][y][x]);
            }
            if (candidates[i].bomb_idx < current_bomb_count) {
                candidate_dist = strategy_direct_bomb_cost_for_score(
                    current_lvl,
                    player_start,
                    current_lvl.bombs[candidates[i].bomb_idx],
                    {static_cast<int8_t>(x), static_cast<int8_t>(y)},
                    strategy_ws.dfs_dist_bomb[depth][candidates[i].bomb_idx][y][x]);
            }

            const int same_wall_assignment_margin = 500000;
            if (candidate_dist + 1 < existing_dist &&
                candidates[i].score + same_wall_assignment_margin >= existing.score) {
                int wall_score = existing.score;
                existing = candidates[i];
                existing.score = wall_score;
            }
            continue;
        }
        selected_wall_index[y][x] = write_idx;
        if (write_idx != i) {
            candidates[write_idx] = candidates[i];
        }
        ++write_idx;
    }
    candidates.length = write_idx;
    int branch_limit = candidates.size() < selection_limit ? candidates.size() : selection_limit;

    if (structural_defect_active && depth == 0 && branch_limit > 1) {
        int scan_limit = candidates.size();

        auto blast_footprint_overlap = [](const BombCandidate& a, const BombCandidate& b) -> int {
            int dx = std::abs(static_cast<int>(a.x) - static_cast<int>(b.x));
            int dy = std::abs(static_cast<int>(a.y) - static_cast<int>(b.y));
            if (dx >= 3 || dy >= 3) return 0;
            return (3 - dx) * (3 - dy);
        };

        auto bomb_used_before = [&](int slot, uint8_t bomb_idx) -> bool {
            for (int s = 0; s < slot; ++s) {
                if (candidates[s].bomb_idx == bomb_idx) return true;
            }
            return false;
        };

        auto overlaps_before = [&](int slot, const BombCandidate& c) -> bool {
            for (int s = 0; s < slot; ++s) {
                if (blast_footprint_overlap(c, candidates[s]) >= 4) return true;
            }
            return false;
        };

        for (int slot = 1; slot < branch_limit; ++slot) {
            int best_idx = -1;
            for (int i = slot; i < scan_limit; ++i) {
                if (bomb_used_before(slot, candidates[i].bomb_idx)) continue;
                if (overlaps_before(slot, candidates[i])) continue;
                best_idx = i;
                break;
            }
            if (best_idx < 0) {
                for (int i = slot; i < scan_limit; ++i) {
                    if (bomb_used_before(slot, candidates[i].bomb_idx)) continue;
                    best_idx = i;
                    break;
                }
            }
            if (best_idx < 0) {
                for (int i = slot; i < scan_limit; ++i) {
                    if (!overlaps_before(slot, candidates[i])) {
                        best_idx = i;
                        break;
                    }
                }
            }
            if (best_idx >= 0 && best_idx != slot) {
                BombCandidate tmp = candidates[slot];
                candidates[slot] = candidates[best_idx];
                candidates[best_idx] = tmp;
            }
        }
    }

    if (depth == 0) {
        this->record_profile_root_candidates(current_lvl, candidates, branch_limit);
    }

    for (int i = 0; i < branch_limit; ++i) {
        BombCandidate c = candidates[i];
        int m = c.bomb_idx;
        if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) continue;

        SokobanLevel next_lvl = current_lvl;
        next_lvl.bombs[m] = {-1, -1};

        StaticArray<BombTask, MAX_BOMBS> next_seq = current_seq;
        BombTask next_task;
        next_task.bomb_start = current_lvl.bombs[m];
        next_task.target_wall = {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)};
        next_task.is_essential = false;
        next_task.net_profit = 0;
        next_task.box_pushes.clear();
        PlanningCommon::apply_blast_effect(next_lvl, next_task.target_wall);
        next_seq.push_back(next_task);

        int execution_cost = strategy_ws.dfs_dist_bomb[depth][m][c.y][c.x] * 1.5f;

        this->record_profile_child_branch();
        this->dfs_phase2_bomb_sequence(
            next_lvl,
            {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)},
            next_seq,
            cost_so_far + execution_cost,
            depth + 1,
            best_res
        );
    }
}

bool StrategicPlanner::materialize_phase2_sequence(
    const SokobanLevel& level,
    StaticArray<BombTask, MAX_BOMBS>& seq)
{
    SokobanLevel work = level;
    point player = level.player_start;

    for (int i = 0; i < seq.size(); ++i) {
        BombTask task = seq[i];
        if (this->apply_executable_bomb_task(work, player, task)) {
            seq[i] = task;
            continue;
        }

        BombTask materialized_task;
        if (!this->materialize_bomb_task(work, player, task, materialized_task, true)) {
            return false;
        }
        if (!this->apply_executable_bomb_task(work, player, materialized_task)) return false;
        seq[i] = materialized_task;
    }
    return true;
}

bool StrategicPlanner::apply_phase2_task_sequence(
    const SokobanLevel& level,
    const StaticArray<BombTask, MAX_BOMBS>& seq,
    SokobanLevel& out_level,
    point& out_player,
    int* out_sequence_cost)
{
    SokobanLevel work = level;
    point player = level.player_start;
    int sequence_cost = 0;

    for (int i = 0; i < seq.size(); ++i) {
        if (!this->apply_executable_bomb_task(work, player, seq[i], &sequence_cost)) return false;
    }

    work.player_start = player;
    out_level = work;
    out_player = player;
    if (out_sequence_cost) *out_sequence_cost = sequence_cost;
    return true;
}

void StrategicPlanner::evaluate_phase2_level_matching(
    const SokobanLevel& level,
    point player,
    int& out_deadlocks,
    int& out_distance)
{
    for (int b = 0; b < level.box_count; ++b) {
        this->fast_push_bfs(
            level,
            level.boxes[b],
            player,
            false,
            strategy_ws.probe_box_dist[b],
            true,
            true
        );
    }
    evaluate_phase2_semantic_matching(level, strategy_ws.probe_box_dist, out_deadlocks, out_distance);
}

bool StrategicPlanner::evaluate_phase2_task_sequence(
    const SokobanLevel& level,
    const StaticArray<BombTask, MAX_BOMBS>& seq,
    int& out_deadlocks,
    int& out_distance,
    int* out_sequence_cost)
{
    SokobanLevel work;
    point player;
    int sequence_cost = 0;
    if (!this->apply_phase2_task_sequence(level, seq, work, player, &sequence_cost)) return false;

    this->evaluate_phase2_level_matching(work, player, out_deadlocks, out_distance);
    if (out_sequence_cost) *out_sequence_cost = sequence_cost;
    return true;
}
