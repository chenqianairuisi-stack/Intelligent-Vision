#include "Strategy.h"
#include <cstring>
#include <algorithm>

// ============================================================================
// 1. 共享实例与跨阶段 helper
// ============================================================================

OCRAM_BSS StrategicPlanner strategic_planner;

using namespace StrategyConfig;

// Phase1/Phase2 顺序运行，共享一份搜索工作区可避免两份巨大 DTCM/BSS 缓存常驻
OCRAM_BSS static StrategyBoxDepthDistances shared_dfs_dist_box;
OCRAM_BSS static StrategyBombDepthDistances shared_dfs_dist_bomb;
OCRAM_BSS static StrategyPlayerReachDepthMap shared_dfs_player_vis;
OCRAM_BSS static StrategyBoxDepthDistances shared_soft_box_dist_by_depth;
OCRAM_BSS static StrategyBoxDistances shared_probe_box_dist;
OCRAM_BSS static StrategyBombDistances shared_probe_bomb_dist;
OCRAM_BSS static StrategyBombDepthDistances shared_hard_bomb_dist_by_depth;
OCRAM_BSS static StrategyBombDepthDistances shared_strict_bomb_dist_by_depth;
OCRAM_BSS static StrategyMatchDp shared_matching_dp;
OCRAM_BSS static StrategyMatchDp shared_matching_next;
OCRAM_BSS static StrategyDfsScratch shared_dfs_ws;
OCRAM_BSS static LogicBlastScores shared_logic_blast_scores;

StrategySearchWorkspace& strategy_search_workspace() {
    // 只在 Common 内绑定一次引用视图，Phase 文件不直接接触底层大数组
    static StrategySearchWorkspace workspace = {
        shared_dfs_dist_box,
        shared_dfs_dist_bomb,
        shared_dfs_player_vis,
        shared_soft_box_dist_by_depth,
        shared_probe_box_dist,
        shared_probe_bomb_dist,
        shared_hard_bomb_dist_by_depth,
        shared_strict_bomb_dist_by_depth,
        shared_matching_dp,
        shared_matching_next,
        shared_dfs_ws,
        shared_logic_blast_scores
    };
    return workspace;
}

int strategy_box_at(const SokobanLevel& lvl, point p) {
    for (int b = 0; b < lvl.box_count; ++b) {
        if (lvl.boxes[b] == p) return b;
    }
    return -1;
}

bool strategy_target_allowed_for_box(const SokobanLevel& lvl, int box_id, int target_id, bool phase2_specific) {
    if (target_id < 0 || target_id >= lvl.target_count) return false;
    if (!phase2_specific) return true;
    if (box_id < 0 || box_id >= lvl.box_count) return false;
    return lvl.box_semantics[box_id] == lvl.target_semantics[target_id];
}

bool strategy_is_goal_for_box(const SokobanLevel& lvl, int box_id, point p, bool phase2_specific) {
    for (int t = 0; t < lvl.target_count; ++t) {
        if (lvl.targets[t] == p && strategy_target_allowed_for_box(lvl, box_id, t, phase2_specific)) return true;
    }
    return false;
}

bool strategy_is_any_target_cell(const SokobanLevel& lvl, point p) {
    for (int t = 0; t < lvl.target_count; ++t) {
        if (lvl.targets[t] == p) return true;
    }
    return false;
}

int strategy_nearest_goal_distance(const SokobanLevel& lvl, int box_id, point p, bool phase2_specific) {
    int best = 99;
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!strategy_target_allowed_for_box(lvl, box_id, t, phase2_specific)) continue;
        int d = std::abs(p.x - lvl.targets[t].x) + std::abs(p.y - lvl.targets[t].y);
        if (d < best) best = d;
    }
    return best;
}

int strategy_bomb_count(const SokobanLevel& lvl) {
    return lvl.bomb_count < MAX_BOMBS ? lvl.bomb_count : MAX_BOMBS;
}

int strategy_direct_bomb_cost_for_score(
    const SokobanLevel& lvl,
    point player,
    point bomb_start,
    point target_wall,
    int fallback_dist)
{
    if (fallback_dist >= INF_DIST || bomb_start.x == -1 || target_wall.x == -1) return INF_DIST;

    BombTask probe;
    probe.bomb_start = bomb_start;
    probe.target_wall = target_wall;
    probe.is_essential = false;
    probe.net_profit = 0;
    probe.box_pushes.clear();

    uint16_t direct_cost = 0;
    point final_player = {-1, -1};
    if (PlanningCommon::get_direct_bomb_push_path_cost(lvl, player, probe, direct_cost, final_player)) {
        return direct_cost;
    }

    // 不可直接执行的 soft 候选仍保留拓扑距离，但必须带上明显机会成本
    return fallback_dist * 8 + 80;
}

int16_t strategy_clamp_i16(int value) {
    if (value < -32768) return -32768;
    if (value > 32767) return 32767;
    return static_cast<int16_t>(value);
}

void mark_soft_deadlock_boxes(const SokobanLevel& lvl, bool out_hard[MAX_BOXES]) {
    std::memset(out_hard, 0, sizeof(bool) * MAX_BOXES);

    auto is_wall = [&](point p) {
        if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
        return lvl.map[p.y][p.x] == 1;
    };

    auto strong_component_size = [&](point start) -> int {
        if (is_wall(start)) return 0;

        static bool fwd_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        static bool rev_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        static point q[MAP_CELL_COUNT];
        std::memset(fwd_vis, 0, sizeof(fwd_vis));
        std::memset(rev_vis, 0, sizeof(rev_vis));

        int head = 0;
        int tail = 0;
        q[tail++] = start;
        fwd_vis[start.y][start.x] = true;
        while (head < tail) {
            point curr = q[head++];
            for (int d = 0; d < 4; ++d) {
                point next = curr + MOVE[d];
                point stand = curr - MOVE[d];
                if (is_wall(next) || is_wall(stand)) continue;
                if (fwd_vis[next.y][next.x]) continue;
                fwd_vis[next.y][next.x] = true;
                q[tail++] = next;
            }
        }

        head = 0;
        tail = 0;
        q[tail++] = start;
        rev_vis[start.y][start.x] = true;
        while (head < tail) {
            point curr = q[head++];
            for (int d = 0; d < 4; ++d) {
                point prev = curr - MOVE[d];
                point stand = prev - MOVE[d];
                if (is_wall(prev) || is_wall(stand)) continue;
                if (rev_vis[prev.y][prev.x]) continue;
                rev_vis[prev.y][prev.x] = true;
                q[tail++] = prev;
            }
        }

        int count = 0;
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                if (fwd_vis[y][x] && rev_vis[y][x]) ++count;
            }
        }
        return count;
    };

    for (int b = 0; b < lvl.box_count; ++b) {
        point p = lvl.boxes[b];
        if (strategy_is_goal_for_box(lvl, -1, p, false)) continue;

        if (strong_component_size(p) <= 1) {
            out_hard[b] = true;
            continue;
        }

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

static void mark_soft_unpushable_boxes(
    const SokobanLevel& lvl,
    point player_start,
    point ignored_obj,
    bool out_hard[MAX_BOXES]) {
    mark_soft_deadlock_boxes(lvl, out_hard);

    bool movable[MAX_BOXES] = {false};
    bool phase1_done[MAX_BOXES] = {false};
    static int8_t box_at[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static uint8_t bomb_at[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            box_at[y][x] = -1;
            bomb_at[y][x] = 0;
        }
    }

    for (int b = 0; b < lvl.box_count; ++b) {
        point p = lvl.boxes[b];
        if (PlanningCommon::in_bounds(p)) {
            box_at[p.y][p.x] = static_cast<int8_t>(b);
        }
        phase1_done[b] = strategy_is_goal_for_box(lvl, b, p, false);
    }

    for (int b = 0; b < lvl.bomb_count; ++b) {
        point p = lvl.bombs[b];
        if (PlanningCommon::in_bounds(p)) {
            bomb_at[p.y][p.x] = 1;
        }
    }

    auto soft_passable = [&](point p) -> bool {
        if (!PlanningCommon::in_bounds(p) || lvl.map[p.y][p.x] == 1) return false;
        if (bomb_at[p.y][p.x]) return false;

        int box_id = box_at[p.y][p.x];
        if (box_id < 0) return true;
        if (out_hard[box_id]) return false;
        if (phase1_done[box_id]) return true;
        return movable[box_id];
    };

    static bool player_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static point q[MAP_CELL_COUNT];

    auto build_reach = [&]() {
        std::memset(player_vis, 0, sizeof(player_vis));
        int head = 0;
        int tail = 0;
        if (soft_passable(player_start)) {
            q[tail++] = player_start;
            player_vis[player_start.y][player_start.x] = true;
        }

        while (head < tail) {
            point curr = q[head++];
            for (int d = 0; d < 4; ++d) {
                point np = curr + MOVE[d];
                if (!PlanningCommon::in_bounds(np) || player_vis[np.y][np.x]) continue;
                if (!soft_passable(np)) continue;
                player_vis[np.y][np.x] = true;
                q[tail++] = np;
            }
        }
    };

    auto destination_clearable = [&](point p) -> bool {
        if (!PlanningCommon::in_bounds(p) || lvl.map[p.y][p.x] == 1) return false;
        if (bomb_at[p.y][p.x]) return false;

        int box_id = box_at[p.y][p.x];
        if (box_id < 0) return true;
        if (out_hard[box_id]) return false;
        if (phase1_done[box_id]) return true;
        return movable[box_id];
    };

    PlanningCommon::calc_player_reach(lvl, player_start, {-1, -1}, {-1, -1}, player_vis);

    auto floor_without_bomb = [&](point p) -> bool {
        return PlanningCommon::in_bounds(p) &&
               lvl.map[p.y][p.x] == 0 &&
               !bomb_at[p.y][p.x];
    };

    auto target_at = [&](point p) -> bool {
        for (int t = 0; t < lvl.target_count; ++t) {
            if (lvl.targets[t] == p) return true;
        }
        return false;
    };

    auto axis_confined_cell = [&](point p, int axis) -> bool {
        if (!floor_without_bomb(p)) return false;
        int d0 = axis == 0 ? 1 : 0;
        int d1 = axis == 0 ? 3 : 2;
        for (int k = 0; k < 2; ++k) {
            int d = k == 0 ? d0 : d1;
            point to = p + MOVE[d];
            point stand = p - MOVE[d];
            if (floor_without_bomb(to) && floor_without_bomb(stand)) return false;
        }
        return true;
    };

    auto has_side_entry = [&](point p, int axis) -> bool {
        int d0 = axis == 0 ? 1 : 0;
        int d1 = axis == 0 ? 3 : 2;
        for (int k = 0; k < 2; ++k) {
            int d = k == 0 ? d0 : d1;
            point side = p + MOVE[d];
            if (!PlanningCommon::in_bounds(side)) continue;
            if (lvl.map[side.y][side.x] == 1 || bomb_at[side.y][side.x]) continue;
            if (box_at[side.y][side.x] >= 0) continue;
            if (player_vis[side.y][side.x]) return true;
        }
        return false;
    };

    auto has_box_at = [&](point p) -> int {
        if (!PlanningCommon::in_bounds(p)) return -1;
        return box_at[p.y][p.x];
    };

    auto hard_empty_cell = [&](point p) -> bool {
        return floor_without_bomb(p) && has_box_at(p) < 0;
    };

    auto box_lateral_exit_open = [&](point p, int side_dir) -> bool {
        point dest = p + MOVE[side_dir];
        point stand = p - MOVE[side_dir];
        return hard_empty_cell(dest) && hard_empty_cell(stand);
    };

    auto soft_parking_cell_valid = [&](point p, int push_dir) -> bool {
        if (!PlanningCommon::in_bounds(p) || lvl.map[p.y][p.x] == 1) return false;
        if (bomb_at[p.y][p.x]) return false;
        if (target_at(p)) return true;

        int occupied_box = has_box_at(p);
        if (occupied_box >= 0) {
            if (phase1_done[occupied_box]) return true;
            if (out_hard[occupied_box]) return false;
            if (!movable[occupied_box]) return false;
        }

        if (PlanningCommon::is_static_deadlock_cell(lvl, p)) return false;

        int side_a = (push_dir % 2) == 0 ? 1 : 0;
        int side_b = (push_dir % 2) == 0 ? 3 : 2;
        bool one_way_lane =
            !box_lateral_exit_open(p, side_a) &&
            !box_lateral_exit_open(p, side_b);
        if (!one_way_lane) return true;

        // 停车位仍在单通路内时，必须能沿推动方向找到箱子可侧向离开的真正出口
        point scan = p + MOVE[push_dir];
        for (int guard = 0; guard < MAP_CELL_COUNT; ++guard) {
            if (!PlanningCommon::in_bounds(scan) || lvl.map[scan.y][scan.x] == 1) return false;
            if (bomb_at[scan.y][scan.x]) return false;
            if (target_at(scan)) return true;

            int box_id = has_box_at(scan);
            if (box_id >= 0) {
                if (phase1_done[box_id]) return true;
                if (out_hard[box_id] || !movable[box_id]) return false;
            }

            if (!PlanningCommon::is_static_deadlock_cell(lvl, scan)) {
                bool still_one_way =
                    !box_lateral_exit_open(scan, side_a) &&
                    !box_lateral_exit_open(scan, side_b);
                if (!still_one_way) return true;
            }

            scan = scan + MOVE[push_dir];
        }
        return false;
    };

    // 单格通道内相邻箱子之间没有侧向入口时，不能互相当作可清开的软障碍
    for (int a = 0; a < lvl.box_count; ++a) {
        if (out_hard[a] || phase1_done[a]) continue;
        for (int b = a + 1; b < lvl.box_count; ++b) {
            if (out_hard[b] || phase1_done[b]) continue;

            point pa = lvl.boxes[a];
            point pb = lvl.boxes[b];
            int axis = -1;
            int start = 0;
            int end = 0;
            if (pa.x == pb.x) {
                axis = 0;
                start = pa.y < pb.y ? pa.y : pb.y;
                end = pa.y < pb.y ? pb.y : pa.y;
            } else if (pa.y == pb.y) {
                axis = 1;
                start = pa.x < pb.x ? pa.x : pb.x;
                end = pa.x < pb.x ? pb.x : pa.x;
            } else {
                continue;
            }

            bool confined = true;
            bool side_entry_between = false;
            int segment_boxes = 0;
            int segment_targets = 0;

            for (int pos = start; pos <= end; ++pos) {
                point p = axis == 0
                    ? point{pa.x, static_cast<int8_t>(pos)}
                    : point{static_cast<int8_t>(pos), pa.y};
                if (!axis_confined_cell(p, axis)) {
                    confined = false;
                    break;
                }
                if (box_at[p.y][p.x] >= 0) ++segment_boxes;
                if (target_at(p)) ++segment_targets;
                if (pos > start && pos < end && has_side_entry(p, axis)) {
                    side_entry_between = true;
                }
            }

            if (!confined || side_entry_between) continue;
            if (segment_boxes < 2 || segment_targets >= segment_boxes) continue;

            for (int pos = start; pos <= end; ++pos) {
                point p = axis == 0
                    ? point{pa.x, static_cast<int8_t>(pos)}
                    : point{static_cast<int8_t>(pos), pa.y};
                int box_id = box_at[p.y][p.x];
                if (box_id >= 0 && !phase1_done[box_id]) out_hard[box_id] = true;
            }
        }
    }

    // 软障碍资格按可清障闭包传播，避免把当前推不开的箱子误当空气
    for (int iter = 0; iter < MAX_BOXES; ++iter) {
        bool changed = false;
        build_reach();

        for (int b = 0; b < lvl.box_count; ++b) {
            if (out_hard[b] || movable[b] || phase1_done[b]) continue;
            if (lvl.boxes[b] == ignored_obj) continue;

            point box = lvl.boxes[b];
            for (int d = 0; d < 4; ++d) {
                point stand = box - MOVE[d];
                point to = box + MOVE[d];
                if (!PlanningCommon::in_bounds(stand) || !player_vis[stand.y][stand.x]) continue;
                if (!destination_clearable(to)) continue;
                if (!soft_parking_cell_valid(to, d)) continue;

                movable[b] = true;
                changed = true;
                break;
            }
        }

        if (!changed) break;
    }

    // 软障碍只代表当前局面中可被清开的箱子
    for (int b = 0; b < lvl.box_count; ++b) {
        if (lvl.boxes[b] == ignored_obj) continue;
        if (phase1_done[b]) continue;
        if (!out_hard[b] && !movable[b]) out_hard[b] = true;
    }
}

// ============================================================================
// 2. MCU 版诊断钩子
// ============================================================================

void StrategicPlanner::begin_profile_eval(uint8_t mode) {
    (void)mode;
}

void StrategicPlanner::set_profile_pass(uint8_t pass) {
    (void)pass;
}

void StrategicPlanner::record_profile_result(uint8_t pass, const DFSResult& result) {
    (void)pass;
    (void)result;
}

void StrategicPlanner::record_profile_selected(uint8_t pass, const DFSResult& result) {
    (void)pass;
    (void)result;
}

void StrategicPlanner::record_profile_root_candidates(
    const SokobanLevel& level,
    const StaticArray<BombCandidate, 256>& candidates,
    int branch_limit) {
    (void)level;
    (void)candidates;
    (void)branch_limit;
}

void StrategicPlanner::record_profile_dfs_node() {
}

void StrategicPlanner::record_profile_fast_bfs_call() {
}

void StrategicPlanner::record_profile_candidate_eval() {
}

void StrategicPlanner::record_profile_candidate_kept() {
}

void StrategicPlanner::record_profile_child_branch() {
}

void StrategicPlanner::record_profile_logic_build() {
}

void StrategicPlanner::record_profile_local_clear_call() {
}

void StrategicPlanner::record_profile_local_clear_success() {
}

void StrategicPlanner::record_profile_local_clear_time(uint32_t elapsed_us) {
    (void)elapsed_us;
}

void StrategicPlanner::record_profile_materialize_call() {
}

void StrategicPlanner::record_profile_materialize_success() {
}

void StrategicPlanner::record_profile_fast_bfs_detail(
    uint32_t elapsed_us,
    uint32_t player_reach_calls,
    uint32_t state_pops,
    uint16_t max_queue) {
    (void)elapsed_us;
    (void)player_reach_calls;
    (void)state_pops;
    (void)max_queue;
}

void StrategicPlanner::record_profile_macro_soft_call() {
}

void StrategicPlanner::record_profile_macro_soft_detail(
    uint32_t elapsed_us,
    uint32_t state_pops,
    uint16_t max_queue) {
    (void)elapsed_us;
    (void)state_pops;
    (void)max_queue;
}

void StrategicPlanner::record_profile_soft_route_build(int route_len) {
    (void)route_len;
}

void StrategicPlanner::record_profile_box_push_check(bool success) {
    (void)success;
}

void StrategicPlanner::record_profile_bomb_path_check(bool success) {
    (void)success;
}

void StrategicPlanner::record_profile_player_path_check() {
}

void StrategicPlanner::record_profile_real_clear_node(int depth) {
    (void)depth;
}

void StrategicPlanner::record_profile_real_clear_candidates(int candidate_count, int try_limit) {
    (void)candidate_count;
    (void)try_limit;
}

void StrategicPlanner::record_shadow_clear_route_attempt(bool success, int blocker_count) {
    (void)success;
    (void)blocker_count;
}

void StrategicPlanner::record_shadow_clear_blocker(StrategyClearReason reason) {
    (void)reason;
}

void StrategicPlanner::record_shadow_push_stand_blocker(int stand_dist) {
    (void)stand_dist;
}

void StrategicPlanner::record_shadow_clear_accept(StrategyClearReason reason, StrategyClearParking parking) {
    (void)reason;
    (void)parking;
}

void StrategicPlanner::record_shadow_clear_decision(StrategyClearReason reason, StrategyClearParking parking, int support_dist, bool accepted) {
    (void)reason;
    (void)parking;
    (void)support_dist;
    (void)accepted;
}

void StrategicPlanner::record_shadow_real_node() {
}

void StrategicPlanner::record_shadow_real_source_distance(int source_support_dist) {
    (void)source_support_dist;
}

void StrategicPlanner::record_shadow_real_push_candidate() {
}

void StrategicPlanner::record_shadow_real_push_executable() {
}

void StrategicPlanner::record_shadow_real_opens_path() {
}

void StrategicPlanner::record_shadow_real_parking(StrategyClearParking parking, bool accepted) {
    (void)parking;
    (void)accepted;
}

StrategyClearRouteTrace* StrategicPlanner::begin_profile_clear(
    const SokobanLevel& level,
    int bomb_idx,
    point target_wall,
    bool phase2_specific,
    bool include_player_access_clear) {
    (void)level;
    (void)bomb_idx;
    (void)target_wall;
    (void)phase2_specific;
    (void)include_player_access_clear;
    return nullptr;
}

void StrategicPlanner::record_profile_clear_route(
    StrategyClearRouteTrace* trace,
    int route_len,
    int blocker_count) {
    (void)trace;
    (void)route_len;
    (void)blocker_count;
}

void StrategicPlanner::record_profile_clear_push(
    StrategyClearRouteTrace* trace,
    uint8_t box_id,
    StrategyClearReason reason,
    StrategyClearParking parking,
    StrategyRescueObligationKind obligation,
    uint8_t owner_task_index,
    point box_start,
    point box_target,
    point owner_bomb_start,
    point owner_target_wall,
    int depth,
    bool opens_bomb_path,
    bool safe_without_open_path,
    int score) {
    (void)trace;
    (void)box_id;
    (void)reason;
    (void)parking;
    (void)obligation;
    (void)owner_task_index;
    (void)box_start;
    (void)box_target;
    (void)owner_bomb_start;
    (void)owner_target_wall;
    (void)depth;
    (void)opens_bomb_path;
    (void)safe_without_open_path;
    (void)score;
}

void StrategicPlanner::finish_profile_clear(
    StrategyClearRouteTrace* trace,
    bool success,
    StrategyClearMethod method,
    int cost) {
    (void)trace;
    (void)success;
    (void)method;
    (void)cost;
}
void StrategicPlanner::merge_clear_obligation(
    StaticArray<StrategyClearObligation, MAX_BOMBS * 8>& obligations,
    const StrategyClearObligation& obligation)
{
    for (int i = 0; i < obligations.size(); ++i) {
        StrategyClearObligation& existing = obligations[i];
        if (existing.box_id == obligation.box_id &&
            existing.creator_task_index == obligation.creator_task_index) {
            existing = obligation;
            return;
        }
    }
    obligations.push_back(obligation);
}

int StrategicPlanner::count_unresolved_clear_obligations(
    const StaticArray<StrategyClearObligation, MAX_BOMBS * 8>& obligations,
    int resolved_task_count)
{
    int unresolved = 0;
    for (int i = 0; i < obligations.size(); ++i) {
        const StrategyClearObligation& obligation = obligations[i];
        switch (obligation.obligation) {
            case StrategyRescueObligationKind::NONE:
                break;
            case StrategyRescueObligationKind::EXPLICIT_PHASE1_TASK:
                if (obligation.owner_task_index == 255 ||
                    obligation.owner_task_index >= resolved_task_count) {
                    ++unresolved;
                }
                break;
            case StrategyRescueObligationKind::EXPLICIT_FUTURE_BOMB:
            case StrategyRescueObligationKind::UNRESOLVED:
                ++unresolved;
                break;
        }
    }
    return unresolved;
}


// ============================================================================
// 3. 搜索 pass 入口与任务标记
// ============================================================================

void StrategicPlanner::execute_phase1_search_pass(
    const SokobanLevel& level,
    uint8_t pass,
    DFSResult& out_res)
{
    out_res.tasks.clear();
    out_res.deadlocks_remaining = 9999;
    out_res.net_profit = -999999;
    out_res.unreachable_pairs_remaining = 9999;
    out_res.bomb_supply_score = 0;

    StaticArray<BombTask, MAX_BOMBS> empty_seq;
    this->set_profile_pass(pass);
    this->dfs_phase1_bomb_sequence(level, level.player_start, empty_seq, 0, 0, out_res);
    this->record_profile_result(pass, out_res);
}

void StrategicPlanner::execute_phase2_search_pass(
    const SokobanLevel& level,
    uint8_t pass,
    DFSResult& out_res)
{
    out_res.tasks.clear();
    out_res.deadlocks_remaining = 9999;
    out_res.net_profit = -999999;
    out_res.unreachable_pairs_remaining = 9999;
    out_res.bomb_supply_score = 0;

    StaticArray<BombTask, MAX_BOMBS> empty_seq;
    this->set_profile_pass(pass);
    this->dfs_phase2_bomb_sequence(level, level.player_start, empty_seq, 0, 0, out_res);
    this->record_profile_result(pass, out_res);
}

void StrategicPlanner::stamp_selected_tasks(DFSResult& result)
{
    bool selected_structurally_solved =
        result.deadlocks_remaining == 0 && result.unreachable_pairs_remaining == 0;

    for (int i = 0; i < result.tasks.size(); ++i) {
        result.tasks[i].is_essential = selected_structurally_solved;
        result.tasks[i].net_profit = result.net_profit;
    }
}

// ============================================================================
// 4. 任务回放与基础可执行性检查
// ============================================================================

bool StrategicPlanner::apply_executable_bomb_task(
    SokobanLevel& work,
    point& player,
    const BombTask& task,
    int* sequence_cost)
{
    StaticArray<point, MAX_PATH_LENGTH> path;
    if (!PlanningCommon::get_bomb_push_path(work, player, task, path)) return false;
    if (sequence_cost) *sequence_cost += PlanningCommon::path_time_cost(player, path);
    if (!path.empty()) player = path.back();
    PlanningCommon::apply_bomb_task_effect(work, task);
    return true;
}

// ============================================================================
// 5. 推物体距离场：Fast Push-BFS
// ============================================================================

/// \brief 计算单个箱子或炸弹被推动到各格子的估计代价
/// \param lvl 当前地图状态
/// \param start_obj 被推动物体的初始位置
/// \param player_start 玩家初始位置
/// \param is_bomb true 表示推动炸弹，允许墙体作为爆破终点被记录
/// \param out_dist 输出距离场，out_dist[y][x] 为物体到达 (x,y) 的代价
/// \param soft_boxes true 时把非目标箱子视作带惩罚的软障碍
/// \param strict_soft_boxes true 时只有当前清障闭包内可推开的箱子才作为软障碍
///
/// \details
/// 搜索状态为“物体坐标 + 玩家相对物体的发力方向”
/// 函数会检查直推、转向和掉头空间，并用玩家可达性剪掉无法绕后的状态
void StrategicPlanner::fast_push_bfs(
    const SokobanLevel& lvl,
    point start_obj,
    point player_start,
    bool is_bomb,
    int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    bool soft_boxes,
    bool strict_soft_boxes) {
    this->record_profile_fast_bfs_call();
    struct QNode { int8_t x, y, dir; int16_t cost; };
    static QNode q[1024];
    int head = 0, tail = 0;
    
    // 初始化距离矩阵和状态成本矩阵
    static int16_t state_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    static uint16_t state_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    static uint16_t cur_state_gen = 0;

    cur_state_gen++;
    if (cur_state_gen == 0) { std::memset(state_gen, 0, sizeof(state_gen)); cur_state_gen = 1; }

    for (int y = 0; y < MAP_MAX_HEIGHT; y++) {
        for (int x = 0; x < MAP_MAX_WIDTH; x++) {
            out_dist[y][x] = INF_DIST;
        }
    }

    bool soft_hard_box[MAX_BOXES] = {false};
    if (soft_boxes) {
        if (strict_soft_boxes) mark_soft_unpushable_boxes(lvl, player_start, start_obj, soft_hard_box);
        else mark_soft_deadlock_boxes(lvl, soft_hard_box);
    }

    static int8_t box_at[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static uint8_t bomb_at[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            box_at[y][x] = -1;
            bomb_at[y][x] = 0;
        }
    }
    for (int i = 0; i < lvl.box_count; ++i) {
        point p = lvl.boxes[i];
        if (p.x >= 0 && p.x < MAP_MAX_WIDTH && p.y >= 0 && p.y < MAP_MAX_HEIGHT) {
            box_at[p.y][p.x] = static_cast<int8_t>(i);
        }
    }
    for (int i = 0; i < lvl.bomb_count; ++i) {
        point p = lvl.bombs[i];
        if (p.x >= 0 && p.x < MAP_MAX_WIDTH && p.y >= 0 && p.y < MAP_MAX_HEIGHT) {
            bomb_at[p.y][p.x] = 1;
        }
    }

    // 预先计算玩家可达性，剪枝不可达状态
    auto is_blocked = [&](point p, point ignored_obj) -> bool {
        if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
        if (lvl.map[p.y][p.x] == 1) return true;
        int box_id = box_at[p.y][p.x];
        if (box_id >= 0) {
            if (p == ignored_obj) return false;
            if (soft_boxes && !(p == start_obj) && !soft_hard_box[box_id]) return false;
            return true;
        }
        if (bomb_at[p.y][p.x] && !(p == ignored_obj)) return true;
        return false;
    };

    auto soft_penalty = [&](point p, point ignored_obj) -> int16_t {
        if (!soft_boxes || p == ignored_obj || p == start_obj) return 0;
        int box_id = box_at[p.y][p.x];
        if (box_id >= 0 && !soft_hard_box[box_id]) {
            return 10;
        }
        return 0;
    };

    auto can_reach = [&](point start_pos, point target_pos, point ignored_obj, point extra_obs) -> bool {
        if (start_pos == target_pos) return true;
        OCRAM_BSS static uint16_t vis_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        static uint16_t cur_vis_gen = 0;
        cur_vis_gen++;
        if (cur_vis_gen == 0) { std::memset(vis_gen, 0, sizeof(vis_gen)); cur_vis_gen = 1; }

        OCRAM_BSS static point rq[256];
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
                state_gen[start_obj.y][start_obj.x][d] = cur_state_gen;
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
                    // 内角受阻时，改用可达性检测
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
                if (is_bomb && PlanningCommon::is_blastable_wall(lvl, next_p)) {
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

/// \brief 对炸弹移动做软障碍拓扑估价
/// \param lvl 当前地图状态
/// \param start_obj 炸弹初始位置
/// \param out_dist 输出距离场，墙体可作为终点但不可穿过
///
/// \details
/// 该函数不精确模拟玩家站位，只评估炸弹拓扑连通性
/// 箱子被视为软障碍并加入惩罚，用于判断是否值得进一步调用局部清障生成推箱让路任务，
/// 后续再由 local_clear_bomb_route 验证真实可执行性
void StrategicPlanner::macro_soft_dijkstra(const SokobanLevel& lvl, point start_obj, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    this->record_profile_macro_soft_call();
    // 使用 SPFA/Dijkstra 变体，放在极速区
    OCRAM_BSS static point q[2048];
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
            
            // 只有内圈墙可以作为爆破终点，边界墙仍是永久障碍
            if (lvl.map[np.y][np.x] == 1) {
                if (PlanningCommon::is_blastable_wall(lvl, np) &&
                    ccost + 1 < out_dist[np.y][np.x]) {
                    out_dist[np.y][np.x] = ccost + 1;
                }
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
// 7. 炸弹任务快速可执行性验证
// ============================================================================

bool StrategicPlanner::are_fast_bomb_tasks_directly_executable(const SokobanLevel& level, const StaticArray<BombTask, MAX_BOMBS>& tasks) {
    SokobanLevel temp = level;
    point player = level.player_start;
    OCRAM_BSS static int16_t direct_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

    for (int i = 0; i < tasks.size(); ++i) {
        const BombTask& task = tasks[i];
        if (!PlanningCommon::is_blastable_wall(temp, task.target_wall)) return false;

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
