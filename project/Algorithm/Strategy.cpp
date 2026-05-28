#include "Strategy.h"
#include <cstring>
#include <algorithm>
#if defined(STRATEGY_DEBUG_PHASE2_RESULT) || defined(STRATEGY_DEBUG_PHASE1_REFINEMENT)
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
DTCM_DATA StrategicPlanner strategic_planner;

namespace StrategyConfig {
    // ------------------------------------------------------------------------
    // DFS 搜索宽度
    // ------------------------------------------------------------------------
    inline constexpr uint8_t PHASE1_SELECTION_RESTRICTIONS = 3;   // Phase1 每层最多保留的高分候选数
    inline constexpr uint8_t PHASE2_SELECTION_RESTRICTIONS = 6;   // Phase2 每层最多保留的高分候选数

    // ------------------------------------------------------------------------
    // 距离与收益阈值
    // ------------------------------------------------------------------------
    inline constexpr int16_t INF_DIST = 9999;                     // 不可达距离的占位值
    inline constexpr int PHASE1_SOFT_REPLACE_PROFIT_MARGIN = 20;  // Phase1 软评估替换的最低收益边际；单位是距离场数值，越大越保守

    // ------------------------------------------------------------------------
    // 局部清障回退
    // ------------------------------------------------------------------------
    inline constexpr int LOCAL_CLEAR_MAX_TASKS = 8;               // 回退时最多生成的推箱任务数，过多会增加排序和验证开销
    inline constexpr int LOCAL_CLEAR_MAX_ITER = 5;                // 回退时最多尝试的推箱迭代次数，过多会增加搜索开销
    inline constexpr int LOCAL_CLEAR_CANDIDATE_LIMIT = 10;        // 回退时每轮最多考虑的候选推箱数，过多会增加搜索开销
    inline constexpr int LOCAL_CLEAR_CHAIN_DEPTH = 1;             // 回退时推箱任务的链式生成深度，过大可能会产生过长的回退路径

    // ------------------------------------------------------------------------
    // Phase1 匹配缓存
    // ------------------------------------------------------------------------
    inline constexpr int PHASE1_MATCH_MASKS = 1 << MAX_BOXES;     // Phase1 匹配状态掩码数量
}

using namespace StrategyConfig;

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

static bool strategy_target_matches_box_semantic(const SokobanLevel& lvl, int box_id, int target_id) {
    if (box_id < 0 || box_id >= lvl.box_count) return false;
    if (target_id < 0 || target_id >= lvl.target_count) return false;
    return lvl.box_semantics[box_id] == lvl.target_semantics[target_id];
}

static bool strategy_is_valid_target_for_box(const SokobanLevel& lvl, int box_id, point p) {
    for (int t = 0; t < lvl.target_count; ++t) {
        if (lvl.targets[t] == p && strategy_target_matches_box_semantic(lvl, box_id, t)) return true;
    }
    return false;
}

static int strategy_nearest_semantic_goal_distance(const SokobanLevel& lvl, int box_id, point p) {
    if (box_id < 0 || box_id >= lvl.box_count) return 99;
    int best = 99;
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!strategy_target_matches_box_semantic(lvl, box_id, t)) continue;
        int d = std::abs(p.x - lvl.targets[t].x) + std::abs(p.y - lvl.targets[t].y);
        if (d < best) best = d;
    }
    return best;
}

static bool strategy_has_reachable_semantic_goal(
    const SokobanLevel& lvl,
    int box_id,
    int16_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!strategy_target_matches_box_semantic(lvl, box_id, t)) continue;
        point target = lvl.targets[t];
        if (dist[target.y][target.x] != INF_DIST) return true;
    }
    return false;
}

static int strategy_bomb_count(const SokobanLevel& lvl) {
    return lvl.bomb_count < MAX_BOMBS ? lvl.bomb_count : MAX_BOMBS;
}

static void mark_soft_deadlock_boxes(const SokobanLevel& lvl, bool out_hard[MAX_BOXES]) {
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
        if (strategy_target_at(lvl, p)) continue;

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

// dfs_dist_box[depth][box][y][x]：当前 DFS 深度下，某箱子被推到 (x,y) 的估计代价
DTCM_DATA static int16_t dfs_dist_box[MAX_BOMBS + 1][MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// dfs_dist_bomb[depth][bomb][y][x]：当前 DFS 深度下，某炸弹被推到 (x,y) 的估计代价
DTCM_DATA static int16_t dfs_dist_bomb[MAX_BOMBS + 1][MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// dfs_player_vis[depth][y][x]：当前 DFS 深度下玩家可达区域
DTCM_DATA static bool dfs_player_vis[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// Phase1 结构判定距离场：非死锁箱子视作可挪开的软障碍，避免把临时箱子阻塞误判为必炸墙体。
DTCM_DATA static int16_t phase1_soft_dist_box[MAX_BOMBS + 1][MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// Phase1 候选二次排序使用的临时距离场。只在少量候选上运行，避免污染递归层缓存。
DTCM_DATA static int16_t phase1_candidate_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
DTCM_DATA static int16_t phase1_candidate_bomb_dist[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// Phase1 任意匹配 DP 缓存，避免递归过程中频繁申请大数组
__attribute__((section(".dtcm_bss"))) static int phase1_match_dp[PHASE1_MATCH_MASKS];
__attribute__((section(".dtcm_bss"))) static int phase1_match_next[PHASE1_MATCH_MASKS];

struct StrategyDfsScratch {
    StaticArray<BombCandidate, 256> preliminary[MAX_BOMBS + 1];
    StaticArray<BombCandidate, 256> candidates[MAX_BOMBS + 1];
    bool probe_valid[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    int probe_deadlocks[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    int probe_unreachable[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    int probe_distance[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
};

static StrategyDfsScratch strategy_dfs_ws;

// ============================================================================
// 缺陷驱动逻辑层：把“不可达/死锁”投影为可修复它的 3x3 爆破区域
// ============================================================================

struct LogicBlastScores {
    int16_t score[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t l1_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t l2_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t l3_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t bomb_unlock_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
};

static __attribute__((section(".dtcm_bss"))) LogicBlastScores logic_blast_scores;

static bool strategy_in_bounds(point p) {
    return p.x >= 0 && p.x < MAP_MAX_WIDTH && p.y >= 0 && p.y < MAP_MAX_HEIGHT;
}

static bool strategy_is_wall(const SokobanLevel& lvl, point p) {
    if (!strategy_in_bounds(p)) return true;
    return lvl.map[p.y][p.x] == 1;
}

static bool strategy_blast_covers(point center, point p) {
    return std::abs(center.x - p.x) <= 1 && std::abs(center.y - p.y) <= 1;
}

static int strategy_dir_between(point from, point to) {
    if (to.x > from.x && to.y == from.y) return 1;
    if (to.x < from.x && to.y == from.y) return 3;
    if (to.y > from.y && to.x == from.x) return 2;
    if (to.y < from.y && to.x == from.x) return 0;
    return -1;
}

static int strategy_path_turns(point start, const StaticArray<point, MAX_PATH_LENGTH>& path) {
    int turns = 0;
    int prev_dir = -1;
    point prev = start;
    for (int i = 0; i < path.size(); ++i) {
        int dir = strategy_dir_between(prev, path[i]);
        if (dir >= 0 && prev_dir >= 0 && dir != prev_dir) ++turns;
        if (dir >= 0) prev_dir = dir;
        prev = path[i];
    }
    return turns;
}

static bool strategy_bomb_line_clear(const SokobanLevel& lvl, point from, point to) {
    int dx = (to.x > from.x) - (to.x < from.x);
    int dy = (to.y > from.y) - (to.y < from.y);
    if (dx != 0 && dy != 0) return false;
    if (dx == 0 && dy == 0) return true;

    point p = {static_cast<int8_t>(from.x + dx), static_cast<int8_t>(from.y + dy)};
    while (!(p == to)) {
        if (strategy_is_wall(lvl, p)) return false;
        if (strategy_box_at(lvl, p) >= 0) return false;
        for (int b = 0; b < strategy_bomb_count(lvl); ++b) {
            if (lvl.bombs[b].x != -1 && lvl.bombs[b] == p) return false;
        }
        p = {static_cast<int8_t>(p.x + dx), static_cast<int8_t>(p.y + dy)};
    }
    return true;
}

static int strategy_task_shape_cost(
    const SokobanLevel& lvl,
    point player,
    const BombTask& task,
    const StaticArray<point, MAX_PATH_LENGTH>& path)
{
    int dx = std::abs(task.target_wall.x - task.bomb_start.x);
    int dy = std::abs(task.target_wall.y - task.bomb_start.y);
    int cost = strategy_path_turns(player, path) * 90;

    if (dx == 0 || dy == 0) {
        if (!strategy_bomb_line_clear(lvl, task.bomb_start, task.target_wall)) cost += 140;
    } else {
        cost += 360 + std::min(dx, dy) * 120;
    }

    int manhattan = dx + dy;
    int detour = path.size() - manhattan;
    if (detour > 0) cost += detour * 8;
    return cost;
}

static int strategy_wall_rank(point wall) {
    return wall.x * 2 + wall.y;
}

static void logic_clear_scores(LogicBlastScores& scores) {
    std::memset(&scores, 0, sizeof(scores));
}

static void logic_add_score(LogicBlastScores& scores, point center, int add, uint8_t layer) {
    if (!strategy_in_bounds(center)) return;
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
    point anchor = required_walls[0];
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int cx = anchor.x + dx;
            int cy = anchor.y + dy;
            if (cx <= 0 || cx >= MAP_MAX_WIDTH - 1 || cy <= 0 || cy >= MAP_MAX_HEIGHT - 1) continue;
            point center = {static_cast<int8_t>(cx), static_cast<int8_t>(cy)};
            if (lvl.map[cy][cx] != 1) continue;
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
    if (!strategy_in_bounds(box_to) || !strategy_in_bounds(push_from)) return false;
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
    if (!strategy_in_bounds(target) || strategy_is_wall(lvl, target)) return;

    static __attribute__((section(".dtcm_bss"))) point q[MAP_CELL_COUNT];
    int head = 0, tail = 0;
    out_dist[target.y][target.x] = 0;
    q[tail++] = target;

    while (head < tail) {
        point curr = q[head++];
        int16_t curr_dist = out_dist[curr.y][curr.x];
        for (int dir = 0; dir < 4; ++dir) {
            point box_prev = curr - MOVE[dir];
            point player_prev = curr - MOVE[dir] - MOVE[dir];
            if (!strategy_in_bounds(box_prev) || !strategy_in_bounds(player_prev)) continue;
            if (strategy_is_wall(lvl, box_prev) || strategy_is_wall(lvl, player_prev)) continue;
            if (out_dist[box_prev.y][box_prev.x] != INF_DIST) continue;
            out_dist[box_prev.y][box_prev.x] = curr_dist + 1;
            q[tail++] = box_prev;
        }
    }
}

template <GameMode Mode>
static void build_logic_blast_scores(
    const SokobanLevel& lvl,
    bool player_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    int16_t box_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    LogicBlastScores& scores) {
    logic_clear_scores(scores);

    bool hard_box_deadlock[MAX_BOXES] = {false};
    mark_soft_deadlock_boxes(lvl, hard_box_deadlock);

    // L1: 箱子静态死锁/无首推方向。直接从箱子周围被墙堵住的推边生成修复爆破区域。
    for (int b = 0; b < lvl.box_count; ++b) {
        point box = lvl.boxes[b];
        bool on_target = false;
        if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
            on_target = strategy_is_valid_target_for_box(lvl, b, box);
        } else {
            on_target = strategy_target_at(lvl, box);
        }

        int static_legal_push_dirs = 0;
        for (int d = 0; d < 4; ++d) {
            point box_to = box + MOVE[d];
            point push_from = box - MOVE[d];
            if (!strategy_in_bounds(box_to) || !strategy_in_bounds(push_from)) continue;
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

    // L1b: 2x2 双箱/多箱贴墙死锁。把长边墙本身投影出来，避免只看单箱首推时漏掉。
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

    // L2: 小车弱连通。寻找“可达地面 | 墙 | 不可达地面”的边界墙，并对靠近重要实体的区域加权。
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (lvl.map[y][x] != 1) continue;
            point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
            bool touches_reachable_floor = false;
            bool touches_unreachable_floor = false;
            for (int d = 0; d < 4; ++d) {
                point np = wall + MOVE[d];
                if (!strategy_in_bounds(np) || lvl.map[np.y][np.x] == 1) continue;
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

    // L3: box-target 推图桥接。F 是箱子当前可推到的区域，R 是能反向推入目标的区域；
    // 如果一条 F -> R 的推边只差墙体，就把这些墙体投影为候选爆破区域。
    static __attribute__((section(".dtcm_bss"))) int16_t reverse_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    bool target_needed[MAX_BOXES] = {false};
    for (int b = 0; b < lvl.box_count; ++b) {
        for (int t = 0; t < lvl.target_count; ++t) {
            if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                if (!strategy_target_matches_box_semantic(lvl, b, t)) continue;
            }
            point target = lvl.targets[t];
            if (box_dist[b][target.y][target.x] == INF_DIST) target_needed[t] = true;
        }
    }
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!target_needed[t]) continue;
        logic_build_reverse_push_reach(lvl, lvl.targets[t], reverse_dist[t]);
    }

    // L3a: 强封闭目标域。如果目标的反向可推入域 R 与任何箱子的正向可推达域 F
    // 都没有交集，只靠“一步桥”找不到墙；这时直接把 R 的边界墙打成候选。
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!target_needed[t]) continue;

        bool target_has_box_contact = false;
        for (int b = 0; b < lvl.box_count && !target_has_box_contact; ++b) {
            if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                if (!strategy_target_matches_box_semantic(lvl, b, t)) continue;
            }
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
                    if (!strategy_in_bounds(wall) || !strategy_in_bounds(outside)) continue;
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

    // L3b: 强封闭箱子域。箱子正向可推达域 F 与所有目标反向域都无交集时，
    // 优先打开 F 的边界墙，给后续 F/R 桥接制造入口。
    for (int b = 0; b < lvl.box_count; ++b) {
        bool box_has_reachable_target = false;
        bool box_has_applicable_target = false;
        for (int t = 0; t < lvl.target_count; ++t) {
            if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                if (!strategy_target_matches_box_semantic(lvl, b, t)) continue;
            }
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
            if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                if (!strategy_target_matches_box_semantic(lvl, b, t)) continue;
            }
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
                    if (!strategy_in_bounds(wall) || !strategy_in_bounds(outside)) continue;
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
            if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                if (!strategy_target_matches_box_semantic(lvl, b, t)) continue;
            }
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
                        if (!strategy_in_bounds(to) || !strategy_in_bounds(push_from)) continue;
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

static int phase1_key_bomb_supply_score(
    const SokobanLevel& lvl,
    int16_t bomb_dist[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    const LogicBlastScores& scores)
{
    int supply = 0;
    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (lvl.map[y][x] != 1) continue;
            int logic = scores.score[y][x];
            if (logic <= 0) continue;

            int best_dist = INF_DIST;
            for (int b = 0; b < strategy_bomb_count(lvl); ++b) {
                if (lvl.bombs[b].x == -1) continue;
                int d = bomb_dist[b][y][x];
                if (d < best_dist) best_dist = d;
            }
            if (best_dist == INF_DIST) continue;

            int capped_logic = logic > 12000 ? 12000 : logic;
            int layer_bonus =
                scores.l1_hits[y][x] * 1800 +
                scores.l2_hits[y][x] * 700 +
                scores.l3_hits[y][x] * 1100 +
                scores.bomb_unlock_hits[y][x] * 900;
            int dist_penalty = best_dist * 20;
            int cell_supply = capped_logic + layer_bonus - dist_penalty;
            if (cell_supply > 0) supply += cell_supply;
        }
    }
    return supply;
}

void StrategicPlanner::reset_profile() {
    profile = StrategyProfile{};
    active_profile_eval = nullptr;
    active_profile_pass = 0;
}

void StrategicPlanner::begin_profile_eval(uint8_t mode) {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) {
        (void)mode;
        active_profile_eval = nullptr;
        active_profile_pass = 0;
        return;
    }
    if (profile.eval_count >= StrategyConfig::PROFILE_EVAL_LIMIT) {
        ++profile.dropped_evals;
        active_profile_eval = nullptr;
        active_profile_pass = 0;
        return;
    }

    StrategyEvalProfile& eval = profile.evals[profile.eval_count++];
    eval = StrategyEvalProfile{};
    eval.mode = mode;
    eval.force_dynamic = force_phase2_dynamic ? 1 : 0;
    active_profile_eval = &eval;
    active_profile_pass = 0;
}

void StrategicPlanner::set_profile_pass(uint8_t pass) {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) {
        (void)pass;
        return;
    }
    active_profile_pass = pass < 3 ? pass : 0;
}

void StrategicPlanner::record_profile_result(uint8_t pass, const DFSResult& result) {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) {
        (void)pass;
        (void)result;
        return;
    }
    if (!active_profile_eval || pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[pass];
    p.result_deadlocks = static_cast<int16_t>(result.deadlocks_remaining);
    p.result_profit = result.net_profit;
    p.result_tasks = static_cast<uint8_t>(result.tasks.size());
}

void StrategicPlanner::record_profile_selected(uint8_t pass, const DFSResult& result) {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) {
        (void)pass;
        (void)result;
        return;
    }
    if (!active_profile_eval) return;
    active_profile_eval->selected_pass = pass;
    active_profile_eval->selected_deadlocks = static_cast<int16_t>(result.deadlocks_remaining);
    active_profile_eval->selected_profit = result.net_profit;
    active_profile_eval->selected_tasks = static_cast<uint8_t>(result.tasks.size());
}

void StrategicPlanner::record_profile_root_candidates(
    const SokobanLevel& level,
    const StaticArray<BombCandidate, 256>& candidates,
    int branch_limit) {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) {
        (void)level;
        (void)candidates;
        (void)branch_limit;
        return;
    }
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    p.root_candidates = static_cast<uint16_t>(candidates.size());
    p.root_branch_limit = static_cast<uint8_t>(branch_limit);
    p.top_count = static_cast<uint8_t>(
        candidates.size() < StrategyConfig::PROFILE_TOP_CANDIDATES ?
        candidates.size() : StrategyConfig::PROFILE_TOP_CANDIDATES);

    for (int i = 0; i < p.top_count; ++i) {
        const BombCandidate& c = candidates[i];
        StrategyCandidateProfile& top = p.top[i];
        if (c.bomb_idx < strategy_bomb_count(level)) {
            top.bomb_x = level.bombs[c.bomb_idx].x;
            top.bomb_y = level.bombs[c.bomb_idx].y;
        }
        top.wall_x = c.x;
        top.wall_y = c.y;
        int score = c.score;
        if (score > 32767) score = 32767;
        if (score < -32768) score = -32768;
        top.score = static_cast<int16_t>(score);
    }
}

void StrategicPlanner::record_profile_dfs_node() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    if (p.dfs_nodes < 65535) ++p.dfs_nodes;
}

void StrategicPlanner::record_profile_fast_bfs_call() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    if (p.fast_bfs_calls < 65535) ++p.fast_bfs_calls;
}

void StrategicPlanner::record_profile_candidate_eval() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    if (p.candidate_evals < 65535) ++p.candidate_evals;
}

void StrategicPlanner::record_profile_candidate_kept() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    if (p.candidate_kept < 65535) ++p.candidate_kept;
}

void StrategicPlanner::record_profile_child_branch() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    if (p.child_branches < 65535) ++p.child_branches;
}

void StrategicPlanner::record_profile_logic_build() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    if (p.logic_builds < 255) ++p.logic_builds;
}

void StrategicPlanner::record_profile_post_refine_test() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    StrategyPassProfile& p = active_profile_eval->passes[active_profile_pass];
    if (p.post_refine_tests < 65535) ++p.post_refine_tests;
}

void StrategicPlanner::record_profile_local_clear_call() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    ++active_profile_eval->passes[active_profile_pass].local_clear_calls;
}

void StrategicPlanner::record_profile_local_clear_success() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    ++active_profile_eval->passes[active_profile_pass].local_clear_successes;
}

void StrategicPlanner::record_profile_materialize_call() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    ++active_profile_eval->passes[active_profile_pass].materialize_calls;
}

void StrategicPlanner::record_profile_materialize_success() {
    if constexpr (!StrategyConfig::ENABLE_PROFILE) return;
    if (!active_profile_eval || active_profile_pass >= 3) return;
    ++active_profile_eval->passes[active_profile_pass].materialize_successes;
}

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

/// \brief 评估 Phase2 同语义箱-目标匹配质量
/// \param lvl 当前炸弹序列执行后的地图状态
/// \param box_dist box_dist[b][y][x] 表示第 b 个箱子被推到 (x,y) 的估计代价
/// \param out_deadlocks 输出：无法匹配到同语义目标的箱子惩罚，按 10 倍权重返回
/// \param out_distance 输出：语义约束下的最小总推动距离
///
/// \details
/// 重复语义时不能只看每个箱子的最近目标，否则两个箱子可能同时“看中”同一个目标点。
/// 这里沿用 bitmask DP，为每个箱子分配一个未使用且同语义可达的目标点。
static void evaluate_phase2_semantic_matching(
    const SokobanLevel& lvl,
    int16_t box_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
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
    for (int mask = 0; mask < mask_limit; ++mask) phase1_match_dp[mask] = assign_inf;
    phase1_match_dp[0] = 0;

    int* cur = phase1_match_dp;
    int* next = phase1_match_next;

    for (int b = 0; b < lvl.box_count; ++b) {
        for (int mask = 0; mask < mask_limit; ++mask) next[mask] = cur[mask];

        for (int mask = 0; mask < mask_limit; ++mask) {
            if (cur[mask] >= assign_inf) continue;
            for (int t = 0; t < lvl.target_count; ++t) {
                if (mask & (1 << t)) continue;
                if (!strategy_target_matches_box_semantic(lvl, b, t)) continue;

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
    for (int mask = 0; mask < mask_limit; ++mask) {
        if (cur[mask] >= assign_inf) continue;
        int matched = bit_count_u16(static_cast<uint16_t>(mask));
        if (matched > best_matched || (matched == best_matched && cur[mask] < best_distance)) {
            best_matched = matched;
            best_distance = cur[mask];
        }
    }

    if (best_matched < 0) {
        best_matched = 0;
        best_distance = 0;
    }
    out_deadlocks = (lvl.box_count - best_matched) * 10;
    out_distance = best_distance;
}

static int count_phase1_unreachable_pairs(
    const SokobanLevel& lvl,
    int16_t box_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH])
{
    int unreachable = 0;
    for (int b = 0; b < lvl.box_count; ++b) {
        for (int t = 0; t < lvl.target_count; ++t) {
            point target = lvl.targets[t];
            if (box_dist[b][target.y][target.x] == INF_DIST) ++unreachable;
        }
    }
    return unreachable;
}

static bool phase1_result_has_structural_defect(const DFSResult& result) {
    return result.deadlocks_remaining > 0 || result.unreachable_pairs_remaining > 0;
}

static bool phase1_result_better_than(
    const DFSResult& candidate,
    const DFSResult& baseline,
    int profit_margin = 0)
{
    if (candidate.deadlocks_remaining != baseline.deadlocks_remaining) {
        return candidate.deadlocks_remaining < baseline.deadlocks_remaining;
    }

    bool structural_phase =
        phase1_result_has_structural_defect(candidate) ||
        phase1_result_has_structural_defect(baseline);
    if (structural_phase) {
        if (candidate.unreachable_pairs_remaining != baseline.unreachable_pairs_remaining) {
            return candidate.unreachable_pairs_remaining < baseline.unreachable_pairs_remaining;
        }
        if (candidate.bomb_supply_score != baseline.bomb_supply_score) {
            return candidate.bomb_supply_score > baseline.bomb_supply_score;
        }
    }

    return candidate.net_profit > baseline.net_profit + profit_margin;
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

    this->begin_profile_eval(Mode == GameMode::PHASE1_ANY ? 0 : 1);
    this->cached_level = level;
    DFSResult best_res; 
    best_res.deadlocks_remaining = 9999; 
    best_res.net_profit = -999999;
    uint8_t selected_profile_pass = 0;

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
    this->set_profile_pass(0);
    this->dfs_bomb_sequence<Mode, false>(level, level.player_start, empty_seq, 0, 0, best_res);
    this->record_profile_result(0, best_res);

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
        bool hard_direct_or_empty =
            hard_res.tasks.size() == 0 ||
            this->are_fast_bomb_tasks_directly_executable(level, hard_res.tasks);
        bool run_soft_pass = hard_res.deadlocks_remaining > 0 || !hard_direct_or_empty;
        if (run_soft_pass) {
            this->phase1_soft_bomb_eval = true;
            this->set_profile_pass(1);
            this->dfs_bomb_sequence<Mode, false>(level, level.player_start, empty_seq, 0, 0, soft_res);
            this->phase1_soft_bomb_eval = false;
            this->record_profile_result(1, soft_res);
        }

        bool soft_better =
            run_soft_pass &&
            phase1_result_better_than(soft_res, hard_res, PHASE1_SOFT_REPLACE_PROFIT_MARGIN);
        if (soft_better) {
            best_res = soft_res;
            selected_soft_res = true;
            selected_profile_pass = 1;
        } else {
            best_res = hard_res;
            selected_profile_pass = 0;
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
                    selected_profile_pass = 1;
                } else {
                    if (selected_soft_res) {
                        best_res = hard_res;
                        selected_profile_pass = 0;
                    }
                    bool need_dynamic_after_failed_materialize =
                        !selected_soft_res || hard_res.tasks.size() == 0;
                    if (need_dynamic_after_failed_materialize) {
                    DFSResult dynamic_res;
                    dynamic_res.deadlocks_remaining = 9999;
                    dynamic_res.net_profit = -999999;
                    this->set_profile_pass(2);
                    this->dfs_bomb_sequence<Mode, true>(level, level.player_start, empty_seq, 0, 0, dynamic_res);
                    this->record_profile_result(2, dynamic_res);

                    if (dynamic_res.tasks.size() > 0 &&
                        phase1_result_better_than(dynamic_res, best_res)) {
                        best_res = dynamic_res;
                        selected_profile_pass = 2;
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
        selected_profile_pass = 0;
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
        debug_phase2_result(force_phase2_dynamic ? "phase2 hard retry" : "phase2 hard", hard_res);
#endif

        DFSResult soft_res;
        soft_res.deadlocks_remaining = 9999;
        soft_res.net_profit = -999999;
        bool run_soft_pass = hard_res.deadlocks_remaining > 0;
        if (run_soft_pass) {
            this->phase2_soft_bomb_eval = true;
            this->set_profile_pass(1);
            this->dfs_bomb_sequence<Mode, false>(level, level.player_start, empty_seq, 0, 0, soft_res);
            this->phase2_soft_bomb_eval = false;
            this->record_profile_result(1, soft_res);
        }
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
        debug_phase2_result(force_phase2_dynamic ? "phase2 soft retry" : "phase2 soft", soft_res);
#endif

        bool soft_better =
            run_soft_pass &&
            (soft_res.deadlocks_remaining < hard_res.deadlocks_remaining ||
             (soft_res.deadlocks_remaining == hard_res.deadlocks_remaining &&
              soft_res.net_profit > hard_res.net_profit));

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
                selected_profile_pass = 1;
            } else {
                best_res = hard_res;
                selected_profile_pass = 0;
            }
        } else {
            if (soft_better && soft_res.deadlocks_remaining < hard_res.deadlocks_remaining) {
                best_res = soft_res;
                selected_profile_pass = 1;
            } else {
                best_res = hard_res;
                selected_profile_pass = 0;
            }
        }
        // Phase2 的动态清障 DFS 非常重，不能仅因静态评估仍有 deadlock 就自动触发。
        // 主流程会先尝试 Phase2 轻量结果和 Phase1 剩余开路任务；只有这些都失败时，
        // 才通过 force_phase2_dynamic 显式进入最后兜底。
        force_dynamic = force_phase2_dynamic;
    }
    if (force_dynamic) {
        // 重置最优记录
        best_res.deadlocks_remaining = 9999; 
        best_res.net_profit = -999999;
        // 启动重型推演引擎
        this->set_profile_pass(2);
        this->dfs_bomb_sequence<Mode, true>(level, level.player_start, empty_seq, 0, 0, best_res);
        this->record_profile_result(2, best_res);
        selected_profile_pass = 2;
#ifdef STRATEGY_DEBUG_PHASE2_RESULT
        if constexpr (Mode == GameMode::PHASE2_SPECIFIC) debug_phase2_result("phase2 dynamic", best_res);
#endif
    }

    bool selected_structurally_solved = (best_res.deadlocks_remaining == 0);
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        selected_structurally_solved =
            selected_structurally_solved && best_res.unreachable_pairs_remaining == 0;
    }
    for (int i = 0; i < best_res.tasks.size(); ++i) {
        best_res.tasks[i].is_essential = selected_structurally_solved;
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

            SokobanLevel origin_after = work;
            PlanningCommon::apply_bomb_task_effect(origin_after, origin_task);
            point origin_player = origin_path.empty() ? player : origin_path.back();
            eval_phase1_pairs(origin_after, origin_player, task_idx + 1, origin_pair_dist, origin_deadlocks, origin_distance);

            BombTask best_task = origin_task;
            StaticArray<point, MAX_PATH_LENGTH> best_path = origin_path;
            int best_deadlocks = origin_deadlocks;
            int best_distance = origin_distance;
            int best_execution_score =
                PlanningCommon::path_time_cost(player, origin_path) +
                strategy_task_shape_cost(work, player, origin_task, origin_path);

            auto eval_full_suffix = [&](const BombTask& first_task,
                                        int& out_deadlocks,
                                        int& out_unreachable,
                                        int& out_distance) -> bool {
                SokobanLevel suffix_work = work;
                point suffix_player = player;

                StaticArray<point, MAX_PATH_LENGTH> path;
                if (!PlanningCommon::get_bomb_push_path(suffix_work, suffix_player, first_task, path)) {
                    return false;
                }
                if (!path.empty()) suffix_player = path.back();
                PlanningCommon::apply_bomb_task_effect(suffix_work, first_task);

                for (int next = task_idx + 1; next < best_res.tasks.size(); ++next) {
                    path.clear();
                    if (!PlanningCommon::get_bomb_push_path(suffix_work, suffix_player, best_res.tasks[next], path)) {
                        return false;
                    }
                    if (!path.empty()) suffix_player = path.back();
                    PlanningCommon::apply_bomb_task_effect(suffix_work, best_res.tasks[next]);
                }

                for (int b = 0; b < suffix_work.box_count; ++b) {
                    this->fast_push_bfs(suffix_work, suffix_work.boxes[b], suffix_player, false,
                                        candidate_pair_dist[b], true);
                }
                evaluate_phase1_any_matching(
                    suffix_work,
                    candidate_pair_dist,
                    best_res.tasks.size(),
                    out_deadlocks,
                    out_distance
                );
                out_unreachable = count_phase1_unreachable_pairs(suffix_work, candidate_pair_dist);
                return true;
            };

            int origin_final_deadlocks = 9999;
            int origin_final_unreachable = 9999;
            int origin_final_distance = 999999;
            bool origin_final_ok = eval_full_suffix(
                origin_task,
                origin_final_deadlocks,
                origin_final_unreachable,
                origin_final_distance
            );
            int best_final_deadlocks = origin_final_deadlocks;
            int best_final_unreachable = origin_final_unreachable;
            int best_final_distance = origin_final_distance;

            const bool origin_structurally_solved =
                origin_final_ok &&
                origin_final_deadlocks == 0 &&
                origin_final_unreachable == 0;
            int best_route_score = PlanningCommon::path_time_cost(player, origin_path);
#ifdef STRATEGY_DEBUG_PHASE1_REFINEMENT
            std::fprintf(stderr,
                        "P1_REFINE task=%d origin bomb=(%d,%d) wall=(%d,%d) exec=%d immediate=(%d,%d) final_ok=%d final=(%d,%d,%d)\n",
                        task_idx,
                        origin_task.bomb_start.x, origin_task.bomb_start.y,
                        origin_task.target_wall.x, origin_task.target_wall.y,
                        best_execution_score,
                        origin_deadlocks, origin_distance,
                        origin_final_ok ? 1 : 0,
                        origin_final_deadlocks, origin_final_unreachable, origin_final_distance);
#endif

            auto try_refine_wall = [&](point wall) {
                if (wall.x <= 0 || wall.x >= MAP_MAX_WIDTH - 1 ||
                    wall.y <= 0 || wall.y >= MAP_MAX_HEIGHT - 1) {
                    return;
                }
                if (work.map[wall.y][wall.x] != 1) return;
                if (wall == best_task.target_wall) return;

                this->record_profile_post_refine_test();

                BombTask candidate = origin_task;
                candidate.target_wall = wall;
                candidate.box_pushes.clear();

                StaticArray<point, MAX_PATH_LENGTH> candidate_path;
                if (!PlanningCommon::get_bomb_push_path(work, player, candidate, candidate_path)) return;

                SokobanLevel candidate_after = work;
                PlanningCommon::apply_bomb_task_effect(candidate_after, candidate);
                point candidate_player = candidate_path.empty() ? player : candidate_path.back();
                int candidate_deadlocks = 0, candidate_distance = 0;
                eval_phase1_pairs(candidate_after, candidate_player, task_idx + 1,
                                candidate_pair_dist, candidate_deadlocks, candidate_distance);

                int final_deadlocks = 9999;
                int final_unreachable = 9999;
                int final_distance = 999999;
                int candidate_route_score = PlanningCommon::path_time_cost(player, candidate_path);
                int candidate_execution_score =
                    candidate_route_score +
                    strategy_task_shape_cost(work, player, candidate, candidate_path);

                bool choose_candidate = false;
                if (origin_structurally_solved) {
                    if (!eval_full_suffix(candidate, final_deadlocks, final_unreachable, final_distance)) return;
                    bool same_solved_effect =
                        final_deadlocks == 0 &&
                        final_unreachable == 0 &&
                        std::abs(final_distance - best_final_distance) <= 3 &&
                        candidate_deadlocks <= best_deadlocks &&
                        candidate_distance <= best_distance + 3;

                    // If the structure is already solved, only allow a local
                    // same-effect retarget.  This keeps the map3_3 fix intact:
                    // a far cheaper but semantically different blast center
                    // should not replace the defect wall just because driving
                    // the bomb there is shorter.
                    if (same_solved_effect &&
                        std::abs(candidate_route_score - best_route_score) <= 8) {
                        int current_wall_rank = strategy_wall_rank(best_task.target_wall);
                        int candidate_wall_rank = strategy_wall_rank(wall);
                        choose_candidate =
                            candidate_route_score < best_route_score ||
                            (candidate_route_score <= best_route_score + 8 &&
                            candidate_wall_rank < current_wall_rank);
                    }
                } else if (origin_final_ok) {
                    if (!eval_full_suffix(candidate, final_deadlocks, final_unreachable, final_distance)) return;

                    bool better_structure =
                        final_deadlocks < best_final_deadlocks ||
                        (final_deadlocks == best_final_deadlocks &&
                        final_unreachable < best_final_unreachable);
                    bool worse_structure =
                        final_deadlocks > best_final_deadlocks ||
                        (final_deadlocks == best_final_deadlocks &&
                        final_unreachable > best_final_unreachable);
                    if (better_structure) {
                        choose_candidate = true;
                    } else if (!worse_structure) {
                        bool same_effect =
                            std::abs(final_distance - best_final_distance) <= 8;
                        bool easier_same_effect =
                            same_effect && candidate_execution_score + 10 < best_execution_score;
                        bool useful_distance_without_route_regression =
                            best_final_deadlocks == 0 &&
                            best_final_unreachable == 0 &&
                            final_distance + 20 < best_final_distance &&
                            candidate_execution_score <= best_execution_score + 10;
                        choose_candidate =
                            easier_same_effect || useful_distance_without_route_regression;
                    }
                } else {
                    bool better_structure =
                        candidate_deadlocks < best_deadlocks;
                    bool worse_structure =
                        candidate_deadlocks > best_deadlocks;
                    if (better_structure) {
                        choose_candidate = true;
                    } else if (!worse_structure) {
                        bool same_effect =
                            std::abs(candidate_distance - best_distance) <= 8;
                        choose_candidate =
                            same_effect && candidate_execution_score + 10 < best_execution_score;
                    }
                }

#ifdef STRATEGY_DEBUG_PHASE1_REFINEMENT
                std::fprintf(stderr,
                            "  cand wall=(%d,%d) exec=%d immediate=(%d,%d) final=(%d,%d,%d) choose=%d best_exec=%d best_final=(%d,%d,%d)\n",
                            wall.x, wall.y,
                            candidate_execution_score,
                            candidate_deadlocks, candidate_distance,
                            final_deadlocks, final_unreachable, final_distance,
                            choose_candidate ? 1 : 0,
                            best_execution_score,
                            best_final_deadlocks, best_final_unreachable, best_final_distance);
#endif

                if (choose_candidate) {
                    best_task = candidate;
                    best_path = candidate_path;
                    best_deadlocks = candidate_deadlocks;
                    best_distance = candidate_distance;
                    best_execution_score = candidate_execution_score;
                    best_route_score = candidate_route_score;
                    if (origin_final_ok) {
                        best_final_deadlocks = final_deadlocks;
                        best_final_unreachable = final_unreachable;
                        best_final_distance = final_distance;
                    }
                }
            };

            for (int dy = -3; dy <= 3; ++dy) {
                for (int dx = -3; dx <= 3; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    point wall = {
                        static_cast<int8_t>(origin_task.target_wall.x + dx),
                        static_cast<int8_t>(origin_task.target_wall.y + dy)
                    };
                    try_refine_wall(wall);
                }
            }

            for (int dir = 0; dir < 4; ++dir) {
                point wall = origin_task.bomb_start + MOVE[dir];
                while (wall.x > 0 && wall.x < MAP_MAX_WIDTH - 1 &&
                        wall.y > 0 && wall.y < MAP_MAX_HEIGHT - 1 &&
                        work.map[wall.y][wall.x] != 1) {
                    wall = wall + MOVE[dir];
                }
                try_refine_wall(wall);
            }

            best_task.is_essential = origin_task.is_essential;
            best_task.net_profit = origin_task.net_profit;
            best_res.tasks[task_idx] = best_task;
            if (!best_path.empty()) player = best_path.back();
            PlanningCommon::apply_bomb_task_effect(work, best_task);
        }

        int task_count = best_res.tasks.size();
        if (task_count > 1 && task_count <= MAX_BOMBS) {
            point live_bombs[MAX_BOMBS];
            int live_count = 0;
            for (int b = 0; b < strategy_bomb_count(level); ++b) {
                if (level.bombs[b].x != -1 && live_count < MAX_BOMBS) {
                    live_bombs[live_count++] = level.bombs[b];
                }
            }

            if (live_count >= task_count) {
                BombTask original_tasks[MAX_BOMBS];
                for (int i = 0; i < task_count; ++i) original_tasks[i] = best_res.tasks[i];

                BombTask best_tasks[MAX_BOMBS];
                BombTask trial_tasks[MAX_BOMBS];
                int assignment[MAX_BOMBS] = {-1, -1, -1};
                bool used_bomb[MAX_BOMBS] = {false, false, false};
                bool have_best_assignment = false;
                int best_assignment_score = 2147483647;
                int original_assignment_score = 2147483647;

                auto segment_cross_penalty = [](point a, point b, point c, point d) {
                    auto orient = [](point p, point q, point r) {
                        int v = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
                        if (v > 0) return 1;
                        if (v < 0) return -1;
                        return 0;
                    };
                    auto between = [](int a0, int b0, int c0) {
                        return (a0 <= b0 && b0 <= c0) || (c0 <= b0 && b0 <= a0);
                    };
                    auto on_segment = [&](point p, point q, point r) {
                        return orient(p, q, r) == 0 &&
                               between(p.x, q.x, r.x) &&
                               between(p.y, q.y, r.y);
                    };

                    if (a == c || a == d || b == c || b == d) return 0;
                    int o1 = orient(a, b, c);
                    int o2 = orient(a, b, d);
                    int o3 = orient(c, d, a);
                    int o4 = orient(c, d, b);
                    bool crosses = (o1 != o2 && o3 != o4) ||
                                   on_segment(a, c, b) ||
                                   on_segment(a, d, b) ||
                                   on_segment(c, a, d) ||
                                   on_segment(c, b, d);
                    return crosses ? 2000 : 0;
                };

                auto evaluate_assignment = [&](BombTask out_tasks[MAX_BOMBS], int& out_score) -> bool {
                    SokobanLevel eval_work = level;
                    point eval_player = level.player_start;
                    int score = 0;

                    for (int i = 0; i < task_count; ++i) {
                        BombTask task = original_tasks[i];
                        task.bomb_start = live_bombs[assignment[i]];
                        task.box_pushes.clear();

                        if (eval_work.map[task.target_wall.y][task.target_wall.x] != 1) return false;

                        StaticArray<point, MAX_PATH_LENGTH> path;
                        if (!PlanningCommon::get_bomb_push_path(eval_work, eval_player, task, path)) {
                            return false;
                        }

                        score += PlanningCommon::path_time_cost(eval_player, path);
                        score += strategy_task_shape_cost(eval_work, eval_player, task, path);
                        score += std::abs(task.bomb_start.x - task.target_wall.x) +
                                 std::abs(task.bomb_start.y - task.target_wall.y);
                        if (!path.empty()) eval_player = path.back();
                        PlanningCommon::apply_bomb_task_effect(eval_work, task);
                        out_tasks[i] = task;
                    }

                    for (int i = 0; i < task_count; ++i) {
                        for (int j = i + 1; j < task_count; ++j) {
                            score += segment_cross_penalty(
                                out_tasks[i].bomb_start, out_tasks[i].target_wall,
                                out_tasks[j].bomb_start, out_tasks[j].target_wall
                            );
                        }
                    }

                    out_score = score;
                    return true;
                };

                auto enumerate_assignments = [&](auto&& self, int idx) -> void {
                    if (idx == task_count) {
                        int score = 0;
                        if (!evaluate_assignment(trial_tasks, score)) return;

                        bool is_original = true;
                        for (int i = 0; i < task_count; ++i) {
                            if (!(trial_tasks[i].bomb_start == original_tasks[i].bomb_start)) {
                                is_original = false;
                                break;
                            }
                        }
                        if (is_original) original_assignment_score = score;

                        if (score < best_assignment_score) {
                            best_assignment_score = score;
                            have_best_assignment = true;
                            for (int i = 0; i < task_count; ++i) best_tasks[i] = trial_tasks[i];
                        }
                        return;
                    }

                    for (int b = 0; b < live_count; ++b) {
                        if (used_bomb[b]) continue;
                        used_bomb[b] = true;
                        assignment[idx] = b;
                        self(self, idx + 1);
                        assignment[idx] = -1;
                        used_bomb[b] = false;
                    }
                };

                enumerate_assignments(enumerate_assignments, 0);

                if (have_best_assignment &&
                    (original_assignment_score == 2147483647 ||
                     best_assignment_score + 4 < original_assignment_score)) {
                    for (int i = 0; i < task_count; ++i) {
                        best_tasks[i].is_essential = original_tasks[i].is_essential;
                        best_tasks[i].net_profit = original_tasks[i].net_profit;
                        best_res.tasks[i] = best_tasks[i];
                    }
                }
            }
        }

    }
    this->record_profile_selected(selected_profile_pass, best_res);
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
    this->record_profile_dfs_node();
    const int current_bomb_count = strategy_bomb_count(current_lvl);
    // =====================================================================
    // 1. 评估当前状态并更新全局最优结果
    // =====================================================================
    int current_deadlocks = 0;  // 当前状态死锁数量
    int current_distance = 0;   // 当前状态所有箱子到目标的总距离（作为成本评估的一部分）

    PlanningCommon::calc_player_reach(current_lvl, player_start, {-1,-1}, {-1,-1}, dfs_player_vis[depth]);

    // 计算每个箱子到目标的距离。Phase1 保留未使用炸弹作为当前障碍，
    // 但候选爆破评估不会因“选中哪颗炸弹”而把该炸弹原地删除刷收益。
    for (int b = 0; b < current_lvl.box_count; ++b) {
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, dfs_dist_box[depth][b], false);
            this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, phase1_soft_dist_box[depth][b], true);
        } else {
            this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, dfs_dist_box[depth][b], this->phase2_soft_bomb_eval);
        }
    }
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        evaluate_phase1_any_matching(current_lvl, phase1_soft_dist_box[depth], current_seq.size(), current_deadlocks, current_distance);
    } else {
        evaluate_phase2_semantic_matching(current_lvl, dfs_dist_box[depth], current_deadlocks, current_distance);
    }
    int current_unreachable_pairs = 9999;
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        current_unreachable_pairs = count_phase1_unreachable_pairs(current_lvl, phase1_soft_dist_box[depth]);
    }

    bool terminal_node = current_seq.size() == current_bomb_count || depth >= MAX_BOMBS;
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        if (terminal_node && current_deadlocks > 0) return;
    }

    // 净收益评估：距离越短越好，已选炸弹越多（成本越高）越差
    // Phase1 先解决任意匹配的残局质量，避免短推炸弹压过真正打开箱-目标通路的墙
    int profit = -current_distance - cost_so_far;
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        profit = -current_distance - cost_so_far;
    }
    // 更新全局最优结果 [优先级：死锁数量（越少越好）> 净收益（越高越好）]
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        bool allow_update = !(force_phase2_dynamic && current_seq.size() == 0 && current_bomb_count > 0);
        if (allow_update &&
        (current_deadlocks < best_res.deadlocks_remaining ||
        (current_deadlocks == best_res.deadlocks_remaining && profit > best_res.net_profit))) {
            best_res.deadlocks_remaining = current_deadlocks;
            best_res.net_profit = profit;
            best_res.tasks = current_seq;
        }
    }

    // 递归边界：如果已选炸弹数量达到上限或没有更多炸弹可选，则返回
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        if (terminal_node) return;
    }


    // =====================================================================
    // 2. 计算存活炸弹可达爆破点
    // =====================================================================
    for (int m = 0; m < current_bomb_count; ++m) {
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

    // 前三层启用缺陷驱动逻辑层。死锁未清时，后续炸弹是否有价值
    // 主要取决于它是否继续打开关键结构，而不是普通距离缩短。
    const bool use_logic_scores = (depth <= 2);
    if (use_logic_scores) {
        this->record_profile_logic_build();
        build_logic_blast_scores<Mode>(
            current_lvl,
            dfs_player_vis[depth],
            Mode == GameMode::PHASE1_ANY ? phase1_soft_dist_box[depth] : dfs_dist_box[depth],
            logic_blast_scores
        );
    }
    int phase1_current_supply_score = 0;
    bool phase1_structural_defect_active = false;
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        phase1_structural_defect_active = current_deadlocks > 0 || current_unreachable_pairs > 0;
        phase1_current_supply_score =
            (phase1_structural_defect_active && use_logic_scores)
                ? phase1_key_bomb_supply_score(current_lvl, dfs_dist_bomb[depth], logic_blast_scores)
                : 0;

        bool better = false;
        if (current_deadlocks < best_res.deadlocks_remaining) {
            better = true;
        } else if (current_deadlocks == best_res.deadlocks_remaining) {
            if (phase1_structural_defect_active) {
                if (current_unreachable_pairs < best_res.unreachable_pairs_remaining) {
                    better = true;
                } else if (current_unreachable_pairs == best_res.unreachable_pairs_remaining &&
                           phase1_current_supply_score > best_res.bomb_supply_score) {
                    better = true;
                }
            } else if (profit > best_res.net_profit) {
                better = true;
            }
        }

        if (better) {
            best_res.deadlocks_remaining = current_deadlocks;
            best_res.net_profit = profit;
            best_res.unreachable_pairs_remaining = current_unreachable_pairs;
            best_res.bomb_supply_score = phase1_current_supply_score;
            best_res.tasks = current_seq;
        }

        if (terminal_node) return;
    }
    // 建立候选动作队列，避免无脑展开过多分支
    StaticArray<BombCandidate, 256>& candidates = strategy_dfs_ws.candidates[depth];
    StaticArray<BombCandidate, 256>& preliminary = strategy_dfs_ws.preliminary[depth];
    candidates.clear();
    preliminary.clear();
    std::memset(strategy_dfs_ws.probe_valid[depth], 0, sizeof(strategy_dfs_ws.probe_valid[depth]));

    int selection_limit = 10;
    if constexpr (Mode == GameMode::PHASE1_ANY) selection_limit = PHASE1_SELECTION_RESTRICTIONS;
    else if constexpr (Mode == GameMode::PHASE2_SPECIFIC) selection_limit = PHASE2_SELECTION_RESTRICTIONS;
    int heavy_eval_limit = 255;
    if (depth == 1) heavy_eval_limit = 12;
    else if (depth > 1) heavy_eval_limit = selection_limit * 2;

    // =====================================================================
    // 3. 按缺陷类型生成候选，并用一次真实爆破评估过滤
    // =====================================================================
    bool structural_defect_active = current_deadlocks > 0;
    if constexpr (Mode == GameMode::PHASE1_ANY) {
        structural_defect_active = phase1_structural_defect_active;
    }

    auto apply_probe_blast = [](SokobanLevel& lvl, point wall) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int ny = wall.y + dy;
                int nx = wall.x + dx;
                if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 &&
                    nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                    lvl.map[ny][nx] = 0;
                }
            }
        }
    };

    auto eval_probe_state = [&](const SokobanLevel& lvl,
                                point eval_player,
                                int selected_count,
                                int& out_deadlocks,
                                int& out_unreachable,
                                int& out_distance) {
        out_deadlocks = 0;
        out_unreachable = 9999;
        out_distance = 0;

        if constexpr (Mode == GameMode::PHASE1_ANY) {
            for (int b = 0; b < lvl.box_count; ++b) {
                this->fast_push_bfs(lvl, lvl.boxes[b], eval_player, false,
                                    phase1_candidate_dist[b], true);
            }
            evaluate_phase1_any_matching(
                lvl,
                phase1_candidate_dist,
                selected_count,
                out_deadlocks,
                out_distance
            );
            out_unreachable = count_phase1_unreachable_pairs(lvl, phase1_candidate_dist);
        } else {
            for (int b = 0; b < lvl.box_count; ++b) {
                this->fast_push_bfs(lvl, lvl.boxes[b], eval_player, false,
                                    phase1_candidate_dist[b], this->phase2_soft_bomb_eval);
            }
            evaluate_phase2_semantic_matching(lvl, phase1_candidate_dist, out_deadlocks, out_distance);
        }
    };

    auto eval_probe_supply = [&](const SokobanLevel& lvl, point eval_player) -> int {
        int supply = 0;
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            if (!phase1_structural_defect_active || !use_logic_scores) return 0;
            for (int b = 0; b < strategy_bomb_count(lvl); ++b) {
                if (lvl.bombs[b].x == -1) continue;
                this->fast_push_bfs(
                    lvl,
                    lvl.bombs[b],
                    eval_player,
                    true,
                    phase1_candidate_bomb_dist[b],
                    this->phase1_soft_bomb_eval
                );
            }
            supply = phase1_key_bomb_supply_score(
                lvl,
                phase1_candidate_bomb_dist,
                logic_blast_scores
            );
        }
        return supply;
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
        if (x <= 0 || x >= MAP_MAX_WIDTH - 1 || y <= 0 || y >= MAP_MAX_HEIGHT - 1) return;
        if (current_lvl.map[y][x] != 1 || dfs_dist_bomb[depth][m][y][x] == INF_DIST) return;

        this->record_profile_candidate_eval();

        int logic_score = use_logic_scores ? logic_blast_scores.score[y][x] : 0;
        int l1_hits = use_logic_scores ? logic_blast_scores.l1_hits[y][x] : 0;
        int l2_hits = use_logic_scores ? logic_blast_scores.l2_hits[y][x] : 0;
        int l3_hits = use_logic_scores ? logic_blast_scores.l3_hits[y][x] : 0;
        int supply_hits = use_logic_scores ? logic_blast_scores.bomb_unlock_hits[y][x] : 0;
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
                if (current_lvl.map[ny][nx] == 0 && !dfs_player_vis[depth][ny][nx]) {
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
        int after_unreachable = 9999;
        int after_distance = 0;
        if (strategy_dfs_ws.probe_valid[depth][y][x]) {
            after_deadlocks = strategy_dfs_ws.probe_deadlocks[depth][y][x];
            after_unreachable = strategy_dfs_ws.probe_unreachable[depth][y][x];
            after_distance = strategy_dfs_ws.probe_distance[depth][y][x];
        } else {
            SokobanLevel probe_lvl = current_lvl;
            point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
            apply_probe_blast(probe_lvl, wall);
            eval_probe_state(
                probe_lvl,
                wall,
                current_seq.size() + 1,
                after_deadlocks,
                after_unreachable,
                after_distance
            );
            strategy_dfs_ws.probe_valid[depth][y][x] = true;
            strategy_dfs_ws.probe_deadlocks[depth][y][x] = after_deadlocks;
            strategy_dfs_ws.probe_unreachable[depth][y][x] = after_unreachable;
            strategy_dfs_ws.probe_distance[depth][y][x] = after_distance;
        }

        int deadlock_gain = current_deadlocks - after_deadlocks;
        int unreachable_gain = 0;
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            unreachable_gain = current_unreachable_pairs - after_unreachable;
        }
        int distance_gain = current_distance - after_distance;

        int supply_gain = 0;
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            if (current_seq.size() + 1 < current_bomb_count) {
                SokobanLevel probe_lvl = current_lvl;
                point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
                apply_probe_blast(probe_lvl, wall);
                SokobanLevel supply_lvl = probe_lvl;
                if (m < strategy_bomb_count(supply_lvl)) supply_lvl.bombs[m] = {-1, -1};
                supply_gain = eval_probe_supply(supply_lvl, wall) - phase1_current_supply_score;
            }
        }

        bool direct_fix = deadlock_gain > 0;
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            direct_fix = direct_fix || unreachable_gain > 0;
        }
        bool supply_fix = supply_gain > 120;
        bool non_regressing_key_defect = key_defect_wall && deadlock_gain >= 0;
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            non_regressing_key_defect = non_regressing_key_defect && unreachable_gain >= 0;
        }

        bool keep = direct_fix || supply_fix || non_regressing_key_defect;
        if (!structural_defect_active) {
            keep = keep || distance_gain > 0 || opens_unreachable_floor || entity_touch_score > 0;
        }
        if (!keep) return;

        int score = 0;
        score += deadlock_gain * 900000;
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            score += unreachable_gain * 70000;
        }
        score += l1_hits * 50000;
        score += l3_hits * 36000;
        score += l2_hits * 24000;
        score += supply_hits * 18000;
        score += logic_score;

        if (opens_unreachable_floor) score += 9000 + weak_open_score;
        if (supply_gain > 0) score += 22000 + std::min(supply_gain, 8000) * 5;
        else if (supply_gain < 0 && structural_defect_active) score += supply_gain * 2;

        if (distance_gain > 0) score += distance_gain * (structural_defect_active ? 14 : 32);
        else score += distance_gain * (structural_defect_active ? 6 : 18);

        score += wall_mass * 260 + entity_touch_score;
        score -= dfs_dist_bomb[depth][m][y][x] * (structural_defect_active ? 12 : 18);

        if (structural_defect_active && !direct_fix && !supply_fix && logic_score <= 0) {
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
                if (current_lvl.map[y][x] != 1 || dfs_dist_bomb[depth][m][y][x] == INF_DIST) {
                    continue;
                }
                int logic_score = use_logic_scores ? logic_blast_scores.score[y][x] : 0;
                int l1_hits = use_logic_scores ? logic_blast_scores.l1_hits[y][x] : 0;
                int l2_hits = use_logic_scores ? logic_blast_scores.l2_hits[y][x] : 0;
                int l3_hits = use_logic_scores ? logic_blast_scores.l3_hits[y][x] : 0;
                int supply_hits = use_logic_scores ? logic_blast_scores.bomb_unlock_hits[y][x] : 0;
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
                        if (current_lvl.map[ny][nx] == 0 && !dfs_player_vis[depth][ny][nx]) {
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
                cheap_score -= dfs_dist_bomb[depth][m][y][x] * (structural_defect_active ? 12 : 18);
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

    // =====================================================================
    // 4. 对候选动作进行排序并限制分支数量，进入下一层递归
    // =====================================================================

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
                existing_dist = dfs_dist_bomb[depth][existing.bomb_idx][y][x];
            }
            if (candidates[i].bomb_idx < current_bomb_count) {
                candidate_dist = dfs_dist_bomb[depth][candidates[i].bomb_idx][y][x];
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
    if (depth == 0) {
        this->record_profile_root_candidates(current_lvl, candidates, branch_limit);
    }

    if constexpr (!Dynamic) {
        // -----------------------------------------------------
        // 【极速模式执行路径】
        // -----------------------------------------------------
        for (int i = 0; i < branch_limit; ++i) {
            BombCandidate c = candidates[i];
            int m = c.bomb_idx;
            if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) continue;
            
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
            BombTask next_task;
            next_task.bomb_start = current_lvl.bombs[m];
            next_task.target_wall = {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)};
            next_task.is_essential = false;
            next_task.net_profit = 0;
            next_task.box_pushes.clear();
            next_seq.push_back(next_task);
            
            int execution_cost = dfs_dist_bomb[depth][m][c.y][c.x] * 1.5f; 
            if constexpr (Mode == GameMode::PHASE1_ANY) {
                if (this->phase1_soft_bomb_eval && current_deadlocks > 0 && execution_cost > 10) {
                    execution_cost = 10 + (execution_cost - 10) / 4;
                }
            }

            // 下一层递归
            this->record_profile_child_branch();
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
            if (c.bomb_idx >= current_bomb_count || current_lvl.bombs[c.bomb_idx].x == -1) continue;
            
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
            this->record_profile_child_branch();
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
    if (soft_boxes) mark_soft_deadlock_boxes(lvl, soft_hard_box);

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
/// \param phase2_specific true 表示按第二阶段语义规则做安全检查
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
    this->record_profile_local_clear_call();
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
            return strategy_is_valid_target_for_box(work, box_id, p);
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
        if (cheb(target_wall, box_pos) <= 2) return true;
        if (phase2_specific) {
            for (int t = 0; t < lvl.target_count; ++t) {
                if (strategy_target_matches_box_semantic(lvl, box_id, t) &&
                    cheb(target_wall, lvl.targets[t]) <= 2) {
                    return true;
                }
            }
        }

        for (int b = 0; b < lvl.bomb_count; ++b) {
            if (b == bomb_idx || lvl.bombs[b].x == -1) continue;
            this->macro_soft_dijkstra(lvl, lvl.bombs[b], soft_dist);
            for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
                for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                    if (lvl.map[y][x] != 1 || soft_dist[y][x] == INF_DIST) continue;
                    point wall = {(int8_t)x, (int8_t)y};
                    if (cheb(wall, box_pos) <= 2) return true;
                    if (phase2_specific) {
                        for (int t = 0; t < lvl.target_count; ++t) {
                            if (strategy_target_matches_box_semantic(lvl, box_id, t) &&
                                cheb(wall, lvl.targets[t]) <= 2) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    };

    auto box_position_is_safe = [&](const SokobanLevel& lvl, int box_id, point check_player) -> bool {
        point box_pos = lvl.boxes[box_id];
        bool on_valid_target = false;
        if (phase2_specific) {
            on_valid_target = strategy_is_valid_target_for_box(lvl, box_id, box_pos);
        } else {
            for (int t = 0; t < lvl.target_count; ++t) {
                if (lvl.targets[t] == box_pos) on_valid_target = true;
            }
        }

        this->fast_push_bfs(lvl, box_pos, check_player, false, box_dist, true);
        bool can_reach_goal = false;
        if (phase2_specific) {
            can_reach_goal = strategy_has_reachable_semantic_goal(lvl, box_id, box_dist);
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
            if (phase2_specific) {
                return strategy_nearest_semantic_goal_distance(work, box_id, p);
            }
            int best = 99;
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
            this->record_profile_local_clear_success();
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
                        if (phase2_specific) {
                            return strategy_nearest_semantic_goal_distance(lvl, b, p);
                        }
                        int best = 99;
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
            this->record_profile_local_clear_success();
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
    this->record_profile_materialize_call();
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
    this->record_profile_materialize_success();
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
