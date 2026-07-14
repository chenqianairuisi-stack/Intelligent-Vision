#include "Strategy.h"
#include <cstring>
#include <algorithm>

using namespace StrategyConfig;

static StrategySearchWorkspace& strategy_ws = strategy_search_workspace();

// ============================================================================
// 1. 缺陷驱动逻辑层：把“不可达/死锁”投影为可修复它的 3x3 爆破区域
// ============================================================================

// 第一阶段评分不变量
// - 不使用 `box_semantics` / `target_semantics` 给候选墙体加权
// - 死锁减少、箱子到目标可达性、全对偶推动距离 是主结构收益
// - 炸弹真实执行代价只做阻尼，不能重新成为主目标
// - `supplyGain` 只提示未来炸弹资源，已闭环或有直接结构收益时必须保持弱权重
// - 同一墙位去重必须保留被选炸弹自己的评分，不能混用旧候选分数

// 1.1 候选评分辅助：连通入口、未来供给和真实执行代价阻尼
static bool strategy_is_wall(const SokobanLevel& lvl, point p) {
    if (!PlanningCommon::in_bounds(p)) return true;
    return lvl.map[p.y][p.x] == 1;
}

static bool strategy_blast_footprint_cell(point center, point p) {
    return std::abs(center.x - p.x) <= 1 && std::abs(center.y - p.y) <= 1;
}

static int phase1_gateway_hint_score(
    const SokobanLevel& lvl,
    bool player_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    point wall)
{
    bool touches_reachable = false;
    bool touches_unreachable = false;
    int opened_walls = 0;
    int resource_score = 0;

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            point p = {
                static_cast<int8_t>(wall.x + dx),
                static_cast<int8_t>(wall.y + dy)
            };
            if (!PlanningCommon::is_inner_map_cell(p)) continue;
            if (lvl.map[p.y][p.x] == 1) ++opened_walls;
            if (lvl.map[p.y][p.x] == 0) {
                if (player_vis[p.y][p.x]) touches_reachable = true;
                else touches_unreachable = true;
            }

            for (int d = 0; d < 4; ++d) {
                point np = p + MOVE[d];
                if (!PlanningCommon::in_bounds(np)) continue;
                if (lvl.map[np.y][np.x] == 0) {
                    if (player_vis[np.y][np.x]) touches_reachable = true;
                    else touches_unreachable = true;
                }
            }
        }
    }

    if (!touches_reachable || !touches_unreachable || opened_walls == 0) return 0;

    for (int b = 0; b < strategy_bomb_count(lvl); ++b) {
        if (lvl.bombs[b].x == -1) continue;
        int dist = std::max(std::abs(lvl.bombs[b].x - wall.x), std::abs(lvl.bombs[b].y - wall.y));
        if (dist <= 4) resource_score += (5 - dist) * 1200;
    }
    for (int b = 0; b < lvl.box_count; ++b) {
        int dist = std::max(std::abs(lvl.boxes[b].x - wall.x), std::abs(lvl.boxes[b].y - wall.y));
        if (dist <= 4) resource_score += (5 - dist) * 500;
    }
    for (int t = 0; t < lvl.target_count; ++t) {
        int dist = std::max(std::abs(lvl.targets[t].x - wall.x), std::abs(lvl.targets[t].y - wall.y));
        if (dist <= 4) resource_score += (5 - dist) * 450;
    }

    return 18000 + opened_walls * 900 + resource_score;
}

static int phase1_gateway_resource_score(
    const SokobanLevel& before_lvl,
    const SokobanLevel& after_lvl,
    bool before_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    point after_player,
    const LogicBlastScores& scores)
{
    static bool after_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    PlanningCommon::calc_player_reach(after_lvl, after_player, {-1, -1}, {-1, -1}, after_vis);

    int new_cells = 0;
    int score = 0;
    bool has_key_resource = false;

    for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
        for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
            if (!after_vis[y][x] || before_vis[y][x] || after_lvl.map[y][x] == 1) continue;
            ++new_cells;
            point cell = {static_cast<int8_t>(x), static_cast<int8_t>(y)};

            for (int b = 0; b < strategy_bomb_count(after_lvl); ++b) {
                if (after_lvl.bombs[b].x == -1) continue;
                int md = std::abs(after_lvl.bombs[b].x - x) + std::abs(after_lvl.bombs[b].y - y);
                if (md <= 1) {
                    score += 22000;
                    has_key_resource = true;
                } else if (md <= 3) {
                    score += (4 - md) * 2200;
                }
            }

            for (int b = 0; b < before_lvl.box_count; ++b) {
                int md = std::abs(before_lvl.boxes[b].x - x) + std::abs(before_lvl.boxes[b].y - y);
                if (md <= 1) {
                    score += 4500;
                    has_key_resource = true;
                }
            }
            for (int t = 0; t < before_lvl.target_count; ++t) {
                int md = std::abs(before_lvl.targets[t].x - x) + std::abs(before_lvl.targets[t].y - y);
                if (md <= 1) {
                    score += 3600;
                    has_key_resource = true;
                }
            }

            for (int d = 0; d < 4; ++d) {
                point np = cell + MOVE[d];
                if (!PlanningCommon::in_bounds(np) || after_lvl.map[np.y][np.x] != 1) continue;
                int logic = scores.score[np.y][np.x];
                if (logic <= 0) continue;
                int capped_logic = logic > 12000 ? 12000 : logic;
                score += capped_logic +
                         scores.l1_hits[np.y][np.x] * 2200 +
                         scores.l2_hits[np.y][np.x] * 900 +
                         scores.l3_hits[np.y][np.x] * 1400 +
                         scores.bomb_unlock_hits[np.y][np.x] * 1800;
                has_key_resource = true;
            }
        }
    }

    if (new_cells == 0 || !has_key_resource) return 0;
    score += (new_cells > 30 ? 30 : new_cells) * 180;
    return score;
}

static bool strategy_blast_covers(point center, point p) {
    return strategy_blast_footprint_cell(center, p);
}

static int phase1_ranked_bomb_effort(int topology_cost, int direct_cost) {
    if (topology_cost >= INF_DIST) return INF_DIST;
    if (direct_cost >= INF_DIST) return topology_cost * 8 + 80;
    int extra_cost = direct_cost - topology_cost;
    if (extra_cost < 0) extra_cost = 0;
    return topology_cost + extra_cost / 6;
}

struct Phase1CandidateLocalFeatures {
    int weak_open_score = 0;
    int wall_mass = 0;
    int entity_touch_score = 0;
    bool opens_unreachable_floor = false;
};

static Phase1CandidateLocalFeatures phase1_candidate_local_features(
    const SokobanLevel& lvl,
    bool player_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    int ignore_bomb,
    point wall)
{
    Phase1CandidateLocalFeatures features;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int ny = wall.y + dy;
            int nx = wall.x + dx;
            if (ny <= 0 || ny >= MAP_MAX_HEIGHT - 1 ||
                nx <= 0 || nx >= MAP_MAX_WIDTH - 1) {
                continue;
            }
            if (lvl.map[ny][nx] == 1) ++features.wall_mass;
            if (lvl.map[ny][nx] == 0 && !player_vis[ny][nx]) {
                features.opens_unreachable_floor = true;
                features.weak_open_score += 700;
            }
            for (int b = 0; b < lvl.box_count; ++b) {
                if (lvl.boxes[b].x == nx && lvl.boxes[b].y == ny) {
                    features.entity_touch_score += 900;
                }
            }
            for (int t = 0; t < lvl.target_count; ++t) {
                if (lvl.targets[t].x == nx && lvl.targets[t].y == ny) {
                    features.entity_touch_score += 700;
                }
            }
            if (PlanningCommon::has_entity(lvl, nx, ny, ignore_bomb)) {
                features.entity_touch_score += 200;
            }
        }
    }
    return features;
}

static void phase1_apply_probe_bomb_transition(SokobanLevel& lvl, int bomb_idx, point wall) {
    if (bomb_idx >= 0 && bomb_idx < lvl.bomb_count) {
        lvl.bombs[bomb_idx] = {-1, -1};
    }
    PlanningCommon::apply_blast_effect(lvl, wall);
}

static void phase1_keep_ranked_candidate(
    StaticArray<BombCandidate, 256>& pool,
    BombCandidate candidate)
{
    if (pool.size() < 255) {
        pool.push_back(candidate);
        return;
    }

    int worst = 0;
    for (int i = 1; i < pool.size(); ++i) {
        if (pool[i].score < pool[worst].score) worst = i;
    }
    if (candidate.score > pool[worst].score) {
        pool[worst] = candidate;
    }
}

static int phase1_blast_footprint_overlap(const BombCandidate& a, const BombCandidate& b) {
    int dx = std::abs(static_cast<int>(a.x) - static_cast<int>(b.x));
    int dy = std::abs(static_cast<int>(a.y) - static_cast<int>(b.y));
    if (dx >= 3 || dy >= 3) return 0;
    return (3 - dx) * (3 - dy);
}

static bool phase1_candidate_bomb_used_before(
    const StaticArray<BombCandidate, 256>& candidates,
    int slot,
    uint8_t bomb_idx)
{
    for (int s = 0; s < slot; ++s) {
        if (candidates[s].bomb_idx == bomb_idx) return true;
    }
    return false;
}

static bool phase1_candidate_overlaps_before(
    const StaticArray<BombCandidate, 256>& candidates,
    int slot,
    const BombCandidate& c)
{
    for (int s = 0; s < slot; ++s) {
        if (phase1_blast_footprint_overlap(c, candidates[s]) >= 4) return true;
    }
    return false;
}

// 1.2 缺陷投影辅助：把死锁、断联和桥接需求累计到墙位分数图
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

static void build_phase1_logic_blast_scores(
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
        bool on_target = strategy_is_goal_for_box(lvl, b, box, false);

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

    // L3: box-target 推图桥接。F 是箱子当前可推到的区域，R 是能反向推入目标的区域；
    // 如果一条 F -> R 的推边只差墙体，就把这些墙体投影为候选爆破区域。
    OCRAM_BSS static int16_t reverse_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    bool target_needed[MAX_BOXES] = {false};
    for (int b = 0; b < lvl.box_count; ++b) {
        for (int t = 0; t < lvl.target_count; ++t) {
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

    // L3b: 强封闭箱子域。箱子正向可推达域 F 与所有目标反向域都无交集时，
    // 优先打开 F 的边界墙，给后续 F/R 桥接制造入口。
    for (int b = 0; b < lvl.box_count; ++b) {
        bool box_has_reachable_target = false;
        bool box_has_applicable_target = false;
        for (int t = 0; t < lvl.target_count; ++t) {
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


// ============================================================================
// 2. Phase1 任意匹配评估与搜索结果比较
// ============================================================================

static inline int bit_count_u16(uint16_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 1);
        mask >>= 1;
    }
    return count;
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

    // DP 状态：cur[mask] 表示已经使用 mask 中目标时的最小总推动距离
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
    if (lvl.box_count == 0) {
        out_deadlocks = 0;
        out_distance = 0;
        return;
    }

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

    int best_matched = 0;
    int best_distance = 0;
    evaluate_matching_dp_core(
        lvl,
        box_dist,
        [](int, int) { return true; },
        best_matched,
        best_distance
    );
    out_deadlocks = lvl.box_count - best_matched;

    // 选了多个炸弹后，策略更偏向真实打开通路，因此适当提高不可达惩罚的权重
    int pair_divisor = selected_task_count >= 2 ? 4 : 6;
    int unreachable_penalty = selected_task_count >= 2 ? 15 : 10;
    out_distance = best_distance + all_pair_distance / pair_divisor + unreachable_pairs * unreachable_penalty + out_deadlocks * 1000;
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

static bool phase1_result_is_complete(const DFSResult& result) {
    return result.deadlocks_remaining == 0 && result.unreachable_pairs_remaining == 0;
}

struct Phase1SequenceEval {
    StaticArray<BombTask, MAX_BOMBS> tasks;
    bool ok = false;
    int deadlocks = 9999;
    int unreachable = 9999;
    int distance = 999999;
    int cost = 999999;
    int unresolved_obligations = 9999;
};

static int phase1_sequence_clear_push_count(const StaticArray<BombTask, MAX_BOMBS>& tasks) {
    int pushes = 0;
    for (int i = 0; i < tasks.size(); ++i) {
        pushes += tasks[i].box_pushes.size();
    }
    return pushes;
}

static bool phase1_sequence_eval_complete(const Phase1SequenceEval& eval) {
    return eval.ok &&
           eval.unresolved_obligations == 0 &&
           eval.deadlocks == 0 &&
           eval.unreachable == 0;
}

static bool phase1_sequence_eval_better_than(
    const Phase1SequenceEval& candidate,
    const Phase1SequenceEval& baseline)
{
    bool candidate_complete = phase1_sequence_eval_complete(candidate);
    bool baseline_complete = phase1_sequence_eval_complete(baseline);
    if (candidate_complete != baseline_complete) return candidate_complete;

    if (candidate.ok != baseline.ok) return candidate.ok;

    if (candidate.unresolved_obligations != baseline.unresolved_obligations) {
        return candidate.unresolved_obligations < baseline.unresolved_obligations;
    }
    if (candidate.unreachable != baseline.unreachable) {
        return candidate.unreachable < baseline.unreachable;
    }
    if (candidate.deadlocks != baseline.deadlocks) {
        return candidate.deadlocks < baseline.deadlocks;
    }

    int candidate_pushes = phase1_sequence_clear_push_count(candidate.tasks);
    int baseline_pushes = phase1_sequence_clear_push_count(baseline.tasks);
    if (candidate_pushes != baseline_pushes) return candidate_pushes < baseline_pushes;

    if (candidate.cost != baseline.cost) return candidate.cost < baseline.cost;
    if (candidate.distance != baseline.distance) return candidate.distance < baseline.distance;
    if (candidate.tasks.size() != baseline.tasks.size()) return candidate.tasks.size() < baseline.tasks.size();
    return false;
}

static bool phase1_result_better_than(
    const DFSResult& candidate,
    const DFSResult& baseline,
    int profit_margin = 0)
{
    bool structural_phase =
        phase1_result_has_structural_defect(candidate) ||
        phase1_result_has_structural_defect(baseline);
    if (structural_phase) {
        // Phase1 的完成目标是所有箱子都能到所有目标点，不是只完成一次任意匹配
        if (candidate.unreachable_pairs_remaining != baseline.unreachable_pairs_remaining) {
            return candidate.unreachable_pairs_remaining < baseline.unreachable_pairs_remaining;
        }
        if (candidate.deadlocks_remaining != baseline.deadlocks_remaining) {
            return candidate.deadlocks_remaining < baseline.deadlocks_remaining;
        }
        if (candidate.bomb_supply_score != baseline.bomb_supply_score) {
            return candidate.bomb_supply_score > baseline.bomb_supply_score;
        }
        if (candidate.tasks.size() != baseline.tasks.size()) {
            return candidate.tasks.size() < baseline.tasks.size();
        }
    }

    if (!structural_phase) {
        if (candidate.net_profit > baseline.net_profit + profit_margin) return true;
        if (baseline.net_profit > candidate.net_profit + profit_margin) return false;
        if (candidate.tasks.size() != baseline.tasks.size()) {
            return candidate.tasks.size() < baseline.tasks.size();
        }
    }

    return candidate.net_profit > baseline.net_profit + profit_margin;
}


// ============================================================================
// 3. 对外入口：评估并生成炸弹任务序列
// ============================================================================

/// \brief Phase1 对外入口：评估并生成炸弹任务序列
/// \param level 当前地图快照
/// \return 建议执行的炸弹任务序列，可能为空
///
/// \details
/// 入口只保留 Phase1 任意匹配流程：hard pass 提供无需清障的基线，soft pass 只作为可实体化候选来源
StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::plan_phase1_bombs(const SokobanLevel& level) {
    this->phase1_phase2_inherited_candidates.clear();
    if (level.bomb_count == 0) return StaticArray<BombTask, MAX_BOMBS>(); 

    this->begin_profile_eval(0);
    this->cached_level = level;
    DFSResult best_res;
    uint8_t selected_profile_pass = 0;

    this->phase1_soft_bomb_eval = false;
    this->phase1_defer_soft_successor = false;

    // =========================================================================
    // 阶段 1：极速静态推演（假定无需推箱子即可破局）
    // =========================================================================
    this->execute_phase1_search_pass(level, 0, best_res);

    // =========================================================================
    // 阶段 2：软障碍评估与可执行任务实体化
    // =========================================================================
    DFSResult hard_res = best_res;
    bool selected_soft_res = false;

    DFSResult soft_res;
    soft_res.deadlocks_remaining = 9999;
    soft_res.net_profit = -999999;
    bool hard_direct_or_empty =
        hard_res.tasks.size() == 0 ||
        this->are_fast_bomb_tasks_directly_executable(level, hard_res.tasks);
    bool hard_has_deadlock = hard_res.deadlocks_remaining > 0;
    bool hard_has_unreachable = hard_res.unreachable_pairs_remaining > 0;
    bool run_soft_pass = hard_has_deadlock || hard_has_unreachable || !hard_direct_or_empty;
    if (run_soft_pass) {
        this->phase1_soft_bomb_eval = true;
        // 仅补全全箱-全目标可达性时，先搜索结构序列，避免 DFS 内反复实体化清障
        this->phase1_defer_soft_successor =
            hard_has_unreachable && !hard_has_deadlock && hard_direct_or_empty;
        this->execute_phase1_search_pass(level, 1, soft_res);
        this->phase1_soft_bomb_eval = false;
        this->phase1_defer_soft_successor = false;
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

    // soft pass 可以把墙视为低成本软障碍，返回前必须实体化成真实可执行推箱任务
    if (best_res.tasks.size() > 0 &&
        !this->are_fast_bomb_tasks_directly_executable(level, best_res.tasks)) {
        StaticArray<BombTask, MAX_BOMBS> repaired = best_res.tasks;

        Phase1SequenceEval repaired_eval;
        StaticArray<StrategyClearObligation, MAX_BOMBS * 8> repaired_obligations;
        repaired_eval.ok =
            this->materialize_phase1_sequence(
                level, repaired, &repaired_eval.cost, &repaired_obligations) &&
            this->evaluate_phase1_task_sequence(
                level,
                repaired,
                repaired_eval.deadlocks,
                repaired_eval.unreachable,
                repaired_eval.distance,
                &repaired_obligations,
                &repaired_eval.unresolved_obligations);
        repaired_eval.tasks = repaired;

        Phase1SequenceEval hard_eval;
        StaticArray<BombTask, MAX_BOMBS> hard_verified = hard_res.tasks;
        StaticArray<StrategyClearObligation, MAX_BOMBS * 8> hard_obligations;
        hard_eval.ok =
            this->materialize_phase1_sequence(
                level, hard_verified, &hard_eval.cost, &hard_obligations) &&
            this->evaluate_phase1_task_sequence(
                level,
                hard_verified,
                hard_eval.deadlocks,
                hard_eval.unreachable,
                hard_eval.distance,
                &hard_obligations,
                &hard_eval.unresolved_obligations);
        hard_eval.tasks = hard_verified;

        bool repaired_beats_hard =
            phase1_sequence_eval_complete(repaired_eval) &&
            phase1_sequence_eval_better_than(repaired_eval, hard_eval);

        // Phase1 只接受真实执行后全对偶可达的序列，避免把残缺候选交给上层兜底
        if (repaired_beats_hard) {
            best_res.tasks = repaired_eval.tasks;
            best_res.deadlocks_remaining = repaired_eval.deadlocks;
            best_res.unreachable_pairs_remaining = repaired_eval.unreachable;
            best_res.net_profit = -repaired_eval.distance - repaired_eval.cost;
        } else if (selected_soft_res) {
            best_res = hard_res;
            selected_profile_pass = 0;
        }
    }

    // 最终再验证一次，防止 hard/soft 共享入口绕过了局部清障义务检查
    if (best_res.tasks.size() > 0) {
        StaticArray<BombTask, MAX_BOMBS> verified = best_res.tasks;
        int verified_cost = 0;
        int verified_deadlocks = 9999;
        int verified_unreachable = 9999;
        int verified_distance = 999999;
        int verified_unresolved_obligations = 9999;
        StaticArray<StrategyClearObligation, MAX_BOMBS * 8> verified_obligations;
        bool verified_ok =
            this->materialize_phase1_sequence(
                level, verified, &verified_cost, &verified_obligations) &&
            this->evaluate_phase1_task_sequence(
                level,
                verified,
                verified_deadlocks,
                verified_unreachable,
                verified_distance,
                &verified_obligations,
                &verified_unresolved_obligations);

        if (verified_ok &&
            verified_unresolved_obligations == 0 &&
            verified_deadlocks == 0 &&
            verified_unreachable == 0) {
            best_res.tasks = verified;
            best_res.deadlocks_remaining = verified_deadlocks;
            best_res.unreachable_pairs_remaining = verified_unreachable;
            best_res.net_profit = -verified_distance - verified_cost;
        } else {
            // Phase1 对巡图只返回全对偶闭环任务；未闭环但真实可执行的结构候选
            // 只作为 Phase2 前缀缓存，避免巡图阶段提前执行半套任务
            if (verified_ok && verified_unresolved_obligations == 0 && verified.size() > 0) {
                this->phase1_phase2_inherited_candidates = verified;
            }
            best_res.tasks.clear();
            if (verified_ok) {
                best_res.deadlocks_remaining = verified_deadlocks;
                best_res.unreachable_pairs_remaining = verified_unreachable;
                best_res.net_profit = -verified_distance - verified_cost;
            }
        }
    }

    this->stamp_selected_tasks(best_res);
    this->record_profile_selected(selected_profile_pass, best_res);
    this->phase1_defer_soft_successor = false;
    return best_res.tasks;
}


// ============================================================================
// 4. DFS 策略搜索：枚举候选墙体并评估收益
// ============================================================================

/// \brief Phase1 DFS 枚举炸弹序列并更新全局最优结果
/// \param current_lvl 当前地图状态
/// \param player_start 当前玩家位置
/// \param current_seq 当前已经选择的炸弹任务序列
/// \param cost_so_far 当前序列累计代价
/// \param depth DFS 深度，同时用于复用距离场缓存层
/// \param best_res 全局最优结果，递归过程中被持续更新
///
/// \details
/// 该函数只处理 Phase1 任意匹配：先评估结构缺陷，再用逻辑层和真实 successor 过滤 soft pass 候选
void StrategicPlanner::dfs_phase1_bomb_sequence(
    const SokobanLevel& current_lvl, point player_start,
    StaticArray<BombTask, MAX_BOMBS> current_seq, int cost_so_far, 
    int depth, DFSResult& best_res) 
{
    this->record_profile_dfs_node();
    const int current_bomb_count = strategy_bomb_count(current_lvl);
    // 主流程顺序：结构评估 -> 候选生成 -> 真实代价评分 -> 候选排序 -> successor 验证
    // =====================================================================
    // 1. 评估当前状态并更新全局最优结果
    // =====================================================================
    int current_deadlocks = 0;  // 当前状态死锁数量
    int current_distance = 0;   // 当前状态所有箱子到目标的总距离（作为成本评估的一部分）

    PlanningCommon::calc_player_reach(current_lvl, player_start, {-1,-1}, {-1,-1}, strategy_ws.dfs_player_vis[depth]);

    // 状态评估：Phase1 同时维护硬距离和软障碍距离，任意匹配只看 soft pass 的结构通路
    for (int b = 0; b < current_lvl.box_count; ++b) {
        this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, strategy_ws.dfs_dist_box[depth][b], false);
        this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, strategy_ws.soft_box_dist_by_depth[depth][b], true, true);
    }
    evaluate_phase1_any_matching(current_lvl, strategy_ws.soft_box_dist_by_depth[depth], current_seq.size(), current_deadlocks, current_distance);
    int current_unreachable_pairs = count_phase1_unreachable_pairs(current_lvl, strategy_ws.soft_box_dist_by_depth[depth]);

    bool terminal_node = current_seq.size() == current_bomb_count || depth >= MAX_BOMBS;

    // 净收益评估：Phase1 无语义绑定，闭环后先比较任意匹配距离，再用真实执行代价做次级代价
    // 这样短推炸弹不会压过真正打开箱-目标通路的墙
    int profit = -current_distance - cost_so_far;

    // =====================================================================
    // 2. 计算存活炸弹可达爆破点
    // =====================================================================
    for (int m = 0; m < current_bomb_count; ++m) {
        if (current_lvl.bombs[m].x != -1) {
            // Phase1 soft 先保留结构候选，真实可执行性由 successor/materialize 验证
            this->fast_push_bfs(
                current_lvl,
                current_lvl.bombs[m],
                player_start,
                true,
                strategy_ws.dfs_dist_bomb[depth][m],
                this->phase1_soft_bomb_eval,
                false
            );
            if (this->phase1_soft_bomb_eval) {
                this->fast_push_bfs(
                    current_lvl,
                    current_lvl.bombs[m],
                    player_start,
                    true,
                    strategy_ws.strict_bomb_dist_by_depth[depth][m],
                    true,
                    true
                );
                this->fast_push_bfs(
                    current_lvl,
                    current_lvl.bombs[m],
                    player_start,
                    true,
                    strategy_ws.hard_bomb_dist_by_depth[depth][m],
                    false
                );
            }
        }
    }

    // 前三层启用缺陷驱动逻辑层。死锁未清时，后续炸弹是否有价值
    // 主要取决于它是否继续打开关键结构，而不是普通距离缩短。
    const bool use_logic_scores = (depth <= 2);
    if (use_logic_scores) {
        this->record_profile_logic_build();
        build_phase1_logic_blast_scores(
            current_lvl,
            strategy_ws.dfs_player_vis[depth],
            strategy_ws.soft_box_dist_by_depth[depth],
            strategy_ws.logic
        );
    }
    int phase1_current_supply_score = 0;
    bool phase1_structural_defect_active = current_deadlocks > 0 || current_unreachable_pairs > 0;
    if (phase1_structural_defect_active && use_logic_scores) {
        if (this->phase1_soft_bomb_eval) {
            // 供给分衡量剩余炸弹的真实开路能力，不能沿用软障碍幻想距离
            for (int b = 0; b < strategy_bomb_count(current_lvl); ++b) {
                if (current_lvl.bombs[b].x == -1) continue;
                this->fast_push_bfs(
                    current_lvl,
                    current_lvl.bombs[b],
                    player_start,
                    true,
                    strategy_ws.probe_bomb_dist[b],
                    false
                );
            }
            phase1_current_supply_score = phase1_key_bomb_supply_score(
                current_lvl,
                strategy_ws.probe_bomb_dist,
                strategy_ws.logic
            );
        } else {
            phase1_current_supply_score = phase1_key_bomb_supply_score(
                current_lvl,
                strategy_ws.dfs_dist_bomb[depth],
                strategy_ws.logic
            );
        }
    }

    bool better = false;
    bool best_structural_defect =
        best_res.deadlocks_remaining > 0 || best_res.unreachable_pairs_remaining > 0;
    if (current_unreachable_pairs < best_res.unreachable_pairs_remaining) {
        better = true;
    } else if (current_unreachable_pairs == best_res.unreachable_pairs_remaining) {
        if (phase1_structural_defect_active || best_structural_defect) {
            if (current_deadlocks < best_res.deadlocks_remaining) {
                better = true;
            } else if (current_deadlocks == best_res.deadlocks_remaining &&
                       phase1_current_supply_score > best_res.bomb_supply_score) {
                better = true;
            } else if (current_deadlocks == best_res.deadlocks_remaining &&
                       phase1_current_supply_score == best_res.bomb_supply_score &&
                       current_seq.size() < best_res.tasks.size()) {
                better = true;
            }
        } else {
            if (this->phase1_soft_bomb_eval) {
                // soft pass 闭环后不再靠大容差保护首个解，真实执行成本应能替换绕路候选
                const int solved_profit_margin = 10;
                if (profit > best_res.net_profit + solved_profit_margin) {
                    better = true;
                } else if (profit + solved_profit_margin >= best_res.net_profit &&
                           current_seq.size() < best_res.tasks.size()) {
                    better = true;
                }
            } else if (current_seq.size() < best_res.tasks.size()) {
                // hard pass 的 bomb 图不需要清障，优先保留更短炸弹序列，避免为局部分数过度用弹
                better = true;
            } else if (current_seq.size() == best_res.tasks.size() &&
                       profit > best_res.net_profit) {
                better = true;
            }
        }
    }

    if (better) {
        best_res.deadlocks_remaining = current_deadlocks;
        best_res.net_profit = profit;
        best_res.unreachable_pairs_remaining = current_unreachable_pairs;
        best_res.bomb_supply_score = phase1_current_supply_score;
        best_res.tasks = current_seq;
    }

    // Phase1 只负责打开结构闭环，闭环后保留剩余炸弹给语义阶段
    if (!phase1_structural_defect_active) return;
    if (terminal_node) return;
    // 建立候选动作队列，避免无脑展开过多分支
    StaticArray<BombCandidate, 256>& candidates = strategy_ws.dfs.candidates[depth];
    StaticArray<BombCandidate, 256>& preliminary = strategy_ws.dfs.preliminary[depth];
    candidates.clear();
    preliminary.clear();
    std::memset(strategy_ws.dfs.probe_valid[depth], 0, sizeof(strategy_ws.dfs.probe_valid[depth]));

    int selection_limit = PHASE1_SELECTION_RESTRICTIONS;
    int heavy_eval_limit = 255;
    if (depth == 1) heavy_eval_limit = 12;
    else if (depth > 1) heavy_eval_limit = selection_limit * 2;
    if (this->phase1_soft_bomb_eval &&
        (depth + 1 >= current_bomb_count || depth + 1 >= MAX_BOMBS)) {
        heavy_eval_limit = PHASE1_EXECUTABLE_FINAL_SCAN_LIMIT;
    }

    // =====================================================================
    // 3. 按缺陷类型生成候选，并用一次真实爆破评估过滤
    // =====================================================================
    bool structural_defect_active = phase1_structural_defect_active;
    int total_pairs = current_lvl.box_count * current_lvl.target_count;
    bool phase1_full_disconnect = total_pairs > 0 && current_unreachable_pairs >= total_pairs;

    // 结构评估只看任意箱-目标通路，不读取语义绑定
    auto eval_probe_state = [&](const SokobanLevel& lvl,
                                point eval_player,
                                int selected_count,
                                int& out_deadlocks,
                                int& out_unreachable,
                                int& out_distance) {
        out_deadlocks = 0;
        out_unreachable = 9999;
        out_distance = 0;

        for (int b = 0; b < lvl.box_count; ++b) {
            this->fast_push_bfs(lvl, lvl.boxes[b], eval_player, false,
                                strategy_ws.probe_box_dist[b], true);
        }
        evaluate_phase1_any_matching(
            lvl,
            strategy_ws.probe_box_dist,
            selected_count,
            out_deadlocks,
            out_distance
        );
        out_unreachable = count_phase1_unreachable_pairs(lvl, strategy_ws.probe_box_dist);
    };

    // 供给评估只提示未来炸弹能否触达关键墙位，不替代直接结构收益
    auto eval_probe_supply = [&](const SokobanLevel& lvl, point eval_player) -> int {
        if (!phase1_structural_defect_active || !use_logic_scores) return 0;
        for (int b = 0; b < strategy_bomb_count(lvl); ++b) {
            if (lvl.bombs[b].x == -1) continue;
            this->fast_push_bfs(
                lvl,
                lvl.bombs[b],
                eval_player,
                true,
                strategy_ws.probe_bomb_dist[b],
                false
            );
        }
        return phase1_key_bomb_supply_score(
            lvl,
            strategy_ws.probe_bomb_dist,
            strategy_ws.logic
        );
    };

    // 候选池只保留高分墙位，同墙去重时仍保留候选自己的评分
    auto keep_candidate = [&](BombCandidate candidate) {
        this->record_profile_candidate_kept();
        phase1_keep_ranked_candidate(candidates, candidate);
    };

    // 预筛候选用便宜结构信号排序，减少完整 probe 次数
    auto keep_preliminary_candidate = [&](BombCandidate candidate) {
        phase1_keep_ranked_candidate(preliminary, candidate);
    };

    // 完整评分先计算爆破后的结构变化，再用真实路线代价做阻尼
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

        Phase1CandidateLocalFeatures local_features = phase1_candidate_local_features(
            current_lvl,
            strategy_ws.dfs_player_vis[depth],
            m,
            candidate_wall);

        int after_deadlocks = 0;
        int after_unreachable = 9999;
        int after_distance = 0;
        SokobanLevel probe_lvl = current_lvl;
        point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
        phase1_apply_probe_bomb_transition(probe_lvl, m, wall);
        if (strategy_ws.dfs.probe_valid[depth][y][x] &&
            strategy_ws.dfs.probe_bomb_idx[depth][y][x] == static_cast<uint8_t>(m)) {
            after_deadlocks = strategy_ws.dfs.probe_deadlocks[depth][y][x];
            after_unreachable = strategy_ws.dfs.probe_unreachable[depth][y][x];
            after_distance = strategy_ws.dfs.probe_distance[depth][y][x];
        } else {
            // 候选重评估必须和递归 successor 一样移除已使用炸弹
            eval_probe_state(
                probe_lvl,
                wall,
                current_seq.size() + 1,
                after_deadlocks,
                after_unreachable,
                after_distance
            );
            strategy_ws.dfs.probe_valid[depth][y][x] = true;
            strategy_ws.dfs.probe_bomb_idx[depth][y][x] = static_cast<uint8_t>(m);
            strategy_ws.dfs.probe_deadlocks[depth][y][x] = after_deadlocks;
            strategy_ws.dfs.probe_unreachable[depth][y][x] = after_unreachable;
            strategy_ws.dfs.probe_distance[depth][y][x] = after_distance;
        }

        int deadlock_gain = current_deadlocks - after_deadlocks;
        int unreachable_gain = current_unreachable_pairs - after_unreachable;
        int distance_gain = current_distance - after_distance;

        int supply_gain = 0;
        if (current_seq.size() + 1 < current_bomb_count) {
            SokobanLevel probe_lvl = current_lvl;
            point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
            phase1_apply_probe_bomb_transition(probe_lvl, m, wall);
            supply_gain = eval_probe_supply(probe_lvl, wall) - phase1_current_supply_score;
        }
        int gateway_gain = 0;
        // gateway 只服务真实死锁闭环，避免普通可达图被远端开区收益带偏
        if (current_deadlocks > 0 && current_seq.size() + 1 < current_bomb_count) {
            SokobanLevel probe_lvl = current_lvl;
            point wall = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
            phase1_apply_probe_bomb_transition(probe_lvl, m, wall);
            gateway_gain = phase1_gateway_resource_score(
                current_lvl,
                probe_lvl,
                strategy_ws.dfs_player_vis[depth],
                wall,
                strategy_ws.logic
            );
        }

        bool direct_fix = deadlock_gain > 0 || unreachable_gain > 0;
        bool phase1_soft_only_candidate =
            this->phase1_soft_bomb_eval &&
            phase1_structural_defect_active &&
            strategy_ws.strict_bomb_dist_by_depth[depth][m][y][x] == INF_DIST;
        if (phase1_soft_only_candidate) {
            direct_fix = false;
        }
        bool supply_fix = supply_gain > 120;
        bool non_regressing_key_defect = key_defect_wall && deadlock_gain >= 0 && unreachable_gain >= 0;
        bool gateway_fix =
            phase1_structural_defect_active &&
            gateway_gain > 0 &&
            deadlock_gain >= 0 &&
            unreachable_gain >= 0;

        bool keep = direct_fix || supply_fix || non_regressing_key_defect || gateway_fix;
        if (!structural_defect_active) {
            keep = keep || distance_gain > 0 ||
                   local_features.opens_unreachable_floor ||
                   local_features.entity_touch_score > 0;
        }
        if (!keep) return;

        int score = 0;
        score += unreachable_gain * (phase1_soft_only_candidate ? 220000 : 950000);
        score += deadlock_gain * (phase1_soft_only_candidate ? 90000 : 220000);
        score += l1_hits * 50000;
        score += l3_hits * 36000;
        score += l2_hits * 24000;
        score += supply_hits * 18000;
        score += logic_score;

        if (local_features.opens_unreachable_floor) score += 9000 + local_features.weak_open_score;
        if (gateway_gain > 0) score += 26000 + std::min(gateway_gain, 80000);
        bool candidate_solves_phase1 = after_deadlocks == 0 && after_unreachable == 0;
        if (!candidate_solves_phase1 && supply_gain > 0) {
            if (direct_fix) score += std::min(supply_gain, 6000);
            else score += 22000 + std::min(supply_gain, 8000) * 5;
        } else if (!candidate_solves_phase1 && supply_gain < 0 && structural_defect_active) {
            // `supplyGain` 只是未来资源提示，不能让直接结构修复主要输给剩余供给下降
            // 供给下降是机会成本，不能线性压过当前墙体的结构证据
            int supply_penalty = direct_fix ? supply_gain / 4 : supply_gain * 2;
            int penalty_floor = direct_fix ? -15000 : -100000;
            if (supply_penalty < penalty_floor) supply_penalty = penalty_floor;
            score += supply_penalty;
        }

        if (distance_gain > 0) score += distance_gain * (structural_defect_active ? 14 : 32);
        else score += distance_gain * (structural_defect_active ? 6 : 18);

        score += local_features.wall_mass * 260 + local_features.entity_touch_score;
        int route_dist_for_score = strategy_ws.dfs_dist_bomb[depth][m][y][x];
        if (this->phase1_soft_bomb_eval &&
            strategy_ws.strict_bomb_dist_by_depth[depth][m][y][x] != INF_DIST) {
            route_dist_for_score = strategy_ws.strict_bomb_dist_by_depth[depth][m][y][x];
        }
        int direct_route_cost = strategy_direct_bomb_cost_for_score(
            current_lvl,
            player_start,
            current_lvl.bombs[m],
            candidate_wall,
            route_dist_for_score);
        // 真实车位和炸弹路径只做阻尼，墙体仍主要由爆破后打开的推图结构决定
        int route_cost_for_score = phase1_ranked_bomb_effort(route_dist_for_score, direct_route_cost);
        score -= route_cost_for_score * (structural_defect_active ? 12 : 18);
        if (phase1_soft_only_candidate) {
            score -= 140000;
        }

        if (structural_defect_active && !direct_fix && !supply_fix && !gateway_fix && logic_score <= 0) {
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

                Phase1CandidateLocalFeatures local_features = phase1_candidate_local_features(
                    current_lvl,
                    strategy_ws.dfs_player_vis[depth],
                    m,
                    wall);

                int gateway_hint = 0;
                // 预筛同样限制在真实死锁态，保留无死锁图的短路径优先级
                if (current_deadlocks > 0) {
                    gateway_hint = phase1_gateway_hint_score(
                        current_lvl,
                        strategy_ws.dfs_player_vis[depth],
                        wall
                    );
                }

                if (structural_defect_active && !key_defect_wall &&
                    !local_features.opens_unreachable_floor && gateway_hint <= 0) {
                    continue;
                }
                if (!structural_defect_active && !key_defect_wall &&
                    !local_features.opens_unreachable_floor &&
                    local_features.entity_touch_score == 0 &&
                    local_features.wall_mass < 3) {
                    continue;
                }

                int cheap_score = 0;
                cheap_score += l1_hits * 50000;
                cheap_score += l3_hits * 36000;
                cheap_score += l2_hits * 24000;
                cheap_score += supply_hits * 18000;
                cheap_score += logic_score;
                if (local_features.opens_unreachable_floor) {
                    cheap_score += 9000 + local_features.weak_open_score;
                }
                if (gateway_hint > 0) cheap_score += gateway_hint;
                cheap_score += local_features.wall_mass * 260 + local_features.entity_touch_score;
                cheap_score -= strategy_ws.dfs_dist_bomb[depth][m][y][x] * (structural_defect_active ? 12 : 18);
                if (structural_defect_active && !key_defect_wall && gateway_hint <= 0) cheap_score -= 60000;

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

            // 同墙位换更近炸弹时必须带着新候选自己的评分，避免炸弹和旧分数混合
            const int same_wall_assignment_margin = 500000;
            if (candidate_dist + 1 < existing_dist &&
                candidates[i].score + same_wall_assignment_margin >= existing.score) {
                existing = candidates[i];
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
    std::sort(candidates.begin(), candidates.end());
    int branch_limit = candidates.size() < selection_limit ? candidates.size() : selection_limit;

    if (structural_defect_active && depth == 0 && branch_limit > 1) {
        int scan_limit = candidates.size();

        for (int slot = 1; slot < branch_limit; ++slot) {
            int best_idx = -1;
            for (int i = slot; i < scan_limit; ++i) {
                if (phase1_candidate_bomb_used_before(candidates, slot, candidates[i].bomb_idx)) continue;
                if (phase1_candidate_overlaps_before(candidates, slot, candidates[i])) continue;
                best_idx = i;
                break;
            }
            if (best_idx < 0) {
                for (int i = slot; i < scan_limit; ++i) {
                    if (phase1_candidate_bomb_used_before(candidates, slot, candidates[i].bomb_idx)) continue;
                    best_idx = i;
                    break;
                }
            }
            if (best_idx < 0) {
                for (int i = slot; i < scan_limit; ++i) {
                    if (!phase1_candidate_overlaps_before(candidates, slot, candidates[i])) {
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

    if (this->phase1_soft_bomb_eval &&
        depth == 0 &&
        branch_limit >= 3 &&
        phase1_full_disconnect) {
        int scan_limit = candidates.size() < PHASE1_CLEAR_DIVERSITY_SCAN_LIMIT ?
            candidates.size() : PHASE1_CLEAR_DIVERSITY_SCAN_LIMIT;
        int best_clear_idx = -1;
        int best_pushes = 99;
        int best_score = -999999;

        for (int i = branch_limit; i < scan_limit; ++i) {
            BombCandidate c = candidates[i];
            int m = c.bomb_idx;
            if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) continue;

            SokobanLevel probe_lvl;
            int real_cost = 0;
            StaticArray<BoxPushTask, 8> pushes;
            point target_wall = {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)};
            if (!this->local_clear_bomb_route(
                    current_lvl,
                    m,
                    target_wall,
                    false,
                    probe_lvl,
                    real_cost,
                    pushes)) {
                continue;
            }
            if (pushes.size() == 0 || pushes.size() > 2) continue;

            if (pushes.size() < best_pushes ||
                (pushes.size() == best_pushes && c.score > best_score)) {
                best_clear_idx = i;
                best_pushes = pushes.size();
                best_score = c.score;
            }
        }

        if (best_clear_idx >= 0) {
            candidates[branch_limit - 1] = candidates[best_clear_idx];
        }
    }
    if (this->phase1_soft_bomb_eval &&
        depth == 0 &&
        branch_limit > 0 &&
        phase1_structural_defect_active) {
        int scan_limit = candidates.size() < PHASE1_EXECUTABLE_ROOT_SCAN_LIMIT ?
            candidates.size() : PHASE1_EXECUTABLE_ROOT_SCAN_LIMIT;
        int materialize_probe_budget = 3;

        auto estimate_root_candidate_effort = [&](const BombCandidate& c,
                                                  int& out_cost,
                                                  int& out_pushes) -> bool {
            int m = c.bomb_idx;
            if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) return false;

            BombTask probe;
            probe.bomb_start = current_lvl.bombs[m];
            probe.target_wall = {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)};
            probe.is_essential = false;
            probe.net_profit = 0;
            probe.box_pushes.clear();

            StaticArray<point, MAX_PATH_LENGTH> path;
            if (PlanningCommon::get_bomb_push_path(current_lvl, player_start, probe, path)) {
                out_cost = path.size();
                out_pushes = 0;
                return true;
            }

            if (materialize_probe_budget <= 0) return false;
            --materialize_probe_budget;

            SokobanLevel probe_lvl;
            StaticArray<BoxPushTask, 8> pushes;
            int real_cost = 0;
            if (!this->local_clear_bomb_route(
                    current_lvl,
                    m,
                    probe.target_wall,
                    false,
                    probe_lvl,
                    real_cost,
                    pushes,
                    true)) {
                return false;
            }
            out_cost = real_cost;
            out_pushes = pushes.size();
            return true;
        };

        for (int slot = 0; slot < 1 && slot < branch_limit && materialize_probe_budget > 0; ++slot) {
            int selected_bomb = candidates[slot].bomb_idx;
            bool duplicated_selected_bomb = false;
            for (int other = 0; other < branch_limit; ++other) {
                if (other == slot) continue;
                if (candidates[other].bomb_idx == selected_bomb) {
                    duplicated_selected_bomb = true;
                    break;
                }
            }
            if (duplicated_selected_bomb) continue;

            bool has_close_same_bomb_alternative = false;
            for (int i = branch_limit; i < scan_limit; ++i) {
                if (candidates[i].bomb_idx != selected_bomb) continue;
                if (candidates[i].score + 240000 >= candidates[slot].score) {
                    has_close_same_bomb_alternative = true;
                    break;
                }
            }
            if (!has_close_same_bomb_alternative) continue;

            int selected_cost = 0;
            int selected_pushes = 0;
            int best_idx = slot;
            int best_utility = candidates[slot].score - 999999;
            if (estimate_root_candidate_effort(candidates[slot], selected_cost, selected_pushes)) {
                best_utility =
                    candidates[slot].score -
                    selected_cost * 18000 -
                    selected_pushes * 140000;
            }

            for (int i = branch_limit; i < scan_limit && materialize_probe_budget > 0; ++i) {
                if (candidates[i].bomb_idx != selected_bomb) continue;
                if (candidates[i].score + 240000 < candidates[slot].score) continue;

                int candidate_cost = 0;
                int candidate_pushes = 0;
                if (!estimate_root_candidate_effort(candidates[i], candidate_cost, candidate_pushes)) continue;

                int utility =
                    candidates[i].score -
                    candidate_cost * 18000 -
                    candidate_pushes * 140000;
                if (utility > best_utility + 60000) {
                    best_utility = utility;
                    best_idx = i;
                }
            }

            if (best_idx != slot) {
                candidates[slot] = candidates[best_idx];
            }
        }
    }
    if (depth == 0) {
        this->record_profile_root_candidates(current_lvl, candidates, branch_limit);
    }

    // -----------------------------------------------------
    // 静态/软评估执行路径
    // -----------------------------------------------------
    bool use_real_phase1_successor =
        this->phase1_soft_bomb_eval &&
        !this->phase1_defer_soft_successor &&
        (phase1_full_disconnect || phase1_structural_defect_active);
    if (use_real_phase1_successor && depth < MAX_BOMBS) {
        int phase1_scan_budget =
            depth == 0 ? PHASE1_EXECUTABLE_ROOT_SCAN_LIMIT : PHASE1_EXECUTABLE_BRANCH_SCAN_LIMIT;
        int phase1_branch_budget = branch_limit;
        bool final_executable_layer = depth + 1 >= current_bomb_count || depth + 1 >= MAX_BOMBS;
        if (final_executable_layer) {
            phase1_scan_budget = PHASE1_EXECUTABLE_FINAL_SCAN_LIMIT;
            phase1_branch_budget = PHASE1_EXECUTABLE_FINAL_BRANCH_LIMIT;
        }
        int scan_limit = candidates.size() < phase1_scan_budget ?
            candidates.size() : phase1_scan_budget;
        int valid_branches = 0;
        auto try_phase1_successor = [&](const BombCandidate& c) -> bool {
            int m = c.bomb_idx;
            if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) return false;

            BombTask soft_task;
            soft_task.bomb_start = current_lvl.bombs[m];
            soft_task.target_wall = {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)};
            soft_task.is_essential = false;
            soft_task.net_profit = 0;
            soft_task.box_pushes.clear();

            point target_wall = {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)};

            SokobanLevel next_lvl = current_lvl;
            point next_player = player_start;
            int real_execution_cost = 0;
            if (this->apply_executable_bomb_task(
                    next_lvl,
                    next_player,
                    soft_task,
                    &real_execution_cost)) {
                next_lvl.player_start = next_player;

                StaticArray<BombTask, MAX_BOMBS> next_seq = current_seq;
                next_seq.push_back(soft_task);

                ++valid_branches;
                this->record_profile_child_branch();
                this->dfs_phase1_bomb_sequence(
                    next_lvl,
                    next_lvl.player_start,
                    next_seq,
                    cost_so_far + real_execution_cost,
                    depth + 1,
                    best_res
                );
                if (!final_executable_layer &&
                    best_res.deadlocks_remaining == 0 &&
                    best_res.unreachable_pairs_remaining == 0) {
                    return true;
                }
                return false;
            }

            // Phase1 soft 的分支名额按真实 successor 计数
            StaticArray<BoxPushTask, 8> extracted_pushes;
            bool physically_possible = this->local_clear_bomb_route(
                current_lvl,
                m,
                target_wall,
                false,
                next_lvl,
                real_execution_cost,
                extracted_pushes,
                false
            );
            if (!physically_possible) {
                physically_possible = this->local_clear_bomb_route(
                    current_lvl,
                    m,
                    target_wall,
                    false,
                    next_lvl,
                    real_execution_cost,
                    extracted_pushes,
                    true
                );
            }
            if (!physically_possible) return false;

            next_lvl.bombs[m] = {-1, -1};
            PlanningCommon::apply_blast_effect(next_lvl, target_wall);

            StaticArray<BombTask, MAX_BOMBS> next_seq = current_seq;
            next_seq.push_back({
                current_lvl.bombs[m],
                {static_cast<int8_t>(c.x), static_cast<int8_t>(c.y)},
                false,
                0,
                extracted_pushes
            });

            ++valid_branches;
            this->record_profile_child_branch();
            this->dfs_phase1_bomb_sequence(
                next_lvl,
                next_lvl.player_start,
                next_seq,
                cost_so_far + real_execution_cost,
                depth + 1,
                best_res
            );
            if (!final_executable_layer &&
                best_res.deadlocks_remaining == 0 &&
                best_res.unreachable_pairs_remaining == 0) {
                return true;
            }
            return false;
        };

        for (int i = 0; i < scan_limit && valid_branches < phase1_branch_budget; ++i) {
            BombCandidate c = candidates[i];
            int m = c.bomb_idx;
            if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) continue;
            if (strategy_ws.strict_bomb_dist_by_depth[depth][m][c.y][c.x] == INF_DIST) continue;
            if (try_phase1_successor(c)) return;
        }
        bool phase1_still_defective =
            best_res.deadlocks_remaining > 0 ||
            best_res.unreachable_pairs_remaining > 0;
        if (valid_branches < phase1_branch_budget &&
            (final_executable_layer || phase1_still_defective)) {
            int fallback_scan = final_executable_layer ?
                scan_limit :
                (scan_limit < branch_limit ? scan_limit : branch_limit);
            for (int i = 0; i < fallback_scan && valid_branches < phase1_branch_budget; ++i) {
                BombCandidate c = candidates[i];
                int m = c.bomb_idx;
                if (m < 0 || m >= current_bomb_count || current_lvl.bombs[m].x == -1) continue;
                if (strategy_ws.strict_bomb_dist_by_depth[depth][m][c.y][c.x] != INF_DIST) continue;
                if (try_phase1_successor(c)) return;
            }
        }
        return;
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

        // Hard pass 的最终择优必须使用真实直推代价。fast_push_bfs 的距离只描述
        // 炸弹位移，低估了小车绕行和换边站位成本，会把“绕一圈”的墙位误判为便宜。
        int direct_execution_cost = strategy_direct_bomb_cost_for_score(
            current_lvl,
            player_start,
            current_lvl.bombs[m],
            next_task.target_wall,
            strategy_ws.dfs_dist_bomb[depth][m][c.y][c.x]);
        int execution_cost = phase1_ranked_bomb_effort(
            strategy_ws.dfs_dist_bomb[depth][m][c.y][c.x],
            direct_execution_cost);
        if (this->phase1_soft_bomb_eval && current_deadlocks > 0 && execution_cost > 10) {
            execution_cost = 10 + (execution_cost - 10) / 4;
        }

        // hard pass 不能在首个闭环后提前返回，同层候选还需要按 net_profit 选更优墙位
        this->record_profile_child_branch();
        this->dfs_phase1_bomb_sequence(next_lvl, { (int8_t)c.x, (int8_t)c.y }, next_seq, cost_so_far + execution_cost, depth + 1, best_res);
    }
}



// ============================================================================
// 5. Phase1 序列修复与实体化验证
// ============================================================================

bool StrategicPlanner::materialize_phase1_sequence(
    const SokobanLevel& level,
    StaticArray<BombTask, MAX_BOMBS>& seq,
    int* out_sequence_cost,
    StaticArray<StrategyClearObligation, MAX_BOMBS * 8>* out_obligations)
{
    SokobanLevel work = level;
    point player = level.player_start;
    int sequence_cost = 0;
    StaticArray<StrategyClearObligation, MAX_BOMBS * 8> obligations;
    if (out_obligations) out_obligations->clear();

    auto apply_executable = [&](const BombTask& task) -> bool {
        return this->apply_executable_bomb_task(work, player, task, &sequence_cost);
    };

    auto apply_or_materialize = [&](const BombTask& candidate,
                                    BombTask& applied_task,
                                    int index) -> bool {
        if (apply_executable(candidate)) {
            applied_task = candidate;
            return true;
        }

        BombTask materialized_task;
        StaticArray<StrategyClearObligation, 8> clear_obligations;
        if (this->materialize_bomb_task(
                work,
                player,
                candidate,
                materialized_task,
                false,
                &clear_obligations,
                static_cast<uint8_t>(index)) &&
            apply_executable(materialized_task)) {
            applied_task = materialized_task;
            for (int i = 0; i < clear_obligations.size(); ++i) {
                StrategicPlanner::merge_clear_obligation(obligations, clear_obligations[i]);
            }
            return true;
        }

        int bomb_idx = -1;
        for (int b = 0; b < work.bomb_count; ++b) {
            if (work.bombs[b].x != -1 && work.bombs[b] == candidate.bomb_start) {
                bomb_idx = b;
                break;
            }
        }
        if (bomb_idx >= 0) {
            SokobanLevel clear_probe;
            int clear_cost = 0;
            StaticArray<BoxPushTask, 8> pushes;
            StaticArray<StrategyClearObligation, 8> clear_obligations;
            // Phase1 的清障要同时覆盖推炸弹路线和玩家发力通路
            SokobanLevel clear_start = work;
            clear_start.player_start = player;
            if (this->local_clear_bomb_route(
                    clear_start, bomb_idx, candidate.target_wall, false,
                    clear_probe, clear_cost, pushes, true,
                    &clear_obligations, static_cast<uint8_t>(index))) {
                materialized_task = candidate;
                materialized_task.box_pushes = pushes;
                if (apply_executable(materialized_task)) {
                    applied_task = materialized_task;
                    for (int i = 0; i < clear_obligations.size(); ++i) {
                        StrategicPlanner::merge_clear_obligation(obligations, clear_obligations[i]);
                    }
                    return true;
                }
            }
        }
        return false;
    };

    StaticArray<BombTask, MAX_BOMBS> pending = seq;
    seq.clear();

    for (int out_index = 0; out_index < pending.size(); ++out_index) {
        bool applied = false;
        BombTask selected_task;
        int selected_index = -1;
        StaticArray<StrategyClearObligation, MAX_BOMBS * 8> obligations_before = obligations;
        SokobanLevel work_before = work;
        point player_before = player;
        int cost_before = sequence_cost;

        // Phase1 soft 选的是结构墙集合，真实执行时允许重排可执行顺序
        for (int i = out_index; i < pending.size(); ++i) {
            work = work_before;
            player = player_before;
            sequence_cost = cost_before;
            obligations = obligations_before;

            BombTask applied_task;
            if (!apply_or_materialize(pending[i], applied_task, out_index)) continue;

            selected_task = applied_task;
            selected_index = i;
            applied = true;
            break;
        }

        if (!applied) {
            work = work_before;
            player = player_before;
            sequence_cost = cost_before;
            obligations = obligations_before;
            return false;
        }

        if (selected_index != out_index) {
            BombTask tmp = pending[out_index];
            pending[out_index] = pending[selected_index];
            pending[selected_index] = tmp;
        }
        pending[out_index] = selected_task;
        seq.push_back(selected_task);
    }
    if (out_sequence_cost) *out_sequence_cost = sequence_cost;
    if (out_obligations) *out_obligations = obligations;
    return true;
}
// 真实回放完整炸弹序列，用实体化后的地图重新计算 Phase1 任意匹配质量
bool StrategicPlanner::evaluate_phase1_task_sequence(
    const SokobanLevel& level,
    const StaticArray<BombTask, MAX_BOMBS>& seq,
    int& out_deadlocks,
    int& out_unreachable,
    int& out_distance,
    const StaticArray<StrategyClearObligation, MAX_BOMBS * 8>* obligations,
    int* out_unresolved_obligations)
{
    SokobanLevel work = level;
    point player = level.player_start;
    for (int task_idx = 0; task_idx < seq.size(); ++task_idx) {
        if (!this->apply_executable_bomb_task(work, player, seq[task_idx])) return false;
    }

    this->evaluate_phase1_matching_pairs(
        work, player, seq.size(), true, true, strategy_ws.dfs_dist_box[0], out_deadlocks, out_distance);
    out_unreachable = count_phase1_unreachable_pairs(work, strategy_ws.dfs_dist_box[0]);
    if (out_unresolved_obligations) {
        *out_unresolved_obligations = 0;
        if (obligations) {
            for (int i = 0; i < obligations->size(); ++i) {
                const StrategyClearObligation& obligation = (*obligations)[i];
                if (obligation.obligation == StrategyRescueObligationKind::NONE) continue;
                if (obligation.box_id >= work.box_count) {
                    ++(*out_unresolved_obligations);
                    continue;
                }

                point box_pos = work.boxes[obligation.box_id];
                bool on_valid_target = strategy_is_goal_for_box(work, obligation.box_id, box_pos, false);
                bool can_reach_goal = on_valid_target;
                if (!can_reach_goal) {
                    this->fast_push_bfs(work, box_pos, player, false, strategy_ws.probe_box_dist[0], true, true);
                    for (int t = 0; t < work.target_count; ++t) {
                        point goal = work.targets[t];
                        if (strategy_ws.probe_box_dist[0][goal.y][goal.x] != INF_DIST) {
                            can_reach_goal = true;
                            break;
                        }
                    }
                }
                if (!can_reach_goal) ++(*out_unresolved_obligations);
            }
        }
    }
    return true;
}

// 统一封装 Phase1 任意匹配距离评估，避免入口验证和局部修复各自拼装 BFS 参数
void StrategicPlanner::evaluate_phase1_matching_pairs(
    const SokobanLevel& level,
    point player,
    int selected_count,
    bool soft_boxes,
    bool strict_soft_boxes,
    int16_t out_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
    int& out_deadlocks,
    int& out_distance)
{
    for (int b = 0; b < level.box_count; ++b) {
        this->fast_push_bfs(level, level.boxes[b], player, false, out_dist[b], soft_boxes, strict_soft_boxes);
    }
    evaluate_phase1_any_matching(level, out_dist, selected_count, out_deadlocks, out_distance);
}


// ============================================================================
// 6. 局部清障规划：把 soft pass 候选补成真实可执行的推箱动作
// ============================================================================
// 该函数仍保留 phase2_specific 参数，因为 Phase2 实体化也复用这套局部清障器

struct RealClearCandidate {
    BoxPushTask task;
    int score;
    int order;
    uint8_t box_id;
    StrategyClearParking parking;
    bool opens_bomb_path;
    uint8_t source_support_dist;
    uint8_t target_support_dist;
    uint8_t move_dist;
    uint16_t first_push_cost;
};

struct RealClearMemoEntry {
    uint32_t key;
    uint8_t depth_left;
};

struct RealClearSearchConfig {
    uint8_t max_depth;
    uint8_t max_source_support_dist;
    uint8_t verify_scan_limit;
    uint8_t branch_limit;
    bool allow_theoretical_rescue;
};

struct LocalClearPushReachMap {
    uint16_t cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t pushes[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t turns[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t first_access[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
};

struct LocalClearPushReachWorkspace {
    uint16_t state_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    uint8_t state_pushes[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    uint8_t state_turns[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    uint8_t state_first_access[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    bool state_closed[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    uint16_t micro_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    point micro_q[MAP_CELL_COUNT];
};

static int local_clear_cheb(point a, point b) {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    return dx > dy ? dx : dy;
}

static bool local_clear_is_static_corner(const SokobanLevel& lvl, point p) {
    auto solid = [&](point q) {
        if (!PlanningCommon::in_bounds(q)) return true;
        return lvl.map[q.y][q.x] == 1;
    };
    bool up = solid(p + MOVE[0]);
    bool right = solid(p + MOVE[1]);
    bool down = solid(p + MOVE[2]);
    bool left = solid(p + MOVE[3]);
    return (up && right) || (right && down) || (down && left) || (left && up);
}

static bool local_clear_parking_is_accepted(StrategyClearParking parking) {
    return parking == StrategyClearParking::DIRECT_SAFE ||
           parking == StrategyClearParking::THEORETICAL_RESCUE;
}

static StrategyRescueObligationKind local_clear_obligation_kind_for_parking(StrategyClearParking parking) {
    if (parking == StrategyClearParking::DIRECT_SAFE) return StrategyRescueObligationKind::NONE;
    if (parking == StrategyClearParking::OPEN_PATH_ONLY) return StrategyRescueObligationKind::EXPLICIT_PHASE1_TASK;
    return StrategyRescueObligationKind::UNRESOLVED;
}

static bool local_clear_has_target_on_same_wall_line(
    const SokobanLevel& clear_lvl,
    point p,
    bool vertical_wall)
{
    for (int t = 0; t < clear_lvl.target_count; ++t) {
        if (vertical_wall && clear_lvl.targets[t].x == p.x) return true;
        if (!vertical_wall && clear_lvl.targets[t].y == p.y) return true;
    }
    return false;
}

static bool local_clear_is_permanent_wall(const SokobanLevel& clear_lvl, point p) {
    if (!PlanningCommon::in_bounds(p)) return true;
    return clear_lvl.map[p.y][p.x] == 1 && !PlanningCommon::is_blastable_wall(clear_lvl, p);
}

static bool local_clear_touches_permanent_boundary_wall(const SokobanLevel& clear_lvl, point p) {
    return local_clear_is_permanent_wall(clear_lvl, p + MOVE[0]) ||
           local_clear_is_permanent_wall(clear_lvl, p + MOVE[1]) ||
           local_clear_is_permanent_wall(clear_lvl, p + MOVE[2]) ||
           local_clear_is_permanent_wall(clear_lvl, p + MOVE[3]);
}

static bool local_clear_is_permanent_wall_line_deadlock(const SokobanLevel& clear_lvl, point p) {
    bool left_or_right = local_clear_is_permanent_wall(clear_lvl, p + MOVE[1]) ||
                         local_clear_is_permanent_wall(clear_lvl, p + MOVE[3]);
    if (left_or_right && !local_clear_has_target_on_same_wall_line(clear_lvl, p, true)) return true;

    bool up_or_down = local_clear_is_permanent_wall(clear_lvl, p + MOVE[0]) ||
                      local_clear_is_permanent_wall(clear_lvl, p + MOVE[2]);
    if (up_or_down && !local_clear_has_target_on_same_wall_line(clear_lvl, p, false)) return true;

    return false;
}

static bool local_clear_parking_cell_avoids_static_deadlock(
    const SokobanLevel& clear_lvl,
    int box_id,
    point p,
    bool phase2_specific)
{
    if (box_id < 0 || box_id >= clear_lvl.box_count) return false;
    if (!PlanningCommon::in_bounds(p) || clear_lvl.map[p.y][p.x] == 1) return false;
    if (strategy_is_goal_for_box(clear_lvl, box_id, p, phase2_specific)) return true;
    // 清障临停不能贴永久边界墙，否则箱子只能沿边界线移动
    if (local_clear_touches_permanent_boundary_wall(clear_lvl, p)) return false;
    bool permanent_vertical = local_clear_is_permanent_wall(clear_lvl, p + MOVE[0]) ||
                              local_clear_is_permanent_wall(clear_lvl, p + MOVE[2]);
    bool permanent_horizontal = local_clear_is_permanent_wall(clear_lvl, p + MOVE[1]) ||
                                local_clear_is_permanent_wall(clear_lvl, p + MOVE[3]);
    if (permanent_vertical && permanent_horizontal) return false;
    if (local_clear_is_permanent_wall_line_deadlock(clear_lvl, p)) return false;
    return true;
}

static bool local_clear_parking_avoids_static_deadlock(
    const SokobanLevel& clear_lvl,
    int box_id,
    bool phase2_specific)
{
    if (box_id < 0 || box_id >= clear_lvl.box_count) return false;
    return local_clear_parking_cell_avoids_static_deadlock(
        clear_lvl, box_id, clear_lvl.boxes[box_id], phase2_specific);
}

static uint32_t real_clear_state_key(const SokobanLevel& lvl, point player_pos) {
    uint32_t key = static_cast<uint32_t>((player_pos.y & 15) * MAP_MAX_WIDTH + (player_pos.x & 15));
    for (int b = 0; b < lvl.box_count; ++b) {
        if (!PlanningCommon::in_bounds(lvl.boxes[b])) continue;
        uint32_t cell = static_cast<uint32_t>(lvl.boxes[b].y * MAP_MAX_WIDTH + lvl.boxes[b].x + 1);
        key = key * 131U + cell;
    }
    return key;
}

static bool real_clear_same_line(point a, point b) {
    return a.x == b.x || a.y == b.y;
}

static void real_clear_copy_plan_without_index(
    const StaticArray<BoxPushTask, 8>& src,
    int drop,
    StaticArray<BoxPushTask, 8>& dst)
{
    dst.clear();
    for (int i = 0; i < src.size(); ++i) {
        if (i == drop) continue;
        dst.push_back(src[i]);
    }
}

static void real_clear_copy_diags_without_index(
    const StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT>& src,
    int drop,
    StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT>& dst)
{
    dst.clear();
    for (int i = 0; i < src.size(); ++i) {
        if (i == drop) continue;
        dst.push_back(src[i]);
    }
}

static bool real_clear_plan_has_net_return(const StaticArray<BoxPushTask, 8>& plan) {
    for (int i = 0; i < plan.size(); ++i) {
        point start = plan[i].box_start;
        point pos = plan[i].box_target;
        int chain_len = 1;
        for (int j = i + 1; j < plan.size(); ++j) {
            if (!(plan[j].box_start == pos)) continue;
            pos = plan[j].box_target;
            ++chain_len;
            if (chain_len >= 2 && pos == start) {
                return true;
            }
        }
    }
    return false;
}

static uint8_t local_clear_clamp_u8(uint16_t value) {
    return value > 255 ? 255 : static_cast<uint8_t>(value);
}

static bool local_clear_box_cell_free(const SokobanLevel& lvl, point p, int moving_box) {
    if (!PlanningCommon::in_bounds(p) || lvl.map[p.y][p.x] == 1) return false;
    for (int b = 0; b < lvl.box_count; ++b) {
        if (b != moving_box && lvl.boxes[b] == p) return false;
    }
    for (int b = 0; b < lvl.bomb_count; ++b) {
        if (lvl.bombs[b].x != -1 && lvl.bombs[b] == p) return false;
    }
    return true;
}

static bool local_clear_player_cell_free(
    const SokobanLevel& lvl,
    point p,
    int moving_box,
    point obstacle_box)
{
    if (!local_clear_box_cell_free(lvl, p, moving_box)) return false;
    return !(p == obstacle_box);
}

static uint16_t local_clear_player_distance(
    const SokobanLevel& lvl,
    point start,
    point target,
    int moving_box,
    point obstacle_box,
    LocalClearPushReachWorkspace& ws)
{
    if (!PlanningCommon::in_bounds(start) || !PlanningCommon::in_bounds(target)) return 65535;
    if (start == target) return 0;
    if (!local_clear_player_cell_free(lvl, target, moving_box, obstacle_box)) return 65535;

    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            ws.micro_dist[y][x] = 65535;
        }
    }

    int head = 0;
    int tail = 0;
    ws.micro_q[tail++] = start;
    ws.micro_dist[start.y][start.x] = 0;

    while (head < tail) {
        point curr = ws.micro_q[head++];
        uint16_t next_dist = static_cast<uint16_t>(ws.micro_dist[curr.y][curr.x] + 1);
        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (!local_clear_player_cell_free(lvl, np, moving_box, obstacle_box)) continue;
            if (ws.micro_dist[np.y][np.x] != 65535) continue;
            ws.micro_dist[np.y][np.x] = next_dist;
            if (np == target) return next_dist;
            ws.micro_q[tail++] = np;
        }
    }
    return 65535;
}

static bool local_clear_state_better(
    uint16_t new_cost,
    uint8_t new_pushes,
    uint8_t new_turns,
    uint16_t old_cost,
    uint8_t old_pushes,
    uint8_t old_turns)
{
    if (new_cost != old_cost) return new_cost < old_cost;
    if (new_pushes != old_pushes) return new_pushes < old_pushes;
    return new_turns < old_turns;
}

static void local_clear_build_push_reach_map(
    const SokobanLevel& lvl,
    point player,
    int moving_box,
    point box_start,
    LocalClearPushReachMap& reach,
    LocalClearPushReachWorkspace& ws)
{
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            reach.cost[y][x] = 65535;
            reach.pushes[y][x] = 255;
            reach.turns[y][x] = 255;
            reach.first_access[y][x] = 255;
            for (int d = 0; d < 4; ++d) {
                ws.state_cost[y][x][d] = 65535;
                ws.state_pushes[y][x][d] = 255;
                ws.state_turns[y][x][d] = 255;
                ws.state_first_access[y][x][d] = 255;
                ws.state_closed[y][x][d] = false;
            }
        }
    }

    auto relax_state = [&](point box, int dir, uint16_t cost, uint8_t pushes, uint8_t turns, uint8_t first_access) {
        if (!PlanningCommon::in_bounds(box)) return;
        if (!local_clear_state_better(
                cost,
                pushes,
                turns,
                ws.state_cost[box.y][box.x][dir],
                ws.state_pushes[box.y][box.x][dir],
                ws.state_turns[box.y][box.x][dir])) {
            return;
        }
        ws.state_cost[box.y][box.x][dir] = cost;
        ws.state_pushes[box.y][box.x][dir] = pushes;
        ws.state_turns[box.y][box.x][dir] = turns;
        ws.state_first_access[box.y][box.x][dir] = first_access;
    };

    auto relax_cell = [&](point box, uint16_t cost, uint8_t pushes, uint8_t turns, uint8_t first_access) {
        if (!PlanningCommon::in_bounds(box) || box == box_start) return;
        if (!local_clear_state_better(
                cost,
                pushes,
                turns,
                reach.cost[box.y][box.x],
                reach.pushes[box.y][box.x],
                reach.turns[box.y][box.x])) {
            return;
        }
        reach.cost[box.y][box.x] = cost;
        reach.pushes[box.y][box.x] = pushes;
        reach.turns[box.y][box.x] = turns;
        reach.first_access[box.y][box.x] = first_access;
    };

    for (int d = 0; d < 4; ++d) {
        point push_from = box_start - MOVE[d];
        uint16_t access = local_clear_player_distance(lvl, player, push_from, moving_box, box_start, ws);
        if (access == 65535) continue;
        relax_state(box_start, d, access, 0, 0, local_clear_clamp_u8(access));
    }

    while (true) {
        uint16_t best_cost = 65535;
        point best_box = {-1, -1};
        int best_dir = -1;

        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                for (int d = 0; d < 4; ++d) {
                    if (ws.state_closed[y][x][d]) continue;
                    uint16_t cost = ws.state_cost[y][x][d];
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_box = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
                        best_dir = d;
                    }
                }
            }
        }

        if (best_dir < 0) break;
        ws.state_closed[best_box.y][best_box.x][best_dir] = true;

        uint8_t pushes = ws.state_pushes[best_box.y][best_box.x][best_dir];
        uint8_t turns = ws.state_turns[best_box.y][best_box.x][best_dir];
        uint8_t first_access = ws.state_first_access[best_box.y][best_box.x][best_dir];

        point next_box = best_box + MOVE[best_dir];
        if (local_clear_box_cell_free(lvl, next_box, moving_box)) {
            uint16_t next_cost = static_cast<uint16_t>(best_cost + 1);
            uint8_t next_pushes = pushes < 255 ? static_cast<uint8_t>(pushes + 1) : 255;
            relax_state(next_box, best_dir, next_cost, next_pushes, turns, first_access);
            relax_cell(next_box, next_cost, next_pushes, turns, first_access);
        }

        point current_player = best_box - MOVE[best_dir];
        for (int nd = 0; nd < 4; ++nd) {
            if (nd == best_dir) continue;
            point next_stand = best_box - MOVE[nd];
            uint16_t walk = local_clear_player_distance(lvl, current_player, next_stand, moving_box, best_box, ws);
            if (walk == 65535) continue;
            uint16_t next_cost = static_cast<uint16_t>(best_cost + walk);
            uint8_t next_turns = turns < 255 ? static_cast<uint8_t>(turns + 1) : 255;
            relax_state(best_box, nd, next_cost, pushes, next_turns, first_access);
        }
    }
}

bool StrategicPlanner::local_clear_bomb_route(
    const SokobanLevel& start_lvl,
    int bomb_idx,
    point target_wall,
    bool phase2_specific,
    SokobanLevel& out_lvl,
    int& out_cost,
    StaticArray<BoxPushTask, 8>& out_box_pushes,
    bool include_player_access_clear,
    StaticArray<StrategyClearObligation, 8>* out_obligations,
    uint8_t creator_task_index)
{
    // 6.1 清障缓存与诊断入口
    this->record_profile_local_clear_call();
    if (bomb_idx < 0 || bomb_idx >= start_lvl.bomb_count) return false;
    if (start_lvl.bombs[bomb_idx].x == -1) return false;
    if (!PlanningCommon::is_blastable_wall(start_lvl, target_wall)) return false;

    static uint64_t failure_cache[LOCAL_CLEAR_FAILURE_CACHE_LIMIT] = {};
    static int failure_cache_count = 0;
    static int failure_cache_cursor = 0;

    auto mix_cache_key = [](uint64_t key, uint32_t value) -> uint64_t {
        key ^= static_cast<uint64_t>(value) + 0x9e3779b97f4a7c15ULL + (key << 6) + (key >> 2);
        return key;
    };

    auto local_clear_cache_key = [&](const SokobanLevel& lvl) -> uint64_t {
        uint64_t key = 1469598103934665603ULL;
        key = mix_cache_key(key, static_cast<uint32_t>(bomb_idx));
        key = mix_cache_key(key, static_cast<uint32_t>((target_wall.y & 31) * MAP_MAX_WIDTH + (target_wall.x & 31)));
        key = mix_cache_key(key, phase2_specific ? 1U : 0U);
        key = mix_cache_key(key, include_player_access_clear ? 1U : 0U);
        key = mix_cache_key(key, static_cast<uint32_t>((lvl.player_start.y & 31) * MAP_MAX_WIDTH + (lvl.player_start.x & 31)));
        key = mix_cache_key(key, static_cast<uint32_t>(lvl.box_count));
        key = mix_cache_key(key, static_cast<uint32_t>(lvl.bomb_count));
        key = mix_cache_key(key, static_cast<uint32_t>(lvl.target_count));
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                key = mix_cache_key(key, static_cast<uint32_t>(lvl.map[y][x]));
            }
        }
        for (int b = 0; b < MAX_BOXES; ++b) {
            uint32_t box_cell = PlanningCommon::in_bounds(lvl.boxes[b]) ?
                static_cast<uint32_t>(lvl.boxes[b].y * MAP_MAX_WIDTH + lvl.boxes[b].x + 1) : 0U;
            uint32_t target_cell = PlanningCommon::in_bounds(lvl.targets[b]) ?
                static_cast<uint32_t>(lvl.targets[b].y * MAP_MAX_WIDTH + lvl.targets[b].x + 1) : 0U;
            key = mix_cache_key(key, box_cell);
            // 这里只隔离第一阶段和第二阶段共用清障缓存，不参与第一阶段候选墙评分
            key = mix_cache_key(key, static_cast<uint32_t>(lvl.box_semantics[b]));
            key = mix_cache_key(key, target_cell);
            key = mix_cache_key(key, static_cast<uint32_t>(lvl.target_semantics[b]));
        }
        for (int b = 0; b < MAX_BOMBS; ++b) {
            uint32_t bomb_cell = PlanningCommon::in_bounds(lvl.bombs[b]) ?
                static_cast<uint32_t>(lvl.bombs[b].y * MAP_MAX_WIDTH + lvl.bombs[b].x + 1) : 0U;
            key = mix_cache_key(key, bomb_cell);
        }
        return key;
    };

    uint64_t failure_key = local_clear_cache_key(start_lvl);
    for (int i = 0; i < failure_cache_count; ++i) {
        if (failure_cache[i] == failure_key) return false;
    }

    auto remember_local_clear_failure = [&]() {
        if (failure_cache_count < LOCAL_CLEAR_FAILURE_CACHE_LIMIT) {
            failure_cache[failure_cache_count++] = failure_key;
            return;
        }
        failure_cache[failure_cache_cursor] = failure_key;
        failure_cache_cursor = (failure_cache_cursor + 1) % LOCAL_CLEAR_FAILURE_CACHE_LIMIT;
    };

    SokobanLevel work = start_lvl;
    point player = start_lvl.player_start;
    out_box_pushes.clear();
    if (out_obligations) out_obligations->clear();
    out_cost = 0;
    StrategyClearRouteTrace* clear_diag = this->begin_profile_clear(
        start_lvl, bomb_idx, target_wall, phase2_specific, include_player_access_clear);

    static int16_t soft_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static int16_t box_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static point route[MAP_CELL_COUNT];
    static bool route_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

    // 6.2 停车安全性与未来救援判定
    auto bomb_can_rescue_box = [&](const SokobanLevel& lvl, int box_id, point box_pos) -> bool {
        if (local_clear_cheb(target_wall, box_pos) <= 2) return true;
        if (phase2_specific) {
            for (int t = 0; t < lvl.target_count; ++t) {
                if (strategy_target_allowed_for_box(lvl, box_id, t, true) &&
                    local_clear_cheb(target_wall, lvl.targets[t]) <= 2) {
                    return true;
                }
            }
        }

        for (int b = 0; b < lvl.bomb_count; ++b) {
            if (b == bomb_idx || lvl.bombs[b].x == -1) continue;
            this->macro_soft_dijkstra(lvl, lvl.bombs[b], soft_dist);
            for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
                for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                    point wall = {(int8_t)x, (int8_t)y};
                    if (!PlanningCommon::is_blastable_wall(lvl, wall) ||
                        soft_dist[y][x] == INF_DIST) {
                        continue;
                    }
                    if (local_clear_cheb(wall, box_pos) <= 2) return true;
                    if (phase2_specific) {
                        for (int t = 0; t < lvl.target_count; ++t) {
                            if (strategy_target_allowed_for_box(lvl, box_id, t, true) &&
                                local_clear_cheb(wall, lvl.targets[t]) <= 2) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    };

    auto classify_box_parking = [&](const SokobanLevel& lvl, int box_id, point check_player) -> StrategyClearParking {
        point box_pos = lvl.boxes[box_id];
        bool on_valid_target = strategy_is_goal_for_box(lvl, box_id, box_pos, phase2_specific);
        if (!local_clear_parking_avoids_static_deadlock(lvl, box_id, phase2_specific)) {
            return StrategyClearParking::DEAD_PARKING;
        }

        this->fast_push_bfs(lvl, box_pos, check_player, false, box_dist, true);
        bool can_reach_goal = false;
        for (int t = 0; t < lvl.target_count; ++t) {
            if (!strategy_target_allowed_for_box(lvl, box_id, t, phase2_specific)) continue;
            point goal = lvl.targets[t];
            if (box_dist[goal.y][goal.x] != INF_DIST) {
                can_reach_goal = true;
                break;
            }
        }

        if (can_reach_goal || on_valid_target) return StrategyClearParking::DIRECT_SAFE;
        if (bomb_can_rescue_box(lvl, box_id, box_pos)) return StrategyClearParking::THEORETICAL_RESCUE;
        return StrategyClearParking::DEAD_PARKING;
    };

    auto profile_obligation_for_parking = [&](StrategyClearParking parking) -> StrategyRescueObligationKind {
        // 只有最终实体化序列会传入 out_obligations，搜索探测诊断不能被解释成真实债务
        if (!out_obligations) return StrategyRescueObligationKind::NONE;
        return local_clear_obligation_kind_for_parking(parking);
    };

    auto append_clear_obligation = [&](uint8_t box_id,
                                       StrategyClearReason reason,
                                       StrategyClearParking parking,
                                       point box_start,
                                       point box_target) {
        if (!out_obligations) return;
        StrategyClearObligation obligation;
        obligation.box_id = box_id;
        obligation.reason = reason;
        obligation.parking = parking;
        obligation.obligation = local_clear_obligation_kind_for_parking(parking);
        obligation.creator_task_index = creator_task_index;
        if (obligation.obligation == StrategyRescueObligationKind::EXPLICIT_PHASE1_TASK) {
            obligation.owner_task_index = creator_task_index;
        }
        obligation.box_start = box_start;
        obligation.box_target = box_target;
        out_obligations->push_back(obligation);
    };

    auto owner_bomb_after_clear = [&](const SokobanLevel& clear_lvl,
                                      point clear_player,
                                      SokobanLevel& out_after_blast,
                                      point& out_after_player,
                                      int* out_bomb_path_len = nullptr) -> bool {
        BombTask probe_bomb;
        probe_bomb.bomb_start = clear_lvl.bombs[bomb_idx];
        probe_bomb.target_wall = target_wall;
        probe_bomb.is_essential = false;
        probe_bomb.net_profit = 0;
        probe_bomb.box_pushes.clear();

        StaticArray<point, MAX_PATH_LENGTH> bomb_path;
        bool bomb_ok = PlanningCommon::get_bomb_push_path(clear_lvl, clear_player, probe_bomb, bomb_path);
        this->record_profile_bomb_path_check(bomb_ok);
        if (!bomb_ok) return false;

        out_after_blast = clear_lvl;
        out_after_player = bomb_path.empty() ? clear_player : bomb_path.back();
        out_after_blast.bombs[bomb_idx] = {-1, -1};
        PlanningCommon::apply_blast_effect(out_after_blast, target_wall);
        if (out_bomb_path_len) *out_bomb_path_len = bomb_path.size();
        return true;
    };

    auto parking_avoids_static_deadlock = [&](const SokobanLevel& clear_lvl, int box_id) -> bool {
        return local_clear_parking_avoids_static_deadlock(clear_lvl, box_id, phase2_specific);
    };

    auto open_path_parking_is_valid = [&](StrategyClearParking parking,
                                          const SokobanLevel& clear_lvl,
                                          point clear_player,
                                          int box_id) -> bool {
        if (parking != StrategyClearParking::OPEN_PATH_ONLY) return true;
        // OPEN_PATH_ONLY 只允许可逆中转，边界/角落硬死局不能登记成未来债务
        if (!parking_avoids_static_deadlock(clear_lvl, box_id)) return false;

        SokobanLevel after_blast;
        point after_player = clear_player;
        if (!owner_bomb_after_clear(clear_lvl, clear_player, after_blast, after_player)) {
            return false;
        }
        return parking_avoids_static_deadlock(after_blast, box_id);
    };

    // 6.3 软路线构建与阻挡箱识别
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

    bool clearing_stack[MAX_BOXES] = {false};

    // 6.4 路线规则清障：搬开阻挡箱并验证炸弹通路
    auto first_push_access_cost_for_clear = [&](const SokobanLevel& lvl,
                                                point cur_player,
                                                int moving_box,
                                                point box_start,
                                                point target) -> int {
        point push_dir = {0, 0};
        if (target.x == box_start.x && target.y != box_start.y) {
            push_dir.y = (target.y > box_start.y) ? 1 : -1;
        } else if (target.y == box_start.y && target.x != box_start.x) {
            push_dir.x = (target.x > box_start.x) ? 1 : -1;
        } else {
            return 180;
        }

        point push_from = {
            static_cast<int8_t>(box_start.x - push_dir.x),
            static_cast<int8_t>(box_start.y - push_dir.y)
        };
        if (!PlanningCommon::in_bounds(push_from) || lvl.map[push_from.y][push_from.x] == 1) {
            return 180;
        }

        for (int b = 0; b < lvl.box_count; ++b) {
            if (b != moving_box && lvl.boxes[b] == push_from) return 180;
        }
        for (int b = 0; b < lvl.bomb_count; ++b) {
            if (lvl.bombs[b].x != -1 && lvl.bombs[b] == push_from) return 180;
        }

        // 首推发力位决定清障动作是否顺手，预筛阶段必须显式计入
        this->record_profile_player_path_check();
        uint16_t access = PlanningCommon::bfs_shortest_path(lvl, cur_player, push_from);
        if (access == 65535) return 180;
        return access > 180 ? 180 : static_cast<int>(access);
    };

    auto clear_box_recursive = [&](auto& self, int box_id, int depth, StrategyClearReason reason) -> bool {
        if (box_id < 0 || box_id >= work.box_count) return false;
        if (clearing_stack[box_id]) return false;
        if (out_box_pushes.size() >= LOCAL_CLEAR_MAX_TASKS) return false;

        struct ClearCandidate {
            point p;
            int score;
            bool opens_bomb_path;
            int order;
            uint8_t min_route_dist;
        };

        ClearCandidate candidates[MAP_CELL_COUNT];
        int candidate_count = 0;
        point box_start = work.boxes[box_id];
        clearing_stack[box_id] = true;
        bool real_path_blocker = reason == StrategyClearReason::BOMB_REAL_PATH_BLOCKER;
        static LocalClearPushReachWorkspace push_reach_ws;
        LocalClearPushReachMap push_reach;
        local_clear_build_push_reach_map(work, player, box_id, box_start, push_reach, push_reach_ws);

        auto first_push_access_cost = [&](point target) -> int {
            return first_push_access_cost_for_clear(work, player, box_id, box_start, target);
        };

        auto candidate_open_cost = [&](point target, bool& push_executable) -> int {
            push_executable = false;
            SokobanLevel probe_lvl = work;
            point probe_player = player;
            StaticArray<point, MAX_PATH_LENGTH> push_path;
            BoxPushTask push_task{box_start, target};
            bool push_ok = PlanningCommon::append_box_push_path(probe_lvl, probe_player, push_task, push_path);
            this->record_profile_box_push_check(push_ok);
            if (!push_ok) return 9999;
            push_executable = true;

            BombTask probe_bomb;
            probe_bomb.bomb_start = probe_lvl.bombs[bomb_idx];
            probe_bomb.target_wall = target_wall;
            probe_bomb.is_essential = false;
            probe_bomb.net_profit = 0;
            probe_bomb.box_pushes.clear();

            StaticArray<point, MAX_PATH_LENGTH> bomb_path;
            bool bomb_ok = PlanningCommon::get_bomb_push_path(probe_lvl, probe_player, probe_bomb, bomb_path);
            this->record_profile_bomb_path_check(bomb_ok);
            if (!bomb_ok) return 9999;
            return push_path.size() + bomb_path.size();
        };

        auto target_releases_bomb_path = [&](point target) -> bool {
            point push_dir = {0, 0};
            if (target.x == box_start.x && target.y != box_start.y) {
                push_dir.y = (target.y > box_start.y) ? 1 : -1;
            } else if (target.y == box_start.y && target.x != box_start.x) {
                push_dir.x = (target.x > box_start.x) ? 1 : -1;
            } else {
                return false;
            }

            SokobanLevel probe_lvl = work;
            probe_lvl.boxes[box_id] = target;
            point probe_player = {
                static_cast<int8_t>(target.x - push_dir.x),
                static_cast<int8_t>(target.y - push_dir.y)
            };
            if (!PlanningCommon::in_bounds(probe_player) ||
                probe_lvl.map[probe_player.y][probe_player.x] == 1) {
                return false;
            }

            BombTask probe_bomb;
            probe_bomb.bomb_start = probe_lvl.bombs[bomb_idx];
            probe_bomb.target_wall = target_wall;
            probe_bomb.is_essential = false;
            probe_bomb.net_profit = 0;
            probe_bomb.box_pushes.clear();

            StaticArray<point, MAX_PATH_LENGTH> bomb_path;
            bool bomb_ok = PlanningCommon::get_bomb_push_path(probe_lvl, probe_player, probe_bomb, bomb_path);
            this->record_profile_bomb_path_check(bomb_ok);
            return bomb_ok;
        };

        for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
            for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                point p = {(int8_t)x, (int8_t)y};
                if (!PlanningCommon::in_bounds(p) || work.map[p.y][p.x] == 1 || p == target_wall) continue;

                bool occupied = false;
                for (int b = 0; b < work.box_count; ++b) {
                    if (b != box_id && work.boxes[b] == p) occupied = true;
                }
                for (int b = 0; b < work.bomb_count; ++b) {
                    if (work.bombs[b].x != -1 && work.bombs[b] == p) occupied = true;
                }
                if (occupied) continue;

                // 清障停车点不能占用目标点，否则会被后续推箱回放当作完成目标
                if (strategy_is_any_target_cell(work, p)) continue;

                bool any_target = strategy_is_goal_for_box(work, box_id, p, false);
                bool valid_goal = strategy_is_goal_for_box(work, box_id, p, phase2_specific);
                if (any_target && !valid_goal) continue;
                if (!local_clear_parking_cell_avoids_static_deadlock(work, box_id, p, phase2_specific)) continue;
                if (real_path_blocker && local_clear_is_static_corner(work, p) && !valid_goal) continue;

                int dist_box = std::abs(p.x - box_start.x) + std::abs(p.y - box_start.y);
                if (dist_box == 0) continue;
                if (push_reach.cost[p.y][p.x] == 65535) continue;
                // 真实路径阻断用完整推箱状态图补完拐弯清障，近邻站位仍保持低扰动直线停车
                if (!real_path_blocker && p.x != box_start.x && p.y != box_start.y) continue;
                int min_route_dist = 99;
                for (int ry = 0; ry < MAP_MAX_HEIGHT; ++ry) {
                    for (int rx = 0; rx < MAP_MAX_WIDTH; ++rx) {
                        if (!route_mask[ry][rx]) continue;
                        int rd = std::abs(p.x - rx) + std::abs(p.y - ry);
                        if (rd < min_route_dist) min_route_dist = rd;
                    }
                }

                bool straight_parking = p.x == box_start.x || p.y == box_start.y;
                int score = 0;
                if (straight_parking) {
                    score += dist_box * 8;
                    score += first_push_access_cost(p) * 6;
                } else {
                    score += dist_box * 10;
                    score += static_cast<int>(push_reach.cost[p.y][p.x]) * 8;
                    score += static_cast<int>(push_reach.pushes[p.y][p.x]) * 10;
                    score += static_cast<int>(push_reach.turns[p.y][p.x]) * 35;
                    score += static_cast<int>(push_reach.first_access[p.y][p.x]) * 3;
                }
                if (phase2_specific) {
                    score += (strategy_nearest_goal_distance(work, box_id, p, phase2_specific) -
                              strategy_nearest_goal_distance(work, box_id, box_start, phase2_specific)) * 80;
                } else if (include_player_access_clear) {
                    score += (strategy_nearest_goal_distance(work, box_id, p, false) -
                              strategy_nearest_goal_distance(work, box_id, box_start, false)) * 90;
                }
                if (local_clear_is_static_corner(work, p) && !valid_goal) score += 120;
                if (phase2_specific && valid_goal) score -= 60;

                int order = candidate_count;
                candidates[candidate_count++] = {
                    p,
                    score,
                    false,
                    order,
                    static_cast<uint8_t>(min_route_dist < 255 ? min_route_dist : 255)
                };
            }
        }

        int verify_limit = candidate_count < LOCAL_CLEAR_OPEN_VERIFY_LIMIT ?
            candidate_count : LOCAL_CLEAR_OPEN_VERIFY_LIMIT;
        if (verify_limit > 0) {
            std::partial_sort(
                candidates,
                candidates + verify_limit,
                candidates + candidate_count,
                [](const ClearCandidate& a, const ClearCandidate& b) {
                    if (a.score != b.score) return a.score < b.score;
                    return a.order < b.order;
                });
        }
        for (int i = 0; i < verify_limit; ++i) {
            bool push_executable = false;
            int open_cost = candidate_open_cost(candidates[i].p, push_executable);
            candidates[i].opens_bomb_path = open_cost < 9999;
            if (candidates[i].opens_bomb_path) {
                candidates[i].score += open_cost * 6;
            } else {
                candidates[i].score += 250;
                if (candidates[i].min_route_dist <= 1) candidates[i].score += 80;
                else if (candidates[i].min_route_dist == 2) candidates[i].score += 20;
                if (real_path_blocker &&
                    (push_executable || !target_releases_bomb_path(candidates[i].p))) {
                    candidates[i].score += 100000;
                }
            }
        }
        candidate_count = verify_limit;
        int try_limit = candidate_count < LOCAL_CLEAR_CANDIDATE_LIMIT ? candidate_count : LOCAL_CLEAR_CANDIDATE_LIMIT;
        if (try_limit > 0) {
            std::partial_sort(
                candidates,
                candidates + try_limit,
                candidates + candidate_count,
                [](const ClearCandidate& a, const ClearCandidate& b) {
                    if (a.score != b.score) return a.score < b.score;
                    return a.order < b.order;
                });
        }
        for (int i = 0; i < try_limit; ++i) {
            BoxPushTask task{box_start, candidates[i].p};

            SokobanLevel saved_level = work;
            point saved_player = player;
            StaticArray<BoxPushTask, 8> saved_pushes = out_box_pushes;
            int saved_cost = out_cost;
            uint8_t saved_diag_push_count = clear_diag ? clear_diag->push_count : 0;

            StaticArray<point, MAX_PATH_LENGTH> segment;
            bool push_ok = PlanningCommon::append_box_push_path(work, player, task, segment);
            this->record_profile_box_push_check(push_ok);
            if (push_ok) {
                if (real_path_blocker && !candidates[i].opens_bomb_path) {
                    work = saved_level;
                    player = saved_player;
                    out_box_pushes = saved_pushes;
                    out_cost = saved_cost;
                    if (clear_diag) clear_diag->push_count = saved_diag_push_count;
                    continue;
                }
                StrategyClearParking parking = classify_box_parking(work, box_id, player);
                bool safe_without_open_path = local_clear_parking_is_accepted(parking);
                if (safe_without_open_path &&
                    parking != StrategyClearParking::DIRECT_SAFE &&
                    !parking_avoids_static_deadlock(work, box_id)) {
                    work = saved_level;
                    player = saved_player;
                    out_box_pushes = saved_pushes;
                    out_cost = saved_cost;
                    if (clear_diag) clear_diag->push_count = saved_diag_push_count;
                    continue;
                }
                if (real_path_blocker && !safe_without_open_path) {
                    work = saved_level;
                    player = saved_player;
                    out_box_pushes = saved_pushes;
                    out_cost = saved_cost;
                    if (clear_diag) clear_diag->push_count = saved_diag_push_count;
                    continue;
                }
                if (safe_without_open_path || candidates[i].opens_bomb_path) {
                    StrategyClearParking accepted_parking =
                        safe_without_open_path ? parking : StrategyClearParking::OPEN_PATH_ONLY;
                    if (!open_path_parking_is_valid(accepted_parking, work, player, box_id)) {
                        work = saved_level;
                        player = saved_player;
                        out_box_pushes = saved_pushes;
                        out_cost = saved_cost;
                        if (clear_diag) clear_diag->push_count = saved_diag_push_count;
                        continue;
                    }
                    out_box_pushes.push_back(task);
                    out_cost += segment.size();
                    this->record_shadow_clear_accept(
                        reason,
                        accepted_parking);
                    append_clear_obligation(
                        static_cast<uint8_t>(box_id),
                        reason,
                        accepted_parking,
                        box_start,
                        candidates[i].p);
                    this->record_profile_clear_push(
                        clear_diag,
                        static_cast<uint8_t>(box_id),
                        reason,
                        accepted_parking,
                        profile_obligation_for_parking(accepted_parking),
                        creator_task_index,
                        box_start,
                        candidates[i].p,
                        work.bombs[bomb_idx],
                        target_wall,
                        depth,
                        candidates[i].opens_bomb_path,
                        safe_without_open_path,
                        candidates[i].score);
                    clearing_stack[box_id] = false;
                    return true;
                }
            }

            work = saved_level;
            player = saved_player;
            out_box_pushes = saved_pushes;
            out_cost = saved_cost;
            if (clear_diag) clear_diag->push_count = saved_diag_push_count;

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
                saved_diag_push_count = clear_diag ? clear_diag->push_count : 0;

                if (self(self, other, depth + 1, StrategyClearReason::RECURSIVE_BOX_BLOCKER)) {
                    segment.clear();
                    bool retry_push_ok = PlanningCommon::append_box_push_path(work, player, task, segment);
                    this->record_profile_box_push_check(retry_push_ok);
                    if (retry_push_ok) {
                        bool retry_opens_bomb_path = false;
                        if (real_path_blocker) {
                            BombTask retry_probe;
                            retry_probe.bomb_start = work.bombs[bomb_idx];
                            retry_probe.target_wall = target_wall;
                            retry_probe.is_essential = false;
                            retry_probe.net_profit = 0;
                            retry_probe.box_pushes.clear();
                            StaticArray<point, MAX_PATH_LENGTH> retry_bomb_path;
                            retry_opens_bomb_path = PlanningCommon::get_bomb_push_path(
                                work, player, retry_probe, retry_bomb_path);
                            this->record_profile_bomb_path_check(retry_opens_bomb_path);
                            if (!retry_opens_bomb_path) {
                                work = saved_level;
                                player = saved_player;
                                out_box_pushes = saved_pushes;
                                out_cost = saved_cost;
                                if (clear_diag) clear_diag->push_count = saved_diag_push_count;
                                continue;
                            }
                        }
                        StrategyClearParking parking = classify_box_parking(work, box_id, player);
                        if (local_clear_parking_is_accepted(parking)) {
                            if (parking != StrategyClearParking::DIRECT_SAFE &&
                                !parking_avoids_static_deadlock(work, box_id)) {
                                work = saved_level;
                                player = saved_player;
                                out_box_pushes = saved_pushes;
                                out_cost = saved_cost;
                                if (clear_diag) clear_diag->push_count = saved_diag_push_count;
                                continue;
                            }
                            StrategyClearParking accepted_parking =
                                parking == StrategyClearParking::DIRECT_SAFE ?
                                StrategyClearParking::DIRECT_SAFE :
                                (retry_opens_bomb_path ? StrategyClearParking::OPEN_PATH_ONLY : parking);
                            if (!open_path_parking_is_valid(accepted_parking, work, player, box_id)) {
                                work = saved_level;
                                player = saved_player;
                                out_box_pushes = saved_pushes;
                                out_cost = saved_cost;
                                if (clear_diag) clear_diag->push_count = saved_diag_push_count;
                                continue;
                            }
                            out_box_pushes.push_back(task);
                            out_cost += segment.size();
                            this->record_shadow_clear_accept(reason, accepted_parking);
                            append_clear_obligation(
                                static_cast<uint8_t>(box_id),
                                reason,
                                accepted_parking,
                                box_start,
                                candidates[i].p);
                            this->record_profile_clear_push(
                                clear_diag,
                                static_cast<uint8_t>(box_id),
                                reason,
                                accepted_parking,
                                profile_obligation_for_parking(accepted_parking),
                                creator_task_index,
                                box_start,
                                candidates[i].p,
                                work.bombs[bomb_idx],
                                target_wall,
                                depth,
                                retry_opens_bomb_path,
                                true,
                                candidates[i].score);
                            clearing_stack[box_id] = false;
                            return true;
                        }
                    }
                }

                work = saved_level;
                player = saved_player;
                out_box_pushes = saved_pushes;
                out_cost = saved_cost;
                if (clear_diag) clear_diag->push_count = saved_diag_push_count;
            }
        }

        clearing_stack[box_id] = false;
        return false;
    };

    for (int iter = 0; iter < LOCAL_CLEAR_MAX_ITER; ++iter) {
        StaticArray<point, MAX_PATH_LENGTH> direct_path;
        BombTask direct_probe;
        direct_probe.bomb_start = work.bombs[bomb_idx];
        direct_probe.target_wall = target_wall;
        direct_probe.is_essential = false;
        direct_probe.net_profit = 0;
        direct_probe.box_pushes.clear();
        bool direct_ok = PlanningCommon::get_bomb_push_path(work, player, direct_probe, direct_path);
        this->record_profile_bomb_path_check(direct_ok);
        if (direct_ok) {
            out_lvl = work;
            if (!direct_path.empty()) out_lvl.player_start = direct_path.back();
            else out_lvl.player_start = player;
            out_cost += direct_path.size();
            this->record_profile_local_clear_success();
            this->finish_profile_clear(
                clear_diag,
                true,
                out_box_pushes.empty() ? StrategyClearMethod::DIRECT_BOMB_PATH : StrategyClearMethod::SOFT_ROUTE_CLEAR,
                out_cost);
            return true;
        }

        int route_len = build_soft_route();
        this->record_profile_soft_route_build(route_len);
        if (route_len <= 0) {
            this->finish_profile_clear(clear_diag, false, StrategyClearMethod::NONE, out_cost);
            return false;
        }

        int blockers[MAX_BOXES];
        StrategyClearReason blocker_reasons[MAX_BOXES];
        int blocker_count = 0;
        auto add_blocker = [&](int bid, StrategyClearReason reason, int push_stand_dist = -1) {
            if (bid < 0) return;
            for (int j = 0; j < blocker_count; ++j) {
                if (blockers[j] == bid) return;
            }
            if (blocker_count >= MAX_BOXES) return;
            blockers[blocker_count] = bid;
            blocker_reasons[blocker_count] = reason;
            ++blocker_count;
            this->record_shadow_clear_blocker(reason);
            if (reason == StrategyClearReason::PUSH_STAND_NEARBY) {
                this->record_shadow_push_stand_blocker(push_stand_dist);
            }
        };

        for (int i = 0; i < route_len; ++i) {
            add_blocker(strategy_box_at(work, route[i]), StrategyClearReason::BOMB_CORRIDOR_BLOCKER);
        }

        if constexpr (StrategyConfig::ENABLE_SHADOW_CLEAR_DECISION) {
            if (blocker_count == 0) {
                for (int b = 0; b < work.box_count && blocker_count < MAX_BOXES; ++b) {
                    SokobanLevel probe_lvl = work;
                    point saved_box = probe_lvl.boxes[b];
                    probe_lvl.boxes[b] = {-1, -1};

                    BombTask probe_bomb;
                    probe_bomb.bomb_start = probe_lvl.bombs[bomb_idx];
                    probe_bomb.target_wall = target_wall;
                    probe_bomb.is_essential = false;
                    probe_bomb.net_profit = 0;
                    probe_bomb.box_pushes.clear();

                    StaticArray<point, MAX_PATH_LENGTH> probe_path;
                    bool opens_without_box = PlanningCommon::get_bomb_push_path(
                        probe_lvl, player, probe_bomb, probe_path);
                    this->record_profile_bomb_path_check(opens_without_box);
                    probe_lvl.boxes[b] = saved_box;
                    if (opens_without_box) {
                        // 真实宏/微推炸弹路径被单个箱子阻断，不能按软路线 near-only 降级
                        add_blocker(b, StrategyClearReason::BOMB_REAL_PATH_BLOCKER);
                    }
                }
            }
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
                if constexpr (StrategyConfig::ENABLE_SHADOW_CLEAR_DECISION) {
                    struct NearStandBlocker {
                        int box_id;
                        int dist;
                        int order;
                    };
                    NearStandBlocker near_blockers[MAX_BOXES];
                    int near_count = 0;
                    for (int b = 0; b < work.box_count; ++b) {
                        int md = std::abs(work.boxes[b].x - push_stand.x) + std::abs(work.boxes[b].y - push_stand.y);
                        if (md > SHADOW_CLEAR_NEAR_STAND_MAX_DIST) continue;
                        near_blockers[near_count++] = {b, md, b};
                    }
                    if (near_count > 1) {
                        std::sort(
                            near_blockers,
                            near_blockers + near_count,
                            [](const NearStandBlocker& a, const NearStandBlocker& b) {
                                if (a.dist != b.dist) return a.dist < b.dist;
                                return a.order < b.order;
                            });
                    }
                    int near_limit = near_count < SHADOW_CLEAR_NEAR_STAND_LIMIT ? near_count : SHADOW_CLEAR_NEAR_STAND_LIMIT;
                    for (int n = 0; n < near_limit; ++n) {
                        add_blocker(
                            near_blockers[n].box_id,
                            StrategyClearReason::PUSH_STAND_NEARBY,
                            near_blockers[n].dist);
                        if (blocker_count >= MAX_BOXES) break;
                    }
                } else {
                    for (int b = 0; b < work.box_count; ++b) {
                        int md = std::abs(work.boxes[b].x - push_stand.x) + std::abs(work.boxes[b].y - push_stand.y);
                        if (md > 3) continue;
                        add_blocker(b, StrategyClearReason::PUSH_STAND_NEARBY, md);
                        if (blocker_count >= MAX_BOXES) break;
                    }
                }
                if (blocker_count >= MAX_BOXES) break;
            }
        }

        if (blocker_count == 0) {
            for (int b = 0; b < work.box_count && blocker_count < MAX_BOXES; ++b) {
                for (int i = 0; i < route_len; ++i) {
                    int md = std::abs(work.boxes[b].x - route[i].x) + std::abs(work.boxes[b].y - route[i].y);
                    if (md == 1) {
                        add_blocker(b, StrategyClearReason::ROUTE_NEARBY);
                        break;
                    }
                }
            }
        }
        this->record_profile_clear_route(clear_diag, route_len, blocker_count);
        bool cleared = false;
        for (int i = 0; i < blocker_count; ++i) {
            if (clear_box_recursive(clear_box_recursive, blockers[i], 0, blocker_reasons[i])) {
                cleared = true;
                break;
            }
        }
        this->record_shadow_clear_route_attempt(cleared, blocker_count);
        if (!cleared) {
            break;
        }
    }
    if (clear_diag) clear_diag->push_count = 0;

    // 6.5 真实清障搜索：路线规则失败后的有限 DFS
    // 路线规则失败后，做一个小规模真实清障 DFS
    // 该分支不保存联合状态池，只枚举少量已经能真实执行的单箱推移
    RealClearMemoEntry real_clear_memo[REAL_CLEAR_MEMO_LIMIT];
    int real_clear_memo_count = 0;

    auto real_clear_memo_failed = [&](uint32_t key, int depth_left) -> bool {
        for (int i = 0; i < real_clear_memo_count; ++i) {
            if (real_clear_memo[i].key == key && real_clear_memo[i].depth_left >= depth_left) {
                return true;
            }
        }
        return false;
    };

    auto remember_real_clear_failure = [&](uint32_t key, int depth_left) {
        for (int i = 0; i < real_clear_memo_count; ++i) {
            if (real_clear_memo[i].key != key) continue;
            if (real_clear_memo[i].depth_left < depth_left) {
                real_clear_memo[i].depth_left = static_cast<uint8_t>(depth_left);
            }
            return;
        }
        if (real_clear_memo_count < REAL_CLEAR_MEMO_LIMIT) {
            real_clear_memo[real_clear_memo_count++] = {
                key,
                static_cast<uint8_t>(depth_left)
            };
        }
    };

    auto replay_clear_plan = [&](const StaticArray<BoxPushTask, 8>& plan,
                                 SokobanLevel& out_clear_lvl,
                                 point& out_after_bomb_player,
                                 int& out_plan_cost) -> bool {
        SokobanLevel replay_lvl = start_lvl;
        point replay_player = start_lvl.player_start;
        int replay_cost = 0;

        for (int i = 0; i < plan.size(); ++i) {
            StaticArray<point, MAX_PATH_LENGTH> segment;
            if (!PlanningCommon::append_box_push_path(replay_lvl, replay_player, plan[i], segment)) {
                return false;
            }
            replay_cost += segment.size();
        }

        BombTask probe;
        probe.bomb_start = replay_lvl.bombs[bomb_idx];
        probe.target_wall = target_wall;
        probe.is_essential = false;
        probe.net_profit = 0;
        probe.box_pushes.clear();

        StaticArray<point, MAX_PATH_LENGTH> bomb_path;
        if (!PlanningCommon::get_bomb_push_path(replay_lvl, replay_player, probe, bomb_path)) {
            return false;
        }

        out_clear_lvl = replay_lvl;
        out_after_bomb_player = bomb_path.empty() ? replay_player : bomb_path.back();
        out_plan_cost = replay_cost + bomb_path.size();
        return true;
    };

    auto optimize_successful_clear_plan = [&](
        StaticArray<BoxPushTask, 8>& plan,
        StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT>& diags,
        int& plan_cost) {
        if (plan.size() <= 0) return;

        SokobanLevel replay_lvl;
        point replay_player = {-1, -1};
        int best_cost = plan_cost;
        if (!replay_clear_plan(plan, replay_lvl, replay_player, best_cost)) return;

        bool changed = true;
        while (changed) {
            changed = false;

            for (int i = 0; i + 1 < plan.size(); ++i) {
                if (!(plan[i].box_target == plan[i + 1].box_start)) continue;
                if (!real_clear_same_line(plan[i].box_start, plan[i + 1].box_target)) continue;

                StaticArray<BoxPushTask, 8> trial;
                StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT> trial_diags;
                for (int j = 0; j < i; ++j) trial.push_back(plan[j]);
                bool merged_to_start = plan[i].box_start == plan[i + 1].box_target;
                if (!merged_to_start) {
                    trial.push_back({plan[i].box_start, plan[i + 1].box_target});
                }
                for (int j = i + 2; j < plan.size(); ++j) trial.push_back(plan[j]);

                if (diags.size() == plan.size()) {
                    for (int j = 0; j < i; ++j) trial_diags.push_back(diags[j]);
                    if (!merged_to_start) {
                        StrategyClearPushTrace merged_diag = diags[i + 1];
                        merged_diag.box_start = plan[i].box_start;
                        merged_diag.box_target = plan[i + 1].box_target;
                        trial_diags.push_back(merged_diag);
                    }
                    for (int j = i + 2; j < diags.size(); ++j) trial_diags.push_back(diags[j]);
                } else {
                    trial_diags = diags;
                }

                int trial_cost = 0;
                if (replay_clear_plan(trial, replay_lvl, replay_player, trial_cost) &&
                    trial_cost <= best_cost) {
                    plan = trial;
                    diags = trial_diags;
                    best_cost = trial_cost;
                    changed = true;
                    break;
                }
            }
            if (changed) continue;

            for (int i = plan.size() - 1; i >= 0; --i) {
                StaticArray<BoxPushTask, 8> trial;
                StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT> trial_diags;
                real_clear_copy_plan_without_index(plan, i, trial);
                if (diags.size() == plan.size()) {
                    real_clear_copy_diags_without_index(diags, i, trial_diags);
                } else {
                    trial_diags = diags;
                }

                int trial_cost = 0;
                if (replay_clear_plan(trial, replay_lvl, replay_player, trial_cost) &&
                    trial_cost <= best_cost) {
                    plan = trial;
                    diags = trial_diags;
                    best_cost = trial_cost;
                    changed = true;
                    break;
                }
            }
        }

        plan_cost = best_cost;
    };

    auto real_clear_search = [&](auto& self,
                                SokobanLevel lvl,
                                point cur_player,
                                StaticArray<BoxPushTask, 8> pushes,
                                StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT> push_diags,
                                int cost,
                                int depth,
                                const RealClearSearchConfig& cfg) -> bool {
        this->record_profile_real_clear_node(depth);
        this->record_shadow_real_node();
        int depth_left = static_cast<int>(cfg.max_depth) - depth;
        uint32_t memo_key = real_clear_state_key(lvl, cur_player);
        if (real_clear_memo_failed(memo_key, depth_left)) return false;
        BombTask probe;
        probe.bomb_start = lvl.bombs[bomb_idx];
        probe.target_wall = target_wall;
        probe.is_essential = false;
        probe.net_profit = 0;
        probe.box_pushes.clear();

        StaticArray<point, MAX_PATH_LENGTH> bomb_path;
        bool direct_bomb_ok = PlanningCommon::get_bomb_push_path(lvl, cur_player, probe, bomb_path);
        this->record_profile_bomb_path_check(direct_bomb_ok);
        if (direct_bomb_ok) {
            SokobanLevel after_blast = lvl;
            after_blast.bombs[bomb_idx] = {-1, -1};
            PlanningCommon::apply_blast_effect(after_blast, target_wall);
            bool open_path_remains_reversible = true;
            for (int i = 0; i < push_diags.size(); ++i) {
                const StrategyClearPushTrace& push = push_diags[i];
                if (push.parking != StrategyClearParking::OPEN_PATH_ONLY) continue;
                if (push.box_id >= after_blast.box_count) {
                    open_path_remains_reversible = false;
                    break;
                }
                if (!parking_avoids_static_deadlock(lvl, push.box_id) ||
                    !parking_avoids_static_deadlock(after_blast, push.box_id)) {
                    open_path_remains_reversible = false;
                    break;
                }
            }
            if (!open_path_remains_reversible) {
                return false;
            }
            int optimized_cost = cost + bomb_path.size();
            optimize_successful_clear_plan(pushes, push_diags, optimized_cost);
            // 清障必须留下真实结构收益，不能用箱子往返来单纯给小车换站位
            if (real_clear_plan_has_net_return(pushes)) {
                return false;
            }

            SokobanLevel optimized_lvl;
            point optimized_player = {-1, -1};
            if (!replay_clear_plan(pushes, optimized_lvl, optimized_player, optimized_cost)) {
                return false;
            }

            out_lvl = optimized_lvl;
            out_lvl.player_start = optimized_player;
            out_box_pushes = pushes;
            out_cost = optimized_cost;
            for (int i = 0; i < push_diags.size(); ++i) {
                const StrategyClearPushTrace& push = push_diags[i];
                this->record_shadow_clear_accept(push.reason, push.parking);
                append_clear_obligation(
                    push.box_id,
                    push.reason,
                    push.parking,
                    push.box_start,
                    push.box_target);
                this->record_profile_clear_push(
                    clear_diag,
                    push.box_id,
                    push.reason,
                    push.parking,
                    push.obligation,
                    push.owner_task_index,
                    push.box_start,
                    push.box_target,
                    push.owner_bomb_start,
                    push.owner_target_wall,
                    push.depth,
                    push.opens_bomb_path != 0,
                    push.safe_without_open_path != 0,
                    push.score);
            }
            return true;
        }
        if (depth >= cfg.max_depth || pushes.size() >= LOCAL_CLEAR_MAX_TASKS) {
            remember_real_clear_failure(memo_key, depth_left);
            return false;
        }

        bool support_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        bool access_target_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        std::memset(support_mask, 0, sizeof(support_mask));
        std::memset(access_target_mask, 0, sizeof(access_target_mask));
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
                if (PlanningCommon::in_bounds(push_stand)) {
                    support_mask[push_stand.y][push_stand.x] = true;
                    access_target_mask[push_stand.y][push_stand.x] = true;
                }
                cur = best_p;
            }
            support_mask[lvl.bombs[bomb_idx].y][lvl.bombs[bomb_idx].x] = true;
        }

        if (include_player_access_clear) {
            static point q[MAP_CELL_COUNT];
            static point parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
            static bool vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
            std::memset(vis, 0, sizeof(vis));

            auto soft_player_passable = [&](point p) -> bool {
                if (!PlanningCommon::in_bounds(p) || lvl.map[p.y][p.x] == 1) return false;
                for (int b = 0; b < lvl.bomb_count; ++b) {
                    if (lvl.bombs[b].x != -1 && lvl.bombs[b] == p) return false;
                }
                return true;
            };

            int head = 0;
            int tail = 0;
            if (soft_player_passable(cur_player)) {
                q[tail++] = cur_player;
                vis[cur_player.y][cur_player.x] = true;
                parent[cur_player.y][cur_player.x] = {-1, -1};
            }

            point best_target = {-1, -1};
            while (head < tail && best_target.x == -1) {
                point curr = q[head++];
                if (access_target_mask[curr.y][curr.x]) {
                    best_target = curr;
                    break;
                }
                for (int d = 0; d < 4; ++d) {
                    point np = curr + MOVE[d];
                    if (!soft_player_passable(np) || vis[np.y][np.x]) continue;
                    vis[np.y][np.x] = true;
                    parent[np.y][np.x] = curr;
                    q[tail++] = np;
                }
            }

            for (point p = best_target;
                 PlanningCommon::in_bounds(p);
                 p = parent[p.y][p.x]) {
                support_mask[p.y][p.x] = true;
                if (p == cur_player) break;
            }
        }

        int8_t support_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                support_dist[y][x] = 99;
            }
        }

        point support_q[MAP_CELL_COUNT];
        int support_head = 0;
        int support_tail = 0;
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                if (!support_mask[y][x] || lvl.map[y][x] == 1) continue;
                support_dist[y][x] = 0;
                support_q[support_tail++] = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
            }
        }
        while (support_head < support_tail) {
            point curr = support_q[support_head++];
            int8_t next_dist = static_cast<int8_t>(support_dist[curr.y][curr.x] + 1);
            for (int d = 0; d < 4; ++d) {
                point np = curr + MOVE[d];
                if (!PlanningCommon::in_bounds(np) || lvl.map[np.y][np.x] == 1) continue;
                if (support_dist[np.y][np.x] != 99) continue;
                support_dist[np.y][np.x] = next_dist;
                support_q[support_tail++] = np;
            }
        }

        auto occupied_by_other_entity = [&](point p, int moving_box) -> bool {
            for (int other = 0; other < lvl.box_count; ++other) {
                if (other != moving_box && lvl.boxes[other] == p) return true;
            }
            for (int bomb = 0; bomb < lvl.bomb_count; ++bomb) {
                if (lvl.bombs[bomb].x != -1 && lvl.bombs[bomb] == p) return true;
            }
            return false;
        };

        RealClearCandidate candidates[REAL_CLEAR_CANDIDATE_POOL];
        int candidate_count = 0;
        int candidate_order = 0;

        auto keep_real_candidate = [&](const RealClearCandidate& candidate) {
            if (candidate_count < REAL_CLEAR_CANDIDATE_POOL) {
                candidates[candidate_count++] = candidate;
                return;
            }
            int worst = 0;
            for (int i = 1; i < candidate_count; ++i) {
                if (candidates[i].score > candidates[worst].score ||
                    (candidates[i].score == candidates[worst].score &&
                     candidates[i].order > candidates[worst].order)) {
                    worst = i;
                }
            }
            if (candidate.score < candidates[worst].score ||
                (candidate.score == candidates[worst].score &&
                 candidate.order < candidates[worst].order)) {
                candidates[worst] = candidate;
            }
        };

        for (int b = 0; b < lvl.box_count; ++b) {
            point box = lvl.boxes[b];
            if (!PlanningCommon::in_bounds(box)) continue;
            // 真实清障只搬开未完成的阻挡箱，避免链式搜索把已落位箱子再次挪走
            if (strategy_is_goal_for_box(lvl, b, box, phase2_specific)) continue;
            int source_support_dist = support_dist[box.y][box.x];
            if (source_support_dist == 99) continue;
            this->record_shadow_real_source_distance(source_support_dist);
            if (source_support_dist > cfg.max_source_support_dist) continue;

            bool has_support_tangent = false;
            RealClearCandidate support_tangent = {};
            for (int dir = 0; dir < 4; ++dir) {
                for (int step = 1; step <= REAL_CLEAR_TARGET_STEP_LIMIT; ++step) {
                    point target = {
                        static_cast<int8_t>(box.x + MOVE[dir].x * step),
                        static_cast<int8_t>(box.y + MOVE[dir].y * step)
                    };
                    if (!PlanningCommon::in_bounds(target)) break;
                    if (lvl.map[target.y][target.x] == 1) break;
                    int target_support_dist = support_dist[target.y][target.x];
                    if (target_support_dist == 99) continue;
                    if (occupied_by_other_entity(target, b)) continue;
                    if (target == target_wall) continue;
                    // 真实清障只负责让路，不允许把箱子停到任意目标点上
                    if (strategy_is_any_target_cell(lvl, target)) continue;

                    bool any_target = strategy_is_goal_for_box(lvl, b, target, false);
                    bool valid_goal = strategy_is_goal_for_box(lvl, b, target, phase2_specific);
                    if (any_target && !valid_goal) continue;
                    if (!local_clear_parking_cell_avoids_static_deadlock(lvl, b, target, phase2_specific)) continue;

                    int first_push_cost = first_push_access_cost_for_clear(lvl, cur_player, b, box, target);
                    bool moves_out_of_support = target_support_dist > source_support_dist && target_support_dist != 0;
                    if (!moves_out_of_support && (step != 1 || first_push_cost > 12)) {
                        continue;
                    }
                    int score = source_support_dist * 40 + step * 8 + target_support_dist * 3;
                    score += first_push_cost * 10;
                    if (first_push_cost >= 180) score += 500;
                    if (!moves_out_of_support) {
                        score += 220 + (source_support_dist - target_support_dist) * 40;
                    } else if (source_support_dist == 0 && target_support_dist == 1) {
                        score -= 18;
                    }
                    // 清障中转可以朝目标方向靠近，但不要因为直接入目标挤掉当前开路任务
                    if (!valid_goal && phase2_specific) {
                        score += (strategy_nearest_goal_distance(lvl, b, target, phase2_specific) -
                                  strategy_nearest_goal_distance(lvl, b, box, phase2_specific)) * 200;
                    } else if (!valid_goal && include_player_access_clear) {
                        score += (strategy_nearest_goal_distance(lvl, b, target, false) -
                                  strategy_nearest_goal_distance(lvl, b, box, false)) * 180;
                    }
                    if (local_clear_is_static_corner(lvl, target) && !valid_goal) score += 160;

                    RealClearCandidate candidate = {
                        {box, target},
                        score,
                        candidate_order++,
                        static_cast<uint8_t>(b),
                        StrategyClearParking::UNKNOWN,
                        false,
                        static_cast<uint8_t>(source_support_dist),
                        static_cast<uint8_t>(target_support_dist),
                        static_cast<uint8_t>(step),
                        static_cast<uint16_t>(first_push_cost)
                    };
                    if (!moves_out_of_support) {
                        if (!has_support_tangent ||
                            candidate.score < support_tangent.score ||
                            (candidate.score == support_tangent.score &&
                             candidate.order < support_tangent.order)) {
                            support_tangent = candidate;
                            has_support_tangent = true;
                        }
                    } else {
                        keep_real_candidate(candidate);
                    }
                }
            }
            if (has_support_tangent) {
                keep_real_candidate(support_tangent);
            }
        }

        if (candidate_count > 0) {
            std::sort(
                candidates,
                candidates + candidate_count,
                [](const RealClearCandidate& a, const RealClearCandidate& b) {
                    if (a.score != b.score) return a.score < b.score;
                    return a.order < b.order;
                });
        }

        RealClearCandidate verified[REAL_CLEAR_CANDIDATE_POOL];
        int verified_count = 0;
        int stage_verify_limit = cfg.verify_scan_limit < REAL_CLEAR_VERIFY_SCAN_LIMIT ?
            cfg.verify_scan_limit : REAL_CLEAR_VERIFY_SCAN_LIMIT;
        int verify_scan_limit = candidate_count < stage_verify_limit ?
            candidate_count : stage_verify_limit;

        for (int i = 0; i < verify_scan_limit; ++i) {
            int b = candidates[i].box_id;
            point target = candidates[i].task.box_target;
            SokobanLevel next_lvl = lvl;
            point next_player = cur_player;
            StaticArray<point, MAX_PATH_LENGTH> segment;
            BoxPushTask task = candidates[i].task;
            this->record_shadow_real_push_candidate();
            bool push_ok = PlanningCommon::append_box_push_path(next_lvl, next_player, task, segment);
            this->record_profile_box_push_check(push_ok);
            if (!push_ok) continue;
            this->record_shadow_real_push_executable();

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
            this->record_profile_bomb_path_check(opens_bomb_path);
            if (opens_bomb_path) this->record_shadow_real_opens_path();
            StrategyClearParking parking = StrategyClearParking::OPEN_PATH_ONLY;
            if (!opens_bomb_path) {
                parking = classify_box_parking(next_lvl, b, next_player);
                bool accepted = local_clear_parking_is_accepted(parking);
                this->record_shadow_real_parking(parking, accepted);
                if (!accepted) continue;
                if (parking == StrategyClearParking::THEORETICAL_RESCUE &&
                    !cfg.allow_theoretical_rescue) {
                    continue;
                }
                if (parking != StrategyClearParking::DIRECT_SAFE &&
                    !parking_avoids_static_deadlock(next_lvl, b)) {
                    continue;
                }
            } else if (!parking_avoids_static_deadlock(next_lvl, b)) {
                continue;
            }

            uint16_t push_time = PlanningCommon::path_time_cost(cur_player, segment);
            uint16_t bomb_time = opens_bomb_path ?
                PlanningCommon::path_time_cost(next_player, bomb_after_push) : 0;
            int score = candidates[i].source_support_dist * 30 + candidates[i].move_dist * 8;
            score += candidates[i].first_push_cost * 3;
            if (opens_bomb_path) score += (push_time + bomb_time) * 5;
            else score += 250 + push_time * 5;
            bool target_is_goal = strategy_is_goal_for_box(lvl, b, target, phase2_specific);
            // 非目标中转保留目标距离梯度，直接入目标必须等当前炸弹路径已打开后再奖励
            if (!target_is_goal && phase2_specific) {
                score += (strategy_nearest_goal_distance(lvl, b, target, phase2_specific) -
                          strategy_nearest_goal_distance(lvl, b, lvl.boxes[b], phase2_specific)) * 200;
            } else if (!target_is_goal && include_player_access_clear) {
                score += (strategy_nearest_goal_distance(lvl, b, target, false) -
                          strategy_nearest_goal_distance(lvl, b, lvl.boxes[b], false)) * 180;
            } else if (target_is_goal && opens_bomb_path) {
                score -= 80;
            }
            if (!opens_bomb_path) {
                if (candidates[i].target_support_dist <= 1) score += 120;
                else if (candidates[i].target_support_dist == 2) score += 40;
            }
            if (local_clear_is_static_corner(next_lvl, target) &&
                !strategy_is_goal_for_box(next_lvl, b, target, phase2_specific)) {
                score += 160;
            }

            if (verified_count < REAL_CLEAR_CANDIDATE_POOL) {
                verified[verified_count++] = {
                    task,
                    score,
                    candidates[i].order,
                    static_cast<uint8_t>(b),
                    parking,
                    opens_bomb_path,
                    candidates[i].source_support_dist,
                    candidates[i].target_support_dist,
                    candidates[i].move_dist,
                    candidates[i].first_push_cost
                };
            }
        }

        int stage_branch_limit = cfg.branch_limit < REAL_CLEAR_BRANCH_LIMIT ?
            cfg.branch_limit : REAL_CLEAR_BRANCH_LIMIT;
        int try_limit = verified_count < stage_branch_limit ? verified_count : stage_branch_limit;
        this->record_profile_real_clear_candidates(verified_count, try_limit);
        if (try_limit > 0) {
            std::partial_sort(
                verified,
                verified + try_limit,
                verified + verified_count,
                [](const RealClearCandidate& a, const RealClearCandidate& b) {
                    if (a.score != b.score) return a.score < b.score;
                    return a.order < b.order;
                });
        }
        for (int i = 0; i < try_limit; ++i) {
            SokobanLevel next_lvl = lvl;
            point next_player = cur_player;
            StaticArray<point, MAX_PATH_LENGTH> segment;
            bool push_ok = PlanningCommon::append_box_push_path(next_lvl, next_player, verified[i].task, segment);
            this->record_profile_box_push_check(push_ok);
            if (!push_ok) continue;

            StaticArray<BoxPushTask, 8> next_pushes = pushes;
            next_pushes.push_back(verified[i].task);
            StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT> next_push_diags = push_diags;
            if (next_push_diags.size() < StrategyConfig::REAL_CLEAR_TRACE_LIMIT) {
                StrategyClearPushTrace push_diag;
                push_diag.box_id = verified[i].box_id;
                push_diag.reason = StrategyClearReason::REAL_CLEAR_SUPPORT;
                push_diag.parking = verified[i].parking;
                push_diag.obligation = profile_obligation_for_parking(verified[i].parking);
                push_diag.owner_task_index = creator_task_index;
                push_diag.box_start = verified[i].task.box_start;
                push_diag.box_target = verified[i].task.box_target;
                push_diag.owner_bomb_start = lvl.bombs[bomb_idx];
                push_diag.owner_target_wall = target_wall;
                push_diag.depth = static_cast<uint8_t>(depth < 255 ? depth : 255);
                push_diag.opens_bomb_path = verified[i].opens_bomb_path ? 1 : 0;
                push_diag.safe_without_open_path = verified[i].parking != StrategyClearParking::OPEN_PATH_ONLY ? 1 : 0;
                push_diag.score = strategy_clamp_i16(verified[i].score);
                next_push_diags.push_back(push_diag);
            }
            if (self(self, next_lvl, next_player, next_pushes, next_push_diags, cost + segment.size(), depth + 1, cfg)) {
                return true;
            }
        }

        remember_real_clear_failure(memo_key, depth_left);
        return false;
    };

    {
        StaticArray<BoxPushTask, 8> pushes;
        StaticArray<StrategyClearPushTrace, StrategyConfig::REAL_CLEAR_TRACE_LIMIT> push_diags;
        RealClearSearchConfig cfg = include_player_access_clear ?
            RealClearSearchConfig{3, 2, 10, 5, true} :
            RealClearSearchConfig{3, 2, 12, 6, true};
        if (real_clear_search(
                real_clear_search,
                start_lvl,
                start_lvl.player_start,
                pushes,
                push_diags,
                0,
                0,
                cfg)) {
            this->record_profile_local_clear_success();
            this->finish_profile_clear(clear_diag, true, StrategyClearMethod::REAL_CLEAR_SEARCH, out_cost);
            return true;
        }
    }

    remember_local_clear_failure();
    this->finish_profile_clear(clear_diag, false, StrategyClearMethod::NONE, out_cost);
    return false;
}


// ============================================================================
// 7. 炸弹任务实体化与快速可执行性验证
// ============================================================================

// 将策略层的候选炸弹任务补全为可执行任务，必要时生成推箱让路序列
/// \brief 将候选炸弹任务补全为真实可执行任务
/// \param level 当前地图状态
/// \param player_start 当前玩家位置
/// \param task 策略层生成的候选炸弹任务
/// \param out_task 输出补全后的任务，可能包含 box_pushes
/// \return 成功生成真实可执行任务时返回 true
bool StrategicPlanner::materialize_bomb_task(
    const SokobanLevel& level,
    point player_start,
    const BombTask& task,
    BombTask& out_task,
    bool phase2_specific,
    StaticArray<StrategyClearObligation, 8>* out_obligations,
    uint8_t creator_task_index) {
    this->record_profile_materialize_call();
    SokobanLevel temp = level;
    temp.player_start = player_start;
    if (out_obligations) out_obligations->clear();

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
        next_lvl, real_cost, pushes, false, out_obligations, creator_task_index
    );
    if (!ok) return false;

    out_task = task;
    out_task.box_pushes = pushes;
    this->record_profile_materialize_success();
    return true;
}
