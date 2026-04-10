#include "Strategy.h"
#include <cstring>
#include <algorithm>


__attribute__((section(".dtcm_data"))) StrategicPlanner strategic_planner;

constexpr int16_t INF_DIST = 9999;

// DFS 过程缓存：使用 depth 维度隔离每层递归状态
__attribute__((section(".dtcm_data"))) static int16_t dfs_dist_box[MAX_BOMBS + 1][MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
__attribute__((section(".dtcm_data"))) static int16_t dfs_dist_bomb[MAX_BOMBS + 1][MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
__attribute__((section(".dtcm_data"))) static bool dfs_player_vis[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];


// 对外入口：评估并生成炸弹任务序列
__attribute__((section(".ramfunc")))
StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::evaluate_and_assign_bombs(const SokobanLevel& level, point player_start) {
    this->cached_level = level;

    DFSResult best_res;
    best_res.deadlocks_remaining = 9999;
    best_res.net_profit = -999999;

    StaticArray<BombTask, MAX_BOMBS> empty_seq;
    this->dfs_bomb_sequence(level, player_start, empty_seq, 0, 0, best_res);

    // 若全局死锁可清零，则返回序列标记为 essential
    for (int i = 0; i < best_res.tasks.size(); ++i) {
        best_res.tasks[i].is_essential = (best_res.deadlocks_remaining == 0);
        best_res.tasks[i].net_profit = best_res.net_profit;
    }

    return best_res.tasks;
}


// DFS 推演核心
__attribute__((section(".ramfunc")))
void StrategicPlanner::dfs_bomb_sequence(
    const SokobanLevel& current_lvl, point player_start,
    StaticArray<BombTask, MAX_BOMBS> current_seq, int cost_so_far, 
    int depth, DFSResult& best_res) 
{
    // 1) 统计当前状态
    int current_deadlocks = 0;
    int current_distance = 0;

    this->calc_player_reach(current_lvl, player_start, {-1,-1}, {-1,-1}, dfs_player_vis[depth]);

    for (int b = 0; b < current_lvl.box_count; ++b) {
        this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, dfs_dist_box[depth][b]);
        point target = current_lvl.targets[b];
        if (dfs_dist_box[depth][b][target.y][target.x] == INF_DIST) current_deadlocks++;
        else current_distance += dfs_dist_box[depth][b][target.y][target.x];
    }

    int profit = -current_distance - cost_so_far;
    if (current_deadlocks < best_res.deadlocks_remaining || 
       (current_deadlocks == best_res.deadlocks_remaining && profit > best_res.net_profit)) {
        best_res.deadlocks_remaining = current_deadlocks;
        best_res.net_profit = profit;
        best_res.tasks = current_seq;
    }

    if (current_seq.size() == current_lvl.bomb_count || depth >= MAX_BOMBS) return;

    // 2) 计算存活炸弹可达爆破点
    for (int m = 0; m < current_lvl.bomb_count; ++m) {
        if (current_lvl.bombs[m].x != -1) {
            this->fast_push_bfs(current_lvl, current_lvl.bombs[m], player_start, true, dfs_dist_bomb[depth][m]);
        }
    }

    // 建立候选动作队列，避免无脑展开几百个分支
    StaticArray<BombCandidate, 64> candidates;

    // 3) 枚举可爆破墙体 
    for (int m = 0; m < current_lvl.bomb_count; ++m) {
        if (current_lvl.bombs[m].x == -1) continue;

        for (int y = 1; y < MAP_MAX_HEIGHT - 1; ++y) {
            for (int x = 1; x < MAP_MAX_WIDTH - 1; ++x) {
                if (current_lvl.map[y][x] == 1 && dfs_dist_bomb[depth][m][y][x] != INF_DIST) {
                    
                    bool opens_new = false;
                    bool touches_entity = false;
                    int min_shortcut = INF_DIST, max_shortcut = -1;

                    for (int dy = -2; dy <= 2; ++dy) {
                        for (int dx = -2; dx <= 2; ++dx) {
                            int ny = y + dy, nx = x + dx;
                            if (ny >= 0 && ny < MAP_MAX_HEIGHT && nx >= 0 && nx < MAP_MAX_WIDTH) {
                                
                                // (A) 爆炸区是否覆盖实体
                                if (std::abs(dy) <= 1 && std::abs(dx) <= 1) {
                                    if (this->has_entity(current_lvl, nx, ny, m)) touches_entity = true;
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

                    // 剪枝：无连通收益、无实体收益、无明显捷径收益
                    if (!opens_new && !touches_entity && (max_shortcut - min_shortcut <= 4)) continue;

                    // 4) 综合评估打分：优先级 = 实体覆盖 > 新区域 > 捷径提升 - 距离惩罚
                    int score = (max_shortcut - min_shortcut) * 10;
                    if (touches_entity) score += 500;
                    if (opens_new) score += 300;
                    score -= dfs_dist_bomb[depth][m][y][x] * 15; // 严厉惩罚远距离推行

                    candidates.push_back({(uint8_t)m, (int8_t)x, (int8_t)y, score});
                }
            }
        }
    }

    // 对通过你原版筛选出的候选墙壁进行排序
    std::sort(candidates.begin(), candidates.end());

    // 每层只取最靠谱的 Top 4 动作进行递归
    int branch_limit = candidates.size() < 4 ? candidates.size() : 4;

    for (int i = 0; i < branch_limit; ++i) {
        BombCandidate c = candidates[i];
        int m = c.bomb_idx;
        int x = c.x;
        int y = c.y;

        // 构造下一层状态 
        SokobanLevel next_lvl = current_lvl;
        next_lvl.bombs[m] = {-1, -1}; 
        
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int ny = y + dy, nx = x + dx;
                if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 && nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                    next_lvl.map[ny][nx] = 0;
                }
            }
        }

        StaticArray<BombTask, MAX_BOMBS> next_seq = current_seq;
        next_seq.push_back({current_lvl.bombs[m], { (int8_t)x, (int8_t)y }, false, 0});
        
        int execution_cost = dfs_dist_bomb[depth][m][y][x] * 1.5f; 

        this->dfs_bomb_sequence(next_lvl, { (int8_t)x, (int8_t)y }, next_seq, cost_so_far + execution_cost, depth + 1, best_res);
    }
}


// 物理检测辅助
inline bool StrategicPlanner::is_obstacle(const SokobanLevel& lvl, point p, point ignored_obj) const {
    if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
    if (lvl.map[p.y][p.x] == 1) return true;
    
    // 箱子阻挡
    for (int i = 0; i < lvl.box_count; ++i) {
        if (lvl.boxes[i] == p && !(p == ignored_obj)) return true;
    }
    // 炸弹阻挡（被引爆后坐标为 -1, -1，不再阻挡）
    for (int i = 0; i < lvl.bomb_count; ++i) {
        if (lvl.bombs[i].x != -1 && lvl.bombs[i] == p && !(p == ignored_obj)) return true;
    }
    return false;
}

inline bool StrategicPlanner::has_entity(const SokobanLevel& lvl, int x, int y, int ignored_bomb) const {
    for(int i = 0; i < lvl.box_count; ++i) if(lvl.boxes[i].x == x && lvl.boxes[i].y == y) return true;
    for(int i = 0; i < lvl.target_count; ++i) if(lvl.targets[i].x == x && lvl.targets[i].y == y) return true;
    for(int i = 0; i < lvl.bomb_count; ++i) if(i != ignored_bomb && lvl.bombs[i].x == x && lvl.bombs[i].y == y) return true;
    return false;
}

// 洪泛：计算玩家可达区域
__attribute__((section(".ramfunc")))
void StrategicPlanner::calc_player_reach(const SokobanLevel& lvl, point start_pos, point ignored_obj, point extra_obs, bool out_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    std::memset(out_visited, 0, MAP_MAX_HEIGHT * MAP_MAX_WIDTH);
    point q[256]; 
    int head = 0, tail = 0;
    
    q[tail++] = start_pos;
    out_visited[start_pos.y][start_pos.x] = true;
    
    while(head < tail) {
        point curr = q[head++];
        for(int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (np == extra_obs) continue;
            if (!is_obstacle(lvl, np, ignored_obj) && !out_visited[np.y][np.x]) {
                out_visited[np.y][np.x] = true;
                q[tail++] = np;
            }
        }
    }
}

// Fast Push-BFS：推箱/推炸弹代价搜索
__attribute__((section(".ramfunc")))
void StrategicPlanner::fast_push_bfs(const SokobanLevel& lvl, point start_obj, point player_start, bool is_bomb, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    
    struct QNode { int8_t x, y, dir; int16_t cost; };
    static QNode q[1024];
    int head = 0, tail = 0;
    
    static int16_t state_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    for(int y = 0; y < MAP_MAX_HEIGHT; y++) {
        for(int x = 0; x < MAP_MAX_WIDTH; x++) {
            out_dist[y][x] = INF_DIST;
            for(int d = 0; d < 4; d++) state_cost[y][x][d] = INF_DIST;
        }
    }

    bool player_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    calc_player_reach(lvl, player_start, {-1, -1}, {-1, -1}, player_vis);

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
            if (nd == curr.dir) {
                can_push = true;
            } else if ((nd % 2) != (curr.dir % 2)) {
                point back = curr_p - MOVE[nd];                  
                point corner = curr_p - MOVE[curr.dir] - MOVE[nd]; 
                
                // 内角和发力点都为空，可直接转向
                if (!is_obstacle(lvl, back, start_obj) && !is_obstacle(lvl, corner, start_obj)) {
                    can_push = true;
                } else if (!is_obstacle(lvl, back, start_obj)) {
                    // 内角受阻时，回退到可达性检测
                    bool temp_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
                    point player_current_pos = curr_p - MOVE[curr.dir]; 
                    calc_player_reach(lvl, player_current_pos, start_obj, curr_p, temp_vis);
                    if (back.x >= 0 && back.x < MAP_MAX_WIDTH && back.y >= 0 && back.y < MAP_MAX_HEIGHT) {
                        if (temp_vis[back.y][back.x]) can_push = true;
                    }
                }
            } else {
                // 掉头时要求任一侧 U 形三格通道可通行
                point side1_mid = curr_p + MOVE[(curr.dir+1)%4];
                point side1_back = side1_mid - MOVE[curr.dir]; 
                point side1_front = side1_mid + MOVE[curr.dir];
                
                point side2_mid = curr_p + MOVE[(curr.dir+3)%4];
                point side2_back = side2_mid - MOVE[curr.dir]; 
                point side2_front = side2_mid + MOVE[curr.dir];

                bool can_route1 = !is_obstacle(lvl, side1_back, start_obj) && 
                                  !is_obstacle(lvl, side1_mid, start_obj) && 
                                  !is_obstacle(lvl, side1_front, start_obj);
                                  
                bool can_route2 = !is_obstacle(lvl, side2_back, start_obj) && 
                                  !is_obstacle(lvl, side2_mid, start_obj) && 
                                  !is_obstacle(lvl, side2_front, start_obj);
                                  
                if (can_route1 || can_route2) {
                    can_push = true;
                }
            }

            // 发力位不可达，直接剪枝
            if (!can_push) continue; 

            // 可推进时再检测落点是否可用
            if (is_obstacle(lvl, next_p, start_obj)) {
                // 仅炸弹撞墙时记录为可爆破墙体
                if (is_bomb && next_p.x >= 0 && next_p.x < MAP_MAX_WIDTH && next_p.y >= 0 && next_p.y < MAP_MAX_HEIGHT && lvl.map[next_p.y][next_p.x] == 1) {
                    if (curr.cost + 1 < out_dist[next_p.y][next_p.x]) {
                        out_dist[next_p.y][next_p.x] = curr.cost + 1;
                    }
                }
                continue;
            }

            // 前方为空，继续入队
            int16_t ncost = curr.cost + 1;
            if (ncost < state_cost[next_p.y][next_p.x][nd]) {
                state_cost[next_p.y][next_p.x][nd] = ncost;
                if (ncost < out_dist[next_p.y][next_p.x]) out_dist[next_p.y][next_p.x] = ncost;
                q[tail++] = {next_p.x, next_p.y, (int8_t)nd, ncost};
            }
        }
    }
}