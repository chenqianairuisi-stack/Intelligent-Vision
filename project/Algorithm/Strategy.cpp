#include "Strategy.h"
#include <cstring>
#include <algorithm>


// ============================================================================
// [全局实例生成与 DFS 过程缓存区]
// ============================================================================
__attribute__((section(".dtcm_data"))) StrategicPlanner strategic_planner;
constexpr int16_t INF_DIST = 9999;

__attribute__((section(".dtcm_data"))) static int16_t dfs_dist_box[MAX_BOMBS + 1][MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];   // 预计算玩家到每个箱子的距离矩阵，供 DFS 评估使用
__attribute__((section(".dtcm_data"))) static int16_t dfs_dist_bomb[MAX_BOMBS + 1][MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];  // 预计算玩家到每个箱子/炸弹的距离矩阵，供 DFS 评估使用
__attribute__((section(".dtcm_data"))) static bool dfs_player_vis[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];               // 玩家可达性矩阵，供 DFS 评估使用


// ============================================================================
// 模块 1： 对外入口：评估并生成炸弹任务序列
// ============================================================================
template <GameMode Mode> __attribute__((section(".ramfunc")))
StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::evaluate_and_assign_bombs(const SokobanLevel& level) {
    if (level.bomb_count == 0) return StaticArray<BombTask, MAX_BOMBS>(); 

    this->cached_level = level;
    DFSResult best_res; 
    best_res.deadlocks_remaining = 9999; 
    best_res.net_profit = -999999;

    StaticArray<BombTask, MAX_BOMBS> empty_seq;
    this->dfs_bomb_sequence<Mode>(level, level.player_start, empty_seq, 0, 0, best_res);

    for (int i = 0; i < best_res.tasks.size(); ++i) {
        best_res.tasks[i].is_essential = (best_res.deadlocks_remaining == 0);
        best_res.tasks[i].net_profit = best_res.net_profit;
    }
    return best_res.tasks;
}


// ============================================================================
// 模块 2: DFS 推演核心 [递归参数：当前地图状态、玩家位置、已选炸弹序列、当前成本、递归深度、全局最优结果]
// ============================================================================
template <GameMode Mode> __attribute__((section(".ramfunc")))
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
    bool deadlock_vis[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    std::memset(deadlock_vis, 0, sizeof(deadlock_vis));

    this->calc_player_reach(current_lvl, player_start, {-1,-1}, {-1,-1}, dfs_player_vis[depth]);

    // 计算每个箱子到目标的距离，并统计死锁数量
    for (int b = 0; b < current_lvl.box_count; ++b) {
        this->fast_push_bfs(current_lvl, current_lvl.boxes[b], player_start, false, dfs_dist_box[depth][b]);

        if constexpr (Mode == GameMode::PHASE1_ANY) {
            // 第一阶段：盲区评估，只要能走到任意一个目标点就不算死锁
            for (int t = 0; t < current_lvl.target_count; ++t) {
                point target = current_lvl.targets[t];
                if (dfs_dist_box[depth][b][target.y][target.x] == INF_DIST && !deadlock_vis[t][b]) {
                    current_deadlocks++;
                    deadlock_vis[t][b] = true;
                } else {
                    current_distance += dfs_dist_box[depth][b][target.y][target.x];
                }
            }
        } else {
            // 第二阶段：精准评估，必须能走到专属的目标点才行！
            int t_id = current_lvl.box_ids[b]; // 获取它的专属目标
            point target = current_lvl.targets[t_id];
            if (dfs_dist_box[depth][b][target.y][target.x] == INF_DIST) {
                current_deadlocks += 10; // 定向死锁是致命的，加大惩罚
                deadlock_vis[t_id][b] = true;
            } else {
                current_distance += dfs_dist_box[depth][b][target.y][target.x];
            }
        }
    }

    if (current_seq.size() == current_lvl.bomb_count || depth >= MAX_BOMBS) {
        for (int b = 0; b < current_lvl.box_count; ++b) {
            bool is_dead = true;    
            for (int t = 0; t < current_lvl.target_count; ++t) {
                if (!deadlock_vis[t][b]) {
                    is_dead = false;
                    break;
                }
            }
            if (is_dead) return; // 发现死局，直接剪枝
        }
        for (int t = 0; t < current_lvl.target_count; ++t) {
            bool is_dead = true;
            for (int b = 0; b < current_lvl.box_count; ++b) {
                if (!deadlock_vis[t][b]) {
                    is_dead = false;
                    break;
                }
            }
            if (is_dead) return; // 发现死局，直接剪枝
        }
    }

    // 净收益评估：距离越短越好，已选炸弹越多（成本越高）越差
    int profit = -current_distance - cost_so_far;  
    // 更新全局最优结果 [优先级：死锁数量（越少越好）> 净收益（越高越好）]
    if (current_deadlocks < best_res.deadlocks_remaining || 
       (current_deadlocks == best_res.deadlocks_remaining && profit > best_res.net_profit)) {
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

    // 抓出无解目标点
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
        // 仅对未被选中炸弹进行评估（若已经使用会置为 -1）
        if (current_lvl.bombs[m].x != -1) {  
            this->fast_push_bfs(current_lvl, current_lvl.bombs[m], player_start, true, dfs_dist_bomb[depth][m]);
        }
    }

    // 建立候选动作队列，避免无脑展开过多分支
    StaticArray<BombCandidate, 64> candidates;

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
                    int min_shortcut = INF_DIST, max_shortcut = -1;  // 爆炸后玩家到箱子的距离变化范围（评估是否产生有价值的捷径）

                    // 扫描爆炸范围 3x3，评估 (A) 是否覆盖实体、(B) 是否打通新区域、(C) 是否产生明显捷径
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

                    // 剪枝：无连通收益、无实体收益、无明显捷径收益
                    if (!opens_new && !touches_entity && (max_shortcut - min_shortcut <= 4) && min_iso_target_dist > 3 && min_iso_box_dist > 3) continue;

                    // 综合评估打分：优先级 = 实体覆盖 > 新区域 > 捷径提升 - 距离惩罚
                    int score = (max_shortcut - min_shortcut) * 10;
                    if (touches_entity) score += 500;                    
                    if (opens_new) score += 300;
                    score -= dfs_dist_bomb[depth][m][y][x] * 15; // 严厉惩罚远距离推行

                    // 孤岛解救加分：切比雪夫距离 <= 2 意味着爆炸(半径1)将直接接触该实体，或者贴在它脸上！
                    if (min_iso_target_dist <= 2) score += 600;
                    else if (min_iso_target_dist == 3) score += 300;
                    else if (min_iso_target_dist <= 5) score += 100;

                    if (min_iso_box_dist <= 2) score += 600;
                    else if (min_iso_box_dist == 3) score += 300;
                    else if (min_iso_box_dist <= 5) score += 100;

                    candidates.push_back({(uint8_t)m, (int8_t)x, (int8_t)y, score});
                }
            }
        }
    }

    // =====================================================================
    // 5. 对候选动作进行排序并限制分支数量，进入下一层递归
    // =====================================================================

    // 对筛选出的候选墙壁进行排序
    std::sort(candidates.begin(), candidates.end());

    // 每层只取最靠谱的 Top 6 动作进行递归
    int branch_limit = candidates.size() < 6 ? candidates.size() : 6;

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

        this->dfs_bomb_sequence<Mode>(next_lvl, { (int8_t)x, (int8_t)y }, next_seq, cost_so_far + execution_cost, depth + 1, best_res);
    }
}



// =====================================================================
// 模块 3: Fast Push-BFS：推箱/推炸弹代价搜索 [输入：当前地图状态、要推的对象位置、玩家位置、是否为炸弹、输出距离矩阵]
// =====================================================================
__attribute__((section(".ramfunc")))
void StrategicPlanner::fast_push_bfs(const SokobanLevel& lvl, point start_obj, point player_start, bool is_bomb, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    
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

    // 预先计算玩家可达性，剪枝不可达状态
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

            // 同向推进自然可行
            if (nd == curr.dir) {
                can_push = true;
            // 转向时需要额外空间检测：内角和发力点
            } else if ((nd % 2) != (curr.dir % 2)) {
                point back = curr_p - MOVE[nd];  // 发力点坐标
                point corner = curr_p - MOVE[curr.dir] - MOVE[nd];  // 内角坐标
                
                // 内角和发力点都为空，可直接转向
                if (!is_obstacle(lvl, back, start_obj) && !is_obstacle(lvl, corner, start_obj)) {
                    can_push = true;
                } else if (!is_obstacle(lvl, back, start_obj)) {
                    // 内角受阻时，回退到可达性检测
                    point player_current_pos = curr_p - MOVE[curr.dir]; 
                    if (back.x >= 0 && back.x < MAP_MAX_WIDTH && back.y >= 0 && back.y < MAP_MAX_HEIGHT) {
                        if (can_player_reach(lvl, player_current_pos, back, start_obj, curr_p)) {
                            can_push = true;
                        }
                    }
                }
            } else {
                // 横向掉头需要更严格的空间检测：发力点必须可站立
                point push_stand = curr_p - MOVE[nd]; 
                if (is_obstacle(lvl, push_stand, start_obj)) {
                    continue; // 发力点是墙或箱子，绝对不可能掉头
                }

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
                continue;  // 炸弹使用后不再进入状态队列
            }

            // 前方为空，继续入队
            int16_t ncost = curr.cost + 1;
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
// 模块 4: 工具函数：玩家可达性洪泛、物理碰撞检测、实体检测
// ============================================================================

// 具有提前退出 (Early-Exit) 机制的极速探路，专用于判断是否能绕后发力
__attribute__((section(".ramfunc")))
bool StrategicPlanner::can_player_reach(const SokobanLevel& lvl, point start_pos, point target_pos, point ignored_obj, point extra_obs) {
    if (start_pos == target_pos) return true;
    
    static __attribute__((section(".dtcm_bss"))) uint16_t vis_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static uint16_t cur_vis_gen = 0;
    cur_vis_gen++;
    if (cur_vis_gen == 0) { std::memset(vis_gen, 0, sizeof(vis_gen)); cur_vis_gen = 1; }
    
    static __attribute__((section(".dtcm_bss"))) point q[256];
    int head = 0, tail = 0;
    
    q[tail++] = start_pos;
    vis_gen[start_pos.y][start_pos.x] = cur_vis_gen;
    
    while(head < tail) {
        point curr = q[head++];
        for(int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (np == target_pos) return true; // 找到了立马跑路！不扫全图！
            if (np == extra_obs) continue;
            
            if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                if (vis_gen[np.y][np.x] != cur_vis_gen && !is_obstacle(lvl, np, ignored_obj)) {
                    vis_gen[np.y][np.x] = cur_vis_gen;
                    q[tail++] = np;
                }
            }
        }
    }
    return false;
}

// 可达性检测：计算玩家在当前地图状态下的可达位置矩阵（考虑炸弹和箱子阻挡，支持忽略特定对象）
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


// 物理碰撞检测
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

// 实体检测：检测指定坐标是否有箱子/目标/炸弹（可选择忽略某个炸弹）
inline bool StrategicPlanner::has_entity(const SokobanLevel& lvl, int x, int y, int ignored_bomb) const {
    for(int i = 0; i < lvl.box_count; ++i) if(lvl.boxes[i].x == x && lvl.boxes[i].y == y) return true;
    for(int i = 0; i < lvl.target_count; ++i) if(lvl.targets[i].x == x && lvl.targets[i].y == y) return true;
    for(int i = 0; i < lvl.bomb_count; ++i) if(i != ignored_bomb && lvl.bombs[i].x == x && lvl.bombs[i].y == y) return true;
    return false;
}



template StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::evaluate_and_assign_bombs<GameMode::PHASE1_ANY>(const SokobanLevel&);
template StaticArray<BombTask, MAX_BOMBS> StrategicPlanner::evaluate_and_assign_bombs<GameMode::PHASE2_SPECIFIC>(const SokobanLevel&);
template void StrategicPlanner::dfs_bomb_sequence<GameMode::PHASE1_ANY>(const SokobanLevel&, point, StaticArray<BombTask, MAX_BOMBS>, int, int, DFSResult&);
template void StrategicPlanner::dfs_bomb_sequence<GameMode::PHASE2_SPECIFIC>(const SokobanLevel&, point, StaticArray<BombTask, MAX_BOMBS>, int, int, DFSResult&);