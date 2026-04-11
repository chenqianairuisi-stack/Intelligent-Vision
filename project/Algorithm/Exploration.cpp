#include "Exploration.h"
#include <cmath>
#include <cstring>
#include <algorithm>

__attribute__((section(".dtcm_data"))) Exploration patrol_planner;

// ============================================================================
// 物理学惩罚权重配置
// ============================================================================
static constexpr float COST_PER_GRID = 1.0f;       // 直线行驶 1 格的代价
static constexpr uint16_t COST_INFINITY = 65535;   // 代表无解 (uint16_t 的最大值)


// ============================================================================
// 内存池架构：全部数据结构预先分配在 DTCM 中，避免运行时动态内存分配，提升性能和稳定性
// ============================================================================
struct DPWorkspace {
    uint16_t dist_matrix[MAX_BOMBS + 1][MAX_OBS_POINTS + 1][MAX_OBS_POINTS + 1];
    uint16_t dp[MAX_ENTITY_MASK][MAX_BOMBS + 1][MAX_OBS_POINTS + 1];
    uint8_t  parent_u[MAX_ENTITY_MASK][MAX_BOMBS + 1][MAX_OBS_POINTS + 1];
    uint8_t  parent_k[MAX_ENTITY_MASK][MAX_BOMBS + 1][MAX_OBS_POINTS + 1];
    point    bfs_queue[MAP_CELL_COUNT];  // 高频复用队列，取代局部数组
};
__attribute__((section(".dtcm_data"))) static DPWorkspace dp_ram;

struct BombPathWorkspace {
    // 宏观 BFS 使用
    BombMacroNode q[1024];                              // 极小队列
    uint8_t visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];     // 掩码位图：bit0~3 代表 4 个方向是否访问过
    
    // 微观 BFS 使用 (用于小车绕着炸弹走的内部寻路)
    point micro_q[MAP_CELL_COUNT];
    uint8_t micro_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    point micro_parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t micro_gen; // 代数刷新器，避免高频 memset
};
__attribute__((section(".dtcm_data"))) static BombPathWorkspace b_ws;

// ============================================================================
// [模块 1] 地图解析与点位生成
// ============================================================================
void Exploration::load_level(const SokobanLevel& level) {
    this->cached_level = level; 
}

void Exploration::generate_obs_points() {
    obs_points.clear();
    total_entities = 0;

    auto add_obs_points_for_entity = [&](point entity_pos, bool is_box) {
        for (int d = 0; d < 4; ++d) {
            point obs_p = entity_pos + MOVE[d];
            
            if (obs_p.x < 0 || obs_p.x >= MAP_MAX_WIDTH || obs_p.y < 0 || obs_p.y >= MAP_MAX_HEIGHT) continue;
            if (cached_level.map[obs_p.y][obs_p.x] == 1) continue; // 撞墙
            
            // 碰撞箱体和炸弹检测
            bool hit_other = false;
            for(int b=0; b<cached_level.box_count; ++b) {
                if(cached_level.boxes[b] == obs_p) { hit_other = true; break; }
            }
            for(int b=0; b<cached_level.bomb_count; ++b) {
                if(cached_level.bombs[b].x != -1 && cached_level.bombs[b] == obs_p) { hit_other = true; break; }
            }
            if (hit_other) continue;

            // 存入合法观测点
            obs_points.push_back({obs_p, 270.0f - 90.0f * d, total_entities, is_box});
        }
        total_entities++;
    };

    for (int i = 0; i < cached_level.box_count; ++i) add_obs_points_for_entity(cached_level.boxes[i], true);
    for (int i = 0; i < cached_level.target_count; ++i) add_obs_points_for_entity(cached_level.targets[i], false);
}


// ============================================================================
//[模块 2] 多重分支地图巡图引擎
// ============================================================================
__attribute__((section(".ramfunc"))) 
StaticArray<PatrolAction, 32> Exploration::plan_optimal_patrol(
    point start_pos, const StaticArray<BombTask, MAX_BOMBS>& bomb_tasks) 
{
    generate_obs_points();
    int M = obs_points.size();
    if (M == 0 || total_entities == 0) return StaticArray<PatrolAction, 32>();
    
    if (total_entities >= 8) total_entities = 7; // 防止掩码溢出的最后断言

    int B = bomb_tasks.size();
    const int MACRO_NODE = M; // 状态表中的特殊索引，代表“刚执行完炸弹宏动作的状态”

    static SokobanLevel multi_maps[MAX_BOMBS + 1];
    multi_maps[0] = cached_level; 

    // --- 阶段 1：多重宇宙地形推演 ---
    for (int k = 0; k < B; ++k) {
        multi_maps[k + 1] = multi_maps[k];
        point t_wall = bomb_tasks[k].target_wall;

        multi_maps[k + 1].bombs[k] = {-1, -1}; // 物理销毁炸弹
        // 模拟爆炸毁坏墙体 (3x3范围)
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int ny = t_wall.y + dy, nx = t_wall.x + dx;
                if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 && nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                    multi_maps[k + 1].map[ny][nx] = 0; 
                }
            }
        }
    }

    // --- 阶段 2：预热宏动作节点间的距离矩阵 ---
    for (int k = 0; k <= B; ++k) {
        for (int u = 0; u <= M; ++u) {
            point p_start;
            if (u == MACRO_NODE) {
                if (k == 0) continue; // k=0时不存在上一个炸弹的终点
                // 核心解耦：上一个动作如果是推炸弹，我们直接假设起点就在墙壁废墟处
                p_start = bomb_tasks[k - 1].target_wall;
            } else {
                p_start = obs_points[u].pos;
            }

            for (int v = 0; v <= M; ++v) {
                point p_end;
                if (v < M) {
                    p_end = obs_points[v].pos;
                } else {
                    if (k >= B) continue; 
                    // 核心解耦：要去推下一颗炸弹，目标点直接设为炸弹所在坐标
                    p_end = bomb_tasks[k].bomb_start; 
                }

                if (p_start == p_end) dp_ram.dist_matrix[k][u][v] = 0;
                else dp_ram.dist_matrix[k][u][v] = bfs_shortest_path(multi_maps[k], p_start, p_end);
            }
        }
    }

    // --- 阶段 3：全整数状态机初始化 ---
    int max_mask = (1 << total_entities);
    for (int i = 0; i < max_mask; ++i) {
        for (int k = 0; k <= B; ++k) {
            for (int u = 0; u <= M; ++u) dp_ram.dp[i][k][u] = COST_INFINITY;
        }
    }

    // 注入直接去各个观测点的初始代价
    for (int v = 0; v < M; ++v) {
        uint16_t dist = bfs_shortest_path(multi_maps[0], start_pos, obs_points[v].pos);
        if (dist != COST_INFINITY) {
            uint8_t e_id = obs_points[v].entity_id;
            uint16_t cost = dist * COST_PER_GRID;
            if (cost < dp_ram.dp[1 << e_id][0][v]) dp_ram.dp[1 << e_id][0][v] = cost;
        }
    }
    
    // 注入直接去推第一颗炸弹的初始代价
    if (B > 0) {
        uint16_t dist = bfs_shortest_path(multi_maps[0], start_pos, bomb_tasks[0].bomb_start);
        if (dist != COST_INFINITY) {
            dp_ram.dp[0][1][MACRO_NODE] = dist * COST_PER_GRID; 
        }
    }

    // --- 阶段 4：纯距离代价的主循环博弈 ---
    for (int mask = 0; mask < max_mask; ++mask) {
        for (int k = 0; k <= B; ++k) {
            for (int u = 0; u <= M; ++u) {
                if (dp_ram.dp[mask][k][u] == COST_INFINITY) continue;

                // 动作 A：去巡视下一个目标点
                for (int v = 0; v < M; ++v) {
                    uint8_t next_ent = obs_points[v].entity_id;
                    if (mask & (1 << next_ent)) continue; 

                    uint16_t dist = dp_ram.dist_matrix[k][u][v]; 
                    if (dist == COST_INFINITY) continue;
                    
                    int next_mask = mask | (1 << next_ent);
                    uint32_t total = (uint32_t)dp_ram.dp[mask][k][u] + dist * COST_PER_GRID;

                    if (total < dp_ram.dp[next_mask][k][v]) {
                        dp_ram.dp[next_mask][k][v] = total;
                        dp_ram.parent_u[next_mask][k][v] = u;
                        dp_ram.parent_k[next_mask][k][v] = k; 
                    }
                }

                // 动作 B：穿梭至下一宇宙，去触发下一颗炸弹宏动作
                if (k < B) { 
                    uint16_t dist = dp_ram.dist_matrix[k][u][MACRO_NODE]; 
                    if (dist != COST_INFINITY) {
                        uint32_t total = (uint32_t)dp_ram.dp[mask][k][u] + dist * COST_PER_GRID;

                        if (total < dp_ram.dp[mask][k + 1][MACRO_NODE]) {
                            dp_ram.dp[mask][k + 1][MACRO_NODE] = total;
                            dp_ram.parent_u[mask][k + 1][MACRO_NODE] = u;
                            dp_ram.parent_k[mask][k + 1][MACRO_NODE] = k;
                        }
                    }
                }
            }
        }
    }

    // --- 阶段 5：寻找最优回溯 ---
    int req_boxes = cached_level.box_count - 1;       
    int req_targets = cached_level.target_count - 1;  
    
    uint32_t min_cost = 0xFFFFFFFF; // 无穷大
    int best_mask = -1, best_k = -1, best_u = -1;

    for (int mask = 1; mask < max_mask; ++mask) {
        int box_seen = 0, target_seen = 0;
        for (int e = 0; e < total_entities; ++e) {
            if (mask & (1 << e)) {
                if (e < cached_level.box_count) box_seen++;
                else target_seen++;
            }
        }
        
        // 结束态断言：满足 N-1 观测需求，且必须执行完所有炸弹 (k == B)
        if (box_seen >= req_boxes && target_seen >= req_targets) {
            // BUG FIX: 强迫引擎必须停留在 k == B 的状态
            int k = B; 
            for (int u = 0; u <= M; ++u) {
                if (dp_ram.dp[mask][k][u] == COST_INFINITY) continue;
                uint32_t final_cost = dp_ram.dp[mask][k][u];
                
                // 如果最后停在一个观测点，且不是箱子，略微增加惩罚
                if (u != MACRO_NODE && !obs_points[u].is_box) final_cost += COST_PER_GRID * 5; 

                if (final_cost < min_cost) {
                    min_cost = final_cost; best_mask = mask; best_k = k; best_u = u;
                }
            }
        }
    }

    StaticArray<PatrolAction, 32> best_sequence;
    int curr_mask = best_mask, curr_k = best_k, curr_u = best_u;

    while (curr_mask > 0 || curr_k > 0) {
        PatrolAction act;
        if (curr_u == MACRO_NODE) {
            act.is_bomb_task = true;
            act.bomb = bomb_tasks[curr_k - 1]; 
        } else {
            act.is_bomb_task = false;
            act.obs = obs_points[curr_u];
        }
        best_sequence.push_back(act);

        int p_u = dp_ram.parent_u[curr_mask][curr_k][curr_u];
        int p_k = dp_ram.parent_k[curr_mask][curr_k][curr_u];
        
        if (curr_u != MACRO_NODE) curr_mask ^= (1 << obs_points[curr_u].entity_id);
        curr_k = p_k;
        curr_u = p_u;
    }

    std::reverse(best_sequence.begin(), best_sequence.end());
    return best_sequence;
}


// 向下兼容接口封装
__attribute__((section(".ramfunc"))) 
StaticArray<ObsPoint, 32> Exploration::plan_optimal_patrol(point start_pos) {
    StaticArray<BombTask, MAX_BOMBS> empty_bombs; // 传入空炸弹引发引擎降维塌缩
    StaticArray<PatrolAction, 32> mixed_actions = this->plan_optimal_patrol(start_pos, empty_bombs);

    StaticArray<ObsPoint, 32> legacy_obs_sequence;
    for (int i = 0; i < mixed_actions.size(); ++i) {
        legacy_obs_sequence.push_back(mixed_actions[i].obs);
    }
    return legacy_obs_sequence;
}


// ============================================================================
// [模块 3] 底层物理与轨迹生成器
// ============================================================================

// BFS 生成最短路径长度 (仅返回距离，不生成路径)
__attribute__((section(".ramfunc")))
uint16_t Exploration::bfs_shortest_path(const SokobanLevel& lvl, point start, point end) {
    if (start == end) return 0.0f;
    static int8_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    std::memset(dist, -1, sizeof(dist));
    
    int head = 0, tail = 0;
    dp_ram.bfs_queue[tail++] = start;
    dist[start.y][start.x] = 0;

    while (head < tail) {
        point curr = dp_ram.bfs_queue[head++];
        if (curr == end) return dist[curr.y][curr.x];

        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {

                bool is_wall = (lvl.map[np.y][np.x] == 1);

                if (dist[np.y][np.x] == -1 && (!is_wall || np == end)) {
                    bool hit = false;
                    for(int b=0; b<lvl.box_count; ++b) if(lvl.boxes[b] == np) { hit = true; break; }
                    for(int b=0; b<lvl.bomb_count; ++b) {
                        if(lvl.bombs[b].x != -1 && lvl.bombs[b] == np) { hit = true; break; }
                    }
                    if (!hit || np == end) {
                        dist[np.y][np.x] = dist[curr.y][curr.x] + 1;
                        dp_ram.bfs_queue[tail++] = np;
                    }
                }
            }
        }
    }
    return COST_INFINITY; 
}

// BFS 生成最短路径坐标数组 (考虑炸弹推行时的特殊碰撞规则)
__attribute__((section(".ramfunc")))
bool Exploration::get_grid_path(const SokobanLevel& lvl, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    out_path.clear();
    if (start == end) return true; 

    bool visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    point parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    std::memset(visited, 0, sizeof(visited));
    
    point q[MAP_CELL_COUNT];
    int head = 0, tail = 0;
    
    q[tail++] = start;
    visited[start.y][start.x] = true;
    bool found = false;

    while (head < tail) {
        point curr = q[head++];
        if (curr == end) { found = true; break; }
        
        for (int i = 0; i < 4; ++i) {
            point np = curr + MOVE[i];
            
            if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                if (!visited[np.y][np.x] && lvl.map[np.y][np.x] != 1) {
                    bool hit_obstacle = false;
                    for (int b = 0; b < lvl.box_count; ++b) {
                        if (lvl.boxes[b] == np) { hit_obstacle = true; break; }
                    }
                    if (!hit_obstacle) {
                        for (int b = 0; b < lvl.bomb_count; ++b) {
                            // 【物理校准】必须确认炸弹没被引爆 
                            if (lvl.bombs[b].x != -1 && lvl.bombs[b] == np) { hit_obstacle = true; break; }
                        }
                    }
                    if (!hit_obstacle) {
                        visited[np.y][np.x] = true;
                        parent[np.y][np.x] = curr; 
                        q[tail++] = np;
                    }
                }
            }
        }
    }
    
    if (!found) return false;
    
    point curr = end;
    while (!(curr == start)) {
        out_path.push_back(curr);
        curr = parent[curr.y][curr.x];
    }
    std::reverse(out_path.begin(), out_path.end());
    return true;
}


// 生成小车推炸弹的完整连续路线 (从自身位置一直推入墙体)
__attribute__((section(".ramfunc")))
bool Exploration::get_bomb_push_path(const SokobanLevel& lvl, point player_start, BombTask task, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    out_path.clear();
    std::memset(b_ws.visited, 0, sizeof(b_ws.visited));

    // --- 内部闭包：统一障碍物碰撞检测 ---
    auto is_passable = [&](int x, int y, bool is_bomb_moving) {
        if (x < 0 || x >= MAP_MAX_WIDTH || y < 0 || y >= MAP_MAX_HEIGHT) return false;
        
        // 墙壁判定：除非是炸弹准备被推入目标废墟，否则一律不可穿透
        if (lvl.map[y][x] == 1) {
            if (is_bomb_moving && x == task.target_wall.x && y == task.target_wall.y) {} 
            else return false;
        }
        
        // 箱子绝不可穿透
        for (int i=0; i<lvl.box_count; ++i) 
            if (lvl.boxes[i].x == x && lvl.boxes[i].y == y) return false;
            
        // 未爆的其他炸弹不可穿透
        for (int i=0; i<lvl.bomb_count; ++i) {
            if (lvl.bombs[i].x != -1 && lvl.bombs[i].x == x && lvl.bombs[i].y == y) {
                // 排除当前正在推的这颗炸弹本体 (因为炸弹本体由调用者单独判断)
                if (lvl.bombs[i].x == task.bomb_start.x && lvl.bombs[i].y == task.bomb_start.y) continue; 
                return false;
            }
        }
        return true;
    };

    // --- 内部闭包：微观层小车连通性极速检测 (不记录路径，只看能不能绕过去) ---
    auto check_micro_reachable = [&](point start, point end, point obstacle_bomb) {
        if (start == end) return true;
        // O(1) 极速清零：代数刷新
        b_ws.micro_gen++; if (b_ws.micro_gen == 0) { std::memset(b_ws.micro_visited, 0, sizeof(b_ws.micro_visited)); b_ws.micro_gen = 1; }
        
        int h = 0, t = 0;
        b_ws.micro_q[t++] = start;
        b_ws.micro_visited[start.y][start.x] = b_ws.micro_gen;
        
        while (h < t) {
            point curr = b_ws.micro_q[h++];
            if (curr == end) return true;
            for (int d = 0; d < 4; ++d) {
                point np = curr + MOVE[d];
                if (b_ws.micro_visited[np.y][np.x] != b_ws.micro_gen) {
                    // 小车绕路时，当前的炸弹坐标也是一堵实心墙
                    if (is_passable(np.x, np.y, false) && !(np == obstacle_bomb)) {
                        b_ws.micro_visited[np.y][np.x] = b_ws.micro_gen;
                        b_ws.micro_q[t++] = np;
                    }
                }
            }
        }
        return false;
    };

    // ==========================================
    // 阶段 1：宏观图生成与 BFS (仅搜索推演动作)
    // ==========================================
    int head = 0, tail = 0;
    int target_node_idx = -1;

    // 寻找初始能到达的推炸弹面
    for (int d = 0; d < 4; ++d) {
        point push_pos = {
            static_cast<int8_t>(task.bomb_start.x - MOVE[d].x),
            static_cast<int8_t>(task.bomb_start.y - MOVE[d].y)
        };
        if (is_passable(push_pos.x, push_pos.y, false) && check_micro_reachable(player_start, push_pos, task.bomb_start)) {
            b_ws.visited[task.bomb_start.y][task.bomb_start.x] |= (1 << d);
            b_ws.q[tail++] = {task.bomb_start.x, task.bomb_start.y, (uint8_t)d, 65535};
        }
    }

    while (head < tail) {
        int curr_idx = head++;
        BombMacroNode curr = b_ws.q[curr_idx];

        if (curr.bx == task.target_wall.x && curr.by == task.target_wall.y) {
            target_node_idx = curr_idx;
            break;
        }

        point curr_p = {
            static_cast<int8_t>(curr.bx - MOVE[curr.p_dir].x),
            static_cast<int8_t>(curr.by - MOVE[curr.p_dir].y)
        };

        // 【状态变迁 A】小车往前走一步，推了一下炸弹
        int nbx = curr.bx + MOVE[curr.p_dir].x;
        int nby = curr.by + MOVE[curr.p_dir].y;
        if (is_passable(nbx, nby, true)) {
            if (!(b_ws.visited[nby][nbx] & (1 << curr.p_dir))) {
                b_ws.visited[nby][nbx] |= (1 << curr.p_dir);
                b_ws.q[tail++] = {(int8_t)nbx, (int8_t)nby, curr.p_dir, (uint16_t)curr_idx};
            }
        }

        // 【状态变迁 B】炸弹不动，小车绕圈走到炸弹的另一个面上
        for (int d = 0; d < 4; ++d) {
            if (d == curr.p_dir) continue;
            point adj_p = {
                static_cast<int8_t>(curr.bx - MOVE[d].x),
                static_cast<int8_t>(curr.by - MOVE[d].y)
            };
            
            if (is_passable(adj_p.x, adj_p.y, false)) {
                if (!(b_ws.visited[curr.by][curr.bx] & (1 << d))) {
                    if (check_micro_reachable(curr_p, adj_p, {curr.bx, curr.by})) {
                        b_ws.visited[curr.by][curr.bx] |= (1 << d);
                        b_ws.q[tail++] = {curr.bx, curr.by, (uint8_t)d, (uint16_t)curr_idx};
                    }
                }
            }
        }
    }

    if (target_node_idx == -1) return false; // 地形死锁，无法把炸弹推进目标墙壁

    // ==========================================
    // 阶段 2：提取宏观动作链并展开为底层连续轨迹
    // ==========================================
    StaticArray<BombMacroNode, 256> macro_path;
    int curr_idx = target_node_idx;
    while (curr_idx != 65535) {
        macro_path.push_back(b_ws.q[curr_idx]);
        curr_idx = b_ws.q[curr_idx].parent_idx;
    }
    std::reverse(macro_path.begin(), macro_path.end());

    // --- 内部闭包：生成并附加真实的微观行走坐标 ---
    auto append_micro_path = [&](point start, point end, point obstacle_bomb) {
        if (start == end) return;
        b_ws.micro_gen++; if (b_ws.micro_gen == 0) { std::memset(b_ws.micro_visited, 0, sizeof(b_ws.micro_visited)); b_ws.micro_gen = 1; }
        
        int h = 0, t = 0;
        b_ws.micro_q[t++] = start;
        b_ws.micro_visited[start.y][start.x] = b_ws.micro_gen;
        
        while (h < t) {
            point c = b_ws.micro_q[h++];
            if (c == end) break;
            for (int d = 0; d < 4; ++d) {
                point np = c + MOVE[d];
                if (b_ws.micro_visited[np.y][np.x] != b_ws.micro_gen) {
                    if (is_passable(np.x, np.y, false) && !(np == obstacle_bomb)) {
                        b_ws.micro_visited[np.y][np.x] = b_ws.micro_gen;
                        b_ws.micro_parent[np.y][np.x] = c;
                        b_ws.micro_q[t++] = np;
                    }
                }
            }
        }
        
        StaticArray<point, 256> temp;
        point curr_p = end;
        while (!(curr_p == start)) {
            temp.push_back(curr_p);
            curr_p = b_ws.micro_parent[curr_p.y][curr_p.x];
        }
        for (int i = temp.size() - 1; i >= 0; --i) out_path.push_back(temp[i]);
    };

    // 1. 小车从原点走到炸弹的第一起推面
    point current_car_pos = player_start;
    point first_push_pos = {
        static_cast<int8_t>(macro_path[0].bx - MOVE[macro_path[0].p_dir].x),
        static_cast<int8_t>(macro_path[0].by - MOVE[macro_path[0].p_dir].y)
    };
    append_micro_path(current_car_pos, first_push_pos, task.bomb_start);
    current_car_pos = first_push_pos;

    // 2. 翻译宏动作序列为网格轨迹
    for (int i = 0; i < macro_path.size() - 1; ++i) {
        BombMacroNode c_node = macro_path[i];
        BombMacroNode n_node = macro_path[i+1];
        
        if (c_node.bx != n_node.bx || c_node.by != n_node.by) {
            // [动作A：推] 往前走一格，占据炸弹的旧坐标
            point step_into = {c_node.bx, c_node.by};
            out_path.push_back(step_into);
            current_car_pos = step_into;
        } else {
            // [动作B：绕圈] 炸弹坐标没变，小车需要绕路换面
            point target_face = {
                static_cast<int8_t>(n_node.bx - MOVE[n_node.p_dir].x),
                static_cast<int8_t>(n_node.by - MOVE[n_node.p_dir].y)
            };
            append_micro_path(current_car_pos, target_face, {c_node.bx, c_node.by});
            current_car_pos = target_face;
        }
    }

    // 3. 最后一下必然是推入墙壁！将炸弹原来的坐标点作为小车的最后落脚点
    out_path.push_back({
        static_cast<int8_t>(macro_path.back().bx - MOVE[macro_path.back().p_dir].x),
        static_cast<int8_t>(macro_path.back().by - MOVE[macro_path.back().p_dir].y)
    });

    return true;
}


// ============================================================================
// [模块 5] 视觉语义与身份绑定系统
// ============================================================================
__attribute__((section(".ramfunc"))) 
bool Exploration::match_semantics(const int8_t* semantic_labels, uint8_t* out_matched_ids) const {
    bool target_assigned[SystemConfig::MAX_BOXES] = {false};
    bool box_assigned[SystemConfig::MAX_BOXES] = {false};
    
    for (int i = 0; i < cached_level.box_count; ++i) out_matched_ids[i] = 0;
    bool perfect_vision = true;  

    // 阶段 1: 视觉绑定
    for (int b = 0; b < cached_level.box_count; ++b) {  // 遍历观测过的箱子
        int8_t box_semantic = semantic_labels[b];
        if (box_semantic == -1) continue; 
        
        for (int t = 0; t < cached_level.target_count; ++t) {  // 检查观测过的目标中有没有直接匹配的
            int target_entity_id = cached_level.box_count + t;
            if (semantic_labels[target_entity_id] == box_semantic) {
                out_matched_ids[b] = t;        
                box_assigned[b] = true;
                target_assigned[t] = true;
                break;
            }
        }
    }

    // 阶段 2: N-1 残缺绑定推演
    int unassigned_target_search_idx = 0;
    for (int b = 0; b < cached_level.box_count; ++b) {
        // 如果这个箱子还没有匹配成功，尝试给它分配一个还未被占用的目标
        if (!box_assigned[b]) {  
            while (unassigned_target_search_idx < cached_level.target_count && 
                   target_assigned[unassigned_target_search_idx]) {
                unassigned_target_search_idx++;
            }

            if (unassigned_target_search_idx < cached_level.target_count) {
                out_matched_ids[b] = unassigned_target_search_idx;
                target_assigned[unassigned_target_search_idx] = true;
            } else {
                out_matched_ids[b] = 0; 
                perfect_vision = false;
            }
        }
    }

    return perfect_vision;
}