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
// 内存池架构
// ============================================================================
struct PlannerWorkspace {
    // 距离矩阵：dist_matrix[k][u][v] (MAX_BOMBS=5, MAX_OBS=40 时，仅需占用约 20KB 的 DTCM 空间)
    // - k: 当前处于第 k 个地图快照
    // - u: 当前所在宏节点（观测点或 MACRO_NODE）
    // - v: 下一步要访问的宏节点（观测点或 MACRO_NODE）
    uint16_t dist_matrix[MAX_BOMBS + 1][MAX_OBS_POINTS + 1][MAX_OBS_POINTS + 1];
    point    bfs_queue[MAP_CELL_COUNT];  // 高频复用队列，取代局部数组
};
__attribute__((section(".dtcm_data"))) static PlannerWorkspace p_ws;

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
            
            // 箱体、炸弹和目标点检测
            bool hit_other = false;
            for(int b=0; b<cached_level.box_count; ++b) {
                if(cached_level.boxes[b] == obs_p) { hit_other = true; break; }
            }
            for(int b=0; b<cached_level.bomb_count; ++b) {
                if(cached_level.bombs[b].x != -1 && cached_level.bombs[b] == obs_p) { hit_other = true; break; }
            }
            for(int b=0; b<cached_level.target_count; ++b) {
                if(cached_level.targets[b] == obs_p) { hit_other = true; break; }
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
// [模块 2] 巡图规划器核心算法：多重分支 3D 深搜 + 微型 L1 Cache 置换表
// ===========================================================================

// 限界上下文记录 (存在于快速栈空间)
struct BoundingContext {
    uint32_t best_cost;
    StaticArray<PatrolAction, 32> best_path;
    StaticArray<PatrolAction, 32> current_path;
    uint32_t ops_limit; // 算力保护锁：最多允许探索多少个节点分支
    uint32_t ops_count;
};

__attribute__((section(".ramfunc"))) 
StaticArray<PatrolAction, 32> Exploration::plan_optimal_patrol(
    point start_pos, const StaticArray<BombTask, MAX_BOMBS>& bomb_tasks) 
{
    // 总体流程：
    // 1) 生成可执行观测的候选点 obs_points；
    // 2) 为每个“已完成前 k 个炸弹任务”的地图快照构建距离矩阵；
    // 3) 在状态 (u, k, mask) 上执行 DFS + 限界剪枝；
    // 4) 使用微型置换表 (micro_tt) 剪掉重复且更差的分支。

    generate_obs_points();
    int M = obs_points.size();
    int B = bomb_tasks.size();

    if (M == 0 || total_entities == 0) return StaticArray<PatrolAction, 32>();
    const int MACRO_NODE = M;  // 额外宏节点：表示“执行炸弹任务”。其索引固定为 M

    // multi_maps[k] 表示已执行完前 k 个炸弹任务后的地图状态。
    static SokobanLevel multi_maps[MAX_BOMBS + 1];
    multi_maps[0] = cached_level; 

    //====================================================================
    // 阶段 1：逐步推演炸弹任务对地形的影响，构建多地图快照
    //====================================================================
    for (int k = 0; k < B; ++k) {
        // 先从上一阶段地图复制，再叠加第 k 个炸弹任务带来的变化。
        multi_maps[k + 1] = multi_maps[k];
        point t_wall = bomb_tasks[k].target_wall;
        // 标记该炸弹已引爆，不再作为障碍。
        multi_maps[k + 1].bombs[k] = {-1, -1}; 
        
        // 引爆后清空目标墙附近 3x3 区域（边界外不处理）。
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int ny = t_wall.y + dy, nx = t_wall.x + dx;
                if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 && nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                    multi_maps[k + 1].map[ny][nx] = 0; 
                }
            }
        }
    }


    //==================================================================== 
    // 阶段 2：预计算距离矩阵 
    //====================================================================
    std::memset(p_ws.dist_matrix, 0xFF, sizeof(p_ws.dist_matrix));

    for (int k = 0; k <= B; ++k) {
        for (int u = 0; u <= M; ++u) {
            // 根据当前状态设置起点坐标：若 u=M 则为 “执行炸弹任务” 的宏节点，起点为当前炸弹目标墙；否则为观测点坐标
            point p_start;
            if (u == MACRO_NODE) {
                if (k == 0) continue;  // k=0 时还没有 “上一个炸弹目标墙” 可作为起点
                p_start = bomb_tasks[k - 1].target_wall;
            } else {
                p_start = obs_points[u].pos;
            }

            // 计算从 p_start 到每个 p_end 的最短路径长度，填充 dist_matrix[k][u][v]
            for (int v = 0; v <= M; ++v) {
                point p_end;
                if (v < M) {
                    p_end = obs_points[v].pos;
                } else {
                    if (k >= B) continue;  // k=B 时已经没有“下一颗炸弹”可执行 
                    p_end = bomb_tasks[k].bomb_start; 
                }

                if (p_start == p_end) p_ws.dist_matrix[k][u][v] = 0;
                else p_ws.dist_matrix[k][u][v] = bfs_shortest_path(multi_maps[k], p_start, p_end);
            }
        }
    }


    //====================================================================
    // 阶段 3：初始化微型置换表 (Transposition Table, TT)
    //====================================================================

    // 每个槽位 32 位：高 16 位为签名，低 16 位为该状态已知最优代价
    // 共 1024 个槽位：总计 4KB 
    uint32_t micro_tt[1024];
    std::memset(micro_tt, 0xFF, sizeof(micro_tt));

    BoundingContext ctx;
    ctx.best_cost = 0xFFFFFFFF;
    ctx.ops_limit = 200000;  // 搜索节点上限，防止极端地图导致耗时失控
    ctx.ops_count = 0;

    // 允许最多各遗漏 1 个箱体/目标实体
    int req_boxes = cached_level.box_count - 1;       
    int req_targets = cached_level.target_count - 1;  


    //====================================================================
    // 阶段 4：DFS 主搜索
    //====================================================================

    // 状态定义：
    // - u: 当前宏节点索引（0..M-1 为观测点，M 为 MACRO_NODE）
    // - k: 已完成炸弹任务数
    // - mask: 已覆盖实体集合（按 entity_id 置位）
    // - current_cost: 从起点到当前状态的累计路径代价
    auto dfs = [&](auto& self, int u, int k, uint32_t mask, uint32_t current_cost) -> void {
        // 剪枝 1：算力保护
        if (ctx.ops_count++ > ctx.ops_limit) return;

        // 剪枝 2：分支限界（当前代价已不可能优于全局最优）
        if (current_cost >= ctx.best_cost) return;

        // 剪枝 3：置换表命中（将 (mask, u, k) 打包并哈希，若命中且历史代价更优，则直接截断）
        uint32_t packed_state = (mask << 9) | ((uint32_t)u << 3) | (uint32_t)k;
        uint32_t hash = packed_state * 2654435761U; 
        uint16_t tt_idx = hash & 1023;       // 槽位索引（1024=2^10）
        uint16_t tt_sig = hash >> 16;        // 短签名，用于降低误命中概率
        
        // 拆包旧记录：高 16 位签名，低 16 位代价
        uint16_t entry_sig  = micro_tt[tt_idx] >> 16;
        uint16_t entry_cost = micro_tt[tt_idx] & 0xFFFF;

        if (entry_sig == tt_sig) {
            if (entry_cost <= current_cost) return;
        }
        
        // 更新当前更优代价，超过 uint16_t 范围时进行饱和截断
        uint16_t clamped_cost = (current_cost >= COST_INFINITY) ? COST_INFINITY - 1 : (uint16_t)current_cost;
        micro_tt[tt_idx] = ((uint32_t)tt_sig << 16) | clamped_cost;

        // 终止判定：统计已覆盖箱体与目标数量
        int box_seen = 0, target_seen = 0;
        for (int e = 0; e < total_entities; ++e) {
            if (mask & (1UL << e)) {
                if (e < cached_level.box_count) box_seen++;
                else target_seen++;
            }
        }

        // 若已覆盖足够实体且炸弹任务也已完成，则尝试更新全局最优解并返回
        if (box_seen >= req_boxes && target_seen >= req_targets && k == B) {
            uint32_t final_cost = current_cost;
            // 收尾偏好：若停在目标观测点，增加轻微惩罚，引导停在更稳定位置
            if (u != MACRO_NODE && !obs_points[u].is_box) final_cost += COST_PER_GRID * 5; 
            if (final_cost < ctx.best_cost) {
                ctx.best_cost = final_cost;
                ctx.best_path = ctx.current_path;
            }
            return;
        }

        // 分支展开：枚举“下一个观测点”与“执行下一炸弹任务”两类动作
        struct Edge { int next_u; int next_k; uint16_t cost; };
        Edge edges[MAX_OBS_POINTS + 1];
        int edge_count = 0;

        for (int v = 0; v < M; ++v) {
            uint8_t next_ent = obs_points[v].entity_id;
            if (!(mask & (1UL << next_ent))) {
                uint16_t dist = p_ws.dist_matrix[k][u][v];
                if (dist != COST_INFINITY) edges[edge_count++] = {v, k, dist};
            }
        }
        // 若仍有炸弹任务可执行，则加入宏节点分支（k -> k+1）
        if (k < B) {
            uint16_t dist = p_ws.dist_matrix[k][u][MACRO_NODE];
            if (dist != COST_INFINITY) edges[edge_count++] = {MACRO_NODE, k + 1, dist};
        }

        // 边按代价升序排序：优先探索近距离分支，可更快得到较好上界
        // 这里使用插入排序，因 edge_count 通常很小，常数开销更低
        for (int i = 1; i < edge_count; ++i) {
            Edge key = edges[i];
            int j = i - 1;
            while (j >= 0 && edges[j].cost > key.cost) {
                edges[j + 1] = edges[j];
                j = j - 1;
            }
            edges[j + 1] = key;
        }

        // 深搜递归：压栈动作 -> 递归 -> 回溯
        for (int i = 0; i < edge_count; ++i) {
            int v = edges[i].next_u;
            int next_k = edges[i].next_k;
            uint16_t cost = edges[i].cost;

            PatrolAction act;
            uint32_t next_mask = mask;

            if (v == MACRO_NODE) {
                act.is_bomb_task = true;
                act.bomb = bomb_tasks[k];
            } else {
                act.is_bomb_task = false;
                act.obs = obs_points[v];
                next_mask |= (1UL << obs_points[v].entity_id);
            }

            ctx.current_path.push_back(act);
            self(self, v, next_k, next_mask, current_cost + cost * COST_PER_GRID);
            ctx.current_path.pop_back();
        }
    };

    //====================================================================
    // 阶段 5：初始化起始分支
    //====================================================================
    
    // 分支 A：从 start_pos 直接前往任一观测点
    for (int v = 0; v < M; ++v) {
        uint16_t dist = bfs_shortest_path(multi_maps[0], start_pos, obs_points[v].pos);
        if (dist != COST_INFINITY) {
            PatrolAction act; act.is_bomb_task = false; act.obs = obs_points[v];
            ctx.current_path.push_back(act);
            dfs(dfs, v, 0, (1UL << obs_points[v].entity_id), dist * COST_PER_GRID);
            ctx.current_path.pop_back();
        }
    }

    // 分支 B：若有炸弹任务，则允许第一步先去推第 0 颗炸弹
    if (B > 0) {
        uint16_t dist = bfs_shortest_path(multi_maps[0], start_pos, bomb_tasks[0].bomb_start);
        if (dist != COST_INFINITY) {
            PatrolAction act; act.is_bomb_task = true; act.bomb = bomb_tasks[0];
            ctx.current_path.push_back(act);
            // 进入 MACRO_NODE 且 k=1，表示第 0 个炸弹任务已执行。
            dfs(dfs, MACRO_NODE, 1, 0, dist * COST_PER_GRID); 
            ctx.current_path.pop_back();
        }
    }

    // 返回全局最优动作序列；若无可行解则为空序列
    return ctx.best_path;
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
    p_ws.bfs_queue[tail++] = start;
    dist[start.y][start.x] = 0;

    while (head < tail) {
        point curr = p_ws.bfs_queue[head++];
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
                        p_ws.bfs_queue[tail++] = np;
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
// [模块 4] 视觉语义与身份绑定系统
// ============================================================================
__attribute__((section(".ramfunc"))) 
bool Exploration::match_semantics(const int8_t* semantic_labels, uint8_t* out_matched_ids) const {
    bool target_assigned[SystemConfig::MAX_BOXES] = {false};
    bool box_assigned[SystemConfig::MAX_BOXES] = {false};

    // 初始化输出
    for (int i = 0; i < cached_level.box_count; ++i) out_matched_ids[i] = 0;

    // 阶段 1：直接匹配（箱子与目标语义均有效且相等）
    for (int b = 0; b < cached_level.box_count; ++b) {
        int8_t box_sem = semantic_labels[b];
        if (box_sem == -1) {
            continue; 
        }
        
        for (int t = 0; t < cached_level.target_count; ++t) {
            if (target_assigned[t]) continue;
            
            int8_t target_sem = semantic_labels[cached_level.box_count + t];
            if (target_sem == box_sem) {
                out_matched_ids[b] = t;        
                box_assigned[b] = true;
                target_assigned[t] = true;
                break;
            }
        }
    }

    // 阶段 2：箱子反推目标（箱子有标签，目标盲区）
    for (int b = 0; b < cached_level.box_count; ++b) {
        if (box_assigned[b]) continue;
        
        int8_t box_sem = semantic_labels[b];
        if (box_sem != -1) {
            // 将该箱子分配给尚未占用的盲区目标
            for (int t = 0; t < cached_level.target_count; ++t) {
                if (!target_assigned[t] && semantic_labels[cached_level.box_count + t] == -1) {
                    out_matched_ids[b] = t;
                    box_assigned[b] = true;
                    target_assigned[t] = true;
                    break;
                }
            }
        }
    }

    // 阶段 3：目标反推箱子（目标有标签，箱子盲区）
    for (int t = 0; t < cached_level.target_count; ++t) {
        if (target_assigned[t]) continue;

        int8_t target_sem = semantic_labels[cached_level.box_count + t];
        if (target_sem != -1) {
            // 将该目标分配给尚未占用的盲区箱子
            for (int b = 0; b < cached_level.box_count; ++b) {
                if (!box_assigned[b] && semantic_labels[b] == -1) {
                    out_matched_ids[b] = t;
                    box_assigned[b] = true;
                    target_assigned[t] = true;
                    break;
                }
            }
        }
    }

    // 阶段 4：极值兜底 (处理都为 -1的数据，即未观测的箱子和目标正好是配对的)
    for (int b = 0; b < cached_level.box_count; ++b) {
        if (!box_assigned[b]) {  

            for (int t = 0; t < cached_level.target_count; ++t) {
                if (!target_assigned[t]) {
                    out_matched_ids[b] = t;
                    box_assigned[b] = true;
                    target_assigned[t] = true;
                    break;
                }
            }
        }
    }

    // 最后验证是否所有箱子都成功匹配了目标，没有则返回 false
    for (int i = 0; i < cached_level.box_count; ++i) {
        if (box_assigned[i] == false || target_assigned[i] == false) {
            return false;
        }
    }

    // 表示箱子语义均已匹配
    return true;
}