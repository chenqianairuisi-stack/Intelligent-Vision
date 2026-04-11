#include "Exploration.h"
#include <cmath>
#include <cstring>
#include <algorithm>

__attribute__((section(".dtcm_data"))) Exploration patrol_planner;

// ============================================================================
// 代价模型参数
// ============================================================================
static constexpr float COST_PER_GRID = 1.0f;       // 网格移动代价：每前进 1 格记 1
static constexpr uint16_t COST_INFINITY = 65535;   // 不可达代价标记（uint16_t 上限）


// ============================================================================
// 束搜索（Beam Search）工作区
// ============================================================================
constexpr int MAX_OBS_NODES = 80;    // 观测点上限（含箱子/目标生成的可观测邻接点）
constexpr int MAX_BEAM_WIDTH = 64;   // 每层保留的候选分支数量
constexpr int MAX_CANDIDATES = 512;  // 单层扩展候选池容量（超限时进行截断）

// 束搜索节点：携带路径履历，避免额外父指针回溯表
struct BeamNode {
    uint16_t mask;          // 已访问实体 bitmask
    uint16_t cost;          // 累计代价
    uint8_t  k;             // 已执行炸弹宏动作数量
    uint8_t  u;             // 当前所在节点索引（观测点或宏动作节点）
    uint8_t  step_count;    // 当前步数
    uint8_t  seq[32];       // 路径履历：记录每一步到达的节点索引
};

struct BeamWorkspace {
    // 拓扑距离矩阵：dist_matrix[k][u][v] 表示在第 k 个宇宙状态下 u->v 的最短网格距离
    // 说明：维度中 +2 用于容纳 MACRO_NODE 与 START_NODE
    uint16_t dist_matrix[MAX_BOMBS + 1][MAX_OBS_NODES + 2][MAX_OBS_NODES + 2];
    
    // 束搜索候选池
    BeamNode candidates[MAX_CANDIDATES];
    
    // BFS 复用队列（用于最短路预热）
    point bfs_queue[MAP_CELL_COUNT]; 
};
__attribute__((section(".dtcm_data"))) static BeamWorkspace b_ws_patrol;

struct BombPathWorkspace {
    // 宏观状态搜索缓存
    BombMacroNode q[1024];                               // 宏观状态队列
    uint8_t visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];      // 访问位图：bit0~3 对应 4 个推行朝向
    
    // 微观 BFS 缓存（用于“炸弹不动，小车绕位”）
    point micro_q[MAP_CELL_COUNT];
    uint8_t micro_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    point micro_parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t micro_gen; // 代数刷新标记：减少高频 memset 开销
};
__attribute__((section(".dtcm_data"))) static BombPathWorkspace b_ws;


// ============================================================================
// [模块 1] 地图装载与观测点生成
// ============================================================================
void Exploration::load_level(const SokobanLevel& level) {
    this->cached_level = level; 
}

void Exploration::generate_obs_points() {
    // 规则：对每个实体（箱子/目标）尝试 4 邻接观测位，仅保留可站位且不与动态障碍冲突的位置
    obs_points.clear();
    total_entities = 0;

    auto add_obs_points_for_entity = [&](point entity_pos, bool is_box) {
        for (int d = 0; d < 4; ++d) {
            point obs_p = entity_pos + MOVE[d];
            
            if (obs_p.x < 0 || obs_p.x >= MAP_MAX_WIDTH || obs_p.y < 0 || obs_p.y >= MAP_MAX_HEIGHT) continue;
            if (cached_level.map[obs_p.y][obs_p.x] == 1) continue; // 撞墙
            
            // 与箱子/炸弹的占位冲突检测
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
// [模块 2] 多分支巡图规划（Beam Search）
// ============================================================================
__attribute__((section(".ramfunc"))) 
StaticArray<PatrolAction, 32> Exploration::plan_optimal_patrol(
    point start_pos, const StaticArray<BombTask, MAX_BOMBS>& bomb_tasks) 
{
    generate_obs_points();
    int M = obs_points.size();
    if (M == 0 || total_entities == 0) return StaticArray<PatrolAction, 32>();
    
    int B = bomb_tasks.size();
    const int MACRO_NODE = M;      // 宏动作节点：表示“完成一次推炸弹动作后的位置”
    const int START_NODE = M + 1;  // 起点节点：统一纳入同一距离图进行处理

    static SokobanLevel multi_maps[MAX_BOMBS + 1];
    multi_maps[0] = cached_level; 

    // --- 阶段 1：按炸弹执行顺序构造多宇宙地图 ---
    // multi_maps[k] 表示“前 k 颗炸弹已执行”后的地图状态
    for (int k = 0; k < B; ++k) {
        multi_maps[k + 1] = multi_maps[k];
        point t_wall = bomb_tasks[k].target_wall;

        multi_maps[k + 1].bombs[k] = {-1, -1}; // 标记该炸弹已失效
        // 模拟爆炸效果：清空目标墙体周围 3x3 区域
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int ny = t_wall.y + dy, nx = t_wall.x + dx;
                if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 && nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                    multi_maps[k + 1].map[ny][nx] = 0; 
                }
            }
        }
    }

    // --- 阶段 2：预热距离矩阵 ---
    // 在每个宇宙状态 k 下，预计算节点间最短距离用于后续 O(1) 查表
    for (int k = 0; k <= B; ++k) {
        for (int u = 0; u <= START_NODE; ++u) {
            point p_start;
            if (u == MACRO_NODE) {
                if (k == 0) continue; 
                p_start = bomb_tasks[k - 1].target_wall;
            } else if (u == START_NODE) {
                p_start = start_pos;
            } else {
                p_start = obs_points[u].pos;
            }

            for (int v = 0; v <= MACRO_NODE; ++v) {
                point p_end;
                if (v < M) p_end = obs_points[v].pos;
                else {
                    if (k >= B) continue; 
                    p_end = bomb_tasks[k].bomb_start; 
                }

                if (p_start == p_end) b_ws_patrol.dist_matrix[k][u][v] = 0;
                else b_ws_patrol.dist_matrix[k][u][v] = bfs_shortest_path(multi_maps[k], p_start, p_end);
            }
        }
    }

    // --- 阶段 3：束搜索推演 ---
    int req_boxes = std::max(0, cached_level.box_count - 1);       
    int req_targets = std::max(0, cached_level.target_count - 1);  
    int total_steps = req_boxes + req_targets + B; // 总宏动作步数上限

    StaticArray<BeamNode, MAX_BEAM_WIDTH> current_beam;
    current_beam.push_back({0, 0, 0, (uint8_t)START_NODE, 0, {0}});

    for (int step = 0; step < total_steps; ++step) {
        int num_cands = 0;
        // 重置去重表：0xFFFF 视为空槽
        std::memset(TT, 0xFF, sizeof(TT));

        for (int i = 0; i < current_beam.size(); ++i) {
            const auto& node = current_beam[i];

            // 统计当前分支已完成的箱子/目标观测数量
            int boxes_seen = 0, targets_seen = 0;
            for (int e = 0; e < total_entities; ++e) {
                if (node.mask & (1 << e)) {
                    if (e < cached_level.box_count) boxes_seen++;
                    else targets_seen++;
                }
            }

            // 局部扩展器：生成下一状态并执行去重
            auto try_add = [&](uint16_t n_mask, uint8_t n_k, uint8_t n_u, uint16_t add_cost) {
                if (add_cost == COST_INFINITY) return;
                uint16_t n_cost = node.cost + add_cost;

                uint32_t hash = (n_mask ^ (n_k << 12) ^ (n_u << 8)) & (TT_SIZE - 1);

                // 同构状态剪枝：已有更优或等价状态则丢弃
                if (TT[hash].value <= n_cost && TT[hash].sig == n_mask) return;
                
                TT[hash].value = n_cost;
                TT[hash].sig = n_mask;

                // 写入候选池
                auto& cand = b_ws_patrol.candidates[num_cands++];
                cand.mask = n_mask; cand.cost = n_cost; cand.k = n_k; cand.u = n_u;
                cand.step_count = node.step_count + 1;
                std::memcpy(cand.seq, node.seq, node.step_count); // O(1) 拷贝履历
                cand.seq[node.step_count] = n_u;

                // 候选池溢出保护：局部排序保留前 MAX_BEAM_WIDTH 个低代价分支
                if (num_cands >= MAX_CANDIDATES) {
                    std::partial_sort(b_ws_patrol.candidates, b_ws_patrol.candidates + MAX_BEAM_WIDTH, 
                                      b_ws_patrol.candidates + num_cands,[](const BeamNode& a, const BeamNode& b) { return a.cost < b.cost; });
                    num_cands = MAX_BEAM_WIDTH; 
                }
            };

            // 动作 A：执行下一颗炸弹宏动作（u -> MACRO_NODE）
            if (node.k < B) { 
                try_add(node.mask, node.k + 1, MACRO_NODE, b_ws_patrol.dist_matrix[node.k][node.u][MACRO_NODE] * COST_PER_GRID);
            }

            // 动作 B：访问未观测实体对应的观测点
            for (int e = 0; e < total_entities; ++e) {
                if (!(node.mask & (1 << e))) {
                    // 数量约束剪枝：达到要求后不再扩展同类实体
                    if (e < cached_level.box_count && boxes_seen >= req_boxes) continue;
                    if (e >= cached_level.box_count && targets_seen >= req_targets) continue;

                    for (int v = 0; v < M; ++v) {
                        if (obs_points[v].entity_id == e) {
                            try_add(node.mask | (1 << e), node.k, v, b_ws_patrol.dist_matrix[node.k][node.u][v] * COST_PER_GRID);
                        }
                    }
                }
            }
        }

        // 层结算：保留候选池前 keep 个分支进入下一层
        int keep = std::min(num_cands, MAX_BEAM_WIDTH);
        std::partial_sort(b_ws_patrol.candidates, b_ws_patrol.candidates + keep, 
                          b_ws_patrol.candidates + num_cands,[](const BeamNode& a, const BeamNode& b) { return a.cost < b.cost; });

        current_beam.clear();
        for (int i = 0; i < keep; ++i) current_beam.push_back(b_ws_patrol.candidates[i]);
    }

    // --- 阶段 4：解析最优节点履历为动作序列 ---
    StaticArray<PatrolAction, 32> best_sequence;
    if (current_beam.size() > 0) {
        const BeamNode& best_node = current_beam[0]; // 束内按代价升序，首元素即最优
        
        // 直接按 seq 正序还原动作（seq[0] 为 START_NODE）
        int current_k = 0;
        for (int i = 1; i <= best_node.step_count; ++i) { // seq[0] 是 START_NODE，跳过
            uint8_t u = best_node.seq[i];
            PatrolAction act;
            if (u == MACRO_NODE) {
                act.is_bomb_task = true;
                act.bomb = bomb_tasks[current_k++];
            } else {
                act.is_bomb_task = false;
                act.obs = obs_points[u];
            }
            best_sequence.push_back(act);
        }
    }

    return best_sequence;
}


// ============================================================================
// [模块 3] 底层物理与轨迹生成器
// ============================================================================

// BFS：返回 start 到 end 的最短网格距离；不可达时返回 COST_INFINITY
__attribute__((section(".ramfunc")))
uint16_t Exploration::bfs_shortest_path(const SokobanLevel& lvl, point start, point end) {
    if (start == end) return 0.0f;
    static int8_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    std::memset(dist, -1, sizeof(dist));
    
    int head = 0, tail = 0;
    b_ws_patrol.bfs_queue[tail++] = start;
    dist[start.y][start.x] = 0;

    while (head < tail) {
        point curr = b_ws_patrol.bfs_queue[head++];
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
                        b_ws_patrol.bfs_queue[tail++] = np;
                    }
                }
            }
        }
    }
    return COST_INFINITY; 
}

// BFS：生成网格路径坐标序列（仅小车移动，不包含炸弹推动作）
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


// 生成推炸弹完整路径：包含“走位 + 推行 + 再走位”的连续轨迹
__attribute__((section(".ramfunc")))
bool Exploration::get_bomb_push_path(const SokobanLevel& lvl, point player_start, BombTask task, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    out_path.clear();
    std::memset(b_ws.visited, 0, sizeof(b_ws.visited));

    // --- 内部闭包：统一通行判定 ---
    auto is_passable = [&](int x, int y, bool is_bomb_moving) {
        if (x < 0 || x >= MAP_MAX_WIDTH || y < 0 || y >= MAP_MAX_HEIGHT) return false;
        
        // 墙体判定：仅允许炸弹在“最后一步”进入 target_wall
        if (lvl.map[y][x] == 1) {
            if (is_bomb_moving && x == task.target_wall.x && y == task.target_wall.y) {} 
            else return false;
        }
        
        // 箱子绝不可穿透
        for (int i=0; i<lvl.box_count; ++i) 
            if (lvl.boxes[i].x == x && lvl.boxes[i].y == y) return false;
            
        // 其他未失效炸弹不可穿透
        for (int i=0; i<lvl.bomb_count; ++i) {
            if (lvl.bombs[i].x != -1 && lvl.bombs[i].x == x && lvl.bombs[i].y == y) {
                // 排除当前正在推的这颗炸弹本体 (因为炸弹本体由调用者单独判断)
                if (lvl.bombs[i].x == task.bomb_start.x && lvl.bombs[i].y == task.bomb_start.y) continue; 
                return false;
            }
        }
        return true;
    };

    // --- 内部闭包：微观连通性检测（仅判断可达，不输出路径） ---
    auto check_micro_reachable = [&](point start, point end, point obstacle_bomb) {
        if (start == end) return true;
        // 代数刷新：避免每次 BFS 全图 memset
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
                    // 绕位阶段将当前炸弹视作静态障碍
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
    // 阶段 1：宏观状态搜索（只搜索“推行/换面”动作）
    // ==========================================
    int head = 0, tail = 0;
    int target_node_idx = -1;

    // 初始化：找出小车能先到达的推行站位
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

        // 状态变迁 A：沿当前推行方向推进炸弹
        int nbx = curr.bx + MOVE[curr.p_dir].x;
        int nby = curr.by + MOVE[curr.p_dir].y;
        if (is_passable(nbx, nby, true)) {
            if (!(b_ws.visited[nby][nbx] & (1 << curr.p_dir))) {
                b_ws.visited[nby][nbx] |= (1 << curr.p_dir);
                b_ws.q[tail++] = {(int8_t)nbx, (int8_t)nby, curr.p_dir, (uint16_t)curr_idx};
            }
        }

        // 状态变迁 B：炸弹不动，小车绕位到炸弹其他可推面
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

    if (target_node_idx == -1) return false; // 不存在可行推行序列

    // ==========================================
    // 阶段 2：回溯宏观链并展开为实际连续轨迹
    // ==========================================
    StaticArray<BombMacroNode, 256> macro_path;
    int curr_idx = target_node_idx;
    while (curr_idx != 65535) {
        macro_path.push_back(b_ws.q[curr_idx]);
        curr_idx = b_ws.q[curr_idx].parent_idx;
    }
    std::reverse(macro_path.begin(), macro_path.end());

    // --- 内部闭包：生成并附加微观走位路径 ---
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

    // 1) 从起点走到初始推面
    point current_car_pos = player_start;
    point first_push_pos = {
        static_cast<int8_t>(macro_path[0].bx - MOVE[macro_path[0].p_dir].x),
        static_cast<int8_t>(macro_path[0].by - MOVE[macro_path[0].p_dir].y)
    };
    append_micro_path(current_car_pos, first_push_pos, task.bomb_start);
    current_car_pos = first_push_pos;

    // 2) 将宏观链翻译为网格轨迹
    for (int i = 0; i < macro_path.size() - 1; ++i) {
        BombMacroNode c_node = macro_path[i];
        BombMacroNode n_node = macro_path[i+1];
        
        if (c_node.bx != n_node.bx || c_node.by != n_node.by) {
            // 动作 A：推进行为，车辆前进到炸弹旧位置
            point step_into = {c_node.bx, c_node.by};
            out_path.push_back(step_into);
            current_car_pos = step_into;
        } else {
            // 动作 B：换面行为，车辆绕位到下一推面
            point target_face = {
                static_cast<int8_t>(n_node.bx - MOVE[n_node.p_dir].x),
                static_cast<int8_t>(n_node.by - MOVE[n_node.p_dir].y)
            };
            append_micro_path(current_car_pos, target_face, {c_node.bx, c_node.by});
            current_car_pos = target_face;
        }
    }

    // 3) 末步推入目标墙体后，小车落在炸弹旧位置
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
    bool perfect_vision = true;

    // 初始化输出
    for (int i = 0; i < cached_level.box_count; ++i) out_matched_ids[i] = 0;

    // ==========================================
    // 阶段 1：直接匹配（箱子与目标语义均有效且相等）
    // ==========================================
    for (int b = 0; b < cached_level.box_count; ++b) {
        int8_t box_sem = semantic_labels[b];
        if (box_sem == -1) {
            perfect_vision = false; 
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

    // ==========================================
    // 阶段 2：箱子反推目标（箱子有标签，目标盲区）
    // ==========================================
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

    // ==========================================
    // 阶段 3：目标反推箱子（目标有标签，箱子盲区）
    // ==========================================
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

    // 返回值说明：true 表示箱子语义均已观测到；false 表示存在盲区但已尽力完成匹配
    return perfect_vision;
}