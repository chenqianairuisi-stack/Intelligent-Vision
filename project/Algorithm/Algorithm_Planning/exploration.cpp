#include "exploration.h"
#include <cmath>
#include <cstring>
#include <algorithm>

__attribute__((section(".dtcm_data"))) Exploration patrol_planner;

// 物理代价权重 (需要根据实车动态调整)
static constexpr float COST_PER_GRID = 1.0f;       // 走一格的代价
static constexpr float COST_PER_DEGREE = 0.02f;    // 转 1 度的代价 (例如转90度=1.8代价，相当于走1.8格)


// 加载地图并缓存
void Exploration::load_level(const SokobanLevel& level) {
    this->cached_level = level; // 直接拷贝进内部缓存
}


// 核心外部接口
__attribute__((section(".ramfunc"))) 
StaticArray<ObsPoint, 32> Exploration::plan_optimal_patrol(point start_pos, float start_yaw) {
    
    // Step 1: 生成观测点与计算代价矩阵
    generate_obs_points();
    build_cost_matrix();

    // 如果没有合法的观测点，直接返回空序列
    int M = obs_points.size();
    if (M == 0 || total_entities == 0) return StaticArray<ObsPoint, 32>();

    // 准备动态规划(DP)表，记录到达每一种“进度+位置”组合的最低成本，放在 DTCM 防止局部数组爆栈 (256 * 32 * 4 字节 = 32KB)
    static float dp[256][32];          // dp[mask][u] = 访问了 mask 里的实体(例如00001011代表已访问过1,2,4号实体)，且最后停在观测点 u 的最小代价
    static uint8_t parent[256][32];    // parent[mask][u] = 上一个观测点的索引，用于回溯路径
    
    int max_mask = (1 << total_entities);   // 已访问实体状态总数 (实体最多是8，所以 mask 范围是 0 ~ 255)
    for (int i = 0; i < max_mask; ++i) {
        for (int j = 0; j < M; ++j) dp[i][j] = 999999.0f;
    }


    // Step 2: 初始化起点状态
    for (int v = 0; v < M; ++v) {
        uint8_t e_id = obs_points[v].entity_id;

        float dist = bfs_shortest_path(start_pos, obs_points[v].pos); 
        if (dist >= 9999.0f) continue;

        float yaw_diff = std::abs(obs_points[v].target_yaw - start_yaw);
        if (yaw_diff > 180.0f) yaw_diff = 360.0f - yaw_diff;

        // 计算初始代价：距离 + 朝向差
        float cost = dist * COST_PER_GRID + yaw_diff * COST_PER_DEGREE;
        
        // 如果同一个实体有多个观测点，取最小值
        if (cost < dp[1 << e_id][v]) {
            dp[1 << e_id][v] = cost;
        }
    }


    // Step 3: Bitmask DP 状态转移 (Held-Karp GTSP)
    for (int mask = 1; mask < max_mask; ++mask) {
        for (int u = 0; u < M; ++u) {
            if (dp[mask][u] >= 99999.0f) continue; 
            
            for (int v = 0; v < M; ++v) {    // 下一个要去的观测点
                uint8_t next_entity = obs_points[v].entity_id;
                
                // 如果已经看过了，跳过
                if (mask & (1 << next_entity)) continue; 
                
                // 计算新的 mask，表示访问了 v 这个观测点对应的实体
                int next_mask = mask | (1 << next_entity);

                // 计算从 u 到 v 的代价 (物理距离 + 朝向差)
                float cost_uv = cost_matrix[u][v];
                if (cost_uv >= 9999.0f) continue;
                float yaw_diff = std::abs(obs_points[v].target_yaw - obs_points[u].target_yaw);
                if (yaw_diff > 180.0f) yaw_diff = 360.0f - yaw_diff;

                // 计算总代价：之前的代价 + 这一步的代价
                float total_cost = dp[mask][u] + cost_uv * COST_PER_GRID + yaw_diff * COST_PER_DEGREE;
                
                // 更新 DP 表，如果更优则覆盖原有记录，并记录父节点
                if (total_cost < dp[next_mask][v]) {
                    dp[next_mask][v] = total_cost;
                    parent[next_mask][v] = u; 
                }
            }
        }
    }

    // Step 4: 寻找最优终点 (满足至少看了 N-1 个箱子和 N-1 个目标的状态)，并记录最优解的最后一个观测点和 mask
    int req_boxes = cached_level.box_count - 1;       // 只需要看 N-1 个箱子
    int req_targets = cached_level.target_count - 1;  // 只需要看 N-1 个目标
    
    float min_total_cost = 999999.0f;
    int best_last_obs = -1;
    int best_mask = -1;
    
    // 遍历所有的 DP 组合状态
    for (int mask = 1; mask < max_mask; ++mask) {
        
        // 统计当前 mask 里，包含了几个箱子，几个目标点
        int box_seen = 0, target_seen = 0;
        for (int e = 0; e < total_entities; ++e) {
            if (mask & (1 << e)) {
                if (e < cached_level.box_count) box_seen++;
                else target_seen++;
            }
        }
        
        // 只要满足数量，就是合法的终点状态
        if (box_seen >= req_boxes && target_seen >= req_targets) {
            
            for (int u = 0; u < M; ++u) {
                if (dp[mask][u] >= 99999.0f) continue;
                
                float final_cost = dp[mask][u];
                
                // 【末端惩罚】如果最后看的不是箱子，加上惩罚权重
                if (!obs_points[u].is_box) {
                    final_cost += 6.0f * COST_PER_GRID; 
                }
                
                if (final_cost < min_total_cost) {
                    min_total_cost = final_cost;
                    best_last_obs = u;
                    best_mask = mask;
                }
            }
        }
    }

    // Step 5: 回溯路径
    StaticArray<ObsPoint, 32> best_sequence;
    int curr_mask = best_mask;
    int curr_obs = best_last_obs;

    while (curr_mask > 0 && curr_obs != -1) {
        
        best_sequence.push_back(obs_points[curr_obs]);         // 将当前观测点加入最优序列
        int prev_obs = parent[curr_mask][curr_obs];            // 回退到上一个状态
        curr_mask ^= (1 << obs_points[curr_obs].entity_id);    // 更新 mask，去掉当前观测点对应的实体
        curr_obs = prev_obs;                                   // 更新当前观测点索引
    }

    std::reverse(best_sequence.begin(), best_sequence.end());
    return best_sequence;
}



// 语义匹配函数：根据识别到的乱序标签，输出完美匹配的箱子到目标的映射关系
__attribute__((section(".ramfunc"))) 
bool Exploration::match_semantics(const int8_t* semantic_labels, uint8_t* out_matched_ids) const {
    
    // 记录哪些目标点已经被绑定了，哪些箱子已经找到归宿了
    bool target_assigned[SystemConfig::MAX_BOXES] = {false};
    bool box_assigned[SystemConfig::MAX_BOXES] = {false};
    
    // 默认初始化
    for (int i = 0; i < cached_level.box_count; ++i) out_matched_ids[i] = 0;

    bool perfect_vision = true;  // 记录视觉部分是否没有任何丢包

    // 第一阶段：明确的视觉精确匹配 (双向都有明确数字)
    for (int b = 0; b < cached_level.box_count; ++b) {
        int8_t box_semantic = semantic_labels[b];
        if (box_semantic == -1) continue;  // 没去看，或者没认出，留给第二阶段演绎
        
        for (int t = 0; t < cached_level.target_count; ++t) {
            int target_entity_id = cached_level.box_count + t;
            
            // 如果箱子和目标点看到的数字完全一样，确立绑定关系
            if (semantic_labels[target_entity_id] == box_semantic) {
                out_matched_ids[b] = t;        
                box_assigned[b] = true;
                target_assigned[t] = true;
                break;
            }
        }
    }

    // 第二阶段：逻辑演绎与排除法 (处理 N-1 优化)
    int unassigned_target_search_idx = 0;

    for (int b = 0; b < cached_level.box_count; ++b) {
        if (!box_assigned[b]) {  // 这个箱子没有明确的视觉匹配，需要逻辑推演
            // 寻找第一个还没被占用的目标点
            while (unassigned_target_search_idx < cached_level.target_count && 
                   target_assigned[unassigned_target_search_idx]) {
                unassigned_target_search_idx++;
            }

            if (unassigned_target_search_idx < cached_level.target_count) {
                out_matched_ids[b] = unassigned_target_search_idx;
                
                target_assigned[unassigned_target_search_idx] = true;
            } else {
                // 极端异常保护 (理论上永远进不来，因为箱子数 == 目标数)
                out_matched_ids[b] = 0; 
                perfect_vision = false;
            }
        }
    }

    return perfect_vision;
}


// 生成所有观测点
void Exploration::generate_obs_points() {
    obs_points.clear();
    total_entities = 0;

    auto add_obs_points_for_entity = [&](point entity_pos, bool is_box) {

        for (int d = 0; d < 4; ++d) {
            point obs_p = entity_pos + MOVE[d];
            
            // 越界检查
            if (obs_p.x < 0 || obs_p.x >= MAP_MAX_WIDTH || obs_p.y < 0 || obs_p.y >= MAP_MAX_HEIGHT) continue;
            
            // 检测是否是墙，箱子，炸弹
            if (cached_level.map[obs_p.y][obs_p.x] == 1) continue;
            
            bool hit_other = false;
            for(int b=0; b<cached_level.box_count; ++b) {
                if(cached_level.boxes[b] == obs_p) { hit_other = true; break; }
            }

            for(int b=0; b<cached_level.bomb_count; ++b) {
                if(cached_level.bombs[b] == obs_p) { hit_other = true; break; }
            }
            if (hit_other) continue;

            // 这是一个合法的观测点
            obs_points.push_back({obs_p, 270.0f - 90.0f * d, total_entities, is_box});
        }
        total_entities++;
    };

    // 为所有的箱子生成观测点
    for (int i = 0; i < cached_level.box_count; ++i) {
        add_obs_points_for_entity(cached_level.boxes[i], true);
    }
    // 为所有的目标点生成观测点
    for (int i = 0; i < cached_level.target_count; ++i) {
        add_obs_points_for_entity(cached_level.targets[i], false);
    }
}


// 构建全源物理代价矩阵
void Exploration::build_cost_matrix() {
    int M = obs_points.size();
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < M; ++j) {
            if (i == j) cost_matrix[i][j] = 0.0f;
            else {
                cost_matrix[i][j] = bfs_shortest_path(obs_points[i].pos, obs_points[j].pos);
            }
        }
    }
}


// 简单的防撞箱 BFS (巡图时，所有箱子被视为不可跨越的墙)
float Exploration::bfs_shortest_path(point start, point end) {
    if (start == end) return 0.0f;

    static int8_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    std::memset(dist, -1, sizeof(dist));
    
    point q[256];
    int head = 0, tail = 0;
    
    q[tail++] = start;
    dist[start.y][start.x] = 0;

    while (head < tail) {
        point curr = q[head++];
        if (curr == end) return dist[curr.y][curr.x];

        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            
            if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                if (dist[np.y][np.x] == -1 && cached_level.map[np.y][np.x] != 1) {
                    
                    // 检查是否撞到箱子或炸弹
                    bool hit = false;
                    for(int b=0; b<cached_level.box_count; ++b) {
                        if(cached_level.boxes[b] == np) { hit = true; break; }
                    }
                    for(int b=0; b<cached_level.bomb_count; ++b) {
                        if(cached_level.bombs[b] == np) { hit = true; break; }
                    }
                    
                    if (!hit) {
                        dist[np.y][np.x] = dist[curr.y][curr.x] + 1;
                        q[tail++] = np;
                    }
                }
            }
        }
    }
    return 99999.0f; 
}



// 获取两点之间的实际网格行驶路径 (发给底层 PathTracker 循迹用)
__attribute__((section(".ramfunc")))
bool Exploration::get_grid_path(point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    out_path.clear();
    
    // 如果已经在了，直接返回成功
    if (start == end) {
        return true; 
    }

    // 局部 BFS 内存分配 (地图 16x12，占用极小，直接放栈上安全又高效)
    bool visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    point parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    std::memset(visited, 0, sizeof(visited));
    
    point q[256];
    int head = 0, tail = 0;
    
    q[tail++] = start;
    visited[start.y][start.x] = true;
    
    bool found = false;

    // 标准 BFS 洪泛搜索
    while (head < tail) {
        point curr = q[head++];
        
        if (curr == end) {
            found = true;
            break;
        }
        
        for (int i = 0; i < 4; ++i) {
            point np = curr + MOVE[i];
            
            // 越界检查
            if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                // 如果没访问过，且不是墙
                if (!visited[np.y][np.x] && cached_level.map[np.y][np.x] != 1) {
                    
                    // 【物理防穿模】：把箱子和炸弹当做绝对的墙壁绕开
                    bool hit_obstacle = false;
                    for (int b = 0; b < cached_level.box_count; ++b) {
                        if (cached_level.boxes[b] == np) { hit_obstacle = true; break; }
                    }
                    if (!hit_obstacle) {
                        for (int b = 0; b < cached_level.bomb_count; ++b) {
                            if (cached_level.bombs[b] == np) { hit_obstacle = true; break; }
                        }
                    }
                    
                    // 如果是安全的空地，加入队列
                    if (!hit_obstacle) {
                        visited[np.y][np.x] = true;
                        parent[np.y][np.x] = curr; // 记录是从哪个格子走过来的
                        q[tail++] = np;
                    }
                }
            }
        }
    }
    
    // 如果被死角卡住不可达
    if (!found) return false;
    
    // 路径回溯 (Backtracking)
    point curr = end;
    while (!(curr == start)) {
        // 把路径点塞进数组 (不包含起点，但包含终点)
        out_path.push_back(curr);
        curr = parent[curr.y][curr.x];
    }
    
    // 因为是从终点往起点找的，路径是反的，必须要翻转一下！
    std::reverse(out_path.begin(), out_path.end());
    
    return true;
}