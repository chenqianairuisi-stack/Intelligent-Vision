#include "Sokoban.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

// ============================================================================
// [编译器极致优化指令]
// ============================================================================
// 分支预测优化：告诉编译器条件成立的概率，优化流水线（Pipeline）与指令缓存（I-Cache）
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// ============================================================================
// [全局内存布局与高频数据区]
// ============================================================================
// 全局置换表及求解器实例（放置于 DTCM 保证极致的零等待时钟周期随机访问速度）
__attribute__((section(".dtcm_data"))) Sokoban solver;
__attribute__((section(".dtcm_data"))) TTEntry TT[TT_SIZE];  

// Xorshift32：极速伪随机数生成器（用于初始化 Zobrist 哈希），只需 3 次移位+异或，耗时远低于 rand()
static uint32_t xor_state = 123456789;
static uint32_t xorshift32() {
    xor_state ^= xor_state << 13; 
    xor_state ^= xor_state >> 17; 
    xor_state ^= xor_state << 5;
    return xor_state;
}


// ============================================================================
// 模块 1: 外部数据加载与生命周期管理
// ============================================================================

bool Sokoban::load_from_vision(const SokobanLevel& level) {
    player_start = level.player_start;
    initial_state.player = level.player_start;
    map = level.map;
    
    // 初始化箱子状态与目标状态掩码
    initial_state.num_boxes = level.box_count;
    initial_state.target_mask = (1 << level.box_count) - 1; 
    for (int i = 0; i < level.box_count; ++i) {
        initial_state.box_x[i] = level.boxes[i].x;
        initial_state.box_y[i] = level.boxes[i].y;
    }

    initial_targets.clear();
    for (int i = 0; i < level.target_count; ++i) { initial_targets.push_back(level.targets[i]); }
    
    initial_bombs.clear();
    for (int i = 0; i < level.bomb_count; ++i) { initial_bombs.push_back(level.bombs[i]); }

    // 初始化炸弹与爆炸掩码
    initial_state.num_bombs = 0; 
    initial_state.blown_mask = 0;
    num_bomb_tasks = 0;
    std::memset(wall_clear_mask, 0, sizeof(wall_clear_mask));

    // 触发图层预计算
    init_zobrist();
    precompute_target_distances();
    precompute_deadlocks();

    return true;
}

void Sokoban::bind_semantics(const uint8_t* matched_ids) {
    for (int i = 0; i < initial_state.num_boxes; ++i) {
        initial_state.box_ids[i] = matched_ids[i];
    }
    // 绑定语义后，立即计算初始哈希
    initial_state.hash = compute_hash<GameMode::PHASE2_SPECIFIC>(initial_state);
}

void Sokoban::load_bomb_tasks(const BombTask* tasks, int count) {
    initial_state.num_bombs = initial_bombs.size();
    initial_state.blown_mask = 0;
    num_bomb_tasks = initial_bombs.size(); 
    
    std::memset(wall_clear_mask, 0, sizeof(wall_clear_mask));

    // 解析炸弹任务，生成 O(1) 的物理墙壁坍塌掩码
    for (int b = 0; b < initial_state.num_bombs; ++b) {
        initial_state.bomb_x[b] = initial_bombs[b].x;
        initial_state.bomb_y[b] = initial_bombs[b].y;
        
        bool found = false;
        for (int t = 0; t < count; ++t) {
            if (tasks[t].bomb_start == initial_bombs[b]) {
                bomb_tasks[b] = tasks[t];
                found = true;
                point tw = tasks[t].target_wall;
                // 3x3 墙壁清除掩码注册
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        int ny = tw.y + dy, nx = tw.x + dx;
                        if (ny >= 0 && ny < MAP_MAX_HEIGHT && nx >= 0 && nx < MAP_MAX_WIDTH) {
                            wall_clear_mask[ny][nx] |= (1 << b);
                        }
                    }
                }
                break;
            }
        }
        if (!found) {
            bomb_tasks[b].bomb_start = initial_bombs[b];
            bomb_tasks[b].target_wall = {-1, -1}; 
        }
    }

    // 重构炸弹专属哈希
    for (int b = 0; b < MAX_BOMBS; ++b) {
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
                ZOBRIST_BOMB[b][y][x] = xorshift32();
            }
        }
    }
    for (int i = 0; i < (1 << MAX_BOMBS); ++i) ZOBRIST_BLOWN_MASK[i] = xorshift32();

    precompute_bomb_distances();
    precompute_target_distances();
    precompute_deadlocks();
}


// ============================================================================
// 模块 2: 空间预计算引擎 (离线耗时，换取搜索时的 O(1) 性能)
// ============================================================================

// 预计算 Zobrist 哈希表：为每个可能的元素位置分配一个随机哈希值，保证状态哈希的均匀分布与低碰撞率
void Sokoban::init_zobrist() {
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            ZOBRIST_BOX[y][x] = xorshift32();
            ZOBRIST_PLAYER[y][x] = xorshift32();

            for (int i = 0; i < SystemConfig::MAX_BOXES; ++i) {
                ZOBRIST_SPECIFIC_BOX[i][y][x] = xorshift32();
            }
        }
    }
    for (int i = 0; i < MAX_BOXES; ++i) ZOBRIST_TARGET[i] = xorshift32();
}

// 预计算目标距离场：t_dist[i][y][x] 表示把第 i 个目标点的箱子逆推到 (x,y) 的最少步数，-1 代表死锁
void Sokoban::precompute_target_distances() {
    std::memset(t_dist, -1, sizeof(t_dist));
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1; 

    // 反向 BFS：从目标点拉扯箱子，寻找拉回任意网格的最短推行步数
    for (size_t i = 0; i < initial_targets.size(); ++i) {
        point start = initial_targets[i];
        point q[MAP_CELL_COUNT];
        int head = 0, tail = 0;
        
        q[tail++] = start;
        t_dist[i][start.y][start.x] = 0; 
        
        while (head < tail) {
            point curr = q[head++];
            int dist = t_dist[i][curr.y][curr.x];

            for (int dir = 0; dir < 4; ++dir) {
                point box_prev = curr - MOVE[dir];                 // 箱子上一步的位置
                point player_prev = curr - MOVE[dir] - MOVE[dir];  // 玩家把箱子推过来的站位
                
                if (is_overstep(box_prev) || is_overstep(player_prev)) continue;

                // 物理连通性约束：箱子的来向和玩家的站位都不能是死墙
                if (!is_solid(box_prev, all_blown_mask) && !is_solid(player_prev, all_blown_mask)) {
                    if (t_dist[i][box_prev.y][box_prev.x] == -1) {
                        t_dist[i][box_prev.y][box_prev.x] = dist + 1; 
                        q[tail++] = box_prev;        
                    }
                }
            }
        }
    }
}

// 预计算炸弹距离场：b_dist[b][y][x] 表示把第 b 个炸弹推到 (x,y) 的最少步数，-1 代表不可达
void Sokoban::precompute_bomb_distances() {
    std::memset(b_dist, -1, sizeof(b_dist));
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1;

    for (int b = 0; b < num_bomb_tasks; ++b) {
        if (bomb_tasks[b].target_wall.x == -1) continue;
        point start = bomb_tasks[b].target_wall;

        point q[MAP_CELL_COUNT];
        int head = 0, tail = 0;
        
        q[tail++] = start;
        b_dist[b][start.y][start.x] = 0;

        while (head < tail) {
            point curr = q[head++];
            int dist = b_dist[b][curr.y][curr.x];

            for (int dir = 0; dir < 4; ++dir) {
                point bomb_prev = curr - MOVE[dir];
                point player_prev = curr - MOVE[dir] - MOVE[dir];
                
                if (is_overstep(bomb_prev) || is_overstep(player_prev)) continue;

                if (!is_solid(bomb_prev, all_blown_mask) && !is_solid(player_prev, all_blown_mask)) {
                    if (b_dist[b][bomb_prev.y][bomb_prev.x] == -1) {
                        b_dist[b][bomb_prev.y][bomb_prev.x] = dist + 1;
                        q[tail++] = bomb_prev;
                    }
                }
            }
        }
    }
}

// 预计算死锁格子：任何无法被推向任意目标点的格子，均属于绝对死锁区（is_dead[y][x] = true），推箱过程中必须避免进入
void Sokoban::precompute_deadlocks() {
    std::memset(is_dead, true, sizeof(is_dead)); 
    uint8_t all_blown_mask = (1 << num_bomb_tasks) - 1; 
    
    // 任何无法推向任意目标点的格子，均属于绝对死锁区
    for (int8_t y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int8_t x = 0; x < MAP_MAX_WIDTH; ++x) {
            if (is_solid({x, y}, all_blown_mask)) continue;
            
            for (size_t i = 0; i < initial_targets.size(); ++i) {
                if (t_dist[i][y][x] != -1) {
                    is_dead[y][x] = false;
                    break;
                }
            }
        }
    }
}


// ============================================================================
// 模块 3: 启发式评估器与数学计算
// ============================================================================

// 哈希计算器：根据当前状态的箱子位置、玩家位置、目标点状态和炸弹状态，计算出一个均匀分布的 32 位哈希值，用于置换表查找和状态排重
template <GameMode Mode> 
__attribute__((section(".ramfunc"))) uint32_t Sokoban::compute_hash(const GameState& state) const {
    uint32_t h = 0;
    for (int i = 0; i < state.num_boxes; ++i) {
        if constexpr (Mode == GameMode::PHASE1_ANY) 
            h ^= ZOBRIST_BOX[state.box_y[i]][state.box_x[i]];
        else 
            h ^= ZOBRIST_SPECIFIC_BOX[state.box_ids[i]][state.box_y[i]][state.box_x[i]];
    }
    h ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
    for (size_t i = 0; i < initial_targets.size(); ++i) {
        if (state.target_mask & (1 << i)) h ^= ZOBRIST_TARGET[i];
    }
    return h;
}

// 匈牙利算法/KM算法变体（二分图最小权匹配），用于第一阶段盲推评估
template<size_t N>
__attribute__((section(".ramfunc"))) int Sokoban::min_weight_assignment(int cost[N][N], int n) const {
    if (n == 0) return 0;
    
    int u[N + 1] = {0}, v[N + 1] = {0}, p[N + 1] = {0}, way[N + 1] = {0};                 
    int minv[N + 1];                      
    bool used[N + 1];                     

    for (int i = 1; i <= n; ++i) {
        p[0] = i;  
        int j0 = 0;  
        for (int k = 0; k <= n; ++k) { minv[k] = 9999; used[k] = false; }

        do {
            used[j0] = true;  
            int i0 = p[j0], delta = 9999, j1 = 0;
            
            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    int cur = cost[i0 - 1][j - 1] - u[i0] - v[j];   
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            
            for (int j = 0; j <= n; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; } 
                else { minv[j] -= delta; }  
            }
            j0 = j1;  
        } while (p[j0] != 0);  

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    int total_cost = 0;
    for (int j = 1; j <= n; ++j) {
        int c = cost[p[j] - 1][j - 1];
        if (c >= 9999) return 9999; 
        total_cost += c;
    }
    return total_cost;
}

// IDA* 启发式函数（H值评估）：保证 Admissible (H <= 实际代价)，确保求解最优
template <GameMode Mode>
__attribute__((section(".ramfunc"))) int Sokoban::get_heuristic(const GameState& state) const {
    if (state.num_boxes == 0) return 0; 
    
    int min_p_dist = 9999;
    for (int i = 0; i < state.num_boxes; ++i) {
        int d = std::abs(state.player.x - state.box_x[i]) + std::abs(state.player.y - state.box_y[i]) - 1;
        if (d < 0) d = 0;
        if (d < min_p_dist) min_p_dist = d;
    }
    int p_cost = (min_p_dist != 9999) ? min_p_dist : 0;

    if constexpr (Mode == GameMode::PHASE1_ANY) {
        // 第一阶段盲推：箱子和目标点是多对多关系，利用最小权二分图匹配求下界
        int active_t_idx[MAX_BOXES]; 
        int t_count = 0;
        for (size_t i = 0; i < initial_targets.size(); ++i) {
            if (state.target_mask & (1 << i)) active_t_idx[t_count++] = i;
        }
        if (t_count < state.num_boxes) return 9999; 

        int cost_matrix[MAX_BOXES][MAX_BOXES];
        for (int i = 0; i < state.num_boxes; ++i) {
            for (int j = 0; j < state.num_boxes; ++j) {
                int dist = t_dist[active_t_idx[j]][state.box_y[i]][state.box_x[i]];
                cost_matrix[i][j] = (dist == -1) ? 9999 : dist;
            }
        }

        int min_h = min_weight_assignment<MAX_BOXES>(cost_matrix, state.num_boxes);
        if (min_h >= 9999) return 9999; 
        return min_h + p_cost;
    } 
    else {
        // 第二阶段语义推：一对一约束，加入极其复杂的转移距离推算（带炸弹评估）
        struct EntityTarget { point start, end; int push_cost; };
        EntityTarget entities[MAX_BOXES + MAX_BOMBS];
        int total_entities = 0;

        for (int i = 0; i < state.num_boxes; ++i) {
            int id = state.box_ids[i]; 
            int dist = t_dist[id][state.box_y[i]][state.box_x[i]];
            if (dist == -1) return 9999; // 死锁
            entities[total_entities++] = { {state.box_x[i], state.box_y[i]}, initial_targets[id], dist };
        }
        for (int b = 0; b < state.num_bombs; ++b) {
            if (!(state.blown_mask & (1 << b)) && bomb_tasks[b].target_wall.x != -1) {
                int d = b_dist[b][state.bomb_y[b]][state.bomb_x[b]];
                if (d == -1) return 9999; 
                entities[total_entities++] = { {state.bomb_x[b], state.bomb_y[b]}, bomb_tasks[b].target_wall, d + 2 };
            }
        }

        if (total_entities == 0) return 0;

        int sum_push = 0;
        int base_walk = 0;
        int min_extra = 9999; 

        for (int i = 0; i < total_entities; ++i) {
            sum_push += entities[i].push_cost;
            int p_dist = std::abs(state.player.x - entities[i].start.x) + std::abs(state.player.y - entities[i].start.y) - 1;
            if (p_dist < 0) p_dist = 0;

            int min_other = 9999;
            for (int j = 0; j < total_entities; ++j) {
                if (i == j) continue;
                int w = std::abs(entities[j].end.x - entities[i].start.x) + std::abs(entities[j].end.y - entities[i].start.y) - 1;
                if (w < 0) w = 0;
                if (w < min_other) min_other = w;
            }

            if (total_entities == 1) min_other = 9999; 

            int min_any = (p_dist < min_other) ? p_dist : min_other;
            base_walk += min_any;

            int extra = p_dist - min_any;
            if (extra < min_extra) min_extra = extra;
        }
        return sum_push + base_walk + min_extra;
    }
}


// ============================================================================
// 模块 4: IDA* 核心搜索引擎入口
// ============================================================================

bool Sokoban::solve(GameMode mode) {
    // 强制模板特化分离，避免在极速搜索树中引入运行时分支
    if (mode == GameMode::PHASE1_ANY) 
        return solve_internal<GameMode::PHASE1_ANY>();
    else 
        return solve_internal<GameMode::PHASE2_SPECIFIC>();
}

template <GameMode Mode> bool Sokoban::solve_internal() {
    if (initial_state.num_boxes != initial_targets.size()) return false;
    
    initial_state.hash = compute_hash<Mode>(initial_state);   
    std::memset(TT, 0, sizeof(TT)); // 清空置换表

    int threshold = get_heuristic<Mode>(initial_state);       
    StaticArray<point, MAX_PATH_LENGTH> rev_path;             

    // IDA* 迭代加深核心逻辑
    while (threshold <= MAX_PATH_LENGTH) {
        int res = ida_star_search<Mode>(initial_state, 0, 0, threshold, rev_path, -1);
        
        if (res == -1) {                                      
            rev_path.push_back(initial_state.player);         
            std::reverse(rev_path.begin(), rev_path.end());   
            final_path = rev_path;
            return true;
        }
        if (res >= 9999) break;                               
        threshold = res;                                      
    }
    return false;
}


// ============================================================================
// 模块 5: 最底层高频区 - IDA* 搜索实现体与物理引擎
// ============================================================================

// BFS 代际刷新系统（单片机极速神器：利用 generation 实现 O(1) 状态清理，无需 memset）
static uint16_t bfs_visited_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];  
static uint16_t current_gen = 0;  
static point bfs_q[MAP_CELL_COUNT];
static int8_t bfs_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

template <GameMode Mode> __attribute__((section(".ramfunc")))
int Sokoban::ida_star_search(GameState state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path, int last_entity) {
    if (unlikely(state.num_boxes == 0)) return -1;  // 搜索成功标记

    // 1. TT 表双路哈希查询（排重与剪枝）
    int tt_idx1 = state.hash & (TT_SIZE - 1);  
    int tt_idx2 = (state.hash ^ 0x5BD1E995) & (TT_SIZE - 1); 
    uint16_t sig = (uint16_t)(state.hash >> 16);  
    
    int remaining_threshold = threshold - g;

    if (TT[tt_idx1].sig == sig && TT[tt_idx1].value > remaining_threshold) return g + TT[tt_idx1].value; 
    if (TT[tt_idx2].sig == sig && TT[tt_idx2].value > remaining_threshold) return g + TT[tt_idx2].value; 

    // 2. 启发式裁剪与动态权重控制（Anytime A* 变种，大幅缩减单片机上的树规模）
    int h = get_heuristic<Mode>(state);
    if (unlikely(h >= 9999)) {                
        TT[tt_idx1].sig = sig; TT[tt_idx1].value = 9999; 
        return 9999;
    }

    int active_bombs = 0;
    if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
        for (int b = 0; b < state.num_bombs; ++b) {
            if (!(state.blown_mask & (1 << b)) && bomb_tasks[b].target_wall.x != -1) active_bombs++;
        }
    }
    int active_entities = state.num_boxes + active_bombs;

    int W_num = 10, W_den = 10; 
    if (active_entities >= 8) { W_num = 25; W_den = 10; }      
    else if (active_entities >= 6) { W_num = 20; W_den = 10; } 
    else if (active_entities >= 4) { W_num = 15; W_den = 10; } // W = 1.5
    else { W_num = 10; W_den = 10; }                           

    int f = g + (h * W_num) / W_den;
    if (f > threshold) return f;   

    // 3. O(1) 刷新连通分量 (BFS) 
    current_gen++;  
    if (current_gen == 0) { 
        std::memset(bfs_visited_gen, 0, sizeof(bfs_visited_gen));
        current_gen = 1;
    }

    int head = 0, tail = 0;
    bfs_q[tail++] = state.player;
    bfs_visited_gen[state.player.y][state.player.x] = current_gen;
    bfs_dist[state.player.y][state.player.x] = 0;
    
    point canon_player = state.player;  

    while(head < tail) {
        point curr = bfs_q[head++];
        if (curr.y < canon_player.y || (curr.y == canon_player.y && curr.x < canon_player.x)) {
            canon_player = curr; // 记录联通域的最左上角（Canonical position）
        }
        for(int dir = 0; dir < 4; ++dir) {
            point np = curr + MOVE[dir];
            if(!is_overstep(np)) {
                if(bfs_visited_gen[np.y][np.x] != current_gen && !is_solid(np, state.blown_mask)) {
                    if(find_box_id(state, np) == -1 && get_bomb_id(state, np, Mode) == -1) {   
                        bfs_visited_gen[np.y][np.x] = current_gen;
                        bfs_dist[np.y][np.x] = bfs_dist[curr.y][curr.x] + 1;
                        bfs_q[tail++] = np;
                    }
                }
            }
        }
    }

    // 4. 标准化哈希去重（消除小车在同一联通域中走动造成的等价状态）
    uint32_t canon_hash = state.hash ^ ZOBRIST_PLAYER[state.player.y][state.player.x] ^ ZOBRIST_PLAYER[canon_player.y][canon_player.x];
    for (int i = 0; i < depth; ++i) {
        if (path_hashes[i] == canon_hash) return 9999; // 彻底剪枝环路
    }
    path_hashes[depth] = canon_hash;

    auto is_active_target = [&](point p, int box_idx, int& out_idx) {
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            for (size_t t = 0; t < initial_targets.size(); ++t) {
                if ((state.target_mask & (1 << t)) && initial_targets[t] == p) { out_idx = t; return true; }
            }
            return false;
        } else {
            int b_id = state.box_ids[box_idx]; 
            if ((state.target_mask & (1 << b_id)) && initial_targets[b_id] == p) { out_idx = b_id; return true; }
            return false;
        }
    };

    // 5. 宏动作生成：寻找所有可推动的箱子/炸弹
    struct TinyMove {
        uint8_t entity_idx;   
        uint8_t dir, walk_dist, slide_dist;
        bool triggers_explosion; 
    };
    TinyMove moves[24]; 
    int num_moves = 0;
    int total_entities = state.num_boxes + (Mode == GameMode::PHASE2_SPECIFIC ? state.num_bombs : 0);

    for (uint8_t i = 0; i < total_entities; ++i) {
        bool is_bomb_entity = (i >= state.num_boxes);
        int b_idx = is_bomb_entity ? (i - state.num_boxes) : -1;
        
        if (is_bomb_entity && (state.blown_mask & (1 << b_idx))) continue;
        point pos = is_bomb_entity ? point{state.bomb_x[b_idx], state.bomb_y[b_idx]} : point{state.box_x[i], state.box_y[i]};

        for (uint8_t dir = 0; dir < 4; ++dir) {
            point push_from = pos - MOVE[dir]; 
            
            // 只要推的一侧站立点可达，就能推
            if (!is_overstep(push_from) && bfs_visited_gen[push_from.y][push_from.x] == current_gen) {
                point push_to = pos + MOVE[dir]; 
                if (is_overstep(push_to)) continue;
                
                bool triggers_explosion = false;
                if (is_solid(push_to, state.blown_mask)) {
                    if (is_bomb_entity && push_to == bomb_tasks[b_idx].target_wall) triggers_explosion = true;
                    else continue; 
                }
                if (!triggers_explosion && (find_box_id(state, push_to) != -1 || get_bomb_id(state, push_to, Mode) != -1)) continue;

                // 启发式强剪枝
                if (!triggers_explosion) {
                    if (!is_bomb_entity) {
                        if constexpr (Mode == GameMode::PHASE1_ANY) {
                            int dummy_t;
                            if (is_dead[push_to.y][push_to.x] && !is_active_target(push_to, i, dummy_t)) continue; 
                        } else {
                            if (t_dist[state.box_ids[i]][push_to.y][push_to.x] == -1) continue;
                        }
                    } else {
                        if (bomb_tasks[b_idx].target_wall.x != -1) {
                            if (b_dist[b_idx][push_to.y][push_to.x] == -1) continue;
                        }
                    }
                    
                    // 完美的教科书级 2x2 死锁判定逻辑
                    auto get_next_solid_type = [&](point cp) -> int {
                        if (is_overstep(cp) || is_solid(cp, state.blown_mask)) return 1;
                        int bmb_id = get_bomb_id(state, cp, Mode);
                        if (bmb_id != -1) {
                            if (is_bomb_entity && cp == push_to) return 1; 
                            if (is_bomb_entity && cp == pos) return 0;     
                            return 1; 
                        }
                        int box_id = -1;
                        if (cp == push_to) box_id = (!is_bomb_entity) ? i : -1; 
                        else if (cp == pos) return 0; 
                        else box_id = find_box_id(state, cp);
                        
                        if (box_id != -1) {
                            int out_idx;
                            if (is_active_target(cp, box_id, out_idx)) return 3;
                            return 2;
                        }
                        return 0;
                    };

                    bool is_2x2_deadlock = false;
                    for (int dy = -1; dy <= 0; ++dy) {
                        for (int dx = -1; dx <= 0; ++dx) {
                            int solid_count = 0;
                            bool has_non_target_box = false;
                            for(int cy = 0; cy <= 1; ++cy) {
                                for(int cx = 0; cx <= 1; ++cx) {
                                    point cp = { (int8_t)(push_to.x + dx + cx), (int8_t)(push_to.y + dy + cy) };
                                    int st = get_next_solid_type(cp);
                                    if (st > 0) solid_count++;
                                    if (st == 2) has_non_target_box = true;
                                }
                            }
                            if (solid_count == 4 && has_non_target_box) {
                                is_2x2_deadlock = true; break;
                            }
                        }
                        if (is_2x2_deadlock) break;
                    }
                    if (is_2x2_deadlock) continue; 
                }

                int slide_dist = 0;
                if (!triggers_explosion) {
                    int dummy_t;
                    point final_push_to = push_to;
                    
                    // 完美的教科书级隧道推断（Tunnel Macros）：进入隧道后不要分支搜索，直接滑行到底
                    auto is_tunnel_dynamic = [&](point p, int d) {
                        if (MOVE[d].x == 0) { 
                            bool l = (p.x - 1 < 0) || is_solid({(int8_t)(p.x-1), p.y}, state.blown_mask);
                            bool r = (p.x + 1 > MAP_MAX_WIDTH - 1) || is_solid({(int8_t)(p.x+1), p.y}, state.blown_mask);
                            return l && r;
                        } else { 
                            bool d_wall = (p.y - 1 < 0) || is_solid({p.x, (int8_t)(p.y-1)}, state.blown_mask);
                            bool u_wall = (p.y + 1 > MAP_MAX_HEIGHT - 1) || is_solid({p.x, (int8_t)(p.y+1)}, state.blown_mask);
                            return u_wall && d_wall;
                        }
                    };
                    
                    while (is_tunnel_dynamic(final_push_to, dir)) {
                        if (!is_bomb_entity && is_active_target(final_push_to, i, dummy_t)) break;
                        
                        point next_p = final_push_to + MOVE[dir];
                        if (is_overstep(next_p)) break;

                        if (is_bomb_entity && next_p == bomb_tasks[b_idx].target_wall) break;
                        if (is_solid(next_p, state.blown_mask) || find_box_id(state, next_p) != -1 || get_bomb_id(state, next_p, Mode) != -1) break; 
                        
                        final_push_to = next_p;
                        slide_dist++;
                    }
                }

                if (num_moves < 24) moves[num_moves++] = {
                    i, dir, static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x]), 
                    static_cast<uint8_t>(slide_dist), triggers_explosion
                };
            }
        }
    }

    int min_next_threshold = 9999;

    // ============================================================================
    // O(1) Delta-H 动作重排 (Move Ordering)
    // 根据距离目标点的远近，精确判断每一步是“进攻”还是“被迫让路倒车”
    // ============================================================================
    struct EvalMove { uint8_t move_idx; int16_t sort_key; };
    EvalMove sorted_moves[24]; 

    for (int m = 0; m < num_moves; ++m) {
        TinyMove& mv = moves[m];
        int sort_key = mv.walk_dist + 1; 

        bool is_bomb_entity = (mv.entity_idx >= state.num_boxes);
        int b_idx = is_bomb_entity ? (mv.entity_idx - state.num_boxes) : -1;
        point pos = is_bomb_entity ? point{state.bomb_x[b_idx], state.bomb_y[b_idx]} 
                                   : point{state.box_x[mv.entity_idx], state.box_y[mv.entity_idx]};
        point push_to = pos + MOVE[mv.dir];

        int delta_h = 0;
        if (!mv.triggers_explosion) {
            if (is_bomb_entity) {
                if (bomb_tasks[b_idx].target_wall.x != -1) {
                    delta_h = b_dist[b_idx][push_to.y][push_to.x] - b_dist[b_idx][pos.y][pos.x];
                }
            } else {
                if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                    int b_id = state.box_ids[mv.entity_idx];
                    delta_h = t_dist[b_id][push_to.y][push_to.x] - t_dist[b_id][pos.y][pos.x];
                } else {
                    int min_old = 9999, min_new = 9999;
                    for (size_t t = 0; t < initial_targets.size(); ++t) {
                        if (state.target_mask & (1 << t)) {
                            if (t_dist[t][pos.y][pos.x] < min_old) min_old = t_dist[t][pos.y][pos.x];
                            if (t_dist[t][push_to.y][push_to.x] < min_new) min_new = t_dist[t][push_to.y][push_to.x];
                        }
                    }
                    if (min_old != 9999 && min_new != 9999) delta_h = min_new - min_old;
                }
            }
        }

        // delta_h < 0 表示靠近目标，极大增强优先级；delta_h > 0 表示倒车，极大压后！
        sort_key += delta_h * 100; 

        if (last_entity != -1 && mv.entity_idx == last_entity) sort_key -= 20; 
        if (mv.triggers_explosion) sort_key -= 80; // 炸墙动作极度优先
        sort_key -= mv.slide_dist * 5;             // 鼓励推入通道不回头

        sorted_moves[m] = { static_cast<uint8_t>(m), static_cast<int16_t>(sort_key) };
    }

    // 免函数调用插入排序，完美契合 Cortex-M7 I-Cache 缓存行
    for (int i = 1; i < num_moves; ++i) {
        EvalMove key = sorted_moves[i];
        int j = i - 1;
        while (j >= 0 && sorted_moves[j].sort_key > key.sort_key) {
            sorted_moves[j + 1] = sorted_moves[j];
            j = j - 1;
        }
        sorted_moves[j + 1] = key;
    }

    // 6. 状态迭代与回溯收集
    for (int m = 0; m < num_moves; ++m) {
        TinyMove& mv = moves[sorted_moves[m].move_idx];
        
        bool is_bomb_entity = (mv.entity_idx >= state.num_boxes);
        int b_idx = is_bomb_entity ? (mv.entity_idx - state.num_boxes) : -1;
        
        point pos = is_bomb_entity ? point{state.bomb_x[b_idx], state.bomb_y[b_idx]} 
                                   : point{state.box_x[mv.entity_idx], state.box_y[mv.entity_idx]};
                            
        point push_from = pos - MOVE[mv.dir];
        point push_to = pos + MOVE[mv.dir];

        if (mv.slide_dist > 0) {
            push_to.x += MOVE[mv.dir].x * mv.slide_dist;
            push_to.y += MOVE[mv.dir].y * mv.slide_dist;
        }

        GameState next_state = state;
        next_state.player = push_to - MOVE[mv.dir]; 
        
        // Zobrist 极速异或剥离/覆盖更新
        uint32_t new_hash = state.hash;
        new_hash ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
        new_hash ^= ZOBRIST_PLAYER[next_state.player.y][next_state.player.x];

        if (mv.triggers_explosion) {
            next_state.blown_mask |= (1 << b_idx);                      
            new_hash ^= ZOBRIST_BLOWN_MASK[state.blown_mask];           
            new_hash ^= ZOBRIST_BLOWN_MASK[next_state.blown_mask];      
            new_hash ^= ZOBRIST_BOMB[b_idx][state.bomb_y[b_idx]][state.bomb_x[b_idx]]; 
        } 
        else if (is_bomb_entity) {
            new_hash ^= ZOBRIST_BOMB[b_idx][state.bomb_y[b_idx]][state.bomb_x[b_idx]];
            new_hash ^= ZOBRIST_BOMB[b_idx][push_to.y][push_to.x];
            next_state.bomb_x[b_idx] = push_to.x;
            next_state.bomb_y[b_idx] = push_to.y;
        } 
        else {
            int new_box_count = 0;
            int8_t new_bx[MAX_BOXES], new_by[MAX_BOXES];
            uint8_t new_ids[MAX_BOXES]; 
            uint8_t new_target_mask = state.target_mask;

            for (int i = 0; i < state.num_boxes; ++i) {
                point old_p = {state.box_x[i], state.box_y[i]};
                point p = (i == mv.entity_idx) ? push_to : old_p;

                if constexpr (Mode == GameMode::PHASE1_ANY) {
                    new_hash ^= ZOBRIST_BOX[old_p.y][old_p.x]; 
                    int t_idx = -1;
                    for (size_t t = 0; t < initial_targets.size(); ++t) {
                        if ((new_target_mask & (1 << t)) && initial_targets[t] == p) { t_idx = t; break; }
                    }
                    if (t_idx != -1) {
                        new_target_mask &= ~(1 << t_idx);
                        new_hash ^= ZOBRIST_TARGET[t_idx];  
                    } else {
                        new_bx[new_box_count] = p.x; new_by[new_box_count] = p.y;
                        new_hash ^= ZOBRIST_BOX[p.y][p.x];  
                        new_box_count++;
                    }
                } else { 
                    uint8_t b_id = state.box_ids[i];
                    new_hash ^= ZOBRIST_SPECIFIC_BOX[b_id][old_p.y][old_p.x]; 
                    
                    if (p == initial_targets[b_id]) {
                        new_target_mask &= ~(1 << b_id);
                        new_hash ^= ZOBRIST_TARGET[b_id];  
                    } else {
                        new_bx[new_box_count] = p.x; new_by[new_box_count] = p.y;
                        new_ids[new_box_count] = b_id; 
                        new_hash ^= ZOBRIST_SPECIFIC_BOX[b_id][p.y][p.x];  
                        new_box_count++;
                    }
                }
            }
            next_state.num_boxes = new_box_count;
            next_state.target_mask = new_target_mask;
            for (int i = 0; i < new_box_count; ++i) {
                next_state.box_x[i] = new_bx[i];
                next_state.box_y[i] = new_by[i];
                if constexpr (Mode == GameMode::PHASE2_SPECIFIC) next_state.box_ids[i] = new_ids[i];
            }
        }

        next_state.hash = new_hash;
        int step_cost = mv.walk_dist + 1; 

        // 递归深搜
        int res = ida_star_search<Mode>(next_state, g + step_cost, depth + 1, threshold, path, mv.entity_idx);
        
        if (unlikely(res == -1)) { 
            // 7. 搜索成功后的反向路径重构（由于是单片机，没有堆区内存建完整的树指针，采用就地逆向 BFS 恢复）
            current_gen++;
            if(current_gen == 0) { std::memset(bfs_visited_gen, 0, sizeof(bfs_visited_gen)); current_gen = 1; }
            
            int h2 = 0, t2 = 0;
            static point temp_parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; 
            
            bfs_q[t2++] = state.player;
            bfs_visited_gen[state.player.y][state.player.x] = current_gen;
            
            while(h2 < t2) {
                point curr = bfs_q[h2++];
                if (curr == push_from) break;
                for(int d2 = 0; d2 < 4; ++d2) {
                    point np = curr + MOVE[d2];
                    if(!is_overstep(np)) {
                        if(bfs_visited_gen[np.y][np.x] != current_gen && !is_solid(np, state.blown_mask)) {
                            if(find_box_id(state, np) == -1 && get_bomb_id(state, np, Mode) == -1) {
                                bfs_visited_gen[np.y][np.x] = current_gen;
                                temp_parent[np.y][np.x] = curr;
                                bfs_q[t2++] = np;
                            }
                        }
                    }
                }
            }

            for (int s = mv.slide_dist; s >= 0; --s) {
                point step_p;
                step_p.x = pos.x + MOVE[mv.dir].x * s;
                step_p.y = pos.y + MOVE[mv.dir].y * s;
                path.push_back(step_p);
            }
            
            point walk = push_from;
            while (walk != state.player) {
                path.push_back(walk);
                walk = temp_parent[walk.y][walk.x];
            }
            return -1; 
        }
        
        if (res < min_next_threshold) min_next_threshold = res;
    }

    // 8. TT 置换表回写（始终保持最优的解深度信息）
    int cost_to_go = min_next_threshold - g;
    uint16_t val_to_store = (cost_to_go > 65535) ? 65535 : cost_to_go;

    if (TT[tt_idx1].sig == sig) {
        if (val_to_store > TT[tt_idx1].value) TT[tt_idx1].value = val_to_store;
    } else if (TT[tt_idx2].sig == sig) {
        if (val_to_store > TT[tt_idx2].value) TT[tt_idx2].value = val_to_store;
    } else {
        if (TT[tt_idx1].value <= TT[tt_idx2].value) {
            TT[tt_idx1].sig = sig; 
            TT[tt_idx1].value = val_to_store;
        } else {
            TT[tt_idx2].sig = sig; 
            TT[tt_idx2].value = val_to_store;
        }
    }

    return min_next_threshold;
}


// ============================================================================
// 模块 6: 物理引擎内联区
// ============================================================================

inline bool Sokoban::is_solid(point p, uint8_t blown_mask) const {
    if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
    if (map[p.y][p.x] != 1) return false; 
    // 位运算代替地图写入：若该墙体被分配的炸弹已被引爆，直接视为平地
    return (wall_clear_mask[p.y][p.x] & blown_mask) == 0; 
}

inline bool Sokoban::is_bomb(point p) const {
    for (const auto& b : initial_bombs) if (b == p) return true;
    return false;
}

inline int Sokoban::find_box_id(const GameState& state, point p) const {
    for (int i = 0; i < state.num_boxes; ++i)
        if (state.box_x[i] == p.x && state.box_y[i] == p.y) return i;
    return -1;
}

inline bool Sokoban::is_overstep(point p) const {
    if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
    return false;
}

inline int Sokoban::get_bomb_id(const GameState& state, point p, GameMode Mode) const {
    if (Mode == GameMode::PHASE1_ANY) return -1; 
    for (int b = 0; b < state.num_bombs; ++b) {
        if (!(state.blown_mask & (1 << b)) && state.bomb_x[b] == p.x && state.bomb_y[b] == p.y) return b;
    }
    return -1;
}