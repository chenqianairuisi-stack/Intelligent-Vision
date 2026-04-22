#include "Sokoban.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

// ============================================================================================
// [模块 1] 初始化与预处理 + 对外接口实现
// ============================================================================================

__attribute__((section(".dtcm_data"))) Sokoban solver;
__attribute__((section(".dtcm_data")))TTEntry TT[TT_SIZE];  // 置换表


// 初始化游戏状态，加载地图、箱子、目标点和炸弹等信息
bool Sokoban::load_from_vision(const SokobanLevel& level) {
    // 初始化小车位置
    player_start = level.player_start;
    initial_state.player = level.player_start;

    // 加载静态地图 (墙壁 1，空地 0)
    map = level.map;

    // 加载箱子，初始化游戏状态
    initial_state.num_boxes = level.box_count;
    initial_state.target_mask = (1 << level.box_count) - 1; // 比如3个目标，mask就是 0b111 (即7)
    
    for (int i = 0; i < level.box_count; ++i) {
        initial_state.box_x[i] = level.boxes[i].x;
        initial_state.box_y[i] = level.boxes[i].y;
    }

    // 加载目标点和炸弹
    initial_targets.clear();
    for (int i = 0; i < level.target_count; ++i) { initial_targets.push_back(level.targets[i]);}
    initial_bombs.clear();
    for (int i = 0; i < level.bomb_count; ++i) { initial_bombs.push_back(level.bombs[i]);}

    // 初始化启发式表和哈希
    init_zobrist();
    precompute_target_distances();
    precompute_deadlocks();

    return true;
}

// 绑定箱子-目标对应关系，生成专属死锁表，并更新玩家位置与初始哈希
void Sokoban::bind_semantics(const uint8_t* matched_ids) {
    // 为初始状态的箱子贴上身份标签
    for (int i = 0; i < initial_state.num_boxes; ++i) {
        initial_state.box_ids[i] = matched_ids[i];
    }
    
    // 计算最终的 P2 初始哈希
    initial_state.hash = compute_hash<GameMode::PHASE2_SPECIFIC>(initial_state);
}

// 统一对外接口
bool Sokoban::solve(GameMode mode) {
    if (mode == GameMode::PHASE1_ANY) 
        return solve_internal<GameMode::PHASE1_ANY>();
    else 
        return solve_internal<GameMode::PHASE2_SPECIFIC>();
}


// ============================================================================================
// [模块 2] 核心求解引擎：IDA* + 启发式剪枝 + 置换表 + BFS预处理
// ============================================================================================


// 内部驱动引擎
template <GameMode Mode> bool Sokoban::solve_internal() {
    if (initial_state.num_boxes != initial_targets.size()) return false;
    
    initial_state.hash = compute_hash<Mode>(initial_state);   // 按特定模式计算初始 Hash
    std::memset(TT, 0, sizeof(TT)); 

    int threshold = get_heuristic<Mode>(initial_state);       // IDA* 初始阈值设为启发函数的预估最小步数
    StaticArray<point, MAX_PATH_LENGTH> rev_path;             // 反向路径，IDA*成功时会倒序存储从终点到起点的路径

    // IDA* 主循环：阈值逐渐增大，直到找到解或者超过 MAX_PATH_LENGTH
    while (threshold <= MAX_PATH_LENGTH) {
        int res = ida_star_search<Mode>(initial_state, 0, 0, threshold, rev_path);
        
        if (res == -1) {                                      // 找到解
            rev_path.push_back(initial_state.player);         // 加入起点
            std::reverse(rev_path.begin(), rev_path.end());   // 倒序变为正序路径
            final_path = rev_path;
            return true;
        }
        if (res >= 9999) break;                               // 无解
        threshold = res;                                      // 用新的下界更新阈值
    }
    return false;
}

// 全局变量，避免递归时频繁开辟内存
static uint16_t bfs_visited_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];  // 位图，标记玩家在BFS中访问过哪些格子
static uint16_t current_gen = 0;  // 代数指针：每次 BFS开始时+1，配合 bfs_visited实现 O(1)清空访问标记
static point bfs_q[MAP_CELL_COUNT];
static int8_t bfs_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// DFS函数：IDA* 算法的递归搜索部分 [g: 已走步数, threshold: 当前深度限制阈值] (建议至少分配 32 KB 的栈空间)
template <GameMode Mode> __attribute__((section(".ramfunc")))
int Sokoban::ida_star_search(GameState state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path) {
    // 箱子全消失，返回-1作为成功标志
    if (state.num_boxes == 0) return -1;  

    // 置换表剪枝：如果之前搜索过一个哈希相同的状态，并且当时剩余的容错深度更大，说明当前分支不如之前的分支，剪掉
    int remaining = threshold - g;
    int tt_idx = state.hash & (TT_SIZE - 1);  // 计算置换表索引 (完整哈希的低 16 位)
    uint16_t sig = (uint16_t)(state.hash >> 16);  // 提取特征码 (完整哈希的高 16 位)
    if (TT[tt_idx].sig == sig && TT[tt_idx].value >= remaining) return threshold + 1;  // 特征码匹配且剩余容错更大，说明当前分支不如之前的分支，剪掉


    // 启发式剪枝：如果当前状态的启发值已经超过阈值，说明这个分支不可能成功，返回启发值作为新的下界建议
    int h = get_heuristic<Mode>(state);
    if (h >= 9999) {                // 剪枝：不可达或死锁状态
        TT[tt_idx].sig = sig; TT[tt_idx].value = remaining;
        return 9999;
    }

    // 动态自适应权重 (WA*) ：箱子越多，贪心权重越大，搜索树越窄；箱子越少，越逼近绝对最优
    int W_num = 10, W_den = 10; 
    if (state.num_boxes >= 7) {
        W_num = 25; W_den = 10; // W = 2.5 (极速贪心)
    } else if (state.num_boxes >= 4) {
        W_num = 20; W_den = 10; // W = 2.0 
    } else {
        W_num = 15; W_den = 10; // W = 1.5
    }

    int f = g + (h * W_num) / W_den;
    if (f > threshold) return f;    // 剪枝：超过当前迭代的限制，返回新的阈值建议


    // ---------- 宏操作第一步：BFS搜索玩家能到达的所有空地 ----------
    current_gen++;  // 代数指针+1，等效于瞬间 memset 清空整个数组
    if (current_gen == 0) { 
        std::memset(bfs_visited_gen, 0, sizeof(bfs_visited_gen));
        current_gen = 1;
    }

    int head = 0, tail = 0;
    bfs_q[tail++] = state.player;
    bfs_visited_gen[state.player.y][state.player.x] = current_gen;
    bfs_dist[state.player.y][state.player.x] = 0;
    point canon_player = state.player;  // 规范化玩家位置（连通区内左下角的坐标，用于消除同构状态）

    while(head < tail) {
        point curr = bfs_q[head++];
        // 更新规范化坐标，选出玩家能到达的所有格子中
        if (curr.y < canon_player.y || (curr.y == canon_player.y && curr.x < canon_player.x)) {
            canon_player = curr;
        }
        for(int dir = 0; dir < 4; ++dir) {
            point np = curr + MOVE[dir];
            if(!is_overstep(np)) {
                if(!(bfs_visited_gen[np.y][np.x] == current_gen) && map[np.y][np.x] != 1 && !is_bomb(np)) {
                    if(find_box_id(state, np) == -1) {   
                        bfs_visited_gen[np.y][np.x] = current_gen;
                        bfs_dist[np.y][np.x] = bfs_dist[curr.y][curr.x] + 1;
                        bfs_q[tail++] = np;
                    }
                }
            }
        }
    }

    // 扣除旧玩家哈希，加上规范化位置的哈希，这样只要箱子一样，人处于同一个连通区域，就被认为是同一种状态 [数学原理：$A \oplus B \oplus B = A$]
    uint32_t canon_hash = state.hash ^ ZOBRIST_PLAYER[state.player.y][state.player.x] ^ ZOBRIST_PLAYER[canon_player.y][canon_player.x];

    // 检测路径环（防止人在空转，陷入死循环）
    for (int i = 0; i < depth; ++i) {
        if (path_hashes[i] == canon_hash) return 9999; 
    }
    path_hashes[depth] = canon_hash;

    
    // lambda函数：判断一个格子是否是一个活跃目标点（对于P1来说是任意未消失的目标点，对于P2来说是这个箱子对应的目标点）
    auto is_active_target = [&](point p, int box_idx, int& out_idx) {
        if constexpr (Mode == GameMode::PHASE1_ANY) {
            // P 1: 路过任意一个未消失的目标点都算有效 
            for (size_t t = 0; t < initial_targets.size(); ++t) {
                if ((state.target_mask & (1 << t)) && initial_targets[t] == p) {
                    out_idx = t; return true;
                }
            }
            return false;
        } else {
            // P2：路过别人的目标点就当普通平地，只有专属目标才生效
            int b_id = state.box_ids[box_idx]; 
            if ((state.target_mask & (1 << b_id)) && initial_targets[b_id] == p) {
                out_idx = b_id; return true;
            }
            return false;
        }
    };

    // lambda函数：判断是否为通道（即在某个方向上两侧都是墙），如果是通道则可以连推
    auto is_tunnel = [&](point p, int d) {
        if (MOVE[d].x == 0) {
            bool left_wall = (p.x - 1 < 0) || (map[p.y][p.x - 1] == 1);
            bool right_wall = (p.x + 1 > MAP_MAX_WIDTH - 1) || (map[p.y][p.x + 1] == 1);
            return left_wall && right_wall;
        } else {
            bool down_wall = (p.y - 1 < 0) || (map[p.y - 1][p.x] == 1);
            bool up_wall = (p.y + 1 > MAP_MAX_HEIGHT - 1) || (map[p.y + 1][p.x] == 1);
            return up_wall && down_wall;
        }
    };

    // ---------- 宏操作第二步：寻找有哪些可以执行的推箱子动作 ----------
    struct TinyMove {
        uint8_t box_idx, dir, walk_dist, slide_dist;
    };
    TinyMove moves[24]; 
    int num_moves = 0;

    for (uint8_t i = 0; i < state.num_boxes; ++i) {
        point box_pos = {state.box_x[i], state.box_y[i]};

        for (uint8_t dir = 0; dir < 4; ++dir) {
            // 确保人能站过去
            point push_from = box_pos - MOVE[dir]; // 推箱子时人站的位置
            if (!is_overstep(push_from)) {
                if (bfs_visited_gen[push_from.y][push_from.x] == current_gen) {
                    // 检查推箱子后的位置是否合法
                    point push_to = box_pos + MOVE[dir]; // 箱子要被推到的位置
                    if (is_overstep(push_to)) continue;
                    if (map[push_to.y][push_to.x] == 1 || is_bomb(push_to)) continue;  // 撞墙或撞炸弹
                                        
                    if (find_box_id(state, push_to) == -1) {
                        // 推一个箱子的情况，支持连推：如果推到的位置是个通道，并且后面没有箱子了，可以继续被推
                        int dummy_t;
                        point final_push_to = push_to;
                        int slide_dist = 0;
                        while (is_tunnel(final_push_to, dir) && !is_active_target(final_push_to, i, dummy_t)) {
                            point next_p = final_push_to + MOVE[dir];
                            if (is_overstep(next_p)) break;
                            if (map[next_p.y][next_p.x] == 1 || is_bomb(next_p) || find_box_id(state, next_p) != -1) break; 
                            final_push_to = next_p;
                            slide_dist++;
                        }

                        // 【剪枝】推过去如果是个死角且不是目标点，则不允许推
                        if constexpr (Mode == GameMode::PHASE1_ANY) {
                            if (is_dead[push_to.y][push_to.x] && !is_active_target(push_to, i, dummy_t)) continue; 
                        } else {
                            if (t_dist[state.box_ids[i]][push_to.y][push_to.x] == -1) continue;
                        }
                        
                        if (num_moves < 24) moves[num_moves++] = {
                            static_cast<uint8_t>(i),
                            static_cast<uint8_t>(dir),
                            static_cast<uint8_t>(static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x])),
                            static_cast<uint8_t>(slide_dist)
                        };
                    }
                }
            }
        }
    }

    int min_next_threshold = 9999;

    // ---------- 执行各个动作，递归进行搜索 ----------
    for (int m = 0; m < num_moves; ++m) {
        TinyMove& mv = moves[m];
        point box_pos = {state.box_x[mv.box_idx], state.box_y[mv.box_idx]};
        point push_from = box_pos - MOVE[mv.dir];
        point push_to = box_pos + MOVE[mv.dir];

        // 如果是连推，更新最终箱子位置
        if (mv.slide_dist > 0) {
            push_to.x += MOVE[mv.dir].x * mv.slide_dist;
            push_to.y += MOVE[mv.dir].y * mv.slide_dist;
        }

        GameState next_state = state;
        next_state.player = push_to - MOVE[mv.dir]; 
        
        int new_box_count = 0;
        int8_t new_bx[MAX_BOXES], new_by[MAX_BOXES];
        uint8_t new_ids[MAX_BOXES]; // 缓存新 ID 顺序
        uint8_t new_target_mask = state.target_mask;
        uint32_t new_hash = state.hash;
        
        // 抠掉旧的玩家位置并加上新的玩家位置哈希(原箱子处)
        new_hash ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
        new_hash ^= ZOBRIST_PLAYER[next_state.player.y][next_state.player.x];

        // 高效重组剩余的箱子(若到终点自动剔除)
        for (int i = 0; i < state.num_boxes; ++i) {
            point old_p = {state.box_x[i], state.box_y[i]};
            point p = old_p;

            if (i == mv.box_idx) p = push_to;   // 更新被推箱子的位置

            // 检查此时的箱子是不是踩上了剩余的目标
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
                // P2：精准扣除专属哈希，判定专属洞口
                uint8_t b_id = state.box_ids[i];
                new_hash ^= ZOBRIST_SPECIFIC_BOX[b_id][old_p.y][old_p.x]; 
                
                if (p == initial_targets[b_id]) {
                    new_target_mask &= ~(1 << b_id);
                    new_hash ^= ZOBRIST_TARGET[b_id];  
                } else {
                    new_bx[new_box_count] = p.x; new_by[new_box_count] = p.y;
                    new_ids[new_box_count] = b_id; // 必须继承身份!
                    new_hash ^= ZOBRIST_SPECIFIC_BOX[b_id][p.y][p.x];  
                    new_box_count++;
                }
            }
        }

        next_state.num_boxes = new_box_count;
        next_state.target_mask = new_target_mask;
        next_state.hash = new_hash;
        for (int i = 0; i < new_box_count; ++i) {
            next_state.box_x[i] = new_bx[i];
            next_state.box_y[i] = new_by[i];
            if constexpr (Mode == GameMode::PHASE2_SPECIFIC) next_state.box_ids[i] = new_ids[i];
        }

        int step_cost = mv.walk_dist + 1; // 走到推点距离 + 1下推的动作
        int res = ida_star_search<Mode>(next_state, g + step_cost, depth + 1, threshold, path);
        
        if (res == -1) { 
            // 如果成功找到解，利用当前BFS访问记录，反推出从原点走到推箱子点的精确路径
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
                    if(np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                        if(bfs_visited_gen[np.y][np.x] != current_gen && map[np.y][np.x] != 1 && !is_bomb(np)) {
                            if(find_box_id(state, np) == -1) {
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
                step_p.x = box_pos.x + MOVE[mv.dir].x * s;
                step_p.y = box_pos.y + MOVE[mv.dir].y * s;
                path.push_back(step_p);
            }
            
            point walk = push_from;
            while (walk != state.player) {
                path.push_back(walk);
                walk = temp_parent[walk.y][walk.x];
            }
            return -1; // 继续向上传递成功标志
        }
        if (res < min_next_threshold) min_next_threshold = res;
    }

    // 搜索失败，递归记录当前状态到置换表 TT，防止未来重复搜索浪费时间
    TT[tt_idx].sig = sig; TT[tt_idx].value = remaining;

    return min_next_threshold;
}


// ============================================================================================
// [模块 3] 辅助功能函数：Zobrist 哈希
// ============================================================================================

// 伪随机数生成器 (Xorshift)，用于生成 Zobrist 哈希用的随机数
static uint32_t xor_state = 123456789;
static uint32_t xorshift32() {
    xor_state ^= xor_state << 13; xor_state ^= xor_state >> 17; xor_state ^= xor_state << 5;
    return xor_state;
}

// 初始化 Zobrist 哈希表，给每一个可能的状态分配一个固定的随机数
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

// 计算当前状态的哈希值（用于查表排重）
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
        // 只有当前存活的目标点才计算进哈希
        if (state.target_mask & (1 << i)) h ^= ZOBRIST_TARGET[i];
    }
    return h;
}


// ============================================================================================
// [模块 4] 辅助功能函数：启发式评估与死锁检测
// ============================================================================================

// 预处理：计算地图上每个格子到每个目标点的最短推挤距离（不考虑炸弹和其他箱子），用于启发式评估与死锁检测
void Sokoban::precompute_target_distances() {
    std::memset(t_dist, -1, sizeof(t_dist));
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
                point box_prev = curr - MOVE[dir];
                point player_prev = curr - MOVE[dir] - MOVE[dir]; 
                
                if (is_overstep(box_prev)) continue;
                if (is_overstep(player_prev)) continue;

                // 要能推箱子到 curr，必须保证箱子和玩家的前一个位置都不是墙或炸弹
                if (map[box_prev.y][box_prev.x] != 1 && !is_bomb(box_prev) &&
                    map[player_prev.y][player_prev.x] != 1 && !is_bomb(player_prev)) {
                    
                    if (t_dist[i][box_prev.y][box_prev.x] == -1) {
                        t_dist[i][box_prev.y][box_prev.x] = dist + 1; // 距离现在代表【最少推挤次数】
                        q[tail++] = box_prev;        
                    }
                }
            }
        }
    }
}

// 预处理：死锁检测 (根据预处理的距离表直接生成，注意要在预处理目标距离后调用)
void Sokoban::precompute_deadlocks() {
    std::memset(is_dead, true, sizeof(is_dead)); 
    
    for (int8_t y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int8_t x = 0; x < MAP_MAX_WIDTH; ++x) {
            if (map[y][x] == 1 || is_bomb({x, y})) continue;
            
            // 只要这个格子能把箱子合法反向拉到任意一个目标点，它就是活路
            for (size_t i = 0; i < initial_targets.size(); ++i) {
                if (t_dist[i][y][x] != -1) {
                    is_dead[y][x] = false;
                    break;
                }
            }
        }
    }
}

// 匈牙利算法求解最小权匹配（用于 P1 阶段的启发式评估，N<=10时效率极高）
template<size_t N>
__attribute__((section(".ramfunc"))) int Sokoban::min_weight_assignment(int cost[N][N], int n) const {
    if (n == 0) return 0;
    
    //匹配原则： 箱子只愿意去 Gap = cost[i0 - 1][j - 1] - u[i0] - v[j] = 0 的目标点
    // - cost[i][j] 是箱子 i 到目标点 j 的原始距离（预处理的推挤距离）
    // - u[i] 是箱子 i 的补偿值，v[j] 是目标点 j 的补偿值，初始都为 0，随着算法进行会动态调整，满足 u[i] + v[j] <= cost[i][j]

    int u[N + 1] = {0}, v[N + 1] = {0};   // 箱子和目标的平衡补偿值
    int p[N + 1] = {0};                   // 匹配关系：表示第 j 个目标点现在被第 i 个箱子占着
    int way[N + 1] = {0};                 // 记录增广路径的前驱节点，用于回溯更新
    int minv[N + 1];                      // 辅助数组：存储当前目标点到已考虑箱子的最小距离，用于优化寻找增广路的速度
    bool used[N + 1];                     // 辅助数组：标记目标点是否已经被加入到增广路径的考虑范围内（冲突区）

    // 尝试为第 i 个箱子寻找一个可匹配的目标点，逐步构建完美匹配
    for (int i = 1; i <= n; ++i) {
        p[0] = i;  // p[0] 是个虚拟节点，先把当前要安排的箱子 i 放在这里
        int j0 = 0;  // j0 代表当前正在处理的目标点（0号是虚拟的起点）
        
        for (int k = 0; k <= n; ++k) { minv[k] = 9999; used[k] = false; }

        do {
            used[j0] = true;  // 标记当前目标点被拉入 “冲突区”
            int i0 = p[j0], delta = 9999, j1 = 0;
            
            // 遍历所有未考虑的目标点 j，计算把箱子 i0 匹配到 j 上的实际成本（扣除补偿值），更新最小备选代价和增广路径
            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    int cur = cost[i0 - 1][j - 1] - u[i0] - v[j];   // 计算当前箱子 i0 到目标点 j 的 “实际成本”
                    if (cur < minv[j]) { 
                        minv[j] = cur;    // 更新目标点 j 的最小备选代价（距离 Gap 为 0 还有多少）
                        way[j] = j0;      // 记住是从 j0 这里的箱子过来的 
                    }
                    if (minv[j] < delta) { 
                        delta = minv[j];  // 找到目前所有备选方案中最容免费目标点
                        j1 = j; 
                    }
                }
            }
            
            // 对于已经在冲突区里匹配好的（箱子-目标）对：一加一减，它们之间的 Gap 依然是 0，保持匹配关系不变
            // 对于想跳出冲突区的箱子：u 增加了 delta，那么去冲突区外新目标的 Gap 就减少了 delta，最近目标点刚好变成了 0，解锁了新的通路
            for (int j = 0; j <= n; ++j) {
                if (used[j]) { 
                    u[p[j]] += delta;       // 冲突区里的箱子，补偿值增加
                    v[j] -= delta;          // 冲突区里的目标点，补偿值减少
                } 
                else { minv[j] -= delta; }  // 冲突区外的目标，距离免费更近了 delta 这么多
            }
            j0 = j1;  // 顺着那个最容易变成免费的目标继续找
        } while (p[j0] != 0);  // 直到找到一个还没被任何人占用的目标点

        // 一旦找到了空位，所有在这一轮产生冲突的箱子，都依次往前挪一个位置，这样每个箱子都能得到 Gap==0 的目标点
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    // 统计最终完美匹配的最小总代价
    int total_cost = 0;
    for (int j = 1; j <= n; ++j) {
        int c = cost[p[j] - 1][j - 1];
        if (c >= 9999) return 9999; // 存在无解的匹配
        total_cost += c;
    }
    return total_cost;
}

// 启发式函数 (计算当前状态到通关的最小预估步数，必须小于等于实际步数才能保证最优解)  
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
        int active_t_idx[MAX_BOXES]; 
        int t_count = 0;
        for (size_t i = 0; i < initial_targets.size(); ++i) {
            if (state.target_mask & (1 << i)) active_t_idx[t_count++] = i;
        }
        if (t_count < state.num_boxes) return 9999; 

        // 构建代价矩阵
        int cost_matrix[MAX_BOXES][MAX_BOXES];
        for (int i = 0; i < state.num_boxes; ++i) {
            for (int j = 0; j < state.num_boxes; ++j) {
                // j 映射到可用的目标点集合
                int dist = t_dist[active_t_idx[j]][state.box_y[i]][state.box_x[i]];
                cost_matrix[i][j] = (dist == -1) ? 9999 : dist;
            }
        }

        // 调用 O(N^3) 的极速匈牙利匹配
        int min_h = min_weight_assignment<MAX_BOXES>(cost_matrix, state.num_boxes);
        
        if (min_h >= 9999) return 9999; 
        return min_h + p_cost;
    } 
    else {
        // P2：抛弃全排列，直接极速查表！
        int total_h = 0;
        for (int i = 0; i < state.num_boxes; ++i) {
            int id = state.box_ids[i]; 
            int dist = t_dist[id][state.box_y[i]][state.box_x[i]];
            if (dist == -1) return 9999; 
            total_h += dist;
        }
        return total_h + p_cost;
    }
}


// ============================================================================================
// [模块 5] 工具函数
// ============================================================================================

// 判断一个格子是否是炸弹
inline bool Sokoban::is_bomb(point p) const {
    for (const auto& b : initial_bombs) if (b == p) return true;
    return false;
}

// 根据当前状态，找到指定位置上是否有箱子，如果有返回箱子索引，否则返回-1
inline int Sokoban::find_box_id(const GameState& state, point p) const {
    for (int i = 0; i < state.num_boxes; ++i)
        if (state.box_x[i] == p.x && state.box_y[i] == p.y) return i;
    return -1;
}

// 判断一个格子是否越界
inline bool Sokoban::is_overstep(point p) const {
    if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return true;
    return false;
}