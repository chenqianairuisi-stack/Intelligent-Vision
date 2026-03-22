#include "planning.h"

__attribute__((section(".dtcm_data"))) sokoban solver;


// 伪随机数生成器 (Xorshift)，用于生成 Zobrist 哈希用的随机数
static uint32_t xor_state = 123456789;
static uint32_t xorshift32() {
    xor_state ^= xor_state << 13; xor_state ^= xor_state >> 17; xor_state ^= xor_state << 5;
    return xor_state;
}

sokoban::sokoban() { }

// 初始化游戏状态，加载地图、箱子、目标点和炸弹等信息
bool sokoban::load_from_vision(const SokobanLevel& level) {
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
    for (int i = 0; i < level.target_count; ++i) {
        initial_targets.push_back(level.targets[i]);
    }

    initial_bombs.clear();
    for (int i = 0; i < level.bomb_count; ++i) {
        initial_bombs.push_back(level.bombs[i]);
    }

    // 初始化启发式表和哈希
    init_zobrist();
    precompute_target_distances();
    precompute_deadlocks();
    
    initial_state.hash = compute_hash(initial_state);
    return true;
}


// 启发式函数 (计算当前状态到通关的最小预估步数，必须小于等于实际步数才能保证最优解)  
__attribute__((section(".ramfunc")))
int sokoban::get_heuristic(const GameState& state) const {
    if (state.num_boxes == 0) return 0; 
    
    int min_h = 9999;

    // 初始化箱子排列索引
    int p[MAX_BOXES];
    for (int i = 0; i < state.num_boxes; ++i) p[i] = i;  

    // 初始化目标点排列索引
    int active_t_idx[MAX_BOXES];
    int t_count = 0;
    for (size_t i = 0; i < initial_targets.size(); ++i) {
        if (state.target_mask & (1 << i)) active_t_idx[t_count++] = i;
    }

    if (t_count < state.num_boxes) return 9999; // 异常情况：目标比箱子少，不可能通关

    // 因为箱子最多才4个，这里直接使用全排列 (4! = 24种情况)，找出把现存箱子推到现存目标的总距离最小的组合
    do {
        int current_h = 0;
        for (int i = 0; i < state.num_boxes; ++i) {
            int t_id = active_t_idx[p[i]];  // 根据当前的排列方案，给第 i 号箱子指派一个目标点 t_id
            int dist = t_dist[t_id][state.box_y[i]][state.box_x[i]];  //目标点 t_id 到箱子 i 的距离
            if (dist == -1) { current_h = 9999; break; }  // 如果某个箱子到目标不可达，这个组合作废
            current_h += dist;
        }
        if (current_h < min_h) min_h = current_h;
    } while (std::next_permutation(p, p + state.num_boxes));
    
    if (min_h >= 9999) return 9999; // 所有组合都不可达，说明死局了

    // 加上人跑到离人最近的一个箱子的距离，使得启发函数更精准
    int min_p_dist = 9999;
    for (int i = 0; i < state.num_boxes; ++i) {
        int d = std::abs(state.player.x - state.box_x[i]) + std::abs(state.player.y - state.box_y[i]) - 1;
        if (d < 0) d = 0;
        if (d < min_p_dist) min_p_dist = d;
    }
    if (min_p_dist != 9999) min_h += min_p_dist;
    
    return min_h;
}


// 全局变量，避免递归时频繁开辟内存
static uint16_t bfs_visited[MAP_MAX_HEIGHT];  // 每行一个16位整数的位图，标记玩家在BFS中访问过哪些格子（1表示访问过，0表示未访问）
static point bfs_q[256];
static int8_t bfs_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

// DFS函数：IDA* 算法的递归搜索部分 [g: 已走步数, threshold: 当前深度限制阈值]  
// 使用了递归，移植到嵌入式上时应配置栈区大小，建议至少分配 32 KB 的栈空间！！！！！！！！！！！！！！！！！！！！！！
__attribute__((section(".ramfunc")))
int sokoban::dfs(GameState state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path) {
    int h = get_heuristic(state);
    if (h >= 9999) return 9999;     // 剪枝：死局
    int f = g + h;
    if (f > threshold) return f;    // 剪枝：超过当前迭代的限制，返回新的阈值建议
    
    if (state.num_boxes == 0) return -1;  // 箱子全消失，返回-1作为成功标志

    // ---------- 宏操作第一步：BFS搜索玩家能到达的所有空地 ----------
    std::memset(bfs_visited, 0, sizeof(bfs_visited));
    int head = 0, tail = 0;
    
    bfs_q[tail++] = state.player;
    bfs_visited[state.player.y] |= (1 << state.player.x);
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
            if(np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                if(!(bfs_visited[np.y] & (1 << np.x)) && map[np.y][np.x] != 1 && !is_bomb(np)) {
                    if(find_box_id(state, np) == -1) {   
                        bfs_visited[np.y] |= (1 << np.x);
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

    // 置换表(TT)查表剪枝：如果以前遍历过这个状态，并且当时允许的步数(剩余容错)比现在大，说明当前分支没前途，直接剪掉
    int remaining = threshold - g;
    int tt_idx = state.hash & (TT_SIZE - 1);
    uint16_t sig = (uint16_t)(state.hash >> 16);  // 提取特征码
    if (TT[tt_idx].sig == sig && TT[tt_idx].remaining >= remaining) return threshold + 1;  // 特征码匹配且剩余容错更大，说明当前分支不如之前的分支，剪掉

    auto is_active_target = [&](point p, int& out_idx) {
        for (size_t t = 0; t < initial_targets.size(); ++t) {
            if ((state.target_mask & (1 << t)) && initial_targets[t] == p) {
                out_idx = t; return true;
            }
        }
        return false;
    };

    // ---------- 宏操作第二步：寻找有哪些可以执行的推箱子动作 ----------
    struct TinyMove {
        uint16_t box_idx   : 3; // 0-7   (占 3 bits)
        uint16_t dir       : 2; // 0-3   (占 2 bits)
        uint16_t is_double : 1; // 0或1  (占 1 bit)
        uint16_t box2_idx  : 3; // 0-7   (占 3 bits)
        uint16_t walk_dist : 7; // 0-127 (占 7 bits, 共16 bits)
    };
    TinyMove moves[24]; 
    int num_moves = 0;

    for (uint8_t i = 0; i < state.num_boxes; ++i) {
        point box_pos = {state.box_x[i], state.box_y[i]};

        for (uint8_t dir = 0; dir < 4; ++dir) {
            // 确保人能站过去
            point push_from = box_pos - MOVE[dir]; // 推箱子时人站的位置
            if (push_from.x >= 0 && push_from.x < MAP_MAX_WIDTH && push_from.y >= 0 && push_from.y < MAP_MAX_HEIGHT) {
                if (bfs_visited[push_from.y] & (1 << push_from.x)) {
                    // 检查推箱子后的位置是否合法
                    point push_to = box_pos + MOVE[dir]; // 箱子要被推到的位置
                    if (push_to.x < 0 || push_to.x >= MAP_MAX_WIDTH || push_to.y < 0 || push_to.y >= MAP_MAX_HEIGHT) continue;
                    if (map[push_to.y][push_to.x] == 1 || is_bomb(push_to)) continue;  // 撞墙或撞炸弹
                    
                    int b2 = find_box_id(state, push_to);
                    int dummy_t;
                    if (b2 == -1) {
                        // 【剪枝】推过去如果是个死角且不是目标点，则不允许推
                        if (is_dead[push_to.y][push_to.x] && !is_active_target(push_to, dummy_t)) continue; 
                        if (num_moves < 24) moves[num_moves++] = {
                            static_cast<uint16_t>(i),
                            static_cast<uint16_t>(dir),
                            static_cast<uint16_t>(0),
                            static_cast<uint16_t>(0),
                            static_cast<uint16_t>(static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x]))
                        };
                    } else {
                        // 推两个箱子的情况（支持连推）
                        point push_to_2 = push_to + MOVE[dir];
                        if (push_to_2.x < 0 || push_to_2.x >= MAP_MAX_WIDTH || push_to_2.y < 0 || push_to_2.y >= MAP_MAX_HEIGHT) continue;
                        if (map[push_to_2.y][push_to_2.x] == 1 || is_bomb(push_to_2)) continue;
                        if (find_box_id(state, push_to_2) == -1) {
                            if (is_dead[push_to.y][push_to.x] && !is_active_target(push_to, dummy_t)) continue;
                            if (is_dead[push_to_2.y][push_to_2.x] && !is_active_target(push_to_2, dummy_t)) continue;
                            if (num_moves < 24) moves[num_moves++] = {
                                static_cast<uint16_t>(i),
                                static_cast<uint16_t>(dir),
                                static_cast<uint16_t>(1),
                                static_cast<uint16_t>(b2),
                                static_cast<uint16_t>(static_cast<uint8_t>(bfs_dist[push_from.y][push_from.x]))
                            };
                        }
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

        GameState next_state = state;
        next_state.player = box_pos; // 推完后，人走到了原来箱子的位置
        
        int new_box_count = 0;
        int8_t new_bx[MAX_BOXES];
        int8_t new_by[MAX_BOXES];
        uint8_t new_target_mask = state.target_mask;
        uint32_t new_hash = state.hash;
        
        // 抠掉旧的玩家位置并加上新的玩家位置哈希(原箱子处)
        new_hash ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
        new_hash ^= ZOBRIST_PLAYER[box_pos.y][box_pos.x];

        // 高效重组剩余的箱子(若到终点自动剔除)
        for (int i = 0; i < state.num_boxes; ++i) {
            point p = {state.box_x[i], state.box_y[i]};
            new_hash ^= ZOBRIST_BOX[p.y][p.x];  // 先抹去旧箱子的哈希

            if (i == mv.box_idx) p = push_to;   // 更新被推箱子的位置
            else if (mv.is_double && i == mv.box2_idx) p = push_to + MOVE[mv.dir];  // 连推情况

            // 检查此时的箱子是不是踩上了剩余的目标
            int t_idx = -1;
            for (size_t t = 0; t < initial_targets.size(); ++t) {
                if ((new_target_mask & (1 << t)) && initial_targets[t] == p) { t_idx = t; break; }
            }

            if (t_idx != -1) {
                // 推进目标点，箱子和目标点消失
                new_target_mask &= ~(1 << t_idx);
                new_hash ^= ZOBRIST_TARGET[t_idx];  // 抹去该目标点的哈希
            } else {
                // 未推到目标点
                new_bx[new_box_count] = p.x;
                new_by[new_box_count] = p.y;
                new_hash ^= ZOBRIST_BOX[p.y][p.x];  // 加上新箱子位置的哈希
                new_box_count++;
            }
        }

        next_state.num_boxes = new_box_count;
        next_state.target_mask = new_target_mask;
        next_state.hash = new_hash;
        for (int i = 0; i < new_box_count; ++i) {
            next_state.box_x[i] = new_bx[i];
            next_state.box_y[i] = new_by[i];
        }

        int step_cost = mv.walk_dist + 1; // 走到推点距离 + 1下推的动作
        int res = dfs(next_state, g + step_cost, depth + 1, threshold, path);
        
        if (res == -1) { 
            // 如果成功找到解，利用当前BFS访问记录，反推出从原点走到推箱子点的精确路径
            std::memset(bfs_visited, 0, sizeof(bfs_visited));
            int h2 = 0, t2 = 0;
            static point temp_parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
            
            bfs_q[t2++] = state.player;
            bfs_visited[state.player.y] |= (1 << state.player.x);
            
            while(h2 < t2) {
                point curr = bfs_q[h2++];
                if (curr == push_from) break;
                for(int d2 = 0; d2 < 4; ++d2) {
                    point np = curr + MOVE[d2];
                    if(np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                        if(!(bfs_visited[np.y] & (1 << np.x)) && map[np.y][np.x] != 1 && !is_bomb(np)) {
                            if(find_box_id(state, np) == -1) {
                                bfs_visited[np.y] |= (1 << np.x);
                                temp_parent[np.y][np.x] = curr;
                                bfs_q[t2++] = np;
                            }
                        }
                    }
                }
            }
            
            path.push_back(box_pos); 
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
    TT[tt_idx].sig = sig;
    TT[tt_idx].remaining = remaining;

    return min_next_threshold;
}

// 求解器入口
bool sokoban::solve() {
    if (initial_state.num_boxes != initial_targets.size()) return false;
    
    std::memset(TT, 0, sizeof(TT)); 
    int threshold = get_heuristic(initial_state); // IDA* 初始阈值设为启发函数的预估最小步数
    StaticArray<point, MAX_PATH_LENGTH> rev_path;
    rev_path.reserve(MAX_PATH_LENGTH);  // 预先分配足够的空间，避免递归过程中频繁扩容带来的性能损失

    // IDA* 主循环：阈值逐渐增大，直到找到解或者超过 MAX_PATH_LENGTH
    while (threshold <= MAX_PATH_LENGTH) {
        int res = dfs(initial_state, 0, 0, threshold, rev_path);
        
        // 找到答案
        if (res == -1) { 
            rev_path.push_back(initial_state.player);           // 加入起点
            std::reverse(rev_path.begin(), rev_path.end());     // 倒序变为正序路径
            final_path = rev_path;
            return true;
        }
        if (res >= 9999) break;                                 // 无解
        threshold = res;                                        // 用新的下界更新阈值
    }
    return false;
}




// ------------------------------------------------ 辅助函数 ------------------------------------------------


// 初始化 Zobrist 哈希表，给每一个可能的状态分配一个固定的随机数
void sokoban::init_zobrist() {
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y)
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            ZOBRIST_BOX[y][x] = xorshift32();
            ZOBRIST_PLAYER[y][x] = xorshift32();
        }
    for (int i = 0; i < MAX_BOXES; ++i) ZOBRIST_TARGET[i] = xorshift32();
}

// 计算当前状态的哈希值（用于查表排重）
__attribute__((section(".ramfunc")))
uint32_t sokoban::compute_hash(const GameState& state) const {
    uint32_t h = 0;
    for (int i = 0; i < state.num_boxes; ++i) h ^= ZOBRIST_BOX[state.box_y[i]][state.box_x[i]];
    h ^= ZOBRIST_PLAYER[state.player.y][state.player.x];
    for (size_t i = 0; i < initial_targets.size(); ++i) {
        // 只有当前存活的目标点才计算进哈希
        if (state.target_mask & (1 << i)) h ^= ZOBRIST_TARGET[i];
    }
    return h;
}

// 静态死锁检测预处理
void sokoban::precompute_deadlocks() {
    std::memset(is_dead, 0, sizeof(is_dead));
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            if (map[y][x] == 1) continue; // 是墙不用管
            // 检查上下左右是否有墙
            bool up = (y + 1 < MAP_MAX_HEIGHT) && (map[y+1][x] == 1);
            bool down = (y - 1 >= 0) && (map[y-1][x] == 1);
            bool left = (x - 1 >= 0) && (map[y][x-1] == 1);
            bool right = (x + 1 < MAP_MAX_WIDTH) && (map[y][x+1] == 1);
            // 如果一个非墙的格子，(上或下有墙) 且 (左或右有墙)，那它就是一个死角(Corner Deadlock)
            if ((up || down) && (left || right)) is_dead[y][x] = true;
        }
    }
}

// 反向 BFS: 获取任意空地到每个目标点的最短距离     //？可达？
void sokoban::precompute_target_distances() {
    std::memset(t_dist, -1, sizeof(t_dist));
    for (size_t i = 0; i < initial_targets.size(); ++i) {
        point start = initial_targets[i];
        point q[256];
        int head = 0, tail = 0;
        
        q[tail++] = start;
        t_dist[i][start.y][start.x] = 0; 
        
        while (head < tail) {
            point curr = q[head++];
            int dist = t_dist[i][curr.y][curr.x];
            for (int dir = 0; dir < 4; ++dir) {
                point np = curr + MOVE[dir];
                // 如果在地图内、不是墙、不是炸弹，且尚未访问过
                if (np.x >= 0 && np.x < MAP_MAX_WIDTH && np.y >= 0 && np.y < MAP_MAX_HEIGHT) {
                    if (map[np.y][np.x] != 1 && !is_bomb(np) && t_dist[i][np.y][np.x] == -1) {
                        t_dist[i][np.y][np.x] = dist + 1;
                        q[tail++] = np;
                    }
                }
            }
        }
    }
}


bool sokoban::is_bomb(point p) const {
    for (const auto& b : initial_bombs) if (b == p) return true;
    return false;
}

int sokoban::find_box_id(const GameState& state, point p) const {
    for (int i = 0; i < state.num_boxes; ++i)
        if (state.box_x[i] == p.x && state.box_y[i] == p.y) return i;
    return -1;
}
