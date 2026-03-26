#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include "system_config.h"

using namespace SystemConfig;

// 纯算法层的数据结构，与视觉完全解耦
struct SokobanLevel {
    std::array<std::array<int8_t, MAP_MAX_WIDTH>, MAP_MAX_HEIGHT> map;
    point player_start;
    
    point boxes[MAX_BOXES];
    uint8_t box_count;
    
    point targets[MAX_BOXES];
    uint8_t target_count;

    point bombs[MAX_BOMBS];
    uint8_t bomb_count;
};

// 四个移动方向：上、右、下、左
constexpr point MOVE[4] = {{0,1}, {1,0}, {0,-1}, {-1,0}};

// 游戏状态结构体（用于搜索树中的节点表示）
struct GameState {
    point player;               // 玩家当前坐标
    int8_t box_x[MAX_BOXES];    // 所有幸存箱子的X坐标
    int8_t box_y[MAX_BOXES];    // 所有幸存箱子的Y坐标
    uint8_t num_boxes;          // 当前场上还剩几个箱子
    uint8_t target_mask;        // 位图：记录哪些目标点还没被消耗 (第i位为1表示第i个目标点还在，0表示已消失)
    uint32_t hash;              // 当前状态的 Zobrist 哈希值（用于查表排重）
};

// 置换表（Transposition Table, TT）大小，必须是2的幂
constexpr int TT_SIZE = 16384;  
struct TTEntry {
    uint16_t sig;         // 记录完整哈希，防止哈希冲突
    uint8_t  remaining;   // 记录该状态下剩余允许的搜索深度（步数），用于剪枝
};


class sokoban {
public:
    sokoban();

    bool load_from_vision(const SokobanLevel& level);
    const StaticArray<point, MAX_PATH_LENGTH>& get_result_path() const { return final_path; }

    bool solve();  

protected:
    std::array<std::array<int8_t, MAP_MAX_WIDTH>, MAP_MAX_HEIGHT> map{};  // 静态地图：0空地, 1墙, 2目标
    point player_start;
    StaticArray<point, MAX_BOMBS> initial_bombs;        // 炸弹位置
    StaticArray<point, MAX_BOXES> initial_targets;      // 目标点位置
    StaticArray<point, MAX_PATH_LENGTH> final_path;     // 最终求解路径

    GameState initial_state; // 初始状态

    // Zobrist 随机数表，用于快速计算状态哈希
    uint32_t ZOBRIST_BOX[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];       // 如果坐标 (x, y) 有一个箱子，就对应这个随机数
    uint32_t ZOBRIST_PLAYER[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];    // 如果玩家在坐标 (x, y)，就对应这个随机数
    uint32_t ZOBRIST_TARGET[MAX_BOXES];                        // 对应目标点状态，目标消失也会改变哈希
    
    TTEntry TT[TT_SIZE];                                       // 置换表（记忆化搜索）
    uint32_t path_hashes[MAX_PATH_LENGTH];                     // 记录起点到当前这步沿途所有的状态的哈希，防止环路（死循环）

    bool is_dead[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];               // 死锁表：标记死角  
    int16_t t_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];  // 启发式表：记录地图上任意一点到各个目标点的最短距离

    void init_zobrist();
    void precompute_deadlocks();
    void precompute_target_distances();
    
    uint32_t compute_hash(const GameState& state) const;
    int get_heuristic(const GameState& state) const;       
    int find_box_id(const GameState& state, point p) const;
    bool is_bomb(point p) const;

    // 核心搜索函数：IDA* 的递归实现（其实更应该叫 ida_star_search）
    int dfs(GameState state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path);
};

extern sokoban solver;