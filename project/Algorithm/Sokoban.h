#pragma once
#include <array>
#include "system_config.h"

using namespace SystemConfig;

// 定义赛段模式（编译期路由）
enum class GameMode : uint8_t {
    PHASE1_ANY,       // 第一阶段：任意箱子 -> 任意目标
    PHASE2_SPECIFIC   // 第二阶段：特定箱子 -> 特定目标
};

// 游戏状态结构体（用于搜索树中的节点表示）
struct GameState {
    point player;                // 玩家当前坐标
    int8_t box_x[MAX_BOXES];     // 所有幸存箱子的X坐标
    int8_t box_y[MAX_BOXES];     // 所有幸存箱子的Y坐标
    uint8_t box_ids[MAX_BOXES];  // box_ids[i] 表示第 i 个箱子对于应的目标点ID
    uint8_t num_boxes;           // 当前场上还剩几个箱子
    uint8_t target_mask;         // 位图：记录哪些目标点还没被消耗 (第i位为1表示第i个目标点还在，0表示已消失)
    uint32_t hash;               // 当前状态的 Zobrist 哈希值（用于查表排重）
};


// 置换表 (Transposition Table, TT)，大小必须是2的幂
constexpr uint32_t TT_SIZE = 65536;  
struct alignas(4) TTEntry {
    uint16_t sig;        // 特征码/Mask (Sokoban 和 Exploration 共用)
    uint16_t value;      // 复用字段: Sokoban 存 remaining(剩余步数), Exploration 存 cost(累计代价)
};


class Sokoban {
public:
    Sokoban() = default;

    bool solve(GameMode mode);  
    void bind_semantics(const uint8_t* matched_ids);

    bool load_from_vision(const SokobanLevel& level);
    const StaticArray<point, MAX_PATH_LENGTH>& get_result_path() const { return final_path; }

private:
    std::array<std::array<int8_t, MAP_MAX_WIDTH>, MAP_MAX_HEIGHT> map{};  // 静态地图：0空地, 1墙, 2目标
    point player_start;
    StaticArray<point, MAX_BOMBS> initial_bombs;        // 炸弹位置
    StaticArray<point, MAX_BOXES> initial_targets;      // 目标点位置
    StaticArray<point, MAX_PATH_LENGTH> final_path;     // 最终求解路径

    GameState initial_state;  // 初始状态

    // Zobrist 随机数表，用于快速计算状态哈希
    uint32_t ZOBRIST_BOX[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];                       // 如果坐标 (x, y) 有一个箱子，就对应这个随机数
    uint32_t ZOBRIST_SPECIFIC_BOX[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];   // 专属箱子哈希表（有对应关系的箱子）
    uint32_t ZOBRIST_PLAYER[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];                    // 如果玩家在坐标 (x, y)，就对应这个随机数
    uint32_t ZOBRIST_TARGET[MAX_BOXES];                                        // 对应目标点状态，目标消失也会改变哈希
    
    uint32_t path_hashes[MAX_PATH_LENGTH];              // 记录起点到当前这步沿途所有的状态的哈希，防止环路（死循环）

    int16_t t_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];            // 启发式距离表：记录地图上任意一点到各个目标点的最短推挤距离（-1表示不可达）
    bool is_dead[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];                         // 死锁表


    // 内部辅助函数
    void init_zobrist();
    void precompute_target_distances();
    void precompute_deadlocks();
    void precompute_specific_deadlocks();
    
    inline int find_box_id(const GameState& state, point p) const;
    inline bool is_bomb(point p) const;
    inline bool is_overstep(point p) const;

    // 模板化的内部求解函数，根据不同赛段模式编译成不同机器码
    template <GameMode Mode> bool solve_internal(); 
    template <GameMode Mode> int ida_star_search(GameState state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path);
    template <GameMode Mode> uint32_t compute_hash(const GameState& state) const;
    template <GameMode Mode> int get_heuristic(const GameState& state) const;
};


extern TTEntry TT[TT_SIZE];  // 置换表全局实例（与 exploration 共享）
extern Sokoban solver;