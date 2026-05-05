#pragma once
#include <array>
#include "system_config.h"
#include "Strategy.h"

using namespace SystemConfig;

// ============================================================================
// [核心数据结构]
// ============================================================================

// 游戏状态结构体（用于 IDA* 搜索树中的节点表示）
struct GameState {
    // --- 小车状态 ---
    point player;                // 玩家（小车）当前坐标

    // --- 箱子与目标状态 ---
    int8_t box_x[MAX_BOXES];     // 所有幸存箱子的 X 坐标
    int8_t box_y[MAX_BOXES];     // 所有幸存箱子的 Y 坐标
    uint8_t box_ids[MAX_BOXES];  // 语义绑定数组：box_ids[i] 表示第 i 个箱子所对应的目标点 ID（仅高级阶段有效）
    uint8_t num_boxes;           // 幸存箱子数量
    uint8_t target_mask;         // 目标点位图：第 i 位为 1 表示第 i 个目标点尚未被填满，0 表示已填满并消失

    // --- 炸弹与墙壁状态 ---
    int8_t bomb_x[MAX_BOMBS];    // 所有未引爆炸弹的 X 坐标
    int8_t bomb_y[MAX_BOMBS];    // 所有未引爆炸弹的 Y 坐标
    uint8_t num_bombs;           // 未引爆的炸弹数量
    uint8_t blown_mask;          // 墙体破坏位图：第 i 位为 1 代表第 i 个炸弹已引爆，其对应的 3x3 墙体已物理坍塌

    // --- 哈希特征 ---
    uint32_t hash;               // 当前状态的 Zobrist 全局哈希值（用于 O(1) 置换表查表排重）
};


// 置换表 (Transposition Table) 节点定义，采用 alignas(4) 保证 32-bit 对齐
constexpr uint32_t TT_SIZE = 65536;  // 必须是 2 的幂，用于 & (TT_SIZE - 1) 极速取模
struct alignas(4) TTEntry {
    uint16_t sig;        // 状态特征码 (Hash 的高 16 位，用于哈希碰撞校验)
    uint16_t value;      // 价值缓存 (推箱子存 remaining剩余步数)
};


// ============================================================================
// [推箱子核心求解引擎]
// ============================================================================

class Sokoban {
public:    
    // 从视觉模块导入基础地图数据
    bool load_from_vision(const SokobanLevel& level);        

    // 绑定 N-1 视觉语义匹配结果（第二阶段调用）
    void bind_semantics(const uint8_t* matched_ids);
    
    // 加载宏观战略规划出的炸弹任务（第二阶段调用）
    void load_bomb_tasks(const BombTask* tasks, int count);
    
    // IDA* 混合推箱求解器 [mode GameMode::PHASE1_ANY(盲推) 或 GameMode::PHASE2_SPECIFIC(语义推)]
    bool solve(GameMode mode);  

    // 获取求解出的最优路径（坐标序列）
    const StaticArray<point, MAX_PATH_LENGTH>& get_result_path() const { return final_path; }

private:
    // =========================================================
    // 内部方法
    // =========================================================

    // --- A. 核心算法 (通过模板化展开为不同赛段的机器码，避免运行时分支开销) ---
    template <GameMode Mode> bool solve_internal(); 
    template <GameMode Mode> int ida_star_search(GameState state, int g, int depth, int threshold, StaticArray<point, MAX_PATH_LENGTH>& path, int last_entity = -1);
    
    // --- B. 启发式评估器 (Heuristic) ---
    template <GameMode Mode> int get_heuristic(const GameState& state) const;
    template<size_t N> int min_weight_assignment(int cost[N][N], int n) const;  // 匈牙利算法/最小权匹配
    template <GameMode Mode> uint32_t compute_hash(const GameState& state) const;  // 哈希计算器

    // --- C. 空间预计算引擎  ---
    void init_zobrist();
    void precompute_target_distances();
    void precompute_bomb_distances();
    void precompute_deadlocks();
    
    // --- D. 物理引擎与状态查询 (内联展开) ---
    inline bool is_overstep(point p) const;                              // 边界检查
    inline bool is_solid(point p, uint8_t blown_mask) const;             // 碰撞检测 (结合炸弹坍塌掩码)
    inline bool is_bomb(point p) const;                                  // 检查是否是初始炸弹
    inline int find_box_id(const GameState& state, point p) const;       // 查找某坐标的箱子索引
    inline int get_bomb_id(const GameState& state, point p, GameMode Mode) const; // 查找某坐标的炸弹索引


    // =========================================================
    // 数据存储区 (按访问频次与生命周期分组)
    // =========================================================

    // --- 1. 静态场馆数据 ---
    std::array<std::array<int8_t, MAP_MAX_WIDTH>, MAP_MAX_HEIGHT> map{}; // 静态图纸：0空地, 1墙, 2目标 (注意：墙可能会被炸毁，需配合 mask 判定)
    point player_start;                                                  // 玩家发车坐标
    GameState initial_state;                                             // 搜索树的根节点状态
    StaticArray<point, MAX_BOXES> initial_targets;                       // 全局目标点位置表
    StaticArray<point, MAX_BOMBS> initial_bombs;                         // 全局初始炸弹位置表
    StaticArray<point, MAX_PATH_LENGTH> final_path;                      // 缓存解出的最终路径
    StaticArray<BombTask, MAX_BOMBS> bomb_tasks;                         // 战略层下发的具体炸弹执行任务
    uint8_t num_bomb_tasks = 0;                                          // 有效炸弹任务数
    

    // --- 2. Zobrist 随机哈希表 (用于 O(1) 计算任意图状态的 Hash) ---
    uint32_t ZOBRIST_BOX[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];                       // 盲推阶段：任意箱子在 (x,y) 的哈希
    uint32_t ZOBRIST_SPECIFIC_BOX[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];   // 语义阶段：特定 ID 的箱子在 (x,y) 的哈希
    uint32_t ZOBRIST_PLAYER[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];                    // 玩家在 (x,y) 的哈希
    uint32_t ZOBRIST_TARGET[MAX_BOXES];                                        // 第 i 个目标点是否存活的特征哈希
    uint32_t ZOBRIST_BOMB[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];           // 第 i 个炸弹在 (x,y) 的哈希
    uint32_t ZOBRIST_BLOWN_MASK[1 << MAX_BOMBS];                               // 不同墙体坍塌排列组合的特征哈希
    
    // 搜索回溯环路检测（单路深度最大 MAX_PATH_LENGTH）
    uint32_t path_hashes[MAX_PATH_LENGTH];              

    // --- 3. 启发式与死锁缓存区 (预计算产物) ---
    int16_t t_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 目标距离场：t_dist[i][y][x] 为把第 i 个目标点的箱子逆推到 (x,y) 的最少步数 (-1 为死锁)
    bool is_dead[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];              // 绝对死锁静态判定表：如果推到这格绝对无解，直接剪枝

    int16_t b_dist[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 炸弹距离场：b_dist[i][y][x] 为把炸弹推到其任务引爆点所需的步数
    uint8_t wall_clear_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];   // O(1) 坍塌掩码：wall_clear_mask[y][x] 第 i 位为 1，代表只要第 i 个炸弹爆了，这格墙就变平地
};

// ============================================================================
// 全局实例
// ============================================================================
extern TTEntry TT[TT_SIZE];  // 全局置换表实例（置于 DTCM 内存极速区，与 Exploration 引擎共享内存池以节省 RAM）
extern Sokoban solver;       // 全局唯一求解器实例