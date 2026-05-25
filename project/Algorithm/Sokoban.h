#pragma once

#include <array>
#include <cstring>
#include "PlanningCommon.h"
#include "system_config.h"

using namespace SystemConfig;

#ifndef SOKOBAN_ENABLE_PROFILE
#define SOKOBAN_ENABLE_PROFILE 1
#endif

#ifndef SOKOBAN_INITIAL_THRESHOLD_BOOST
#define SOKOBAN_INITIAL_THRESHOLD_BOOST 0
#endif

// ============================================================================
// IDA* 搜索状态
// ============================================================================

struct GameState {
    point player;              // 玩家当前位置

    int8_t box_x[MAX_BOXES];   // 箱子 x 坐标
    int8_t box_y[MAX_BOXES];   // 箱子 y 坐标
    uint8_t box_ids[MAX_BOXES]; // Phase2 使用：当前箱子对应的目标编号
    uint8_t num_boxes;          // 当前仍未归位的箱子数量
    uint8_t target_mask;        // 尚未完成的目标集合，bit=1 表示该目标仍需处理

    int8_t bomb_x[MAX_BOMBS];  // 炸弹 x 坐标
    int8_t bomb_y[MAX_BOMBS];  // 炸弹 y 坐标
    uint8_t num_bombs;         // 当前地图中的炸弹数量
    uint8_t blown_mask;        // 已引爆炸弹集合，bit=1 表示对应炸弹已经清墙

    uint32_t hash;             // Zobrist 哈希
};

// ============================================================================
// 置换表
// ============================================================================

constexpr uint32_t TT_SIZE = 32768;

struct alignas(4) TTEntry {
    uint16_t sig;              // hash 高 16 位签名
    uint16_t value;            // 该状态已知不可行的剩余阈值下界
};

// ============================================================================
// 推箱子核心求解器
// ============================================================================

// ============================================================================
// Sokoban 搜索性能探针
// ============================================================================

struct SokobanProfile {
    uint32_t expanded_nodes = 0;          // 真正展开并生成动作的搜索节点数
    uint32_t generated_moves = 0;         // 所有节点累计生成的候选推动作数量
    uint32_t tt_hits = 0;                 // 置换表命中并剪枝的次数
    uint32_t heuristic_dead_prunes = 0;   // 启发式判断不可达/死锁而剪枝的次数
    uint32_t threshold_prunes = 0;        // IDA* f 值超过当前阈值的剪枝次数
    uint32_t path_cycle_prunes = 0;       // 当前递归路径内状态重复的剪枝次数
    uint32_t static_deadlock_prunes = 0;  // 推入静态死锁格或目标距离场不可达的剪枝次数
    uint32_t block_2x2_prunes = 0;        // 2x2 团块死锁剪枝次数
    uint16_t max_depth = 0;               // 本次搜索达到的最大递归深度
    uint16_t threshold_iterations = 0;    // IDA* 阈值迭代轮数
    uint16_t final_threshold = 0;         // 搜索结束时的阈值，便于对比启发式强弱
};

class Sokoban {
public:
    // --- 生命周期与外部输入 ---
    bool load_from_vision(const SokobanLevel& level);
    void bind_semantics(const uint8_t* matched_ids);
    void load_bomb_tasks(const BombTask* tasks, int count);

    // --- 求解入口与结果读取 ---
    bool solve(GameMode mode);
    const StaticArray<point, MAX_PATH_LENGTH>& get_result_path() const { return final_path; }
    const SokobanProfile& get_profile() const { return profile; }

private:
    static constexpr int MAX_NODE_MOVES = 24;

    struct TinyMove {
        uint8_t entity_idx;         // [0, num_boxes) 为箱子；其后为炸弹编号偏移
        uint8_t dir;                // 推动方向，对应 MOVE[dir]
        uint8_t walk_dist;          // 玩家从当前位置走到推位的距离
        uint8_t slide_dist;         // 隧道自动滑行的额外格数
        bool triggers_explosion;    // 本次推动是否直接触发炸弹清墙
    };

    struct EvalMove {
        uint8_t move_idx;           // 指向 TinyMove 数组的原始下标
        int16_t sort_key;           // 越小越优先尝试
    };

    // 单节点动态占用表：以 192B 栈空间换掉热路径内反复线性扫描箱子/炸弹数组。
    // cell 中 0..15 表示箱子下标，BOMB_BASE 之后表示炸弹下标，-1 表示空格。
    struct NodeOccupancy {
        static constexpr int8_t EMPTY = -1;
        static constexpr int8_t BOMB_BASE = 16;
        int8_t cell[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

        template <GameMode Mode>
        inline void build(const GameState& state) {
            std::memset(cell, EMPTY, sizeof(cell));
            for (int i = 0; i < state.num_boxes; ++i) {
                cell[state.box_y[i]][state.box_x[i]] = static_cast<int8_t>(i);
            }
            if constexpr (Mode == GameMode::PHASE2_SPECIFIC) {
                for (int b = 0; b < state.num_bombs; ++b) {
                    if (!(state.blown_mask & (1 << b))) {
                        cell[state.bomb_y[b]][state.bomb_x[b]] = static_cast<int8_t>(BOMB_BASE + b);
                    }
                }
            }
        }

        inline int box_at(point p) const {
            if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return -1;
            int v = cell[p.y][p.x];
            return (v >= 0 && v < BOMB_BASE) ? v : -1;
        }

        template <GameMode Mode>
        inline int bomb_at(point p) const {
            if constexpr (Mode == GameMode::PHASE1_ANY) {
                (void)p;
                return -1;
            } else {
                if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return -1;
                int v = cell[p.y][p.x];
                return (v >= BOMB_BASE) ? (v - BOMB_BASE) : -1;
            }
        }
    };
    // --- IDA* 主流程 ---
    template <GameMode Mode> bool solve_internal();
    template <GameMode Mode> int ida_star_search(
        const GameState& state,
        int g,
        int depth,
        int threshold,
        StaticArray<point, MAX_PATH_LENGTH>& path,
        int last_entity = -1,
        uint8_t last_push_dir = 4);
    template <GameMode Mode> int count_active_bomb_tasks(const GameState& state) const;
    template <GameMode Mode> int phase2_box_push_lb_sum_if_needed(const GameState& state, int active_bombs) const;
    template <GameMode Mode> int heuristic_weight_num(int active_entities) const;
    int probe_transposition(uint32_t hash, int g, int threshold);
    void store_transposition(uint32_t hash, int g, int min_next_threshold);
    template <GameMode Mode> bool is_active_target_cell(const GameState& state, point p, int box_idx, int& out_idx) const;
    template <GameMode Mode> bool is_tunnel_dynamic(point p, int dir, uint8_t blown_mask) const;
    template <GameMode Mode> int generate_moves(
        const GameState& state,
        const NodeOccupancy& occupancy,
        int active_bombs,
        TinyMove moves[MAX_NODE_MOVES]);
    template <GameMode Mode> void optimize_final_path_turns();
    template <GameMode Mode> bool append_optimized_walk_segment(
        const GameState& state,
        point start,
        point goal,
        uint8_t prev_dir,
        uint8_t next_dir,
        int max_steps,
        StaticArray<point, MAX_PATH_LENGTH>& out_path,
        uint8_t& out_dir) const;

    // --- 启发式与哈希 ---
    template <GameMode Mode> int get_heuristic(const GameState& state) const;
    int task_route_walk_lower_bound(
        point player,
        const point starts[MAX_BOXES + MAX_BOMBS],
        const point ends[MAX_BOXES + MAX_BOMBS],
        int task_count) const;
    template <size_t N> int min_weight_assignment(int cost[N][N], int n) const;
    template <GameMode Mode> uint32_t compute_hash(const GameState& state) const;

    // --- 空间预计算 ---
    void init_zobrist();
    void precompute_target_distances();
    void precompute_bomb_distances();
    void precompute_walk_distances();
    void precompute_deadlocks();

    // --- 高频物理查询 ---
    inline bool is_overstep(point p) const;
    inline bool is_solid(point p, uint8_t blown_mask) const;
    inline bool is_bomb(point p) const;
    inline int find_box_id(const GameState& state, point p) const;
    inline int get_bomb_id(const GameState& state, point p, GameMode Mode) const;
    inline int walk_dist_between(point from, point to) const;
    inline int walk_to_push_stand_lower_bound(point from, point obj) const;

    // --- 静态地图、初始状态与求解结果 ---
    std::array<std::array<int8_t, MAP_MAX_WIDTH>, MAP_MAX_HEIGHT> map{};
    point player_start;
    GameState initial_state;
    StaticArray<point, MAX_BOXES> initial_targets;
    StaticArray<point, MAX_BOMBS> initial_bombs;
    StaticArray<point, MAX_PATH_LENGTH> final_path;
    StaticArray<BombTask, MAX_BOMBS> bomb_tasks;
    uint8_t num_bomb_tasks = 0;

    // --- Zobrist 随机表与路径环路检测 ---
    uint32_t ZOBRIST_BOX[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint32_t ZOBRIST_SPECIFIC_BOX[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint32_t ZOBRIST_PLAYER[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint32_t ZOBRIST_TARGET[MAX_BOXES];
    uint32_t ZOBRIST_BOMB[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint32_t ZOBRIST_BLOWN_MASK[1 << MAX_BOMBS];
    uint32_t path_hashes[MAX_PATH_LENGTH];

    // --- 启发式距离场与死锁缓存 ---
    int16_t t_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t relaxed_walk_dist[MAP_CELL_COUNT][MAP_CELL_COUNT];
    bool is_dead[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

    int16_t b_dist[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t wall_clear_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    SokobanProfile profile;
};

extern TTEntry TT[TT_SIZE];
extern Sokoban solver;
