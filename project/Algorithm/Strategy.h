#pragma once

#include "PlanningCommon.h"

using namespace SystemConfig;

#ifndef STRATEGY_ENABLE_SHADOW_CLEAR_DECISION
#define STRATEGY_ENABLE_SHADOW_CLEAR_DECISION 1
#endif

namespace StrategyConfig {
    // ------------------------------------------------------------------------
    // 清障候选开关：shadow decision 会影响候选来源，调参需回归清障图
    // ------------------------------------------------------------------------
    inline constexpr bool ENABLE_SHADOW_CLEAR_DECISION = STRATEGY_ENABLE_SHADOW_CLEAR_DECISION != 0; // 候选剪枝：shadow 决策开关，调参需回归清障图
    inline constexpr int REAL_CLEAR_TRACE_LIMIT = 8;              // 接口约束：真实清障搜索保留的推箱链长度

    // ------------------------------------------------------------------------
    // DFS 搜索宽度：候选剪枝，调大更稳但更慢
    // ------------------------------------------------------------------------
    inline constexpr uint8_t PHASE1_SELECTION_RESTRICTIONS = 3;   // 候选剪枝：Phase1 每层最多展开的高分爆破候选数，可调但要回归重点图
    inline constexpr uint8_t PHASE2_SELECTION_RESTRICTIONS = 3;   // 候选剪枝：Phase2 每层最多展开的高分爆破候选数，可调但要回归重点图

    // ------------------------------------------------------------------------
    // 距离与收益阈值：结构评分与执行代价的分界
    // ------------------------------------------------------------------------
    inline constexpr int16_t INF_DIST = 9999;                     // 接口约束：不可达距离占位值，必须大于地图内任何真实路径代价，不建议手调
    inline constexpr int PHASE1_SOFT_REPLACE_PROFIT_MARGIN = 20;  // 结构评分/执行代价：soft pass 替换 hard pass 的最低收益边际，可调

    // ------------------------------------------------------------------------
    // 局部清障实体化：执行代价与候选剪枝
    // ------------------------------------------------------------------------
    inline constexpr int LOCAL_CLEAR_MAX_TASKS = 8;               // 执行代价：单个炸弹任务最多补充的推箱清障动作数，可调
    inline constexpr int LOCAL_CLEAR_MAX_ITER = 5;                // 执行代价：局部清障最多迭代轮数，限制链式搬箱深度，可调
    inline constexpr int LOCAL_CLEAR_CANDIDATE_LIMIT = 10;        // 候选剪枝：每轮清障最多尝试的候选停车点数，可调
    inline constexpr int LOCAL_CLEAR_OPEN_VERIFY_LIMIT = 24;      // 候选剪枝：便宜评分后进入真实开路验证的候选数，可调
    inline constexpr int LOCAL_CLEAR_CHAIN_DEPTH = 1;             // 执行代价：递归搬箱清障深度，过大容易生成长任务链，谨慎调
    inline constexpr int LOCAL_CLEAR_FAILURE_CACHE_LIMIT = 128;   // 候选剪枝：单进程局部清障失败缓存容量，可调
    inline constexpr int SHADOW_CLEAR_NEAR_STAND_MAX_DIST = 3;    // 候选剪枝：shadow 近邻站位候选的最大曼哈顿半径，可调
    inline constexpr int SHADOW_CLEAR_NEAR_STAND_LIMIT = 2;       // 候选剪枝：每段软路线最多保留的近邻站位候选数，可调
    inline constexpr int REAL_CLEAR_SOURCE_SUPPORT_LIMIT = 2;     // 候选剪枝：真实清障源箱子离软路线的近邻阈值，可调
    inline constexpr int REAL_CLEAR_TARGET_STEP_LIMIT = 4;        // 候选剪枝：真实清障停车点沿单方向最多外推步数，可调
    inline constexpr int REAL_CLEAR_CANDIDATE_POOL = 48;          // 候选剪枝：真实清障轻量筛选后的候选池容量，可调
    inline constexpr int REAL_CLEAR_VERIFY_SCAN_LIMIT = 24;       // 候选剪枝：真实清障单阶段最多验证的候选数，可调
    inline constexpr int REAL_CLEAR_BRANCH_LIMIT = 14;            // 候选剪枝：真实清障单阶段最多递归展开的分支数，可调
    inline constexpr int REAL_CLEAR_MEMO_LIMIT = 256;             // 候选剪枝：真实清障失败状态记忆容量，可调

    // ------------------------------------------------------------------------
    // Phase1 soft pass 实体化预算：候选剪枝与执行代价平衡
    // ------------------------------------------------------------------------
    inline constexpr int PHASE1_CLEAR_DIVERSITY_SCAN_LIMIT = 5;   // 候选剪枝：根层全断联时额外扫描的低推箱成本候选数，可调
    inline constexpr int PHASE1_EXECUTABLE_BRANCH_SCAN_LIMIT = 18; // 候选剪枝：非根层 soft pass 真实 successor 扫描上限，可调
    inline constexpr int PHASE1_EXECUTABLE_ROOT_SCAN_LIMIT = 24;   // 候选剪枝：根层 soft pass 真实 successor 扫描上限，可调
    inline constexpr int PHASE1_EXECUTABLE_FINAL_SCAN_LIMIT = 48;  // 候选剪枝：最后一颗炸弹放宽扫描，避免排序误差漏掉闭环墙，可调
    inline constexpr int PHASE1_EXECUTABLE_FINAL_BRANCH_LIMIT = 4;  // 执行代价：最后一层只比较少量真实分支，避免清障实体化拖慢热图，可调

    // ------------------------------------------------------------------------
    // Phase1 任意匹配缓存：接口/缓存容量
    // ------------------------------------------------------------------------
    inline constexpr int PHASE1_MATCH_MASKS = 1 << MAX_BOXES;     // 接口约束：任意箱-目标匹配 DP 的状态掩码数量，由 MAX_BOXES 决定，不手调
}

// ============================================================================
// 策略 cpp 共享 helper 与缓存结构
// ============================================================================
// 这些声明由 StrategyCommon.cpp 提供实现，Phase1/Phase2 只使用，不再各自重复声明

int strategy_box_at(const SokobanLevel& lvl, point p);
bool strategy_target_allowed_for_box(const SokobanLevel& lvl, int box_id, int target_id, bool phase2_specific);
bool strategy_is_goal_for_box(const SokobanLevel& lvl, int box_id, point p, bool phase2_specific);
bool strategy_is_any_target_cell(const SokobanLevel& lvl, point p);
int strategy_nearest_goal_distance(const SokobanLevel& lvl, int box_id, point p, bool phase2_specific);
int strategy_bomb_count(const SokobanLevel& lvl);
int strategy_direct_bomb_cost_for_score(const SokobanLevel& lvl, point player, point bomb_start, point target_wall, int fallback_dist);
int16_t strategy_clamp_i16(int value);
void mark_soft_deadlock_boxes(const SokobanLevel& lvl, bool out_hard[MAX_BOXES]);

struct LogicBlastScores {
    int16_t score[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t l1_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t l2_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t l3_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t bomb_unlock_hits[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
};

// ============================================================================
// 策略层内部数据结构
// ============================================================================

struct BombCandidate {
    uint8_t bomb_idx;     // 使用哪一颗炸弹
    int8_t x, y;          // 候选墙体坐标
    int score;            // 候选收益分数，越高越优先

    bool operator<(const BombCandidate& other) const {
        return score > other.score;
    }
};

struct StrategyDfsScratch {
    StaticArray<BombCandidate, 256> preliminary[MAX_BOMBS + 1];
    StaticArray<BombCandidate, 256> candidates[MAX_BOMBS + 1];
    bool probe_valid[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    uint8_t probe_bomb_idx[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    int probe_deadlocks[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    int probe_unreachable[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    int probe_distance[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
};

using StrategyBoxDepthDistances = int16_t[MAX_BOMBS + 1][MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
using StrategyBombDepthDistances = int16_t[MAX_BOMBS + 1][MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
using StrategyPlayerReachDepthMap = bool[MAX_BOMBS + 1][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
using StrategyBoxDistances = int16_t[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
using StrategyBombDistances = int16_t[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
using StrategyMatchDp = int[StrategyConfig::PHASE1_MATCH_MASKS];

struct StrategySearchWorkspace {
    StrategyBoxDepthDistances& dfs_dist_box;
    StrategyBombDepthDistances& dfs_dist_bomb;
    StrategyPlayerReachDepthMap& dfs_player_vis;
    StrategyBoxDepthDistances& soft_box_dist_by_depth;
    StrategyBoxDistances& probe_box_dist;
    StrategyBombDistances& probe_bomb_dist;
    StrategyBombDepthDistances& hard_bomb_dist_by_depth;
    StrategyBombDepthDistances& strict_bomb_dist_by_depth;
    StrategyMatchDp& matching_dp;
    StrategyMatchDp& matching_next;
    StrategyDfsScratch& dfs;
    LogicBlastScores& logic;
};

// Phase1/Phase2 顺序复用同一块搜索工作区，避免两个阶段各持有一整份大缓存
StrategySearchWorkspace& strategy_search_workspace();

struct DFSResult {
    StaticArray<BombTask, MAX_BOMBS> tasks;
    int deadlocks_remaining;  // 执行 tasks 后仍未解除的死锁数量，越少越好
    int net_profit;           // 综合收益分数，越高越好
    int unreachable_pairs_remaining = 9999; // Phase1：仍不可达的箱子-目标对数量
    int bomb_supply_score = 0;              // Phase1：当前炸弹对关键缺陷墙的可执行供给
};

enum class StrategyRescueObligationKind : uint8_t {
    NONE = 0,
    EXPLICIT_PHASE1_TASK,
    EXPLICIT_FUTURE_BOMB,
    UNRESOLVED
};

enum class StrategyClearReason : uint8_t {
    NONE = 0,
    BOMB_CORRIDOR_BLOCKER,
    BOMB_REAL_PATH_BLOCKER,
    PUSH_STAND_NEARBY,
    ROUTE_NEARBY,
    RECURSIVE_BOX_BLOCKER,
    REAL_CLEAR_SUPPORT
};

enum class StrategyClearParking : uint8_t {
    UNKNOWN = 0,
    DIRECT_SAFE,
    THEORETICAL_RESCUE,
    OPEN_PATH_ONLY,
    DEAD_PARKING
};

struct StrategyClearObligation {
    uint8_t box_id = 255;
    StrategyClearReason reason = StrategyClearReason::NONE;
    StrategyClearParking parking = StrategyClearParking::UNKNOWN;
    StrategyRescueObligationKind obligation = StrategyRescueObligationKind::NONE;
    uint8_t creator_task_index = 255;
    uint8_t owner_task_index = 255;
    point box_start = {-1, -1};
    point box_target = {-1, -1};
    point owner_bomb_start = {-1, -1};
    point owner_target_wall = {-1, -1};
};

enum class StrategyClearMethod : uint8_t {
    NONE = 0,
    DIRECT_BOMB_PATH,
    SOFT_ROUTE_CLEAR,
    REAL_CLEAR_SEARCH
};

struct StrategyClearPushTrace {
    uint8_t box_id = 255;
    StrategyClearReason reason = StrategyClearReason::NONE;
    StrategyClearParking parking = StrategyClearParking::UNKNOWN;
    StrategyRescueObligationKind obligation = StrategyRescueObligationKind::NONE;
    uint8_t owner_task_index = 255;
    point box_start = {-1, -1};
    point box_target = {-1, -1};
    point owner_bomb_start = {-1, -1};
    point owner_target_wall = {-1, -1};
    uint8_t depth = 0;
    uint8_t opens_bomb_path = 0;
    uint8_t safe_without_open_path = 0;
    int16_t score = 0;
};

struct StrategyClearRouteTrace {
    uint8_t push_count = 0;
};

// ============================================================================
// 炸弹战略规划器
// ============================================================================

class StrategicPlanner {
public:
    StrategicPlanner() = default;

    StaticArray<BombTask, MAX_BOMBS> plan_phase1_bombs(const SokobanLevel& level);
    StaticArray<BombTask, MAX_BOMBS> plan_phase2_bombs(
        const SokobanLevel& level,
        const StaticArray<BombTask, MAX_BOMBS>& inherited_tasks);

    bool materialize_bomb_task(
        const SokobanLevel& level,
        point player_start,
        const BombTask& task,
        BombTask& out_task,
        bool phase2_specific = false,
        StaticArray<StrategyClearObligation, 8>* out_obligations = nullptr,
        uint8_t creator_task_index = 255);

private:
    // --- 当前规划上下文 ---
    SokobanLevel cached_level;
    bool phase1_soft_bomb_eval = false;
    bool phase1_defer_soft_successor = false;
    bool phase2_soft_bomb_eval = false;
    StaticArray<BombTask, MAX_BOMBS> phase1_phase2_inherited_candidates;

    // --- DFS 策略搜索 ---
    void dfs_phase1_bomb_sequence(
        const SokobanLevel& current_lvl,
        point player_start,
        StaticArray<BombTask, MAX_BOMBS> current_seq,
        int cost_so_far,
        int depth,
        DFSResult& best_res);
    void dfs_phase2_bomb_sequence(
        const SokobanLevel& current_lvl,
        point player_start,
        StaticArray<BombTask, MAX_BOMBS> current_seq,
        int cost_so_far,
        int depth,
        DFSResult& best_res);

    void execute_phase1_search_pass(const SokobanLevel& level, uint8_t pass, DFSResult& out_res);
    void execute_phase2_search_pass(const SokobanLevel& level, uint8_t pass, DFSResult& out_res);

    void stamp_selected_tasks(DFSResult& result);

    bool apply_executable_bomb_task(SokobanLevel& work, point& player, const BombTask& task, int* sequence_cost = nullptr);
    bool materialize_phase1_sequence(
        const SokobanLevel& level,
        StaticArray<BombTask, MAX_BOMBS>& seq,
        int* out_sequence_cost = nullptr,
        StaticArray<StrategyClearObligation, MAX_BOMBS * 8>* out_obligations = nullptr);
    bool evaluate_phase1_task_sequence(
        const SokobanLevel& level,
        const StaticArray<BombTask, MAX_BOMBS>& seq,
        int& out_deadlocks,
        int& out_unreachable,
        int& out_distance,
        const StaticArray<StrategyClearObligation, MAX_BOMBS * 8>* obligations = nullptr,
        int* out_unresolved_obligations = nullptr);
    void evaluate_phase1_matching_pairs(
        const SokobanLevel& level,
        point player,
        int selected_count,
        bool soft_boxes,
        bool strict_soft_boxes,
        int16_t out_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
        int& out_deadlocks,
        int& out_distance);
    bool materialize_phase2_sequence(const SokobanLevel& level, StaticArray<BombTask, MAX_BOMBS>& seq);
    bool apply_phase2_task_sequence(
        const SokobanLevel& level,
        const StaticArray<BombTask, MAX_BOMBS>& seq,
        SokobanLevel& out_level,
        point& out_player,
        int* out_sequence_cost = nullptr);
    void evaluate_phase2_level_matching(
        const SokobanLevel& level,
        point player,
        int& out_deadlocks,
        int& out_distance);
    bool evaluate_phase2_task_sequence(
        const SokobanLevel& level,
        const StaticArray<BombTask, MAX_BOMBS>& seq,
        int& out_deadlocks,
        int& out_distance,
        int* out_sequence_cost = nullptr);

    // --- 推物体距离场与可执行性验证 ---
    void fast_push_bfs(
        const SokobanLevel& lvl,
        point start_obj,
        point player_start,
        bool is_bomb,
        int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
        bool soft_boxes = false,
        bool strict_soft_boxes = false);
    void macro_soft_dijkstra(const SokobanLevel& lvl, point start_obj, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]);
    bool are_fast_bomb_tasks_directly_executable(const SokobanLevel& level, const StaticArray<BombTask, MAX_BOMBS>& tasks);

    // --- 轻量局部清障实体化 ---
    bool local_clear_bomb_route(
        const SokobanLevel& start_lvl,
        int bomb_idx,
        point target_wall,
        bool phase2_specific,
        SokobanLevel& out_lvl,
        int& out_cost,
        StaticArray<BoxPushTask, 8>& out_box_pushes,
        bool include_player_access_clear = false,
        StaticArray<StrategyClearObligation, 8>* out_obligations = nullptr,
        uint8_t creator_task_index = 255);

    void begin_profile_eval(uint8_t mode);
    void set_profile_pass(uint8_t pass);
    void record_profile_result(uint8_t pass, const DFSResult& result);
    void record_profile_selected(uint8_t pass, const DFSResult& result);
    void record_profile_root_candidates(
        const SokobanLevel& level,
        const StaticArray<BombCandidate, 256>& candidates,
        int branch_limit);
    void record_profile_dfs_node();
    void record_profile_fast_bfs_call();
    void record_profile_candidate_eval();
    void record_profile_candidate_kept();
    void record_profile_child_branch();
    void record_profile_logic_build();
    void record_profile_local_clear_call();
    void record_profile_local_clear_success();
    void record_profile_local_clear_time(uint32_t elapsed_us);
    void record_profile_materialize_call();
    void record_profile_materialize_success();
    void record_profile_fast_bfs_detail(
        uint32_t elapsed_us,
        uint32_t player_reach_calls,
        uint32_t state_pops,
        uint16_t max_queue);
    void record_profile_macro_soft_call();
    void record_profile_macro_soft_detail(
        uint32_t elapsed_us,
        uint32_t state_pops,
        uint16_t max_queue);
    void record_profile_soft_route_build(int route_len);
    void record_profile_box_push_check(bool success);
    void record_profile_bomb_path_check(bool success);
    void record_profile_player_path_check();
    void record_profile_real_clear_node(int depth);
    void record_profile_real_clear_candidates(int candidate_count, int try_limit);
    void record_shadow_clear_route_attempt(bool success, int blocker_count);
    void record_shadow_clear_blocker(StrategyClearReason reason);
    void record_shadow_push_stand_blocker(int stand_dist);
    void record_shadow_clear_accept(StrategyClearReason reason, StrategyClearParking parking);
    void record_shadow_clear_decision(StrategyClearReason reason, StrategyClearParking parking, int support_dist, bool accepted);
    void record_shadow_real_node();
    void record_shadow_real_source_distance(int source_support_dist);
    void record_shadow_real_push_candidate();
    void record_shadow_real_push_executable();
    void record_shadow_real_opens_path();
    void record_shadow_real_parking(StrategyClearParking parking, bool accepted);

    StrategyClearRouteTrace* begin_profile_clear(
        const SokobanLevel& level,
        int bomb_idx,
        point target_wall,
        bool phase2_specific,
        bool include_player_access_clear);
    void record_profile_clear_route(
        StrategyClearRouteTrace* diag,
        int route_len,
        int blocker_count);
    void record_profile_clear_push(
        StrategyClearRouteTrace* diag,
        uint8_t box_id,
        StrategyClearReason reason,
        StrategyClearParking parking,
        StrategyRescueObligationKind obligation,
        uint8_t owner_task_index,
        point box_start,
        point box_target,
        point owner_bomb_start,
        point owner_target_wall,
        int depth,
        bool opens_bomb_path,
        bool safe_without_open_path,
        int score);
    void finish_profile_clear(
        StrategyClearRouteTrace* diag,
        bool success,
        StrategyClearMethod method,
        int cost);
    static void merge_clear_obligation(
        StaticArray<StrategyClearObligation, MAX_BOMBS * 8>& obligations,
        const StrategyClearObligation& obligation);
    static int count_unresolved_clear_obligations(
        const StaticArray<StrategyClearObligation, MAX_BOMBS * 8>& obligations,
        int resolved_task_count);
};

extern StrategicPlanner strategic_planner;
