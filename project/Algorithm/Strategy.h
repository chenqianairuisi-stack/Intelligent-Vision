/// \file Strategy.h
/// \brief Core2 炸弹策略规划器，仅根据请求快照生成建议任务
///
/// \details
/// 定义炸弹候选、搜索工作区、清障诊断和性能统计数据结构
/// 声明第一阶段巡图清障与第二阶段推箱清障的任务规划接口

#pragma once

#include "PlanningCommon.h"

using namespace SystemConfig;

#ifndef STRATEGY_ENABLE_CLEAR_DIAG
#define STRATEGY_ENABLE_CLEAR_DIAG 0
#endif

#ifndef STRATEGY_ENABLE_HOT_PROFILE
#define STRATEGY_ENABLE_HOT_PROFILE 0
#endif

#ifndef STRATEGY_ENABLE_SHADOW_CLEAR_CLASSIFIER
#define STRATEGY_ENABLE_SHADOW_CLEAR_CLASSIFIER 0
#endif

#ifndef STRATEGY_ENABLE_SHADOW_CLEAR_DECISION
#define STRATEGY_ENABLE_SHADOW_CLEAR_DECISION 1
#endif

#ifndef STRATEGY_ENABLE_PHASE1_OPTIMIZATION
#define STRATEGY_ENABLE_PHASE1_OPTIMIZATION 1
#endif

namespace StrategyConfig {
    // ------------------------------------------------------------------------
    // 诊断与 profile 开关：多数只影响日志，shadow decision 会影响清障候选来源
    // ------------------------------------------------------------------------
    inline constexpr bool ENABLE_PROFILE = true;                  // profile 总开关，可按 PC 诊断需要调整
    inline constexpr bool ENABLE_CLEAR_DIAG = STRATEGY_ENABLE_CLEAR_DIAG != 0; // 清障诊断开关，只影响报告
    inline constexpr bool ENABLE_HOT_PROFILE = STRATEGY_ENABLE_HOT_PROFILE != 0; // 热点 profile 开关，只影响日志
    inline constexpr bool ENABLE_SHADOW_CLEAR_CLASSIFIER = STRATEGY_ENABLE_SHADOW_CLEAR_CLASSIFIER != 0; // shadow 分类开关，只影响诊断
    inline constexpr bool ENABLE_SHADOW_CLEAR_DECISION = STRATEGY_ENABLE_SHADOW_CLEAR_DECISION != 0; // 候选剪枝：shadow 决策开关，调参需回归清障图
    inline constexpr bool ENABLE_PHASE1_OPTIMIZATION = STRATEGY_ENABLE_PHASE1_OPTIMIZATION != 0; // Phase1 闭环后优化开关，PC A/B 回归可关闭
    inline constexpr int PROFILE_EVAL_LIMIT = 8;                  // 日志/profile：单次策略评估最多记录的 profile pass 数，可调
    inline constexpr int PROFILE_TOP_CANDIDATES = 3;              // 日志/profile：根层候选诊断只保留前几个高分墙位，可调
    inline constexpr int CLEAR_DIAG_LIMIT = 48;                   // 日志/profile：清障诊断最多记录的任务条目数，可调
    inline constexpr int CLEAR_DIAG_PUSH_LIMIT = 4;               // 日志/profile：单个炸弹任务最多记录的清障推箱动作数，可调

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
    // Phase1 闭环后优化：只使用可直接推动的剩余炸弹，避免为非必需任务额外搬箱
    // ------------------------------------------------------------------------
    inline constexpr int PHASE1_OPTIMIZATION_DISTANCE_WEIGHT = 20; // 路径收益：全对偶推距下降的权重，可调
    inline constexpr int PHASE1_OPTIMIZATION_PATROL_WEIGHT = 8;   // 巡图收益：实体通行距离下降的权重，可调
    inline constexpr int PHASE1_OPTIMIZATION_CLARITY_WEIGHT = 2;   // 开图收益：多实体中间墙和局部开区的权重，可调
    inline constexpr int PHASE1_OPTIMIZATION_ROUTE_WEIGHT = 16;    // 执行代价：巡图阶段推弹路径的权重，可调
    inline constexpr int PHASE1_OPTIMIZATION_MIN_SCORE = 36;       // 收益阈值：过滤只有微弱结构提示的可选炸弹，可调
    inline constexpr int PHASE1_OPTIMIZATION_VIEW_MIN_SCORE = 24;  // 观测阈值：无推距收益时必须明显改善多实体观测空间，可调
    inline constexpr int PHASE1_OPTIMIZATION_MIN_DISTANCE_GAIN = 3; // 路径收益：小于该值时不为可选任务提前消耗炸弹，可调
    inline constexpr int PHASE1_OPTIMIZATION_MIN_PATROL_GAIN = 12; // 巡图收益：过滤只缩短少量局部通行距离的爆破，可调
    inline constexpr int PHASE1_OPTIMIZATION_SCAN_LIMIT = 12;      // 候选剪枝：无死锁根层最多做真实爆破重评估的墙位数，可调
    inline constexpr int PHASE1_OPTIMIZATION_SUFFIX_SCAN_LIMIT = 8; // 候选剪枝：已有结构前缀或第二层时保留巡图捷径候选，可调
    inline constexpr int PHASE1_OPTIMIZATION_BRANCH_LIMIT = 2;     // 候选剪枝：每层最多递归展开的优化任务数，可调

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
// 策略实现共享 helper 与缓存结构
// ============================================================================
// 这些声明由 Strategy.cpp 提供实现，Phase1/Phase2 共用

int strategy_box_at(const SokobanLevel& lvl, point p);
bool strategy_target_allowed_for_box(const SokobanLevel& lvl, int box_id, int target_id, bool phase2_specific);
bool strategy_is_goal_for_box(const SokobanLevel& lvl, int box_id, point p, bool phase2_specific);
bool strategy_is_any_target_cell(const SokobanLevel& lvl, point p);
int strategy_nearest_goal_distance(const SokobanLevel& lvl, int box_id, point p, bool phase2_specific);
int strategy_bomb_count(const SokobanLevel& lvl);
int strategy_direct_bomb_cost_for_score(const SokobanLevel& lvl, point player, point bomb_start, point target_wall, int fallback_dist);
int16_t strategy_clamp_i16(int value);
uint32_t strategy_profile_now_us();
uint32_t strategy_profile_elapsed_us(uint32_t start_us);
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

// ============================================================================
// 炸弹战略规划器
// ============================================================================

struct StrategyCandidateProfile {
    int8_t bomb_x = -1;
    int8_t bomb_y = -1;
    int8_t wall_x = -1;
    int8_t wall_y = -1;
    int32_t score = 0;
};

struct StrategyPassProfile {
    int16_t result_deadlocks = 9999;
    int32_t result_profit = -999999;
    uint8_t result_tasks = 0;
    uint16_t root_candidates = 0;
    uint8_t root_branch_limit = 0;
    uint16_t dfs_nodes = 0;
    uint16_t fast_bfs_calls = 0;
    uint16_t candidate_evals = 0;
    uint16_t candidate_kept = 0;
    uint16_t child_branches = 0;
    uint8_t logic_builds = 0;
    uint8_t top_count = 0;
    StrategyCandidateProfile top[StrategyConfig::PROFILE_TOP_CANDIDATES];
    uint16_t local_clear_calls = 0;
    uint16_t local_clear_successes = 0;
    uint16_t materialize_calls = 0;
    uint16_t materialize_successes = 0;
};

enum class StrategyPhase1RepairReject : uint8_t {
    NONE = 0,
    NOT_RUN,
    MATERIALIZE_FAILED,
    UNRESOLVED_OBLIGATION,
    RESIDUAL_DEADLOCKS,
    RESIDUAL_UNREACHABLE,
    NOT_BETTER_THAN_HARD
};

enum class StrategyRescueObligationKind : uint8_t {
    NONE = 0,
    EXPLICIT_PHASE1_TASK,
    EXPLICIT_FUTURE_BOMB,
    UNRESOLVED
};

struct StrategyPhase1RepairStepProfile {
    uint8_t index = 0;
    uint8_t direct_executable = 0;
    uint8_t materialized = 0;
    uint8_t apply_ok = 0;
    uint8_t outstanding_obligations = 0;
    int16_t deadlocks = 9999;
    int16_t unreachable = 9999;
    int32_t distance = 999999;
    int32_t sequence_cost = 0;
    point player = {-1, -1};
    BombTask task;
};

struct StrategyPhase1RepairProfile {
    uint8_t valid = 0;
    uint8_t selected_soft = 0;
    uint8_t source_pass = 0;
    uint8_t repaired_ok = 0;
    uint8_t beats_hard = 0;
    uint8_t repaired_outstanding_obligations = 0;
    uint8_t hard_outstanding_obligations = 0;
    StrategyPhase1RepairReject reject_reason = StrategyPhase1RepairReject::NOT_RUN;

    int16_t soft_deadlocks = 9999;
    int16_t soft_unreachable = 9999;
    int32_t soft_profit = -999999;
    int16_t hard_deadlocks = 9999;
    int16_t hard_unreachable = 9999;
    int32_t hard_distance = 999999;
    int16_t repaired_deadlocks = 9999;
    int16_t repaired_unreachable = 9999;
    int32_t repaired_distance = 999999;
    int32_t repaired_cost = 0;

    StaticArray<BombTask, MAX_BOMBS> raw_tasks;
    StaticArray<BombTask, MAX_BOMBS> repaired_tasks;
    uint8_t step_count = 0;
    StrategyPhase1RepairStepProfile steps[MAX_BOMBS];
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

struct StrategyClearPushProfile {
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

struct StrategyClearRouteProfile {
    uint8_t valid = 0;
    uint8_t eval_index = 255;
    uint8_t pass = 255;
    uint8_t success = 0;
    StrategyClearMethod method = StrategyClearMethod::NONE;
    uint8_t phase2_specific = 0;
    uint8_t include_player_access_clear = 0;
    point bomb_start = {-1, -1};
    point target_wall = {-1, -1};
    int16_t cost = 0;
    uint8_t route_len = 0;
    uint8_t blocker_count = 0;
    uint8_t push_count = 0;
    StrategyClearPushProfile pushes[StrategyConfig::CLEAR_DIAG_PUSH_LIMIT];
};

struct StrategyEvalProfile {
    uint8_t mode = 0;
    uint8_t selected_pass = 255;
    int16_t selected_deadlocks = 9999;
    int32_t selected_profit = -999999;
    uint8_t selected_tasks = 0;
    StrategyPassProfile passes[3];
};

struct StrategyHotProfile {
    uint32_t fast_bfs_calls = 0;
    uint32_t fast_bfs_us = 0;
    uint32_t fast_bfs_player_reach_calls = 0;
    uint32_t fast_bfs_state_pops = 0;
    uint16_t fast_bfs_max_queue = 0;

    uint32_t macro_soft_calls = 0;
    uint32_t macro_soft_us = 0;
    uint32_t macro_soft_state_pops = 0;
    uint16_t macro_soft_max_queue = 0;

    uint32_t local_clear_calls = 0;
    uint32_t local_clear_successes = 0;
    uint32_t local_clear_us = 0;
    uint32_t soft_route_builds = 0;
    uint32_t soft_route_successes = 0;

    uint32_t box_push_checks = 0;
    uint32_t box_push_successes = 0;
    uint32_t bomb_path_checks = 0;
    uint32_t bomb_path_successes = 0;
    uint32_t player_path_checks = 0;

    uint32_t real_clear_nodes = 0;
    uint32_t real_clear_candidate_total = 0;
    uint32_t real_clear_try_total = 0;
    uint16_t real_clear_max_depth = 0;
};

#if STRATEGY_ENABLE_SHADOW_CLEAR_CLASSIFIER
struct StrategyShadowClearProfile {
    uint32_t route_clear_attempts = 0;
    uint32_t route_clear_successes = 0;
    uint32_t route_clear_failed_no_blocker = 0;
    uint32_t blocker_bomb_corridor = 0;
    uint32_t blocker_bomb_real_path = 0;
    uint32_t blocker_push_stand_nearby = 0;
    uint32_t blocker_push_stand_exact = 0;
    uint32_t blocker_push_stand_near_only = 0;
    uint32_t blocker_route_nearby = 0;
    uint32_t blocker_recursive = 0;
    uint32_t blocker_real_support = 0;

    uint32_t accepted_direct_safe = 0;
    uint32_t accepted_theoretical_rescue = 0;
    uint32_t accepted_open_path_only = 0;
    uint32_t accepted_dead_parking = 0;
    uint32_t accepted_exact_reason = 0;
    uint32_t accepted_nearby_reason = 0;

    uint32_t real_nodes = 0;
    uint32_t real_source_exact = 0;
    uint32_t real_source_near = 0;
    uint32_t real_source_far = 0;
    uint32_t real_push_candidates = 0;
    uint32_t real_push_executable = 0;
    uint32_t real_opens_path = 0;
    uint32_t real_parking_checks = 0;
    uint32_t real_parking_direct_safe = 0;
    uint32_t real_parking_theoretical = 0;
    uint32_t real_parking_dead = 0;
    uint32_t real_parking_rejected = 0;

    uint32_t decide_keep_exact_blocker = 0;
    uint32_t decide_deprioritize_near_stand = 0;
    uint32_t decide_deprioritize_route_near = 0;
    uint32_t decide_keep_recursive = 0;
    uint32_t decide_validate_real_exact = 0;
    uint32_t decide_validate_real_near = 0;
    uint32_t decide_deprioritize_real_far = 0;
    uint32_t decide_accept_direct_safe = 0;
    uint32_t decide_require_theory_proof = 0;
    uint32_t decide_require_open_path = 0;
    uint32_t decide_reject_dead_parking = 0;
    uint32_t decide_reject_no_blocker = 0;
};
#endif

struct StrategyProfile {
    uint8_t eval_count = 0;
    uint16_t dropped_evals = 0;
    StrategyEvalProfile evals[StrategyConfig::PROFILE_EVAL_LIMIT];
    StrategyPhase1RepairProfile phase1_repair;
    StrategyHotProfile hot;
#if STRATEGY_ENABLE_SHADOW_CLEAR_CLASSIFIER
    // PC 端 softpass 清障影子分类，移植 MCU 时关闭 STRATEGY_ENABLE_SHADOW_CLEAR_CLASSIFIER 可裁掉
    StrategyShadowClearProfile shadow_clear;
#endif
#if STRATEGY_ENABLE_CLEAR_DIAG
    // PC 端 softpass 清障诊断，移植 MCU 时关闭 STRATEGY_ENABLE_CLEAR_DIAG 可裁掉
    uint8_t clear_diag_count = 0;
    uint16_t dropped_clear_diags = 0;
    StrategyClearRouteProfile clear_diags[StrategyConfig::CLEAR_DIAG_LIMIT];
#endif
};

class StrategicPlanner {
public:
    StrategicPlanner() = default;

    void reset_profile();
    const StrategyProfile& get_profile() const { return profile; }

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

    void stamp_selected_tasks(DFSResult& result, bool preserve_essential = false);
    void optimize_phase1_bomb_assignment(const SokobanLevel& level, DFSResult& result);
    void append_phase1_optimization_tasks(const SokobanLevel& level, DFSResult& result);

    bool apply_executable_bomb_task(SokobanLevel& work, point& player, const BombTask& task, int* sequence_cost = nullptr);
    bool materialize_phase1_sequence(
        const SokobanLevel& level,
        StaticArray<BombTask, MAX_BOMBS>& seq,
        int* out_sequence_cost = nullptr,
        StrategyPhase1RepairProfile* repair_diag = nullptr,
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

    StrategyProfile profile;
    StrategyEvalProfile* active_profile_eval = nullptr;
    uint8_t active_profile_eval_index = 255;
    uint8_t active_profile_pass = 0;

    StrategyClearRouteProfile* begin_profile_clear(
        const SokobanLevel& level,
        int bomb_idx,
        point target_wall,
        bool phase2_specific,
        bool include_player_access_clear);
    void record_profile_clear_route(
        StrategyClearRouteProfile* diag,
        int route_len,
        int blocker_count);
    void record_profile_clear_push(
        StrategyClearRouteProfile* diag,
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
        StrategyClearRouteProfile* diag,
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
