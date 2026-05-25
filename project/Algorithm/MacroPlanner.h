#pragma once

#include "Exploration.h"

/*
MacroPlanner 模块说明

前半阶段的职责边界：
1. Strategy 负责定“必做骨架”，从全局角度选出必须做或很值得做的炸弹任务。
2. Exploration 负责把骨架展开成高质量参考序列，序列中包含观测任务和寻图必推炸弹。
3. MacroPlanner 负责在线实时调度。它不重新做全局搜索，而是在参考序列主线上维护固定候选位：
    - 参考候选位：严格按 Exploration 输入顺序执行观测或必推炸弹。参考任务必须执行。
    如果当前参考任务不可直接执行，只允许在该位置插入清障推箱，清障不消耗参考游标。
    - 完成式推箱候选位：已实现。语义明确后，评估是否提前把箱子直接推到对应目标点。
    - 接近式推箱候选位：预留。后续用于对已观测箱子做短推接近。
    - 机会性炸弹候选位：预留。后续用于调度 Strategy 中未进入参考序列的近距离炸弹。

调度原则：
- 参考序列是前半阶段主线，保证巡图不会被局部收益牵走。
- 非参考候选位只做插入式动作，必须超过阈值才抢占当前参考动作。
- 参考序列全部执行完，并达到 n-1 箱子 / n-1 目标点观测要求后，Macro 阶段结束。
- 结束时由 MacroPlanner 根据已观测语义补全配对，再进入第二阶段 IDA* 融合规划。

当前实现重点：
- 完成式推箱评分不惩罚推箱路径本身，因为这部分早晚要做。
- 它主要衡量推箱后接回下一参考任务的成本变化、未来返程节省、以及降低后续搜索压力的收益。
*/

/// \brief MacroPlanner 维护的在线语义知识状态
///
/// \details
/// observed_mask 的低 box_count 位表示箱子，后续 target_count 位表示目标点。
/// candidate_targets / bound_target 用于把 ART2 识别结果逐步收敛成
/// 第二阶段 Sokoban 需要的 box -> target 配对。
struct KnowledgeState {
    uint32_t observed_mask = 0;              // 已观测实体 bitmask
    uint16_t candidate_targets[MAX_BOXES];   // 每个箱子仍可能绑定到哪些目标点
    uint8_t bound_target[MAX_BOXES];         // 已确定绑定时的目标点编号
    bool is_bound[MAX_BOXES];                // 对应箱子是否已经完成语义绑定
};

/// \brief 单次在线决策所需的动态上下文
struct MacroPlanContext {
    SokobanLevel level;                                      // 当前地图快照
    point player;                                            // 当前小车位置
    float yaw = -1.0f;                                       // 当前观测朝向，负数表示未知
    const StaticArray<BombTask, MAX_BOMBS>* bomb_tasks = nullptr; // Strategy 输出的炸弹任务集合
};

static constexpr int MAX_ACTION_CANDIDATES = 16;  // 每轮决策的最大动作候选数

class MacroPlanner {
public:
    // ------------------------------------------------------------------------
    // 生命周期
    // ------------------------------------------------------------------------

    /// \brief 开始一张新地图的 Macro 阶段
    /// \param level 当前逻辑地图
    void reset(const SokobanLevel& level);

    /// \brief 装入 Exploration 生成的参考序列，并将参考游标归零
    /// \param plan 离线巡图参考动作序列
    void set_reference_plan(const StaticArray<MacroAction, 32>& plan);

    // ------------------------------------------------------------------------
    // 在线调度入口
    // ------------------------------------------------------------------------

    /// \brief 按固定候选位选择下一条宏动作
    /// \param ctx 当前地图、位置、朝向和炸弹任务上下文
    /// \param out_action 输出本轮要执行的宏动作
    /// \return 成功选出动作时返回 true；参考主线不可推进且无法清障时返回 false
    bool plan_next_action(const MacroPlanContext& ctx, MacroAction& out_action);

    // ------------------------------------------------------------------------
    // 语义知识接口
    // ------------------------------------------------------------------------

    /// \brief 写入一次观测结果
    /// \param level 当前逻辑地图
    /// \param mask 本次观测覆盖的实体掩码
    void apply_observation(const SokobanLevel& level, uint32_t mask);

    /// \brief 同步上层视觉语义池，并更新 Macro 内部配对状态
    /// \param labels ART2 语义标签数组，-1 表示未知
    void sync_semantics(const int8_t* labels);

    /// \brief 判断 Macro 阶段是否可以交给 Sokoban
    /// \param level 当前逻辑地图
    /// \return 参考序列完成且语义信息足够时返回 true
    bool ready_for_sokoban(const SokobanLevel& level) const;

    const KnowledgeState& knowledge() const { return knowledge_state; }

    /// \brief 输出第二阶段需要的 box -> target 配对
    /// \param out_ids 输出数组，out_ids[box_id] = target_id
    /// \param box_count 当前箱子数量
    /// \return 配对完整时返回 true
    bool fill_matched_ids(uint8_t* out_ids, uint8_t box_count);

private:
    // ------------------------------------------------------------------------
    // 内部类型
    // ------------------------------------------------------------------------

    enum class SlotKind : uint8_t {
        NONE,
        REFERENCE,
        COMPLETION_PUSH,
        APPROACH_PUSH,
        OPPORTUNISTIC_BOMB
    };

    struct SlotCandidate {
        MacroAction action;
        StaticArray<MacroAction, 4> followups;
        SlotKind slot = SlotKind::NONE;
        int score = 0;
        bool valid = false;
        bool consumes_reference = false;
    };

    // ------------------------------------------------------------------------
    // 内部状态
    // ------------------------------------------------------------------------

    KnowledgeState knowledge_state;
    StaticArray<MacroAction, 32> reference_plan;
    StaticArray<MacroAction, 4> pending_actions;
    int reference_cursor = 0;
    int pending_cursor = 0;

    int8_t semantic_labels[MAX_ENTITIES];  // 语义标签缓存，-1 表示未知
    uint8_t box_count = 0;
    uint8_t target_count = 0;

    // ------------------------------------------------------------------------
    // 语义知识维护
    // ------------------------------------------------------------------------

    uint16_t default_candidate_mask() const;
    bool match_semantics(const SokobanLevel& level, const int8_t* semantic_labels, uint8_t* out_matched_ids) const;
    bool has_required_observations(const SokobanLevel& level) const;
    bool reference_sequence_done(const SokobanLevel& level) const;

    // ------------------------------------------------------------------------
    // 参考主线推进
    // ------------------------------------------------------------------------

    void advance_reference_cursor(const SokobanLevel& level);
    bool refresh_observe_action(const SokobanLevel& level, MacroAction& action) const;
    bool prepare_reference_action(const MacroPlanContext& ctx, const MacroAction& raw_action, MacroAction& prepared_action) const;
    bool build_reference_slot(const MacroPlanContext& ctx, SlotCandidate& slot);
    bool build_bomb_observe_split(const MacroPlanContext& ctx, const MacroAction& bomb_action,
                                MacroAction& prefix_action, MacroAction& observe_action,
                                MacroAction& suffix_action) const;
    bool build_reference_clearance(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const;

    // ------------------------------------------------------------------------
    // 插入候选槽位
    // ------------------------------------------------------------------------

    bool build_completion_push_slot(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const;
    bool build_approach_push_slot(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const;
    bool build_opportunistic_bomb_slot(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const;

    // ------------------------------------------------------------------------
    // 动作验证与评分辅助
    // ------------------------------------------------------------------------

    bool simulate_action(const SokobanLevel& level, point player, uint32_t observed_mask, const MacroAction& action,
                        SokobanLevel& out_level, point& out_player, uint32_t& out_observed, int& out_cost) const;
    int reference_access_cost(const SokobanLevel& level, point player, float observe_yaw, const MacroAction& reference_action) const;
    int estimate_deferred_return_cost(const SokobanLevel& level, uint8_t box_id, uint8_t target_id) const;

    bool validate_completion_push(const SokobanLevel& level, point player, uint8_t box_id, uint8_t target_id,
                                MacroAction& action, SokobanLevel& after_level, point& after_player,
                                int& push_path_cost) const;
};

extern MacroPlanner macro_planner;
