/// \file MacroPlanner.h
/// \brief 巡图宏动作调度器
///
/// \details
/// 定义在线规划上下文和语义知识状态，声明参考序列推进、观测同步、
/// 宏动作选择及向最终 Sokoban 阶段交接所需的接口

#pragma once

#include "PlanningCommon.h"

/// \brief MacroPlanner 维护的在线语义知识状态
///
/// observed_mask 的低 box_count 位表示箱子，后续 target_count 位表示目标点
struct KnowledgeState {
    uint32_t observed_mask = 0;              // 已观测实体 bitmask
    int8_t inferred_semantics[MAX_ENTITIES]; // 推断后的实体语义，-1 表示仍未知
    bool semantics_ready = false;            // 是否已经得到完整实体语义
    uint8_t bound_target[MAX_BOXES];         // 已确定绑定时的目标点编号
    bool is_bound[MAX_BOXES];                // 箱子是否已有唯一同语义目标
};

/// \brief 单次在线决策所需的动态上下文
struct MacroPlanContext {
    SokobanLevel level;                                      // 当前地图快照
    point player;                                            // 当前小车位置
    float yaw = -1.0f;                                       // 当前观测朝向，负数表示未知
    const StaticArray<BombTask, MAX_BOMBS>* bomb_tasks = nullptr; // Strategy 输出的炸弹任务集合
};

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

    // 最近一次规划失败是否需要 Core2 按当前地图重新生成巡图序列
    bool needs_exploration_replan() const { return exploration_replan_needed; }

    // 返回已提交识别的实体集合，供重新请求 Exploration 时跳过已观测实体
    uint32_t observed_mask() const { return knowledge_state.observed_mask; }

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

    /// \brief 返回当前语义知识状态供现有执行层只读访问
    const KnowledgeState& knowledge() const { return knowledge_state; }

    /// \brief 输出 Macro 推断出的实体语义标签
    /// \param out_labels 输出数组，前 box_count 项为箱子，后 target_count 项为目标点
    /// \return 语义推断完整时返回 true
    bool fill_semantic_labels(int8_t* out_labels) const;

    /// \brief 将已推断语义写回逻辑地图
    bool apply_semantics_to_level(SokobanLevel& level) const;

private:
    // ------------------------------------------------------------------------
    // 内部类型
    // ------------------------------------------------------------------------

    enum class SlotKind : uint8_t {
        NONE,
        REFERENCE,
        COMPLETION_PUSH,
    };

    enum class SemanticInferenceStatus : uint8_t {
        INSUFFICIENT,
        INFERRED,
        INVALID
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
    bool exploration_replan_needed = false;

    int8_t semantic_labels[MAX_ENTITIES];  // 语义标签缓存，-1 表示未知
    uint8_t box_count = 0;
    uint8_t target_count = 0;

    // ------------------------------------------------------------------------
    // 语义知识维护
    // ------------------------------------------------------------------------

    uint16_t default_candidate_mask() const;
    SemanticInferenceStatus infer_semantics(
        const SokobanLevel& level,
        const int8_t* labels,
        int8_t* out_labels) const;
    bool apply_inferred_semantics(const SokobanLevel& level, const int8_t* inferred_labels);
    bool build_semantic_matched_ids(const SokobanLevel& level, const int8_t* labels, uint8_t* out_matched_ids) const;
    bool has_required_observations(const SokobanLevel& level) const;
    bool reference_sequence_done(const SokobanLevel& level) const;
    bool semantics_ready() const;

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
