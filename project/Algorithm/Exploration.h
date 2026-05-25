#pragma once

#include "PlanningCommon.h"

/// \brief 单个候选观测位姿及其跨炸弹阶段的可见实体信息
struct ViewPose {
    point    pos;                         // 车辆驻留网格
    float    target_yaw;                  // 观测目标朝向
    uint32_t mask[MAX_BOMBS + 1];         // 每个阶段可观测实体掩码
    uint16_t penalty[MAX_BOMBS + 1];      // 每个阶段的观测位姿惩罚
};

/// \brief 巡图宏动作类型
enum class MacroActionKind : uint8_t {
    OBSERVE,
    PUSH_BOX,
    PUSH_BOMB
};

/// \brief 观测宏动作载荷
struct ObserveAction {
    ViewPose view;            // 实际执行的观测位姿
    uint32_t active_mask = 0; // 本动作贡献的实体掩码
};

/// \brief 巡图阶段输出给上层调度的宏动作
struct MacroAction {
    MacroActionKind kind = MacroActionKind::OBSERVE; // 动作类型
    ObserveAction observe;                           // 观测动作数据
    BoxPushAction box_push;                          // 推箱动作数据
    BombPushAction bomb_push;                        // 推炸弹动作数据
    uint16_t real_cost = 0;                          // 上层可直接使用的真实代价
};

// 构造观测宏动作
inline MacroAction make_observe_macro_action(const ViewPose& view, uint32_t active_mask, uint16_t real_cost = 0) {
    MacroAction action{};
    action.kind = MacroActionKind::OBSERVE;
    action.observe = {view, active_mask};
    action.real_cost = real_cost;
    return action;
}

// 构造推箱宏动作
inline MacroAction make_box_push_macro_action(const BoxPushTask& task, uint8_t box_id, uint16_t real_cost = 0) {
    MacroAction action{};
    action.kind = MacroActionKind::PUSH_BOX;
    action.box_push = make_box_push_action(task, box_id);
    action.real_cost = real_cost;
    return action;
}

// 构造终止引爆型推炸弹宏动作
inline MacroAction make_bomb_push_macro_action(const BombTask& task, uint16_t real_cost = 0) {
    MacroAction action{};
    action.kind = MacroActionKind::PUSH_BOMB;
    action.bomb_push = make_terminal_bomb_push_action(task);
    action.real_cost = real_cost;
    return action;
}

// 构造指定推炸弹载荷的宏动作
inline MacroAction make_bomb_push_macro_action(const BombPushAction& bomb_push, uint16_t real_cost = 0) {
    MacroAction action{};
    action.kind = MacroActionKind::PUSH_BOMB;
    action.bomb_push = bomb_push;
    action.real_cost = real_cost;
    return action;
}

// 从宏动作还原推箱任务
inline BoxPushTask macro_box_task(const MacroAction& action) {
    return make_box_push_task(action.box_push);
}

// 从宏动作还原炸弹任务
inline BombTask macro_bomb_task(const MacroAction& action) {
    return make_bomb_task(action.bomb_push);
}

/// \brief 巡图观测规划器
class Exploration {
public:
    Exploration() = default;

    // 缓存当前地图快照
    void load_level(const SokobanLevel& level);

    // 返回当前缓存地图中的实体数量
    uint8_t get_entity_count() const { return total_entities; }

    /// \brief 搜索当前地图的巡图宏动作序列
    /// \param start_pos 巡图起点
    /// \param bomb_tasks 策略层给出的炸弹任务序列
    /// \param start_yaw 起始朝向，负值表示忽略初始转向代价
    /// \param start_mask 已经观测到的实体掩码
    /// \return 可执行的巡图宏动作序列
    StaticArray<MacroAction, 32> plan_optimal_patrol(
        point start_pos,
        const StaticArray<BombTask, MAX_BOMBS>& bomb_tasks,
        float start_yaw = -1.0f,
        uint32_t start_mask = 0
    );

    // 构建当前地图所有候选观测位姿
    StaticArray<ViewPose, MAX_OBS_POINTS> build_current_views(const SokobanLevel& level);

private:
    StaticArray<ViewPose, 40> entity_views[32]; // 每个实体对应的候选观测位姿
    uint8_t total_entities;                     // 当前巡图实体总数
    SokobanLevel cached_level;                  // 最近一次加载的地图快照

    void build_entity_views(const SokobanLevel* multi_maps, int B);
    StaticArray<BombTask, MAX_BOMBS> optimize_bomb_timeline(const SokobanLevel& initial_lvl, point start_pos, const StaticArray<BombTask, MAX_BOMBS>& raw_tasks);
    void apply_macro_bomb_effect(SokobanLevel& lvl, const BombTask& task) const;
};

extern Exploration patrol_planner;
