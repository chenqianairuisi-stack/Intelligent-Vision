/// \file Exploration.h
/// \brief 巡图观测规划器，生成候选观测位姿与最优巡图序列
///
/// \details
/// 声明地图快照加载、候选视角构建、炸弹时间线优化和巡图搜索接口
/// 规划结果仅作为参考序列，真实地图状态由 GameManager 和 MacroPlanner 维护

#pragma once

#include "PlanningCommon.h"

/// \brief Core2 巡图观测规划器
/// \details 核心职责：
///          1. 为地图中每个实体预生成候选观测位姿（build_entity_views）
///          2. 使用带剪枝的 DFS 搜索最优巡图宏动作序列（plan_optimal_patrol）
///          3. 优化炸弹引爆时间线（optimize_bomb_timeline）
///
/// 搜索空间：DFS 深度受约束，使用换位表 (transposition table) 和 LRU 网格时间缓存
/// 加速重复状态检测；用贪心上界作为剪枝种子。
///
/// 内存注意：entity_views 为 32 实体 × 40 位姿 × (sizeof(ViewPose) ≈ 56 bytes)，
///          约 70KB，由 OCRAM_BSS 宏放入 OCRAM
///          输入仅为 Core1 发送的地图快照，输出仅作为参考宏序列回复
class Exploration {
public:
    Exploration() = default;

    // 缓存当前地图快照并预生成所有候选观测位姿，下一请求会覆盖本地快照
    void load_level(const SokobanLevel& level);

    /// \brief 返回当前缓存地图中的实体数量（箱子 + 炸弹）
    uint8_t get_entity_count() const { return total_entities; }

    /// \brief 搜索当前地图的巡图宏动作序列
    /// \param start_pos 巡图起点（栅格坐标）
    /// \param bomb_tasks 策略层给出的炸弹任务序列（按执行顺序排列）
    /// \param start_yaw 起始朝向（角度），负值表示忽略初始转向代价
    /// \param start_mask 已经观测到的实体掩码（默认为 0）
    /// \return 可执行的巡图宏动作序列，最多 32 个动作，由 Core1 决定是否提交执行
    StaticArray<MacroAction, 32> plan_optimal_patrol(
        point start_pos,
        const StaticArray<BombTask, MAX_BOMBS>& bomb_tasks,
        float start_yaw = -1.0f,
        uint32_t start_mask = 0
    );

    /// \brief 构建当前地图所有候选观测位姿（调试用）
    StaticArray<ViewPose, MAX_OBS_POINTS> build_current_views(const SokobanLevel& level);

private:
    StaticArray<ViewPose, 40> entity_views[32]; ///< 每个实体对应的候选观测位姿
    uint8_t total_entities;                     ///< 当前巡图实体总数
    SokobanLevel cached_level;                  ///< 最近一次加载的地图快照

    /// \brief 为所有实体预生成候选观测位姿
    void build_entity_views(const SokobanLevel* multi_maps, int B);
    /// \brief 按可执行代价重排炸弹任务时间线
    StaticArray<BombTask, MAX_BOMBS> optimize_bomb_timeline(const SokobanLevel& initial_lvl, point start_pos, const StaticArray<BombTask, MAX_BOMBS>& raw_tasks);
    /// \brief 将宏动作中的炸弹效果应用到地图（推演用，不修改真实逻辑地图）
    void apply_macro_bomb_effect(SokobanLevel& lvl, const BombTask& task) const;
};

extern Exploration patrol_planner;
