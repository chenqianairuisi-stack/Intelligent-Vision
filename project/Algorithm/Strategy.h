/*
Strategy 模块业务逻辑说明

模块定位：
Strategy 是规划链路里的“炸弹战略层”。它不负责输出最终逐步行走路径，也不直接完成推箱子的完整求解；
它负责从当前地图快照中判断哪些墙值得用炸弹打开，并把这些选择整理成BombTask 序列，后续进行融合与执行。

核心目标：
1. 找出阻断箱子到目标点通路的关键墙体。
2. 在有限炸弹数量内选择收益最高的爆破顺序。
3. 尽量优先返回可直接执行的炸弹任务。
4. 当炸弹路线被箱子挡住时，尝试生成少量前置推箱清障任务，使候选炸弹任务变成真实可执行任务。

业务流程：
    1. evaluate_and_assign_bombs 是对外入口。调用方传入 SokobanLevel 后，Strategy 先缓存地图，
然后从空炸弹序列开始做 DFS 搜索。
    2. DFS 每一层代表“已经使用若干颗炸弹后的地图状态”。模块会先评估当前状态中箱子到目标点
的可达性、剩余死锁数量和综合距离成本，再枚举下一颗炸弹可以炸开的墙体。
    3. 枚举候选墙体时，Strategy 会评估墙体是否打开新区域、是否靠近孤立箱子/目标点、是否改善
箱子到目标点的连通性，以及推动炸弹到该墙体的代价。候选按收益排序，只展开高分分支，
以控制单片机上的搜索时间。
    4. 每选择一个候选炸弹任务后，模块会模拟爆炸效果，更新地图并继续递归。全局最优结果按
“剩余死锁更少优先，其次净收益更高优先”更新。
    5. 快速 DFS 失败、仍有死锁，或结果任务不能直接执行时，Strategy 会启用软障碍/动态回退：
先把箱子视作带惩罚的软障碍寻找拓扑上可能的炸弹路线，再用 local_clear_bomb_route 验证
是否能通过少量真实推箱动作清出路线。
    6. materialize_bomb_task 用于把策略层的候选 BombTask 补全为可执行任务。若炸弹到墙体之间
需要先移动箱子，它会把这些前置动作写入 BombTask::box_pushes。

阶段差异：
- PHASE1_ANY：语义绑定尚未完成，任意箱子可以匹配任意目标点。评估时用箱子-目标集合匹配
    来衡量地图整体可解性，重点是打开“让更多箱子能到更多目标”的通路。
- PHASE2_SPECIFIC：箱子和目标点已经绑定。评估时必须检查每个箱子能否到达自己的专属目标，
    对定向不可达的情况施加更高惩罚，适合作为最终 Sokoban 求解前的炸弹补救策略。

输出约定：
- 返回的 StaticArray<BombTask, MAX_BOMBS> 是建议炸弹序列，可能为空。
- BombTask::is_essential 表示该序列是否已经把当前评估中的死锁全部解除。
- BombTask::net_profit 是 Strategy 的综合评分，供上层调度比较任务价值。
- BombTask::box_pushes 非空时，表示执行炸弹前必须先完成这些推箱让路动作。

实现约束：
- 核心搜索和缓存数组面向 RT1064/单片机移植，避免动态内存分配。
- DFS、距离场和局部清障都使用固定容量 StaticArray 与静态缓存，调整 MAX_* 常量时需要同步关注 RAM/DTCM 占用。
*/

#pragma once

#include "PlanningCommon.h"

using namespace SystemConfig;

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

struct DFSResult {
    StaticArray<BombTask, MAX_BOMBS> tasks;
    int deadlocks_remaining;  // 执行 tasks 后仍未解除的死锁数量，越少越好
    int net_profit;           // 综合收益分数，越高越好
};

// ============================================================================
// 炸弹战略规划器
// ============================================================================
class StrategicPlanner {
public:
    StrategicPlanner() = default;

    void set_phase2_dynamic_fallback(bool enabled) { force_phase2_dynamic = enabled; }

    template <GameMode Mode>
    StaticArray<BombTask, MAX_BOMBS> evaluate_and_assign_bombs(const SokobanLevel& level);

    bool materialize_bomb_task(const SokobanLevel& level, point player_start, const BombTask& task, BombTask& out_task, bool phase2_specific = false);

private:
    // --- 当前规划上下文 ---
    SokobanLevel cached_level;
    bool force_phase2_dynamic = false;
    bool phase1_soft_bomb_eval = false;
    bool phase2_soft_bomb_eval = false;

    // --- DFS 策略搜索 ---
    template <GameMode Mode, bool Dynamic>
    void dfs_bomb_sequence(
        const SokobanLevel& current_lvl,
        point player_start,
        StaticArray<BombTask, MAX_BOMBS> current_seq,
        int cost_so_far,
        int depth,
        DFSResult& best_res);

    // --- 推物体距离场与可执行性验证 ---
    void fast_push_bfs(const SokobanLevel& lvl, point start_obj, point player_start, bool is_bomb, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH], bool soft_boxes = false);
    void macro_soft_dijkstra(const SokobanLevel& lvl, point start_obj, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]);
    bool are_fast_bomb_tasks_directly_executable(const SokobanLevel& level, const StaticArray<BombTask, MAX_BOMBS>& tasks);

    // --- 轻量局部清障回退 ---
    bool local_clear_bomb_route(
        const SokobanLevel& start_lvl,
        int bomb_idx,
        point target_wall,
        bool phase2_specific,
        SokobanLevel& out_lvl,
        int& out_cost,
        StaticArray<BoxPushTask, 8>& out_box_pushes);
};

extern StrategicPlanner strategic_planner;
