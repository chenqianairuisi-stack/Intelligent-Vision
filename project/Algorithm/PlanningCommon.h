#pragma once

#include "system_config.h"

using namespace SystemConfig;

// ============================================================================
// 跨模块任务数据结构
// ============================================================================

/// \brief 推箱让路子任务
struct BoxPushTask {
    point box_start;      // 箱子的当前坐标
    point box_target;     // 箱子需要被推到的目标坐标
};

/// \brief 炸弹任务的轻量意图描述
struct BombIntent {
    point bomb_start;       // 炸弹初始坐标
    point target_wall;      // 目标墙体坐标
    bool is_essential = false; // 是否是关键炸弹
    int net_profit = 0;     // 策略层收益分数
};

/// \brief 可执行炸弹宏任务
struct BombTask {
    point bomb_start;     // 要推动的炸弹初始坐标
    point target_wall;    // 要炸开的墙体坐标
    bool is_essential;    // 是否是解除死锁所必需的任务
    int net_profit;       // 策略层评估出的收益分数

    StaticArray<BoxPushTask, 8> box_pushes; // 执行炸弹前需要完成的推箱让路任务
};

/// \brief 已展开到宏动作层的推箱动作
struct BoxPushAction {
    point box_start;      // 箱子起点
    point box_target;     // 箱子终点
    uint8_t box_id = 255; // 箱子编号，255 表示未绑定
};

/// \brief 已展开到宏动作层的推炸弹动作
struct BombPushAction {
    point bomb_start;           // 炸弹起点
    point bomb_target;          // 炸弹终点
    point blast_wall;           // 爆破墙体
    bool detonates = true;      // 是否在本动作末尾引爆
    bool is_essential = false;  // 是否是关键炸弹
    int net_profit = 0;         // 策略层收益分数
};

// 从完整任务提取轻量炸弹意图
inline BombIntent make_bomb_intent(const BombTask& task) {
    return {task.bomb_start, task.target_wall, task.is_essential, task.net_profit};
}

// 从轻量意图生成不含推箱前置任务的炸弹任务
inline BombTask make_bomb_task(const BombIntent& intent) {
    BombTask task;
    task.bomb_start = intent.bomb_start;
    task.target_wall = intent.target_wall;
    task.is_essential = intent.is_essential;
    task.net_profit = intent.net_profit;
    task.box_pushes.clear();
    return task;
}

// 从推箱宏动作还原推箱任务
inline BoxPushTask make_box_push_task(const BoxPushAction& action) {
    return {action.box_start, action.box_target};
}

// 从推箱任务生成推箱宏动作
inline BoxPushAction make_box_push_action(const BoxPushTask& task, uint8_t box_id = 255) {
    return {task.box_start, task.box_target, box_id};
}

// 从炸弹任务生成终止引爆型推炸弹动作
inline BombPushAction make_terminal_bomb_push_action(const BombTask& task) {
    return {
        task.bomb_start,
        task.target_wall,
        task.target_wall,
        true,
        task.is_essential,
        task.net_profit
    };
}

// 从推炸弹动作还原炸弹任务
inline BombTask make_bomb_task(const BombPushAction& action) {
    BombTask task;
    task.bomb_start = action.bomb_start;
    task.target_wall = action.detonates ? action.blast_wall : action.bomb_target;
    task.is_essential = action.is_essential;
    task.net_profit = action.net_profit;
    task.box_pushes.clear();
    return task;
}

// ============================================================================
// planning 公共底层工具
// ============================================================================
namespace PlanningCommon {

// --- 地图与实体查询 ---
bool in_bounds(point p);
bool is_inner_map_cell(point p);
bool is_blastable_wall(const SokobanLevel& lvl, point p);
bool has_box(const SokobanLevel& lvl, point p);
bool has_bomb(const SokobanLevel& lvl, point p, int ignored_bomb = -1);
bool has_entity(const SokobanLevel& lvl, int x, int y, int ignored_bomb = -1);
bool is_obstacle(const SokobanLevel& lvl, point p, point ignored_obj = {-1, -1});

// --- 地图状态更新 ---
void apply_box_push_task_effect(SokobanLevel& lvl, const BoxPushTask& task);
void apply_box_push_action_effect(SokobanLevel& lvl, const BoxPushAction& action);
void apply_blast_effect(SokobanLevel& lvl, point target_wall);
void apply_bomb_task_effect(SokobanLevel& lvl, const BombTask& task);
void apply_bomb_push_action_effect(SokobanLevel& lvl, const BombPushAction& action);

/// \brief 根据已执行的推炸弹动作同步剩余炸弹任务列表
/// \param tasks 需要原地更新的剩余炸弹任务列表
/// \param action 已完成执行的推炸弹宏动作
void sync_bomb_tasks_after_push(StaticArray<BombTask, MAX_BOMBS>& tasks, const BombPushAction& action);

/// \brief 应用真实执行完成后的推炸弹结果
/// \param lvl 需要原地更新的真实逻辑地图
/// \param tasks 需要同步的剩余炸弹任务列表
/// \param action 已完成执行的推炸弹宏动作
void apply_executed_bomb_push_result(SokobanLevel& lvl,
                                    StaticArray<BombTask, MAX_BOMBS>& tasks,
                                    const BombPushAction& action);

// --- 普通网格寻路 ---
uint16_t bfs_shortest_path(const SokobanLevel& lvl, point start, point end);
bool get_grid_path(const SokobanLevel& lvl, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path);

// --- 麦轮底盘时间代价工具 ---
// 说明：
// - 原 bfs_shortest_path / get_grid_path 仍只负责拓扑可达性和传统最短步数路径
// - 下列接口只用于第一阶段评分、排序和普通巡图移动，避免把拐点代价误注入推箱可达性判断
// - 时间模型参数在 PlanningCommon.cpp 的 MotionCost 命名空间中调整，全部使用 inline constexpr
uint16_t path_time_cost(point start, const StaticArray<point, MAX_PATH_LENGTH>& path);
uint16_t shortest_grid_time_cost(const SokobanLevel& lvl, point start, point end);
bool get_grid_time_path(const SokobanLevel& lvl, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path);
void build_grid_time_map(const SokobanLevel& lvl, point start, uint16_t out_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]);
uint16_t yaw_turn_time_cost(float from_yaw, float to_yaw);

bool can_player_reach(const SokobanLevel& lvl, point start_pos, point target_pos, point ignored_obj, point extra_obs);
void calc_player_reach(const SokobanLevel& lvl, point start_pos, point ignored_obj, point extra_obs, bool out_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]);
bool is_static_deadlock_cell(const SokobanLevel& lvl, point p);
bool is_2x2_box_deadlock(const SokobanLevel& lvl, point p);
bool box_has_candidate_target_path(const SokobanLevel& lvl, uint8_t box_id, uint16_t candidate_mask);
bool is_box_position_safe(const SokobanLevel& lvl, uint8_t box_id, uint16_t candidate_mask);

// --- 推物体路径生成 ---
bool append_box_push_path(SokobanLevel& lvl, point& player_pos, const BoxPushTask& task, StaticArray<point, MAX_PATH_LENGTH>& out_path);
bool get_direct_bomb_push_path_cost(const SokobanLevel& lvl, point player_start, const BombTask& task, uint16_t& out_cost, point& out_final_player);
bool get_bomb_push_path(const SokobanLevel& lvl, point player_start, const BombTask& task, StaticArray<point, MAX_PATH_LENGTH>& out_path);

}
