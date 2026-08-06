/// \file planning_common.h
/// \brief 麦克纳姆时间代价寻路与推箱/炸弹路径展开
#pragma once

#include "system_config.h"

using namespace SystemConfig;

// ============================================================================
// planning 公共底层工具
// ============================================================================
namespace PlanningCommon {

namespace ObservationConfig {
// 箱子只允许正前方 1 格或 2 格无遮挡直视；两种距离不设置额外优先级
inline constexpr bool ENABLE_FACE_TO_FACE = true;
inline constexpr bool ENABLE_OPTIMAL_DIST = true;

// 目标点允许正前方 1/2/3 格，以及前两格、前三格斜角观测
inline constexpr bool ENABLE_DIAGONAL = true;
inline constexpr bool ENABLE_FAR_DIAGONAL = true;
inline constexpr bool ENABLE_TARGET_FAR_FACE_TO_FACE = true;
inline constexpr uint8_t MAX_TARGETS_PER_OBSERVATION = 3u;
inline constexpr uint16_t TARGET_OPTIMAL_PENALTY = 0u;       // 隔一格目标已满足识别距离，不为靠近制造移动
inline constexpr uint16_t TARGET_FAR_PENALTY = 1u;           // 正前方第 3 格略低于第 1/2 格
inline constexpr uint16_t TARGET_DIAGONAL_PENALTY = 1u;      // 斜视略低于同距离正视
}

// --- 地图与实体查询 ---
bool in_bounds(point p);
bool is_inner_map_cell(point p);
bool is_blastable_wall(const SokobanLevel& lvl, point p);
bool has_box(const SokobanLevel& lvl, point p);
bool has_bomb(const SokobanLevel& lvl, point p, int ignored_bomb = -1);
bool has_entity(const SokobanLevel& lvl, int x, int y, int ignored_bomb = -1);
bool is_obstacle(const SokobanLevel& lvl, point p, point ignored_obj = {-1, -1});

// 按当前地图重新计算单个观测位姿的可见实体与平均惩罚。
// 箱子候选只返回一个箱子；目标点候选最多返回三个并检查真实栅格射线路径无遮挡。
bool evaluate_observe_pose(const SokobanLevel& lvl,
                           point view_pos,
                           float target_yaw,
                           uint32_t& out_mask,
                           uint16_t& out_penalty);

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
// initial_dir 为进入 start 前的 MOVE 下标；-1 表示没有历史方向，不计首步拐点。
uint16_t path_time_cost(point start,
                        const StaticArray<point, MAX_PATH_LENGTH>& path,
                        int initial_dir = -1);
uint16_t shortest_grid_time_cost(const SokobanLevel& lvl,
                                 point start,
                                 point end,
                                 int initial_dir = -1);
bool get_grid_time_path(const SokobanLevel& lvl,
                        point start,
                        point end,
                        StaticArray<point, MAX_PATH_LENGTH>& out_path,
                        int initial_dir = -1);
void build_grid_time_map(const SokobanLevel& lvl,
                         point start,
                         uint16_t out_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
                         int initial_dir = -1,
                         uint8_t out_final_dir[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = nullptr);
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
