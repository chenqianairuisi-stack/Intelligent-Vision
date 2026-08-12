/// \file planning_common.h
/// \brief 麦克纳姆时间代价寻路与推箱/炸弹路径展开
#pragma once

#include "system_config.h"

using namespace SystemConfig;

// ============================================================================
// planning 公共底层工具
// ============================================================================
namespace PlanningCommon {

// 规划器与仿真统计共用的动作代价模型
namespace MotionCostConfig {
inline constexpr uint16_t MOVE_STEP = 1u;       // 普通移动每格代价
inline constexpr uint16_t STOP_NODE = 2u;       // 停止节点代价（包括拐点、反向和观测点）
inline constexpr uint16_t OBSERVE_EXTRA = 1u;   // 完成一次识别的额外代价
inline constexpr uint16_t TURN_EXTRA = 3u;      // 原地对齐观测朝向的额外代价
inline constexpr uint16_t OBSERVE_ACTION = STOP_NODE + OBSERVE_EXTRA;
} // namespace MotionCostConfig

// 斜向移动配置
namespace ObserveRouteConfig {
inline constexpr bool ENABLE_OBSERVE_ROUTE_OPTIMIZATION = true;   // 启用观测路径斜向优化
inline constexpr bool ENABLE_OBSERVE_ENDPOINT_ADJUST = true;      // 启用观测路径终点微调
inline constexpr int ENDPOINT_ADJUST_RADIUS = 1;                  // 邻域半径，1 表示 3x3，2 表示 5x5
} // namespace ObserveRouteConfig

// 箱子额外观测位配置
namespace BoxObservationConfig {
inline constexpr bool ENABLE_F2 = true;   // F2 正前方
} // namespace BoxObservationConfig

// 目标点额外观测位配置
namespace TargetObservationConfig {
// 单个目标观测槽位
inline constexpr bool ENABLE_F2 = true;             // F2 正前方
inline constexpr bool ENABLE_F2_DIAGONAL = true;    // F2 左右斜角
inline constexpr bool ENABLE_F3 = true;             // F3 正前方

// 联合观测槽位
inline constexpr bool ENABLE_JOINT_F2_DIAGONAL = true;   // F2 左右斜角联合观测
inline constexpr bool ENABLE_JOINT_F1_DIAGONAL = true;   // F1 左右斜角联合观测

// 定义各个观测槽位的索引
inline constexpr int TARGET_SLOT_F1 = 0;         // F1 正前1格槽位
inline constexpr int TARGET_SLOT_F2_CORE = 1;    // F2 正前2格槽位
inline constexpr int TARGET_SLOT_F2_LEFT = 2;    // F2 左侧斜角槽位
inline constexpr int TARGET_SLOT_F2_RIGHT = 3;   // F2 右侧斜角槽位
inline constexpr int TARGET_SLOT_F3 = 4;         // F3 正前3格槽位
inline constexpr int TARGET_SLOT_F1_LEFT = 5;    // F1 左侧槽位
inline constexpr int TARGET_SLOT_F1_RIGHT = 6;   // F1 右侧槽位

inline constexpr uint8_t MAX_TARGETS_PER_OBSERVATION = 3u;   // 一次联合观测最多识别 3 个目标
inline constexpr int TARGET_BASE_SLOT_COUNT = 5;             // 基础可见性槽位数量，含 F2 左右斜角
inline constexpr int TARGET_OBSERVE_SLOT_COUNT = 7;          // 总共记录 7 种相对位置
inline constexpr uint16_t TARGET_OPTIMAL_PENALTY = 1u;       // F2 比 F1 多一格观测代价
inline constexpr uint16_t TARGET_FAR_PENALTY = 2u;           // F3 比 F1 多两格观测代价
inline constexpr uint16_t TARGET_DIAGONAL_PENALTY = 1u;      // 斜视略低于同距离正视
} // namespace TargetObservationConfig

/// \brief 单个目标观测几何槽位的可见实体与代价
///
/// slots 前五项是单目标候选，后两项是 F1 左右斜角，仅用于联合观测
struct TargetObserveSlots {
    uint32_t mask[TargetObservationConfig::TARGET_OBSERVE_SLOT_COUNT] = {};
    uint16_t penalty[TargetObservationConfig::TARGET_OBSERVE_SLOT_COUNT] = {};
};


// --- 地图与实体查询 ---
bool in_bounds(point p);
bool is_inner_map_cell(point p);
bool is_blastable_wall(const SokobanLevel& lvl, point p);
bool has_box(const SokobanLevel& lvl, point p);
bool has_bomb(const SokobanLevel& lvl, point p, int ignored_bomb = -1);
bool has_entity(const SokobanLevel& lvl, int x, int y, int ignored_bomb = -1);
bool is_obstacle(const SokobanLevel& lvl, point p, point ignored_obj = {-1, -1});

/// \brief 按当前地图重新计算单个观测位姿的可见实体与几何惩罚
/// \param lvl 当前地图
/// \param view_pos 车体观测位置
/// \param target_yaw 车头朝向
/// \param out_mask 输出当前可见实体集合
/// \param out_penalty 输出目标几何惩罚
/// \param out_target_slots 可选输出目标点单槽位可见性，供 Exploration 组合单目标和联合目标动作
bool evaluate_observe_pose(const SokobanLevel& lvl,
                           point view_pos,
                           float target_yaw,
                           uint32_t& out_mask,
                           uint16_t& out_penalty,
                           TargetObserveSlots* out_target_slots = nullptr);

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
// - 时间模型参数统一在 MotionCostConfig 中调整
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

/// \brief 生成支持任意斜率航点的观测移动路径
/// \param lvl 当前地图状态
/// \param start 当前小车格点
/// \param end 固定观测终点
/// \param out_path 输出优化后的航点，不包含起点并包含终点
/// \param initial_dir 进入起点前的 MOVE 下标，负值表示未知
///
/// \details
/// 只有候选路径严格减少内部拐点且总代价更低时才替换原网格路径
/// 同样减少拐点时优先线段更少、斜线段更少的路径，避免连续多段不同斜率
bool get_optimized_observe_path(const SokobanLevel& lvl,
                                point start,
                                point end,
                                StaticArray<point, MAX_PATH_LENGTH>& out_path,
                                int initial_dir = -1);

/// \brief 在原观测点邻域中联合优化终点和移动航点
/// \param lvl 当前地图状态
/// \param start 当前小车格点
/// \param required_mask 新观测点必须完整覆盖的实体集合
/// \param inout_view 输入参考观测位姿，输出实际采用的观测位姿
/// \param out_path 输出到新观测点的优化航点
/// \param initial_dir 进入起点前的 MOVE 下标，负值表示未知
///
/// \details 固定终点优先，仅在原点直斜线受阻时尝试改点，且必须保持合法观测模式并减少拐点
bool optimize_observe_route(const SokobanLevel& lvl,
                            point start,
                            uint32_t required_mask,
                            ViewPose& inout_view,
                            StaticArray<point, MAX_PATH_LENGTH>& out_path,
                            int initial_dir = -1);

/// \brief 计算任意斜率观测航点的近似时间代价
uint16_t observe_route_time_cost(point start,
                                 const StaticArray<point, MAX_PATH_LENGTH>& path,
                                 int initial_dir = -1);

/// \brief 判断航点折线是否穿过指定网格
bool path_crosses_cell(point start,
                       const StaticArray<point, MAX_PATH_LENGTH>& path,
                       point cell);

bool can_player_reach(const SokobanLevel& lvl, point start_pos, point target_pos, point ignored_obj, point extra_obs);
void calc_player_reach(const SokobanLevel& lvl, point start_pos, point ignored_obj, point extra_obs, bool out_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]);
bool is_static_deadlock_cell(const SokobanLevel& lvl, point p);
bool is_2x2_box_deadlock(const SokobanLevel& lvl, point p);
bool box_has_candidate_target_path(const SokobanLevel& lvl, uint8_t box_id, uint16_t candidate_mask);
bool is_box_position_safe(const SokobanLevel& lvl, uint8_t box_id, uint16_t candidate_mask);

// --- 推物体路径生成 ---
bool append_box_push_path(SokobanLevel& lvl, point& player_pos, const BoxPushTask& task, StaticArray<point, MAX_PATH_LENGTH>& out_path);
// 在不改变箱子起点/目标的前提下，优先选择小车绕行代价较低的推箱宏路径
bool append_box_push_optimized_path(SokobanLevel& lvl,
                                    point& player_pos,
                                    const BoxPushTask& task,
                                    StaticArray<point, MAX_PATH_LENGTH>& out_path,
                                    int initial_dir = -1,
                                    int required_final_dir = -1);
bool get_direct_bomb_push_path_cost(const SokobanLevel& lvl, point player_start, const BombTask& task, uint16_t& out_cost, point& out_final_player);
bool get_bomb_push_path(const SokobanLevel& lvl, point player_start, const BombTask& task, StaticArray<point, MAX_PATH_LENGTH>& out_path);

}
