#include "tuning_config.h"
#include "Tracker.h"
#include "RobotState.h"
#include "PoseEstimate.h"
#include <cmath>

using namespace SystemConfig;

namespace Algorithm::Tracker {
namespace {
    [[gnu::always_inline]] inline int sign_delta(int delta) {
        return (delta > 0) - (delta < 0);
    }

    [[gnu::always_inline]] inline bool same_direction(point a, point b, point c) {
        // 只比较方向符号，连续走多格仍然算同向直线
        return sign_delta(b.x - a.x) == sign_delta(c.x - b.x) &&
                sign_delta(b.y - a.y) == sign_delta(c.y - b.y);
    }

    [[gnu::always_inline]] inline int manhattan(point a, point b) {
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;
        return dx + dy;
    }

    [[gnu::always_inline]] inline bool valid_grid(point p) {
        return p.x >= 0 && p.x < MAP_MAX_WIDTH && p.y >= 0 && p.y < MAP_MAX_HEIGHT;
    }

    [[gnu::always_inline]] inline Point2D grid_to_physical(point p) {
        return {p.x * GRID_SIZE_CM + MAP_OFFSET_X, p.y * GRID_SIZE_CM + MAP_OFFSET_Y};
    }

    [[gnu::always_inline]] inline Point2D current_pose_point() {
        const auto& pose = App::g_state.physical.pose;
        return {pose.x, pose.y};
    }

    /// \brief 判断直线段是否可以提前切到下一段
    /// \param segment_start 当前直线段起点
    /// \param target 当前直线段终点
    ///
    /// \details
    /// 只在主运动轴接近终点且横向误差较小时切换，减少拐角停车又避免斜切过早
    ///
    [[gnu::always_inline]] inline bool check_corner_switch(Point2D segment_start, Point2D target) {
        const auto& pose = App::g_state.physical.pose;
        float sx = target.x - segment_start.x;
        float sy = target.y - segment_start.y;
        float abs_sx = std::abs(sx);
        float abs_sy = std::abs(sy);

        if (abs_sx >= abs_sy && abs_sx > 0.001f) {
            float dir = sx >= 0.0f ? 1.0f : -1.0f;
            float remaining = (target.x - pose.x) * dir;
            float lateral_err = std::abs(target.y - pose.y);
            return remaining <= tune.tracker.corner_switch_window &&
                    lateral_err <= tune.tracker.corner_line_tolerance;
        }

        if (abs_sy > 0.001f) {
            float dir = sy >= 0.0f ? 1.0f : -1.0f;
            float remaining = (target.y - pose.y) * dir;
            float lateral_err = std::abs(target.x - pose.x);
            return remaining <= tune.tracker.corner_switch_window &&
                    lateral_err <= tune.tracker.corner_line_tolerance;
        }

        return false;
    }

    // 没有可信逻辑起点时，沿用原路径首点作为压缩起点
    void compress_without_start(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                                StaticArray<point, MAX_PATH_LENGTH>& out_path) {
        // 没有可信逻辑起点时，沿用原路径首点作为压缩起点
        out_path.push_back(raw_path[0]);
        for (int i = 1; i < raw_path.size() - 1; ++i) {
            if (!same_direction(raw_path[i - 1], raw_path[i], raw_path[i + 1])) {
                out_path.push_back(raw_path[i]);
            }
        }
        if (raw_path.size() > 1) {
            out_path.push_back(raw_path.back());
        }
    }
}


/// \brief 载入并压缩规划路径，生成底盘可追踪的物理航点
/// \param raw_path 规划层输出的原始网格路径
/// \param has_start_grid 是否提供真实逻辑起点
/// \param start_grid 当前逻辑地图上的小车格点
///
/// \details
/// 路径压缩只保留拐点和终点，并在新路径开始时重置视觉校正状态
/// has_start_grid 为 true 时，会把当前位置也纳入第一段共线判断
///
static void load_path_impl(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                            bool has_start_grid,
                            point start_grid) {
    auto& plan = App::g_state.planning;
    auto& ctrl = App::g_state.control;

    plan.grid_path.clear();
    plan.physical_path.clear();
    ctrl.tracker_state = TrackerState::FINISHED;
    plan.vision_segment_start = current_pose_point();
    plan.vision_correction_done = false;
    App::g_state.vision.art1_pose_updated = false;

    if (raw_path.size() == 0) return;

    bool can_use_start = false;
    bool raw_includes_start = false;
    if (has_start_grid && valid_grid(start_grid)) {
        // 路径有时从当前位置开始，有时从下一格开始，两种都允许用逻辑起点做压缩参考
        raw_includes_start = (raw_path[0] == start_grid);
        can_use_start = raw_includes_start || manhattan(start_grid, raw_path[0]) == 1;
    }

    if (can_use_start) {
        point prev = start_grid;
        int first_idx = raw_includes_start ? 1 : 0;

        // 用真实逻辑起点参与共线判断，避免第一段方向判断偏掉
        for (int i = first_idx; i < raw_path.size(); ++i) {
            bool is_last = (i == raw_path.size() - 1);
            if (is_last || !same_direction(prev, raw_path[i], raw_path[i + 1])) {
                plan.grid_path.push_back(raw_path[i]);
            }
            prev = raw_path[i];
        }
    } else {
        compress_without_start(raw_path, plan.grid_path);
    }

    if (plan.grid_path.empty()) {
        return;
    }

    for (int i = 0; i < plan.grid_path.size(); ++i) {
        plan.physical_path.push_back(grid_to_physical(plan.grid_path[i]));
    }

    // 视觉校正需要知道当前直线段的真实起点，用来区分前进轴和横向轴
    plan.vision_segment_start = can_use_start ? grid_to_physical(start_grid) : current_pose_point();
    plan.current_wp_idx = 0;
    ctrl.tracker_state = TrackerState::TRACKING;
    ctrl.mode = ControlMode::AUTO_TRACKING;
}

// 载入网格路径并启动自动跟踪 [无逻辑起点时会以 raw_path 首点作为压缩起点，兼容旧调用]
void load_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path) {
    load_path_impl(raw_path, false, {0, 0});
}

// 载入网格路径并使用真实逻辑起点辅助压缩 [start_grid 参与第一段方向判断，可避免路径首点缺失导致的拐点压缩错误]
void load_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path, point start_grid) {
    load_path_impl(raw_path, true, start_grid);
}


/// \brief 更新当前跟踪目标，供底盘控制周期调用
///
/// \details
/// 该函数负责航点切换、拐角提前切换和每段一次的视觉轴向校正
///
__attribute__((section(".ramfunc"))) void update_target() {
    auto& plan = App::g_state.planning;
    auto& ctrl = App::g_state.control;

    if (ctrl.tracker_state != TrackerState::TRACKING) return;

    bool is_last_point = (plan.current_wp_idx == plan.physical_path.size() - 1);
    Point2D target_phys = plan.physical_path[plan.current_wp_idx];
    float current_radius = is_last_point ? tune.tracker.reach_radius_min : tune.tracker.reach_radius;
    // 非终点航点允许提前切换，终点必须按更小半径进入并停稳
    bool arrived = is_last_point ?
        check_arrival(target_phys, current_radius) :
        (check_arrival(target_phys, current_radius) || check_corner_switch(plan.vision_segment_start, target_phys));

    if (arrived) {
        if (!plan.vision_correction_done) {
            // 每个直线段只做一次视觉轴向修正，避免同一帧视觉数据反复覆盖里程计
            Subsystem::PoseEstimator::apply_vision_axis_correction(plan.vision_segment_start, target_phys);
            plan.vision_correction_done = true;
        }

        if (!is_last_point) {
            // 切到下一段后重新允许视觉修正，并丢弃上一段残留的视觉更新标志
            plan.vision_segment_start = target_phys;
            plan.vision_correction_done = false;
            App::g_state.vision.art1_pose_updated = false;
            plan.current_wp_idx++;
            target_phys = plan.physical_path[plan.current_wp_idx];
        } else if (App::g_state.physical.is_stopped) {
            ctrl.tracker_state = TrackerState::FINISHED;
        }
    }

    ctrl.current_target.x = target_phys.x;
    ctrl.current_target.y = target_phys.y;
}


/// \brief 直接追踪一个物理目标点
/// \param target 目标位姿，坐标单位为 cm，yaw 单位为度
///
/// \details
/// 该接口会清空路径跟踪状态，进入 POINT_TRACKING，主要供遥测调试和手动移动使用
///
void track_point(const Pose2D& target) {
    auto& plan = App::g_state.planning;
    auto& ctrl = App::g_state.control;

    plan.grid_path.clear();
    plan.physical_path.clear();
    plan.vision_segment_start = current_pose_point();
    plan.vision_correction_done = false;
    App::g_state.vision.art1_pose_updated = false;

    ctrl.current_target = target;
    ctrl.mode = ControlMode::POINT_TRACKING;
}

// 检查是否到达当前目标点
bool check_arrival(Point2D target, float radius) {
    auto& current_pos = App::g_state.physical.pose;

    float dx = target.x - current_pos.x;
    float dy = target.y - current_pos.y;
    float dist_sq = dx * dx + dy * dy;

    return dist_sq <= radius * radius;
}

} // namespace Algorithm::Tracker
