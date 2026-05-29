#include "tuning_config.h"
#include "Tracker.h"
#include "RobotState.h"
#include "PoseEstimate.h"
#include "CoreScheduler.h"
#include "Vision.h"
#include <algorithm>
#include <cmath>

using namespace SystemConfig;

namespace Algorithm::Tracker {
namespace {
    constexpr bool TRACKING_VISION_ASSIST_ENABLED = true;
    constexpr float DEFAULT_BOX_PUSH_FINAL_PRESS_CM = 0.2f;
    constexpr float MIN_BOX_PUSH_FINAL_PRESS_CM = 0.0f;
    constexpr float MAX_BOX_PUSH_FINAL_PRESS_CM = 1.0f;
    constexpr float LONG_SEGMENT_FINAL_CHECK_CM = 60.0f;
    constexpr float FINAL_COORD_CHECK_TOLERANCE_CM = 0.8f;
    constexpr float FINAL_COORD_CHECK_REJECT_CM = 25.0f;
    constexpr float VISION_ASSIST_TARGET_FREEZE_RADIUS_CM = 0.5f;
    constexpr uint32_t FINAL_COORD_CHECK_TIMEOUT_MS = 800U;
    constexpr uint32_t FINAL_COORD_CHECK_REQUEST_INTERVAL_MS = 180U;
    constexpr float DEFAULT_WAYPOINT_REACH_RADIUS_CM = 0.3f;
    constexpr float MAX_WAYPOINT_REACH_RADIUS_CM = 1.0f;
    constexpr float DEFAULT_CORNER_SWITCH_WINDOW_CM = 0.7f;
    constexpr float MAX_CORNER_SWITCH_WINDOW_CM = 1.0f;
    constexpr float DEFAULT_CORNER_LINE_TOLERANCE_CM = 1.0f;
    constexpr float MAX_CORNER_LINE_TOLERANCE_CM = 2.0f;
    constexpr float STOP_SETTLE_RELEASE_RADIUS_CM = 1.0f;
    DTCM_DATA float s_box_push_final_press_cm = DEFAULT_BOX_PUSH_FINAL_PRESS_CM;
    DTCM_DATA bool s_final_coord_check_required = false;
    DTCM_DATA bool s_final_coord_check_active = false;
    DTCM_DATA uint32_t s_final_coord_check_start_ms = 0U;
    DTCM_DATA uint32_t s_final_coord_check_last_request_ms = 0U;
    DTCM_DATA uint32_t s_final_coord_check_start_seq = 0U;
    DTCM_DATA bool s_force_vision_assist_current_segment = false;
    DTCM_DATA bool s_stop_settle_active = false;
    DTCM_DATA uint16_t s_stop_settle_wp_idx = 0U;
    DTCM_DATA Point2D s_stop_settle_target = {0.0f, 0.0f};

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

    [[gnu::always_inline]] inline float clamp_float(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    [[gnu::always_inline]] inline bool valid_grid(point p) {
        return p.x >= 0 && p.x < MAP_MAX_WIDTH && p.y >= 0 && p.y < MAP_MAX_HEIGHT;
    }

    [[gnu::always_inline]] inline Point2D grid_to_physical(point p) {
        return {p.x * GRID_SIZE_CM + MAP_OFFSET_X, p.y * GRID_SIZE_CM + MAP_OFFSET_Y};
    }

    [[gnu::always_inline]] inline Point2D grid_delta_to_physical_unit(point delta) {
        if (delta.x > 0) return {1.0f, 0.0f};
        if (delta.x < 0) return {-1.0f, 0.0f};
        if (delta.y > 0) return {0.0f, 1.0f};
        if (delta.y < 0) return {0.0f, -1.0f};
        return {0.0f, 0.0f};
    }

    [[gnu::always_inline]] inline Point2D current_pose_point() {
        const auto& pose = App::g_state.physical.pose;
        return {pose.x, pose.y};
    }

    [[gnu::always_inline]] inline void reset_vision_assist(Point2D segment_start,
                                                           bool force_vision_assist = false) {
        auto& plan = App::g_state.planning;
        plan.vision_segment_start = segment_start;
        plan.vision_last_correction_seq = App::g_state.vision.art1_pose_seq;
        s_force_vision_assist_current_segment = force_vision_assist;
    }

    [[gnu::always_inline]] inline bool has_trackable_segment(Point2D segment_start, Point2D target) {
        float dx = target.x - segment_start.x;
        float dy = target.y - segment_start.y;
        return (dx * dx + dy * dy) >= 1.0f;
    }

    [[gnu::always_inline]] inline bool is_long_vision_segment(Point2D segment_start, Point2D target) {
        float dx = target.x - segment_start.x;
        float dy = target.y - segment_start.y;
        return (dx * dx + dy * dy) > LONG_SEGMENT_FINAL_CHECK_CM * LONG_SEGMENT_FINAL_CHECK_CM;
    }

    [[gnu::always_inline]] inline bool art1_pose_recent(uint32_t now) {
        const auto& vision = App::g_state.vision;
        constexpr uint32_t POSE_RECENT_MS = 300U;
        return vision.art1_pose_seq != 0U &&
               vision.art1_pose_stable_count >= App::ART1_POSE_STABLE_REQUIRED_FRAMES &&
               (now - vision.art1_pose_tick_ms) <= POSE_RECENT_MS;
    }

    [[gnu::always_inline]] inline bool vision_pose_control_allowed() {
        const auto phase = static_cast<uint8_t>(App::g_state.game.phase);
        const auto wait_map = static_cast<uint8_t>(GamePhase::WAIT_FOR_VISION);
        return App::g_state.vision.art1_map_ready && phase > wait_map;
    }

    [[gnu::always_inline]] inline float terminal_reach_radius() {
        return tune.tracker.reach_radius_min;
    }

    [[gnu::always_inline]] inline float waypoint_reach_radius() {
        if (!std::isfinite(tune.tracker.reach_radius) || tune.tracker.reach_radius < 0.0f) {
            return DEFAULT_WAYPOINT_REACH_RADIUS_CM;
        }
        return std::min(tune.tracker.reach_radius, MAX_WAYPOINT_REACH_RADIUS_CM);
    }

    [[gnu::always_inline]] inline float corner_switch_window() {
        if (!std::isfinite(tune.tracker.corner_switch_window) ||
            tune.tracker.corner_switch_window < 0.0f) {
            return DEFAULT_CORNER_SWITCH_WINDOW_CM;
        }
        return std::min(tune.tracker.corner_switch_window, MAX_CORNER_SWITCH_WINDOW_CM);
    }

    [[gnu::always_inline]] inline float corner_line_tolerance() {
        if (!std::isfinite(tune.tracker.corner_line_tolerance) ||
            tune.tracker.corner_line_tolerance < 0.0f) {
            return DEFAULT_CORNER_LINE_TOLERANCE_CM;
        }
        return std::min(tune.tracker.corner_line_tolerance, MAX_CORNER_LINE_TOLERANCE_CM);
    }

    [[gnu::always_inline]] inline float distance_sq(Point2D a, Point2D b) {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        return dx * dx + dy * dy;
    }

    [[gnu::always_inline]] inline bool is_force_stop_waypoint(uint16_t idx) {
        const auto& plan = App::g_state.planning;
        return idx < plan.force_stop_at_wp.size() && plan.force_stop_at_wp[idx] != 0U;
    }

    [[gnu::always_inline]] inline void force_stop_all_waypoints() {
        auto& plan = App::g_state.planning;
        for (int i = 0; i < plan.force_stop_at_wp.size(); ++i) {
            plan.force_stop_at_wp[i] = 1U;
        }
    }

    [[gnu::always_inline]] inline void reset_final_coord_check() {
        s_final_coord_check_active = false;
        s_final_coord_check_start_ms = 0U;
        s_final_coord_check_last_request_ms = 0U;
        s_final_coord_check_start_seq = 0U;
    }

    [[gnu::always_inline]] inline void disable_final_coord_check() {
        s_final_coord_check_required = false;
        reset_final_coord_check();
    }

    [[gnu::always_inline]] inline void clear_stop_settle() {
        s_stop_settle_active = false;
        s_stop_settle_wp_idx = 0U;
        s_stop_settle_target = {0.0f, 0.0f};
    }

    [[gnu::always_inline]] inline void push_unique_grid_waypoint(
        StaticArray<point, MAX_PATH_LENGTH>& path,
        point waypoint) {
        if (path.empty() || path.back() != waypoint) {
            path.push_back(waypoint);
        }
    }

    bool final_coord_check_pending(Point2D target) {
        if (!s_final_coord_check_required || !vision_pose_control_allowed()) {
            reset_final_coord_check();
            return false;
        }

        uint32_t now = Core::Scheduler::get_sys_tick_ms();
        auto& vision = App::g_state.vision;
        auto& pose = App::g_state.physical.pose;

        if (!s_final_coord_check_active) {
            s_final_coord_check_active = true;
            s_final_coord_check_start_ms = now;
            s_final_coord_check_last_request_ms = 0U;
            s_final_coord_check_start_seq = vision.art1_pose_seq;
        }

        if (s_final_coord_check_last_request_ms == 0U ||
            now - s_final_coord_check_last_request_ms >= FINAL_COORD_CHECK_REQUEST_INTERVAL_MS) {
            Subsystem::Vision::schedule_pose_request_ART1();
            s_final_coord_check_last_request_ms = now;
        }

        if (now - s_final_coord_check_start_ms > FINAL_COORD_CHECK_TIMEOUT_MS) {
            s_final_coord_check_required = false;
            reset_final_coord_check();
            return false;
        }

        if (vision.art1_pose_seq == s_final_coord_check_start_seq || !art1_pose_recent(now)) {
            return true;
        }

        const Pose2D vision_pose = vision.art1_pose_buffer[vision.art1_pose_publish_idx];
        if (!std::isfinite(vision_pose.x) || !std::isfinite(vision_pose.y)) {
            return true;
        }

        float err_x = target.x - vision_pose.x;
        float err_y = target.y - vision_pose.y;
        float err_sq = err_x * err_x + err_y * err_y;
        float tol_sq = FINAL_COORD_CHECK_TOLERANCE_CM * FINAL_COORD_CHECK_TOLERANCE_CM;
        float reject_sq = FINAL_COORD_CHECK_REJECT_CM * FINAL_COORD_CHECK_REJECT_CM;
        if (err_sq >= reject_sq) {
            s_final_coord_check_required = false;
            reset_final_coord_check();
            return false;
        }

        if (err_sq > tol_sq) {
            pose.x = vision_pose.x;
            pose.y = vision_pose.y;
            s_final_coord_check_required = false;
            reset_final_coord_check();
            return false;
        }

        s_final_coord_check_required = false;
        reset_final_coord_check();
        return false;
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
        float switch_window = corner_switch_window();
        float line_tolerance = corner_line_tolerance();

        if (abs_sx >= abs_sy && abs_sx > 0.001f) {
            float dir = sx >= 0.0f ? 1.0f : -1.0f;
            float remaining = (target.x - pose.x) * dir;
            float lateral_err = std::abs(target.y - pose.y);
            return remaining <= switch_window && lateral_err <= line_tolerance;
        }

        if (abs_sy > 0.001f) {
            float dir = sy >= 0.0f ? 1.0f : -1.0f;
            float remaining = (target.y - pose.y) * dir;
            float lateral_err = std::abs(target.x - pose.x);
            return remaining <= switch_window && lateral_err <= line_tolerance;
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
                push_unique_grid_waypoint(out_path, raw_path[i]);
            }
        }
        if (raw_path.size() > 1) {
            push_unique_grid_waypoint(out_path, raw_path.back());
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
    plan.force_stop_at_wp.clear();
    disable_final_coord_check();
    clear_stop_settle();
    ctrl.tracker_state = TrackerState::FINISHED;
    reset_vision_assist(current_pose_point());

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
                push_unique_grid_waypoint(plan.grid_path, raw_path[i]);
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
        Point2D phys = grid_to_physical(plan.grid_path[i]);
        plan.physical_path.push_back(phys);
    }

    for (int i = 0; i < plan.physical_path.size(); ++i) {
        plan.force_stop_at_wp.push_back(0U);
    }

    Point2D prev_phys = can_use_start ? grid_to_physical(start_grid) : current_pose_point();
    constexpr float LONG_SEGMENT_FINAL_CHECK_SQ =
        LONG_SEGMENT_FINAL_CHECK_CM * LONG_SEGMENT_FINAL_CHECK_CM;
    for (int i = 0; i < plan.physical_path.size() - 1; ++i) {
        prev_phys = plan.physical_path[i];
    }

    if (distance_sq(prev_phys, plan.physical_path.back()) > LONG_SEGMENT_FINAL_CHECK_SQ) {
        s_final_coord_check_required = true;
    }

    // 视觉校正需要知道当前直线段的真实起点，用来区分前进轴和横向轴
    reset_vision_assist(can_use_start ? grid_to_physical(start_grid) : current_pose_point());
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

void load_box_push_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                        point start_grid,
                        point,
                        point box_target,
                        const SokobanLevel&) {
    load_path_impl(raw_path, true, start_grid);
    force_stop_all_waypoints();

    auto& plan = App::g_state.planning;
    if (plan.physical_path.empty() || raw_path.size() < 2) {
        return;
    }

    point final_car_grid = raw_path.back();
    point push_dir = box_target - final_car_grid;
    Point2D unit = grid_delta_to_physical_unit(push_dir);
    float press = clamp_float(s_box_push_final_press_cm,
                              MIN_BOX_PUSH_FINAL_PRESS_CM,
                              MAX_BOX_PUSH_FINAL_PRESS_CM);

    plan.physical_path[plan.physical_path.size() - 1].x += unit.x * press;
    plan.physical_path[plan.physical_path.size() - 1].y += unit.y * press;

    // Box contact must finish from encoder; a late ART1 correction can re-push.
    disable_final_coord_check();
}

void load_bomb_push_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                         point start_grid,
                         point bomb_target) {
    load_path_impl(raw_path, true, start_grid);
    force_stop_all_waypoints();

    auto& plan = App::g_state.planning;
    if (plan.physical_path.empty() || raw_path.size() == 0) {
        disable_final_coord_check();
        return;
    }

    point final_car_grid = raw_path.back();
    point push_dir = bomb_target - final_car_grid;
    Point2D unit = grid_delta_to_physical_unit(push_dir);
    float press = clamp_float(s_box_push_final_press_cm,
                              MIN_BOX_PUSH_FINAL_PRESS_CM,
                              MAX_BOX_PUSH_FINAL_PRESS_CM);

    plan.physical_path[plan.physical_path.size() - 1].x += unit.x * press;
    plan.physical_path[plan.physical_path.size() - 1].y += unit.y * press;

    // Bomb contact is physical; do not let ART1 pull after blast.
    disable_final_coord_check();
}

void set_box_push_final_press_cm(float press_cm) {
    if (!std::isfinite(press_cm)) {
        return;
    }
    s_box_push_final_press_cm = clamp_float(press_cm,
                                            MIN_BOX_PUSH_FINAL_PRESS_CM,
                                            MAX_BOX_PUSH_FINAL_PRESS_CM);
}

float get_box_push_final_press_cm() {
    return s_box_push_final_press_cm;
}

/// \brief 运动中的视觉辅助：周期请求 ART1，并在速度规划前应用延时补偿后的视觉修正
__attribute__((section(".ramfunc"))) void update_vision_assist(const Point2D& target) {
    if (!TRACKING_VISION_ASSIST_ENABLED) {
        return;
    }

    if (!App::g_state.vision.art1_map_ready) {
        return;
    }

    auto& plan = App::g_state.planning;
    const Point2D segment_start = plan.vision_segment_start;

    if (!has_trackable_segment(segment_start, target)) {
        return;
    }

    const auto& pose = App::g_state.physical.pose;
    float target_dx = target.x - pose.x;
    float target_dy = target.y - pose.y;
    if (target_dx * target_dx + target_dy * target_dy <=
        VISION_ASSIST_TARGET_FREEZE_RADIUS_CM * VISION_ASSIST_TARGET_FREEZE_RADIUS_CM) {
        return;
    }

    // 只在仍有平移运动时参与，避免终点停车后视觉噪声继续拉扯底盘。
    // 靠近终点但尚未停稳时仍允许最后几帧视觉修正，防止估计位姿先进入半径后提前刹停。
    if (App::g_state.physical.is_stopped) {
        return;
    }

    uint32_t now = Core::Scheduler::get_sys_tick_ms();
    bool pose_recent = art1_pose_recent(now);
    if (!pose_recent) {
        return;
    }

    Subsystem::PoseEstimator::apply_vision_axis_correction(
        segment_start,
        target,
        plan.vision_last_correction_seq
    );
}


/// \brief 更新当前跟踪目标，供底盘控制周期调用
///
/// \details
/// 该函数负责航点切换、拐角提前切换和运动中的视觉辅助
///
__attribute__((section(".ramfunc"))) void update_target() {
    auto& plan = App::g_state.planning;
    auto& ctrl = App::g_state.control;

    if (ctrl.tracker_state != TrackerState::TRACKING) return;

    if (plan.physical_path.empty() || plan.current_wp_idx >= plan.physical_path.size()) {
        clear_stop_settle();
        ctrl.tracker_state = TrackerState::FINISHED;
        Point2D hold = current_pose_point();
        ctrl.current_target.x = hold.x;
        ctrl.current_target.y = hold.y;
        ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
        return;
    }

    bool is_last_point = (plan.current_wp_idx == plan.physical_path.size() - 1);
    bool force_stop_wp = is_force_stop_waypoint(plan.current_wp_idx);
    Point2D target_phys = plan.physical_path[plan.current_wp_idx];
    bool must_stop_at_wp = is_last_point || force_stop_wp;

    float current_radius = must_stop_at_wp ?
        terminal_reach_radius() :
        waypoint_reach_radius();
    // 非终点航点允许提前切换，终点必须按更小半径进入并停稳
    bool arrived = false;

    if (must_stop_at_wp) {
        if (s_stop_settle_active && s_stop_settle_wp_idx != plan.current_wp_idx) {
            clear_stop_settle();
        }

        if (s_stop_settle_active) {
            if (!check_arrival(s_stop_settle_target, STOP_SETTLE_RELEASE_RADIUS_CM)) {
                clear_stop_settle();
            } else if (!App::g_state.physical.is_stopped) {
                Point2D hold = current_pose_point();
                ctrl.current_target.x = hold.x;
                ctrl.current_target.y = hold.y;
                ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
                return;
            } else {
                arrived = check_arrival(s_stop_settle_target, current_radius);
                if (!arrived) {
                    clear_stop_settle();
                }
            }
        }

        if (!arrived) {
            update_vision_assist(target_phys);

            if (check_arrival(target_phys, current_radius)) {
                if (!App::g_state.physical.is_stopped) {
                    s_stop_settle_active = true;
                    s_stop_settle_wp_idx = plan.current_wp_idx;
                    s_stop_settle_target = target_phys;
                    Point2D hold = current_pose_point();
                    ctrl.current_target.x = hold.x;
                    ctrl.current_target.y = hold.y;
                    ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
                    return;
                }
                arrived = true;
            }
        }
    } else {
        update_vision_assist(target_phys);
        arrived = check_arrival(target_phys, current_radius) ||
                  check_corner_switch(plan.vision_segment_start, target_phys);
    }

    if (arrived) {
        if (!is_last_point) {
            // 切到下一段后按新运动方向重新判定横向轴，并忽略已经见过的视觉帧
            reset_vision_assist(target_phys);
            plan.current_wp_idx++;
            clear_stop_settle();
            target_phys = plan.physical_path[plan.current_wp_idx];
        } else if (App::g_state.physical.is_stopped) {
            if (!final_coord_check_pending(target_phys)) {
                ctrl.tracker_state = TrackerState::FINISHED;
                clear_stop_settle();
                reset_vision_assist(current_pose_point());
                target_phys = current_pose_point();
                ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
            }
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
void track_point_impl(const Pose2D& target, bool force_vision_assist) {
    auto& plan = App::g_state.planning;
    auto& ctrl = App::g_state.control;
    Point2D start = current_pose_point();
    Point2D target_point = {target.x, target.y};

    plan.grid_path.clear();
    plan.physical_path.clear();
    plan.force_stop_at_wp.clear();
    clear_stop_settle();
    s_final_coord_check_required = force_vision_assist || is_long_vision_segment(start, target_point);
    reset_final_coord_check();
    reset_vision_assist(start, force_vision_assist);

    ctrl.current_target = target;
    ctrl.mode = ControlMode::POINT_TRACKING;
}

void track_point(const Pose2D& target) {
    track_point_impl(target, false);
}

void track_point_with_vision_assist(const Pose2D& target) {
    track_point_impl(target, true);
}

bool final_coord_check_pending_for_target(Point2D target) {
    return final_coord_check_pending(target);
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
