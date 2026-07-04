#include "tuning_config.h"
#include "Tracker.h"
#include "RobotState.h"
#include "PoseEstimate.h"
#include "CoreScheduler.h"
#include "Vision.h"
#include "MotionControl.h"
#include <algorithm>
#include <cmath>

using namespace SystemConfig;

namespace Algorithm::Tracker {
namespace {
    constexpr bool TRACKING_VISION_ASSIST_ENABLED = true; // 动中视觉辅助开关，默认关闭，调试时可打开观察效果
    constexpr float DEFAULT_BOX_PUSH_FINAL_PRESS_CM = 0.2f;
    constexpr float MIN_BOX_PUSH_FINAL_PRESS_CM = 0.0f;
    constexpr float MAX_BOX_PUSH_FINAL_PRESS_CM = 1.0f;
    [[maybe_unused]] constexpr float VISION_ASSIST_MIN_SEGMENT_CM = 15.0f;
    [[maybe_unused]] constexpr float VISION_ASSIST_TARGET_FREEZE_RADIUS_CM = 10.0f;
    constexpr float FINAL_LOCK_RADIUS_CM = 1.5f;
    constexpr float FINISH_WITHOUT_STOP_RADIUS_CM = 0.2f;
    constexpr float PUSH_EXTRA_REACH_RADIUS_CM = 0.05f;
    constexpr float DEFAULT_WAYPOINT_REACH_RADIUS_CM = 0.3f;
    constexpr float MAX_WAYPOINT_REACH_RADIUS_CM = 1.0f;
    constexpr float DEFAULT_CORNER_SWITCH_WINDOW_CM = 0.0f;   // 异常回退：宁可不提前切也不乱切
    constexpr float MAX_CORNER_SWITCH_WINDOW_CM = 8.0f;       // 提前切换窗口上限：现场用 !SN 在 0~8cm 间调
    constexpr float DEFAULT_CORNER_LINE_TOLERANCE_CM = 0.5f;
    constexpr float MAX_CORNER_LINE_TOLERANCE_CM = 2.0f;     // 横向容差上限：放开以配合激进过弯调参
    constexpr float STOP_SETTLE_RELEASE_RADIUS_CM = 2.0f;
    // 终点粘滞锁：进过终点半径就锁死原地刹车，只有被推离超过此半径才解锁重新接管。
    // 取大值(远超正常惯性滑行)，杜绝末端惯性滑出小半径后规划器反向追点的极限环。
    constexpr float FINAL_LOCK_RELEASE_RADIUS_CM = 5.0f;
    // 终点锁 arm 半径的硬下限：无论 reach_radius_min 被调多小，锁窗口都不小于此，
    // 保证至少有一拍落进窗口把锁 arm 上（否则穿窗而过 → 锁永远 arm 不上）。
    constexpr float FINAL_LOCK_MIN_ARM_RADIUS_CM = 0.8f;
    // 逐点停车模式下切段合速度的硬上限 cm/s：仅在手动启用逐点停车时生效
    constexpr float STRAIGHT_MODE_SWITCH_SPEED_CAP_CM_S = 5.0f;
    DTCM_DATA float s_box_push_final_press_cm = DEFAULT_BOX_PUSH_FINAL_PRESS_CM;
    // 默认连贯模式：中间拐点不强停，靠 corner_pass_speed/end_speed 保留切向动量。
    // 终点、推箱补点和 force_stop 航点仍会停车锁死，避免末端反追点。
    DTCM_DATA bool s_stop_at_every_waypoint = false;
    DTCM_DATA bool s_finish_without_stop = false;
    DTCM_DATA bool s_force_vision_assist_current_segment = false;
    DTCM_DATA bool s_vision_correction_suppressed = false;  // 炸弹爆炸窗口期屏蔽视觉修正
    DTCM_DATA bool s_stop_settle_active = false;
    DTCM_DATA uint16_t s_stop_settle_wp_idx = 0U;
    DTCM_DATA Point2D s_stop_settle_target = {0.0f, 0.0f};
    DTCM_DATA bool s_final_lock_active = false;  // 终点粘滞锁：进过终点半径后锁死原地刹车，直到停稳

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

    // 是否已沿"段起点→目标点"方向越过目标点（惯性冲过头判定）。
    // 冲过头时 check_arrival 的小半径窗口可能被单拍滑行跳过，仅靠半径 arm 不上锁 →
    // velocity_planning_1d 翻向反追 → 来回震荡。加"越过目标线"判定，冲过即锁。
    [[gnu::always_inline]] inline bool crossed_target(Point2D segment_start, Point2D target) {
        const auto& pose = App::g_state.physical.pose;
        float sdx = target.x - segment_start.x;
        float sdy = target.y - segment_start.y;
        // 段向量退化(起点≈终点)方向不可靠，不做越过判定，避免误锁
        if (sdx * sdx + sdy * sdy < 1.0f) return false;   // <1cm 段
        float pdx = pose.x - target.x;
        float pdy = pose.y - target.y;
        return pdx * sdx + pdy * sdy >= 0.0f;   // 当前位置投影已过目标点
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
        plan.vision_last_request_tick_ms = 0U;
        s_force_vision_assist_current_segment = force_vision_assist;
        // 同步给底盘 Stanley 贴线用：段起点随切段一起更新
        App::g_state.control.segment_start = segment_start;
    }

    [[maybe_unused]] [[gnu::always_inline]] inline bool has_trackable_segment(Point2D segment_start, Point2D target) {
        float dx = target.x - segment_start.x;
        float dy = target.y - segment_start.y;
        return (dx * dx + dy * dy) >
               VISION_ASSIST_MIN_SEGMENT_CM * VISION_ASSIST_MIN_SEGMENT_CM;
    }

    [[gnu::always_inline]] inline bool art1_pose_recent(uint32_t now) {
        const auto& vision = App::g_state.vision;
        constexpr uint32_t POSE_RECENT_MS = 300U;
        return vision.art1_pose_seq != 0U &&
               vision.art1_pose_stable_count >= App::ART1_POSE_STABLE_REQUIRED_FRAMES &&
               (now - vision.art1_pose_tick_ms) <= POSE_RECENT_MS;
    }

    [[gnu::always_inline]] inline uint32_t vision_request_interval_ms() {
        float interval = tune.tracker.vision_request_interval_ms;
        if (!std::isfinite(interval)) {
            interval = TuningDefaults::DEFAULT_VISION_REQUEST_INTERVAL_MS;
        }
        if (interval < TuningDefaults::MIN_VISION_REQUEST_INTERVAL_MS) {
            interval = TuningDefaults::MIN_VISION_REQUEST_INTERVAL_MS;
        }
        if (interval > TuningDefaults::MAX_VISION_REQUEST_INTERVAL_MS) {
            interval = TuningDefaults::MAX_VISION_REQUEST_INTERVAL_MS;
        }
        return static_cast<uint32_t>(interval);
    }

    [[gnu::always_inline]] inline void request_forced_vision_pose(uint32_t now) {
        auto& plan = App::g_state.planning;
        uint32_t interval = vision_request_interval_ms();
        if (plan.vision_last_request_tick_ms == 0U ||
            now - plan.vision_last_request_tick_ms >= interval) {
            plan.vision_last_request_tick_ms = now;
            Subsystem::Vision::schedule_pose_request_ART1();
        }
    }

    [[gnu::always_inline]] inline float terminal_reach_radius() {
        float r;
        if (!std::isfinite(tune.tracker.reach_radius_min) || tune.tracker.reach_radius_min < 0.0f) {
            r = FINAL_LOCK_RADIUS_CM;  // 降级默认值
        } else {
            r = tune.tracker.reach_radius_min;
        }
        // 硬下限：锁窗口太小会被单拍滑行穿过、锁 arm 不上 → 末端反向追点震荡
        return std::max(r, FINAL_LOCK_MIN_ARM_RADIUS_CM);
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

    [[gnu::always_inline]] inline bool is_force_stop_waypoint(uint16_t idx) {
        const auto& plan = App::g_state.planning;
        return idx < plan.force_stop_at_wp.size() && plan.force_stop_at_wp[idx] == 1U;
    }

    [[gnu::always_inline]] inline bool is_push_extra_waypoint(uint16_t idx) {
        const auto& plan = App::g_state.planning;
        return idx < plan.force_stop_at_wp.size() && plan.force_stop_at_wp[idx] == 2U;
    }

    [[gnu::always_inline]] inline void force_stop_all_waypoints() {
        auto& plan = App::g_state.planning;
        for (int i = 0; i < plan.force_stop_at_wp.size(); ++i) {
            plan.force_stop_at_wp[i] = 1U;
        }
    }

    [[gnu::always_inline]] inline int find_box_at(const SokobanLevel& level, point p) {
        for (int i = 0; i < level.box_count; ++i) {
            if (level.boxes[i] == p) {
                return i;
            }
        }
        return -1;
    }

    [[gnu::always_inline]] inline bool is_unit_grid_step(point delta) {
        int dx = delta.x;
        int dy = delta.y;
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;
        return dx + dy == 1;
    }

    [[gnu::always_inline]] inline bool insert_push_extra_waypoint(int idx, Point2D extra_phys) {
        auto& plan = App::g_state.planning;
        if (idx < 0 ||
            idx >= plan.physical_path.size() ||
            plan.physical_path.size() >= MAX_PATH_LENGTH ||
            plan.grid_path.size() >= MAX_PATH_LENGTH ||
            plan.force_stop_at_wp.size() >= MAX_PATH_LENGTH) {
            return false;
        }

        int insert_idx = idx + 1;

        plan.physical_path.push_back(plan.physical_path.back());
        for (int i = plan.physical_path.size() - 1; i > insert_idx; --i) {
            plan.physical_path[i] = plan.physical_path[i - 1];
        }
        plan.physical_path[insert_idx] = extra_phys;

        plan.grid_path.push_back(plan.grid_path.back());
        for (int i = plan.grid_path.size() - 1; i > insert_idx; --i) {
            plan.grid_path[i] = plan.grid_path[i - 1];
        }
        plan.grid_path[insert_idx] = plan.grid_path[idx];

        plan.force_stop_at_wp.push_back(plan.force_stop_at_wp.back());
        for (int i = plan.force_stop_at_wp.size() - 1; i > insert_idx; --i) {
            plan.force_stop_at_wp[i] = plan.force_stop_at_wp[i - 1];
        }
        plan.force_stop_at_wp[insert_idx] = 2U;
        return true;
    }

    void apply_box_push_extra_from_raw_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                                            point start_grid,
                                            const SokobanLevel& level) {
        auto& plan = App::g_state.planning;
        if (raw_path.empty() || plan.grid_path.empty() || plan.physical_path.empty()) {
            return;
        }

        float press = clamp_float(s_box_push_final_press_cm,
                                  MIN_BOX_PUSH_FINAL_PRESS_CM,
                                  MAX_BOX_PUSH_FINAL_PRESS_CM);
        if (press <= 0.0f || !valid_grid(start_grid)) {
            return;
        }

        SokobanLevel work = level;
        point car = start_grid;
        int plan_idx = 0;

        for (int i = 0; i < raw_path.size(); ++i) {
            point next = raw_path[i];

            if (next == car) {
                if (plan_idx < plan.grid_path.size() && plan.grid_path[plan_idx] == next) {
                    ++plan_idx;
                }
                continue;
            }

            point delta = next - car;
            if (!is_unit_grid_step(delta)) {
                car = next;
                continue;
            }

            int box_idx = find_box_at(work, next);
            bool inserted_extra = false;
            if (box_idx >= 0) {
                Point2D unit = grid_delta_to_physical_unit(delta);
                if (plan_idx < plan.grid_path.size() && plan.grid_path[plan_idx] == next) {
                    Point2D extra_phys = plan.physical_path[plan_idx];
                    extra_phys.x += unit.x * press;
                    extra_phys.y += unit.y * press;
                    inserted_extra = insert_push_extra_waypoint(plan_idx, extra_phys);
                }

                point pushed_to = next + delta;
                if (valid_grid(pushed_to)) {
                    work.boxes[box_idx] = pushed_to;
                }
            }

            car = next;
            if (plan_idx < plan.grid_path.size() && plan.grid_path[plan_idx] == next) {
                plan_idx += inserted_extra ? 2 : 1;
            }
        }
    }

    [[gnu::always_inline]] inline void clear_stop_settle() {
        s_stop_settle_active = false;
        s_stop_settle_wp_idx = 0U;
        s_stop_settle_target = {0.0f, 0.0f};
        s_final_lock_active = false;
    }

    // 当前底盘合速度大小 cm/s（由四轮反馈正运动学求出）
    [[gnu::always_inline]] inline float current_speed_mag() {
        const auto& w = App::g_state.physical.current_wheel_speed;
        Velocity2D v = Algorithm::Motion::Kinematics::forward(w.lf, w.lb, w.rf, w.rb);
        return std::sqrt(v.vx * v.vx + v.vy * v.vy);
    }

    // 拐点略停切换阈值 cm/s：合速度低于此即可切下一段（替代等完全停稳）
    [[gnu::always_inline]] inline float corner_pause_speed() {
        float s = tune.tracker.corner_pause_speed;
        if (!std::isfinite(s) || s < 0.0f) {
            return TuningDefaults::DEFAULT_CORNER_PAUSE_SPEED;
        }
        return s;
    }

    // 过弯保留速度 cm/s：拐点不停顿时喂给速度规划器的段末速度，>0 即带速切向下一段
    [[gnu::always_inline]] inline float corner_pass_speed() {
        float s = tune.tracker.corner_pass_speed;
        if (!std::isfinite(s) || s < 0.0f) {
            return 0.0f;
        }
        return std::min(s, tune.dynamics.max_vel);
    }

    [[gnu::always_inline]] inline void push_unique_grid_waypoint(
        StaticArray<point, MAX_PATH_LENGTH>& path,
        point waypoint) {
        if (path.empty() || path.back() != waypoint) {
            path.push_back(waypoint);
        }
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
    clear_stop_settle();
    s_finish_without_stop = false;
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

void load_sokoban_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                       point start_grid,
                       const SokobanLevel& level) {
    load_path_impl(raw_path, true, start_grid);
    s_finish_without_stop = true;
    apply_box_push_extra_from_raw_path(raw_path, start_grid, level);
}

void load_box_push_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                        point start_grid,
                        point,
                        point,
                        const SokobanLevel& level) {
    load_path_impl(raw_path, true, start_grid);
    s_finish_without_stop = true;
    apply_box_push_extra_from_raw_path(raw_path, start_grid, level);
}

void load_bomb_push_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                         point start_grid,
                         point bomb_target,
                         const SokobanLevel& level) {
    load_path_impl(raw_path, true, start_grid);
    force_stop_all_waypoints();
    apply_box_push_extra_from_raw_path(raw_path, start_grid, level);

    auto& plan = App::g_state.planning;
    if (plan.physical_path.empty() || raw_path.size() == 0) {
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

/// \brief 设置/清除视觉修正抑制（炸弹爆炸窗口期用）
void set_vision_correction_suppressed(bool suppressed) {
    s_vision_correction_suppressed = suppressed;
}

/// \brief 运动中的视觉辅助：周期请求 ART1，并在速度规划前应用延时补偿后的视觉修正
__attribute__((section(".ramfunc"))) void update_vision_assist(const Point2D& target) {
    if (!TRACKING_VISION_ASSIST_ENABLED) {
        return;
    }

    // 炸弹爆炸闪光会污染视觉坐标，等待窗口内不让视觉修正进入控制环（只锁位+里程计）
    if (s_vision_correction_suppressed) {
        return;
    }

    if (!App::g_state.vision.art1_map_ready) {
        return;
    }

    auto& plan = App::g_state.planning;
    const Point2D segment_start = plan.vision_segment_start;
    uint32_t now = Core::Scheduler::get_sys_tick_ms();

    // 摄像头连续推流：保留按需请求作为兜底，不依赖它
    if (s_force_vision_assist_current_segment) {
        request_forced_vision_pose(now);
    }

    // 纯视觉：不再按段长/近终点/是否停稳设门，只要有近期稳定视觉帧就
    // 全程持续做全 2D 纠偏（轴向/冻结判定已下沉到 PoseEstimate，这里不再拦截）
    if (!art1_pose_recent(now)) {
        return;
    }

    Subsystem::PoseEstimator::apply_vision_axis_correction(
        segment_start,
        target,
        plan.vision_last_correction_seq,
        true
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

    // 默认按停车规划；只有确认是"过弯不停顿"的拐点才在下面改成保留速度
    ctrl.segment_end_speed = 0.0f;

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
    bool push_extra_wp = is_push_extra_waypoint(plan.current_wp_idx);
    Point2D target_phys = plan.physical_path[plan.current_wp_idx];
    bool finish_without_stop_at_last = is_last_point && s_finish_without_stop;
    bool must_stop_at_wp = (!finish_without_stop_at_last && is_last_point) ||
                           force_stop_wp ||
                           s_stop_at_every_waypoint;

    // 终点粘滞锁：一旦进过终点半径就锁死，之后只原地刹车、绝不再朝 target_phys 反向追点，
    // 直到 is_stopped 才 FINISHED。解决"reach_radius 窗口 < 单拍滑行 → 惯性滑出窗口后
    // velocity_planning_1d 把速度矢量翻 180° 反向追点"的末端来回震荡。
    // 每拍把 target 重设成当前位姿(dx≈0→规划器输出0)，是"纯刹车不追点"的关键，别改回追 target_phys。
    if (!finish_without_stop_at_last && is_last_point) {
        if (!s_final_lock_active &&
            (check_arrival(target_phys, terminal_reach_radius()) ||
             crossed_target(plan.vision_segment_start, target_phys))) {
            s_final_lock_active = true;
        }
        if (s_final_lock_active) {
            // 只有被大幅推离(远超正常惯性滑行)才解锁重新接管；小幅滑出仍保持原地刹车
            if (!check_arrival(target_phys, FINAL_LOCK_RELEASE_RADIUS_CM)) {
                s_final_lock_active = false;
            } else {
                Point2D hold = current_pose_point();
                ctrl.current_target.x = hold.x;
                ctrl.current_target.y = hold.y;
                ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
                if (App::g_state.physical.is_stopped) {
                    ctrl.tracker_state = TrackerState::FINISHED;
                    clear_stop_settle();
                    reset_vision_assist(current_pose_point());
                }
                return;
            }
        }
    }

    float current_radius = must_stop_at_wp ?
        terminal_reach_radius() :
        waypoint_reach_radius();
    if (push_extra_wp) {
        current_radius = PUSH_EXTRA_REACH_RADIUS_CM;
    } else if (finish_without_stop_at_last) {
        current_radius = FINISH_WITHOUT_STOP_RADIUS_CM;
    }
    // 非终点航点允许提前切换，终点必须按更小半径进入并停稳
    bool arrived = false;

    if (must_stop_at_wp) {
        // 中间拐点（force-stop 但非终点、非推箱补点）只需略停顿：合速度低于阈值即可切；
        // 真正终点/推箱补点仍要求完全停稳，保证精度不变。
        bool brief_pause_corner = !is_last_point && !push_extra_wp && !finish_without_stop_at_last;
        // 逐点停车模式：切段合速度阈值封顶，杜绝 flash 残留高值 → 带速切向磨圆弧
        float switch_speed = corner_pause_speed();
        if (s_stop_at_every_waypoint) {
            switch_speed = std::min(switch_speed, STRAIGHT_MODE_SWITCH_SPEED_CAP_CM_S);
        }
        bool settled = brief_pause_corner ?
            (current_speed_mag() < switch_speed) :
            App::g_state.physical.is_stopped;

        if (s_stop_settle_active && s_stop_settle_wp_idx != plan.current_wp_idx) {
            clear_stop_settle();
        }

        if (s_stop_settle_active) {
            if (!check_arrival(s_stop_settle_target, STOP_SETTLE_RELEASE_RADIUS_CM)) {
                clear_stop_settle();
            } else if (!settled) {
                Point2D hold = current_pose_point();
                ctrl.current_target.x = hold.x;
                ctrl.current_target.y = hold.y;
                ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
                return;
            } else {
                arrived = true;
            }
        }

        if (!arrived) {
            update_vision_assist(target_phys);

            // 普通拐点(brief_pause)冲过头也算到达 → arm stop_settle 原地刹车，不反向追点。
            // 推箱补点/不停顿终点保持严格半径，保精度不变。
            bool reached = check_arrival(target_phys, current_radius);
            if (brief_pause_corner) {
                reached = reached || crossed_target(plan.vision_segment_start, target_phys);
            }
            if (reached) {
                if (!settled) {
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
        // 纯过弯航点（非终点、非强停、非推箱补点）：给速度规划器一个非零段末速度，
        // 让车带速直接切向下一段，不再减速停车。推箱补点/不停顿终点仍按停车规划。
        if (!push_extra_wp && !finish_without_stop_at_last) {
            ctrl.segment_end_speed = corner_pass_speed();
        }
        update_vision_assist(target_phys);
        arrived = check_arrival(target_phys, current_radius) ||
                  (!push_extra_wp && check_corner_switch(plan.vision_segment_start, target_phys));
    }

    if (arrived) {
        if (!is_last_point) {
            // 切到下一段后按新运动方向重新判定横向轴，并忽略已经见过的视觉帧
            reset_vision_assist(target_phys);
            plan.current_wp_idx++;
            clear_stop_settle();
            target_phys = plan.physical_path[plan.current_wp_idx];
        } else if (finish_without_stop_at_last || App::g_state.physical.is_stopped) {
            ctrl.tracker_state = TrackerState::FINISHED;
            clear_stop_settle();
            reset_vision_assist(current_pose_point());
            if (!finish_without_stop_at_last) {
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
    plan.grid_path.clear();
    plan.physical_path.clear();
    plan.force_stop_at_wp.clear();
    clear_stop_settle();
    s_finish_without_stop = false;
    reset_vision_assist(current_pose_point(), force_vision_assist);

    ctrl.current_target = target;
    ctrl.segment_end_speed = 0.0f;   // 锁点/收尾一律按停车规划
    ctrl.mode = ControlMode::POINT_TRACKING;
}

void track_point(const Pose2D& target) {
    track_point_impl(target, false);
}

void track_point_with_vision_assist(const Pose2D& target) {
    track_point_impl(target, true);
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
