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
    constexpr float FINISH_WITHOUT_STOP_RADIUS_CM = 0.2f;
    constexpr float PUSH_EXTRA_REACH_RADIUS_CM = 0.05f;
    constexpr float DEFAULT_WAYPOINT_REACH_RADIUS_CM = 0.3f;
    constexpr float MAX_WAYPOINT_REACH_RADIUS_CM = 1.0f;
    constexpr float WAYPOINT_SWITCH_RADIUS_CM = 1.0f;
    constexpr float DEFAULT_CORNER_SWITCH_WINDOW_CM = 0.0f;   // 异常回退：宁可不提前切也不乱切
    constexpr float MAX_CORNER_SWITCH_WINDOW_CM = 8.0f;       // 提前切换窗口上限：现场用 !SN 在 0~8cm 间调
    constexpr float DEFAULT_CORNER_LINE_TOLERANCE_CM = 0.5f;
    constexpr float MAX_CORNER_LINE_TOLERANCE_CM = 2.0f;     // 横向容差上限：放开以配合激进过弯调参
    constexpr float MIN_TERMINAL_REACH_RADIUS_CM = 0.8f;
    // 逐点停车模式下切段合速度的硬上限 cm/s：仅在手动启用逐点停车时生效
    constexpr float STRAIGHT_MODE_SWITCH_SPEED_CAP_CM_S = 5.0f;
    // 2026-07-30 到点即硬锁（治"到绿框位置后原地蠕动徘徊"）：停车航点一进到达/越线判定就置
    // ctrl.hard_lock，由 ChassisControl 清零四轮目标 + 4 拍内插铺到 0 主动刹停，不再靠位置伺服
    // 贴点（伺服无死区 + 视觉 10ms 拽 pose = 持续追点）。置 false 回旧的位置伺服保持行为。
    constexpr bool HARD_LOCK_ON_ARRIVAL = true;
    DTCM_DATA float s_box_push_final_press_cm = DEFAULT_BOX_PUSH_FINAL_PRESS_CM;
    // 参考 2he-new：中间拐点提前切段，终点和 force_stop 航点才停车
    DTCM_DATA bool s_stop_at_every_waypoint = false;
    DTCM_DATA bool s_finish_without_stop = false;
    DTCM_DATA bool s_force_vision_assist_current_segment = false;
    DTCM_DATA bool s_vision_correction_suppressed = false;  // 炸弹爆炸窗口期屏蔽视觉修正
    DTCM_DATA bool s_stop_settle_active = false;
    DTCM_DATA uint16_t s_stop_settle_wp_idx = 0U;

    // === 15ms 视觉修正节拍（PIT_CH2）冻结状态机 ===
    // 停车/保持时冻结视觉修正（不写 pose，只推进帧序号"边收边丢"），起步/长保持解冻后走一段
    // 取新帧黑窗，杜绝把"停车途中采集的延时旧帧"喂进控制环导致刹停后冲一下。
    DTCM_DATA bool s_vision_frozen = true;            // 是否处于冻结态（开机默认冻结，直到真正开始追踪）
    DTCM_DATA bool s_vision_hold_freeze = true;       // 本次冻结是否为"长保持/起步"型（解冻需走黑窗；拐点刹停 blip 不走）
    DTCM_DATA uint32_t s_vision_restart_tick = 0U;    // 上次从保持态解冻的时刻，供黑窗计时
    DTCM_DATA bool s_vision_blackout_active = false;  // 解冻后取新帧黑窗是否生效中

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
        // 用零延迟编码器位姿判定：视觉管线 ~L 延迟下融合位姿反应慢，会造成"已冲过却还没检测到"
        const Pose2D enc = Subsystem::PoseEstimator::get_encoder_pose();
        float sdx = target.x - segment_start.x;
        float sdy = target.y - segment_start.y;
        // 段向量退化(起点≈终点)方向不可靠，不做越过判定，避免误锁
        if (sdx * sdx + sdy * sdy < 1.0f) return false;   // <1cm 段
        float pdx = enc.x - target.x;
        float pdy = enc.y - target.y;
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

        // 纵向重同步：切段时把编码器纯积分位姿 XY 贴齐当前融合位姿，消除横向纠偏累积的纵向漂移。
        // set_encoder_pose_xy 只平移历史整体（不清历史），保留延时匹配所需的帧间差值。
        const auto& fused = App::g_state.physical.pose;
        Subsystem::PoseEstimator::set_encoder_pose_xy(fused.x, fused.y);
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
            interval = DEFAULT_TUNE_CONFIG.tracker.vision_request_interval_ms;
        }
        if (interval < 20.0f) {
            interval = 20.0f;
        }
        if (interval > 1500.0f) {
            interval = 1500.0f;
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
        // 线性末端刹车开启时，到达半径跟随 stop_dist
        if (MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_BRAKE) {
            return std::max(LinearTerminalConfig::STOP_DIST_CM, MIN_TERMINAL_REACH_RADIUS_CM);
        }

        float r;
        if (!std::isfinite(tune.tracker.reach_radius_min) || tune.tracker.reach_radius_min < 0.0f) {
            r = MIN_TERMINAL_REACH_RADIUS_CM;  // 降级默认值
        } else {
            r = tune.tracker.reach_radius_min;
        }
        // 到达窗口保留硬下限，避免单拍滑行直接穿过
        return std::max(r, MIN_TERMINAL_REACH_RADIUS_CM);
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
    }

    // 当前底盘合速度大小 cm/s（由四轮反馈正运动学求出）
    [[maybe_unused]] [[gnu::always_inline]] inline float current_speed_mag() {
        const auto& w = App::g_state.physical.current_wheel_speed;
        Velocity2D v = Algorithm::Motion::Kinematics::forward(w.lf, w.lb, w.rf, w.rb);
        return std::sqrt(v.vx * v.vx + v.vy * v.vy);
    }

    // 拐点略停切换阈值 cm/s：合速度低于此即可切下一段（替代等完全停稳）
    // 现已改为每航点等完全停稳，这里保留供未来调参，暂不使用。
    [[maybe_unused]] [[gnu::always_inline]] inline float corner_pause_speed() {
        float s = tune.tracker.corner_pause_speed;
        if (!std::isfinite(s) || s < 0.0f) {
            return DEFAULT_TUNE_CONFIG.tracker.corner_pause_speed;
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
        // 切段时使用零延迟编码器位姿；融合位姿包含视觉延迟，可能已经晚于实际切点。
        const Pose2D pose = Subsystem::PoseEstimator::get_encoder_pose();
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
    ctrl.hard_lock = false;   // 新路径载入：解除上一任务遗留的硬锁
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

// 推箱 / Sokoban 路径载入：现与观测 load_path 行为完全一致——不再插 0.2cm 顶死补点，
// 终点按 must_stop 正常停稳一次。原先的 s_finish_without_stop + apply_box_push_extra 会在
// 终点 0.2cm 内制造"停→弹射→停"，把目标加速度打成正负 doublet → 底盘"猛地一下"（推箱专属抖动）。
// 去掉后推箱收尾与观测同样干净。代价：丢箱子最后 0.2cm 顶死余量（按用户取舍：消抖优先）。
// level 参数保留以维持接口/调用点不变，当前实现不再使用（顶死补点的遥测调参 '3' 随之失效但无害）。
void load_sokoban_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                       point start_grid,
                       const SokobanLevel& level) {
    (void)level;
    load_path_impl(raw_path, true, start_grid);
}

void load_box_push_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path,
                        point start_grid,
                        point,
                        point,
                        const SokobanLevel& level) {
    (void)level;
    load_path_impl(raw_path, true, start_grid);
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


/// \brief 15ms 视觉修正节拍（PIT_CH2 中断调用）
///
/// \details
/// 从 20ms 慢环解耦出来的纯 2D 视觉纠偏，按 ~15ms 贴合帧率跑，并叠加"停车冻结 + 起步取新帧"：
///  - 冻结（停车保持/AUTO 已完成/视觉不可用/爆炸屏蔽）：不写 pose，但每拍把
///    vision_last_correction_seq 同步到最新（边收边丢），防解冻瞬间爆发一串积压旧帧。
///  - 解冻：长保持/起步型冻结解冻后开一段 VISION_RESTART_BLACKOUT_MS 黑窗，窗内继续只推进
///    序号、不应用——保证第一帧应用的是"起步之后采集"的新帧（黑窗≈管线延时，滤掉停车途中采集的
///    延时帧）。拐点刹停的短暂 hard_lock blip 不走黑窗，靠切段 reset_vision_assist 的序号重同步即可，
///    避免每个弯起步都 320ms 不纠偏。
///
__attribute__((section(".ramfunc"))) void vision_correction_tick() {
    if (!TRACKING_VISION_ASSIST_ENABLED) {
        return;
    }

    const auto& ctrl = App::g_state.control;
    const auto& vision = App::g_state.vision;
    auto& plan = App::g_state.planning;
    uint32_t now = Core::Scheduler::get_sys_tick_ms();

    // 长保持/起步型冻结原因（解冻需走取新帧黑窗）
    bool hold_freeze =
        s_vision_correction_suppressed ||
        !vision.art1_map_ready ||
        (ctrl.mode == ControlMode::AUTO_TRACKING && ctrl.tracker_state != TrackerState::TRACKING) ||
        (ctrl.mode == ControlMode::POINT_TRACKING && App::g_state.physical.is_stopped &&
         !s_force_vision_assist_current_segment);
    // 拐点刹停/原地锁死型冻结（短暂，解冻不走黑窗）
    bool brake_freeze = ctrl.hard_lock;
    bool freeze = hold_freeze || brake_freeze;

    // 摄像头连续推流；仅 force-assist 段保留按需请求兜底（只置 pending 位，主循环发包）
    if (s_force_vision_assist_current_segment) {
        request_forced_vision_pose(now);
    }

    if (freeze) {
        plan.vision_last_correction_seq = vision.art1_pose_seq;  // 边收边丢，防积压
        s_vision_frozen = true;
        s_vision_hold_freeze = hold_freeze;   // hold 型优先记账，供解冻判定是否开黑窗
        return;
    }

    // 冻结→运动 边沿：再同步一次序号丢掉保持期间所有帧；hold 型解冻开黑窗
    if (s_vision_frozen) {
        s_vision_frozen = false;
        plan.vision_last_correction_seq = vision.art1_pose_seq;
        if (s_vision_hold_freeze) {
            s_vision_restart_tick = now;
            s_vision_blackout_active = true;
        } else {
            s_vision_blackout_active = false;
        }
        s_vision_hold_freeze = false;
    }

    // 取新帧黑窗：窗内只推进序号、不应用，等一帧"起步之后采集"的新帧
    if (s_vision_blackout_active) {
        if ((uint32_t)(now - s_vision_restart_tick) < SystemConfig::VISION_RESTART_BLACKOUT_MS) {
            plan.vision_last_correction_seq = vision.art1_pose_seq;
            return;
        }
        s_vision_blackout_active = false;
    }

    if (!art1_pose_recent(now)) {
        return;
    }

    Subsystem::PoseEstimator::apply_vision_axis_correction(
        plan.vision_segment_start,
        {ctrl.current_target.x, ctrl.current_target.y},
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
    // 每拍先清 hard_lock：行进途中一律不锁，只有下面停车航点的到达/停稳分支才重新置位
    // （见 HARD_LOCK_ON_ARRIVAL）。这样解锁靠"下一拍不再置位"自然完成，不需要额外解锁路径。
    ctrl.hard_lock = false;

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
    // 中间拐点默认连续通过，终点与 force_stop 航点停稳
    bool must_stop_at_wp = force_stop_wp ||
                           (is_last_point ? !finish_without_stop_at_last
                                          : s_stop_at_every_waypoint);

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
    bool crossed_continuous_corner = false;

    if (must_stop_at_wp) {
        // 终点和 force_stop 航点等四轮完全停住再放行
        // 2026-07-30：越线即到达。旧代码把 brief_pause_corner 写死 false，使 crossed_target 恒不生效 →
        // 车冲过 2cm 到达球后 check_arrival 永远 false → 位置伺服反向追点 + 末端速度地板 → 原地蠕动徘徊。
        // 现在停车航点一律"半径内 或 已越过目标线"即算到达，冲过头就地刹停不反追，
        // 残留误差留给下一段视觉横向纠偏（= 编码器为主 + 视觉纠偏）。
        // 推箱补点保持严格半径不变：它靠持续顶箱产生压紧行程，越线即停会少压一段。
        bool brief_pause_corner = !push_extra_wp;
        bool settled = App::g_state.physical.is_stopped;
        // 推箱补点不锁：它靠位置伺服持续顶箱产生压紧行程，清零四轮会少压一段
        bool lock_on_arrival = HARD_LOCK_ON_ARRIVAL && !push_extra_wp;

        if (s_stop_settle_active && s_stop_settle_wp_idx != plan.current_wp_idx) {
            clear_stop_settle();
        }

        if (s_stop_settle_active) {
            if (!settled) {
                // 到达后原地锁死等轮速停稳。
                // 2026-07-30：旧做法是"给位置规划器零距离目标"靠位置伺服保持，但伺服无死区、
                // 视觉每 10ms 还在拽 pose → 目标点跟着漂 → 车持续小幅追点 = 原地蠕动。
                // 现在直接 hard_lock：ChassisControl 把四轮目标清零并用 publish_wheel_target_interp
                // 在 4 拍(80ms)内线性铺到 0（不是速度阶跃），PID 全程主动刹，停得干脆且不再追点。
                // HARD_LOCK_ON_ARRIVAL=false 回旧的位置伺服保持。
                Point2D hold = current_pose_point();
                ctrl.current_target.x = hold.x;
                ctrl.current_target.y = hold.y;
                ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
                if (lock_on_arrival) {
                    ctrl.hard_lock = true;
                }
                return;
            } else {
                arrived = true;
            }
        }

        if (!arrived) {
            // 视觉修正已移到 PIT_CH2 的 15ms 节拍（vision_correction_tick），此处不再触发。
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
                    Point2D hold = current_pose_point();
                    ctrl.current_target.x = hold.x;
                    ctrl.current_target.y = hold.y;
                    ctrl.current_target.yaw = App::g_state.physical.pose.yaw;
                    // 到点首拍即锁死（同上：4 拍内插铺到 0，非阶跃），不给位置伺服反追的机会
                    if (lock_on_arrival) {
                        ctrl.hard_lock = true;
                    }
                    return;
                }
                arrived = true;
            }
        }
    } else {
        // 普通中间拐点沿旧段纵向投影提前切换，避免实际位置越过理想拐点后再开新段。
        // 圆形窗口保留作横向误差/异常回退；越线判定保证单拍跨过窗口时不会继续追旧点。
        float switch_radius = (push_extra_wp || finish_without_stop_at_last)
            ? current_radius
            : WAYPOINT_SWITCH_RADIUS_CM;
        arrived = check_arrival(target_phys, switch_radius);
        if (!push_extra_wp && !finish_without_stop_at_last) {
            crossed_continuous_corner = crossed_target(plan.vision_segment_start, target_phys);
            arrived = arrived || check_corner_switch(plan.vision_segment_start, target_phys) ||
                      crossed_continuous_corner;
        }
    }

    if (arrived) {
        if (!is_last_point) {
            // 正常提前切段时保留实际位置，形成连续切向；若已越过理想拐点，不能再用
            // 越点后的 P 直接连 C，否则 P->C 会在旧方向上产生反向分量。
            Point2D next_segment_start = crossed_continuous_corner
                ? target_phys
                : current_pose_point();
            reset_vision_assist(next_segment_start);
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
    ctrl.hard_lock = false;          // 点追踪由位置伺服贴点，解除硬锁
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
    // 用零延迟编码器位姿判定：避免视觉管线 ~L(≈320ms) 延迟使到达判定偏晚 → 过冲
    const Pose2D enc = Subsystem::PoseEstimator::get_encoder_pose();

    float dx = target.x - enc.x;
    float dy = target.y - enc.y;
    float dist_sq = dx * dx + dy * dy;

    return dist_sq <= radius * radius;
}

} // namespace Algorithm::Tracker
