#include "ChassisControl.h"
#include "RobotState.h"
#include "tuning_config.h"

#include "Tracker.h"
#include "MotionControl.h"
#include <cmath>

#include "zf_common_headfile.h"
#include "Motor.h"
#include "Encoder.h"


namespace Subsystem::Chassis {

// ====================================================================
// 内部私有实现
// ====================================================================

__attribute__((section(".dtcm_data"))) static Motor motors[4] = {
    {C6,  PWM2_MODULE0_CHB_C7,  false},   // LF
    {C11, PWM2_MODULE2_CHA_C10, false},   // LB
    {C8,  PWM2_MODULE1_CHB_C9,  true },   // RF
    {D3,  PWM2_MODULE3_CHA_D2,  true }    // RB
};

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::Speed_PosPid pid_wheels[4] = {
    Algorithm::Motion::Speed_PosPid(tune.pid_speed), Algorithm::Motion::Speed_PosPid(tune.pid_speed),
    Algorithm::Motion::Speed_PosPid(tune.pid_speed), Algorithm::Motion::Speed_PosPid(tune.pid_speed)
};

// __attribute__((section(".dtcm_data"))) static Algorithm::Motion::Angle_PosPid pid_pos_yaw(tune.pid_yaw);

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::PathLineFollower path_follower;
__attribute__((section(".dtcm_data"))) static Algorithm::Motion::YawProfiled yaw_controller;
// 慢环(20ms)产出、快环(5ms)消费的四轮目标速度。两段同在 PIT 中断、不并发，无需加锁。
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
__attribute__((section(".dtcm_data"))) static bool control_history_ready = false;
__attribute__((section(".dtcm_data"))) static ControlMode last_control_mode = ControlMode::AUTO_TRACKING;
__attribute__((section(".dtcm_data"))) static TrackerState last_tracker_state = TrackerState::NONE;
__attribute__((section(".dtcm_data"))) static Pose2D last_control_target = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};

// --- 内部辅助函数声明 ---
namespace {
    // 防止偏航角误差出现 359度 变成 -1度 导致的疯狂原地打转，将角度归一化到 [-pi, pi] 范围内
    __attribute__((always_inline)) inline float normalize_angle(float angle) {
        if (angle > 180.0f)       angle -= 360.0f;
        else if (angle < -180.0f) angle += 360.0f;
        return angle * SystemConfig::DEG_TO_RAD;
    }

    __attribute__((always_inline)) inline float smooth_sign(float val) {
        // 0.5f 决定了过渡带的陡峭程度，值越小越接近阶跃，但不突变
        return val / (std::abs(val) + 0.5f);
    }

    // 静摩擦前馈：轮子接近静止、要启动时给满 ka 破静摩擦（动作干脆）；轮子转起来后
    // 动摩擦更小，按实测轮速衰减 ka，避免低速/换向段被常量 ka 过量驱动 → 极限环啸叫。
    __attribute__((always_inline)) inline float friction_ff(float target, float current) {
        constexpr float KA_FADE_V = 6.0f;  // cm/s：轮速超过此量级后静摩擦补偿基本退场
        // 近零死区：目标轮速极小(停稳/换向过零)时不给 ka 踢，否则过零瞬间 ka 反号 +
        // 硬刹车过冲 → 原地小幅晃/抖（brake_limit 偏大时尤甚）。真要动时目标远大于此、不受影响。
        constexpr float KA_MIN_TARGET = 1.0f;  // cm/s
        if (std::abs(target) < KA_MIN_TARGET) return 0.0f;
        float fade = KA_FADE_V / (std::abs(current) + KA_FADE_V);  // current≈0→1，高速→~0
        return smooth_sign(target) * tune.ff.ka * fade;
    }

    // 主动刹车/锁定前馈（"加速度前瞻"）：当目标轮速明显低于当前轮速（在减速/想停但轮子
    // 还带速）时，额外给一个与残余轮速反向的制动占空比，把车更快钉到目标上。
    //   - 治"冲过头→回拉→晃"：减速全程轮子略快于目标 → 持续小幅主动制动 → 缩短过冲；
    //   - 治"终点锁不死"：锁定时目标≈0、轮子还在滑 → 强主动制动 → 锁得更死。
    // 只在减速方向介入，不会反向加速；低于接入门限不介入，避免停稳后在零附近抖。
    // 增益走 tune.feel.brake_hold_gain（菜单 BrkHold / !T G 现场调，Save 持久化）。
    __attribute__((always_inline)) inline float brake_ff(float target, float current) {
        constexpr float BRAKE_ENGAGE_SPEED = 0.5f;  // cm/s：轮速低于此不介入，避免停稳抖
        if (std::abs(current) < BRAKE_ENGAGE_SPEED) return 0.0f;
        // 只有"目标比当前更慢"(在减速)才主动制动；加速/匀速时不介入
        if (std::abs(target) >= std::abs(current)) return 0.0f;
        return -tune.feel.brake_hold_gain * (current - target);
    }

    __attribute__((always_inline)) inline void reset_motion_residue() {
        path_follower.reset();
        yaw_controller.reset();
        s_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        for (auto& pid : pid_wheels) {
            pid.reset();
        }
    }

    __attribute__((always_inline)) inline bool target_changed_for_point_mode(const Pose2D& target) {
        return std::abs(target.x - last_control_target.x) > 0.01f ||
               std::abs(target.y - last_control_target.y) > 0.01f ||
               std::abs(target.yaw - last_control_target.yaw) > 0.01f;
    }

    __attribute__((always_inline)) inline void guard_motion_residue(const App::RobotState& state) {
        const auto& ctrl = state.control;
        if (!control_history_ready) {
            control_history_ready = true;
            last_control_mode = ctrl.mode;
            last_tracker_state = ctrl.tracker_state;
            last_control_target = ctrl.current_target;
            return;
        }

        bool mode_changed = (ctrl.mode != last_control_mode);
        bool auto_tracking_started =
            ctrl.mode == ControlMode::AUTO_TRACKING &&
            ctrl.tracker_state == TrackerState::TRACKING &&
            last_tracker_state != TrackerState::TRACKING;
        bool point_target_changed =
            ctrl.mode == ControlMode::POINT_TRACKING &&
            target_changed_for_point_mode(ctrl.current_target);

        if (mode_changed || auto_tracking_started || point_target_changed) {
            reset_motion_residue();
        }

        last_control_mode = ctrl.mode;
        last_tracker_state = ctrl.tracker_state;
        last_control_target = ctrl.current_target;
    }

    // 检查位姿是否明显跑出地图边界（允许有一定容错，防止里程计轻微抖动误触发）
    __attribute__((always_inline)) inline bool is_pose_outside_field(const Pose2D& pose) {
        // 自动循迹时若里程计明显跑出地图，立即进入错误态防止继续冲出场地
        constexpr float FIELD_MARGIN_CM = 5.0f;
        constexpr float FIELD_MAX_X_CM = SystemConfig::MAP_MAX_WIDTH * SystemConfig::GRID_SIZE_CM;
        constexpr float FIELD_MAX_Y_CM = SystemConfig::MAP_MAX_HEIGHT * SystemConfig::GRID_SIZE_CM;

        return pose.x < -FIELD_MARGIN_CM || pose.x > FIELD_MAX_X_CM + FIELD_MARGIN_CM ||
                pose.y < -FIELD_MARGIN_CM || pose.y > FIELD_MAX_Y_CM + FIELD_MARGIN_CM;
    }

    // 当检测到明显的边界错误时，立即停止底盘并切换到 POINT_TRACKING 模式锁死当前位置，同时通知 GameManager 进入错误状态
    __attribute__((always_inline)) inline void stop_on_boundary_error(const Pose2D& pose) {
        auto& ctrl = App::g_state.control;
        // 目标锁在当前位置，先让底盘停止，再把错误交给 GameManage 处理
        ctrl.current_target.x = pose.x;
        ctrl.current_target.y = pose.y;
        ctrl.current_target.yaw = pose.yaw;
        ctrl.mode = ControlMode::POINT_TRACKING;
        ctrl.tracker_state = TrackerState::FINISHED;
        App::g_state.game.error_stage = 9;
        App::g_state.game.phase = GamePhase::ERROR_OCCURRED;
    }

    // 速度内环控制：输入四个轮子的目标转速，执行 PID 计算并驱动电机
    __attribute__((always_inline)) inline void run_speed_loop(const WheelSpeed4& targets) {
        const auto& current_speeds = App::g_state.physical.current_wheel_speed;
        const auto& Kv = tune.ff.kv;

        // 计算前馈占空比 (简单的线性模型)，静摩擦补偿按轮速衰减（见 friction_ff）
        // 再叠加主动刹车前馈（brake_ff）：减速/锁定时额外制动，缩短过冲、锁得更死。
        float ff_lf = targets.lf * Kv + friction_ff(targets.lf, current_speeds.lf) + brake_ff(targets.lf, current_speeds.lf);
        float ff_lb = targets.lb * Kv + friction_ff(targets.lb, current_speeds.lb) + brake_ff(targets.lb, current_speeds.lb);
        float ff_rf = targets.rf * Kv + friction_ff(targets.rf, current_speeds.rf) + brake_ff(targets.rf, current_speeds.rf);
        float ff_rb = targets.rb * Kv + friction_ff(targets.rb, current_speeds.rb) + brake_ff(targets.rb, current_speeds.rb);

        // 速度环位置式 PID + 前馈计算占空比输出
        float duty_lf = ff_lf + pid_wheels[0].calculate(targets.lf, current_speeds.lf);
        float duty_lb = ff_lb + pid_wheels[1].calculate(targets.lb, current_speeds.lb);
        float duty_rf = ff_rf + pid_wheels[2].calculate(targets.rf, current_speeds.rf);
        float duty_rb = ff_rb + pid_wheels[3].calculate(targets.rb, current_speeds.rb);

        // 驱动底层电机
        motors[0].set_duty(duty_lf, tune.dynamics.max_duty);
        motors[1].set_duty(duty_lb, tune.dynamics.max_duty);
        motors[2].set_duty(duty_rf, tune.dynamics.max_duty);
        motors[3].set_duty(duty_rb, tune.dynamics.max_duty);
    }
}

// ====================================================================
// 对外公开接口实现
// ====================================================================

void init() {
    for(auto& m : motors) m.init();    // 电机初始化 (gpio + pwm)
    encoders.init();                   // 编码器初始化 (encoder)
}

/// \brief 20ms 底盘控制周期
///
/// \details
/// 根据当前控制模式选择 Tracker 目标或上位机目标，完成速度规划、坐标转换、Yaw 控制和四轮速度闭环
/// 自动循迹时会检查地图边界，并在非终点航点向速度规划器传入过弯末速度
///
__attribute__((section(".ramfunc"))) void update_20ms_tick() {
    auto& posi = App::g_state.physical.pose;
    auto& yaw = App::g_state.physical.pose.yaw;
    auto& ctrl = App::g_state.control;

    if (ctrl.mode == ControlMode::AUTO_TRACKING && is_pose_outside_field(posi)) {
        stop_on_boundary_error(posi);
    }

    // 根据模式决定是否听 Tracker 的话
    // 如果是 POINT_TRACKING，我们就不调用 update_target，直接用上位机写入 ctrl.current_target 的值
    if (ctrl.mode == ControlMode::AUTO_TRACKING && ctrl.tracker_state == TrackerState::TRACKING) {
        Algorithm::Tracker::update_target();
    } else if (ctrl.mode == ControlMode::POINT_TRACKING) {
        Algorithm::Tracker::update_vision_assist({ctrl.current_target.x, ctrl.current_target.y});
    }

    guard_motion_residue(App::g_state);

    // 硬锁：到达停车航点半径内后 Tracker 置位 hard_lock。此时直接把四轮目标速度清零，
    // 不再经过 path_follower / yaw 规划 / 逆运动学，靠 5ms 速度环 PID 顶在 0 主动刹停锁死轮胎。
    // 同步清掉规划器与偏航控制器的残留速度，解锁进入下一段时从零平滑起步。
    if (ctrl.hard_lock) {
        path_follower.reset();
        yaw_controller.reset();
        s_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        return;
    }

    float err_yaw = normalize_angle(ctrl.current_target.yaw - yaw);

    // 过弯不停顿：末速度由 Tracker 按"当前航点是否需要停稳"写入。
    // 拐点（非终点、非强停）给非零保留速度让车直接带速切向；需停稳处为 0 按停车规划。
    float target_end_speed = ctrl.segment_end_speed;

    // 沿路径线跟踪 + Stanley 横向纠偏：贴着 segment_start→current_target 这条线走，
    // 而不是只朝目标点收敛。段长过短或 Stanley 关闭时内部自动退化为纯朝点。
    Speed2D expected_global_vel = path_follower.follow(
        posi.x, posi.y,
        ctrl.segment_start.x, ctrl.segment_start.y,
        ctrl.current_target.x, ctrl.current_target.y,
        SystemConfig::PIT_CH1_DT_S, target_end_speed);

    // 把规划器输出的全局期望速度暴露给延时补偿：它是指令、不含打滑，
    // 末尾刹车时趋近 0，用来给编码器外推位移封顶，挡掉打滑虚增的过冲。
    ctrl.commanded_vel = expected_global_vel;

    // 将全局期望速度投影到小车自身的局部坐标系
    float current_yaw_rad = yaw * SystemConfig::DEG_TO_RAD;  // 转换为弧度
    float cos_theta = cosf(current_yaw_rad);
    float sin_theta = sinf(current_yaw_rad);
    float expected_local_vx = expected_global_vel.vx * sin_theta - expected_global_vel.vy * cos_theta;
    float expected_local_vy = expected_global_vel.vx * cos_theta + expected_global_vel.vy * sin_theta;


    // Yaw 角速度规划：根据当前的偏航误差，计算出一个平滑的期望角速度
    bool is_translating = (std::abs(expected_local_vx) > 2.0f || std::abs(expected_local_vy) > 2.0f);
    float expected_local_vw = yaw_controller.calculate(err_yaw, SystemConfig::PIT_CH1_DT_S, is_translating);


    // 逆运动学解算：将期望的底盘全向速度分配给 4 个轮子，得到每个轮子的目标转速 (v1, v2, v3, v4)
    // 只产出目标速度，交给 5ms 快环闭环；本慢环不再直接驱动电机。
    s_target_wheel_speeds = Algorithm::Motion::Kinematics::inverse(expected_local_vx, expected_local_vy, expected_local_vw);
}


/// \brief 5ms 轮速内环（200Hz 快环）
///
/// \details
/// 消费慢环(20ms)最近一次产出的四轮目标速度，配合 5ms 新测得的轮速跑 PID+前馈出占空比。
/// 内环提速到 200Hz 后才能在高速/急刹下紧跟目标，不再因 50Hz 跟不上而过冲互顶（啸叫根因）。
///
__attribute__((section(".ramfunc"))) void update_speed_loop_5ms() {
    run_speed_loop(s_target_wheel_speeds);
}


/// \brief 更新底盘静止状态
///
/// \details
/// 只根据四轮反馈速度判断是否停稳，供 Tracker 终点完成判定和遥测运动计时使用
///
__attribute__((section(".ramfunc"))) void check_is_stopped() {

    const auto& cur_spd = App::g_state.physical.current_wheel_speed;
    const auto& target_spd = s_target_wheel_speeds;
    constexpr float STOPPED_WHEEL_SPEED_EPS = 0.5f;
    constexpr float STOPPED_TARGET_SPEED_EPS = 0.5f;
        
    // 判定条件：上一帧的控制目标几乎为0，且当前四个轮子的真实反馈速度极小
    App::g_state.physical.is_stopped = 
        (std::abs(target_spd.lf) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(target_spd.lb) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(target_spd.rf) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(target_spd.rb) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(cur_spd.lf) < STOPPED_WHEEL_SPEED_EPS &&
         std::abs(cur_spd.lb) < STOPPED_WHEEL_SPEED_EPS &&
         std::abs(cur_spd.rf) < STOPPED_WHEEL_SPEED_EPS &&
         std::abs(cur_spd.rb) < STOPPED_WHEEL_SPEED_EPS);
}

}
