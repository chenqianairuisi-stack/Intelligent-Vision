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

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::Trajectory velocity_planner;
__attribute__((section(".dtcm_data"))) static Algorithm::Motion::YawProfiled yaw_controller;
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

    __attribute__((always_inline)) inline void reset_motion_residue() {
        velocity_planner.reset();
        yaw_controller.reset();
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

        // 计算前馈占空比 (简单的线性模型)，并加入符号判断实现静摩擦补偿
        float ff_lf = targets.lf * Kv + smooth_sign(targets.lf) * tune.ff.ka;
        float ff_lb = targets.lb * Kv + smooth_sign(targets.lb) * tune.ff.ka;
        float ff_rf = targets.rf * Kv + smooth_sign(targets.rf) * tune.ff.ka;
        float ff_rb = targets.rb * Kv + smooth_sign(targets.rb) * tune.ff.ka;

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

    // 计算全局误差
    float err_global_x = ctrl.current_target.x - posi.x;
    float err_global_y = ctrl.current_target.y - posi.y;
    float err_yaw = normalize_angle(ctrl.current_target.yaw - yaw);

    // 如果当前正在自动循迹追一个非最后的航点，允许速度规划器保留一定的过弯速度，否则默认在每个航点都完全停下来（end_speed=0）
    float target_end_speed = 0.0f;
    if (ctrl.mode == ControlMode::AUTO_TRACKING && ctrl.tracker_state == TrackerState::TRACKING) {
        const auto& plan = App::g_state.planning;
        int path_size = plan.physical_path.size();
        int current_wp = static_cast<int>(plan.current_wp_idx);
        if (path_size > 0 && current_wp < path_size - 1) {
            // 非最后一个航点不刹停，保留过弯速度给速度规划器
            bool force_stop_wp =
                current_wp < plan.force_stop_at_wp.size() && plan.force_stop_at_wp[current_wp] != 0U;
            target_end_speed = force_stop_wp ? 0.0f : tune.tracker.corner_pass_speed;
        }
    }

    // 平移速度规划
    Speed2D expected_global_vel = velocity_planner.velocity_planning_1d(
        err_global_x, err_global_y, SystemConfig::PIT_CH1_DT_S, target_end_speed);

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
    WheelSpeed4 target_wheel_speeds = Algorithm::Motion::Kinematics::inverse(expected_local_vx, expected_local_vy, expected_local_vw);

    // 速度内环控制
    run_speed_loop(target_wheel_speeds);
}


/// \brief 更新底盘静止状态
///
/// \details
/// 只根据四轮反馈速度判断是否停稳，供 Tracker 终点完成判定和遥测运动计时使用
///
__attribute__((section(".ramfunc"))) void check_is_stopped() {

    const auto& cur_spd = App::g_state.physical.current_wheel_speed;
        
    // 判定条件：上一帧的控制目标几乎为0，且当前四个轮子的真实反馈速度极小
    App::g_state.physical.is_stopped = 
        (std::abs(cur_spd.lf) < 0.2f && std::abs(cur_spd.lb) < 0.2f
        && std::abs(cur_spd.rf) < 0.2f && std::abs(cur_spd.rb) < 0.2f);
}

}
