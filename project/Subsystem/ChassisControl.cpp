#include "ChassisControl.h"
#include "RobotState.h"
#include "tuning_config.h"

#include "Tracker.h"
#include "MotionControl.h"
#include "math.h"

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

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::IncPid pid_wheels[4] = {
    Algorithm::Motion::IncPid(tune.pid_speed), Algorithm::Motion::IncPid(tune.pid_speed),
    Algorithm::Motion::IncPid(tune.pid_speed), Algorithm::Motion::IncPid(tune.pid_speed)
};

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::PosPid pid_pos_yaw(tune.pid_yaw);

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::Trajectory velocity_planner;


// --- 内部辅助函数声明 (使用匿名 namespace 彻底隐藏) ---
namespace {
    // 防止偏航角误差出现 359度 变成 -1度 导致的疯狂原地打转，将角度归一化到 [-pi, pi] 范围内
    __attribute__((always_inline)) inline float normalize_angle(float angle) {
        if (angle > 180.0f)       angle -= 360.0f;
        else if (angle < -180.0f) angle += 360.0f;
        return angle * SystemConfig::DEG_TO_RAD;
    }

    // 提取符号，用于静摩擦前馈方向判断，并引入死区防止零点震荡
    __attribute__((always_inline)) inline float get_sign(float val) {
        if (val > 1.0f) return 1.0f;
        if (val < -1.0f) return -1.0f;
        return val / 1.0f; // 在 -1.0 到 1.0 之间，平滑地从 -1 过渡到 1，绝不突变
    }


    // 速度内环控制：输入四个轮子的目标转速，执行 PID 计算并驱动电机
    __attribute__((always_inline)) inline void run_speed_loop(const WheelSpeed4& targets) {
        const auto& current_speeds = App::g_state.physical.current_wheel_speed;
        const auto& Kv = tune.ff.kv;

        // 计算前馈占空比 (简单的线性模型)，并加入符号判断实现静摩擦补偿
        float ff_lf = targets.lf * Kv + get_sign(targets.lf) * tune.ff.ka;
        float ff_lb = targets.lb * Kv + get_sign(targets.lb) * tune.ff.ka;
        float ff_rf = targets.rf * Kv + get_sign(targets.rf) * tune.ff.ka;
        float ff_rb = targets.rb * Kv + get_sign(targets.rb) * tune.ff.ka;

        // 速度环增量式 PID 计算占空比输出
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

__attribute__((section(".ramfunc"))) void update_20ms_tick() {
    auto& posi = App::g_state.physical.pose;
    auto& yaw = App::g_state.physical.pose.yaw;
    auto& ctrl = App::g_state.control;

    // 根据模式决定是否听 Tracker 的话
    // 如果是 POINT_TRACKING，我们就不调用 update_target，直接用上位机写入 ctrl.current_target 的值
    if (ctrl.mode == ControlMode::AUTO_TRACKING && ctrl.tracker_state == TrackerState::TRACKING) {
        Algorithm::Tracker::update_target();
    }

    // 计算全局误差
    float err_global_x = ctrl.current_target.x - posi.x;
    float err_global_y = ctrl.current_target.y - posi.y;
    float err_yaw = normalize_angle(ctrl.current_target.yaw - yaw);

    // 速度规划
    Speed2D expected_global_vel = velocity_planner.velocity_planning_1d(err_global_x, err_global_y, SystemConfig::PIT_CH1_DT_S);

    // 将全局期望速度投影到小车自身的局部坐标系
    float current_yaw_rad = yaw * SystemConfig::DEG_TO_RAD;  // 转换为弧度
    float cos_theta = cosf(current_yaw_rad);
    float sin_theta = sinf(current_yaw_rad);
    float expected_local_vx = expected_global_vel.vx * sin_theta - expected_global_vel.vy * cos_theta;
    float expected_local_vy = expected_global_vel.vx * cos_theta + expected_global_vel.vy * sin_theta;

    // PID 单独计算期望的旋转速度(rad/s)并限幅
    float expected_local_vw = pid_pos_yaw.calculate(err_yaw, 0.0f);
    if(expected_local_vw > tune.dynamics.max_ang_speed) expected_local_vw = tune.dynamics.max_ang_speed; 
    if(expected_local_vw < -tune.dynamics.max_ang_speed) expected_local_vw = -tune.dynamics.max_ang_speed;

    // 逆运动学解算：将期望的底盘全向速度分配给 4 个轮子，得到每个轮子的目标转速 (v1, v2, v3, v4)
    WheelSpeed4 target_wheel_speeds = Algorithm::Motion::Kinematics::inverse(expected_local_vx, expected_local_vy, expected_local_vw);

    // 速度内环控制
    run_speed_loop(target_wheel_speeds);
}


__attribute__((section(".ramfunc"))) void check_is_stopped() {

    const auto& cur_spd = App::g_state.physical.current_wheel_speed;
        
    // 判定条件：上一帧的控制目标几乎为0，且当前四个轮子的真实反馈速度极小
    App::g_state.physical.is_stopped = 
        (std::abs(cur_spd.lf) < 0.2f && std::abs(cur_spd.lb) < 0.2f &&
         std::abs(cur_spd.rf) < 0.2f && std::abs(cur_spd.rb) < 0.2f);
}

}
