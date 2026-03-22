#include <cmath>
#include "zf_common_headfile.h" 
#include "system_config.h"
#include "tuning_config.h"
#include "zf_device_gnss.h"
#include "task_control.h"
#include "tracker.h"

using namespace SystemConfig;

__attribute__((section(".dtcm_data"))) ChassisControl chassis_task;

// 生成电机实例和初始化 PID 参数
ChassisControl::ChassisControl() 
    : motors {
        {C9,  PWM2_MODULE1_CHA_C8, false}, // LF
        {C7,  PWM2_MODULE0_CHA_C6, false}, // LB
        {D2,  PWM2_MODULE3_CHB_D3, true},  // RF
        {C10, PWM2_MODULE2_CHB_C11, true}  // RB
      },
      pid_wheels {  
        IncPid(tune.pid_speed), IncPid(tune.pid_speed), 
        IncPid(tune.pid_speed), IncPid(tune.pid_speed)
      },
      pid_pos_x(tune.pid_x),
      pid_pos_y(tune.pid_y),
      pid_pos_yaw(tune.pid_yaw) {}

// 初始化电机
void ChassisControl::init() {
    for(auto& m : motors) m.init();
}

void ChassisControl::set_target_pose(const Pose2D& target) {
    target_pose = target;
}


// 执行控制算法，更新电机输出 (由 20ms 定时器中断直接调用)
__attribute__((section(".ramfunc"))) void ChassisControl::update_control_20ms_tick() {

    // 获取当前位姿（全局坐标 + 角度）
    Point2D current_pos = chassis_odometry.get_position();
    float current_yaw = imu_sensor.get_yaw() * PI / 180.0f;  // IMU 的 yaw 是以度为单位的，这里转换成弧度
    
    // 如果正在跟踪路径，更新目标位姿为当前航点，否则保持原目标不变
    if (path_tracker.get_state() == TrackerState::TRACKING) {
        target_pose = path_tracker.update_and_get_target(current_pos);
    }

    // 计算误差并旋转坐标系
    float err_global_x = target_pose.x - current_pos.x;
    float err_global_y = target_pose.y - current_pos.y;
    float err_yaw = normalize_angle(target_pose.yaw - current_yaw);

    float cos_theta = cosf(current_yaw);
    float sin_theta = sinf(current_yaw);
    
    float e_x =  err_global_x * cos_theta + err_global_y * sin_theta;
    float e_y = -err_global_x * sin_theta + err_global_y * cos_theta;
    float e_theta = err_yaw;
    

    // 位置外环控制得到目标轮速，速度内环控制并驱动电机
    WheelSpeed4 target_wheel_speeds = run_position_loop(e_x, e_y, e_theta);  

    // 速度内环控制：输入目标转速，执行 PID 并驱动电机
    run_speed_loop_and_drive(target_wheel_speeds);
}




//--------辅助函数--------

// 防止偏航角误差出现 359度 变成 -1度 导致的疯狂原地打转
__attribute__((always_inline)) inline float ChassisControl::normalize_angle(float angle) {

    while (angle > PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// 位置外环控制：输入局部误差，输出四个轮子的目标转速
__attribute__((always_inline)) inline WheelSpeed4 ChassisControl::run_position_loop(float ex, float ey, float etheta) {
    // 位置环 PID 计算期望的底盘速度 (vx, vy, vw)
    float expected_vx = pid_pos_x.calculate(ex, 0.0f);
    float expected_vy = pid_pos_y.calculate(ey, 0.0f);
    float expected_vw = pid_pos_yaw.calculate(etheta, 0.0f);

    // 限幅，防止车速过快 (比如最大 50cm/s)
    if(expected_vx > tune.tracker.max_speed) expected_vx = tune.tracker.max_speed;
    if(expected_vx < -tune.tracker.max_speed) expected_vx = -tune.tracker.max_speed;
    if(expected_vy > tune.tracker.max_speed) expected_vy = tune.tracker.max_speed;
    if(expected_vy < -tune.tracker.max_speed) expected_vy = -tune.tracker.max_speed;
    if(expected_vw > tune.tracker.max_ang_speed) expected_vw = tune.tracker.max_ang_speed;
    if(expected_vw < -tune.tracker.max_ang_speed) expected_vw = -tune.tracker.max_ang_speed;

    // 逆运动学解算:将期望的底盘全向速度分配给 4 个轮子，得到每个轮子的目标转速 (v1, v2, v3, v4)
    return Kinematics::inverse_kinematics(expected_vx, expected_vy, expected_vw);
}

// 速度内环控制：输入四个轮子的目标转速，执行 PID 计算并驱动电机
__attribute__((always_inline)) inline void ChassisControl::run_speed_loop_and_drive(const WheelSpeed4& target_speeds) {
    // 提取每个轮子真实速度（cm/s）
    float current_speeds[4];
    current_speeds[0] = encoders.get_speed_cm_s(0);
    current_speeds[1] = encoders.get_speed_cm_s(1);
    current_speeds[2] = encoders.get_speed_cm_s(2);
    current_speeds[3] = encoders.get_speed_cm_s(3);

    // 速度环增量式 PID 计算占空比输出
    float duty_lf = pid_wheels[0].calculate(target_speeds.lf, current_speeds[0]);
    float duty_lb = pid_wheels[1].calculate(target_speeds.lb, current_speeds[1]);
    float duty_rf = pid_wheels[2].calculate(target_speeds.rf, current_speeds[2]);
    float duty_rb = pid_wheels[3].calculate(target_speeds.rb, current_speeds[3]);

    // 驱动底层电机
    motors[0].set_duty(duty_lf);
    motors[1].set_duty(duty_lb);
    motors[2].set_duty(duty_rf);
    motors[3].set_duty(duty_rb);
}