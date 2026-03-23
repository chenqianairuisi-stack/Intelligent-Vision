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
      pid_pos_yaw(tune.pid_yaw) {}

// 初始化电机
void ChassisControl::init() {
    for(auto& m : motors) m.init();
}


// 执行控制算法，更新电机输出 (由 20ms 定时器中断直接调用)
__attribute__((section(".ramfunc"))) void ChassisControl::update_control_20ms_tick() {

    // 获取当前位姿（全局坐标 + 角度 deg）
    Point2D current_pos = chassis_odometry.get_position();
    float current_yaw = imu_sensor.get_yaw();  
    
    // 如果正在跟踪路径，更新目标位姿为当前航点，否则保持原目标不变
    if (path_tracker.get_state() == TrackerState::TRACKING) {
        target_pose = path_tracker.update_and_get_target(current_pos);
    }

    // 计算全局误差与直线距离
    float err_global_x = target_pose.x - current_pos.x;
    float err_global_y = target_pose.y - current_pos.y;
    float err_yaw = normalize_angle(target_pose.yaw - current_yaw);  // 转换为 [-pi, pi] 范围内的误差，单位 rad
    float distance = std::sqrt(err_global_x * err_global_x + err_global_y * err_global_y);

    // 轨迹规划器根据当前距离算出一个合适的速度
    float v_mag = tra_planner.velocity_planning(distance, tune.tracker.max_speed, tune.tracker.max_acc, 0.02f);

    current_planned_v = v_mag;    // 供 telemetry 模块发送波形数据

    float expected_global_vx = 0.0f;
    float expected_global_vy = 0.0f;

    // 将标量总速度，沿着目标点的直线方向按比例分解
    if (distance > 0.1f) {
        expected_global_vx = v_mag * (err_global_x / distance);
        expected_global_vy = v_mag * (err_global_y / distance);
    }

    // 将全局期望速度投影到小车自身的局部坐标系
    float cos_theta = cosf(current_yaw);
    float sin_theta = sinf(current_yaw);
    float expected_local_vx = expected_global_vx * sin_theta - expected_global_vy * cos_theta;
    float expected_local_vy = expected_global_vx * cos_theta + expected_global_vy * sin_theta;

    // PID 单独计算期望的旋转速度
    float expected_local_vw = pid_pos_yaw.calculate(err_yaw, 0.0f);
    if(expected_local_vw > tune.tracker.max_ang_speed) expected_local_vw = tune.tracker.max_ang_speed; 
    if(expected_local_vw < -tune.tracker.max_ang_speed) expected_local_vw = -tune.tracker.max_ang_speed;

    // 逆运动学解算：将期望的底盘全向速度分配给 4 个轮子，得到每个轮子的目标转速 (v1, v2, v3, v4)
    WheelSpeed4 target_wheel_speeds = Kinematics::inverse_kinematics(expected_local_vx, expected_local_vy, expected_local_vw);

    // 速度内环控制：输入目标转速，执行 PID 并驱动电机
    run_speed_loop_and_drive(target_wheel_speeds);
}


//--------辅助函数--------

// 防止偏航角误差出现 359度 变成 -1度 导致的疯狂原地打转，将角度归一化到 [-pi, pi] 范围内
__attribute__((always_inline)) inline float ChassisControl::normalize_angle(float angle) {

    if (angle > 180.0f)       angle -= 360.0f;
    else if (angle < -180.0f) angle += 360.0f;

    return angle * PI / 180.0f;  
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