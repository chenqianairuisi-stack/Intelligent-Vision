#include <cmath>
#include "zf_common_headfile.h" 
#include "system_config.h"
#include "tuning_config.h"
#include "zf_device_gnss.h"
#include "task_control.h"
#include "tracker.h"
#include "telemetry.h"

extern float planned_v_debug;
extern float speed_y_debug;
extern float current_local_x;
extern float current_local_y;

__attribute__((section(".dtcm_data"))) ChassisControl chassis_task;

// 生成电机实例和初始化 PID 参数
ChassisControl::ChassisControl() 
    : motors {
        {C6,  PWM2_MODULE0_CHB_C7,  false},   // RF
        {C11, PWM2_MODULE2_CHA_C10, false},   // LB
        {C8,  PWM2_MODULE1_CHB_C9,  true },   // LF
        {D3,  PWM2_MODULE3_CHA_D2,  true }    // RB
      },
      pid_wheels {  
        IncPid(tune.pid_speed), IncPid(tune.pid_speed),
        IncPid(tune.pid_speed), IncPid(tune.pid_speed)
      },
      pid_pos_yaw(tune.pid_yaw),
      target_pose {0.0f, 0.0f, 90.0f} {}

// 初始化电机
void ChassisControl::init() {
    for(auto& m : motors) m.init();
}


// 执行控制算法，更新电机输出 (由 20ms 定时器中断直接调用)
__attribute__((section(".ramfunc"))) void ChassisControl::update_control_20ms_tick() {
    // 获取当前位姿（全局坐标 + 角度 deg）
    Point2D current_pos = chassis_odometry.get_position();
    float current_yaw = imu_sensor.get_yaw();  

    // 更新目标位姿
    if (path_tracker.get_state() == TrackerState::TRACKING) {
        target_pose = path_tracker.update_and_get_target(current_pos);
    }

    // 计算全局误差
    float err_global_x = target_pose.x - current_pos.x;
    float err_global_y = target_pose.y - current_pos.y;
    float err_yaw = normalize_angle(target_pose.yaw - current_yaw);  // 转换为 [-pi, pi] 范围内的误差，单位 rad

    // 轨迹规划器根据当前距离算出一个合适的速度
    Speed2D expected_global_vel = tra_planner.velocity_planning_2d(err_global_x, err_global_y, 0.02f);

    // 将全局期望速度投影到小车自身的局部坐标系
    float current_yaw_rad = current_yaw * SystemConfig::DEG_TO_RAD;  // 转换为弧度
    float cos_theta = cosf(current_yaw_rad);
    float sin_theta = sinf(current_yaw_rad);
    float expected_local_vx = expected_global_vel.vx * sin_theta - expected_global_vel.vy * cos_theta;
    float expected_local_vy = expected_global_vel.vx * cos_theta + expected_global_vel.vy * sin_theta;

    // PID 单独计算期望的旋转速度(rad/s)并限幅
    float expected_local_vw = pid_pos_yaw.calculate(err_yaw, 0.0f);
    if(expected_local_vw > tune.dynamics.max_ang_speed) expected_local_vw = tune.dynamics.max_ang_speed; 
    if(expected_local_vw < -tune.dynamics.max_ang_speed) expected_local_vw = -tune.dynamics.max_ang_speed;

    // 逆运动学解算：将期望的底盘全向速度分配给 4 个轮子，得到每个轮子的目标转速 (v1, v2, v3, v4)
    WheelSpeed4 target_wheel_speeds = Kinematics::inverse_kinematics(expected_local_vx, expected_local_vy, expected_local_vw);

    // 速度内环控制：输入目标转速，执行 PID 并驱动电机
    run_speed_loop_and_drive(target_wheel_speeds);
}


// 供调试使用的控制更新函数
__attribute__((section(".ramfunc"))) void ChassisControl::update_control_debug_20ms_tick() {
    Point2D current_pos = chassis_odometry.get_position();
    float current_yaw = imu_sensor.get_yaw();

    // ~~~ target pose 由上位机命令直接设定，不使用 Tracker 输出 ~~~

    float err_global_x = target_pose.x - current_pos.x;
    float err_global_y = target_pose.y - current_pos.y;
    float err_yaw = normalize_angle(target_pose.yaw - current_yaw);

    Speed2D expected_global_vel = tra_planner.velocity_planning_2d(err_global_x, err_global_y, 0.02f);

    float current_yaw_rad = current_yaw * SystemConfig::DEG_TO_RAD;  // 转换为弧度
    float cos_theta = cosf(current_yaw_rad);
    float sin_theta = sinf(current_yaw_rad);
    float expected_local_vx = expected_global_vel.vx * sin_theta - expected_global_vel.vy * cos_theta;
    float expected_local_vy = expected_global_vel.vx * cos_theta + expected_global_vel.vy * sin_theta;

    float expected_local_vw = pid_pos_yaw.calculate(err_yaw, 0.0f);
    if(expected_local_vw > tune.dynamics.max_ang_speed) expected_local_vw = tune.dynamics.max_ang_speed; 
    if(expected_local_vw < -tune.dynamics.max_ang_speed) expected_local_vw = -tune.dynamics.max_ang_speed;

    planned_v_debug = std::sqrtf(expected_local_vx * expected_local_vx + expected_local_vy * expected_local_vy); 

    WheelSpeed4 target_wheel_speeds = Kinematics::inverse_kinematics(expected_local_vx, expected_local_vy, expected_local_vw);

    // 用于屏幕单独测试某个电机
    // WheelSpeed4 target_wheel_speeds = {
    //     tune.motors.lf_speed,
    //     tune.motors.lb_speed,
    //     tune.motors.rf_speed,
    //     tune.motors.rb_speed
    // };

    run_speed_loop_and_drive(target_wheel_speeds);
}



// 供调试使用的控制更新函数 (纯局部 X/Y 动力学调参专用)
// __attribute__((section(".ramfunc"))) void ChassisControl::update_control_debug_20ms_tick() {
    
//     // 提取真实轮速，正解出底盘真实的“局部速度”
//     float v_lf = encoders.get_speed_cm_s(0);
//     float v_lb = encoders.get_speed_cm_s(1);
//     float v_rf = encoders.get_speed_cm_s(2);
//     float v_rb = encoders.get_speed_cm_s(3);

//     // 麦轮正运动学：纯局部 Y 轴速度 (前进) 和 X 轴速度 (横移)
//     float real_local_vy = (v_lf + v_lb + v_rf + v_rb) / 4.0f;
//     float real_local_vx = (v_lf - v_lb - v_rf + v_rb) / 4.0f;

//     // 积分计算纯局部坐标系下的相对位移 (替代漂移的里程计)
//     // 注意：如果是多次测试，上位机下发新目标时，最好加个标志位清零这两个变量
//     current_local_x += real_local_vx * 0.02f;
//     current_local_y += real_local_vy * 0.02f;

//     // 计算局部误差 (把 target_pose 当作局部期望相对位移)
//     float err_local_x = target_pose.x - current_local_x;
//     float err_local_y = target_pose.y - current_local_y;

//     // 直接输入局部误差，输出局部平滑速度
//     Speed2D expected_local_vel = tra_planner.velocity_planning_2d(
//         err_local_x, err_local_y, 0.02f
//     );
//     // 计算标量速度，用于发送波形调试
//     planned_v_debug = std::sqrtf(expected_local_vel.vx * expected_local_vel.vx + expected_local_vel.vy * expected_local_vel.vy); 

//     // 绕过陀螺仪和三角函数投影
//     float expected_local_vx = expected_local_vel.vx;
//     float expected_local_vy = expected_local_vel.vy;
//     float expected_local_vw = 0.0f; 

//     // 逆解算与驱动
//     WheelSpeed4 target_wheel_speeds = Kinematics::inverse_kinematics(expected_local_vx, expected_local_vy, expected_local_vw);

//     run_speed_loop_and_drive(target_wheel_speeds);
// }





// ------------------- 辅助函数实现 --------------------

// 防止偏航角误差出现 359度 变成 -1度 导致的疯狂原地打转，将角度归一化到 [-pi, pi] 范围内
__attribute__((always_inline)) inline float ChassisControl::normalize_angle(float angle) {

    if (angle > 180.0f)       angle -= 360.0f;
    else if (angle < -180.0f) angle += 360.0f;

    return angle * 3.1415926535f / 180.0f; 
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
