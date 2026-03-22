#pragma once
#include "motor.h"
#include "pid.h"
#include "kinematics.h"
#include "odometry.h"
#include "imu.h"
#include "encoder.h"
#include "system_config.h"

class ChassisControl {
public:
    ChassisControl();
    void init();

    void set_target_pose(const Pose2D& target);
    void update_control_20ms_tick();  
private:
    Motor motors[4];
    IncPid pid_wheels[4]; // 四个轮子的速度内环 PID
    PosPid pid_pos_x;
    PosPid pid_pos_y;
    PosPid pid_pos_yaw;

    Pose2D target_pose = {0.0f, 0.0f, 0.0f};

    inline float normalize_angle(float angle);  // 将角度归一化到 [-pi, pi] 范围
    inline WheelSpeed4 run_position_loop(float ex, float ey, float etheta);  // 位置外环控制：输入局部误差，输出四个轮子的目标转速
    inline void run_speed_loop_and_drive(const WheelSpeed4& target_speeds);  // 速度内环控制：输入目标转速，执行 PID 并驱动电机
};

extern ChassisControl chassis_task;