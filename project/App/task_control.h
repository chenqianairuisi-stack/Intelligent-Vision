#pragma once
#include "motor.h"
#include "pid.h"
#include "trajectory.h"
#include "kinematics.h"
#include "odometry.h"
#include "imu.h"
#include "encoder.h"
#include "system_config.h"

class ChassisControl {
public:
    ChassisControl();
    void init();

    // 设置/获取目标位姿
    void set_target_pose(const Pose2D& target) {target_pose = target;} 
    const Pose2D& get_target_pose() const { return target_pose;}

    // 控制任务更新函数
    void update_control_20ms_tick();  
private:
    Motor motors[4];
    IncPid pid_wheels[4]; 
    PosPid pid_pos_yaw;
    Trajectory tra_planner;

    Pose2D target_pose;      // 目标位姿 (cm, cm, deg)

    inline float normalize_angle(float angle);  // 将角度归一化到 [-pi, pi] 范围
    inline void run_speed_loop_and_drive(const WheelSpeed4& target_speeds);  // 速度内环控制：输入目标转速，执行 PID 并驱动电机
};

extern ChassisControl chassis_task;