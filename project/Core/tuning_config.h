#pragma once
#include "system_config.h"

// 可复用 PID 参数块
struct PidParams {
    float kp;  // 比例系数
    float ki;  // 积分系数
    float kd;  // 微分系数
};

// 底盘前馈补偿参数块
struct FeedforwardParams {
    float kv;          // 速度前馈系数
    float ka;          // 加速度/静摩擦补偿系数
    float k_stiction;  // 角度环静摩擦补偿
};

// 集中管理全车所有可调参数
struct TuningConfig {
    PidParams pid_yaw;       // 航向角速度控制参数
    PidParams pid_speed;     // 轮速闭环控制参数
    FeedforwardParams ff;    // 底盘前馈补偿参数

    struct {
        float max_duty;          // 电机最大占空比
        float max_vel;           // 自动跟踪最大线速度 cm/s
        float max_acc;           // 自动跟踪最大线加速度 cm/s^2
        float max_ang_vel;       // 自动跟踪最大角速度 rad/s
        float max_ang_acc;       // 自动跟踪最大角加速度 rad/s^2
        float kinematic_gain_x;  // X 向运动学补偿
        float kinematic_gain_y;  // Y 向运动学补偿
        float brake_limit;       // 刹车加速度比例，实际值为 max_acc * brake_limit
    } dynamics;

    struct {
        float reach_radius;           // 普通路径点到达半径 cm
        float reach_radius_min;       // 终点到达半径 cm
        float corner_pass_speed;      // 非终点过弯保留速度 cm/s
        float corner_switch_window;   // 拐角提前切换窗口 cm
        float corner_line_tolerance;  // 拐角切换横向允许误差 cm
        float vision_request_distance; // 预留：距离路径点多远请求视觉定位 cm
        float vision_reject_dist;     // 视觉轴向校正最大接受误差 cm
        float ang_tolerance;          // 航向角死区 rad
    } tracker;

    struct {
        float mahony_kp;  // Mahony 姿态融合比例系数
    } estimate;

    struct {
        float lf_speed;  // 左前轮测试速度
        float lb_speed;  // 左后轮测试速度
        float rf_speed;  // 右前轮测试速度
        float rb_speed;  // 右后轮测试速度
    } motors;
};

// 全局调参实例，放在 DTCM 区域，供高频控制和业务模块访问
DTCM_DATA inline TuningConfig tune {
    {3.7f, 0.0f, 0.0f},      // pid_yaw
    {0.45f, 0.08f, 0.0f},    // pid_speed
    {0.2f, 4.0f, 0.54f},     // feedforward
    {
        80.0f,   // max_duty
        150.0f,  // max_vel
        65.0f,   // max_acc
        3.0f,    // max_ang_vel
        4.6f,    // max_ang_acc
        1.044f,  // kinematic_gain_x
        1.015f,  // kinematic_gain_y
        0.6f     // brake_limit
    },
    {
        0.3f,    // reach_radius
        0.2f,    // reach_radius_min
        24.0f,   // corner_pass_speed
        0.7f,    // corner_switch_window
        1.0f,    // corner_line_tolerance
        18.0f,   // vision_request_distance
        3.0f,    // vision_reject_dist
        0.006f   // ang_tolerance
    },
    {
        1.0f,    // mahony_kp
    },
    {
        0.0f,    // lf_speed
        0.0f,    // lb_speed
        0.0f,    // rf_speed
        0.0f     // rb_speed
    }
};
