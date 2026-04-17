#pragma once

// 可复用 PID 参数块
struct PidParams {
    float kp;
    float ki;
    float kd;
};

struct FeedforwardParams {
    float kv;  // 速度前馈系数: 多少占空比能维持 1cm/s 的稳态速度
    float ka;  // 加速度前馈系数: 克服转子惯性需要的额外占空比
};

// 集中管理全车所有可调参数
struct TuningConfig {
    PidParams pid_yaw;
    PidParams pid_speed;
    FeedforwardParams ff; 

    struct {
        float max_duty;          // 输出最大占空 (0~100)
        float max_speed;         // 跟踪时的最大线速度 (cm/s)
        float max_acc;           // 跟踪时的最大加速度 (cm/s^2)
        float max_jerk;          // 跟踪时的最大加加速度 (cm/s^3)
        float max_ang_speed;     // 跟踪时的最大角速度 (rad/s)
        float kinematic_gain_x;  // 运动学增益 x
        float kinematic_gain_y;  // 运动学增益 y
    }dynamics;

    struct {
        float reach_radius;
        float reach_radius_min;
    } tracker;

    struct {
        float lf_speed;
        float lb_speed;
        float rf_speed;
        float rb_speed;
    } motors;

};

// 全局调参实例，放在 DTCM 区域，供所有模块访问
__attribute__((section(".dtcm_data"))) inline TuningConfig tune {
    {9.0f, 0.5f, 0.6f},         // pid_yaw
    {0.2f, 0.1f, 0.0f},         // pid_speed
    {0.1f, 0.0f},               // feedforward

    // Dynamics 动力学预测参数
    {
        65.0f,     // max_duty
        120.0f,    // max_speed
        80.0f,     // max_acc
        1200.0f,   // max_jerk
        3.0f,      // max_ang_speed
        1.03f,     // kinematic_gain_x
        1.01f      // kinematic_gain_y
    },
    
    // Tracker 几何预测参数
    {
        0.8f,     // reach_radius: 10cm 切弯
        0.4f       // reach_radius_min: 终点停稳极小宽容度
    },
    
    // Motors 电机速度调节参数 (用于测试电机接线和转向)
    {
        0.0f,      // lf_speed: 左前轮速度
        0.0f,      // lb_speed: 左后轮速度
        0.0f,      // rf_speed: 右前轮速度
        0.0f       // rb_speed: 右后轮速度
    }
};