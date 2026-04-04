#pragma once

// 可复用 PID 参数块
struct PidParams {
    float kp;
    float ki;
    float kd;
};

// 集中管理全车所有可调参数
struct TuningConfig {
    PidParams pid_yaw;
    PidParams pid_speed;

    struct {
        float max_speed;                   // 跟踪时的最大线速度 (cm/s)
        float max_acc;                     // 跟踪时的最大加速度 (cm/s^2)
        float max_jerk;                    // 跟踪时的最大加加速度 (cm/s^3)
        float max_ang_speed;               // 跟踪时的最大角速度 (rad/s)
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
    {2.5f, 0.0f, 0.8f},         // pid_yaw
    {0.4f, 0.5f, 0.0f},         // pid_speed
    
    // Dynamics 动力学预测参数
    {
        120.0f,    // max_speed: 1m/s，极速过弯
        85.0f,     // max_acc: 0.25G 极限抓地力
        1200.0f,   // t_acc_jerk: 0.1秒起步柔化
        2.0f       // max_ang_speed: 约 230度/秒，旋转敏捷
    },
    
    // Tracker 几何预测参数
    {
        10.0f,     // reach_radius: 10cm 切弯
        1.0f       // reach_radius_min: 终点停稳极小宽容度
    },
    
    // Motors 电机速度调节参数 (用于测试电机接线和转向)
    {
        0.0f,      // lf_speed: 左前轮速度
        0.0f,      // lb_speed: 左后轮速度
        0.0f,      // rf_speed: 右前轮速度
        0.0f       // rb_speed: 右后轮速度
    }
};