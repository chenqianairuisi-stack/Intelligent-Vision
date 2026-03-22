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
        float max_speed;                     // 跟踪时的最大线速度 (cm/s)
        float max_acc;                       // 跟踪时的最大加速度 (cm/s^2)
        float max_ang_speed;                 // 跟踪时的最大角速度 (rad/s)
        float reach_radius;
        float reach_radius_min;
    } tracker;

};

__attribute__((section(".dtcm_data"))) inline TuningConfig tune {
    {2.5f, 0.0f, 0.8f},                      // pid_yaw
    {1.2f, 0.5f, 0.0f},                      // pid_speed
    {50.0f, 2.0f, 3.0f, 8.0f, 2.0f},         // tracker
};
