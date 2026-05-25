#pragma once

// 可复用 PID 参数块
struct PidParams {
    float kp;
    float ki;
    float kd;
};

struct FeedforwardParams {
    float kv;  // 速度前馈系数
    float ka;  // 加速度前馈系数
    float k_stiction;  // 静摩擦力前馈系数 (用于克服角度环静摩擦力)
};

// 集中管理全车所有可调参数
struct TuningConfig {
    PidParams pid_yaw;
    PidParams pid_speed;
    FeedforwardParams ff; 

    struct {
        float max_duty;          // 输出最大占空 (0~100)
        float max_vel;         // 跟踪时的最大线速度 (cm/s)
        float max_acc;           // 跟踪时的最大加速度 (cm/s^2)
        float max_ang_vel;     // 跟踪时的最大角速度 (rad/s)
        float max_ang_acc;       // 跟踪时的最大角加速度 (rad/s^2)
        float kinematic_gain_x;  // 运动学增益 x
        float kinematic_gain_y;  // 运动学增益 y
        float brake_limit;       // 刹车限制，刹车时最大加速度为 max_acc * brake_limit
    }dynamics;

    struct {
        float reach_radius;
        float reach_radius_min;
        float ang_tolerance;
    } tracker;

    struct {
        float mahony_kp;              // Mahony算法的KP参数，增大它会更信任加速度计，但可能引入更多噪声
    } estimate;

    struct {
        float lf_speed;
        float lb_speed;
        float rf_speed;
        float rb_speed;
    } motors;

};

// 全局调参实例，放在 DTCM 区域，供所有模块访问
__attribute__((section(".dtcm_data"))) inline TuningConfig tune {
    {3.7f, 0.0f, 0.0f},           // pid_yaw
    {0.45f, 0.08f, 0.0f},         // pid_speed
    {0.2f, 4.0f, 0.54f},          // feedforward

    // Dynamics 动力学预测参数
    {
        80.0f,     // max_duty
        150.0f,    // max_vel
        65.0f,     // max_acc
        3.0f,      // max_ang_vel
        4.6f,      // max_ang_acc
        1.044f,    // kinematic_gain_x
        1.015f,    // kinematic_gain_y
        0.6f       // brake_limit
    },
    
    // Tracker 几何预测参数
    {
        0.3f,      // reach_radius: 10cm 切弯
        0.2f,      // reach_radius_min: 终点停稳极小宽容度
        0.006f     // ang_tolerance: 角度容差
    },
    
    // Odometer 里程计融合参数 
    {
        1.0f,     // mahony_kp
    },

    // Motors 电机速度调节参数 (用于测试电机接线和转向)
    {
        0.0f,      // lf_speed: 左前轮速度
        0.0f,      // lb_speed: 左后轮速度
        0.0f,      // rf_speed: 右前轮速度
        0.0f       // rb_speed: 右后轮速度
    }
};