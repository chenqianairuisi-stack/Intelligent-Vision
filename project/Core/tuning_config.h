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
        float brake_limit;       // 刹车限制，刹车时最大加速度为 max_acc * brake_limit
    }dynamics;

    struct {
        float reach_radius;
        float reach_radius_min;
    } tracker;

    struct {
        float mahony_kp;              // Mahony算法的KP参数，增大它会更信任加速度计，但可能引入更多噪声
        float reject_threshold;       // 视觉异常值拒绝阈值（cm），用于里程计与视觉融合时判断视觉数据是否异常
        float max_trust_alpha;        // 停车时的最大滤波系数 (越大约快，越小越平滑)
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
    {5.4f, 0.01f, 0.4f},         // pid_yaw
    {0.4f, 0.25f, 0.0f},         // pid_speed
    {0.16f, 4.0f},               // feedforward

    // Dynamics 动力学预测参数
    {
        70.0f,     // max_duty
        100.0f,    // max_speed
        70.0f,     // max_acc
        1000.0f,   // max_jerk
        3.0f,      // max_ang_speed
        1.045f,    // kinematic_gain_x
        1.013f,    // kinematic_gain_y
        0.6f       // brake_limit
    },
    
    // Tracker 几何预测参数
    {
        0.8f,      // reach_radius: 10cm 切弯
        0.4f       // reach_radius_min: 终点停稳极小宽容度
    },
    
    // Odometer 里程计融合参数 
    {
        1.0f,     // mahony_kp
        10.0f,    // reject_threshold
        0.15f     // max_trust_alpha
    },

    // Motors 电机速度调节参数 (用于测试电机接线和转向)
    {
        0.0f,      // lf_speed: 左前轮速度
        0.0f,      // lb_speed: 左后轮速度
        0.0f,      // rf_speed: 右前轮速度
        0.0f       // rb_speed: 右后轮速度
    }
};