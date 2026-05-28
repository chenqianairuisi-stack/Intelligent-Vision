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
        float vision_request_interval_ms; // Reserved: no periodic ART1 pose request while tracking
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

    struct {
        float encoder_latency_gain; // Encoder odometry latency compensation gain
        float vision_latency_ms;    // Vision pose pipeline latency estimate ms
    } latency;
};

namespace TuningDefaults {
    inline constexpr float DEFAULT_REACH_RADIUS_MIN = 0.25f;
    inline constexpr float TERMINAL_REACH_RADIUS_CAP_CM = 0.25f;
    inline constexpr float DEFAULT_CORNER_PASS_SPEED = 18.0f;
    inline constexpr float DEFAULT_BRAKE_LIMIT = 0.85f;
    inline constexpr float DEFAULT_ENCODER_LATENCY_GAIN = 1.00f;
    inline constexpr float DEFAULT_VISION_LATENCY_MS = 300.0f;
    inline constexpr float DEFAULT_VISION_REQUEST_INTERVAL_MS = 250.0f;
    inline constexpr float DEFAULT_VISION_REJECT_DIST = 3.0f;

    inline constexpr float MIN_REACH_RADIUS_MIN = 0.10f;
    inline constexpr float MAX_REACH_RADIUS_MIN = 3.0f;
    inline constexpr float MIN_CORNER_PASS_SPEED = 0.0f;
    inline constexpr float MAX_CORNER_PASS_SPEED = 80.0f;
    inline constexpr float MIN_BRAKE_LIMIT = 0.10f;
    inline constexpr float MAX_BRAKE_LIMIT = 2.0f;
    inline constexpr float MIN_ENCODER_LATENCY_GAIN = 0.01f;
    inline constexpr float MAX_ENCODER_LATENCY_GAIN = 2.00f;
    inline constexpr float MIN_VISION_LATENCY_MS = 10.0f;
    inline constexpr float MAX_VISION_LATENCY_MS = 1000.0f;
    inline constexpr float MIN_VISION_REQUEST_INTERVAL_MS = 100.0f;
    inline constexpr float MAX_VISION_REQUEST_INTERVAL_MS = 1500.0f;
    inline constexpr float MIN_VISION_REJECT_DIST = 0.5f;
    inline constexpr float MAX_VISION_REJECT_DIST = 8.0f;

    [[nodiscard]] inline bool repair_if_outside(float& value, float min_value, float max_value, float default_value) {
        if (value >= min_value && value <= max_value) {
            return false;
        }

        value = default_value;
        return true;
    }

    [[nodiscard]] inline bool clamp_if_outside(float& value, float min_value, float max_value, float default_value) {
        if (value >= min_value && value <= max_value) {
            return false;
        }

        if (value < min_value) {
            value = min_value;
        } else if (value > max_value) {
            value = max_value;
        } else {
            value = default_value;
        }
        return true;
    }

    [[nodiscard]] inline bool sanitize(TuningConfig& config) {
        bool changed = false;

        changed = repair_if_outside(config.dynamics.brake_limit,
                                    MIN_BRAKE_LIMIT,
                                    MAX_BRAKE_LIMIT,
                                    DEFAULT_BRAKE_LIMIT) || changed;
        changed = repair_if_outside(config.tracker.reach_radius_min,
                                    MIN_REACH_RADIUS_MIN,
                                    MAX_REACH_RADIUS_MIN,
                                    DEFAULT_REACH_RADIUS_MIN) || changed;
        changed = repair_if_outside(config.tracker.corner_pass_speed,
                                    MIN_CORNER_PASS_SPEED,
                                    MAX_CORNER_PASS_SPEED,
                                    DEFAULT_CORNER_PASS_SPEED) || changed;
        changed = repair_if_outside(config.tracker.vision_request_interval_ms,
                                    MIN_VISION_REQUEST_INTERVAL_MS,
                                    MAX_VISION_REQUEST_INTERVAL_MS,
                                    DEFAULT_VISION_REQUEST_INTERVAL_MS) || changed;
        changed = repair_if_outside(config.tracker.vision_reject_dist,
                                    MIN_VISION_REJECT_DIST,
                                    MAX_VISION_REJECT_DIST,
                                    DEFAULT_VISION_REJECT_DIST) || changed;
        changed = clamp_if_outside(config.latency.encoder_latency_gain,
                                   MIN_ENCODER_LATENCY_GAIN,
                                   MAX_ENCODER_LATENCY_GAIN,
                                   DEFAULT_ENCODER_LATENCY_GAIN) || changed;
        changed = clamp_if_outside(config.latency.vision_latency_ms,
                                   MIN_VISION_LATENCY_MS,
                                   MAX_VISION_LATENCY_MS,
                                   DEFAULT_VISION_LATENCY_MS) || changed;

        return changed;
    }
}

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
        TuningDefaults::DEFAULT_BRAKE_LIMIT  // brake_limit
    },
    {
        0.3f,    // reach_radius
        TuningDefaults::DEFAULT_REACH_RADIUS_MIN,   // reach_radius_min
        TuningDefaults::DEFAULT_CORNER_PASS_SPEED,  // corner_pass_speed
        0.7f,    // corner_switch_window
        1.0f,    // corner_line_tolerance
        250.0f,  // vision_request_interval_ms
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
    },
    {
        TuningDefaults::DEFAULT_ENCODER_LATENCY_GAIN,  // encoder_latency_gain
        TuningDefaults::DEFAULT_VISION_LATENCY_MS      // vision_latency_ms
    }
};
