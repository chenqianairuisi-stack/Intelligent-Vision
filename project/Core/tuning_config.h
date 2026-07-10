#pragma once

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
        float vision_request_interval_ms; // Reserved: ART1 pose stream is requested once, then consumed continuously
        float vision_reject_dist;     // 视觉轴向校正最大接受误差 cm
        float ang_tolerance;          // 航向角死区 rad
        float corner_pause_speed;     // 拐点略停切换阈值 cm/s（合速度低于此即切下一段，替代等完全停稳）
    } tracker;

    // Stanley 式平移横向纠偏（贴路径线跟踪，运动控制阶段新增）
    struct {
        bool  enable;        // 总开关：false 则退回纯朝目标点收敛
        float k_ct;          // 横向误差增益
        float k_soft;        // 软化速度 cm/s（防静止除零、低速平滑）
        float v_lat_max;     // 横纠速度上限 cm/s
    } stanley;

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
        float vision_latency_ms;    // Vision pose pipeline latency estimate ms (估计失效时的回退默认值)

        // 拐点对齐的实时视觉延时估计 (foundation 阶段新增)
        bool  enable_estimation;    // 总开关：false 则固定使用 vision_latency_ms
        float turn_thresh_deg;      // 拐点触发：速度矢量转角阈值 deg
        float enc_v_min;            // 编码器速度门 cm/20ms-tick（低于此判为静止/原地旋转）
        float vis_v_min;            // 视觉速度门 cm/frame
        float refractory_ms;        // 同一拐点去抖最小间隔 ms
        float l_min_ms;             // 估计延时接受下限 ms
        float l_max_ms;             // 估计延时接受上限 ms
        float dtheta_tol_deg;       // 编码器/视觉转角量级一致性容差 deg
        float lowpass_alpha;        // L 低通系数 (0~1)
        float l_stale_ms;           // L 过期时间，超时回退默认 ms
    } latency;

    // 炸弹推入后按需等待爆炸（运动控制阶段新增）
    struct {
        float explosion_wait_ms;    // 引信时长：仅当下一步需穿过被炸墙时等待 ms（上车实测标定）
    } bomb;

    // 刹车/切向手感（2026-07-04 追加在结构体末尾：不 bump magic，靠 sanitize 把旧 flash
    // 读出的越界/垃圾值兜回默认——见 storage.cpp。上车可菜单/!T G/!T A 直接调并 Save 持久化）
    struct {
        float brake_hold_gain;   // 主动刹车前馈增益：减速/锁定时残余轮速差→制动占空比，大=刹更狠锁更死
        float corner_turn_acc;   // 切向方向变化加速度限 cm/s^2：大=到点切得更直不磨圆，小=过弯更柔
    } feel;

    // Yaw 轴控制链（2026-07-10 从 Branch 融合）：追加在结构体**末尾**，不 bump magic，
    // 旧 flash 无此区、memcpy 读出 0/垃圾 → 由 sanitize 兜回默认（见 storage.cpp 尾部追加约定）。
    // YawProfiled 三层规划(sqrt远端/线性近端/陀螺阻尼)专用，替代旧 pid_yaw.kp + ff.k_stiction。
    struct {
        float lin_band;        // yaw 误差线性带宽 rad：带内用有界增益线性律替 sqrt，消除近端发散
        float kd;              // 陀螺阻尼系数：对 IMU 实测角速度做微分反馈，抑制冲过/回摆
        float translate_gain;  // 平移段 yaw 纠偏速度倍率
        float kd_translate;    // 平移段陀螺阻尼系数
        float stiction;        // 原地转向静摩擦补偿角速度 rad/s
    } yaw;
};

namespace TuningDefaults {
    inline constexpr float DEFAULT_REACH_RADIUS_MIN = 1.00f;  // 终点锁窗口需 > 单拍滑行量，太小(如0.2)会被穿窗导致反向追点震荡
    inline constexpr float DEFAULT_CORNER_PASS_SPEED = 0.0f;   // 过弯保留速度 cm/s：保持 0，不靠 end_speed 带速斜切
    inline constexpr float DEFAULT_BRAKE_LIMIT = 0.65f;  // 别贪高：过大减速超出轮速环带宽→打滑过冲+原地晃
    inline constexpr float DEFAULT_ENCODER_LATENCY_GAIN = 1.00f;
    inline constexpr float DEFAULT_VISION_LATENCY_MS = 310.0f;
    inline constexpr float DEFAULT_VISION_REQUEST_INTERVAL_MS = 100.0f;
    inline constexpr float DEFAULT_VISION_REJECT_DIST = 3.0f;

    // 拐点对齐实时延时估计默认参数
    inline constexpr bool  DEFAULT_LATENCY_EST_ENABLE = true;
    inline constexpr float DEFAULT_TURN_THRESH_DEG    = 45.0f;
    inline constexpr float DEFAULT_ENC_V_MIN          = 0.30f;   // cm/20ms ≈ 15 cm/s
    inline constexpr float DEFAULT_VIS_V_MIN          = 0.50f;   // cm/frame
    inline constexpr float DEFAULT_REFRACTORY_MS      = 100.0f;
    inline constexpr float DEFAULT_L_MIN_MS           = 50.0f;
    inline constexpr float DEFAULT_L_MAX_MS           = 400.0f;
    inline constexpr float DEFAULT_DTHETA_TOL_DEG     = 30.0f;
    inline constexpr float DEFAULT_L_LOWPASS_ALPHA    = 0.30f;
    inline constexpr float DEFAULT_L_STALE_MS         = 5000.0f;

    // 运动控制阶段默认参数
    inline constexpr float DEFAULT_CORNER_PAUSE_SPEED = 5.0f;  // cm/s：仅 force_stop/逐点停车模式使用的略停切段阈值，默认连贯模式不走它
    inline constexpr float DEFAULT_DYNAMICS_MAX_VEL = 150.0f;  // cm/s
    inline constexpr bool  DEFAULT_STANLEY_ENABLE     = false;  // 带速切向靠横移过渡，Stanley 再叠横纠会在拐点抖，默认关
    inline constexpr float DEFAULT_STANLEY_K_CT       = 1.2f;
    inline constexpr float DEFAULT_STANLEY_K_SOFT     = 20.0f;  // cm/s
    inline constexpr float DEFAULT_STANLEY_V_LAT_MAX  = 40.0f;  // cm/s
    inline constexpr float DEFAULT_EXPLOSION_WAIT_MS  = 900.0f; // ms（推炸弹后原地等墙炸开的时长；墙炸开约~1s；上车用 !T B 按实测标定）

    // 刹车/切向手感默认（追加末尾，不入 magic）
    inline constexpr float DEFAULT_BRAKE_HOLD_GAIN = 0.30f;   // 主动刹车前馈增益
    inline constexpr float DEFAULT_CORNER_TURN_ACC = 250.0f;  // 切向方向变化加速度限 cm/s^2

    // Yaw 轴控制链默认（2026-07-10 从 Branch 融合，追加末尾不入 magic）
    inline constexpr float DEFAULT_YAW_LIN_BAND       = 0.18f;  // rad，约 7 度线性带
    inline constexpr float DEFAULT_YAW_KD             = 0.14f;  // 陀螺阻尼，先小后逐步加大
    inline constexpr float DEFAULT_YAW_TRANSLATE_GAIN = 1.00f;  // 平移途中增强航向纠偏
    inline constexpr float DEFAULT_YAW_KD_TRANSLATE   = 0.36f;  // 平移途中减小阻尼削弱
    inline constexpr float DEFAULT_YAW_STICTION       = 0.24f;  // 原地转向静摩擦补偿

    inline constexpr float MIN_BRAKE_LIMIT = 0.0f;
    inline constexpr float MAX_BRAKE_LIMIT = 1.0f;
    inline constexpr float MIN_DYNAMICS_MAX_VEL = 0.0f;
    inline constexpr float MAX_DYNAMICS_MAX_VEL = 500.0f;
    inline constexpr float MIN_REACH_RADIUS_MIN = 0.10f;
    inline constexpr float MAX_REACH_RADIUS_MIN = 3.0f;
    inline constexpr float MIN_CORNER_PASS_SPEED = 0.0f;
    inline constexpr float MAX_CORNER_PASS_SPEED = 80.0f;
    inline constexpr float MIN_CORNER_PAUSE_SPEED = 0.0f;
    inline constexpr float MAX_CORNER_PAUSE_SPEED = 50.0f;
    inline constexpr float MIN_STANLEY_K_CT = 0.0f;
    inline constexpr float MAX_STANLEY_K_CT = 10.0f;
    inline constexpr float MIN_STANLEY_K_SOFT = 0.1f;
    inline constexpr float MAX_STANLEY_K_SOFT = 200.0f;
    inline constexpr float MIN_STANLEY_V_LAT_MAX = 0.0f;
    inline constexpr float MAX_STANLEY_V_LAT_MAX = 200.0f;
    inline constexpr float MIN_EXPLOSION_WAIT_MS = 0.0f;
    inline constexpr float MAX_EXPLOSION_WAIT_MS = 10000.0f;
    inline constexpr float MIN_ENCODER_LATENCY_GAIN = 0.01f;
    inline constexpr float MAX_ENCODER_LATENCY_GAIN = 2.00f;
    inline constexpr float MIN_VISION_LATENCY_MS = 0.0f;
    inline constexpr float MAX_VISION_LATENCY_MS = 1000.0f;
    inline constexpr float MIN_VISION_REQUEST_INTERVAL_MS = 100.0f;
    inline constexpr float MAX_VISION_REQUEST_INTERVAL_MS = 1500.0f;
    inline constexpr float MIN_VISION_REJECT_DIST = 0.5f;
    inline constexpr float MAX_VISION_REJECT_DIST = 8.0f;

    // 刹车/切向手感范围：MIN 取 >0，使旧 flash 追加区读出的 0/垃圾值必被 sanitize 兜回默认
    inline constexpr float MIN_BRAKE_HOLD_GAIN = 0.01f;
    inline constexpr float MAX_BRAKE_HOLD_GAIN = 2.0f;
    inline constexpr float MIN_CORNER_TURN_ACC = 20.0f;
    inline constexpr float MAX_CORNER_TURN_ACC = 1000.0f;

    // Yaw 轴控制链范围：MIN 取 >0，旧 flash 尾部追加区读出的 0/垃圾值必被 sanitize 兜回默认
    inline constexpr float MIN_YAW_LIN_BAND       = 0.02f;  // 过小会退回 sqrt 的无穷斜率问题
    inline constexpr float MAX_YAW_LIN_BAND       = 1.0f;
    inline constexpr float MIN_YAW_KD             = 0.0f;   // 允许关阻尼
    inline constexpr float MAX_YAW_KD             = 2.0f;
    inline constexpr float MIN_YAW_TRANSLATE_GAIN = 0.1f;
    inline constexpr float MAX_YAW_TRANSLATE_GAIN = 3.0f;
    inline constexpr float MIN_YAW_KD_TRANSLATE   = 0.0f;   // 允许关阻尼
    inline constexpr float MAX_YAW_KD_TRANSLATE   = 2.0f;
    inline constexpr float MIN_YAW_STICTION       = 0.0f;   // 允许关静摩擦补偿
    inline constexpr float MAX_YAW_STICTION       = 2.0f;

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
        changed = clamp_if_outside(config.dynamics.max_vel,
                                   MIN_DYNAMICS_MAX_VEL,
                                   MAX_DYNAMICS_MAX_VEL,
                                   DEFAULT_DYNAMICS_MAX_VEL) || changed;
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

        // 运动控制阶段参数修复
        changed = clamp_if_outside(config.tracker.corner_pause_speed,
                                   MIN_CORNER_PAUSE_SPEED,
                                   MAX_CORNER_PAUSE_SPEED,
                                   DEFAULT_CORNER_PAUSE_SPEED) || changed;
        changed = clamp_if_outside(config.stanley.k_ct,
                                   MIN_STANLEY_K_CT,
                                   MAX_STANLEY_K_CT,
                                   DEFAULT_STANLEY_K_CT) || changed;
        changed = clamp_if_outside(config.stanley.k_soft,
                                   MIN_STANLEY_K_SOFT,
                                   MAX_STANLEY_K_SOFT,
                                   DEFAULT_STANLEY_K_SOFT) || changed;
        changed = clamp_if_outside(config.stanley.v_lat_max,
                                   MIN_STANLEY_V_LAT_MAX,
                                   MAX_STANLEY_V_LAT_MAX,
                                   DEFAULT_STANLEY_V_LAT_MAX) || changed;
        changed = clamp_if_outside(config.bomb.explosion_wait_ms,
                                   MIN_EXPLOSION_WAIT_MS,
                                   MAX_EXPLOSION_WAIT_MS,
                                   DEFAULT_EXPLOSION_WAIT_MS) || changed;

        // 追加末尾字段：旧 flash 无此区，memcpy 读出 0/垃圾 → 这里兜回默认（免 bump magic）
        changed = clamp_if_outside(config.feel.brake_hold_gain,
                                   MIN_BRAKE_HOLD_GAIN,
                                   MAX_BRAKE_HOLD_GAIN,
                                   DEFAULT_BRAKE_HOLD_GAIN) || changed;
        changed = clamp_if_outside(config.feel.corner_turn_acc,
                                   MIN_CORNER_TURN_ACC,
                                   MAX_CORNER_TURN_ACC,
                                   DEFAULT_CORNER_TURN_ACC) || changed;

        // Yaw 轴控制链字段：同为尾部追加区，旧 flash 读出 0/垃圾/NaN → 兜回默认（免 bump magic）
        changed = clamp_if_outside(config.yaw.lin_band,
                                   MIN_YAW_LIN_BAND,
                                   MAX_YAW_LIN_BAND,
                                   DEFAULT_YAW_LIN_BAND) || changed;
        changed = clamp_if_outside(config.yaw.kd,
                                   MIN_YAW_KD,
                                   MAX_YAW_KD,
                                   DEFAULT_YAW_KD) || changed;
        changed = clamp_if_outside(config.yaw.translate_gain,
                                   MIN_YAW_TRANSLATE_GAIN,
                                   MAX_YAW_TRANSLATE_GAIN,
                                   DEFAULT_YAW_TRANSLATE_GAIN) || changed;
        changed = clamp_if_outside(config.yaw.kd_translate,
                                   MIN_YAW_KD_TRANSLATE,
                                   MAX_YAW_KD_TRANSLATE,
                                   DEFAULT_YAW_KD_TRANSLATE) || changed;
        changed = clamp_if_outside(config.yaw.stiction,
                                   MIN_YAW_STICTION,
                                   MAX_YAW_STICTION,
                                   DEFAULT_YAW_STICTION) || changed;

        return changed;
    }
}

// 全局调参实例，放在 DTCM 区域，供高频控制和业务模块访问
__attribute__((section(".dtcm_data"))) inline TuningConfig tune {
    {3.7f, 0.0f, 0.0f},      // pid_yaw
    {0.45f, 0.08f, 0.0f},    // pid_speed
    {0.2f, 4.0f, 0.54f},     // feedforward
    {
        80.0f,   // max_duty
        TuningDefaults::DEFAULT_DYNAMICS_MAX_VEL,  // max_vel cm/s
        65.0f,   // max_acc
        10.0f,   // max_ang_vel  —— 与 Branch 对齐：sqrt 型 yaw 规划需要更高角速度上限
        40.0f,   // max_ang_acc  —— 与 Branch 对齐：决定 sqrt(2·a·err) 曲线激进程度与近端线性带斜率
        1.044f,  // kinematic_gain_x
        1.015f,  // kinematic_gain_y
        TuningDefaults::DEFAULT_BRAKE_LIMIT  // brake_limit
    },
    {
        0.3f,    // reach_radius
        TuningDefaults::DEFAULT_REACH_RADIUS_MIN,   // reach_radius_min
        TuningDefaults::DEFAULT_CORNER_PASS_SPEED,  // corner_pass_speed
        2.0f,    // corner_switch_window —— 带速过弯模式提前切段窗口，上限 8cm，!SN 调
        0.7f,    // corner_line_tolerance
        TuningDefaults::DEFAULT_VISION_REQUEST_INTERVAL_MS,  // vision_request_interval_ms
        3.0f,    // vision_reject_dist
        0.010f,  // ang_tolerance —— 与 Branch 对齐：yaw 死区（配合陀螺阻尼压近端抖动）
        TuningDefaults::DEFAULT_CORNER_PAUSE_SPEED  // corner_pause_speed —— force_stop/逐点停车略停阈值；默认连贯模式不使用
    },
    {
        false,                                    // stanley.enable —— 带速切向时关，避免横纠叠加拐点抖
        TuningDefaults::DEFAULT_STANLEY_K_CT,       // stanley.k_ct
        TuningDefaults::DEFAULT_STANLEY_K_SOFT,     // stanley.k_soft
        TuningDefaults::DEFAULT_STANLEY_V_LAT_MAX   // stanley.v_lat_max
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
        TuningDefaults::DEFAULT_VISION_LATENCY_MS,     // vision_latency_ms
        TuningDefaults::DEFAULT_LATENCY_EST_ENABLE,    // enable_estimation
        TuningDefaults::DEFAULT_TURN_THRESH_DEG,       // turn_thresh_deg
        TuningDefaults::DEFAULT_ENC_V_MIN,             // enc_v_min
        TuningDefaults::DEFAULT_VIS_V_MIN,             // vis_v_min
        TuningDefaults::DEFAULT_REFRACTORY_MS,         // refractory_ms
        TuningDefaults::DEFAULT_L_MIN_MS,              // l_min_ms
        TuningDefaults::DEFAULT_L_MAX_MS,              // l_max_ms
        TuningDefaults::DEFAULT_DTHETA_TOL_DEG,        // dtheta_tol_deg
        TuningDefaults::DEFAULT_L_LOWPASS_ALPHA,       // lowpass_alpha
        TuningDefaults::DEFAULT_L_STALE_MS             // l_stale_ms
    },
    {
        TuningDefaults::DEFAULT_EXPLOSION_WAIT_MS      // bomb.explosion_wait_ms
    },
    {
        TuningDefaults::DEFAULT_BRAKE_HOLD_GAIN,       // feel.brake_hold_gain
        TuningDefaults::DEFAULT_CORNER_TURN_ACC        // feel.corner_turn_acc
    },
    {
        TuningDefaults::DEFAULT_YAW_LIN_BAND,          // yaw.lin_band
        TuningDefaults::DEFAULT_YAW_KD,                // yaw.kd
        TuningDefaults::DEFAULT_YAW_TRANSLATE_GAIN,    // yaw.translate_gain
        TuningDefaults::DEFAULT_YAW_KD_TRANSLATE,      // yaw.kd_translate
        TuningDefaults::DEFAULT_YAW_STICTION           // yaw.stiction
    }
};
