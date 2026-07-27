#pragma once

// ==========================================
// 末端刹车模式开关（两个独立状态机，编译期切换）
// ==========================================
// 停车段末端减速曲线有两套实现，各由下面一个开关单独控制，互不依赖：
//   · ENABLE_NONLINEAR_TERMINAL_BRAKE：非线性（接近区双段 sqrt 提前刹车，见 tune.approach_*）
//   · ENABLE_LINEAR_TERMINAL_BRAKE   ：距离归一化曲线（当前用 k^1.5 整形，见 LinearTerminalConfig）
// 用哪套就把对应开关置 true、另一套置 false。两个都 false → 退回原始单段 sqrt 直落。
// 两个都 true 时 k^1.5 曲线优先（双段 sqrt 分支不进），避免两条曲线互相打架。
namespace MotionFeatureSwitches {
    inline constexpr bool ENABLE_NONLINEAR_TERMINAL_BRAKE = false;  // 非线性双段 sqrt 末端刹车
    inline constexpr bool ENABLE_LINEAR_TERMINAL_BRAKE    = true;   // k^1.5 整形末端刹车
}

// 末端速度参数（距离归一化沿用 2he-new，当前对 k 做 1.5 次方整形）
namespace LinearTerminalConfig {
    inline constexpr float CRUISE_SPEED_CM_S = 100.0f;         // 末端接近巡航速度 cm/s
    inline constexpr float SHORT_MIN_SPEED_CM_S = 15.0f;       // 短段到达前最低速度 cm/s
    inline constexpr float SHORT_SLOWDOWN_DIST_CM = 15.0f;     // 短段开始减速距离 cm
    inline constexpr float LONG_MIN_SPEED_CM_S = 12.0f;        // 长段到达前最低速度 cm/s
    inline constexpr float LONG_SLOWDOWN_DIST_CM = 35.0f;      // 长段开始减速距离 cm
    inline constexpr float STOP_DIST_CM = 2.0f;                // 到达判定距离 cm，须小于两类减速距离
    inline constexpr float LONG_SEGMENT_THRESHOLD_CM = 60.0f;  // 段长 >= 此值为长段，短段不含边界
    inline constexpr float LONG_CRUISE_GAIN = 1.50f;           // 长直段巡航速度倍率
    inline constexpr float LONG_MAX_CRUISE_CM_S = 150.0f;      // 长直段巡航速度封顶 cm/s
    inline constexpr float LONG_ACCEL_GAIN = 1.40f;            // 长直段加速倍率，对齐 2he-new
    inline constexpr float DECEL_STEP_CM_S = 36.0f;            // 每个控制周期最大降速 cm/s

    static_assert(SHORT_SLOWDOWN_DIST_CM > STOP_DIST_CM,
                  "short terminal slowdown distance must exceed stop distance");
    static_assert(LONG_SLOWDOWN_DIST_CM > STOP_DIST_CM,
                  "long terminal slowdown distance must exceed stop distance");
}

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

// 单轮速度环与前馈参数（从 Branch 搬入：每轮独立整定，治四轮跟踪不齐→平移歪）
// pid 走每轮独立 kp/ki/kd；kv 速度前馈、ks 静摩擦补偿常态生效；ka/kb 为目标加速度前馈
// （加速用 ka、刹车用 kb），阶段1 暂不喂 target_acc，故 ka/kb 暂不参与，留待阶段2。
struct WheelControlParams {
    PidParams pid;  // 单轮速度 PID
    float kv;       // 速度前馈系数
    float ka;       // 目标加速度前馈系数（加速工况）
    float kb;       // 目标刹车加速度前馈系数（减速工况）
    float ks;       // 静摩擦补偿系数
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

    // 每轮独立轮速环参数（顺序 LF, LB, RF, RB）。追加在结构体**末尾**，不 bump magic：
    // 旧 flash 无此区、memcpy 读出垃圾 → 由 sanitize 兜回 Branch 调好的默认值（见 storage.cpp 尾部约定）。
    // 新轮速环走此表；旧 pid_speed / ff.kv / ff.ka 字段保留但不再被速度内环使用。
    WheelControlParams wheels[4];

    // 【已废弃占位】停车近端线性带宽度 cm（2026-07-15 删 StopBand 功能：实测无用，停车减速回到
    // 纯 sqrt 直落）。字段**保留占位不删**：它后面还有 stop_approach_brake_gain，物理删除会使后者
    // 偏移前移 4 字节 → 破坏 flash 布局 → 被迫 bump magic 丢掉全部已调参数。保留字段+sanitize
    // 兜底=零风险；无任何代码读它。若未来 bump magic 时可一并物理删除。
    float stop_approach_band_cm;

    // 停车接近区刹车倍率（2026-07-15 追加在结构体**最末尾**，不 bump magic）：仅停车工况（is_stop，
    // 含 AUTO 推箱/终点）把速度规划的**每拍减速上限** max_dv_dec 乘上此倍率，让带速冲进停车航点的车
    // 更"抖"地掉速——一步压到 sqrt 目标曲线上，而不是被 brake_acc·dt 的温柔限幅卡住冲过点。
    // 只放大减速限幅、不动目标曲线/加速斜坡/过弯带速：车已在曲线上时限幅不 binding→零行为改变；
    // 车冲得快时才生效，进点速度低→固定 300ms 视觉延迟造成的过冲随之缩小（不靠加大视觉增益，避免自激）。
    // 1.0=完全等于旧行为；旧 flash 无此区、memcpy 读出 0/垃圾 → sanitize 兜回 1.0。上车 !T H 直接调。
    float stop_approach_brake_gain;

    // 短距离移动起步加速提升（2026-07-15 追加在结构体**最末尾**，不 bump magic）：非长段且段全长 <= short_seg_len_cm
    // 时，速度规划器**只**把加速斜坡上限抬到 short_seg_accel（刹车/sqrt 减速曲线保持温柔不变），让短段
    // 起步更快、不磨蹭；末端因刹车温柔不变而**自然物理慢下来**（不是把末端交给视觉控制，控制链仍是常规
    // 编码器+里程计闭环，只是慢了更准/更跟得上）。
    // 与其他尾部字段"默认零行为改变"不同，此二者**默认启用**（60/300，用户已实车拍板）。旧 flash 尾部
    // 读出 0/垃圾 → sanitize 用 repair 兜回 60/300（默认开）；上车 !T D / !T V 直接调，!T D 1≈关闭。
    float short_seg_len_cm;   // 短段高加速阈值 cm（仅非长段生效；1 以下≈关闭）
    float short_seg_accel;    // 短段起步加速度 cm/s^2（只压加速斜坡，不动刹车）

    // 停车接近区双段刹车（2026-07-15 追加在结构体**最末尾**，不 bump magic）：治"末端总是出去一些、
    // StopBrkG 大小都调不好"。根因=过冲发生在 hard_lock 视觉冻结窗口内，事后又有粘滞锁死区兜着，
    // 靠单一 sqrt 曲线+减速限幅永远调不准。修法=**提前刹车**：停车段最后 zone cm 换一条**更缓**的
    // sqrt 曲线（approach_brake_acc << brake_acc），外段正常刹车曲线落到缓曲线上（C0 连续）——
    // 车末端明显慢下来（非固定爬行速度），低速下视觉延迟误差≈速度×310ms 变小、编码器不打滑，
    // 融合在进点前就收敛到真实位置。zone = min(approach_zone_cm, 段全长×approach_zone_ratio)：
    // 长距离提前 40cm、20cm 段提前 5cm（用户拍板）。**默认启用**；!T Z 0.5≈关闭（回纯 sqrt 直落）。
    // 旧 flash 尾部读出 0/垃圾 → sanitize 用 repair 兜回默认（默认开）。!T Z / !T R / !T C 在线调。
    float approach_zone_cm;     // 接近区提前刹车距离上限 cm（默认 40；0.5≈关闭）
    float approach_zone_ratio;  // 接近区占段全长比例（默认 0.25：20cm 段→5cm，160cm+→封顶 40cm）
    float approach_brake_acc;   // 接近区缓减速度 cm/s^2（默认 15，须 < brake_acc 才生效）
    float approach_enable;      // 接近区总开关：>=0.5 开、<0.5 关（菜单/串口一键切；默认 1=开）。
                                // 用 float 存（沿用尾部 float 追加约定，免为一个 bool 破坏 flash 布局）
};

namespace TuningDefaults {
    inline constexpr float DEFAULT_REACH_RADIUS_MIN = 1.00f;  // 终点锁窗口需 > 单拍滑行量，太小(如0.2)会被穿窗导致反向追点震荡
    inline constexpr float DEFAULT_CORNER_PASS_SPEED = 0.0f;   // 过弯保留速度 cm/s：保持 0，不靠 end_speed 带速斜切
    inline constexpr float DEFAULT_BRAKE_LIMIT = 0.65f;  // 别贪高：过大减速超出轮速环带宽→打滑过冲+原地晃
    inline constexpr float DEFAULT_ENCODER_LATENCY_GAIN = 1.00f;
    inline constexpr float DEFAULT_VISION_LATENCY_MS = 310.0f;
    inline constexpr float DEFAULT_VISION_REQUEST_INTERVAL_MS = 100.0f;
    inline constexpr float DEFAULT_VISION_REJECT_DIST = 5.0f;  // 与 FULL_MAX_STEP_CM=5 匹配：差<5cm 都能一帧纠到位

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

    // 【已废弃占位】停车近端线性带默认：字段保留占位（见结构体处注释），无代码读它，
    // 仅供 sanitize 把 flash 读出的垃圾值兜回有限值，避免 NaN 存回 flash。
    inline constexpr float DEFAULT_STOP_APPROACH_BAND_CM = 3.0f;  // cm

    // 停车接近区刹车倍率默认（最末尾追加，不入 magic）：1.0 = 每拍减速上限不变 = 旧行为，零风险。
    // 上车若"到航点冲过头"就把它调大（2~4），让冲进停车航点的车更抖地掉速、进点更慢、过冲更小。
    inline constexpr float DEFAULT_STOP_APPROACH_BRAKE_GAIN = 1.0f;  // ×

    // 短距离起步加速提升默认（最末尾追加，不入 magic）：**默认启用** 60cm / 300cm/s^2（用户实车拍板）。
    // 段全长 <= 60cm 的移动起步加速抬到 300，只压加速斜坡、不动刹车曲线。
    inline constexpr float DEFAULT_SHORT_SEG_LEN_CM = 60.0f;   // cm
    inline constexpr float DEFAULT_SHORT_SEG_ACCEL  = 300.0f;  // cm/s^2

    // 停车接近区双段刹车默认（最末尾追加，不入 magic）：**默认启用**（用户拍板"末端要提前刹车"）。
    // zone = min(40, 段全长×0.25)：20cm 段→5cm、160cm+ 段→封顶 40cm；接近区内走 15cm/s^2 缓 sqrt，
    // 末端明显慢下来让视觉+编码器融合收敛。!T Z 0.5≈关闭（回纯 sqrt 直落）。
    inline constexpr float DEFAULT_APPROACH_ZONE_CM    = 40.0f;  // cm
    inline constexpr float DEFAULT_APPROACH_ZONE_RATIO = 0.25f;  // ×段全长
    inline constexpr float DEFAULT_APPROACH_BRAKE_ACC  = 15.0f;  // cm/s^2
    inline constexpr float DEFAULT_APPROACH_ENABLE     = 1.0f;   // 1=开（默认启用）

    // Yaw 轴控制链默认（2026-07-10 从 Branch 融合，追加末尾不入 magic）
    inline constexpr float DEFAULT_YAW_LIN_BAND       = 0.18f;  // rad，约 7 度线性带
    inline constexpr float DEFAULT_YAW_KD             = 0.14f;  // 陀螺阻尼，先小后逐步加大
    inline constexpr float DEFAULT_YAW_TRANSLATE_GAIN = 1.00f;  // 平移途中增强航向纠偏
    inline constexpr float DEFAULT_YAW_KD_TRANSLATE   = 0.36f;  // 平移途中减小阻尼削弱
    inline constexpr float DEFAULT_YAW_STICTION       = 0.24f;  // 原地转向静摩擦补偿

    // 每轮独立轮速环默认（顺序 LF, LB, RF, RB）：直接取 Branch 在**同一辆车**上调好的值。
    // 格式 {{kp,ki,kd}, kv, ka, kb, ks}。旧 flash 尾部无此区、读出垃圾 → sanitize 兜回这里。
    inline constexpr WheelControlParams DEFAULT_WHEELS[4] = {
        {{0.72f, 0.56f, 0.0f}, 0.046f, 0.020f, 0.030f, 5.0f},  // LF
        {{0.65f, 0.56f, 0.0f}, 0.054f, 0.016f, 0.024f, 5.0f},  // LB
        {{0.64f, 0.74f, 0.0f}, 0.052f, 0.014f, 0.028f, 5.0f},  // RF
        {{0.74f, 0.56f, 0.0f}, 0.062f, 0.024f, 0.032f, 6.4f},  // RB
    };

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

    // 停车近端线性带范围：MIN 取 >0，使旧 flash 追加区读出的 0/垃圾值必被 sanitize 兜回默认
    inline constexpr float MIN_STOP_APPROACH_BAND_CM = 0.5f;
    inline constexpr float MAX_STOP_APPROACH_BAND_CM = 30.0f;

    // 停车接近区刹车倍率范围：MIN 取 1.0（=零行为改变下限），旧 flash 追加区读出的 0/垃圾（<1）被
    // clamp 兜回 1.0=旧行为，绝不会读成"减速比旧行为还慢"的危险值；上限 100.0。
    inline constexpr float MIN_STOP_APPROACH_BRAKE_GAIN = 1.0f;
    inline constexpr float MAX_STOP_APPROACH_BRAKE_GAIN = 100.0f;

    // 短距离起步加速提升范围：MIN 取 >0，使旧 flash 尾部读出的 0/NaN/垃圾被 repair 兜回默认（=默认启用）。
    inline constexpr float MIN_SHORT_SEG_LEN_CM = 1.0f;      // <1cm≈关闭（无真实段这么短，最小移动 20cm）
    inline constexpr float MAX_SHORT_SEG_LEN_CM = 200.0f;
    inline constexpr float MIN_SHORT_SEG_ACCEL  = 20.0f;
    inline constexpr float MAX_SHORT_SEG_ACCEL  = 1000.0f;

    // 停车接近区双段刹车范围：MIN 取 >0，使旧 flash 尾部读出的 0/NaN/垃圾被 repair 兜回默认（=默认启用）。
    // zone_cm 允许调到 0.5（=运行期判定 <1 视为关闭）；ratio 上限 1.0（整段都算接近区）。
    inline constexpr float MIN_APPROACH_ZONE_CM    = 0.5f;
    inline constexpr float MAX_APPROACH_ZONE_CM    = 100.0f;
    inline constexpr float MIN_APPROACH_ZONE_RATIO = 0.05f;
    inline constexpr float MAX_APPROACH_ZONE_RATIO = 1.0f;
    inline constexpr float MIN_APPROACH_BRAKE_ACC  = 1.0f;
    inline constexpr float MAX_APPROACH_BRAKE_ACC  = 200.0f;
    // 接近区开关：合法区间 [0,1]，0=关/1=开。旧 flash 尾部无此字段读出 0/垃圾时——
    // 0 落在区间内会被当成"关"，这不安全（默认应为开）。故 sanitize 用 repair 而非 clamp：
    // 只有落在 {0,1} 附近才保留，其余（含旧 flash 的 0.0 之外的垃圾/NaN）弹回默认 1=开。
    // 注：旧 flash 该尾部区读出恰好 0.0 的概率极低（memcpy 越界读到的是相邻数据/0xFF→NaN），
    // 且首次 Save 后即写入真实 1.0，此后不再依赖兜底。
    inline constexpr float MIN_APPROACH_ENABLE     = 0.0f;
    inline constexpr float MAX_APPROACH_ENABLE     = 1.0f;

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

        // 每轮轮速环参数（尾部追加区）：旧 flash 无此区，memcpy 读出垃圾/NaN → 兜回 Branch 默认。
        // 判据：kp 或 ks 越界/NaN 视为该轮未初始化（真实整定 kp≥0.1、ks≥0.5），整轮兜回默认；
        // 其余字段（可为 0）各自限幅。NaN 因比较恒 false 会自动落入越界分支被兜回。
        for (int w = 0; w < 4; ++w) {
            WheelControlParams& wp = config.wheels[w];
            const WheelControlParams& def = DEFAULT_WHEELS[w];
            bool wheel_bad = !(wp.pid.kp >= 0.1f && wp.pid.kp <= 5.0f) ||
                             !(wp.ks     >= 0.5f && wp.ks     <= 30.0f);
            if (wheel_bad) {
                wp = def;
                changed = true;
                continue;
            }
            changed = clamp_if_outside(wp.pid.ki, 0.0f, 5.0f, def.pid.ki) || changed;
            changed = clamp_if_outside(wp.pid.kd, 0.0f, 2.0f, def.pid.kd) || changed;
            changed = clamp_if_outside(wp.kv,     0.0f, 1.0f, def.kv)     || changed;
            changed = clamp_if_outside(wp.ka,     0.0f, 1.0f, def.ka)     || changed;
            changed = clamp_if_outside(wp.kb,     0.0f, 1.0f, def.kb)     || changed;
        }

        // 【已废弃占位】停车近端线性带：无代码读它，仅兜回有限值防 NaN 存回 flash（字段保留占位见结构体注释）
        changed = clamp_if_outside(config.stop_approach_band_cm,
                                   MIN_STOP_APPROACH_BAND_CM,
                                   MAX_STOP_APPROACH_BAND_CM,
                                   DEFAULT_STOP_APPROACH_BAND_CM) || changed;

        // 停车接近区刹车倍率（结构体最末尾追加字段）：旧 flash 读出 0/垃圾（<MIN=1.0）被 clamp 兜回 1.0
        // = 旧行为，NaN 落 else 分支取默认 1.0，均安全（免 bump magic）
        changed = clamp_if_outside(config.stop_approach_brake_gain,
                                   MIN_STOP_APPROACH_BRAKE_GAIN,
                                   MAX_STOP_APPROACH_BRAKE_GAIN,
                                   DEFAULT_STOP_APPROACH_BRAKE_GAIN) || changed;

        // 短距离起步加速提升（结构体最末尾追加字段）：用 repair（非 clamp），使旧 flash 尾部读出的
        // 0/NaN/越界值统一兜回默认 60/300（=默认启用），而不会被 clamp 到边界。免 bump magic。
        changed = repair_if_outside(config.short_seg_len_cm,
                                    MIN_SHORT_SEG_LEN_CM,
                                    MAX_SHORT_SEG_LEN_CM,
                                    DEFAULT_SHORT_SEG_LEN_CM) || changed;
        changed = repair_if_outside(config.short_seg_accel,
                                    MIN_SHORT_SEG_ACCEL,
                                    MAX_SHORT_SEG_ACCEL,
                                    DEFAULT_SHORT_SEG_ACCEL) || changed;

        // 停车接近区双段刹车（结构体最末尾追加字段）：用 repair，旧 flash 尾部读出的 0/NaN/越界
        // 统一兜回默认 40/0.25/15（=默认启用）。免 bump magic。
        changed = repair_if_outside(config.approach_zone_cm,
                                    MIN_APPROACH_ZONE_CM,
                                    MAX_APPROACH_ZONE_CM,
                                    DEFAULT_APPROACH_ZONE_CM) || changed;
        changed = repair_if_outside(config.approach_zone_ratio,
                                    MIN_APPROACH_ZONE_RATIO,
                                    MAX_APPROACH_ZONE_RATIO,
                                    DEFAULT_APPROACH_ZONE_RATIO) || changed;
        changed = repair_if_outside(config.approach_brake_acc,
                                    MIN_APPROACH_BRAKE_ACC,
                                    MAX_APPROACH_BRAKE_ACC,
                                    DEFAULT_APPROACH_BRAKE_ACC) || changed;
        // 开关：clamp 到 [0,1]，NaN/越界（含旧 flash 尾部垃圾）落 else 分支取默认 1=开；
        // 0 和 1 都在区间内会被保留——用户存的"关(0)"不会被误弹回"开"。
        changed = clamp_if_outside(config.approach_enable,
                                   MIN_APPROACH_ENABLE,
                                   MAX_APPROACH_ENABLE,
                                   DEFAULT_APPROACH_ENABLE) || changed;

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
        TuningDefaults::DEFAULT_VISION_REJECT_DIST,  // vision_reject_dist
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
    },
    {
        TuningDefaults::DEFAULT_WHEELS[0],  // wheels[LF]
        TuningDefaults::DEFAULT_WHEELS[1],  // wheels[LB]
        TuningDefaults::DEFAULT_WHEELS[2],  // wheels[RF]
        TuningDefaults::DEFAULT_WHEELS[3]   // wheels[RB]
    },
    TuningDefaults::DEFAULT_STOP_APPROACH_BAND_CM,   // stop_approach_band_cm（结构体末尾追加）
    TuningDefaults::DEFAULT_STOP_APPROACH_BRAKE_GAIN, // stop_approach_brake_gain（结构体末尾追加）
    TuningDefaults::DEFAULT_SHORT_SEG_LEN_CM,        // short_seg_len_cm（结构体最末尾追加）
    TuningDefaults::DEFAULT_SHORT_SEG_ACCEL,         // short_seg_accel（结构体最末尾追加）
    TuningDefaults::DEFAULT_APPROACH_ZONE_CM,        // approach_zone_cm（结构体最末尾追加）
    TuningDefaults::DEFAULT_APPROACH_ZONE_RATIO,     // approach_zone_ratio（结构体最末尾追加）
    TuningDefaults::DEFAULT_APPROACH_BRAKE_ACC,      // approach_brake_acc（结构体最末尾追加）
    TuningDefaults::DEFAULT_APPROACH_ENABLE          // approach_enable（结构体最末尾追加）
};
