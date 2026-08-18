/// \file tuning_config.h
/// \brief 全车参数、运动策略和上位机注册表

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace MotionFeatureSwitches {

inline constexpr bool ENABLE_NONLINEAR_TERMINAL_BRAKE = false;
inline constexpr bool ENABLE_LINEAR_TERMINAL_BRAKE = true;

} // namespace MotionFeatureSwitches

namespace LinearTerminalConfig {

inline constexpr float CRUISE_SPEED_CM_S = 100.0f;
inline constexpr float SHORT_MIN_SPEED_CM_S = 20.0f;
inline constexpr float SHORT_SLOWDOWN_DIST_CM = 12.0f;
inline constexpr float LONG_MIN_SPEED_CM_S = 30.0f;
inline constexpr float LONG_SLOWDOWN_DIST_CM = 35.0f;
inline constexpr float STOP_DIST_CM = 0.8f;
inline constexpr float LONG_SEGMENT_THRESHOLD_CM = 60.0f;
inline constexpr float LONG_CRUISE_GAIN = 1.50f;
inline constexpr float LONG_MAX_CRUISE_CM_S = 200.0f;
inline constexpr float LONG_ACCEL_GAIN = 1.40f;
inline constexpr float DECEL_STEP_CM_S = 36.0f;
inline constexpr bool USE_K15_SHAPING = false;
inline constexpr bool AUTO_WINDOW_ENABLE = true;
inline constexpr float SLOWDOWN_SEG_RATIO = 0.50f;
inline constexpr float WINDOW_ACC_MARGIN = 0.80f;
inline constexpr bool USE_FIXED_DECEL_STEP = false;
inline constexpr bool USE_LEGACY_MIN_SPEED = true;
inline constexpr float TERMINAL_MIN_SPEED_CM_S = 12.0f;

static_assert(SHORT_SLOWDOWN_DIST_CM > STOP_DIST_CM,
              "short terminal slowdown distance must exceed stop distance");
static_assert(LONG_SLOWDOWN_DIST_CM > STOP_DIST_CM,
              "long terminal slowdown distance must exceed stop distance");

} // namespace LinearTerminalConfig

struct PidParams {
    float kp;
    float ki;
    float kd;
};

struct WheelControlParams {
    PidParams pid;
    float kv;                         // 速度前馈
    float ka;                         // 加速前馈
    float kb;                         // 制动前馈
    float ks;                         // 静摩擦补偿
};

inline constexpr std::size_t TUNING_WHEEL_COUNT = 4u;

/// \brief 控制、定位与比赛流程共用的运行期参数
///
/// \details
/// 结构体仅保留有实际消费者的字段，持久化格式由 Storage 的版本头管理
/// 新增或调整字段时必须同步修改默认值、参数注册表和 Storage 校验规则
struct TuningConfig {
    struct {
        float max_duty;               // 最大电机占空比 %
        float max_vel;                // 最大平移速度 cm/s
        float max_acc;                // 最大平移加速度 cm/s2
        float max_ang_vel;            // 最大角速度 rad/s
        float max_ang_acc;            // 最大角加速度 rad/s2
        float kinematic_gain_x;       // 车体 X 方向运动学标定倍率
        float kinematic_gain_y;       // 车体 Y 方向运动学标定倍率
        float brake_limit;            // 制动能力相对 max_acc 的比例
        // 轮胎-地面附着力决定的刹车减速度绝对上限 cm/s2，与 max_acc 无关。
        // brake_limit×max_acc 是"想要多强"，本项是"地面给得起多强"，实际取两者较小值。
        // 标定：从已知速度 v 全力刹车，量滑行距离 d，a = v^2/(2d)。
        // 默认按当前 max_acc×brake_limit 的 390cm/s2 作为基准，后续只需按实测制动距离校准本项。
        float brake_acc_ceiling;
    } dynamics;

    struct {
        float reach_radius;           // 普通路径点到达半径 cm
        float reach_radius_min;       // 最终目标点到达半径 cm
        float corner_pass_speed;      // 非终点过弯保留速度 cm/s
        float corner_switch_window;   // 拐角提前切换窗口 cm
        float corner_line_tolerance;  // 拐角切换横向允许误差 cm
        float vision_request_interval_ms; // ART1 位姿请求最小间隔 ms
        float vision_reject_dist;     // 视觉轴向校正最大接受误差 cm
        float ang_tolerance;          // 最终航向到达容差 rad
        float corner_pause_speed;     // 强制停车航点略停切换阈值 cm/s
    } tracker;

    struct {
        float mahony_kp;              // Mahony 姿态融合比例增益
    } estimate;

    struct {
        float encoder_latency_gain;   // 编码器位移延迟补偿倍率
        float vision_latency_ms;      // 视觉管线固定延时 ms：补偿唯一采用的 L
                                      // 2026-08-07 起拐点在线估计不再参与补偿，本键就是最终值
        float enable_estimation;      // 拐点延时测量开关，0 关闭，1 开启
                                      // 只影响遥测 mode 3 的 est_raw/est_filt，不进补偿链路
        float turn_thresh_deg;        // 拐点触发转角阈值 deg
        float enc_v_min;              // 编码器速度门 cm/20ms
        float vis_v_min;              // 视觉速度门 cm/frame
        float refractory_ms;          // 拐点检测不应期 ms
        float l_min_ms;               // 可接受延迟下限 ms
        float l_max_ms;               // 可接受延迟上限 ms
        float dtheta_tol_deg;         // 编码器与视觉转角匹配容差 deg
        float lowpass_alpha;          // 延迟估计低通系数
        float l_stale_ms;             // 延迟估计过期时间 ms
        // turn_thresh_deg 到 l_stale_ms 这一组同样只作用于上面的测量通路
        // 结构体尾部字段不删不移，避免改变偏移让已保存的 flash 配置整体失效
    } latency;

    struct {
        float explosion_wait_ms;      // 推炸弹后的最长等待时间 ms
    } bomb;

    struct {
        float lin_band;               // 偏航误差线性区半宽 rad
        float kd;                     // 原地旋转角速度阻尼
        float translate_gain;         // 平移偏航控制增益
        float kd_translate;           // 平移角速度阻尼
        float stiction;               // 原地旋转静摩擦补偿 rad/s
    } yaw;

    struct {
        float stop_brake_gain;        // 停车接近区每拍制动力倍率
        float short_segment_cm;       // 短路段判定阈值 cm
        float short_acceleration;     // 短路段起步加速度 cm/s2
        float approach_zone_cm;       // 终点接近区长度上限 cm
        float approach_ratio;         // 接近区占整段长度的比例
        float approach_acceleration;  // 接近区减速度 cm/s2
        float approach_enable;        // 接近区开关，0 关闭，1 开启
    } terminal;

    WheelControlParams wheels[TUNING_WHEEL_COUNT]; // LF、LB、RF、RB

    // 纵向（沿运动方向）视觉缓慢修正：段中间用延时补偿后的视觉纠编码器打滑累积，
    // 进入刹车/切向区后完全冻结、纯靠零延迟编码器收尾。横向纠偏与本组无关、全程不变。
    // 新字段一律追加在结构体尾部：中间插入会改变后续字段偏移，已保存的 Flash 配置
    // 会因 payload_size 不符被判无效而静默回默认值。
    struct {
        float enable;                 // 总开关，0 关闭（行为与改前逐位一致），1 开启
        float freeze_floor_cm;        // 冻结窗口硬下限 cm：与速度无关，由视觉自身精度决定。
                                      // 低速时 v*L 与刹车距离都塌到 1cm 以下，此时若仍让视觉推
                                      // 位置，±0.5cm 视觉噪声会灌进停车判定 → 自激振荡。
        float latency_window_gain;    // v*L 项倍率，1.0=严格按估计延迟；调小=更早开始信视觉
        float max_step_cm;            // 普通段单帧限步 cm。上限依据：位置跳变引起的速度指令
                                      // 跳变须远小于每拍降速限幅 brake_acc*dt
        float push_max_step_cm;       // 推箱段单帧限步 cm：接触面容不下速度突变，取更保守值
        float reject_dist_cm;         // 纵向粗差闸 cm：超过判为视觉异常本帧不修。必须大于待治的
                                      // 打滑累积量级（1~3cm），不能沿用横向的 vision_reject_dist
                                      // （默认 1cm），否则要修的误差全被判成误检、修正恒为 0
        float scale_learn_enable;     // 视觉里程比例在线学习开关，0 关闭，1 开启
        float scale_learn_alpha;      // 比例低通更新系数，越大适应越快
        float scale_sample_min_cm;    // 单次比例样本要求的最小直线位移 cm
        float scale_min;              // 学习比例下限，防止异常视觉把里程拉坏
        float scale_max;              // 学习比例上限，防止异常视觉把里程拉坏
    } vision_long;

    // 横向视觉修正独立调参，避免和纵向末端冻结及里程学习互相影响
    struct {
        float max_step_cm;            // 普通视觉帧沿段法向最多修正多少 cm
        float gain;                   // 横向误差本帧采用比例，1.0 表示限步后全部采用
    } vision_lateral;

    struct {
        float diagonal_move_enable;         // 斜向移动总开关，0 关闭，1 开启
        float box_extra_observe_enable;     // 箱子额外观测位总开关，0 关闭，1 开启
        float target_extra_observe_enable;  // 目标点额外观测位总开关，0 关闭，1 开启
    } planning_extra;
};

inline constexpr TuningConfig DEFAULT_TUNE_CONFIG = {
    {
        80.0f,   // dynamics.max_duty
        200.0f,  // dynamics.max_vel
        800.0f,  // dynamics.max_acc
        6.0f,   // dynamics.max_ang_vel
        36.0f,   // dynamics.max_ang_acc
        1.090f,  // dynamics.kinematic_gain_x
        1.000f,  // dynamics.kinematic_gain_y
        0.80f,   // dynamics.brake_limit
        450.0f,  // dynamics.brake_acc_ceiling：与当前 600×0.65 一致，不改变现有线性减速手感；
                 // 后续调高 max_acc 时刹车能力不再被同步虚高。
    },
    {
        0.3f,    // tracker.reach_radius
        1.0f,    // tracker.reach_radius_min
        0.0f,    // tracker.corner_pass_speed
        2.0f,    // tracker.corner_switch_window
        0.7f,    // tracker.corner_line_tolerance
        100.0f,  // tracker.vision_request_interval_ms
        5.0f,   // tracker.vision_reject_dist：横向误差接受范围，避免偏差超过 1cm 后反而完全不修
        0.010f,  // tracker.ang_tolerance
        5.0f,    // tracker.corner_pause_speed
    },
    {
        1.0f,    // estimate.mahony_kp
    },
    {
        1.0f,     // latency.encoder_latency_gain
        380.0f,   // latency.vision_latency_ms
        1.0f,     // latency.enable_estimation
        45.0f,    // latency.turn_thresh_deg
        0.30f,    // latency.enc_v_min
        0.50f,    // latency.vis_v_min
        100.0f,   // latency.refractory_ms
        50.0f,    // latency.l_min_ms
        400.0f,   // latency.l_max_ms
        30.0f,    // latency.dtheta_tol_deg
        0.30f,    // latency.lowpass_alpha
        5000.0f,  // latency.l_stale_ms
    },
    {
        900.0f, // bomb.explosion_wait_ms
    },
    {
        0.22f,    // yaw.lin_band
        0.22f,    // yaw.kd
        1.0f,     // yaw.translate_gain
        0.36f,    // yaw.kd_translate
        0.00f,    // yaw.stiction
    },
    {
        1.0f,     // terminal.stop_brake_gain
        60.0f,    // terminal.short_segment_cm
        600.0f,   // terminal.short_acceleration
        40.0f,    // terminal.approach_zone_cm
        0.25f,    // terminal.approach_ratio
        15.0f,    // terminal.approach_acceleration
        1.0f,     // terminal.approach_enable
    },
    {
        {{0.38f, 0.8f, 0.0f}, 0.10f, 0.020f, 0.012f, 1.0f},  // lf
        {{0.38f, 0.8f, 0.0f}, 0.10f, 0.020f, 0.012f, 1.0f},  // lb
        {{0.38f, 0.8f, 0.0f}, 0.11f, 0.020f, 0.016f, 1.0f},  // rf
        {{0.38f, 0.8f, 0.0f}, 0.10f, 0.020f, 0.016f, 1.0f},  // rb
    },
    {
        0.0f,     // vision_long.enable：默认开启，中远段缓慢修正纵向里程，末端冻结逻辑保持不变
        3.0f,     // vision_long.freeze_floor_cm
        0.5f,     // vision_long.latency_window_gain
        1.0f,     // vision_long.max_step_cm
        1.0f,     // vision_long.push_max_step_cm
        // reject_dist_cm 是"粗差闸"（超过即整帧丢弃），不是限速——限速是 max_step_cm。
        // 曾设 2.0：纵向误差一旦越过 2cm 就完全不修，与 15cm 硬重置之间留下 13cm 死区，
        // 误差越大越不修。长段累积必然越过 2cm → 修正开关式关闭 → 剩余段纯靠已跑偏的编码器
        // → 多跑；短段撑不到 2cm 所以一直准。取 12 与硬重置衔接，不留空档。
        12.0f,    // vision_long.reject_dist_cm
        1.0f,     // vision_long.scale_learn_enable
        // 比例是全局持久状态、乘在每个编码器增量上，误差正比于段长。alpha 越大波动越快、
        // run-to-run 差异越大（"偶尔多跑"的来源）。保守取 0.10。
        0.10f,    // vision_long.scale_learn_alpha
        10.0f,    // vision_long.scale_sample_min_cm
        0.85f,    // vision_long.scale_min
        1.15f,    // vision_long.scale_max
    },
    {
        3.0f,     // vision_lateral.max_step_cm
        1.8f,     // vision_lateral.gain
    },
    {
        1.0f,     // planning_extra.diagonal_move_enable
        0.0f,     // planning_extra.box_extra_observe_enable
        0.0f,     // planning_extra.target_extra_observe_enable
    },
};

// 高频控制直接读取该实例，存储加载时在短临界区内整体替换
__attribute__((section(".dtcm_data"))) inline TuningConfig tune = DEFAULT_TUNE_CONFIG;

// ============================================================================
// 屏幕与上位机共用的参数注册表
// ============================================================================
namespace TuningRegistry {

enum class ScreenEditGroup {
    SINGLE, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_KV, WHEEL_KA, WHEEL_KB, WHEEL_KS,
};

enum class SetResult { OK, UNKNOWN_KEY, OUT_OF_RANGE };

struct ParamItem {
    const char* name;
    const char* key;
    float* value;
    float step;
    float minimum;
    float maximum;
    ScreenEditGroup screen_group = ScreenEditGroup::SINGLE;
    bool screen_visible = true;
};

inline ParamItem params[] = {
    {"MaxDuty ", "MD", &tune.dynamics.max_duty, 1.0f, 1.0f, 100.0f},
    {"MaxVel  ", "MV", &tune.dynamics.max_vel, 1.0f, 1.0f, 500.0f},
    {"MaxAcc  ", "MA", &tune.dynamics.max_acc, 5.0f, 1.0f, 1000.0f},
    {"MaxAVel ", "AV", &tune.dynamics.max_ang_vel, 0.1f, 0.1f, 30.0f},
    {"MaxAAcc ", "AA", &tune.dynamics.max_ang_acc, 0.5f, 0.1f, 200.0f},
    {"GainX   ", "KX", &tune.dynamics.kinematic_gain_x, 0.001f, 0.5f, 1.5f},
    {"GainY   ", "KY", &tune.dynamics.kinematic_gain_y, 0.001f, 0.5f, 1.5f},
    {"BrakeLim", "BL", &tune.dynamics.brake_limit, 0.1f, 0.05f, 2.0f},
    {"BrakeCap", "BC", &tune.dynamics.brake_acc_ceiling, 10.0f, 50.0f, 5000.0f},
    {"Reach   ", "TR", &tune.tracker.reach_radius, 0.1f, 0.05f, 10.0f},
    {"ReachMin", "TM", &tune.tracker.reach_radius_min, 0.1f, 0.1f, 3.0f},
    {"TurnVel ", "CV", &tune.tracker.corner_pass_speed, 1.0f, 0.0f, 80.0f},
    {"TurnWin ", "CW", &tune.tracker.corner_switch_window, 0.1f, 0.0f, 20.0f},
    {"LineTol ", "CT", &tune.tracker.corner_line_tolerance, 0.1f, 0.0f, 10.0f},
    {"VisIntvl", "VI", &tune.tracker.vision_request_interval_ms, 10.0f, 20.0f, 1500.0f},
    {"VisRej  ", "VR", &tune.tracker.vision_reject_dist, 0.5f, 0.1f, 20.0f},
    {"AngTol  ", "AT", &tune.tracker.ang_tolerance, 0.001f, 0.001f, 0.5f},
    {"PauseSpd", "CP", &tune.tracker.corner_pause_speed, 0.5f, 0.0f, 50.0f},
    {"Mahony  ", "MK", &tune.estimate.mahony_kp, 0.01f, 0.0f, 10.0f},
    {"EncGain ", "EG", &tune.latency.encoder_latency_gain, 0.01f, 0.01f, 2.0f},
    {"VisLag  ", "VL", &tune.latency.vision_latency_ms, 10.0f, 0.0f, 1000.0f},
    {"LagEn   ", "LE", &tune.latency.enable_estimation, 1.0f, 0.0f, 1.0f},
    {"LagTurn ", "LT", &tune.latency.turn_thresh_deg, 5.0f, 1.0f, 180.0f},
    {"EncVMin ", "EV", &tune.latency.enc_v_min, 0.05f, 0.0f, 20.0f},
    {"VisVMin ", "VV", &tune.latency.vis_v_min, 0.05f, 0.0f, 20.0f},
    {"LagRef  ", "LR", &tune.latency.refractory_ms, 10.0f, 0.0f, 5000.0f},
    {"LagMin  ", "LN", &tune.latency.l_min_ms, 10.0f, 0.0f, 1000.0f},
    {"LagMax  ", "LX", &tune.latency.l_max_ms, 10.0f, 0.0f, 2000.0f},
    {"LagAngle", "LD", &tune.latency.dtheta_tol_deg, 1.0f, 0.0f, 180.0f},
    {"LagAlpha", "LA", &tune.latency.lowpass_alpha, 0.05f, 0.0f, 1.0f},
    {"LagStale", "LS", &tune.latency.l_stale_ms, 100.0f, 100.0f, 60000.0f},
    {"BombWait", "BW", &tune.bomb.explosion_wait_ms, 50.0f, 500.0f, 10000.0f},
    {"VLongEn ", "VN", &tune.vision_long.enable, 1.0f, 0.0f, 1.0f},
    {"VLFloor ", "VF", &tune.vision_long.freeze_floor_cm, 0.5f, 0.0f, 30.0f},
    {"VLLagG  ", "VG", &tune.vision_long.latency_window_gain, 0.05f, 0.0f, 2.0f},
    {"VLStep  ", "VS", &tune.vision_long.max_step_cm, 0.05f, 0.0f, 3.0f},
    {"VLPushSt", "VP", &tune.vision_long.push_max_step_cm, 0.05f, 0.0f, 3.0f},
    {"VLRej   ", "VJ", &tune.vision_long.reject_dist_cm, 0.5f, 0.5f, 20.0f},
    {"VLScEn  ", "VE", &tune.vision_long.scale_learn_enable, 1.0f, 0.0f, 1.0f},
    {"VLScAlph", "VA", &tune.vision_long.scale_learn_alpha, 0.05f, 0.01f, 1.0f},
    {"VLScDist", "VD", &tune.vision_long.scale_sample_min_cm, 5.0f, 5.0f, 100.0f},
    {"VLScMin ", "SM", &tune.vision_long.scale_min, 0.01f, 0.5f, 1.5f},
    {"VLScMax ", "SX", &tune.vision_long.scale_max, 0.01f, 0.5f, 1.5f},
    {"HorStep ", "HS", &tune.vision_lateral.max_step_cm, 0.1f, 0.0f, 5.0f},
    {"HorGain ", "HG", &tune.vision_lateral.gain, 0.05f, 0.0f, 1.5f},
    {"YawBand ", "YB", &tune.yaw.lin_band, 0.01f, 0.02f, 1.0f},
    {"YawKd   ", "YD", &tune.yaw.kd, 0.01f, 0.0f, 2.0f},
    {"YawGain ", "YG", &tune.yaw.translate_gain, 0.05f, 0.1f, 3.0f},
    {"YawTrKd ", "YT", &tune.yaw.kd_translate, 0.01f, 0.0f, 2.0f},
    {"YawKs   ", "YS", &tune.yaw.stiction, 0.01f, 0.0f, 2.0f},
    {"StopBrk ", "TB", &tune.terminal.stop_brake_gain, 0.25f, 1.0f, 100.0f},
    {"ShortLen", "SL", &tune.terminal.short_segment_cm, 5.0f, 1.0f, 200.0f},
    {"ShortAcc", "SA", &tune.terminal.short_acceleration, 10.0f, 20.0f, 1000.0f},
    {"AprZone ", "AZ", &tune.terminal.approach_zone_cm, 1.0f, 0.5f, 100.0f},
    {"AprRatio", "AR", &tune.terminal.approach_ratio, 0.05f, 0.05f, 1.0f},
    {"AprAcc  ", "AC", &tune.terminal.approach_acceleration, 1.0f, 1.0f, 200.0f},
    {"AprEn   ", "AE", &tune.terminal.approach_enable, 1.0f, 0.0f, 1.0f},
    {"ALL Kp  ", "KP", &tune.wheels[0].pid.kp, 0.01f, 0.0f, 5.0f, ScreenEditGroup::WHEEL_KP},
    {"ALL Ki  ", "KI", &tune.wheels[0].pid.ki, 0.05f, 0.0f, 5.0f, ScreenEditGroup::WHEEL_KI},
    {"ALL Kd  ", "KD", &tune.wheels[0].pid.kd, 0.001f, 0.0f, 2.0f, ScreenEditGroup::WHEEL_KD},
    {"ALL Kv  ", "KV", &tune.wheels[0].kv, 0.001f, 0.0f, 1.0f, ScreenEditGroup::WHEEL_KV},
    {"ALL Ka  ", "KA", &tune.wheels[0].ka, 0.001f, 0.0f, 1.0f, ScreenEditGroup::WHEEL_KA},
    {"ALL Kb  ", "KB", &tune.wheels[0].kb, 0.001f, 0.0f, 1.0f, ScreenEditGroup::WHEEL_KB},
    {"ALL Ks  ", "KS", &tune.wheels[0].ks, 0.1f, 0.0f, 30.0f, ScreenEditGroup::WHEEL_KS},
    {"DiagMove", "DM", &tune.planning_extra.diagonal_move_enable, 1.0f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"BoxExtra", "BX", &tune.planning_extra.box_extra_observe_enable, 1.0f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"TgtExtra", "TX", &tune.planning_extra.target_extra_observe_enable, 1.0f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
};

// 单轮键由 0=LF、1=LB、2=RF、3=RB 组成，屏幕只显示上面的 ALL 项
inline ParamItem wheel_params[] = {
    {"LF Kp", "0P", &tune.wheels[0].pid.kp, 0.01f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"LF Ki", "0I", &tune.wheels[0].pid.ki, 0.05f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"LF Kd", "0D", &tune.wheels[0].pid.kd, 0.001f, 0.0f, 2.0f, ScreenEditGroup::SINGLE, false},
    {"LF Kv", "0V", &tune.wheels[0].kv, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"LF Ka", "0A", &tune.wheels[0].ka, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"LF Kb", "0B", &tune.wheels[0].kb, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"LF Ks", "0S", &tune.wheels[0].ks, 0.1f, 0.0f, 30.0f, ScreenEditGroup::SINGLE, false},
    {"LB Kp", "1P", &tune.wheels[1].pid.kp, 0.01f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"LB Ki", "1I", &tune.wheels[1].pid.ki, 0.05f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"LB Kd", "1D", &tune.wheels[1].pid.kd, 0.001f, 0.0f, 2.0f, ScreenEditGroup::SINGLE, false},
    {"LB Kv", "1V", &tune.wheels[1].kv, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"LB Ka", "1A", &tune.wheels[1].ka, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"LB Kb", "1B", &tune.wheels[1].kb, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"LB Ks", "1S", &tune.wheels[1].ks, 0.1f, 0.0f, 30.0f, ScreenEditGroup::SINGLE, false},
    {"RF Kp", "2P", &tune.wheels[2].pid.kp, 0.01f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"RF Ki", "2I", &tune.wheels[2].pid.ki, 0.05f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"RF Kd", "2D", &tune.wheels[2].pid.kd, 0.001f, 0.0f, 2.0f, ScreenEditGroup::SINGLE, false},
    {"RF Kv", "2V", &tune.wheels[2].kv, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"RF Ka", "2A", &tune.wheels[2].ka, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"RF Kb", "2B", &tune.wheels[2].kb, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"RF Ks", "2S", &tune.wheels[2].ks, 0.1f, 0.0f, 30.0f, ScreenEditGroup::SINGLE, false},
    {"RB Kp", "3P", &tune.wheels[3].pid.kp, 0.01f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"RB Ki", "3I", &tune.wheels[3].pid.ki, 0.05f, 0.0f, 5.0f, ScreenEditGroup::SINGLE, false},
    {"RB Kd", "3D", &tune.wheels[3].pid.kd, 0.001f, 0.0f, 2.0f, ScreenEditGroup::SINGLE, false},
    {"RB Kv", "3V", &tune.wheels[3].kv, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"RB Ka", "3A", &tune.wheels[3].ka, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"RB Kb", "3B", &tune.wheels[3].kb, 0.001f, 0.0f, 1.0f, ScreenEditGroup::SINGLE, false},
    {"RB Ks", "3S", &tune.wheels[3].ks, 0.1f, 0.0f, 30.0f, ScreenEditGroup::SINGLE, false},
};

inline char upper_ascii(char value) {
    return value >= 'a' && value <= 'z' ? static_cast<char>(value - 'a' + 'A') : value;
}

inline bool same_key(const char* lhs, const char* rhs) {
    return lhs != nullptr && rhs != nullptr && lhs[0] != '\0' && lhs[1] != '\0' &&
           lhs[2] == '\0' && rhs[2] == '\0' && upper_ascii(lhs[0]) == upper_ascii(rhs[0]) &&
           upper_ascii(lhs[1]) == upper_ascii(rhs[1]);
}

inline bool value_valid(const ParamItem& item, float value) {
    return std::isfinite(value) && value >= item.minimum && value <= item.maximum;
}

inline const ParamItem* find(const char* key) {
    for (const auto& item : params) if (same_key(item.key, key)) return &item;
    for (const auto& item : wheel_params) if (same_key(item.key, key)) return &item;
    return nullptr;
}

inline constexpr std::size_t count() {
    return sizeof(params) / sizeof(params[0]) + sizeof(wheel_params) / sizeof(wheel_params[0]);
}

inline bool config_valid(const TuningConfig& config) {
    const std::uintptr_t tune_base = reinterpret_cast<std::uintptr_t>(&tune);
    const auto* config_base = reinterpret_cast<const unsigned char*>(&config);
    const auto check_table = [&](const auto& table) {
        for (const auto& item : table) {
            const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(item.value);
            if (address < tune_base) return false;
            const std::size_t offset = static_cast<std::size_t>(address - tune_base);
            if (offset > sizeof(config) - sizeof(float)) return false;
            if (!value_valid(item, *reinterpret_cast<const float*>(config_base + offset))) return false;
        }
        return true;
    };
    return check_table(params) && check_table(wheel_params) &&
           config.latency.l_min_ms <= config.latency.l_max_ms &&
           config.vision_long.scale_min <= config.vision_long.scale_max;
}

inline float& wheel_value(WheelControlParams& wheel, ScreenEditGroup group) {
    switch (group) {
    case ScreenEditGroup::WHEEL_KP: return wheel.pid.kp;
    case ScreenEditGroup::WHEEL_KI: return wheel.pid.ki;
    case ScreenEditGroup::WHEEL_KD: return wheel.pid.kd;
    case ScreenEditGroup::WHEEL_KV: return wheel.kv;
    case ScreenEditGroup::WHEEL_KA: return wheel.ka;
    case ScreenEditGroup::WHEEL_KB: return wheel.kb;
    case ScreenEditGroup::WHEEL_KS: return wheel.ks;
    case ScreenEditGroup::SINGLE: return wheel.pid.kp;
    }
    return wheel.pid.kp;
}

inline void assign(ParamItem& item, float value) {
    if (item.screen_group == ScreenEditGroup::SINGLE) {
        *item.value = value;
        return;
    }
    for (auto& wheel : tune.wheels) wheel_value(wheel, item.screen_group) = value;
}

inline SetResult set_by_key(const char* key, float value) {
    const ParamItem* found = find(key);
    if (found == nullptr) return SetResult::UNKNOWN_KEY;
    if (!value_valid(*found, value)) return SetResult::OUT_OF_RANGE;
    for (auto& item : params) {
        if (same_key(item.key, key)) {
            assign(item, value);
            return SetResult::OK;
        }
    }
    *found->value = value;
    return SetResult::OK;
}

inline std::size_t screen_count() {
    std::size_t result = 0u;
    for (const auto& item : params) if (item.screen_visible) ++result;
    return result;
}

inline ParamItem& screen_param(std::size_t index) {
    for (auto& item : params) if (item.screen_visible && index-- == 0u) return item;
    return params[0];
}

inline void adjust_from_screen(ParamItem& item, float delta) {
    const float next = *item.value + delta;
    const float bounded = next < item.minimum ? item.minimum : next > item.maximum ? item.maximum : next;
    assign(item, bounded);
}

} // namespace TuningRegistry
