#ifdef __cplusplus
extern "C" {
#endif
#include "zf_driver_delay.h"
#ifdef __cplusplus
}
#endif
#include "PoseEstimate.h"
#include "RobotState.h"
#include "system_config.h"
#include "tuning_config.h"

#include "MotionControl.h"
#include "Telemetry.h"

#include "zf_common_headfile.h"
#include "Icm42688.h"
#include "Encoder.h"
#include <cmath>

// =============================================================================
// 内部数据结构和状态变量
// =============================================================================

namespace Subsystem::PoseEstimator {
namespace { 
    #define DTCM_DATA __attribute__((section(".dtcm_data")))

    DTCM_DATA DebugProbes g_probes = {0}; 

    // 基础状态变量
    DTCM_DATA float gyro_x_offset = 0.0f;     
    DTCM_DATA float gyro_y_offset = 0.0f;     
    DTCM_DATA float gyro_z_offset = 0.0f;         // 陀螺仪Z轴零偏 (deg/s)
    DTCM_DATA bool is_calibrated = false;         // 标定完成标志位
    DTCM_DATA float dynamic_deadband = 0.0f;      // 动态计算出的噪声死区

    // ZUPT 相关变量
    DTCM_DATA uint16_t stop_settle_counter = 0;   // 停车缓冲震荡计数器
    DTCM_DATA float gyro_buffer[20] = {0};
    DTCM_DATA uint8_t buf_idx = 0;

    // 四元数相关变量，四元数初始化为绝对水平、Yaw为 90度 [cos(45deg) = 0.70710678f, sin(45deg) = 0.70710678f]
    DTCM_DATA float q0 = 0.70710678f, q1 = 0.0f, q2 = 0.0f, q3 = 0.70710678f;

    // 视觉标定相关变量
    DTCM_DATA VisionCalibrator vision_calibrator;                     // 视觉标定器实例
    DTCM_DATA AsyncCalibState s_calib_state = AsyncCalibState::IDLE;  // 当前标定状态
    DTCM_DATA uint32_t s_last_calib_vision_seq = 0;                   // 标定流程已消费的最后一帧视觉序号

    constexpr uint32_t MAX_VISION_LATENCY_LOOKBACK_MS = 1000U;
    constexpr uint32_t ODOM_HISTORY_LOOKBACK_MS = 1000U;
    static_assert(ODOM_HISTORY_LOOKBACK_MS >= MAX_VISION_LATENCY_LOOKBACK_MS,
                  "Odom history must cover the configured vision latency range");
    constexpr uint16_t ODOM_HISTORY_SIZE =
        static_cast<uint16_t>(ODOM_HISTORY_LOOKBACK_MS / SystemConfig::PIT_CH1_PERIOD_MS + 3U);
    constexpr uint32_t ODOM_HISTORY_COVERAGE_MS =
        static_cast<uint32_t>(ODOM_HISTORY_SIZE - 1U) * SystemConfig::PIT_CH1_PERIOD_MS;

    struct OdomPoseSample {
        uint32_t tick_ms = 0;
        Pose2D pose = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};
        Pose2D raw_pose = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};
        bool valid = false;
    };

    // 编码器积分轨迹，用于把延时视觉坐标外推到当前时刻。
    // 横向视觉修正不写这里（保持切向判定的横向容差零延迟、不被视觉污染）；
    // 纵向视觉修正会整体平移它（含 s_odom_history），因为到达/越线/切段判定都读
    // s_encoder_pose，纵向修正不写进来就无人消费。平移保持帧间差值不变，延时匹配照常。
    DTCM_DATA Pose2D s_encoder_pose = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};
    // 原始轨迹只积累 KX/KY 标定后的编码器增量，不接受视觉平移和在线里程比例
    // 学习时只比较原始轨迹与同采集时刻视觉，避免把已经修过的量再次拿来学习
    DTCM_DATA Pose2D s_raw_encoder_pose = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};
    DTCM_DATA OdomPoseSample s_odom_history[ODOM_HISTORY_SIZE];
    DTCM_DATA uint16_t s_odom_history_idx = 0;
    DTCM_DATA VisionLatencyDebug s_vision_latency_debug = {};
    DTCM_DATA MileageScaleDebug s_mileage_scale_debug = {};

    struct MileageScaleAxisLearner {
        float candidate = 1.0f;
        uint8_t consistent_count = 0;
    };

    struct MileageScaleAnchor {
        Pose2D vision_pose = {};
        Pose2D raw_odom_pose = {};
        float along_x = 0.0f;
        float along_y = 0.0f;
        uint8_t axis = 0;
        bool valid = false;
    };

    DTCM_DATA volatile float s_mileage_scale_x = 1.0f;
    DTCM_DATA volatile float s_mileage_scale_y = 1.0f;
    DTCM_DATA MileageScaleAxisLearner s_mileage_axis[2];
    DTCM_DATA MileageScaleAnchor s_mileage_anchor;

    // 常量定义
    constexpr float PLUSE_TO_CM = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / SystemConfig::PULSES_PER_REV;  // 编码器计数转换为轮子移动的距离（cm）
    constexpr float SAMPLE_FREQ = 1.0f / SystemConfig::PIT_CH0_DT_S;  // 采样频率 (Hz)，根据系统定时器周期计算
    constexpr uint32_t VISION_POSE_MAX_AGE_MS = 300;  // 视觉位姿数据的最大有效时间，超过这个时间就丢弃
    constexpr float DEFAULT_ENCODER_LATENCY_GAIN =
        DEFAULT_TUNE_CONFIG.latency.encoder_latency_gain;
    constexpr float MIN_ENCODER_LATENCY_GAIN = 0.01f;
    constexpr float MAX_ENCODER_LATENCY_GAIN = 2.0f;
    constexpr float DEFAULT_VISION_LATENCY_MS =
        DEFAULT_TUNE_CONFIG.latency.vision_latency_ms;
    constexpr float VISION_LATERAL_DEADBAND_CM = 0.15f;
    constexpr float VISION_ENCODER_RESET_THRESHOLD_CM = 15.0f;
    constexpr float MILEAGE_AXIS_ALIGNMENT_MIN = 0.92f;
    constexpr float MILEAGE_SAMPLE_CONSISTENCY_TOL = 0.03f;
    constexpr float MILEAGE_MAX_CROSS_TRACK_CM = 3.0f;
    constexpr float MILEAGE_MAX_CROSS_TRACK_RATIO = 0.15f;
    constexpr uint8_t MILEAGE_REQUIRED_CONSISTENT_SAMPLES = 2U;
    // 段法向修正要求段向量至少 1cm，否则方向不可靠
    constexpr float VISION_LATERAL_MIN_SEGMENT_LEN_SQ = 1.0f;   // (1 cm)^2
    // 延时外推封顶余量：指令位移(合速度模) × 该系数为外推矢量模上限。1.0=严格按指令；
    // >1 给运动学增益/yaw 投影误差留裕度。取 1.5 兼顾"匀速不误伤、刹车能夹掉打滑虚增"。
    constexpr float VISION_LATENCY_CMD_CLAMP_MARGIN = 1.5f;
    [[maybe_unused]] constexpr float VISION_TARGET_FREEZE_RADIUS_CM = 10.0f;

    // 快速逆平方根函数，供 Mahony 算法使用
    [[gnu::always_inline]] inline float fast_inv_sqrt(float x) {
        return 1.0f / __builtin_sqrtf(x);
    }

    [[gnu::always_inline]] inline void push_odom_history(uint32_t tick_ms,
                                                        const Pose2D& pose,
                                                        const Pose2D& raw_pose) {
        s_odom_history[s_odom_history_idx].tick_ms = tick_ms;
        s_odom_history[s_odom_history_idx].pose = pose;
        s_odom_history[s_odom_history_idx].raw_pose = raw_pose;
        s_odom_history[s_odom_history_idx].valid = true;
        s_odom_history_idx = static_cast<uint16_t>((s_odom_history_idx + 1U) % ODOM_HISTORY_SIZE);
    }

    [[gnu::always_inline]] inline float encoder_latency_gain() {
        float gain = tune.latency.encoder_latency_gain;
        if (!std::isfinite(gain)) {
            gain = DEFAULT_ENCODER_LATENCY_GAIN;
        }
        if (gain < MIN_ENCODER_LATENCY_GAIN) {
            return MIN_ENCODER_LATENCY_GAIN;
        }
        if (gain > MAX_ENCODER_LATENCY_GAIN) {
            return MAX_ENCODER_LATENCY_GAIN;
        }
        return gain;
    }

    void reset_odom_history(const Pose2D& pose, uint32_t tick_ms) {
        s_encoder_pose = pose;
        s_raw_encoder_pose = pose;
        s_odom_history_idx = 0;
        for (auto& sample : s_odom_history) {
            sample.valid = false;
        }
        push_odom_history(tick_ms, s_encoder_pose, s_raw_encoder_pose);
    }

    [[gnu::always_inline]] inline bool mileage_scale_enabled() {
        return tune.vision_long.enable > 0.5f &&
               tune.vision_long.scale_learn_enable > 0.5f;
    }

    [[gnu::always_inline]] inline float mileage_scale_for_axis(uint8_t axis) {
        if (!mileage_scale_enabled()) return 1.0f;
        float value = axis == 0U ? s_mileage_scale_x : s_mileage_scale_y;
        float lo = tune.vision_long.scale_min;
        float hi = tune.vision_long.scale_max;
        if (!std::isfinite(value) || !std::isfinite(lo) || !std::isfinite(hi) || lo > hi) {
            return 1.0f;
        }
        return value < lo ? lo : (value > hi ? hi : value);
    }

    void reset_mileage_scale_learner(bool full) {
        s_mileage_anchor = {};
        for (auto& axis : s_mileage_axis) {
            axis.candidate = 1.0f;
            axis.consistent_count = 0U;
        }
        if (full) {
            s_mileage_scale_x = 1.0f;
            s_mileage_scale_y = 1.0f;
        }
        s_mileage_scale_debug = {};
        s_mileage_scale_debug.scale_x = s_mileage_scale_x;
        s_mileage_scale_debug.scale_y = s_mileage_scale_y;
    }

    [[gnu::always_inline]] inline float pose_distance_sq_xy(const Pose2D& a, const Pose2D& b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    // =========================================================================
    // 拐点对齐的实时视觉延时估计 (foundation 阶段新增)
    // 思路：方向突变时编码器速度矢量在 T_enc 转向，视觉速度矢量在收到帧的
    //       T_vis 才转向，L ≈ T_vis − T_enc。直接测物理延时再做 tick 查表补偿。
    // 并发：编码器拐点在 20ms PIT 中断里入 FIFO；视觉拐点在主循环里配对并写 L。
    //       FIFO 消费者(主循环)用 PRIMASK 临界区屏蔽中断，生产者(中断)无需加锁。
    //
    // 2026-08-07 起本估计器降级为纯测量仪：结果只写 s_vision_latency_debug
    // （波形 mode 3 的 est_raw/est_filt 照常可看，用来标定固定 L），不再参与
    // 位姿补偿，见 VISION_LATENCY_ONLINE_ESTIMATE 与 current_L()。
    // LE 键从此只控制"是否继续测量"，不影响控制链路。
    // =========================================================================

    // 拐点估计出的 L 是否参与位姿补偿。false = 只做遥测测量，补偿一律用固定
    // tune.latency.vision_latency_ms（默认 380ms）
    //
    // 2026-08-07 关闭原因：管线延时（曝光 + ART1 处理 + 串口）物理上是常数，
    // 在线估计只是在追一个常数，方差却直接进落点。冲刺段尤其致命——外推距离
    // 是 v*L，v=150 时 L 偏 50ms 就是 7.5cm。更糟的是 L 只在拐点更新，长直线
    // 段内无拐点，跑够 l_stale_ms 就整体跳回 380ms，落点误差因此呈双峰
    // （实测 2~4cm 与 10cm 两簇）。改常数后误差退化为确定的 v*ΔL，可一次标掉。
    // 改回 true 即恢复在线估计。
    constexpr bool VISION_LATENCY_ONLINE_ESTIMATE = false;

    constexpr uint8_t L_MEDIAN_WIN   = 5;     // L 中值窗口
    constexpr uint8_t PENDING_FIFO_CAP = 3;   // 待配对编码器拐点容量
    constexpr float   INFLECT_SLEW_ALPHA = 0.15f; // ref 航向跟踪渐变曲率的慢速系数

    [[gnu::always_inline]] inline float clampf(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    // 将角度收敛到 [-pi, pi]（输入差值范围 [-2pi, 2pi]，单次校正即可）
    [[gnu::always_inline]] inline float wrap_pi(float a) {
        if (a > PI)       a -= 2.0f * PI;
        else if (a < -PI) a += 2.0f * PI;
        return a;
    }

    // 短暂屏蔽中断的 RAII 临界区，保护与 20ms 中断共享的 FIFO
    struct IrqLock {
        uint32_t primask;
        IrqLock()  { primask = __get_PRIMASK(); __disable_irq(); }
        ~IrqLock() { __set_PRIMASK(primask); }
    };

    // 速度矢量拐点边沿检测器：比较新航向与"当前段参考航向"，跨阈值即触发，
    // 渐变曲率被慢速跟踪掉、原地旋转/静止被速度门挡掉
    struct InflectionDetector {
        float    ref_heading = 0.0f;     // 当前直线段速度朝向
        bool     has_ref = false;
        uint32_t prev_fast_tick = 0;     // 上一个过速度门样本的时刻（用于锚定拐点中点）
        uint32_t last_inflect_tick = 0;

        void reset() { has_ref = false; ref_heading = 0.0f; prev_fast_tick = 0; last_inflect_tick = 0; }

        // 喂入一帧速度矢量 (vx,vy 为该周期位移即可，朝向与量纲无关)；
        // 触发拐点时返回 true，并输出拐点中点时刻与带符号转角
        bool feed(float vx, float vy, uint32_t tick, float v_min, float thresh_rad,
                  uint32_t refractory_ms, uint32_t& out_tick, float& out_dtheta) {
            float speed_sq = vx * vx + vy * vy;
            if (speed_sq < v_min * v_min) {
                return false;  // 慢速：保持 ref 与 prev_fast_tick（跨过弯角速度凹陷）
            }
            float h = atan2f(vy, vx);
            if (!has_ref) {
                ref_heading = h; has_ref = true; prev_fast_tick = tick;
                return false;
            }
            float dtheta = wrap_pi(h - ref_heading);
            bool fired = false;
            if (std::abs(dtheta) >= thresh_rad &&
                (uint32_t)(tick - last_inflect_tick) > refractory_ms) {
                // 拐点发生在 prev_fast_tick 与 tick 之间，取中点对齐（两流对称）
                out_tick = prev_fast_tick + ((uint32_t)(tick - prev_fast_tick) >> 1);
                out_dtheta = dtheta;
                last_inflect_tick = tick;
                ref_heading = h;             // 锁定到新段方向
                fired = true;
            } else {
                ref_heading = wrap_pi(ref_heading + INFLECT_SLEW_ALPHA * dtheta);
            }
            prev_fast_tick = tick;
            return fired;
        }
    };

    struct PendingCorner {
        uint32_t enc_tick;
        float    enc_dtheta;
        uint32_t expiry_tick;
    };

    DTCM_DATA InflectionDetector s_enc_det;
    DTCM_DATA InflectionDetector s_vis_det;
    DTCM_DATA PendingCorner s_pending[PENDING_FIFO_CAP];
    DTCM_DATA uint8_t  s_pending_count = 0;

    DTCM_DATA float    s_L_filt = DEFAULT_VISION_LATENCY_MS;
    DTCM_DATA bool     s_L_locked = false;
    DTCM_DATA uint32_t s_L_last_update_tick = 0;
    DTCM_DATA float    s_L_median[L_MEDIAN_WIN] = {0};
    DTCM_DATA uint8_t  s_L_med_count = 0;
    DTCM_DATA uint8_t  s_L_med_idx = 0;

    // 清空估计器状态。full=true 连同已学到的 L 一起清（开机用）；
    // full=false 只清检测器与 FIFO（位姿硬重置/瞬移时用，延时本身不变）
    void reset_latency_estimator(bool full) {
        s_enc_det.reset();
        s_vis_det.reset();
        s_pending_count = 0;
        if (full) {
            s_L_filt = DEFAULT_VISION_LATENCY_MS;
            s_L_locked = false;
            s_L_last_update_tick = 0;
            s_L_med_count = 0;
            s_L_med_idx = 0;
        }
    }

    [[gnu::always_inline]] inline float median_of(const float* buf, uint8_t n) {
        float tmp[L_MEDIAN_WIN];
        for (uint8_t i = 0; i < n; ++i) tmp[i] = buf[i];
        for (uint8_t i = 1; i < n; ++i) {     // 插入排序，n<=5
            float key = tmp[i];
            int8_t j = (int8_t)i - 1;
            while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; --j; }
            tmp[j + 1] = key;
        }
        return tmp[n / 2];
    }

    // 当前补偿采用的延时：固定取 tune.latency.vision_latency_ms
    // VISION_LATENCY_ONLINE_ESTIMATE 为 false 时下面的在线分支整段被编译器折掉，
    // 因此 flash 里存着的 LE=1 也不会让估计值重新进入补偿链路（不需要 storage 迁移）
    float current_L() {
        float fallback = tune.latency.vision_latency_ms;
        if (!std::isfinite(fallback)) {
            fallback = DEFAULT_VISION_LATENCY_MS;
        }
        if (VISION_LATENCY_ONLINE_ESTIMATE && tune.latency.enable_estimation && s_L_locked) {
            uint32_t now = Core::Scheduler::get_sys_tick_ms();
            if ((uint32_t)(now - s_L_last_update_tick) <= (uint32_t)tune.latency.l_stale_ms) {
                s_vision_latency_debug.used_l_ms = s_L_filt;
                return s_L_filt;
            }
        }
        s_vision_latency_debug.used_l_ms = fallback;
        return fallback;
    }

    // 丢弃已过期的待配对拐点（now 超过 expiry）
    void pending_sweep(uint32_t now) {
        uint8_t w = 0;
        for (uint8_t r = 0; r < s_pending_count; ++r) {
            if ((int32_t)(now - s_pending[r].expiry_tick) < 0) {
                s_pending[w++] = s_pending[r];
            }
        }
        s_pending_count = w;
    }

    // 生产者(20ms 中断)：编码器拐点入队，满则丢最旧
    void push_pending_corner(uint32_t enc_tick, float dtheta, uint32_t now, float l_max) {
        pending_sweep(now);
        if (s_pending_count >= PENDING_FIFO_CAP) {
            for (uint8_t i = 1; i < s_pending_count; ++i) s_pending[i - 1] = s_pending[i];
            --s_pending_count;
        }
        s_pending[s_pending_count].enc_tick = enc_tick;
        s_pending[s_pending_count].enc_dtheta = dtheta;
        s_pending[s_pending_count].expiry_tick = enc_tick + (uint32_t)l_max;
        ++s_pending_count;
        s_vision_latency_debug.est_pending_count = s_pending_count;
    }

    // 消费者(主循环)：视觉拐点按时间序配对编码器拐点，命中返回 L=T_vis−T_enc
    bool match_vision_corner(uint32_t vis_tick, float vis_dtheta, uint32_t now,
                             float l_min, float l_max, float dtheta_tol_rad, float& out_L) {
        IrqLock lk;  // 临界区：屏蔽 20ms 中断，独占访问 FIFO
        pending_sweep(now);
        int found = -1;
        for (uint8_t i = 0; i < s_pending_count; ++i) {     // 最旧在前，L 由大到小
            int32_t L = (int32_t)(vis_tick - s_pending[i].enc_tick);
            if (L > (int32_t)l_max) continue;               // 太旧（理论已被 sweep 清掉）
            if (L < (int32_t)l_min) break;                  // 此项及更新的都太近，无匹配
            float e = s_pending[i].enc_dtheta;
            if ((e > 0.0f) != (vis_dtheta > 0.0f)) continue;             // 转向符号须一致
            if (std::abs(std::abs(vis_dtheta) - std::abs(e)) > dtheta_tol_rad) continue; // 量级须相近
            out_L = (float)L;
            found = (int)i;
            break;
        }
        if (found < 0) {
            s_vision_latency_debug.est_pending_count = s_pending_count;
            return false;
        }
        // 消费命中项及其之前所有未配对的旧拐点，防后续误配
        uint8_t w = 0;
        for (uint8_t i = (uint8_t)found + 1; i < s_pending_count; ++i) s_pending[w++] = s_pending[i];
        s_pending_count = w;
        s_vision_latency_debug.est_pending_count = s_pending_count;
        return true;
    }

    // 命中后更新 L：clamp → 中值 → 慢速低通
    void update_L_filter(float raw_L, uint32_t now) {
        raw_L = clampf(raw_L, tune.latency.l_min_ms, tune.latency.l_max_ms);
        s_L_median[s_L_med_idx] = raw_L;
        s_L_med_idx = (uint8_t)((s_L_med_idx + 1) % L_MEDIAN_WIN);
        if (s_L_med_count < L_MEDIAN_WIN) ++s_L_med_count;
        float med = median_of(s_L_median, s_L_med_count);
        float a = clampf(tune.latency.lowpass_alpha, 0.0f, 1.0f);
        if (!s_L_locked) { s_L_filt = med; s_L_locked = true; }
        else             { s_L_filt = (1.0f - a) * s_L_filt + a * med; }
        s_L_last_update_tick = now;
        s_vision_latency_debug.est_raw_l_ms = raw_L;
        s_vision_latency_debug.est_filt_l_ms = s_L_filt;
        s_vision_latency_debug.est_last_match_tick_ms = now;
        s_vision_latency_debug.est_locked = true;
    }

    // 编码器侧：在 20ms 中断里喂入全局位移矢量，触发拐点则入队
    void feed_encoder_sample(float vx, float vy, uint32_t now) {
        if (!tune.latency.enable_estimation) return;
        uint32_t ftick; float fdtheta;
        if (s_enc_det.feed(vx, vy, now, tune.latency.enc_v_min,
                           tune.latency.turn_thresh_deg * SystemConfig::DEG_TO_RAD,
                           (uint32_t)tune.latency.refractory_ms, ftick, fdtheta)) {
            push_pending_corner(ftick, fdtheta, now, tune.latency.l_max_ms);
        }
    }

    // 视觉侧：在主循环里喂入帧间位移矢量，触发拐点则配对并更新 L
    void feed_vision_sample(float vx, float vy, uint32_t now) {
        if (!tune.latency.enable_estimation) return;
        uint32_t ftick; float fdtheta;
        if (s_vis_det.feed(vx, vy, now, tune.latency.vis_v_min,
                           tune.latency.turn_thresh_deg * SystemConfig::DEG_TO_RAD,
                           (uint32_t)tune.latency.refractory_ms, ftick, fdtheta)) {
            float rawL;
            if (match_vision_corner(ftick, fdtheta, now,
                                    tune.latency.l_min_ms, tune.latency.l_max_ms,
                                    tune.latency.dtheta_tol_deg * SystemConfig::DEG_TO_RAD, rawL)) {
                update_L_filter(rawL, now);
            }
        }
    }

    // 按时间查表估计视觉帧的"采集时刻"对应的编码器位姿：
    //   target = receive_tick − current_L()，在编码器历史里取该 tick 的样本（相邻两点线性插值）。
    // 比旧的"按 XY 距离找最近点"更可靠：XY 匹配恰在拐点处歧义/出错，而 tick 查表无歧义、
    // 用的是直测物理延时；s_odom_history 已存 tick_ms，无需新结构。
    bool match_vision_pose_to_odom_history(const Pose2D& delayed_vision_pose,
                                           uint32_t receive_tick_ms,
                                           Pose2D& out_odom_pose,
                                           Pose2D& out_raw_odom_pose,
                                           uint32_t& out_capture_tick_ms) {
        (void)delayed_vision_pose;  // 不再用视觉 XY 匹配，仅按时间查表
        constexpr int32_t SINGLE_SIDED_TOL_MS = 2 * (int32_t)SystemConfig::PIT_CH1_PERIOD_MS;

        uint32_t target_tick = receive_tick_ms - (uint32_t)current_L();

        // 用带符号差（值都在 ~1s 窗口内）安全跨越无符号回绕：
        //   delta = sample.tick − target，<0 表示样本在 target 之前
        const OdomPoseSample* before = nullptr;  // delta<=0 中最接近 0 的
        const OdomPoseSample* after  = nullptr;  // delta>=0 中最接近 0 的
        int32_t best_before = INT32_MIN;
        int32_t best_after  = INT32_MAX;

        for (const auto& sample : s_odom_history) {
            if (!sample.valid) continue;
            int32_t delta = (int32_t)(sample.tick_ms - target_tick);
            if (delta <= 0 && delta > best_before) { best_before = delta; before = &sample; }
            if (delta >= 0 && delta < best_after)  { best_after  = delta; after  = &sample; }
        }

        if (before != nullptr && after != nullptr) {
            uint32_t span = after->tick_ms - before->tick_ms;
            if (span == 0U) {
                out_odom_pose = before->pose;
                out_raw_odom_pose = before->raw_pose;
            } else {
                float t = (float)(int32_t)(target_tick - before->tick_ms) / (float)span;
                out_odom_pose.x   = before->pose.x   + (after->pose.x   - before->pose.x)   * t;
                out_odom_pose.y   = before->pose.y   + (after->pose.y   - before->pose.y)   * t;
                out_odom_pose.yaw = after->pose.yaw;  // yaw 仅供调试，取较新值
                out_raw_odom_pose.x = before->raw_pose.x +
                                      (after->raw_pose.x - before->raw_pose.x) * t;
                out_raw_odom_pose.y = before->raw_pose.y +
                                      (after->raw_pose.y - before->raw_pose.y) * t;
                out_raw_odom_pose.yaw = after->raw_pose.yaw;
            }
            out_capture_tick_ms = target_tick;
            return true;
        }
        // 只有单侧样本：在容差内就近取用，否则放弃（历史未覆盖该时刻）
        if (before != nullptr && (-best_before) <= SINGLE_SIDED_TOL_MS) {
            out_odom_pose = before->pose;
            out_raw_odom_pose = before->raw_pose;
            out_capture_tick_ms = before->tick_ms;
            return true;
        }
        if (after != nullptr && best_after <= SINGLE_SIDED_TOL_MS) {
            out_odom_pose = after->pose;
            out_raw_odom_pose = after->raw_pose;
            out_capture_tick_ms = after->tick_ms;
            return true;
        }
        return false;
    }

    bool odom_delta_since_matched_vision_pose(const Pose2D& delayed_vision_pose,
                                             uint32_t receive_tick_ms,
                                             uint32_t& out_capture_tick_ms,
                                             float& out_dx,
                                             float& out_dy,
                                             Pose2D& out_raw_odom_at_capture) {
        out_dx = 0.0f;
        out_dy = 0.0f;

        Pose2D odom_at_capture;
        if (!match_vision_pose_to_odom_history(
                delayed_vision_pose,
                receive_tick_ms,
                odom_at_capture,
                out_raw_odom_at_capture,
                out_capture_tick_ms)) {
            return false;
        }

        float gain = encoder_latency_gain();
        out_dx = (s_encoder_pose.x - odom_at_capture.x) * gain;
        out_dy = (s_encoder_pose.y - odom_at_capture.y) * gain;

        // 打滑封顶：编码器在延时窗口 L 内积分出的位移，末尾刹车/打滑时会比真实多，
        // 直接把外推量甩过头→过冲。用规划器指令速度(无打滑)算出该窗口物理上能走的
        // 最大位移做封顶。按**合速度矢量模**封顶(不逐轴)：过弯不停顿时速度大小≈Turn_V
        // 不变、只方向转→bound 大不触发、切向照切；真正停车时速度大小才趋 0→夹掉虚增。
        // 只缩放外推矢量、保方向，比逐轴归零更准。
        float L_sec = current_L() * 0.001f;
        const Speed2D cmd = App::g_state.control.commanded_vel;
        if (L_sec > 0.0f && std::isfinite(cmd.vx) && std::isfinite(cmd.vy)) {
            float cmd_speed = __builtin_sqrtf(cmd.vx * cmd.vx + cmd.vy * cmd.vy);
            // 末尾刹车时 commanded_vel→0，只用它封顶会把"真实滑行位移"也夹掉→上报位姿滞后
            // →到点判定晚一格→过冲。用编码器实测合速度兜底：滑行(编码器无打滑地反映真实速度)
            // 时按真速度放行，不再滞后；再以 max_vel 封顶，挡住加速打滑时编码器虚增的那部分。
            const auto& w = App::g_state.physical.current_wheel_speed;
            Velocity2D enc_v = Algorithm::Motion::Kinematics::forward(w.lf, w.lb, w.rf, w.rb);
            float enc_speed = __builtin_sqrtf(enc_v.vx * enc_v.vx + enc_v.vy * enc_v.vy);
            float bound_speed = enc_speed > cmd_speed ? enc_speed : cmd_speed;
            bound_speed = clampf(bound_speed, 0.0f, tune.dynamics.max_vel);
            float bound = bound_speed * L_sec * VISION_LATENCY_CMD_CLAMP_MARGIN;
            float delta_sq = out_dx * out_dx + out_dy * out_dy;
            if (delta_sq > bound * bound && delta_sq > 1e-6f) {
                float scale = bound / __builtin_sqrtf(delta_sq);
                out_dx *= scale;
                out_dy *= scale;
            }
        }
        return true;
    }

    bool compensate_vision_latency(const Pose2D& delayed_vision_pose,
                                   uint32_t receive_tick_ms,
                                   Pose2D& compensated,
                                   Pose2D& raw_odom_at_capture) {
        compensated = delayed_vision_pose;
        float odom_dx = 0.0f;
        float odom_dy = 0.0f;
        uint32_t capture_tick_ms = receive_tick_ms;
        s_vision_latency_debug.valid = false;
        s_vision_latency_debug.raw_pose = delayed_vision_pose;
        s_vision_latency_debug.receive_tick_ms = receive_tick_ms;
        if (!odom_delta_since_matched_vision_pose(
                delayed_vision_pose,
                receive_tick_ms,
                capture_tick_ms,
                odom_dx,
                odom_dy,
                raw_odom_at_capture)) {
            return false;
        }

        compensated.x += odom_dx;
        compensated.y += odom_dy;
        s_vision_latency_debug.compensated_pose = compensated;
        s_vision_latency_debug.odom_dx = odom_dx;
        s_vision_latency_debug.odom_dy = odom_dy;
        s_vision_latency_debug.capture_tick_ms = capture_tick_ms;
        s_vision_latency_debug.valid = true;
        return true;
    }

    [[gnu::always_inline]] inline bool calc_smoothed_correction_step(float err,
                                                                     float deadband,
                                                                     float max_step,
                                                                     float gain,
                                                                     float reject_dist,
                                                                     float& out_step) {
        float abs_err = std::abs(err);

        if (abs_err > reject_dist) {
            return false;
        }
        if (abs_err <= deadband) {
            return false;
        }

        if (max_step > 0.0f && abs_err > max_step) {
            err = std::copysign(max_step, err);
        }

        out_step = err * gain;
        return true;
    }

    void set_mileage_scale_anchor(const Pose2D& vision_pose,
                                  const Pose2D& raw_odom_pose,
                                  float along_x,
                                  float along_y,
                                  uint8_t axis) {
        s_mileage_anchor.vision_pose = vision_pose;
        s_mileage_anchor.raw_odom_pose = raw_odom_pose;
        s_mileage_anchor.along_x = along_x;
        s_mileage_anchor.along_y = along_y;
        s_mileage_anchor.axis = axis;
        s_mileage_anchor.valid = true;
        s_mileage_scale_debug.anchor_valid = true;
    }

    void invalidate_mileage_scale_anchor() {
        s_mileage_anchor.valid = false;
        s_mileage_scale_debug.anchor_valid = false;
    }

    /// \brief 用同采集时刻的视觉与原始编码器位移学习 X/Y 里程比例
    ///
    /// \details
    /// 只接受接近全局 X/Y 的直线段，至少走满配置距离后才形成一个样本
    /// 连续两个样本相差不超过 3% 才低通写入，转弯和横移明显的窗口直接丢弃
    ///
    void update_mileage_scale_learning(const Pose2D& raw_vision_pose,
                                       const Pose2D& raw_odom_at_capture,
                                       float along_x,
                                       float along_y) {
        if (!mileage_scale_enabled()) {
            invalidate_mileage_scale_anchor();
            return;
        }

        const float abs_x = std::abs(along_x);
        const float abs_y = std::abs(along_y);
        uint8_t axis = 0xFFU;
        if (abs_x >= MILEAGE_AXIS_ALIGNMENT_MIN && abs_x >= abs_y) axis = 0U;
        else if (abs_y >= MILEAGE_AXIS_ALIGNMENT_MIN) axis = 1U;
        if (axis > 1U) {
            invalidate_mileage_scale_anchor();
            return;
        }

        if (!s_mileage_anchor.valid || s_mileage_anchor.axis != axis ||
            s_mileage_anchor.along_x * along_x + s_mileage_anchor.along_y * along_y < 0.98f) {
            set_mileage_scale_anchor(raw_vision_pose, raw_odom_at_capture,
                                     along_x, along_y, axis);
            return;
        }

        float odom_dx = raw_odom_at_capture.x - s_mileage_anchor.raw_odom_pose.x;
        float odom_dy = raw_odom_at_capture.y - s_mileage_anchor.raw_odom_pose.y;
        float vision_dx = raw_vision_pose.x - s_mileage_anchor.vision_pose.x;
        float vision_dy = raw_vision_pose.y - s_mileage_anchor.vision_pose.y;
        float encoder_ds = odom_dx * along_x + odom_dy * along_y;
        float vision_ds = vision_dx * along_x + vision_dy * along_y;

        float min_distance = tune.vision_long.scale_sample_min_cm;
        if (!std::isfinite(min_distance) || min_distance < 1.0f) min_distance = 20.0f;
        if (encoder_ds < min_distance) {
            if (encoder_ds < -1.0f) {
                set_mileage_scale_anchor(raw_vision_pose, raw_odom_at_capture,
                                         along_x, along_y, axis);
            }
            return;
        }

        float normal_x = -along_y;
        float normal_y = along_x;
        float encoder_cross = odom_dx * normal_x + odom_dy * normal_y;
        float vision_cross = vision_dx * normal_x + vision_dy * normal_y;
        float cross_limit = encoder_ds * MILEAGE_MAX_CROSS_TRACK_RATIO;
        if (cross_limit < MILEAGE_MAX_CROSS_TRACK_CM) cross_limit = MILEAGE_MAX_CROSS_TRACK_CM;
        if (std::abs(encoder_cross) > cross_limit || std::abs(vision_cross) > cross_limit) {
            set_mileage_scale_anchor(raw_vision_pose, raw_odom_at_capture,
                                     along_x, along_y, axis);
            return;
        }

        float sample = vision_ds / encoder_ds;
        float lo = tune.vision_long.scale_min;
        float hi = tune.vision_long.scale_max;
        MileageScaleAxisLearner& learner = s_mileage_axis[axis];
        if (!std::isfinite(sample) || !std::isfinite(lo) || !std::isfinite(hi) ||
            lo > hi || sample < lo || sample > hi) {
            learner.consistent_count = 0U;
            set_mileage_scale_anchor(raw_vision_pose, raw_odom_at_capture,
                                     along_x, along_y, axis);
            return;
        }

        if (learner.consistent_count == 0U ||
            std::abs(sample - learner.candidate) > MILEAGE_SAMPLE_CONSISTENCY_TOL) {
            learner.candidate = sample;
            learner.consistent_count = 1U;
        } else {
            learner.candidate = 0.5f * (learner.candidate + sample);
            if (learner.consistent_count < MILEAGE_REQUIRED_CONSISTENT_SAMPLES) {
                ++learner.consistent_count;
            }
        }

        s_mileage_scale_debug.last_sample = sample;
        s_mileage_scale_debug.last_sample_distance_cm = encoder_ds;
        s_mileage_scale_debug.last_axis = static_cast<uint8_t>(axis + 1U);
        s_mileage_scale_debug.consistent_x = s_mileage_axis[0].consistent_count;
        s_mileage_scale_debug.consistent_y = s_mileage_axis[1].consistent_count;

        if (learner.consistent_count >= MILEAGE_REQUIRED_CONSISTENT_SAMPLES) {
            float alpha = clampf(tune.vision_long.scale_learn_alpha, 0.0f, 1.0f);
            float old_scale = axis == 0U ? s_mileage_scale_x : s_mileage_scale_y;
            float new_scale = clampf((1.0f - alpha) * old_scale + alpha * learner.candidate,
                                     lo, hi);
            if (axis == 0U) s_mileage_scale_x = new_scale;
            else            s_mileage_scale_y = new_scale;
            learner.consistent_count = 0U;
            s_mileage_scale_debug.consistent_x = s_mileage_axis[0].consistent_count;
            s_mileage_scale_debug.consistent_y = s_mileage_axis[1].consistent_count;
        }

        s_mileage_scale_debug.scale_x = s_mileage_scale_x;
        s_mileage_scale_debug.scale_y = s_mileage_scale_y;
        set_mileage_scale_anchor(raw_vision_pose, raw_odom_at_capture,
                                 along_x, along_y, axis);
    }

    // 段法向修正（当前在用）：只把视觉误差沿段法向投影后限步收敛，沿运动方向的分量丢弃
    [[gnu::always_inline]] inline bool apply_projected_lateral_correction(const Pose2D& vision_pose,
                                                                          Pose2D& odom_pose,
                                                                          float segment_dx,
                                                                          float segment_dy,
                                                                          float segment_len_sq) {
        float inv_len = 1.0f / __builtin_sqrtf(segment_len_sq);
        float normal_x = -segment_dy * inv_len;
        float normal_y = segment_dx * inv_len;
        float err = (vision_pose.x - odom_pose.x) * normal_x +
                    (vision_pose.y - odom_pose.y) * normal_y;

        float step = 0.0f;
        if (!calc_smoothed_correction_step(
                err,
                VISION_LATERAL_DEADBAND_CM,
                tune.vision_lateral.max_step_cm,
                tune.vision_lateral.gain,
                tune.tracker.vision_reject_dist,
                step)) {
            return false;
        }

        odom_pose.x += normal_x * step;
        odom_pose.y += normal_y * step;
        return true;
    }

    // 纵向（沿运动方向）缓慢修正：治编码器打滑累积（加速打滑=虚增→欠到；刹车打滑=虚减→过冲）。
    //
    // 与横向的本质区别：横向误差不随时间恶化成"冲过头"，晚 L 知道也能纠回来，所以全程可信；
    // 纵向误差直接变成"该刹车时还以为没到"，因此必须在离目标足够远时才修、进刹车区一律冻结。
    //
    // 冻结窗口 = max(当前速度下的刹车距离, v*L*gain, freeze_floor)：
    //   刹车距离 —— 进了刹车段就别再动位置，否则改一次位置等于改一次刹车曲线起点；
    //   v*L      —— 视觉这帧描述的是 L 毫秒前的车，这段距离内视觉说不了话；
    //   floor    —— 与速度无关的硬下限。低速时前两项都塌到 1cm 以下，此时若仍让视觉推位置，
    //               ±0.5cm 视觉噪声会直接灌进停车判定 → 自激振荡（2026-07-14 踩过）。
    // 两项交叉点 v = 2*brake_acc*L：brake_acc=390、L=0.31s 时约 242cm/s，高于当前 max_vel，
    // 故今天由 v*L 主导；刹车距离项是 L 被在线估计压低后的保险。
    //
    // 关键：step 同时写入 s_encoder_pose。physical.pose 与 s_encoder_pose 吃同一个编码器增量，
    // 差别仅在于视觉修正只写前者；而到达/越线/切段判定全部读后者（要零延迟）。若只写
    // physical.pose，纵向修正就落在一个判定侧不读的量上，对停车精度贡献恰好为零。
    // 因此纵向修正必须两边都写；横向仍只写 physical.pose，保持 s_encoder_pose 横向纯净
    // （切向判定的横向容差不能被视觉延迟污染）。
    [[gnu::always_inline]] inline bool apply_projected_longitudinal_correction(
            const Pose2D& vision_pose,
            const Pose2D& raw_vision_pose,
            const Pose2D& raw_odom_at_capture,
            Pose2D& odom_pose,
            float segment_dx,
            float segment_dy,
            float segment_len_sq,
            const Point2D& segment_end,
            bool is_push_segment) {
        if (!(tune.vision_long.enable > 0.5f)) {
            invalidate_mileage_scale_anchor();
            return false;
        }

        float inv_len = 1.0f / __builtin_sqrtf(segment_len_sq);
        float along_x = segment_dx * inv_len;
        float along_y = segment_dy * inv_len;

        // 剩余距离按零延迟编码器位姿算：判定侧用的就是它，窗口判据必须与判定同源
        float s_remain = (segment_end.x - s_encoder_pose.x) * along_x +
                         (segment_end.y - s_encoder_pose.y) * along_y;
        if (!(s_remain > 0.0f)) {
            invalidate_mileage_scale_anchor();
            return false;
        }

        // 当前速度取编码器实测合速度（无视觉延迟，且刹车滑行时能反映真实速度）
        const auto& w = App::g_state.physical.current_wheel_speed;
        Velocity2D enc_v = Algorithm::Motion::Kinematics::forward(w.lf, w.lb, w.rf, w.rb);
        float v = __builtin_sqrtf(enc_v.vx * enc_v.vx + enc_v.vy * enc_v.vy);
        if (!std::isfinite(v)) return false;

        float brake_acc = tune.dynamics.max_acc * tune.dynamics.brake_limit;
        {
            float cap = tune.dynamics.brake_acc_ceiling;
            if (std::isfinite(cap) && cap > 1.0f && cap < brake_acc) brake_acc = cap;
        }
        float freeze = tune.vision_long.freeze_floor_cm;
        if (brake_acc > 1.0f) {
            float brake_dist = (v * v) / (2.0f * brake_acc);
            if (brake_dist > freeze) freeze = brake_dist;
        }
        float lag_window = v * (current_L() * 0.001f) * tune.vision_long.latency_window_gain;
        if (std::isfinite(lag_window) && lag_window > freeze) freeze = lag_window;

        if (!(s_remain > freeze)) {
            invalidate_mileage_scale_anchor();
            return false;
        }

        if (is_push_segment) invalidate_mileage_scale_anchor();
        else update_mileage_scale_learning(raw_vision_pose, raw_odom_at_capture,
                                           along_x, along_y);

        float err = (vision_pose.x - odom_pose.x) * along_x +
                    (vision_pose.y - odom_pose.y) * along_y;

        float max_step = is_push_segment ? tune.vision_long.push_max_step_cm
                                         : tune.vision_long.max_step_cm;
        if (!(max_step > 0.0f)) return false;

        // 粗差闸传纵向自己的 reject_dist_cm，不能沿用横向的 vision_reject_dist（默认 1cm）：
        // 那是"横向偏这么多就是误检"的尺度，而纵向要治的打滑累积本身就是 1~3cm，沿用会把
        // 待修误差全判成误检、修正恒为 0。更大的真异常仍由 15cm 粗差硬拽兜底。
        float step = 0.0f;
        if (!calc_smoothed_correction_step(
                err,
                VISION_LATERAL_DEADBAND_CM,
                max_step,
                0.70f,
                tune.vision_long.reject_dist_cm,
                step)) {
            return false;
        }

        float step_x = along_x * step;
        float step_y = along_y * step;
        odom_pose.x += step_x;
        odom_pose.y += step_y;
        // 判定侧读的是 s_encoder_pose，纵向必须同步，否则本次修正无人消费。
        // 历史必须整体平移（同 set_encoder_pose_xy 的理由）：延时补偿算的是
        // s_encoder_pose - odom_at_capture，只平移当前值会让这个差每帧多算一个 step，
        // 修正量被反复计入 → 外推越来越大。平移整段历史保持帧间差值不变。
        s_encoder_pose.x += step_x;
        s_encoder_pose.y += step_y;
        for (auto& sample : s_odom_history) {
            if (!sample.valid) continue;
            sample.pose.x += step_x;
            sample.pose.y += step_y;
        }
        return true;
    }

    // 纯视觉全 2D 纠偏：X/Y 两轴各自限步收敛。
    // 2026-07-28 起纵向交给零延迟编码器、视觉只纠段法向，本函数已不被调用，
    // 保留作回滚路径（把 apply_vision_axis_correction 里的调用换回来即可）。
    [[maybe_unused]] [[gnu::always_inline]] inline bool apply_full_2d_correction(const Pose2D& vision_pose,
                                                               Pose2D& odom_pose) {
        constexpr float FULL_MAX_STEP_CM = 5.0f;  // 每帧每轴最大纠偏步长（原 1.5，用户要求加大到 5）
        float step_x = 0.0f, step_y = 0.0f;
        bool any = false;
        if (calc_smoothed_correction_step(vision_pose.x - odom_pose.x,
                                          VISION_LATERAL_DEADBAND_CM, FULL_MAX_STEP_CM,
                                          0.90f,
                                          tune.tracker.vision_reject_dist, step_x)) {
            odom_pose.x += step_x; any = true;
        }
        if (calc_smoothed_correction_step(vision_pose.y - odom_pose.y,
                                          VISION_LATERAL_DEADBAND_CM, FULL_MAX_STEP_CM,
                                          0.90f,
                                          tune.tracker.vision_reject_dist, step_y)) {
            odom_pose.y += step_y; any = true;
        }
        return any;
    }

}


// =============================================================================
// 模块 1: 初始化与坐标重置
// =============================================================================

void init() {
    gyro_z_offset = 0.0f; gyro_x_offset = 0.0f; gyro_y_offset = 0.0f;
    dynamic_deadband = 0.1f;
    is_calibrated = false;
    q0 = 0.70710678f; q1 = 0.0f; q2 = 0.0f; q3 = 0.70710678f;  // 初始化四元数 (Yaw = 90度)
    reset_odom_history(App::g_state.physical.pose, 0);
    reset_mileage_scale_learner(true);
    reset_latency_estimator(true);  // 开机全清，包括已学到的 L
    imu_icm42688.init();  // ICM42688 IMU 初始化 (spi)
}

/// \brief 强制设置当前物理位姿
/// \param x 全局 X 坐标 cm
/// \param y 全局 Y 坐标 cm
/// \param yaw_deg 全局 yaw 角度，单位度
///
/// \details
/// 会同时重置 yaw 四元数和全局位姿，常用于出库前初始定位或视觉标定成功后同步状态
///
void set_position(float x, float y, float yaw_deg) {
    // 1. 重置坐标
    App::g_state.physical.pose.x = x;
    App::g_state.physical.pose.y = y;

    // 2. 将传入的初始偏航角(度)转换为四元数
    float yaw_rad_half = (yaw_deg * SystemConfig::DEG_TO_RAD) * 0.5f;
    q0 = std::cos(yaw_rad_half); q1 = 0.0f; q2 = 0.0f; q3 = std::sin(yaw_rad_half);
    
    // 3. 同步状态树
    App::g_state.physical.pose.yaw = yaw_deg;
    reset_odom_history(App::g_state.physical.pose, Core::Scheduler::get_sys_tick_ms());
    reset_mileage_scale_learner(false);
    reset_latency_estimator(false);  // 瞬移/硬重置：清检测器与 FIFO，保留已学 L（延时本身不变）
}


// =============================================================================
// 模块 2: IMU 数据更新与融合算法
// =============================================================================

const DebugProbes& get_debug_probes() {
    return g_probes;
}

/// \brief IMU 开机静态标定
///
/// \details
/// 丢弃前 100 帧后累计 600 帧陀螺仪数据求零偏，并根据 Z 轴噪声标准差生成动态死区
/// 函数是阻塞式启动流程，只应在开机静止时调用
///
void calibrate_gyro_step() {
    if (is_calibrated) return; 
    
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;     
    float calib_sq_sum = 0.0f; 

    // 丢弃前100次数据，等传感器内部滤波稳定
    for(int i=0; i<100; ++i) {
        imu_icm42688.update_gyro_only(); 
        system_delay_ms(5);
    }

    for (int i = 0; i < 600; ++i) {
        imu_icm42688.update_gyro_only();       
        gx_sum += imu_icm42688.data.gyro_x;
        gy_sum += imu_icm42688.data.gyro_y; 
        gz_sum += imu_icm42688.data.gyro_z;
        calib_sq_sum += (imu_icm42688.data.gyro_z * imu_icm42688.data.gyro_z);
        system_delay_ms(5);                    
    }

    gyro_x_offset = gx_sum / 600.0f;
    gyro_y_offset = gy_sum / 600.0f;
    gyro_z_offset = gz_sum / 600.0f;

    // 计时标准差，将死区设为标准差的 3 倍，外加一个极小的系统余量(0.02)防低频震动
    float variance = (calib_sq_sum / 600.0f) - (gyro_z_offset * gyro_z_offset);
    float std_dev = std::sqrt(std::abs(variance));
    dynamic_deadband = std_dev * 3.0f + 0.02f; 
    
    // ===========================================================================
    // 初始化探针的起始 Pitch，让三根线开局重合
    imu_icm42688.update_accel_only();
    float ax = imu_icm42688.data.acc_x;
    float ay = imu_icm42688.data.acc_y;
    float az = imu_icm42688.data.acc_z;
    if (std::sqrt(ax*ax + ay*ay + az*az) > 0.5f) { // 防止出现全0异常
        g_probes.pitch_gyro = std::atan2(-ax, __builtin_sqrtf(ay*ay + az*az)) * SystemConfig::RAD_TO_DEG;
    } else {
        g_probes.pitch_gyro = 0.0f;
    }
    // ===========================================================================

    is_calibrated = true; 
}

/// \brief 自适应 Mahony 六轴姿态融合
///
/// \details
/// 当加速度模长偏离 1G 时衰减加速度修正增益，减少急加减速对姿态的污染
/// 输入角速度单位为 rad/s，加速度单位为 G
///
__attribute__((section(".ramfunc")))
void adaptive_mahony_update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;

    // 计算加速度模长
    float acc_sq = ax*ax + ay*ay + az*az;
    float acc_norm = __builtin_sqrtf(acc_sq);

    // 动态 Kp 屏蔽线加速度干扰
    float acc_error = std::abs(acc_norm - 1.0f); // 假设 acc 已经标定到 1.0 = 1G
    float Kp_adaptive = tune.estimate.mahony_kp; // 基础 Kp 参数

    if (acc_error > 0.1f) {
        // 线性映射: error 从 0.1 到 0.2，增益从 1 衰减到 0
        float attenuation = 1.0f - (acc_error - 0.1f) * 10.0f; 
        Kp_adaptive *= std::clamp(attenuation, 0.0f, 1.0f);
    }

    // ===========================================================================
    // 将计算出的实时 Kp 喂给探针，用于波形分析和调参参考
    // g_probes.kp_adaptive = Kp_adaptive;
    // ===========================================================================

    // 只在有加速度数据时进行重力纠正
    if(acc_sq > 1e-6f) {
        // 归一化加速度
        norm = fast_inv_sqrt(acc_sq);
        ax *= norm; ay *= norm; az *= norm;

        // 根据当前四元数推算出的重力分量
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 叉乘计算误差
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // PI 修正
        gx += Kp_adaptive * ex;
        gy += Kp_adaptive * ey;
        gz += Kp_adaptive * ez;
    }

    // 四元数的一阶龙格库塔积分
    float half_dt = 0.5f / SAMPLE_FREQ;
    gx *= half_dt; gy *= half_dt; gz *= half_dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数，防止发散
    norm = fast_inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}

/// \brief 1ms 姿态解算周期
///
/// \details
/// 从最新 IMU 数据扣除零偏和死区，执行 Mahony 融合并更新全局 yaw
/// 该函数在 PIT_CH0 中断中调用，应保持固定耗时
///
__attribute__((section(".ramfunc")))
void update_yaw_1ms_tick() {
    if (!is_calibrated) return; 

    float raw_gx = imu_icm42688.data.gyro_x;
    float raw_gy = imu_icm42688.data.gyro_y;
    float raw_gz = imu_icm42688.data.gyro_z;
    float ax = imu_icm42688.data.acc_x;
    float ay = imu_icm42688.data.acc_y;
    float az = imu_icm42688.data.acc_z;

    // ZUPT 零速修正
    // gyro_buffer[buf_idx] = raw_gz;
    // buf_idx = (buf_idx + 1) % 20;

    // float max_g = gyro_buffer[0], min_g = gyro_buffer[0];
    // for(int i=1; i<20; i++) {
    //     if(gyro_buffer[i] > max_g) max_g = gyro_buffer[i];
    //     if(gyro_buffer[i] < min_g) min_g = gyro_buffer[i];
    // }

    // // 如果指令停止，且物理底噪极小 (小于标定死区的 2倍)
    // if (App::g_state.physical.is_stopped && (max_g - min_g) < (dynamic_deadband * 2.0f)) { 
    //     stop_settle_counter++;
    //     if (stop_settle_counter > 50) {
    //         // 极速吸收温漂
    //         gyro_z_offset = gyro_z_offset * 0.999f + raw_gz * 0.001f; 
    //         raw_gz = gyro_z_offset; // 强行让本次纯净输入归零
    //     }
    // } else {
    //     stop_settle_counter = 0;
    // }

    // 扣除零偏，并压入动态死区
    float pure_gx = raw_gx - gyro_x_offset;
    float pure_gy = raw_gy - gyro_y_offset;  
    float pure_gz = raw_gz - gyro_z_offset; 
    if (pure_gz > -dynamic_deadband && pure_gz < dynamic_deadband) {
        pure_gz = 0.0f;
    }

    // 转换为 弧度/秒
    float gx_rad = pure_gx * SystemConfig::DEG_TO_RAD;
    float gy_rad = pure_gy * SystemConfig::DEG_TO_RAD;
    float gz_rad = pure_gz * SystemConfig::DEG_TO_RAD;

    // 发布 yaw 角速度供控制层做陀螺阻尼，符号与下面 Mahony/yaw 积分方向一致
    App::g_state.physical.yaw_rate = gz_rad;

    // ===============================================================================================
    // // 探针数据更新
    // float acc_sq = ax*ax + ay*ay + az*az;
    // float acc_norm = __builtin_sqrtf(acc_sq);
    // g_probes.acc_norm = acc_norm; 

    // if (acc_norm > 0.1f) {
    //     g_probes.pitch_acc = std::atan2(-ax, __builtin_sqrtf(ay*ay + az*az)) * SystemConfig::RAD_TO_DEG;
    // }
    
    // // 陀螺仪积分 (不受加速度污染)
    // g_probes.pitch_gyro += (pure_gy * SystemConfig::DEG_TO_RAD) * (1.0f / SAMPLE_FREQ) * SystemConfig::RAD_TO_DEG;
    // ===============================================================================================

    // 利用带有加速度屏蔽的 Mahony算法 进行 6 轴融合
    adaptive_mahony_update(gx_rad, gy_rad, gz_rad, ax, ay, az);

    // ===============================================================================================
    // 记录融合后的 Pitch
    // g_probes.pitch_mahony = std::asin(2.0f * (q0 * q2 - q1 * q3)) * SystemConfig::RAD_TO_DEG;
    // ===============================================================================================

    // 从四元数中提取 Yaw 轴角度 [数学公式：Yaw = atan2(2(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)]
    float yaw_rad = std::atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    float yaw_deg = yaw_rad * SystemConfig::RAD_TO_DEG; 

    // 角度规范化 (0 ~ 360)
    if (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    }
    
    // 更新全局状态
    App::g_state.physical.pose.yaw = yaw_deg;
}


// =============================================================================
// 模块 3: 编码器定位
// =============================================================================

/// \brief 20ms 里程计位置更新
/// \param encoder_counts 四轮编码器周期增量
/// \param current_yaw_deg 当前 yaw 角度，单位度
///
/// \details
/// 先把四轮增量解算为车体系位移，再用当前 yaw 投影到全局坐标
/// kinematic_gain_x/y 用于补偿底盘安装和滑移造成的比例误差
///
__attribute__((section(".ramfunc")))
void update_position_20ms_tick(const int16_t* encoder_counts, float current_yaw_deg) {

    float current_yaw_rad = current_yaw_deg * SystemConfig::DEG_TO_RAD;

    // 将编码器计数转换为轮子移动的距离（cm）
    float d_lf = encoder_counts[0] * PLUSE_TO_CM;
    float d_lb = encoder_counts[1] * PLUSE_TO_CM;
    float d_rf = encoder_counts[2] * PLUSE_TO_CM;
    float d_rb = encoder_counts[3] * PLUSE_TO_CM;

    // 计算机器人在局部坐标系中的位移
    float dx_local_raw = (d_lf - d_lb - d_rf + d_rb) / 4.0f;
    float dy_local_raw = (d_lf + d_lb + d_rf + d_rb) / 4.0f;

    // 动力学滑移增益补偿
    float dx_local = dx_local_raw / tune.dynamics.kinematic_gain_x;
    float dy_local = dy_local_raw / tune.dynamics.kinematic_gain_y;

    // 将局部坐标系的位移转换到全局坐标系
    float cos_yaw = cosf(current_yaw_rad);
    float sin_yaw = sinf(current_yaw_rad);
    float dx_global = dx_local * sin_yaw + dy_local * cos_yaw;
    float dy_global = -dx_local * cos_yaw + dy_local * sin_yaw;

    // 原始轨迹专供视觉学习，始终保留未吃在线比例的编码器增量
    s_raw_encoder_pose.x += dx_global;
    s_raw_encoder_pose.y += dy_global;
    s_raw_encoder_pose.yaw = current_yaw_deg;

    // X/Y 比例分别作用于之后的里程增量，末端冻结视觉时仍继续使用最后确认值
    float corrected_dx_global = dx_global * mileage_scale_for_axis(0U);
    float corrected_dy_global = dy_global * mileage_scale_for_axis(1U);

    App::g_state.physical.pose.x += corrected_dx_global;
    App::g_state.physical.pose.y += corrected_dy_global;

    s_encoder_pose.x += corrected_dx_global;
    s_encoder_pose.y += corrected_dy_global;
    s_encoder_pose.yaw = current_yaw_deg;

    uint32_t now = Core::Scheduler::get_sys_tick_ms();
    push_odom_history(now, s_encoder_pose, s_raw_encoder_pose);

    // 用本周期的全局位移矢量驱动编码器侧拐点检测（实时延时估计）
    feed_encoder_sample(corrected_dx_global, corrected_dy_global, now);
}


// =============================================================================
// 模块 4: 视觉标定与坐标修正
// =============================================================================

/// \brief 取零延迟控制里程位姿
Pose2D get_encoder_pose() {
    return s_encoder_pose;
}

/// \brief 把控制里程位姿 XY 重新钉到给定坐标
///
/// \details
/// 只做平移：s_encoder_pose 与整个 s_odom_history 一起加同一个偏移量。
/// 不能用 reset_odom_history —— 那会把历史清空，导致此后 ~L(≈320ms) 内
/// match_vision_pose_to_odom_history 找不到样本，横向修正整段丢失。
void set_encoder_pose_xy(float x, float y) {
    float ox = x - s_encoder_pose.x;
    float oy = y - s_encoder_pose.y;
    if (!std::isfinite(ox) || !std::isfinite(oy)) {
        return;
    }
    s_encoder_pose.x = x;
    s_encoder_pose.y = y;
    for (auto& sample : s_odom_history) {
        if (!sample.valid) continue;
        sample.pose.x += ox;
        sample.pose.y += oy;
    }
    invalidate_mileage_scale_anchor();
}

/// \brief 用最新视觉位姿修正段法向、纵向里程和在线比例
/// \param segment_start 当前直线段起点，与 segment_end 一起定义运动方向
/// \param segment_end 当前直线段终点（当前追踪目标）
/// \param last_consumed_seq 外层保存的最后一帧已处理视觉序号
/// \return 成功应用视觉修正时返回 true
///
/// \details
/// 先用编码器纯积分补偿视觉管线延时(tick 查表 + 实时 L)，再把视觉误差投影到
/// **段法向**做限步收敛，并在末端冻结区外缓慢修正纵向坐标
/// 同采集时刻的原始视觉/编码器位移还会用于学习之后的 X/Y 编码器里程比例
/// 段向量退化(<1cm，如原地保持/锁点)时方向不可靠，本帧不修，纯靠编码器保持。
/// 与编码器积分偏离过大(粗差跳变)时仍走硬贴合并重置历史保护。
/// 按视觉帧序号消费新数据，不清 art1_pose_updated，避免与标定/调试流程抢同一个 bool 标志。
///
bool apply_vision_axis_correction(const Point2D& segment_start, const Point2D& segment_end,
                                  uint32_t& last_consumed_seq,
                                  bool allow_near_target_correction,
                                  bool is_push_segment) {
    (void)allow_near_target_correction;

    auto& vision_data = App::g_state.vision;
    uint32_t seq = vision_data.art1_pose_seq;
    if (seq == 0 || seq == last_consumed_seq) {
        return false;
    }
    last_consumed_seq = seq;

    // ART1 位姿可能比底盘周期慢，过期数据不参与闭环修正
    uint32_t pose_age_ms = Core::Scheduler::get_sys_tick_ms() - vision_data.art1_pose_tick_ms;
    if (pose_age_ms > VISION_POSE_MAX_AGE_MS) {
        return false;
    }

    Pose2D raw_vision_pose = vision_data.art1_pose_buffer[vision_data.art1_pose_publish_idx];
    if (!std::isfinite(raw_vision_pose.x) || !std::isfinite(raw_vision_pose.y) || !std::isfinite(raw_vision_pose.yaw)) {
        return false;
    }
    Pose2D vision_pose;
    Pose2D raw_odom_at_capture;
    if (!compensate_vision_latency(raw_vision_pose, vision_data.art1_pose_tick_ms,
                                   vision_pose, raw_odom_at_capture)) {
        return false;
    }

    auto& pose = App::g_state.physical.pose;
    // 粗差跳变保护：与编码器纯积分偏离过大时直接硬贴合并重置历史
    float encoder_err_x = vision_pose.x - s_encoder_pose.x;
    float encoder_err_y = vision_pose.y - s_encoder_pose.y;
    float encoder_err_sq = encoder_err_x * encoder_err_x + encoder_err_y * encoder_err_y;
    if (encoder_err_sq >= VISION_ENCODER_RESET_THRESHOLD_CM * VISION_ENCODER_RESET_THRESHOLD_CM) {
        float before_x = pose.x;
        float before_y = pose.y;
        pose.x = vision_pose.x;
        pose.y = vision_pose.y;
        reset_odom_history(pose, Core::Scheduler::get_sys_tick_ms());
        reset_mileage_scale_learner(false);
        s_vision_latency_debug.correction_x = pose.x - before_x;
        s_vision_latency_debug.correction_y = pose.y - before_y;
        return true;
    }

    // 段法向：全程纠（横向对延迟不敏感，防蹭箱/贴线该信视觉）
    // 段方向：只在离目标足够远时缓慢纠（治打滑累积），进刹车/切向区冻结、交零延迟编码器
    float seg_dx = segment_end.x - segment_start.x;
    float seg_dy = segment_end.y - segment_start.y;
    float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
    if (seg_len_sq < VISION_LATERAL_MIN_SEGMENT_LEN_SQ) {
        invalidate_mileage_scale_anchor();
        return false;   // 段太短/退化：方向不可靠，本帧不修
    }

    float before_x = pose.x;
    float before_y = pose.y;
    bool applied = apply_projected_lateral_correction(vision_pose, pose,
                                                     seg_dx, seg_dy, seg_len_sq);
    applied = apply_projected_longitudinal_correction(vision_pose,
                                                      raw_vision_pose,
                                                      raw_odom_at_capture,
                                                      pose,
                                                      seg_dx, seg_dy, seg_len_sq,
                                                      segment_end, is_push_segment) || applied;

    s_vision_latency_debug.correction_x = pose.x - before_x;
    s_vision_latency_debug.correction_y = pose.y - before_y;
    return applied;
}

/// \brief 主循环在每个稳定视觉帧上喂入帧间位移，驱动视觉侧拐点检测与延时配对
/// \param dx 相对上一帧的 X 位移 cm
/// \param dy 相对上一帧的 Y 位移 cm
/// \param gap_ms 与上一帧的时间间隔 ms
__attribute__((section(".ramfunc")))
void notify_vision_inflection(float dx, float dy, uint32_t gap_ms) {
    if (gap_ms == 0U) return;  // 防止零间隔（同一帧重复）
    feed_vision_sample(dx, dy, Core::Scheduler::get_sys_tick_ms());
}

void reset_async_calibrate() {
    s_calib_state = AsyncCalibState::IDLE;
}

/// \brief 异步非阻塞视觉标定函数
/// \param timeout_ms 标定最长等待时间 ms
/// \param reject_threshold 视觉结果与当前位姿的最大允许偏差 cm
/// \return 当前标定状态
///
/// \details
/// 成功时会用收敛后的视觉坐标覆盖当前里程计位置，yaw 保持当前陀螺仪估计
/// 标定过程按视觉帧序号消费新数据，不依赖 art1_pose_updated 标志
///
__attribute__((section(".ramfunc"))) 
AsyncCalibState async_calibrate_vision(uint32_t timeout_ms, float reject_threshold) {
    auto& pos = App::g_state.physical.pose;
    auto& ctrl = App::g_state.control;
    auto& vision_data = App::g_state.vision;

    // 如果是第一次进入，执行初始化动作
    if (s_calib_state == AsyncCalibState::IDLE) {
        ctrl.mode = ControlMode::POINT_TRACKING; // 锁死底盘
        vision_calibrator.reset();
        // 记录进入标定前的序号，只消费之后到达的新视觉帧
        s_last_calib_vision_seq = vision_data.art1_pose_seq;
        s_calib_state = AsyncCalibState::BUSY;
        return AsyncCalibState::BUSY;
    }

    // 如果已经结束了，直接返回结果，防止被重复执行
    if (s_calib_state != AsyncCalibState::BUSY) {
        return s_calib_state;
    }

    // --- 下面是 BUSY 状态下的非阻塞逻辑 ---
    // 检查超时
    if (vision_calibrator.is_timed_out(timeout_ms)) {
        char msg[128]; snprintf(msg, sizeof(msg), "[VIS_CALIB] TIMEOUT! Frames collected: %d. Skipped.\r\n", vision_calibrator.get_count());
        wireless_uart_send_buffer((uint8_t*)msg, strlen(msg));

        s_calib_state = AsyncCalibState::ERROR;
        return s_calib_state;
    }

    // 喂入新数据
    uint32_t seq = vision_data.art1_pose_seq;
    if (seq != s_last_calib_vision_seq) {
        // 按序号取新帧，而不是靠 bool 标志，避免标志被其它校正流程清掉
        s_last_calib_vision_seq = seq;
        Pose2D vision_pose = vision_data.art1_pose_buffer[vision_data.art1_pose_publish_idx];
        vision_calibrator.push(vision_pose.x, vision_pose.y, vision_pose.yaw);
    }

    // 检查收敛
    if (vision_calibrator.is_converged()) {
        auto optimal = vision_calibrator.get_optimal_pose();
        
        float dx = std::abs(optimal.x - pos.x);
        float dy = std::abs(optimal.y - pos.y);

        if (dx > reject_threshold || dy > reject_threshold) {
            Subsystem::Telemetry::log_vision_calibration(optimal.x, optimal.y, optimal.yaw, pos.x, pos.y, pos.yaw, false);
            s_calib_state = AsyncCalibState::ERROR;
        } else {
            Subsystem::Telemetry::log_vision_calibration(optimal.x, optimal.y, optimal.yaw, pos.x, pos.y, pos.yaw, true);

            pos.x = optimal.x; 
            pos.y = optimal.y;
            ctrl.current_target.x = optimal.x; 
            ctrl.current_target.y = optimal.y;

            s_calib_state = AsyncCalibState::SUCCESS;
        }
    }

    return s_calib_state; // BUSY
}

const VisionLatencyDebug& get_vision_latency_debug() {
    return s_vision_latency_debug;
}

const MileageScaleDebug& get_mileage_scale_debug() {
    s_mileage_scale_debug.scale_x = s_mileage_scale_x;
    s_mileage_scale_debug.scale_y = s_mileage_scale_y;
    return s_mileage_scale_debug;
}


} // namespace Subsystem::PoseEstimator
