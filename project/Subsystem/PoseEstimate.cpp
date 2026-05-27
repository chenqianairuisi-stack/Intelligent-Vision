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

    // 常量定义
    constexpr float PLUSE_TO_CM = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / SystemConfig::PULSES_PER_REV;  // 编码器计数转换为轮子移动的距离（cm）
    constexpr float SAMPLE_FREQ = 1.0f / SystemConfig::PIT_CH0_DT_S;  // 采样频率 (Hz)，根据系统定时器周期计算
    constexpr uint32_t VISION_POSE_MAX_AGE_MS = 300;  // 视觉位姿数据的最大有效时间，超过这个时间就丢弃

    // 快速逆平方根函数，供 Mahony 算法使用
    [[gnu::always_inline]] inline float fast_inv_sqrt(float x) {
        return 1.0f / __builtin_sqrtf(x);
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

    // 更新全局物理位姿
    App::g_state.physical.pose.x += dx_global;
    App::g_state.physical.pose.y += dy_global;
}


// =============================================================================
// 模块 4: 视觉标定与坐标修正
// =============================================================================

/// \brief 根据最新视觉位姿修正当前直线段的横向坐标
/// \param segment_start 当前直线段起点
/// \param segment_end 当前直线段终点
/// \return 成功应用视觉修正时返回 true
///
/// \details
/// 只修横向轴，不修前进轴，避免视觉抖动影响沿路径前进的里程计积分
/// 函数会消费 art1_pose_updated 标志，同一路径段外层只应调用一次
///
bool apply_vision_axis_correction(const Point2D& segment_start, const Point2D& segment_end) {
    float dx = segment_end.x - segment_start.x;
    float dy = segment_end.y - segment_start.y;
    if ((dx * dx + dy * dy) < 1.0f) {
        // 段长过短时无法可靠判断主轴和横向轴
        return false;
    }

    auto& vision_data = App::g_state.vision;
    if (!vision_data.art1_pose_updated) {
        return false;
    }
    // 本次校正消费掉这帧视觉位姿，下一段必须等新帧
    vision_data.art1_pose_updated = false;

    if (vision_data.art1_pose_seq == 0) {
        return false;
    }

    // ART1 位姿可能比底盘周期慢，过期数据不参与闭环修正
    uint32_t pose_age_ms = Core::Scheduler::get_sys_tick_ms() - vision_data.art1_pose_tick_ms;
    if (pose_age_ms > VISION_POSE_MAX_AGE_MS) {
        return false;
    }

    const Pose2D& vision_pose = vision_data.art1_pose;
    if (!std::isfinite(vision_pose.x) || !std::isfinite(vision_pose.y) || !std::isfinite(vision_pose.yaw)) {
        return false;
    }

    auto& pose = App::g_state.physical.pose;
    float abs_dx = std::abs(dx);
    float abs_dy = std::abs(dy);

    // 水平段只修 Y，竖直段只修 X，前进轴继续交给编码器积分
    if (abs_dx >= abs_dy && abs_dy <= 2.0f) {
        if (std::abs(vision_pose.y - pose.y) > tune.tracker.vision_reject_dist) {
            return false;
        }
        pose.y = vision_pose.y;
        return true;
    }

    if (abs_dy > abs_dx && abs_dx <= 2.0f) {
        if (std::abs(vision_pose.x - pose.x) > tune.tracker.vision_reject_dist) {
            return false;
        }
        pose.x = vision_pose.x;
        return true;
    }

    return false;
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


} // namespace Subsystem::PoseEstimator
