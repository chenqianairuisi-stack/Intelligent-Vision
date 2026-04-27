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

#include "Icm42688.h"
#include "Encoder.h"
#include <cmath>

namespace Subsystem::PoseEstimator {

namespace { 
    // ==========================================
    // 基础状态变量
    // ==========================================
    float gyro_z_offset = 0.0f;         // 陀螺仪Z轴零偏 (deg/s)
    float acc_x_offset = 0.0f;          // 加速度计X轴偏移 (cm/s^2)，用于标定后修正倾斜引起的静态加速度误差
    float acc_y_offset = 0.0f;          // 加速度计Y轴偏移 (cm/s^2)，同上
    bool is_calibrated = false;         // 标定完成标志位
    float dynamic_deadband = 0.0f;      // 动态计算出的噪声死区
    uint16_t stop_settle_counter = 0;   // 停车缓冲震荡计数器

    constexpr float PLUSE_TO_CM = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / SystemConfig::PULSES_PER_REV;  // 编码器计数转换为轮子移动的距离（cm）

    // ==========================================
    // 四元数与 Mahony 算法专用变量
    // ==========================================
    // 四元数初始化为绝对水平、Yaw为 90度 [cos(45deg) = 0.70710678f, sin(45deg) = 0.70710678f]
    float q0 = 0.70710678f, q1 = 0.0f, q2 = 0.0f, q3 = 0.70710678f;

    constexpr float SAMPLE_FREQ = 1.0f / SystemConfig::PIT_CH0_DT_S;  // 采样频率 (Hz)，根据系统定时器周期计算
}


// 初始化与标定
void init() {
    gyro_z_offset = 0.0f;
    dynamic_deadband = 0.1f; // 给一个安全的初始值
    is_calibrated = false;

    // 初始化四元数 (Yaw = 90度)
    q0 = 0.70710678f; q1 = 0.0f; q2 = 0.0f; q3 = 0.70710678f; 

    imu_icm42688.init();  // ICM42688 IMU 初始化 (spi)
}

// IMU 开机静态标定，累计 600 次数据求平均，得到 gyro_z_offset
void calibrate_gyro_step() {
    if (is_calibrated) return; 
    
    float calib_sum = 0.0f;       
    float ax_sum = 0.0f;
    float ay_sum = 0.0f;   
    float calib_sq_sum = 0.0f; 

    for (int i = 0; i < 600; ++i) {
        imu_icm42688.update_gyro_only();       
        float z = imu_icm42688.data.gyro_z;
        calib_sum += z; 
        calib_sq_sum += (z * z);
        ax_sum += imu_icm42688.data.acc_x;
        ay_sum += imu_icm42688.data.acc_y;
        system_delay_ms(5);                    
    }

    gyro_z_offset = calib_sum / 600.0f;
    acc_x_offset = ax_sum / 600.0f;
    acc_y_offset = ay_sum / 600.0f;    

    // 计算标准差 (Standard Deviation)
    float variance = (calib_sq_sum / 600.0f) - (gyro_z_offset * gyro_z_offset);
    float std_dev = std::sqrt(std::abs(variance));
    
    // 3-Sigma 原则：将死区设为标准差的 3 倍，外加一个极小的系统余量(0.02)防低频震动
    dynamic_deadband = std_dev * 3.0f + 0.02f; 
    
    is_calibrated = true; 
}


// 强行重置里程计坐标
void set_position(float x, float y, float yaw_deg) {
    // 1. 重置坐标
    App::g_state.physical.pose.x = x;
    App::g_state.physical.pose.y = y;

    // 2. 将传入的初始偏航角(度)转换为四元数
    float yaw_rad_half = (yaw_deg * SystemConfig::DEG_TO_RAD) * 0.5f;
    
    // 因为是平放在地上摆车，所以假设 Roll 和 Pitch 均为 0
    q0 = std::cos(yaw_rad_half);
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = std::sin(yaw_rad_half);
    
    // 3. 同步状态树
    App::g_state.physical.pose.yaw = yaw_deg;
}


// ==========================================
// 自适应四元数解算引擎 (Adaptive Mahony)
// ==========================================
__attribute__((section(".ramfunc")))
void adaptive_mahony_update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 快速逆平方根函数
    auto invSqrt = [](float x) -> float {
        return 1.0f / std::sqrt(x); 
    };

    // 动态 Kp 屏蔽线加速度干扰
    float Kp_adaptive = tune.estimate.mahony_kp; // 基础 Kp 参数
    float acc_norm = std::sqrt(ax*ax + ay*ay + az*az);
    
    // 当合加速度不在 0.85g ~ 1.15g 范围内时（急加速/急刹车/剧烈撞击）
    if (acc_norm < 0.9f || acc_norm > 1.1f) {
        Kp_adaptive = 0.0f; // 瞬间屏蔽加速度计，纯靠陀螺仪硬扛！防止矩阵畸变
    }

    // 只在有加速度数据时进行重力纠正
    if(ax != 0.0f || ay != 0.0f || az != 0.0f) {
        // 归一化加速度
        norm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= norm; ay *= norm; az *= norm;

        // 根据当前四元数推算出的重力分量
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 叉乘计算误差
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // PI 修正
        gx += Kp_adaptive * ex;
        gy += Kp_adaptive * ey;
        gz += Kp_adaptive * ez;
    }

    // 四元数的一阶龙格库塔积分
    gx *= (0.5f * (1.0f / SAMPLE_FREQ));
    gy *= (0.5f * (1.0f / SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / SAMPLE_FREQ));

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数，防止发散
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}


// ==========================================
// 1ms 姿态解算定时器
// ==========================================
__attribute__((section(".ramfunc")))
void update_yaw_1ms_tick() {
    if (!is_calibrated) return; 

    float raw_gx = imu_icm42688.data.gyro_x;
    float raw_gy = imu_icm42688.data.gyro_y;
    float raw_gz = imu_icm42688.data.gyro_z;
    float ax = imu_icm42688.data.acc_x - acc_x_offset;
    float ay = imu_icm42688.data.acc_y - acc_y_offset;
    float az = imu_icm42688.data.acc_z;

    // 1. 扣除零偏，并压入动态死区 (只对 Z 轴做死区，X Y留给四元数吸收震动)
    float pure_gz = raw_gz - gyro_z_offset;   
    if (pure_gz > -dynamic_deadband && pure_gz < dynamic_deadband) {
        pure_gz = 0.0f;
    }

    // 2. 转换为 弧度/秒
    float gx_rad = raw_gx * SystemConfig::DEG_TO_RAD;
    float gy_rad = raw_gy * SystemConfig::DEG_TO_RAD;
    float gz_rad = pure_gz * SystemConfig::DEG_TO_RAD;

    //3. ZUPT 零速修正
    // if (App::g_state.physical.is_stopped) { 
    //     stop_settle_counter++;
        
    //     // 只有当持续静止超过 200ms 后，开始偷偷吸收温漂
    //     if (stop_settle_counter > 200) {
    //         // 极慢速低通滤波吸收 Z 轴漂移 (0.998 与 0.002 构成完美低通)
    //         gyro_z_offset = gyro_z_offset * 0.996f + raw_gz * 0.004f; 
    //     }
        
    //     // 强制角速度归零，不让它积分虚假震荡噪声，并让 Mahony 利用纯净的 1G 重力纠正状态到绝对水平
    //     gx_rad = 0.0f; gy_rad = 0.0f; gz_rad = 0.0f;
    // } else {
    //     stop_settle_counter = 0;
    // }

    // 4. 利用带有加速度屏蔽的 Mahony算法 进行 6 轴融合
    adaptive_mahony_update(gx_rad, gy_rad, gz_rad, ax, ay, az);

    // 5. 从四元数中提取 Yaw 轴角度 [数学公式：Yaw = atan2(2(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)]
    float yaw_rad = std::atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    float yaw_deg = yaw_rad * SystemConfig::RAD_TO_DEG; 

    // 角度规范化 (0 ~ 360)
    if (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    }
    
    // 更新全局状态
    App::g_state.physical.pose.yaw = yaw_deg;
}

// ==========================================
// PIT_CH1 里程计更新定时器
// ==========================================
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

} // namespace Subsystem::PoseEstimator