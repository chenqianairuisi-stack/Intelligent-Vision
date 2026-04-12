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

#include "Icm42688.h"
#include "Encoder.h"
#include <cmath>

namespace Subsystem::PoseEstimator {

namespace { 
    float last_gyro_z = 0.0f;         // 上一次的陀螺仪Z轴读数
    float gyro_z_offset = 0.0f;       // 陀螺仪Z轴零偏 (单位：度/秒)
    bool is_calibrated = false;       // 标定完成标志位
    float dynamic_deadband = 0.0f;    // 动态计算出的噪声死区
    uint16_t stop_settle_counter = 0; // 停车缓冲震荡计数器

    constexpr float PLUSE_TO_CM = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / SystemConfig::PULSES_PER_REV;  // 编码器计数转换为轮子移动的距离（cm）
}

// 初始化与标定
void init() {
    gyro_z_offset = 0.0f;
    dynamic_deadband = 0.1f; // 给一个安全的初始值
    is_calibrated = false;
    imu_icm42688.init();  // ICM42688 IMU 初始化 (spi)
}

// IMU 开机静态标定，累计 500 次数据求平均，得到 gyro_z_offset
void calibrate_gyro_step() {
    if (is_calibrated) return; 
    
    float calib_sum = 0.0f;          
    float calib_sq_sum = 0.0f; 

    for (int i = 0; i < 600; ++i) {
        imu_icm42688.update_gyro_only();       
        float z = imu_icm42688.data.gyro_z;
        calib_sum += z; 
        calib_sq_sum += (z * z);
        system_delay_ms(5);                    
    }

    gyro_z_offset = calib_sum / 600.0f;
    
    // 计算标准差 (Standard Deviation)
    float variance = (calib_sq_sum / 600.0f) - (gyro_z_offset * gyro_z_offset);
    float std_dev = std::sqrt(std::abs(variance));
    
    // 3-Sigma 原则：将死区设为标准差的 3 倍，外加一个极小的系统余量(0.02)防低频震动
    dynamic_deadband = std_dev * 3.0f + 0.02f; 
    
    is_calibrated = true; 
}


// 强行重置里程计坐标
void set_position(float x, float y) {
    App::g_state.physical.pose.x = x;
    App::g_state.physical.pose.y = y;
}


// 5ms 中断调用，进行 yaw_angle 更新
__attribute__((section(".ramfunc")))
void update_yaw_5ms_tick() {
    if (!is_calibrated) return; // 没校准完绝对不积分

    float raw_gyro_z = imu_icm42688.data.gyro_z;

    // ==========================================
    // 带缓冲死锁的 ZUPT 零速修正
    // ==========================================
    // if (App::g_state.physical.is_stopped) {
    //     stop_settle_counter++;
        
    //     // 只有当持续静止超过 200ms 后，开始吸收温漂
    //     if (stop_settle_counter > 40) {
    //         // 极慢速低通滤波吸收漂移
    //         gyro_z_offset = gyro_z_offset * 0.998f + raw_gyro_z * 0.002f; 
    //     }
        
    //     // 不管过没过缓冲期，只要是停车状态，积分器死锁清零
    //     last_gyro_z = 0.0f; 
    //     return; 
    // } else {
    //     stop_settle_counter = 0; // 车一动，立刻清零缓冲计数器
    // }

    // ==========================================
    // 正常行驶时的积分逻辑
    // ==========================================
    auto & yaw = App::g_state.physical.pose.yaw;

    // 获取当前原始陀螺仪 Z 轴读数，扣除零偏，并进行死区处理
    float pure_gyro_z = imu_icm42688.data.gyro_z - gyro_z_offset;   
    if (pure_gyro_z > -dynamic_deadband && pure_gyro_z < dynamic_deadband) pure_gyro_z = 0.0f;

    // 进行离散梯形积分并归一化 (保证角度永远在 0~360度之间，防止浮点数溢出)
    yaw += (pure_gyro_z + last_gyro_z) * 0.5f * 0.005f; 
    while (yaw >= 360.0f) yaw -= 360.0f;
    while (yaw < 0.0f)    yaw += 360.0f;

    // 更新历史值
    last_gyro_z = pure_gyro_z;
}


// 20ms 中断调用，进行里程计更新
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

    float dx_local = dx_local_raw / tune.dynamics.kinematic_gain_x;
    float dy_local = dy_local_raw / tune.dynamics.kinematic_gain_y;

    // 将局部坐标系的位移转换到全局坐标系
    float cos_yaw = cosf(current_yaw_rad);
    float sin_yaw = sinf(current_yaw_rad);

    float dx_global = dx_local * sin_yaw + dy_local * cos_yaw;
    float dy_global = -dx_local * cos_yaw + dy_local * sin_yaw;

    App::g_state.physical.pose.x += dx_global;
    App::g_state.physical.pose.y += dy_global;
}

} // namespace Subsystem::PoseEstimator