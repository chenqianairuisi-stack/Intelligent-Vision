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

#include "Icm42688.h"
#include "Encoder.h"
#include <cmath>

namespace Subsystem::PoseEstimator {

namespace { 
    float last_gyro_z = 0.0f;        // 上一次的陀螺仪Z轴读数
    float gyro_z_offset = 0.0f;      // 陀螺仪Z轴零偏 (单位：度/秒)
    bool is_calibrated = false;      // 标定完成标志位

    constexpr float PLUSE_TO_CM = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / SystemConfig::PULSES_PER_REV;  // 编码器计数转换为轮子移动的距离（cm）
}

// 初始化与标定
void init() {
    gyro_z_offset = 0.0f;
    is_calibrated = false;
    imu_icm42688.init();  // ICM42688 IMU 初始化 (spi)
}

// IMU 开机静态标定，累计 500 次数据求平均，得到 gyro_z_offset
void calibrate_gyro_step() {
    if (is_calibrated) return; 
    float calib_sum = 0.0f;          // 标定样本的累积和 (单位：度/秒)

    for (int i = 0; i < 500; ++i) {
        imu_icm42688.update_gyro_only();       // 触发底层的陀螺仪数据更新
        calib_sum += imu_icm42688.data.gyro_z; // 累加 Z 轴角速度
        system_delay_ms(5);                    // 每次采样间隔 5ms
    }

    // 求平均值得到零偏 (零点漂移)
    gyro_z_offset = calib_sum / 500.0f;
    is_calibrated = true; // 标定完成
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

    auto & yaw = App::g_state.physical.pose.yaw;

    // 获取当前原始陀螺仪 Z 轴读数，扣除零偏，并进行死区处理
    float pure_gyro_z = imu_icm42688.data.gyro_z - gyro_z_offset;   
    if (pure_gyro_z > -0.05f && pure_gyro_z < 0.05f) pure_gyro_z = 0.0f;

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
    float dx_local = (d_lf - d_lb - d_rf + d_rb) / 4.0f;
    float dy_local = (d_lf + d_lb + d_rf + d_rb) / 4.0f;

    // 将局部坐标系的位移转换到全局坐标系
    float cos_yaw = cosf(current_yaw_rad);
    float sin_yaw = sinf(current_yaw_rad);

    float dx_global = dx_local * sin_yaw + dy_local * cos_yaw;
    float dy_global = -dx_local * cos_yaw + dy_local * sin_yaw;

    App::g_state.physical.pose.x += dx_global;
    App::g_state.physical.pose.y += dy_global;
}

} // namespace Subsystem::PoseEstimator