#include "imu_process.h"

__attribute__((section(".dtcm_data"))) ImuProcessor imu_sensor;

ImuProcessor::ImuProcessor() : yaw_angle(90.0f), gyro_z_offset(0.0f), 
                last_gyro_z_dps(0.0f), is_calibrated(false), 
                calib_sample_count(0), calib_sum(0.0f) {}


// 开机静态标定任务，传入底层驱动读到的原始陀螺仪 Z 轴数据（单位：度/秒）。返回 true 表示标定完成。
bool ImuProcessor::calibrate_step() {
    if (is_calibrated) return true;

    // 这里我们把你的 100 次改成了 500 次 (因为 200Hz 下 500次才 2.5 秒)
    if (calib_sample_count < 500) {
        calib_sum += imu_icm42688.data.gyro_z;
        calib_sample_count++;
        return false;
    } else {
        gyro_z_offset = calib_sum / 500.0f;
        is_calibrated = true;
        return true;
    }
}


// 获取陀螺仪数据并积分 (5ms PIT 中断调用)
__attribute__((section(".ramfunc"))) 
void ImuProcessor::update_yaw_5ms_tick() {
    if (!is_calibrated) return; // 没校准完绝对不积分

    // 扣除零偏
    float pure_gyro_z = imu_icm42688.data.gyro_z - gyro_z_offset;
        
    // 极小死区
    if (pure_gyro_z > -0.05f && pure_gyro_z < 0.05f) pure_gyro_z = 0.0f;

    // 离散梯形积分
    yaw_angle += (pure_gyro_z + last_gyro_z_dps) * 0.5f * 0.005f; 

    // 角度归一化 (保证角度永远在 0~360度之间，防止浮点数溢出)
    while (yaw_angle >= 360.0f) yaw_angle -= 360.0f;
    while (yaw_angle < 0.0f)    yaw_angle += 360.0f;

    // 更新历史值
    last_gyro_z_dps = pure_gyro_z;
}