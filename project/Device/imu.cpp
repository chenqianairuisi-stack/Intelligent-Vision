#include "zf_common_headfile.h"
#include "imu.h"

__attribute__((section(".dtcm_data"))) Imu imu_sensor;

Imu::Imu() : yaw_angle(90.0f), gyro_z_dps(0.0f), gyro_z_offset(0.0f) {}

bool Imu::init() {
    if (imu660ra_init() != 0) return false;
    system_delay_ms(100);   // 等待 IMU 稳定

    // 采集 100 次静态数据求平均，计算 Z 轴零漂
    int32_t offset_sum = 0;
    for (int i = 0; i < 100; i++) {
        imu660ra_get_gyro();
        offset_sum += imu660ra_gyro_z;
        system_delay_ms(5);
    }
    gyro_z_offset = (float)offset_sum / 100.0f;
    yaw_angle = 90.0f;

    return true;
}

// 获取陀螺仪数据并积分 (5ms PIT 中断调用)
__attribute__((section(".ramfunc"))) void Imu::update_yaw_5ms_tick() {
    imu660ra_get_gyro(); 
    
    // 转换为角速度
    gyro_z_dps = ((float)imu660ra_gyro_z - gyro_z_offset) / imu660ra_transition_factor[1];
    
    // 进行离散积分 (逆时针为正，单位为度)
    yaw_angle += gyro_z_dps * 0.005f; 
}
