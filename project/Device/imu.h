#pragma once

class Imu {
public:
    
    bool init();  // 初始化IMU并校准陀螺仪零偏（车模必须静止不动）
    
    void update_yaw_5ms_tick();  // 读取数据并积分(由 5ms 定时器中断调用)

    float get_yaw() const { return yaw_angle; }                      // 获取当前车身偏航角 [单位：度]    
    float get_gyro_z() const { return gyro_z_dps; }                  // 获取当前Z轴角速度 [单位：度/秒]

private:

    float yaw_angle = 0.0f;        // 单位：度
    float gyro_z_dps = 0.0f;       // 单位：度/秒
    float gyro_z_offset = 0.0f;    // Z轴零偏 [单位：度/秒]
};


// 声明全局单例，供外部使用
extern Imu imu_sensor;