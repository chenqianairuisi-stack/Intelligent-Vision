#pragma once

class Imu {
public:
    Imu();        
    bool init();  // 初始化IMU并校准陀螺仪零偏（车模必须静止不动）
    
    void update_yaw_5ms_tick();  // 读取数据并积分(由 5ms 定时器中断调用)

    float get_yaw() const { return yaw_angle; }                      // 获取当前车身偏航角 [单位：deg]    
    float get_gyro_z() const { return gyro_z_dps; }                  // 获取当前Z轴角速度 [单位：deg/s]

private:

    float yaw_angle;         // 单位：deg (初始值为90度，表示初始车头朝向全局坐标系Y轴正方向)
    float gyro_z_dps;        // 单位：deg/s
    float gyro_z_offset;     // Z轴零偏 [单位：deg/s]
};


extern Imu imu_sensor;