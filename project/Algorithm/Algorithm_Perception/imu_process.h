#pragma once
#include "icm42688.h"


class ImuProcessor {
public:
    ImuProcessor();
    bool calibrate_step();          // 开机静态标定，累计 500 次数据求平均，得到 gyro_z_offset
    void update_yaw_5ms_tick();     // 200Hz 中断调用，进行离散积分更新 yaw_angle
    
    // 对外数据接口
    float get_yaw() const { return yaw_angle; }
    float get_gyro_z() const { return last_gyro_z_dps; }

private:
    // IMU 数据
    float yaw_angle;             // 当前航向角 (单位：度，0~360)
    float last_gyro_z_dps;       // 上一时刻的陀螺仪Z轴角速度 (单位：度/秒)，用于梯形积分

    // 标定数据与状态机参数
    float gyro_z_offset;         // 陀螺仪Z轴零偏 (单位：度/秒)
    bool is_calibrated;          // 标定完成标志
    int32_t calib_sample_count;  // 已采集的标定样本数量
    float calib_sum;             // 标定样本的累积和 (单位：度/秒)
};

extern ImuProcessor imu_sensor;