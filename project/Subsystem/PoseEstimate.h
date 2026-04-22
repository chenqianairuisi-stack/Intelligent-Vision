#pragma once
#include <cstdint>

namespace Subsystem::PoseEstimator {
    // 初始化与标定
    void init();
    void calibrate_gyro_step(); // 返回 true 表示标定完成

    // 强行重置里程计坐标
    void set_position(float x, float y, float yaw_deg);

    // 两个高频中断钩子
    void update_yaw_1ms_tick();
    void update_position_20ms_tick(const int16_t* encoder_counts, float current_yaw_deg);
}