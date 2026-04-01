#pragma once
#include <cstdint>
#include "zf_device_gnss.h"
#include "system_config.h"

class Odometry {
public:
    static constexpr float PLUSE_TO_CM = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / SystemConfig::PULSES_PER_REV;

    Odometry();

    // 获取/设置当前全局坐标
    Point2D get_position() const { return {global_x, global_y}; }
    void set_position(float x, float y) { global_x = x; global_y = y; }

    // 更新全局坐标
    void update_position_20ms_tick(const int16_t* encoder_counts, float imu_yaw);

private:
    float global_x;      // 机器人在全局坐标系中的 x 位置，单位 cm
    float global_y;      // 机器人在全局坐标系中的 y 位置，单位 cm
};


extern Odometry chassis_odometry;