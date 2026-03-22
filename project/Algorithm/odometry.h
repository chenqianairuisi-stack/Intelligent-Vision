#pragma once
#include <cstdint>
#include "zf_device_gnss.h"
#include "system_config.h"

class Odometry {
public:
    static constexpr float PLUSE_TO_CM = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / SystemConfig::PULSES_PER_REV;
    Odometry();

    void update_global_position(const int16_t* encoder_counts, float imu_yaw);

    Point2D get_position() const { return {global_x, global_y}; }


private:
    float global_x = 0.0f;      // 机器人在全局坐标系中的 x 位置，单位 cm
    float global_y = 0.0f;      // 机器人在全局坐标系中的 y 位置，单位 cm
};

extern Odometry chassis_odometry;
