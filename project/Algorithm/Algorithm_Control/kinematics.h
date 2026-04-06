#pragma once
#include "system_config.h"

class Kinematics {
public:
    // 麦轮逆运动学解算：底盘速度 -> 四轮独立速度
    static WheelSpeed4 inverse_kinematics(float vx, float vy, float vw);

    // 麦轮正运动学解算：四轮独立速度 -> 底盘速度
    static Velocity2D forward_kinematics(float v_lf, float v_lb, float v_rf, float v_rb);
};