#include "kinematics.h"

// 麦轮逆运动学解算：底盘速度 -> 四轮独立速度
__attribute__((section(".ramfunc"))) 
WheelSpeed4 Kinematics::inverse_kinematics(float vx, float vy, float vw) {
    WheelSpeed4 speeds;
    speeds.lf = vy + vx - vw * (SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS);
    speeds.lb = vy - vx - vw * (SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS);
    speeds.rf = vy - vx + vw * (SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS);
    speeds.rb = vy + vx + vw * (SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS);
    return speeds;
}

// 麦轮正运动学解算：四轮独立速度 -> 底盘速度
__attribute__((section(".ramfunc")))
Velocity2D Kinematics::forward_kinematics(float v_lf, float v_lb, float v_rf, float v_rb) {
    Velocity2D vel;
    vel.vx = ( v_lf - v_lb - v_rf + v_rb) / 4.0f;
    vel.vy = ( v_lf + v_lb + v_rf + v_rb) / 4.0f;
    vel.vw = (-v_lf - v_lb + v_rf + v_rb) / (4.0f * (SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS));
    return vel;
}