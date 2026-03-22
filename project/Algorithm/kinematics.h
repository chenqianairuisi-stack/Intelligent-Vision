#pragma once

// 定义四个轮子的转速结构体
struct WheelSpeed4 {
    float lf;      // Left Front
    float lb;      // Left Back
    float rf;      // Right Front
    float rb;      // Right Back
};

class Kinematics {
public:
    // 麦轮逆运动学解算：底盘速度 -> 四轮独立速度
    // vx: 左右平移 (右为正)
    // vy: 前后直行 (前为正)
    // vw: 旋转速度 (顺时针为正)
    static WheelSpeed4 inverse_kinematics(float vx, float vy, float vw);
};