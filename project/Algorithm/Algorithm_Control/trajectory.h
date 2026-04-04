#pragma once
#include "system_config.h"

class Trajectory {
public:
    Trajectory();

    // 核心规划算法：根据当前距离，算出下一刻的最优速度大小
    Speed2D velocity_planning_2d(float dx, float dy, float dt);
    
    // 强制刹车或重置状态
    void reset();

private:
    // 缓存上一周期的真实期望速度矢量
    float current_vx; 
    float current_vy;

    // 缓存上一周期的真实加速度矢量 (用于计算 Jerk)
    float current_ax;
    float current_ay;
};