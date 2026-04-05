#pragma once
#include "system_config.h"

class Trajectory {
public:
    Trajectory();

    // 核心规划算法：根据当前距离，算出下一刻的最优速度大小
    Speed2D velocity_planning_1d(float dx, float dy, float dt);
    
    // 强制刹车或重置状态
    void reset();

private:
    float current_v; 
    float current_a;
};