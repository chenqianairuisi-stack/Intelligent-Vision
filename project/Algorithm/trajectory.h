#pragma once

class Trajectory {
private:
    float current_v_mag; // 当前维持的标量速度

public:
    Trajectory();

    // 核心规划算法：根据当前距离，算出下一刻的最优速度大小
    float velocity_planning(float distance, float max_v, float max_acc, float dt);
    
    // 强制刹车或重置状态
    void reset();
};