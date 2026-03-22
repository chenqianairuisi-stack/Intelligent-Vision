#include "trajectory.h"
#include <cmath>
#include <algorithm>

Trajectory::Trajectory() : current_v_mag(0.0f) {}

// 强制刹车或重置状态
void Trajectory::reset() {
    current_v_mag = 0.0f;
}


// 根据当前距离，算出下一刻的最优速度大小
__attribute__((section(".ramfunc"))) 
float Trajectory::velocity_planning(float distance, float max_v, float max_acc, float dt) {
    
    // 计算理论安全刹车速度 (公式: V^2 = 2 * a * s  =>  V = sqrt(2*a*s))
    float dec_v;
    if (distance > 2.0f) {
        dec_v = std::sqrt(2.0f * max_acc * distance);
    } else {
        // 防震荡修正：当距离小于 2cm 时，sqrt 曲线会非常陡峭，此时无缝切入线性 P 控制，保证平滑停稳。
        float kp = std::sqrt(2.0f * max_acc * 2.0f) / 2.0f;
        dec_v = kp * distance;
    }

    // 目标速度取“最大速度”和“安全刹车速度”的较小值
    float target_v = std::min(max_v, dec_v);

    // 执行加速度/减速度斜坡限幅
    float dv = max_acc * dt;    //  20ms 内最大速度变化量
    
    if (current_v_mag < target_v) {
        // 加速段
        current_v_mag += dv;
        if (current_v_mag > target_v) current_v_mag = target_v;
    } 
    else if (current_v_mag > target_v) {
        // 减速段
        current_v_mag -= dv;
        if (current_v_mag < target_v) current_v_mag = target_v;
    }

    // 防微小浮点数漂移
    if (current_v_mag < 0.1f && distance < 0.5f) {
        current_v_mag = 0.0f; 
    }

    return current_v_mag;
}