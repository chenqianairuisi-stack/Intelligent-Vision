#include "trajectory.h"
#include <cmath>
#include <algorithm>

Trajectory::Trajectory() : current_vx(0.0f), current_vy(0.0f), current_ax(0.0f), current_ay(0.0f)  {}

// 强制刹车或重置状态
void Trajectory::reset() {
    current_vx = 0.0f; current_vy = 0.0f; current_ax = 0.0f; current_ay = 0.0f;
}


// 二维速度规划算法：输入当前位置与目标位置的 dx, dy，输出平滑的期望速度 (vx, vy) 和标量速度 v_mag
__attribute__((section(".ramfunc"))) 
Speed2D Trajectory::velocity_planning_2d(float dx, float dy, float max_v, float max_acc,float max_jerk, float dt) {
    
    // 计算当前点到目标点的距离和当前速度的标量大小
    float distance = std::sqrtf(dx * dx + dy * dy);  
    float current_v_mag = std::sqrtf(current_vx * current_vx + current_vy * current_vy);  

    // Jerk 动态滞后补偿，计算刹车力建立期间，车体将会“多溜出去”的距离
    float t_jerk = max_acc / max_jerk;                      // 刹车加速度建立所需时间 (s)
    float jerk_lag_dist = 0.5f * current_v_mag * t_jerk;    // 额外滑行的物理距离 (cm)

    // 欺骗规划器：把溜车距离从总距离中扣除，强迫它随速度动态“提前刹车”
    float safe_distance = distance - jerk_lag_dist;
    if (safe_distance < 0.0f) safe_distance = 0.0f;         // 防下溢出

    // 标量刹车防撞规划，计算理论安全刹车速度 (公式: V^2 = 2 * a * s  =>  V = sqrt(2*a*s))
    float target_v;
    if (safe_distance > 2.0f) {
        target_v = std::sqrtf(2.0f * max_acc * safe_distance);
    } else {
        float kp = std::sqrtf(2.0f * max_acc * 2.0f) / 2.0f;
        target_v = kp * safe_distance;
    }

    // 目标速度取“最大速度”和“安全刹车速度”的较小值
    target_v = std::min(max_v, target_v);  

    // 防微小浮点数漂移彻底停稳
    if (target_v < 0.1f && distance < 0.5f) {
        target_v = 0.0f; 
    }

    // 理想二维全局速度分解：沿着目标点的直线方向按比例分解
    float ideal_vx = 0.0f;
    float ideal_vy = 0.0f;
    if (distance > 0.1f) {
        ideal_vx = target_v * (dx / distance);
        ideal_vy = target_v * (dy / distance);
    }

    // 计算当前需要的期望加速度矢量
    float req_ax = (ideal_vx - current_vx) / dt;
    float req_ay = (ideal_vy - current_vy) / dt;


    // 【第一级滤波】：Jerk (加加速度) 矢量限幅，限制每拍加速度变化的剧烈程度，防止电机扭矩阶跃
    float djx = req_ax - current_ax;
    float djy = req_ay - current_ay;
    float dj_mag = std::sqrtf(djx * djx + djy * djy);
    float max_dj = max_jerk * dt; // 本周期内允许的最大加速度变化量

    if (dj_mag > max_dj) {
        float scale = max_dj / dj_mag;
        current_ax += djx * scale;
        current_ay += djy * scale;
    } else {
        current_ax = req_ax;
        current_ay = req_ay;
    }


    // 【第二级滤波】：Accel (加速度) 矢量限幅，确保合成摩擦力总和不超过轮胎物理抓地力极限
    float a_mag = std::sqrtf(current_ax * current_ax + current_ay * current_ay);
    
    if (a_mag > max_acc) {
        float scale = max_acc / a_mag;
        current_ax *= scale;
        current_ay *= scale;
    }

    // 更新物理速度
    current_vx += current_ax * dt;
    current_vy += current_ay * dt;

    // 返回平滑后的结果
    return {current_vy, current_vx, std::sqrtf(current_vx * current_vx + current_vy * current_vy)};
}