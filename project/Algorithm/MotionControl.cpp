#include "MotionControl.h"


namespace Algorithm::Motion {

// 一维速度规划算法：严格算出标量梯形速度，最后投影分解
__attribute__((section(".ramfunc"))) 
Speed2D Trajectory::velocity_planning_1d(float dx, float dy, float dt) {
    
    // 1. 计算当前距离，死区判断
    float distance = std::sqrtf(dx * dx + dy * dy);  
    if (distance < tune.tracker.reach_radius_min) {
        reset();
        return {0.0f, 0.0f}; 
    }

    // 2. 读取 DTCM 中的全局极限参数
    float max_acc = tune.dynamics.max_acc;
    float max_jerk = tune.dynamics.max_jerk;
    float max_v_limit = tune.dynamics.max_speed;

    // 3. Jerk 动态滞后补偿 (直接使用标量速度，计算极大简化)
    float t_jerk = max_acc / max_jerk;                      // 刹车加速度建立所需时间 (s)
    float jerk_lag_dist = 0.5f * current_v * t_jerk;        // 额外滑行的物理距离 (cm)

    // 4. 计算理论安全刹车速度 (梯形速度规划)
    float safe_distance = distance - jerk_lag_dist;
    if (safe_distance < 0.0f) safe_distance = 0.0f;         

    float target_v;
    if (safe_distance > 2.0f) {
        target_v = std::sqrtf(2.0f * max_acc * safe_distance);
    } else {
        float kp = std::sqrtf(max_acc);
        target_v = kp * safe_distance;
    }

    target_v = std::min(max_v_limit, target_v);  
    
    // 计算当前需要的期望标量加速度
    float req_a = (target_v - current_v) / dt;

    // 【第一级滤波】：Jerk 标量限幅
    float da = req_a - current_a;
    float max_da = max_jerk * dt; 

    if (da > max_da) {
        current_a += max_da;
    } else if (da < -max_da) {
        current_a -= max_da;
    } else {
        current_a = req_a;
    }

    // 【第二级滤波】：Accel 标量限幅
    if (current_a > max_acc) {
        current_a = max_acc;
    } else if (current_a < -max_acc) {
        current_a = -max_acc;
    }

    // 更新真实的物理标量速度
    current_v += current_a * dt;
    if (current_v < 0.0f) current_v = 0.0f; // 防下溢倒车

    // 最终投影：顺着目标点的方向线，将绝对纯净的标量速度分解
    float ux = dx / distance;
    float uy = dy / distance;

    return {current_v * ux, current_v * uy};
}

} // namespace Algorithm::Motion