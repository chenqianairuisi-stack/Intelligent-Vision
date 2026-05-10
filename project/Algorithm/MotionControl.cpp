#include "MotionControl.h"


namespace Algorithm::Motion {

// 一维速度规划
__attribute__((section(".ramfunc"))) 
Speed2D Trajectory::velocity_planning_1d(float dx, float dy, float dt) {
    
    float distance = std::sqrtf(dx * dx + dy * dy);  

    if (distance < tune.tracker.reach_radius_min) {
        current_v = 0.0f; // 动量清零
        return {0.0f, 0.0f}; 
    }

    float max_speed   = tune.dynamics.max_vel;
    float max_acc     = tune.dynamics.max_acc;
    float brake_acc   = max_acc * tune.dynamics.brake_limit;
    
    // 理论所需刹车距离: v^2 / (2 * a)
    float brake_dist = (max_speed * max_speed) / (2.0f * brake_acc);

    float target_v = max_speed; // 默认巡航
    
    if (distance <= brake_dist) {
        // 刹车段：由于 v = sqrt(2 * a * S)，直接利用乘法
        target_v = std::sqrtf(2.0f * brake_acc * distance);
        
        // 克服静摩擦的最小蠕行速度
        constexpr float MIN_CREEP_SPEED = 10.0f; 
        if (target_v < MIN_CREEP_SPEED) {
            target_v = MIN_CREEP_SPEED;
        }
    }

    float max_dv_acc = max_acc * dt;
    float max_dv_dec = brake_acc * dt;

    if (target_v > current_v) {
        current_v = std::min(current_v + max_dv_acc, target_v);
    } else {
        current_v = std::max(current_v - max_dv_dec, target_v);
    }

    float inv_dist = 1.0f / distance;
    float ux = dx * inv_dist;
    float uy = dy * inv_dist;

    return {current_v * ux, current_v * uy};
}


// Yaw 角速度规划
__attribute__((section(".ramfunc")))
float YawProfiled::calculate(float err, float dt, bool is_translating) {

    // 容差死区判断：到达目标后彻底切断动力，防止持续微调引起的抖动
    if (std::abs(err) <= tune.tracker.ang_tolerance) {
        current_vw = 0.0f;
        return 0.0f; 
    }

    // 纯 P 控制：距离越近，要求速度越小
    float target_vw = tune.pid_yaw.kp * err;

    // 静摩擦补偿：当目标速度过小时，提供一个最小的补偿速度，帮助克服静摩擦阈值
    if (!is_translating) {
        if (std::abs(target_vw) < tune.ff.k_stiction) {
            target_vw = std::copysign(tune.ff.k_stiction, err);
        }
    }

    // 速度物理限幅
    target_vw = std::clamp(target_vw, -tune.dynamics.max_ang_vel, tune.dynamics.max_ang_vel);

    // 加速度物理限幅：防止起步打滑和急刹打滑
    float max_dv = tune.dynamics.max_ang_acc * dt;
    if (target_vw > current_vw + max_dv) {
        current_vw += max_dv;       
    } else if (target_vw < current_vw - max_dv) {
        current_vw -= max_dv;      
    } else {
        current_vw = target_vw; 
    }

    return current_vw;
}

} // namespace Algorithm::Motion