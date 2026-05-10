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

    float max_speed   = tune.dynamics.max_speed;
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

} // namespace Algorithm::Motion