#include "MotionControl.h"
#include "RobotState.h"
#include "CoreScheduler.h"


namespace Algorithm::Motion {
namespace {
    [[gnu::always_inline]] inline bool has_recent_vision_pose() {
        const auto& vision = App::g_state.vision;
        constexpr uint32_t POSE_RECENT_MS = 300U;
        return vision.art1_map_ready &&
               vision.art1_pose_seq != 0U &&
               vision.art1_pose_stable_count >= App::ART1_POSE_STABLE_REQUIRED_FRAMES &&
               (Core::Scheduler::get_sys_tick_ms() - vision.art1_pose_tick_ms) <= POSE_RECENT_MS;
    }
}

/// \brief 根据当前位置误差生成平滑的全局平移速度
/// \param dx 目标点相对当前位置的 X 向误差 cm
/// \param dy 目标点相对当前位置的 Y 向误差 cm
/// \param dt 控制周期 s
/// \param end_speed 到达当前段末端时希望保留的速度 cm/s
///
/// \details
/// end_speed 为 0 时按终点停车规划，非 0 时用于路径拐角不停顿通过
/// 函数内部保存 current_v，因此切换任务或急停后应先调用 reset
///
__attribute__((section(".ramfunc"))) 
Speed2D Trajectory::velocity_planning_1d(float dx, float dy, float dt, float end_speed) {
    float distance = sqrtf(dx * dx + dy * dy);

    float max_speed = tune.dynamics.max_vel;
    float max_acc = tune.dynamics.max_acc;
    float brake_acc = max_acc * tune.dynamics.brake_limit;
    float target_end_speed = std::clamp(end_speed, 0.0f, max_speed);
    float terminal_reach_radius =
        std::min(tune.tracker.reach_radius_min, TuningDefaults::TERMINAL_REACH_RADIUS_CAP_CM);

    // 只有终点才允许直接清零，普通拐角保留 end_speed 继续滑过
    if (distance <= terminal_reach_radius && target_end_speed <= 0.1f) {
        current_v = 0.0f;
        return {0.0f, 0.0f};
    }

    if (distance < 0.001f) {
        return {0.0f, 0.0f};
    }

    // 刹车距离按末速度反推，end_speed 越高，越晚开始减速
    float brake_dist = (max_speed * max_speed - target_end_speed * target_end_speed) / (2.0f * brake_acc);
    if (brake_dist < 0.0f) {
        brake_dist = 0.0f;
    }

    float target_v = max_speed;
    if (distance <= brake_dist) {
        // 反推当前距离下允许的速度，保证到段尾时仍可保留目标末速度
        target_v = sqrtf(target_end_speed * target_end_speed + 2.0f * brake_acc * distance);

        constexpr float MIN_CREEP_SPEED = 15.0f;
        float min_speed = target_end_speed > 0.1f ? target_end_speed : MIN_CREEP_SPEED;
        if (target_v < min_speed) {
            target_v = min_speed;
        }
    }

    // 最后一层速度变化限幅，避免目标速度突变直接打到轮速环
    float target_blend = (target_end_speed <= 0.1f && has_recent_vision_pose()) ? 1.0f : 0.0f;
    constexpr float FINAL_CAP_BLEND_RATE = 4.0f;
    float blend_step = FINAL_CAP_BLEND_RATE * dt;
    if (final_cap_blend < target_blend) {
        final_cap_blend = std::min(final_cap_blend + blend_step, target_blend);
    } else if (final_cap_blend > target_blend) {
        final_cap_blend = std::max(final_cap_blend - blend_step, target_blend);
    }

    if (target_end_speed <= 0.1f && final_cap_blend > 0.0f) {
        constexpr float FINAL_APPROACH_ZONE_CM = 45.0f;
        constexpr float FINAL_APPROACH_MIN_SPEED = 15.0f;
        constexpr float FINAL_APPROACH_SPEED_SLOPE = 0.65f;
        if (distance < FINAL_APPROACH_ZONE_CM) {
            float final_cap = FINAL_APPROACH_MIN_SPEED + distance * FINAL_APPROACH_SPEED_SLOPE;
            if (target_v > final_cap) {
                target_v = target_v * (1.0f - final_cap_blend) + final_cap * final_cap_blend;
            }
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


/// \brief 规划平滑的 yaw 角速度
/// \param err yaw 误差，单位 rad
/// \param dt 控制周期 s
/// \param is_translating 当前是否正在明显平移
/// \return 期望角速度 rad/s
///
/// \details
/// 角速度由剩余角度和最大角加速度反推，保证接近目标时提前进入可刹停速度
/// 平移时不额外叠加静摩擦补偿，避免平移和旋转耦合后产生抖动
/// 静止转向时只在刹车距离足够时补偿 k_stiction，避免小角度来回越界
///
__attribute__((section(".ramfunc")))
float YawProfiled::calculate(float err, float dt, bool is_translating) {

    // 容差死区判断：到达目标后彻底切断动力，防止持续微调引起的抖动
    float abs_err = std::abs(err);
    if (abs_err <= tune.tracker.ang_tolerance) {
        current_vw = 0.0f;
        return 0.0f; 
    }

    float max_ang_vel = std::max(tune.dynamics.max_ang_vel, 0.0f);
    float max_ang_acc = std::max(tune.dynamics.max_ang_acc, 0.001f);

    // 按剩余角度反推当前允许角速度，速度曲线不再受 yaw Kp 影响
    float target_abs_vw = sqrtf(2.0f * max_ang_acc * abs_err);
    target_abs_vw = std::min(target_abs_vw, max_ang_vel);

    // 静摩擦补偿：当目标速度过小时，提供一个最小的补偿速度，帮助克服静摩擦阈值
    if (!is_translating) {
        float stiction_vw = std::min(tune.ff.k_stiction, max_ang_vel);
        float stiction_brake_err = (stiction_vw * stiction_vw) / (2.0f * max_ang_acc);
        if (target_abs_vw < stiction_vw && abs_err > stiction_brake_err) {
            target_abs_vw = stiction_vw;
        }
    }

    float target_vw = std::copysign(target_abs_vw, err);

    // 加速度物理限幅：防止起步打滑和急刹打滑
    float max_dv = max_ang_acc * dt;
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
