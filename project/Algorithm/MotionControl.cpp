#include "MotionControl.h"
#include "RobotState.h"


namespace Algorithm::Motion {
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

    if (distance < 0.001f) {
        current_v = 0.0f;
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

        if (target_end_speed > 0.1f && target_v < target_end_speed) {
            target_v = target_end_speed;
        }
    }

    // 不再额外做近终点慢速爬行：梯形刹车规划本身已把 target_v 平滑收敛到 0，
    // 直接按正常速度减速进点停车（靠近箱子/拐点不再缓慢蹭进）。
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
/// 平移时不额外叠加静摩擦补偿，避免平移和旋转耦合后产生抖动
/// 静止转向时会给很小目标角速度补偿 k_stiction，帮助克服启动摩擦
/// 死区带迟滞：进入死区判定"已稳"后，要等误差涨过更大的再触发阈值才重新给力，
/// 否则陀螺噪声在死区边界反复跨越会触发 k_stiction 猛踢 → 原地嗡嗡抖（静摩擦极限环）
///
__attribute__((section(".ramfunc")))
float YawProfiled::calculate(float err, float dt, bool is_translating) {

    float abs_err = std::abs(err);
    float ang_tol = tune.tracker.ang_tolerance;
    // 再触发阈值 = 死区 × 迟滞倍数：已稳后误差必须涨过它才重新介入，把噪声关在死区里
    constexpr float HOLD_HYSTERESIS = 4.0f;
    float reengage = ang_tol * HOLD_HYSTERESIS;

    if (settled) {
        // 已判定停稳：误差还在再触发阈值内就保持零动力，噪声不再喂出抖动
        if (abs_err <= reengage) {
            current_vw = 0.0f;
            return 0.0f;
        }
        settled = false;  // 误差确实变大（真有新目标/明显漂移），重新介入
    } else if (abs_err <= ang_tol) {
        // 收敛进死区：锁存"已稳"，彻底切断动力
        settled = true;
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


/// \brief 沿路径线跟踪 + Stanley 横向纠偏，输出全局期望速度
///
/// \details
/// 沿轨方向：用剩余投影距离喂梯形规划器得到主速度。
/// 横向：用经典 Stanley 的 atan 横向项把车压回路径线，低速大增益、高速温和。
/// 段长过短时 perp 方向不可靠，退化为纯朝目标点。
///
__attribute__((section(".ramfunc")))
Speed2D PathLineFollower::follow(float px, float py, float sx, float sy,
                                 float tx, float ty, float dt, float end_speed) {
    float seg_dx = tx - sx;
    float seg_dy = ty - sy;
    float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

    // 段长过短或 Stanley 关闭：退化为纯朝目标点（沿误差方向收敛）
    if (!tune.stanley.enable || seg_len_sq < 9.0f) {  // <3cm
        return planner.velocity_planning_1d(tx - px, ty - py, dt, end_speed);
    }

    float inv_len = 1.0f / sqrtf(seg_len_sq);
    float along_x = seg_dx * inv_len;
    float along_y = seg_dy * inv_len;
    float perp_x = -along_y;   // 沿轨方向逆时针转 90° = 路径线法向
    float perp_y = along_x;

    // 沿轨剩余距离（投影到段方向），用它喂规划器得到主速度大小
    float s_remain = (tx - px) * along_x + (ty - py) * along_y;
    Speed2D along_vel = planner.velocity_planning_1d(
        along_x * s_remain, along_y * s_remain, dt, end_speed);
    float v_along_mag = along_vel.vx * along_x + along_vel.vy * along_y;  // 带符号主速度

    // 横向偏差（带符号）：车在路径线哪一侧、离线多远
    float e_ct = (px - sx) * perp_x + (py - sy) * perp_y;

    // Stanley 横向项：atan(k*e/(|v|+soft))，归一化到 [-v_lat_max, v_lat_max]
    float denom = std::abs(v_along_mag) + tune.stanley.k_soft;
    float stanley = atanf(tune.stanley.k_ct * e_ct / denom) * (2.0f / PI);  // [-1,1]
    float v_perp = -stanley * tune.stanley.v_lat_max;                       // 压回线上

    return {along_vel.vx + perp_x * v_perp,
            along_vel.vy + perp_y * v_perp};
}

} // namespace Algorithm::Motion
