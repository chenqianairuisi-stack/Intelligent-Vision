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

    // 弹簧静止点（治空载/保持点"原地颤抖"根因）：停车工况进"到达半径"内直接停伺服、命令零速，
    // 不再追毫米级残差——sqrt 在 d→0 处斜率无穷，会把残差放大成大速度→原地猛冲抖，max_acc 越大越抖。
    //   · 无状态：车被推出半径立即恢复伺服，绝不锁存、绝不在接近途中提前停（区别于被否掉的到达锁存）。
    //   · 仅"非 AUTO 正在循迹"启用（空载/观测保持/!M 调试点）；AUTO 正循迹(航点/推箱)不启用，
    //     保精度、由 Tracker 的 hard_lock 收尾。等效于 yaw 环的 ang_tolerance，yaw 从不原地抖。
    float rest_radius = tune.tracker.reach_radius;
    bool active_auto_track = (App::g_state.control.mode == ControlMode::AUTO_TRACKING &&
                             App::g_state.control.tracker_state == TrackerState::TRACKING);
    if (target_end_speed <= 0.1f && !active_auto_track && distance <= rest_radius) {
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
        if (target_end_speed <= 0.1f) {
            // 弹簧终端：远端 sqrt(时间最优减速)，近端 TERMINAL_LIN_BAND_CM 内切成线性律 v=k·distance
            // （斜率有界）——越靠近目标阻尼越大、线性柔和收敛到 0（"撞弹簧不撞墙"），且毫米级残差只
            // 映射成小速度、不再被 sqrt 无穷斜率放大成猛冲。带宽先用常量，需要现调可再接菜单。
            constexpr float TERMINAL_LIN_BAND_CM = 3.0f;
            float v_edge = sqrtf(2.0f * brake_acc * TERMINAL_LIN_BAND_CM);
            target_v = (distance >= TERMINAL_LIN_BAND_CM)
                ? sqrtf(2.0f * brake_acc * distance)
                : (v_edge / TERMINAL_LIN_BAND_CM) * distance;
        } else {
            // 过弯带速通过：按末速度反推 sqrt，保留切向动量（不加弹簧终端）
            target_v = sqrtf(target_end_speed * target_end_speed + 2.0f * brake_acc * distance);
            if (target_v < target_end_speed) {
                target_v = target_end_speed;
            }
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
/// \param yaw_rate IMU 实测 yaw 角速度，单位 rad/s，符号与 err 一致（逆时针为正）
/// \return 期望角速度 rad/s
///
/// \details
/// 速度规划分三层：
/// 1) 远离目标时用 sqrt(2·a·err) 时间最优减速曲线，保证大角度快速收敛；
/// 2) 进入 yaw.lin_band 线性带后，改用与 sqrt 在带边相切的线性律，
///    使等效比例增益在 err→0 处不再发散，从根上消除"越接近越硬"导致的极限环；
/// 3) 叠加 -yaw.kd·yaw_rate 陀螺阻尼，直接对被控对象角速度做微分反馈，
///    与误差符号无关地抑制冲过/回摆，是真正的止抖项（无 setpoint kick、无编码器差分噪声）。
/// 静止转向时仍在刹车距离足够处补偿 yaw.stiction，避免小角度越界来回蹭。
///
__attribute__((section(".ramfunc")))
float YawProfiled::calculate(float err, float dt, bool is_translating, float yaw_rate) {

    // 容差死区判断：到达目标后彻底切断动力，防止持续微调引起的抖动
    float abs_err = std::abs(err);
    if (abs_err <= tune.tracker.ang_tolerance) {
        current_vw = 0.0f;
        current_aw = 0.0f;
        return 0.0f;
    }

    float max_ang_vel = std::max(tune.dynamics.max_ang_vel, 0.0f);
    float max_ang_acc = std::max(tune.dynamics.max_ang_acc, 0.001f);

    // 线性带宽下限护栏：过小会退化回 sqrt 的无穷斜率问题
    float lin_band = std::max(tune.yaw.lin_band, 0.005f);

    // 带边速度与斜率：sqrt 曲线在 err=lin_band 处的值和导数
    // v_edge = sqrt(2·a·band)，斜率 k = dv/d(err)|edge = a / v_edge
    // 线性带内用 v = k·err 保证带边 C0 连续，且 err→0 处等效增益恒为 k（有界）
    float v_edge = sqrtf(2.0f * max_ang_acc * lin_band);

    float target_abs_vw;
    if (abs_err >= lin_band) {
        // 远端：时间最优 sqrt 减速曲线
        target_abs_vw = sqrtf(2.0f * max_ang_acc * abs_err);
    } else {
        // 近端：与 sqrt 相切的线性律，等效 Kp = v_edge / lin_band 有界
        target_abs_vw = (v_edge / lin_band) * abs_err;
    }
    if (is_translating) {
        target_abs_vw *= tune.yaw.translate_gain;
    }
    target_abs_vw = std::min(target_abs_vw, max_ang_vel);

    // 静摩擦补偿：提供一个最小的纠正角速度地板，避免小偏差纠正力度不足
    // 平移时轮子动态摩擦低于静止，地板减至 40%，仍然保证基本纠正能力
    {
        float stiction_vw = std::min(tune.yaw.stiction, max_ang_vel);
        if (is_translating) stiction_vw *= 0.4f;
        float stiction_brake_err = (stiction_vw * stiction_vw) / (2.0f * max_ang_acc);
        if (target_abs_vw < stiction_vw && abs_err > stiction_brake_err) {
            target_abs_vw = stiction_vw;
        }
    }

    float target_vw = std::copysign(target_abs_vw, err);

    // 陀螺阻尼：平移时减小阻尼，避免被动带歪阶段削弱主动纠偏
    // 放在加速度限幅之前，使阻尼也受物理步长约束，不会引入单帧跳变
    float yaw_kd = is_translating ? tune.yaw.kd_translate : tune.yaw.kd;
    target_vw -= yaw_kd * yaw_rate;

    // 加速度物理限幅：防止起步打滑和急刹打滑
    float last_vw = current_vw;
    float max_dv = max_ang_acc * dt;
    if (target_vw > current_vw + max_dv) {
        current_vw += max_dv;
    } else if (target_vw < current_vw - max_dv) {
        current_vw -= max_dv;
    } else {
        current_vw = target_vw;
    }

    current_aw = (current_vw - last_vw) / std::max(dt, 0.001f);
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
