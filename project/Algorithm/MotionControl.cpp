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
Speed2D Trajectory::velocity_planning_1d(float dx, float dy, float dt, float end_speed,
                                        float seg_len_cm) {
    float distance = sqrtf(dx * dx + dy * dy);

    float max_speed = tune.dynamics.max_vel;
    float max_acc = tune.dynamics.max_acc;
    float brake_acc = max_acc * tune.dynamics.brake_limit;
    float target_end_speed = std::clamp(end_speed, 0.0f, max_speed);

    // 短段起步加速提升：段全长 <= short_seg_len_cm 时，**只**把加速斜坡上限抬到 short_seg_accel，
    // 让短距离移动起步更快、不磨蹭。刹车加速度 brake_acc 与下方 sqrt 减速曲线**保持不变**（温柔刹车），
    // 末端因此自然物理慢下来（不是把末端交给视觉控制，只是放慢；慢下来后编码器+视觉融合更准）。
    // seg_len_cm<=0（未知）或段长超阈值：不提升，退回 max_acc，行为与改前完全一致。
    float accel_ramp = max_acc;
    {
        float boost = tune.short_seg_accel;
        float len_thresh = tune.short_seg_len_cm;
        if (std::isfinite(seg_len_cm) && seg_len_cm >= 1.0f &&
            std::isfinite(len_thresh) && seg_len_cm <= len_thresh &&
            std::isfinite(boost) && boost > 0.0f) {
            accel_ramp = boost;   // 只用于加速斜坡；不参与 brake_acc / sqrt 曲线
        }
    }

    if (distance < 0.001f) {
        current_v = 0.0f;
        return {0.0f, 0.0f};
    }

    // 停车工况终端整形：
    //   · 弹簧静止死区（进 rest_radius 直接命令零速，治空载/观测保持/!M 调试点"原地颤抖"）：仅非 AUTO
    //     启用；AUTO 正循迹不启用，避免近端零速死区削掉顶箱力气，停车由 Tracker 的 hard_lock 收尾。
    //   （2026-07-15 删 StopBand 后一度回纯 sqrt 直落；同日改为下方"接近区双段 sqrt 提前刹车"）
    float rest_radius = tune.tracker.reach_radius;
    bool active_auto_track = (App::g_state.control.mode == ControlMode::AUTO_TRACKING &&
                             App::g_state.control.tracker_state == TrackerState::TRACKING);
    bool is_stop = (target_end_speed <= 0.1f);              // 停车工况（含 AUTO 推箱/终点）
    bool spring_terminal = (is_stop && !active_auto_track); // 弹簧静止死区仅非 AUTO 启用（AUTO 靠 hard_lock 收尾）

    // 弹簧静止点（治空载/保持点"原地颤抖"根因）：进"到达半径"内直接停伺服、命令零速，不再追毫米级
    // 残差——sqrt 在 d→0 斜率无穷会把残差放大成大速度→原地猛冲抖，max_acc 越大越抖。
    // 无状态：车被推出半径立即恢复伺服，绝不锁存、绝不在接近途中提前停。等效 yaw 环 ang_tolerance。
    if (spring_terminal && distance <= rest_radius) {
        current_v = 0.0f;
        return {0.0f, 0.0f};
    }

    // 停车接近区双段刹车（治"末端总是出去一些、StopBrkG 大小都调不好"）：
    // 过冲发生在 hard_lock 视觉冻结窗口内、事后又有粘滞锁死区兜着，单一 sqrt 曲线永远调不准。
    // 修法=**提前刹车**：停车段最后 zone cm 换一条更缓的 sqrt 曲线（approach_brake_acc），
    // 外段仍按原 brake_acc 刹车、在 zone 边界与缓曲线 C0 连续衔接——刹车点因此显著提前，
    // 车末端明显慢下来（非固定爬行速度），低速下视觉延迟误差≈速度×310ms 变小、编码器不打滑，
    // 视觉+编码器融合在进点前就收敛到真实位置。zone = min(zone_cm, 段全长×ratio)：
    // 长距离封顶 40cm，20cm 段→5cm（用户拍板）。参数垃圾/关闭(!T Z 0.5)时 zone=0 → 回纯 sqrt 直落。
    float approach_zone = 0.0f;
    float approach_acc = 0.0f;
    if (is_stop && tune.approach_enable >= 0.5f) {   // 总开关关闭(approach_enable<0.5)→zone=0→回纯 sqrt
        float z_cap = tune.approach_zone_cm;
        float z_ratio = tune.approach_zone_ratio;
        float a_apr = tune.approach_brake_acc;
        if (std::isfinite(z_cap) && z_cap >= 1.0f &&
            std::isfinite(a_apr) && a_apr >= 1.0f && a_apr < brake_acc) {
            approach_zone = z_cap;
            if (std::isfinite(z_ratio) && z_ratio > 0.01f &&
                std::isfinite(seg_len_cm) && seg_len_cm >= 1.0f) {
                approach_zone = std::min(approach_zone, seg_len_cm * z_ratio);
            }
            approach_acc = a_apr;
        }
    }

    // 刹车距离按末速度反推，end_speed 越高，越晚开始减速
    float brake_dist = (max_speed * max_speed - target_end_speed * target_end_speed) / (2.0f * brake_acc);
    if (brake_dist < 0.0f) {
        brake_dist = 0.0f;
    }

    float target_v = max_speed;
    if (is_stop && approach_zone > 0.5f) {
        // 停车工况+接近区有效：双段 sqrt 减速。
        //   近端(distance<=zone)：缓曲线 v=sqrt(2·a_apr·d)，末端慢慢收进点；
        //   外段：v=sqrt(v_edge²+2·brake_acc·(d-zone))，正常刹车强度、提前 zone 开始，
        //         在 zone 边界正好落到缓曲线的 v_edge 上（C0 连续，无台阶）。
        // min(max_speed) 天然给出提前后的刹车起点，无需另算 brake_dist。
        float v_edge_sq = 2.0f * approach_acc * approach_zone;
        if (distance <= approach_zone) {
            target_v = sqrtf(2.0f * approach_acc * distance);
        } else {
            target_v = sqrtf(v_edge_sq + 2.0f * brake_acc * (distance - approach_zone));
        }
        target_v = std::min(target_v, max_speed);
    } else if (distance <= brake_dist) {
        if (is_stop) {
            // 停车工况（接近区关闭/参数无效的回退）：纯 sqrt 时间最优减速直落到点。
            target_v = sqrtf(2.0f * brake_acc * distance);
        } else {
            // 过弯带速通过：sqrt 反推保切向动量，末速度不低于目标保留速度
            target_v = sqrtf(target_end_speed * target_end_speed + 2.0f * brake_acc * distance);
            if (target_v < target_end_speed) {
                target_v = target_end_speed;
            }
        }
    }

    // 每拍速度斜坡限幅：
    float max_dv_acc = accel_ramp * dt;   // 加速斜坡：短段被 accel_ramp 抬高（只影响此处）
    float max_dv_dec = brake_acc * dt;    // 刹车斜坡：始终用 brake_acc（接近区缓曲线是目标更低，不需更大限幅）

    // 停车接近区抖减速：仅停车工况放大**每拍减速上限**（不动 target 曲线/加速斜坡/过弯带速）。
    // 带速冲进停车航点的车会被原始 brake_acc·dt 温柔限幅卡住、来不及掉到 sqrt 目标曲线上就冲过点；
    // 乘上 stop_approach_brake_gain 让它一步抖到曲线上→进点速度低→固定视觉延迟造成的过冲随之缩小。
    // 车已在曲线上时 target_v≈current_v，限幅本就不 binding→放大它零行为改变；gain=1.0 完全等于旧行为。
    if (is_stop) {
        float brake_gain = tune.stop_approach_brake_gain;
        if (!(brake_gain >= 1.0f && brake_gain <= 100.0f)) {
            brake_gain = 1.0f;   // 运行期兜底：NaN/越界（比较恒 false 落此）→ 退回旧行为，不依赖 sanitize
        }
        max_dv_dec *= brake_gain;
    }

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
    float seg_len = sqrtf(seg_len_sq);   // 段全长，喂规划器做短段加速提升判定

    // 段长过短或 Stanley 关闭：退化为纯朝目标点（沿误差方向收敛）
    if (!tune.stanley.enable || seg_len_sq < 9.0f) {  // <3cm
        return planner.velocity_planning_1d(tx - px, ty - py, dt, end_speed, seg_len);
    }

    float inv_len = 1.0f / sqrtf(seg_len_sq);
    float along_x = seg_dx * inv_len;
    float along_y = seg_dy * inv_len;
    float perp_x = -along_y;   // 沿轨方向逆时针转 90° = 路径线法向
    float perp_y = along_x;

    // 沿轨剩余距离（投影到段方向），用它喂规划器得到主速度大小
    float s_remain = (tx - px) * along_x + (ty - py) * along_y;
    Speed2D along_vel = planner.velocity_planning_1d(
        along_x * s_remain, along_y * s_remain, dt, end_speed, seg_len);
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
