#include "MotionControl.h"
#include "RobotState.h"


namespace Algorithm::Motion {

namespace {
// 末端速度曲线：先把剩余距离归一化为 k，再用 k^1.5 加强减速区中段压速
//   k        = clamp((d - d_stop) / (d_slow - d_stop), 0, 1)
//   k_shape  = k · sqrt(k) = k^1.5
//   v_target = v_min + (v_cruise - v_min) · k_shape
// k=1 和 k=0 两端保持不变，只降低 0<k<1 时的目标速度
constexpr float terminal_distance_progress(float remaining_dist,
                                           float slowdown_dist,
                                           float stop_dist) {
    float span = slowdown_dist - stop_dist;
    if (!(span > 0.0f)) return 0.0f;

    return std::clamp((remaining_dist - stop_dist) / span, 0.0f, 1.0f);
}

inline float shaped_terminal_target_speed(float remaining_dist,
                                          float slowdown_dist,
                                          float stop_dist,
                                          float cruise_speed,
                                          float min_speed) {
    float k = terminal_distance_progress(remaining_dist, slowdown_dist, stop_dist);
    float shaped_k = k * sqrtf(k);
    return min_speed + (cruise_speed - min_speed) * shaped_k;
}

static_assert(terminal_distance_progress(35.0f, 35.0f, 2.0f) == 1.0f,
              "terminal slowdown start mismatch");
static_assert(terminal_distance_progress(18.5f, 35.0f, 2.0f) == 0.5f,
              "terminal slowdown midpoint mismatch");
static_assert(terminal_distance_progress(2.0f, 35.0f, 2.0f) == 0.0f,
              "terminal slowdown stop mismatch");

constexpr float PATH_DIRECTION_FILTER_ALPHA = 0.25f;
constexpr float PATH_LOOKAHEAD_MIN_CM = 6.0f;
constexpr float PATH_LOOKAHEAD_MAX_CM = 16.0f;
constexpr float PATH_LOOKAHEAD_TIME_S = 0.25f;
constexpr float PATH_LATERAL_LOOKAHEAD_K = 0.8f;
constexpr float PATH_LATERAL_SLOW_START_CM = 3.0f;
constexpr float PATH_LATERAL_SLOW_FULL_CM = 12.0f;
constexpr float PATH_LATERAL_MIN_SPEED_SCALE = 0.55f;
constexpr float PATH_TRACK_DEADBAND_CM = 4.0f;
constexpr float PATH_TRACK_GAIN_S = 3.5f;
constexpr float PATH_TRACK_MAX_RATIO = 0.45f;
constexpr float PATH_CORNER_APPROACH_CM = 28.0f;

constexpr float LONG_PATH_LOOKAHEAD_MAX_CM = 22.0f;
constexpr float LONG_PATH_LATERAL_LOOKAHEAD_K = 0.35f;
constexpr float LONG_PATH_LATERAL_SLOW_START_CM = 5.0f;
constexpr float LONG_PATH_LATERAL_SLOW_FULL_CM = 16.0f;
constexpr float LONG_PATH_LATERAL_MIN_SPEED_SCALE = 0.65f;
constexpr float LONG_PATH_TRACK_GAIN_S = 1.8f;
constexpr float LONG_PATH_TRACK_MAX_RATIO = 0.22f;
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
Speed2D Trajectory::velocity_planning_1d(float dx, float dy, float dt, float end_speed,
                                        float seg_len_cm) {
    float distance = sqrtf(dx * dx + dy * dy);

    float max_speed = tune.dynamics.max_vel;
    float max_acc = tune.dynamics.max_acc;
    float brake_acc = max_acc * tune.dynamics.brake_limit;
    float target_end_speed = std::clamp(end_speed, 0.0f, max_speed);
    bool long_segment = std::isfinite(seg_len_cm) &&
                        seg_len_cm >= LinearTerminalConfig::LONG_SEGMENT_THRESHOLD_CM;

    // 短段起步加速提升：非长段且段全长 <= short_seg_len_cm 时，**只**把加速斜坡上限抬到 short_seg_accel，
    // 让短距离移动起步更快、不磨蹭。刹车加速度 brake_acc 与下方 sqrt 减速曲线**保持不变**（温柔刹车），
    // 末端因此自然物理慢下来（不是把末端交给视觉控制，只是放慢；慢下来后编码器+视觉融合更准）。
    // seg_len_cm<=0（未知）或段长超阈值：不提升，退回 max_acc，行为与改前完全一致。
    float accel_ramp = max_acc;
    {
        float boost = tune.short_seg_accel;
        float len_thresh = tune.short_seg_len_cm;
        if (!long_segment &&
            std::isfinite(seg_len_cm) && seg_len_cm >= 1.0f &&
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
    //     启用；AUTO 正循迹不启用，避免近端零速死区削掉顶箱力气，停车由 Tracker 的停稳状态收尾
    //   （2026-07-15 删 StopBand 后一度回纯 sqrt 直落；同日改为下方"接近区双段 sqrt 提前刹车"）
    float rest_radius = tune.tracker.reach_radius;
    bool active_auto_track = (App::g_state.control.mode == ControlMode::AUTO_TRACKING &&
                             App::g_state.control.tracker_state == TrackerState::TRACKING);
    bool is_stop = (target_end_speed <= 0.1f);              // 停车工况（含 AUTO 推箱/终点）
    bool spring_terminal = (is_stop && !active_auto_track); // 弹簧静止死区仅非 AUTO 启用（AUTO 由 Tracker 到达状态收尾）

    // k^1.5 模式下长直段提高加速上限，末端减速窗口在下方按长短段分别选择
    if (is_stop && MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_BRAKE && long_segment) {
        accel_ramp *= LinearTerminalConfig::LONG_ACCEL_GAIN;
    }

    // POINT_TRACKING 的业务到达判定使用 reach_radius_min，停车死区不能比它更大
    // 否则发车会在观测点外提前停住，GameManage 永远无法进入地图请求状态
    if (spring_terminal && MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_BRAKE &&
        std::isfinite(tune.tracker.reach_radius_min) && tune.tracker.reach_radius_min >= 0.05f) {
        rest_radius = std::min(rest_radius, tune.tracker.reach_radius_min);
    }
    if (!std::isfinite(rest_radius) || rest_radius < 0.01f) {
        rest_radius = 0.3f;
    }

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
    // 非线性末端刹车状态机总开关关闭、或 k^1.5 曲线已接管 → zone=0，不进双段 sqrt 分支
    float approach_zone = 0.0f;
    float approach_acc = 0.0f;
    if (is_stop && MotionFeatureSwitches::ENABLE_NONLINEAR_TERMINAL_BRAKE &&
        !MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_BRAKE &&
        tune.approach_enable >= 0.5f) {   // 运行期旋钮 !T N 0 仍可临时关掉非线性接近区
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
    if (is_stop && MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_BRAKE) {
        // ---- k^1.5 整形末端刹车状态机 ----
        float terminal_cruise_speed = std::min(LinearTerminalConfig::CRUISE_SPEED_CM_S, max_speed);
        float cruise_speed = terminal_cruise_speed;
        float configured_min_speed = long_segment
            ? LinearTerminalConfig::LONG_MIN_SPEED_CM_S
            : LinearTerminalConfig::SHORT_MIN_SPEED_CM_S;
        float min_speed = std::min(configured_min_speed, cruise_speed);
        float stop_dist = LinearTerminalConfig::STOP_DIST_CM;
        float slowdown_dist = long_segment
            ? LinearTerminalConfig::LONG_SLOWDOWN_DIST_CM
            : LinearTerminalConfig::SHORT_SLOWDOWN_DIST_CM;

        // 长直段提高远端巡航速度，并使用独立的末端减速窗口和最低速度
        if (long_segment) {
            cruise_speed = std::min(cruise_speed * LinearTerminalConfig::LONG_CRUISE_GAIN,
                                    LinearTerminalConfig::LONG_MAX_CRUISE_CM_S);
            cruise_speed = std::min(cruise_speed, max_speed);
            min_speed = std::min(min_speed, cruise_speed);
        }

        // 直接使用配置的减速距离和当前真实剩余距离，不再按段长压缩减速窗口
        target_v = shaped_terminal_target_speed(distance,
                                                slowdown_dist,
                                                stop_dist,
                                                cruise_speed,
                                                min_speed);
        target_v = std::min(target_v, max_speed);
    } else if (is_stop && approach_zone > 0.5f) {
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
    // 刹车斜坡：k^1.5 末端刹车用它自己的每拍降速步长；否则用 brake_acc·dt
    // （非线性接近区缓曲线只是目标更低，不需要更大限幅）
    float max_dv_dec = (is_stop && MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_BRAKE)
        ? LinearTerminalConfig::DECEL_STEP_CM_S
        : brake_acc * dt;

    // 停车接近区抖减速：仅停车工况放大**每拍减速上限**（不动 target 曲线/加速斜坡/过弯带速）。
    // 带速冲进停车航点的车会被原始 brake_acc·dt 温柔限幅卡住、来不及掉到 sqrt 目标曲线上就冲过点；
    // 乘上 stop_approach_brake_gain 让它一步抖到曲线上→进点速度低→固定视觉延迟造成的过冲随之缩小。
    // 车已在曲线上时 target_v≈current_v，限幅本就不 binding→放大它零行为改变；gain=1.0 完全等于旧行为。
    if (is_stop && MotionFeatureSwitches::ENABLE_NONLINEAR_TERMINAL_BRAKE &&
        !MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_BRAKE) {
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


/// \brief 前瞻路径线跟踪，输出全局期望速度
///
/// \details
/// 沿轨剩余距离交给速度规划器，动态前瞻点决定行驶方向
/// 横向误差过大时同步降速并增加朝路径线的拉回分量
/// 最终二维方向通过 alpha=0.25 一阶滤波，切段时不清空滤波状态
///
__attribute__((section(".ramfunc")))
Speed2D PathLineFollower::filter_direction(Speed2D desired, float speed_limit) {
    if (!std::isfinite(speed_limit) || speed_limit <= 0.001f ||
        !std::isfinite(desired.vx) || !std::isfinite(desired.vy)) {
        filtered_vel = {0.0f, 0.0f};
        return filtered_vel;
    }

    filtered_vel.vx += (desired.vx - filtered_vel.vx) * PATH_DIRECTION_FILTER_ALPHA;
    filtered_vel.vy += (desired.vy - filtered_vel.vy) * PATH_DIRECTION_FILTER_ALPHA;

    float filtered_speed = sqrtf(filtered_vel.vx * filtered_vel.vx +
                                 filtered_vel.vy * filtered_vel.vy);
    if (filtered_speed > speed_limit && filtered_speed > 0.001f) {
        float scale = speed_limit / filtered_speed;
        filtered_vel.vx *= scale;
        filtered_vel.vy *= scale;
    }
    return filtered_vel;
}

__attribute__((section(".ramfunc")))
Speed2D PathLineFollower::follow(float px, float py, float sx, float sy,
                                 float tx, float ty, float dt, float end_speed) {
    float seg_dx = tx - sx;
    float seg_dy = ty - sy;
    float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
    float seg_len = sqrtf(seg_len_sq);

    if (seg_len < 0.001f) {
        Speed2D direct = planner.velocity_planning_1d(
            tx - px, ty - py, dt, end_speed, seg_len);
        float speed_limit = sqrtf(direct.vx * direct.vx + direct.vy * direct.vy);
        return filter_direction(direct, speed_limit);
    }

    float inv_len = 1.0f / seg_len;
    float along_x = seg_dx * inv_len;
    float along_y = seg_dy * inv_len;
    float perp_x = -along_y;
    float perp_y = along_x;

    float s_remain = (tx - px) * along_x + (ty - py) * along_y;
    Speed2D along_vel = planner.velocity_planning_1d(
        along_x * s_remain, along_y * s_remain, dt, end_speed, seg_len);
    float scalar_speed = sqrtf(along_vel.vx * along_vel.vx + along_vel.vy * along_vel.vy);
    if (scalar_speed <= 0.001f) {
        return filter_direction({0.0f, 0.0f}, 0.0f);
    }

    bool long_segment = seg_len >= LinearTerminalConfig::LONG_SEGMENT_THRESHOLD_CM;
    float lookahead_max = long_segment ? LONG_PATH_LOOKAHEAD_MAX_CM : PATH_LOOKAHEAD_MAX_CM;
    float lateral_lookahead_k = long_segment ? LONG_PATH_LATERAL_LOOKAHEAD_K : PATH_LATERAL_LOOKAHEAD_K;
    float lateral_slow_start = long_segment ? LONG_PATH_LATERAL_SLOW_START_CM : PATH_LATERAL_SLOW_START_CM;
    float lateral_slow_full = long_segment ? LONG_PATH_LATERAL_SLOW_FULL_CM : PATH_LATERAL_SLOW_FULL_CM;
    float lateral_min_scale = long_segment ? LONG_PATH_LATERAL_MIN_SPEED_SCALE : PATH_LATERAL_MIN_SPEED_SCALE;
    float track_gain = long_segment ? LONG_PATH_TRACK_GAIN_S : PATH_TRACK_GAIN_S;
    float track_max_ratio = long_segment ? LONG_PATH_TRACK_MAX_RATIO : PATH_TRACK_MAX_RATIO;

    float rel_x = px - sx;
    float rel_y = py - sy;
    float along = rel_x * along_x + rel_y * along_y;
    float lateral = rel_x * perp_x + rel_y * perp_y;
    float abs_lateral = std::abs(lateral);
    float remain = std::max(s_remain, 0.0f);

    float lookahead = PATH_LOOKAHEAD_MIN_CM + scalar_speed * PATH_LOOKAHEAD_TIME_S -
                      abs_lateral * lateral_lookahead_k;
    lookahead = std::clamp(lookahead, PATH_LOOKAHEAD_MIN_CM, lookahead_max);
    if (remain < PATH_CORNER_APPROACH_CM && lookahead > remain) {
        lookahead = remain;
    }

    along = std::clamp(along, 0.0f, seg_len);
    float aim_along = std::min(along + lookahead, seg_len);
    float aim_x = sx + along_x * aim_along;
    float aim_y = sy + along_y * aim_along;
    float aim_dx = aim_x - px;
    float aim_dy = aim_y - py;
    float aim_norm = sqrtf(aim_dx * aim_dx + aim_dy * aim_dy);
    if (aim_norm < 0.001f) {
        aim_dx = tx - px;
        aim_dy = ty - py;
        aim_norm = sqrtf(aim_dx * aim_dx + aim_dy * aim_dy);
        if (aim_norm < 0.001f) {
            return filter_direction({0.0f, 0.0f}, 0.0f);
        }
    }

    float speed_scale = 1.0f;
    if (abs_lateral > lateral_slow_start) {
        float slow_span = lateral_slow_full - lateral_slow_start;
        speed_scale = 1.0f - (abs_lateral - lateral_slow_start) / slow_span *
                               (1.0f - lateral_min_scale);
        speed_scale = std::clamp(speed_scale, lateral_min_scale, 1.0f);
    }

    float desired_speed = scalar_speed * speed_scale;
    Speed2D desired = {
        aim_dx / aim_norm * desired_speed,
        aim_dy / aim_norm * desired_speed
    };

    if (abs_lateral > PATH_TRACK_DEADBAND_CM) {
        float lateral_pull = (abs_lateral - PATH_TRACK_DEADBAND_CM) * track_gain;
        lateral_pull = std::min(lateral_pull, desired_speed * track_max_ratio);
        if (lateral > 0.0f) {
            lateral_pull = -lateral_pull;
        }
        desired.vx += lateral_pull * perp_x;
        desired.vy += lateral_pull * perp_y;
    }

    return filter_direction(desired, scalar_speed);
}

} // namespace Algorithm::Motion
