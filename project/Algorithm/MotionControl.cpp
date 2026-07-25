#include "MotionControl.h"
#include "RobotState.h"


namespace Algorithm::Motion {

namespace {
constexpr float DEFAULT_MAX_ANG_VEL = 10.0f;
constexpr float DEFAULT_MAX_ANG_ACC = 40.0f;

constexpr float linear_terminal_target_speed(float remaining_dist,
                                             float slowdown_dist,
                                             float stop_dist,
                                             float cruise_speed,
                                             float min_speed) {
    if (remaining_dist >= slowdown_dist) return cruise_speed;
    if (remaining_dist <= stop_dist) return min_speed;

    float k = (remaining_dist - stop_dist) / (slowdown_dist - stop_dist);
    return min_speed + (cruise_speed - min_speed) * k;
}

static_assert(linear_terminal_target_speed(45.0f, 45.0f, 2.0f, 60.0f, 12.0f) == 60.0f,
              "linear slowdown start mismatch");
static_assert(linear_terminal_target_speed(23.5f, 45.0f, 2.0f, 60.0f, 12.0f) == 36.0f,
              "linear slowdown midpoint mismatch");
static_assert(linear_terminal_target_speed(2.0f, 45.0f, 2.0f, 60.0f, 12.0f) == 12.0f,
              "linear slowdown stop mismatch");

struct LinearRuntimeParams {
    float cruise_speed;
    float min_speed;
    float slowdown_dist;
    float stop_dist;
    float decel_step;
    float long_seg_thresh;
};

inline float valid_linear_value(float value, float min_value, float max_value, float fallback) {
    return std::isfinite(value) && value >= min_value && value <= max_value ? value : fallback;
}

inline LinearRuntimeParams read_linear_runtime_params() {
    LinearRuntimeParams p = {
        valid_linear_value(tune.linear.cruise_speed,
                           TuningDefaults::MIN_LINEAR_CRUISE_SPEED,
                           TuningDefaults::MAX_LINEAR_CRUISE_SPEED,
                           TuningDefaults::DEFAULT_LINEAR_CRUISE_SPEED),
        valid_linear_value(tune.linear.min_speed,
                           TuningDefaults::MIN_LINEAR_MIN_SPEED,
                           TuningDefaults::MAX_LINEAR_MIN_SPEED,
                           TuningDefaults::DEFAULT_LINEAR_MIN_SPEED),
        valid_linear_value(tune.linear.slowdown_dist,
                           TuningDefaults::MIN_LINEAR_SLOWDOWN_DIST,
                           TuningDefaults::MAX_LINEAR_SLOWDOWN_DIST,
                           TuningDefaults::DEFAULT_LINEAR_SLOWDOWN_DIST),
        valid_linear_value(tune.linear.stop_dist,
                           TuningDefaults::MIN_LINEAR_STOP_DIST,
                           TuningDefaults::MAX_LINEAR_STOP_DIST,
                           TuningDefaults::DEFAULT_LINEAR_STOP_DIST),
        valid_linear_value(tune.linear.decel_step,
                           TuningDefaults::MIN_LINEAR_DECEL_STEP,
                           TuningDefaults::MAX_LINEAR_DECEL_STEP,
                           TuningDefaults::DEFAULT_LINEAR_DECEL_STEP),
        valid_linear_value(tune.linear.long_seg_thresh,
                           TuningDefaults::MIN_LINEAR_LONG_SEG_THRESH,
                           TuningDefaults::MAX_LINEAR_LONG_SEG_THRESH,
                           TuningDefaults::DEFAULT_LINEAR_LONG_SEG_THRESH)
    };

    if (p.min_speed > p.cruise_speed) {
        p.min_speed = std::min(p.cruise_speed, TuningDefaults::DEFAULT_LINEAR_MIN_SPEED);
    }
    if (p.stop_dist >= p.slowdown_dist) {
        p.stop_dist = TuningDefaults::DEFAULT_LINEAR_STOP_DIST;
        p.slowdown_dist = TuningDefaults::DEFAULT_LINEAR_SLOWDOWN_DIST;
    }
    return p;
}

// 只对速度方向做低通，速度大小直接跟随轨迹规划器的标量输出。
// 这样拐点的方向仍然平滑，但 90 度切段不会因为两个分量互相抵消而凭空掉速。
inline Speed2D filter_velocity_direction(float desired_vx,
                                         float desired_vy,
                                         float desired_speed,
                                         float& filtered_vx,
                                         float& filtered_vy) {
    if (!std::isfinite(desired_speed) || desired_speed <= 0.001f) {
        filtered_vx = 0.0f;
        filtered_vy = 0.0f;
        return {0.0f, 0.0f};
    }

    float desired_norm = hypotf(desired_vx, desired_vy);
    if (!std::isfinite(desired_norm) || desired_norm <= 0.001f) {
        float previous_norm = hypotf(filtered_vx, filtered_vy);
        if (!std::isfinite(previous_norm) || previous_norm <= 0.001f) {
            filtered_vx = 0.0f;
            filtered_vy = 0.0f;
            return {0.0f, 0.0f};
        }
        filtered_vx *= desired_speed / previous_norm;
        filtered_vy *= desired_speed / previous_norm;
        return {filtered_vx, filtered_vy};
    }

    const float desired_ux = desired_vx / desired_norm;
    const float desired_uy = desired_vy / desired_norm;
    float previous_norm = hypotf(filtered_vx, filtered_vy);
    if (!std::isfinite(previous_norm) || previous_norm <= 0.001f) {
        filtered_vx = desired_ux * desired_speed;
        filtered_vy = desired_uy * desired_speed;
        return {filtered_vx, filtered_vy};
    }

    const float previous_ux = filtered_vx / previous_norm;
    const float previous_uy = filtered_vy / previous_norm;
    float ux = previous_ux + (desired_ux - previous_ux) * LookaheadConfig::FILTER_ALPHA;
    float uy = previous_uy + (desired_uy - previous_uy) * LookaheadConfig::FILTER_ALPHA;
    float direction_norm = hypotf(ux, uy);
    if (!std::isfinite(direction_norm) || direction_norm <= 0.001f) {
        ux = desired_ux;
        uy = desired_uy;
        direction_norm = 1.0f;
    }

    filtered_vx = ux / direction_norm * desired_speed;
    filtered_vy = uy / direction_norm * desired_speed;
    return {filtered_vx, filtered_vy};
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
Speed2D Trajectory::velocity_planning_1d(float dx, float dy, float dt, float end_speed,
                                        float seg_len_cm) {
    float distance = sqrtf(dx * dx + dy * dy);

    float max_speed = tune.dynamics.max_vel;
    float max_acc = tune.dynamics.max_acc;
    float brake_acc = max_acc * tune.dynamics.brake_limit;
    float target_end_speed = std::clamp(end_speed, 0.0f, max_speed);
    const LinearRuntimeParams linear = read_linear_runtime_params();

    // 长短距离起步加速度切换：原有组合功能开关打开且段全长 <= short_seg_len_cm 时，
    // 只把加速斜坡切换为 short_seg_accel，刹车加速度与目标减速曲线保持不变
    // seg_len_cm<=0（未知）、段长超阈值或开关关闭时统一使用 max_acc
    float accel_ramp = max_acc;
    {
        float short_accel = tune.short_seg_accel;
        float len_thresh = tune.short_seg_len_cm;
        if (MotionFeatureSwitches::ENABLE_STEP_BRAKE_AND_SHORT_ACCEL &&
            std::isfinite(seg_len_cm) && seg_len_cm >= 1.0f &&
            std::isfinite(len_thresh) && seg_len_cm <= len_thresh &&
            std::isfinite(short_accel) && short_accel > 0.0f) {
            accel_ramp = short_accel;
        }
    }

    if (distance < 0.001f) {
        current_v = 0.0f;
        return {0.0f, 0.0f};
    }

    // 停车工况终端整形：所有停车目标都由规划器自然收速到零，不依赖追点过程中的硬锁。
    // 这样视觉/里程计把车暂时推离目标时，控制器仍能恢复追点，不会被清零分支永久截住。
    float rest_radius = tune.tracker.reach_radius;
    bool is_stop = (target_end_speed <= 0.1f);              // 停车工况（含 AUTO 推箱/终点）
    if (MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_DECEL) {
        // 终端窗口至少覆盖一拍可观测的位移，避免小半径被单拍直接穿过。
        rest_radius = std::max(rest_radius, std::max(linear.stop_dist, 0.8f));
    }
    if (!std::isfinite(rest_radius) || rest_radius < 0.01f) {
        rest_radius = MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_DECEL
            ? std::max(0.8f, TuningDefaults::DEFAULT_LINEAR_STOP_DIST)
            : 0.3f;
    }
    bool spring_terminal = is_stop;

    // 弹簧静止点（治空载/保持点"原地颤抖"根因）：进"到达半径"内直接停伺服、命令零速，不再追毫米级
    // 残差——sqrt 在 d→0 斜率无穷会把残差放大成大速度→原地猛冲抖，max_acc 越大越抖。
    // 无状态：车被推出半径立即恢复伺服，绝不锁存、绝不在接近途中提前停。等效 yaw 环 ang_tolerance。
    if (spring_terminal && distance <= rest_radius) {
        current_v = 0.0f;
        return {0.0f, 0.0f};
    }

    // 停车接近区双段刹车（治"末端总是出去一些、StopBrkG 大小都调不好"）：
    // 单一 sqrt 曲线难以同时兼顾高速接近和精确停车。
    // 修法=**提前刹车**：停车段最后 zone cm 换一条更缓的 sqrt 曲线（approach_brake_acc），
    // 外段仍按原 brake_acc 刹车、在 zone 边界与缓曲线 C0 连续衔接——刹车点因此显著提前，
    // 车末端明显慢下来（非固定爬行速度），低速下视觉延迟误差≈速度×310ms 变小、编码器不打滑，
    // 视觉+编码器融合在进点前就收敛到真实位置。zone = min(zone_cm, 段全长×ratio)：
    // 长距离封顶 40cm，20cm 段→5cm（用户拍板）。参数垃圾/关闭(!T Z 0.5)时 zone=0 → 回纯 sqrt 直落。
    float approach_zone = 0.0f;
    float approach_acc = 0.0f;
    if (is_stop && !MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_DECEL &&
        tune.approach_enable >= 0.5f) {   // 线性模式关闭后才使用原双段 sqrt
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
    if (is_stop && MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_DECEL) {
        // 线性末端参数改为读 tune.linear（可菜单/串口在线调并持久化），默认值 = -1he-new 一套。
        float terminal_cruise_speed = std::min(linear.cruise_speed, max_speed);
        float cruise_speed = terminal_cruise_speed;
        float min_speed = std::min(linear.min_speed, cruise_speed);
        float stop_dist = linear.stop_dist;
        float slowdown_dist = linear.slowdown_dist;

        // 长直段只提高远端巡航速度，停车末端与短段共用同一减速窗口
        if (std::isfinite(seg_len_cm) && seg_len_cm >= linear.long_seg_thresh) {
            cruise_speed = std::min(cruise_speed * LinearTerminalConfig::LONG_CRUISE_GAIN,
                                    LinearTerminalConfig::LONG_MAX_CRUISE_CM_S);
            cruise_speed = std::min(cruise_speed, max_speed);
            min_speed = std::min(min_speed, cruise_speed);
        }

        if (std::isfinite(seg_len_cm) && seg_len_cm > 0.0f) {
            // 末端窗口只参考单格短段长度，不能随长段全长扩大；否则同一剩余距离下
            // 长段 target_v 会比短段更低，表现为长距离末端拖慢。
            float reference_len = std::min(seg_len_cm, LinearTerminalConfig::SHORT_SEG_REFERENCE_CM);
            slowdown_dist = std::min(slowdown_dist,
                                     reference_len * LinearTerminalConfig::SHORT_SEG_SLOWDOWN_RATIO);
            // 减速带须仍大于到达判定距离，避免退化成"没有减速段直接判到达"
            slowdown_dist = std::max(slowdown_dist, stop_dist + 1.0f);
        }

        // 末端"防重新加速闸"（治：减速到低速后、临近点一帧视觉把 pose 往回拽 → distance 跳大
        // → 线性曲线把 target_v 又拔回高位 → max_acc 斜坡顶着往前冲刺一下）。
        // 用单调不增的锁存距离喂曲线：远端(未进减速区)持续把锁存重置为当前 distance；一旦进减速区
        // 就只记录见过的最小距离——视觉回拽使 distance 变大也不抬 target_v。短段从低速起步时
        // distance 单调减小、锁存跟随下降，起步加速不受影响（不会像"钳到 current_v"那样把车卡死）。
        if (distance >= slowdown_dist) {
            terminal_min_dist = distance;         // 未进减速区：跟随，随时准备进区锁存
        } else if (distance < terminal_min_dist) {
            terminal_min_dist = distance;         // 进减速区：只锁存更小的距离
        }
        float shaping_dist = std::min(distance, terminal_min_dist);

        // 长段远端巡航与短段不同，但进入共用末端窗口时平滑过渡到短段巡航值
        float curve_cruise_speed = terminal_cruise_speed;
        if (cruise_speed > terminal_cruise_speed && shaping_dist > slowdown_dist) {
            constexpr float LONG_CRUISE_BLEND_DIST_CM = 20.0f;
            float blend = (shaping_dist - slowdown_dist) / LONG_CRUISE_BLEND_DIST_CM;
            blend = std::clamp(blend, 0.0f, 1.0f);
            curve_cruise_speed += (cruise_speed - terminal_cruise_speed) * blend;
        }
        target_v = linear_terminal_target_speed(shaping_dist,
                                                slowdown_dist,
                                                stop_dist,
                                                curve_cruise_speed,
                                                min_speed);
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
    float max_dv_acc = accel_ramp * dt;   // 加速斜坡：短段可切换到 short_seg_accel
    float max_dv_dec = (is_stop && MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_DECEL)
        ? linear.decel_step
        : brake_acc * dt;

    // 停车接近区抖减速：仅停车工况放大**每拍减速上限**（不动 target 曲线/加速斜坡/过弯带速）。
    // 带速冲进停车航点的车会被原始 brake_acc·dt 温柔限幅卡住、来不及掉到 sqrt 目标曲线上就冲过点；
    // 乘上 stop_approach_brake_gain 让它一步抖到曲线上→进点速度低→固定视觉延迟造成的过冲随之缩小。
    // 车已在曲线上时 target_v≈current_v，限幅本就不 binding→放大它零行为改变；gain<=1.0 完全等于旧行为。
    if (is_stop && !MotionFeatureSwitches::ENABLE_LINEAR_TERMINAL_DECEL &&
        MotionFeatureSwitches::ENABLE_STEP_BRAKE_AND_SHORT_ACCEL) {
        float brake_gain = tune.stop_approach_brake_gain;
        if (!(brake_gain >= 0.0f && brake_gain <= 100.0f)) {
            brake_gain = 1.0f;   // 运行期兜底：NaN/越界（比较恒 false 落此）→ 退回旧行为，不依赖 sanitize
        } else if (brake_gain < 1.0f) {
            brake_gain = 1.0f;   // 0 表示关闭额外刹车，等效旧行为
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
/// 旋转主体严格沿用 2026-07-16 版本：sqrt 远端、线性近端、静摩擦补偿、IMU 阻尼和角加速度限幅。
/// 本次只把容差边缘的瞬时清零改为带滞回的连续收尾，避免每 20ms 启停造成一顿一顿；
/// 这里不引用 7.16 的任何平移速度曲线。
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

    // 调参或传感器异常时仍保持有限输出，避免 NaN 把四轮目标速度传播成永久停机。
    if (!std::isfinite(err)) {
        reset();
        return 0.0f;
    }
    if (!std::isfinite(dt) || dt <= 0.001f) {
        dt = SystemConfig::PIT_CH1_DT_S;
    }
    if (!std::isfinite(yaw_rate)) {
        yaw_rate = 0.0f;
    }

    // 先准备 7.16 主体使用的物理限幅；容差内也沿同一角加速度限制连续收尾。
    const float max_ang_acc = valid_linear_value(tune.dynamics.max_ang_acc,
                                                 0.001f,
                                                 500.0f,
                                                 DEFAULT_MAX_ANG_ACC);
    const float max_ang_vel = valid_linear_value(tune.dynamics.max_ang_vel,
                                                 0.0f,
                                                 100.0f,
                                                 DEFAULT_MAX_ANG_VEL);
    float abs_err = std::abs(err);
    const float control_dt = std::clamp(dt, 0.001f, 0.1f);
    const float max_dv = max_ang_acc * control_dt;
    const float angle_tolerance = valid_linear_value(tune.tracker.ang_tolerance,
                                                     0.001f,
                                                     0.5f,
                                                     0.01f);
    // 在容差边缘保持一小段滞回，避免 IMU 噪声让角速度每 20ms 反复启停。
    const float tolerance_release = std::max(angle_tolerance * 2.0f, 0.02f);
    if (tolerance_hold) {
        if (abs_err <= tolerance_release) {
            float target_vw = -valid_linear_value(tune.yaw.kd,
                                                  TuningDefaults::MIN_YAW_KD,
                                                  TuningDefaults::MAX_YAW_KD,
                                                  TuningDefaults::DEFAULT_YAW_KD) * yaw_rate;
            target_vw = std::clamp(target_vw, -max_ang_vel, max_ang_vel);
            float last_vw = current_vw;
            if (target_vw > current_vw + max_dv) {
                current_vw += max_dv;
            } else if (target_vw < current_vw - max_dv) {
                current_vw -= max_dv;
            } else {
                current_vw = target_vw;
            }
            if (std::abs(current_vw) < 0.01f && std::abs(yaw_rate) < 0.05f) {
                current_vw = 0.0f;
            }
            current_aw = (current_vw - last_vw) / control_dt;
            return current_vw;
        }
        tolerance_hold = false;
    }
    if (abs_err <= angle_tolerance) {
        tolerance_hold = true;
        // 进入容差后仍按物理斜率收掉残余角速度，而不是瞬间清零。
        float target_vw = -valid_linear_value(tune.yaw.kd,
                                              TuningDefaults::MIN_YAW_KD,
                                              TuningDefaults::MAX_YAW_KD,
                                              TuningDefaults::DEFAULT_YAW_KD) * yaw_rate;
        target_vw = std::clamp(target_vw, -max_ang_vel, max_ang_vel);
        float last_vw = current_vw;
        if (target_vw > current_vw + max_dv) {
            current_vw += max_dv;
        } else if (target_vw < current_vw - max_dv) {
            current_vw -= max_dv;
        } else {
            current_vw = target_vw;
        }
        current_aw = (current_vw - last_vw) / control_dt;
        return current_vw;
    }

    // 线性带宽下限护栏：过小会退化回 sqrt 的无穷斜率问题
    float lin_band = valid_linear_value(tune.yaw.lin_band,
                                        0.005f,
                                        2.0f,
                                        TuningDefaults::DEFAULT_YAW_LIN_BAND);

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
        float translate_gain = valid_linear_value(tune.yaw.translate_gain,
                                                  TuningDefaults::MIN_YAW_TRANSLATE_GAIN,
                                                  TuningDefaults::MAX_YAW_TRANSLATE_GAIN,
                                                  TuningDefaults::DEFAULT_YAW_TRANSLATE_GAIN);
        target_abs_vw *= translate_gain;
    }
    target_abs_vw = std::min(target_abs_vw, max_ang_vel);

    // 静摩擦补偿：提供一个最小的纠正角速度地板，避免小偏差纠正力度不足
    // 平移时轮子动态摩擦低于静止，地板减至 40%，仍然保证基本纠正能力
    {
        float stiction = valid_linear_value(tune.yaw.stiction,
                                            TuningDefaults::MIN_YAW_STICTION,
                                            TuningDefaults::MAX_YAW_STICTION,
                                            TuningDefaults::DEFAULT_YAW_STICTION);
        float stiction_vw = std::min(stiction, max_ang_vel);
        if (is_translating) stiction_vw *= 0.4f;
        float stiction_brake_err = (stiction_vw * stiction_vw) / (2.0f * max_ang_acc);
        if (target_abs_vw < stiction_vw && abs_err > stiction_brake_err) {
            target_abs_vw = stiction_vw;
        }
    }

    float target_vw = std::copysign(target_abs_vw, err);

    // 陀螺阻尼：平移时减小阻尼，避免被动带歪阶段削弱主动纠偏
    // 放在加速度限幅之前，使阻尼也受物理步长约束，不会引入单帧跳变
    float yaw_kd = is_translating
        ? valid_linear_value(tune.yaw.kd_translate,
                             TuningDefaults::MIN_YAW_KD_TRANSLATE,
                             TuningDefaults::MAX_YAW_KD_TRANSLATE,
                             TuningDefaults::DEFAULT_YAW_KD_TRANSLATE)
        : valid_linear_value(tune.yaw.kd,
                             TuningDefaults::MIN_YAW_KD,
                             TuningDefaults::MAX_YAW_KD,
                             TuningDefaults::DEFAULT_YAW_KD);
    target_vw -= yaw_kd * yaw_rate;
    target_vw = std::clamp(target_vw, -max_ang_vel, max_ang_vel);

    // 加速度物理限幅：防止起步打滑和急刹打滑
    float last_vw = current_vw;
    if (target_vw > current_vw + max_dv) {
        current_vw += max_dv;
    } else if (target_vw < current_vw - max_dv) {
        current_vw -= max_dv;
    } else {
        current_vw = target_vw;
    }

    current_aw = (current_vw - last_vw) / control_dt;
    return current_vw;
}


/// \brief 沿路径线跟踪：lookahead 前瞻瞄点 + 速度矢量低通，输出全局期望速度
///
/// \details
/// 完全照搬 -1he-new/code/control.c 的切向机制（2026-07-25 用户规格）：
///   1. 沿轨方向：用剩余投影距离喂梯形规划器得主速度大小 cmd_speed（含末端线性减速曲线）。
///   2. lookahead 前瞻瞄点：瞄向段上前方 6~16cm（随速度伸长、随横偏收缩，长段前瞻更大）而非
///      终点，提前偏舵在拐点画平滑弧而非直角急停；临近段末前瞻收缩到剩余距离避免越点乱瞄。
///   3. 速度方向一阶低通(FILTER_ALPHA=0.45)：方向突变被平滑成 2~3 拍过渡，速度大小跟随规划器，
///      避免切段时两个分量抵消造成无意义的掉速。低通历史在急停/切段停车由 reset() 清零，
///      中间拐点不 reset → 速度矢量跨界连续，实现"带速切向"。
///   4. 速度大小始终取规划器的 cmd_speed：方向低通不会把末端减速拖慢，也不会放大规划速度。
/// 段长 <3cm 时 perp 方向不可靠，退化为纯朝目标点（仍走低通平滑）。
///
__attribute__((section(".ramfunc")))
Speed2D PathLineFollower::follow(float px, float py, float sx, float sy,
                                 float tx, float ty, float dt, float end_speed) {
    float seg_dx = tx - sx;
    float seg_dy = ty - sy;
    float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
    float seg_len = sqrtf(seg_len_sq);   // 段全长，喂规划器做长短段起步加速度判定

    // Keep the planner's velocity continuous at a corner, while clearing the
    // monotonic terminal-distance latch belonging to the previous segment.
    constexpr float SEGMENT_KEY_EPS_CM = 0.01f;
    if (!segment_history_valid ||
        std::abs(sx - last_sx) > SEGMENT_KEY_EPS_CM ||
        std::abs(sy - last_sy) > SEGMENT_KEY_EPS_CM ||
        std::abs(tx - last_tx) > SEGMENT_KEY_EPS_CM ||
        std::abs(ty - last_ty) > SEGMENT_KEY_EPS_CM) {
        planner.reset_terminal_profile();
        last_sx = sx;
        last_sy = sy;
        last_tx = tx;
        last_ty = ty;
        segment_history_valid = true;
    }

    // 段长过短：退化为纯朝目标点（前瞻几何在极短段不可靠），仍走低通平滑
    if (seg_len_sq < 9.0f) {  // <3cm
        Speed2D v = planner.velocity_planning_1d(tx - px, ty - py, dt, end_speed, seg_len);
        return filter_velocity_direction(v.vx, v.vy, hypotf(v.vx, v.vy), cmd_vx, cmd_vy);
    }

    float inv_len = 1.0f / seg_len;
    float along_x = seg_dx * inv_len;    // 段单位方向
    float along_y = seg_dy * inv_len;
    float perp_x = -along_y;             // 路径线法向（逆时针 90°）
    float perp_y = along_x;

    // 沿轨投影只用于前瞻几何；越过目标投影面后，速度规划改用真实点距继续收敛
    float rel_x = px - sx;
    float rel_y = py - sy;
    float raw_along = rel_x * along_x + rel_y * along_y;
    float raw_remain = seg_len - raw_along;
    float along = std::clamp(raw_along, 0.0f, seg_len);
    float target_dist = hypotf(tx - px, ty - py);
    float planning_dist = raw_remain;
    if (planning_dist < LinearTerminalConfig::STOP_DIST_CM) {
        planning_dist = target_dist;
    }
    planning_dist = std::max(planning_dist, 0.0f);
    Speed2D along_vel = planner.velocity_planning_1d(
        along_x * planning_dist, along_y * planning_dist, dt, end_speed, seg_len);
    float cmd_speed = hypotf(along_vel.vx, along_vel.vy);  // 规划器给出的期望速率（低通后封顶用）

    float s_remain = std::max(raw_remain, 0.0f);

    // ---- lookahead 前瞻瞄点（照搬 -1he-new）：瞄向段上前方一点而非终点，提前偏舵画平滑弧 ----
    float lateral = rel_x * perp_x + rel_y * perp_y;   // 带符号横偏
    float abs_lateral = std::abs(lateral);
    bool is_long_seg = (seg_len >= LinearTerminalConfig::LONG_SEGMENT_THRESHOLD_CM);
    float look_max = is_long_seg ? LookaheadConfig::LONG_LOOKAHEAD_MAX_CM
                                 : LookaheadConfig::LOOKAHEAD_MAX_CM;
    float lateral_k = is_long_seg ? LookaheadConfig::LONG_LATERAL_LOOKAHEAD_K
                                  : LookaheadConfig::LATERAL_LOOKAHEAD_K;

    float lookahead = LookaheadConfig::LOOKAHEAD_MIN_CM
                    + cmd_speed * LookaheadConfig::LOOKAHEAD_TIME_S
                    - abs_lateral * lateral_k;
    lookahead = std::clamp(lookahead, LookaheadConfig::LOOKAHEAD_MIN_CM, look_max);
    // 临近段末：前瞻不超过剩余距离，避免越过终点乱瞄
    if (s_remain < LookaheadConfig::CORNER_APPROACH_CM && lookahead > s_remain) {
        lookahead = s_remain;
    }
    if (lookahead < LookaheadConfig::LOOKAHEAD_MIN_CM && s_remain > LookaheadConfig::LOOKAHEAD_MIN_CM) {
        lookahead = LookaheadConfig::LOOKAHEAD_MIN_CM;
    }

    float aim_along = std::min(along + lookahead, seg_len);
    float aim_dx = (sx + along_x * aim_along) - px;
    float aim_dy = (sy + along_y * aim_along) - py;
    float aim_norm = hypotf(aim_dx, aim_dy);
    if (aim_norm < 0.001f) {   // 已到瞄点：退回朝终点
        aim_dx = tx - px;
        aim_dy = ty - py;
        aim_norm = hypotf(aim_dx, aim_dy);
        if (aim_norm < 0.001f) aim_norm = 0.001f;
    }

    float desired_vx = aim_dx / aim_norm * cmd_speed;
    float desired_vy = aim_dy / aim_norm * cmd_speed;

    // ---- 速度方向一阶低通：方向突变被平滑成 2~3 拍过渡，标量速度不再被向量抵消拖慢 ----
    return filter_velocity_direction(desired_vx, desired_vy, cmd_speed, cmd_vx, cmd_vy);
}

} // namespace Algorithm::Motion
