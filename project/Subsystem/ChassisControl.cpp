#include "ChassisControl.h"
#include "RobotState.h"
#include "tuning_config.h"

#include "Tracker.h"
#include "MotionControl.h"
#include <cmath>

#include "zf_common_headfile.h"
#include "Motor.h"
#include "Encoder.h"


namespace Subsystem::Chassis {

// ====================================================================
// 内部私有实现
// ====================================================================

__attribute__((section(".dtcm_data"))) static Motor motors[4] = {
    {C6,  PWM2_MODULE0_CHB_C7,  false},   // LF
    {C11, PWM2_MODULE2_CHA_C10, false},   // LB
    {C8,  PWM2_MODULE1_CHB_C9,  true },   // RF
    {D3,  PWM2_MODULE3_CHA_D2,  true }    // RB
};

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::Speed_PosPid pid_wheels[4] = {
    Algorithm::Motion::Speed_PosPid(tune.wheels[0].pid),   // LF：每轮独立整定
    Algorithm::Motion::Speed_PosPid(tune.wheels[1].pid),   // LB
    Algorithm::Motion::Speed_PosPid(tune.wheels[2].pid),   // RF
    Algorithm::Motion::Speed_PosPid(tune.wheels[3].pid)    // RB
};

// __attribute__((section(".dtcm_data"))) static Algorithm::Motion::Angle_PosPid pid_pos_yaw(tune.pid_yaw);

__attribute__((section(".dtcm_data"))) static Algorithm::Motion::PathLineFollower path_follower;
__attribute__((section(".dtcm_data"))) static Algorithm::Motion::YawProfiled yaw_controller;
// 慢环(20ms)产出、快环(5ms)消费的四轮目标速度。两段同在 PIT 中断、不并发，无需加锁。
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
// 每轮目标加速度(cm/s^2) = 慢环相邻两拍目标轮速之差/dt，供轮速环 ka/kb 加速度前馈（起步/刹车更脆）。
// 与上一拍目标轮速一同由 20ms 慢环维护，5ms 快环消费。
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_target_wheel_accels = {0.0f, 0.0f, 0.0f, 0.0f};
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_prev_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
// 20ms→5ms 线性内插：把慢环产出的目标轮速/加速度在 4 个快环拍上线性铺开，消除 20ms 台阶
// 对速度环的周期性激励（低速刹车段"微颤"根因）。快环消费 s_interp_*，慢环末尾发布步长。
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_interp_speed = {0.0f, 0.0f, 0.0f, 0.0f};
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_interp_accel = {0.0f, 0.0f, 0.0f, 0.0f};
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_speed_step = {0.0f, 0.0f, 0.0f, 0.0f};
__attribute__((section(".dtcm_data"))) static WheelSpeed4 s_accel_step = {0.0f, 0.0f, 0.0f, 0.0f};
__attribute__((section(".dtcm_data"))) static uint8_t s_interp_ticks_left = 0;
__attribute__((section(".dtcm_data"))) static bool control_history_ready = false;
__attribute__((section(".dtcm_data"))) static ControlMode last_control_mode = ControlMode::AUTO_TRACKING;
__attribute__((section(".dtcm_data"))) static TrackerState last_tracker_state = TrackerState::NONE;
__attribute__((section(".dtcm_data"))) static Pose2D last_control_target = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};

// --- 内部辅助函数声明 ---
namespace {
    // 防止偏航角误差出现 359度 变成 -1度 导致的疯狂原地打转，将角度归一化到 [-pi, pi] 范围内
    __attribute__((always_inline)) inline float normalize_angle(float angle) {
        if (angle > 180.0f)       angle -= 360.0f;
        else if (angle < -180.0f) angle += 360.0f;
        return angle * SystemConfig::DEG_TO_RAD;
    }

    __attribute__((always_inline)) inline float smooth_sign(float val) {
        // 0.5f 决定了过渡带的陡峭程度，值越小越接近阶跃，但不突变
        return val / (std::abs(val) + 0.5f);
    }

    // 单轮前馈（从 Branch 搬入，每轮参数独立）：
    //   kv·target  —— 速度前馈
    //   ks·smooth_sign(target) —— 静摩擦补偿（常态生效，破静摩擦）
    //   ka/kb·target_acc —— 目标加速度前馈（加速用 ka、刹车用 kb，按 target_acc 是否与
    //                        当前运动方向相反判别 braking）
    // 注意：这里**没有** master 旧的 brake_ff 主动制动锁定项。零速收尾改为松力滑行
    // （见 run_speed_loop 的零速死区），所以搬入后"锁死"现象自然消失。
    // 阶段1：run_speed_loop 传 target_acc=0，故 ka/kb 项恒为 0，只有 kv+ks 生效。
    __attribute__((always_inline)) inline float wheel_feedforward(
            const WheelControlParams& p, float target, float current, float target_acc) {
        float motion_ref = (std::abs(current) > 1.0f) ? current : target;
        bool braking = (motion_ref * target_acc) < -0.1f;
        float accel_gain = braking ? p.kb : p.ka;
        return target * p.kv + smooth_sign(target) * p.ks + target_acc * accel_gain;
    }

    __attribute__((always_inline)) inline void reset_motion_residue() {
        path_follower.reset();
        yaw_controller.reset();
        s_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        s_target_wheel_accels = {0.0f, 0.0f, 0.0f, 0.0f};
        s_prev_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        // 内插状态一并清零：切任务/急停后从零重新铺，避免残留步长把速度带偏
        s_interp_speed = {0.0f, 0.0f, 0.0f, 0.0f};
        s_interp_accel = {0.0f, 0.0f, 0.0f, 0.0f};
        s_speed_step = {0.0f, 0.0f, 0.0f, 0.0f};
        s_accel_step = {0.0f, 0.0f, 0.0f, 0.0f};
        s_interp_ticks_left = 0;
        for (auto& pid : pid_wheels) {
            pid.reset();
        }
    }

    __attribute__((always_inline)) inline bool target_changed_for_point_mode(const Pose2D& target) {
        return std::abs(target.x - last_control_target.x) > 0.01f ||
               std::abs(target.y - last_control_target.y) > 0.01f ||
               std::abs(target.yaw - last_control_target.yaw) > 0.01f;
    }

    __attribute__((always_inline)) inline void guard_motion_residue(const App::RobotState& state) {
        const auto& ctrl = state.control;
        if (!control_history_ready) {
            control_history_ready = true;
            last_control_mode = ctrl.mode;
            last_tracker_state = ctrl.tracker_state;
            last_control_target = ctrl.current_target;
            return;
        }

        bool mode_changed = (ctrl.mode != last_control_mode);
        bool auto_tracking_started =
            ctrl.mode == ControlMode::AUTO_TRACKING &&
            ctrl.tracker_state == TrackerState::TRACKING &&
            last_tracker_state != TrackerState::TRACKING;
        bool point_target_changed =
            ctrl.mode == ControlMode::POINT_TRACKING &&
            target_changed_for_point_mode(ctrl.current_target);

        if (mode_changed || auto_tracking_started || point_target_changed) {
            reset_motion_residue();
        }

        last_control_mode = ctrl.mode;
        last_tracker_state = ctrl.tracker_state;
        last_control_target = ctrl.current_target;
    }

    // 检查位姿是否明显跑出地图边界（允许有一定容错，防止里程计轻微抖动误触发）
    __attribute__((always_inline)) inline bool is_pose_outside_field(const Pose2D& pose) {
        // 自动循迹时若里程计明显跑出地图，立即进入错误态防止继续冲出场地
        constexpr float FIELD_MARGIN_CM = 5.0f;
        constexpr float FIELD_MAX_X_CM = SystemConfig::MAP_MAX_WIDTH * SystemConfig::GRID_SIZE_CM;
        constexpr float FIELD_MAX_Y_CM = SystemConfig::MAP_MAX_HEIGHT * SystemConfig::GRID_SIZE_CM;

        return pose.x < -FIELD_MARGIN_CM || pose.x > FIELD_MAX_X_CM + FIELD_MARGIN_CM ||
                pose.y < -FIELD_MARGIN_CM || pose.y > FIELD_MAX_Y_CM + FIELD_MARGIN_CM;
    }

    // 当检测到明显的边界错误时，立即停止底盘并切换到 POINT_TRACKING 模式锁死当前位置，同时通知 GameManager 进入错误状态
    __attribute__((always_inline)) inline void stop_on_boundary_error(const Pose2D& pose) {
        auto& ctrl = App::g_state.control;
        // 目标锁在当前位置，先让底盘停止，再把错误交给 GameManage 处理
        ctrl.current_target.x = pose.x;
        ctrl.current_target.y = pose.y;
        ctrl.current_target.yaw = pose.yaw;
        ctrl.mode = ControlMode::POINT_TRACKING;
        ctrl.tracker_state = TrackerState::FINISHED;
        App::g_state.game.error_stage = 9;
        App::g_state.game.phase = GamePhase::ERROR_OCCURRED;
    }

    // 慢环(20ms)末尾发布：把"当前内插值→新目标"的差分成 N=4 拍，交给快环逐拍线性逼近，
    // 消除 20ms 台阶对速度环的周期性激励（低速刹车微颤根因）。速度与加速度目标一同内插。
    __attribute__((always_inline)) inline void publish_wheel_target_interp() {
        constexpr float N = static_cast<float>(SystemConfig::PIT_CH1_PERIOD_MS) /
                            static_cast<float>(SystemConfig::SPEED_LOOP_PERIOD_MS);  // 20/5 = 4
        s_speed_step.lf = (s_target_wheel_speeds.lf - s_interp_speed.lf) / N;
        s_speed_step.lb = (s_target_wheel_speeds.lb - s_interp_speed.lb) / N;
        s_speed_step.rf = (s_target_wheel_speeds.rf - s_interp_speed.rf) / N;
        s_speed_step.rb = (s_target_wheel_speeds.rb - s_interp_speed.rb) / N;
        s_accel_step.lf = (s_target_wheel_accels.lf - s_interp_accel.lf) / N;
        s_accel_step.lb = (s_target_wheel_accels.lb - s_interp_accel.lb) / N;
        s_accel_step.rf = (s_target_wheel_accels.rf - s_interp_accel.rf) / N;
        s_accel_step.rb = (s_target_wheel_accels.rb - s_interp_accel.rb) / N;
        s_interp_ticks_left = static_cast<uint8_t>(N);
    }

    // 速度内环控制（从 Branch 搬入：每轮独立 PID + 前馈）：输入四轮目标转速，逐轮
    // 前馈(kv + ks + ka/kb·目标加速度) + 独立 PID(积分门控) 出占空比。零速死区改为松力滑行，
    // 无主动制动锁定。目标速度/加速度均经 20ms→5ms 内插喂入，起步弹射/刹车稳停更脆更平顺。
    __attribute__((always_inline)) inline void run_speed_loop(const WheelSpeed4& targets) {
        const auto& current_speeds = App::g_state.physical.current_wheel_speed;
        const float max_duty = tune.dynamics.max_duty;
        const float dt = SystemConfig::SPEED_LOOP_DT_S;

        // 零速松力滑行门限（Branch 值）：目标、实测、目标加速度均低于此才关输出，避免 ks 放大微小
        // 爬行目标；加了加速度门限，起步(目标小但加速度大)不会被误判成零速滑掉起步。
        constexpr float WHEEL_ZERO_TARGET_CM_S = 0.8f;
        constexpr float WHEEL_ZERO_ACCEL_CM_S2 = 8.0f;

        const float tgt[4] = {targets.lf, targets.lb, targets.rf, targets.rb};
        const float cur[4] = {current_speeds.lf, current_speeds.lb, current_speeds.rf, current_speeds.rb};
        const float acc[4] = {s_interp_accel.lf, s_interp_accel.lb,
                              s_interp_accel.rf, s_interp_accel.rb};

        for (int i = 0; i < 4; ++i) {
            // 零速死区：目标、实测、目标加速度都≈0 → 松力滑行(输出0)。注意这不是"没到点提前停"——
            // 减速全程 PID 仍跟着目标主动刹（无 brake_ff 锁定），只有真正停到零附近才滑。
            // 与 Branch 一致：此处只关输出、不清 PID 历史（真正停稳时 calculate 内部会自清）。
            if (std::abs(tgt[i]) < WHEEL_ZERO_TARGET_CM_S &&
                std::abs(cur[i]) < WHEEL_ZERO_TARGET_CM_S &&
                std::abs(acc[i]) < WHEEL_ZERO_ACCEL_CM_S2) {
                motors[i].set_duty(0.0f, max_duty);
                continue;
            }

            const WheelControlParams& p = tune.wheels[i];
            float ff  = wheel_feedforward(p, tgt[i], cur[i], acc[i]);
            float pid = pid_wheels[i].calculate(tgt[i], cur[i], acc[i], dt);
            motors[i].set_duty(ff + pid, max_duty);
        }
    }
}

// ====================================================================
// 对外公开接口实现
// ====================================================================

void init() {
    for(auto& m : motors) m.init();    // 电机初始化 (gpio + pwm)
    encoders.init();                   // 编码器初始化 (encoder)
}

/// \brief 20ms 底盘控制周期
///
/// \details
/// 根据当前控制模式选择 Tracker 目标或上位机目标，完成速度规划、坐标转换、Yaw 控制和四轮速度闭环
/// 自动循迹时会检查地图边界，并在非终点航点向速度规划器传入过弯末速度
///
__attribute__((section(".ramfunc"))) void update_20ms_tick() {
    auto& posi = App::g_state.physical.pose;
    auto& yaw = App::g_state.physical.pose.yaw;
    auto& ctrl = App::g_state.control;

    if (ctrl.mode == ControlMode::AUTO_TRACKING && is_pose_outside_field(posi)) {
        stop_on_boundary_error(posi);
    }

    // 根据模式决定是否听 Tracker 的话
    // 如果是 POINT_TRACKING，我们就不调用 update_target，直接用上位机写入 ctrl.current_target 的值。
    // 视觉修正（含 POINT_TRACKING 保持）已移到 PIT_CH2 的 15ms 节拍 vision_correction_tick，此处不再触发。
    if (ctrl.mode == ControlMode::AUTO_TRACKING && ctrl.tracker_state == TrackerState::TRACKING) {
        Algorithm::Tracker::update_target();
    }

    guard_motion_residue(App::g_state);

    // 硬锁：到达停车航点半径内后 Tracker 置位 hard_lock。此时直接把四轮目标速度清零，
    // 不再经过 path_follower / yaw 规划 / 逆运动学，靠 5ms 速度环 PID 顶在 0 主动刹停锁死轮胎。
    // 同步清掉规划器与偏航控制器的残留速度，解锁进入下一段时从零平滑起步。
    if (ctrl.hard_lock) {
        path_follower.reset();
        yaw_controller.reset();
        s_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        s_target_wheel_accels = {0.0f, 0.0f, 0.0f, 0.0f};
        s_prev_target_wheel_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        // 目标已置 0，仍走内插把当前速度在 4 拍内线性铺到 0（比直接阶跃到 0 更平顺，减少刹停微颤），
        // PID 全程主动刹，停得依旧干脆。
        publish_wheel_target_interp();
        return;
    }

    float err_yaw = normalize_angle(ctrl.current_target.yaw - yaw);

    // 过弯不停顿：末速度由 Tracker 按"当前航点是否需要停稳"写入。
    // 拐点（非终点、非强停）给非零保留速度让车直接带速切向；需停稳处为 0 按停车规划。
    float target_end_speed = ctrl.segment_end_speed;

    // 沿路径线跟踪 + Stanley 横向纠偏：贴着 segment_start→current_target 这条线走，
    // 而不是只朝目标点收敛。段长过短或 Stanley 关闭时内部自动退化为纯朝点。
    Speed2D expected_global_vel = path_follower.follow(
        posi.x, posi.y,
        ctrl.segment_start.x, ctrl.segment_start.y,
        ctrl.current_target.x, ctrl.current_target.y,
        SystemConfig::PIT_CH1_DT_S, target_end_speed);

    // 把规划器输出的全局期望速度暴露给延时补偿：它是指令、不含打滑，
    // 末尾刹车时趋近 0，用来给编码器外推位移封顶，挡掉打滑虚增的过冲。
    ctrl.commanded_vel = expected_global_vel;

    // 将全局期望速度投影到小车自身的局部坐标系
    float current_yaw_rad = yaw * SystemConfig::DEG_TO_RAD;  // 转换为弧度
    float cos_theta = cosf(current_yaw_rad);
    float sin_theta = sinf(current_yaw_rad);
    float expected_local_vx = expected_global_vel.vx * sin_theta - expected_global_vel.vy * cos_theta;
    float expected_local_vy = expected_global_vel.vx * cos_theta + expected_global_vel.vy * sin_theta;


    // Yaw 角速度规划：三层规划(sqrt远端/线性近端)叠加 IMU 陀螺阻尼，抑制冲过/回摆
    bool is_translating = (std::abs(expected_local_vx) > 2.0f || std::abs(expected_local_vy) > 2.0f);
    float expected_local_vw = yaw_controller.calculate(err_yaw, SystemConfig::PIT_CH1_DT_S, is_translating, App::g_state.physical.yaw_rate);


    // 逆运动学解算：将期望的底盘全向速度分配给 4 个轮子，得到每个轮子的目标转速 (v1, v2, v3, v4)
    // 只产出目标速度，交给 5ms 快环闭环；本慢环不再直接驱动电机。
    s_target_wheel_speeds = Algorithm::Motion::Kinematics::inverse(expected_local_vx, expected_local_vy, expected_local_vw);

    // 每轮目标加速度 = 相邻两拍目标轮速之差 / dt，供 5ms 轮速环的 ka/kb 加速度前馈：
    // 起步时前喂 ka·a → 弹射更脆(消 PID 追斜坡的滞后)；刹车时前喂 kb·a → 稳停更利落。
    // 限幅到物理上限，避免切段/yaw 纠偏引起目标突变时产生前馈尖峰。
    {
        const float dt = SystemConfig::PIT_CH1_DT_S;
        const float max_wheel_acc = tune.dynamics.max_acc +
            tune.dynamics.max_ang_acc * Algorithm::Motion::Kinematics::L;
        auto acc_of = [&](float now, float prev) {
            return std::clamp((now - prev) / dt, -max_wheel_acc, max_wheel_acc);
        };
        s_target_wheel_accels.lf = acc_of(s_target_wheel_speeds.lf, s_prev_target_wheel_speeds.lf);
        s_target_wheel_accels.lb = acc_of(s_target_wheel_speeds.lb, s_prev_target_wheel_speeds.lb);
        s_target_wheel_accels.rf = acc_of(s_target_wheel_speeds.rf, s_prev_target_wheel_speeds.rf);
        s_target_wheel_accels.rb = acc_of(s_target_wheel_speeds.rb, s_prev_target_wheel_speeds.rb);
        s_prev_target_wheel_speeds = s_target_wheel_speeds;
    }

    // 发布 20ms→5ms 内插步长：快环 4 拍线性铺到新目标，消除台阶激励（低速刹车微颤）。
    publish_wheel_target_interp();
}


/// \brief 5ms 轮速内环（200Hz 快环）
///
/// \details
/// 消费慢环(20ms)产出目标，经 20ms→5ms 线性内插后配合 5ms 新测轮速跑 PID+前馈出占空比。
/// 内插消除 20ms 台阶对速度环的周期性激励（低速刹车"微颤"根因），同时内环 200Hz 紧跟目标。
///
__attribute__((section(".ramfunc"))) void update_speed_loop_5ms() {
    if (s_interp_ticks_left > 0) {
        s_interp_speed.lf += s_speed_step.lf;  s_interp_speed.lb += s_speed_step.lb;
        s_interp_speed.rf += s_speed_step.rf;  s_interp_speed.rb += s_speed_step.rb;
        s_interp_accel.lf += s_accel_step.lf;  s_interp_accel.lb += s_accel_step.lb;
        s_interp_accel.rf += s_accel_step.rf;  s_interp_accel.rb += s_accel_step.rb;
        --s_interp_ticks_left;
    } else {
        // 内插拍数用尽：锁定到慢环最新目标，等下一次 publish 再铺
        s_interp_speed = s_target_wheel_speeds;
        s_interp_accel = s_target_wheel_accels;
    }
    run_speed_loop(s_interp_speed);
}


/// \brief 更新底盘静止状态
///
/// \details
/// 只根据四轮反馈速度判断是否停稳，供 Tracker 终点完成判定和遥测运动计时使用
///
__attribute__((section(".ramfunc"))) void check_is_stopped() {

    const auto& cur_spd = App::g_state.physical.current_wheel_speed;
    const auto& target_spd = s_target_wheel_speeds;
    constexpr float STOPPED_WHEEL_SPEED_EPS = 1.5f;
    constexpr float STOPPED_TARGET_SPEED_EPS = 1.5f;
        
    // 判定条件：上一帧的控制目标几乎为0，且当前四个轮子的真实反馈速度极小
    App::g_state.physical.is_stopped = 
        (std::abs(target_spd.lf) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(target_spd.lb) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(target_spd.rf) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(target_spd.rb) < STOPPED_TARGET_SPEED_EPS &&
         std::abs(cur_spd.lf) < STOPPED_WHEEL_SPEED_EPS &&
         std::abs(cur_spd.lb) < STOPPED_WHEEL_SPEED_EPS &&
         std::abs(cur_spd.rf) < STOPPED_WHEEL_SPEED_EPS &&
         std::abs(cur_spd.rb) < STOPPED_WHEEL_SPEED_EPS);
}

}
