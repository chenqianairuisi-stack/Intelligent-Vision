#pragma once
#include "system_config.h"
#include "tuning_config.h"
#include <cmath>
#include <algorithm>

namespace Algorithm::Motion {

// ==========================================
// 1. 轨迹规划器
// ==========================================
class Trajectory {
public:
    inline void reset() { current_v = 0.0f; current_a = 0.0f; }
    // 生成平滑平移速度，end_speed 用于拐角不停顿通过
    // seg_len_cm：当前直线段全长 cm（由 follow 传入）。<=0 表示未知，不做短段加速提升；
    // 非长段且段长 <= tune.terminal.short_segment_cm 时只提高加速斜坡
    // 停车段末端曲线由 MotionFeatureSwitches 的两个开关选择：线性 / 非线性双段 sqrt / 都关=原始 sqrt。
    Speed2D velocity_planning_1d(float dx, float dy, float dt, float end_speed = 0.0f,
                                float seg_len_cm = -1.0f);
private:
    float current_v = 0.0f;
    float current_a = 0.0f;
};


class YawProfiled {
public:
    void reset() { current_vw = 0.0f; current_aw = 0.0f; }
    // err/yaw_rate 单位 rad、rad/s，yaw_rate 为 IMU 实测角速度，用于陀螺阻尼
    float calculate(float err, float dt, bool is_translating, float yaw_rate);
    // 获取最近一次角速度规划产生的角加速度（rad/s^2）
    float accel() const { return current_aw; }

private:
    float current_vw = 0.0f;
    float current_aw = 0.0f;
};


// ==========================================
// 1b. 前瞻式路径线跟踪
// ==========================================
// 沿路径线选择动态前瞻点，并按横向误差降速、拉回
// 二维速度方向使用 alpha=0.25 一阶滤波，切段时保留连续切向
class PathLineFollower {
public:
    inline void reset() {
        planner.reset();
        filtered_vel = {0.0f, 0.0f};
    }

    /// \brief 计算沿路径线跟踪的全局期望速度
    /// \param px,py 当前全局位置 cm
    /// \param sx,sy 当前直线段起点 cm
    /// \param tx,ty 当前段目标点 cm
    /// \param dt 控制周期 s
    /// \param end_speed 段末保留速度（拐点不停顿时>0）
    /// \return 全局期望速度 (vx,vy) cm/s
    Speed2D follow(float px, float py, float sx, float sy,
                   float tx, float ty, float dt, float end_speed = 0.0f);

private:
    Speed2D filter_direction(Speed2D desired, float speed_limit);
    Trajectory planner;
    Speed2D filtered_vel = {0.0f, 0.0f};
};


// ==========================================
// 2. 运动学库
// ==========================================
namespace Kinematics {
    constexpr float L = SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS;
    // 实车横移标定：左右横移每 200 cm 会串入约 10 cm 的反向前后位移
    inline constexpr float LATERAL_DRIFT_COMPENSATION = 0.00f;

    __attribute__((always_inline))
    inline WheelSpeed4 inverse(float vx, float vy, float vw) noexcept {
        float vx_compensated = vx * tune.dynamics.kinematic_gain_x;
        // 横移引起的前后串扰方向随 vx 反向，先在轮速层加入等量反向前后指令
        float vy_compensated = (vy + LATERAL_DRIFT_COMPENSATION * vx) *
                               tune.dynamics.kinematic_gain_y;
        return {
            vy_compensated + vx_compensated - vw * L,  // LF
            vy_compensated - vx_compensated - vw * L,  // LB
            vy_compensated - vx_compensated + vw * L,  // RF
            vy_compensated + vx_compensated + vw * L   // RB
        };
    }

    __attribute__((always_inline))
    inline Velocity2D forward(float v_lf, float v_lb, float v_rf, float v_rb) noexcept {
        return {
            ( v_lf - v_lb - v_rf + v_rb) / 4.0f,
            ( v_lf + v_lb + v_rf + v_rb) / 4.0f,
            (-v_lf - v_lb + v_rb + v_rf) / (4.0f * L)
        };
    }
}

// ==========================================
// 3. PID 算法库
// ==========================================

// 角度环 PID
class Angle_PosPid { 
public:
    Angle_PosPid(const PidParams& p) : params(p) {}
    
    __attribute__((always_inline))
    inline float calculate(float target, float current) {
        float err = target - current;  // 注意：这里的 err 是弧度
        
        // 积分分离 (Integral Separation):当偏差小于 10 度时，才允许 Ki 介入
        if (std::abs(err) < 0.17f) {
            i_out += params.ki * err;
            // 微调补偿限幅：只需要提供约 1.0 rad/s 的旋转补偿就足够克服摩擦力了
            i_out = std::clamp(i_out, -1.0f, 1.0f);
        } else {
            i_out = 0.0f; // 大偏差时，瞬间清空积分，防止回弹过猛
        }
        
        float d_term = params.kd * (err - last_err);
        last_err = err;
        return (params.kp * err) + i_out + d_term;

    }
private:
    const PidParams& params;
    float last_err = 0.0f, i_out = 0.0f;
};

// 速度环 PID（从 Branch 搬入：每轮独立整定）
// - 积分门控：大加减速段（|target_acc| 或 |err| 大）主要靠前馈跟随，积分退避（*0.98），
//   只在接近稳态时兜慢偏差，避免平台切入/刹停把加速段积累的历史误差释放成过冲。
// - 滤波微分：只阻尼"计划外加速度"(measured_accel - target_acc)，不抵消 ka/kb 前馈。
// calculate 需传目标加速度 target_acc 与 dt（阶段1 target_acc 传 0，ka/kb/微分自然不参与）。
class Speed_PosPid {
public:
    Speed_PosPid(const PidParams& p) : params(p) {}

    void reset() {  // 断电/急停/初始化时，必须调用此函数清零历史状态
        i_out = 0.0f;
        d_filt = 0.0f;
        last_current = 0.0f;
        has_last = false;
    }

    __attribute__((always_inline))
    inline float calculate(float target, float current, float target_acc, float dt) {
        if (std::abs(target) < 0.1f && std::abs(current) < 0.5f) {
            i_out = 0.0f;
            d_filt = 0.0f;
            has_last = false;
            return 0.0f;
        }

        float err = target - current;

        // 大加减速段主要靠前馈跟随，积分只在接近稳态时兜慢偏差
        constexpr float INTEGRAL_ACCEL_BAND = 80.0f;
        constexpr float INTEGRAL_ERR_BAND = 15.0f;
        if (std::abs(target_acc) < INTEGRAL_ACCEL_BAND && std::abs(err) < INTEGRAL_ERR_BAND) {
            i_out += params.ki * err * dt;
        } else {
            i_out *= 0.98f;
        }

        // 积分抗饱和
        i_out = std::clamp(i_out, -10.0f, 10.0f);

        float d_out = 0.0f;
        if (has_last && dt > 0.001f) {
            constexpr float D_FILTER_ALPHA = 0.35f;
            constexpr float D_ACCEL_DEADBAND = 8.0f;
            float measured_accel = (current - last_current) / dt;
            float accel_err = measured_accel - target_acc;
            if (std::abs(accel_err) < D_ACCEL_DEADBAND) {
                accel_err = 0.0f;
            }
            d_filt += D_FILTER_ALPHA * (accel_err - d_filt);
            d_out = -params.kd * d_filt;   // D 项只阻尼计划外加速度，避免抵消 ka/kb 前馈
        } else {
            d_filt = 0.0f;
            has_last = true;
        }
        last_current = current;

        return (params.kp * err) + i_out + d_out;
    }

private:
    const PidParams& params;
    float i_out = 0.0f;
    float d_filt = 0.0f;
    float last_current = 0.0f;
    bool has_last = false;
};

} // namespace Algorithm::Motion
