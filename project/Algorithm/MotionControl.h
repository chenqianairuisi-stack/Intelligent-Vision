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
    Speed2D velocity_planning_1d(float dx, float dy, float dt, float end_speed = 0.0f);
private:
    float current_v = 0.0f;
    float current_a = 0.0f;
};


class YawProfiled {
public:
    void reset() { current_vw = 0.0f; settled = false; }
    float calculate(float err, float dt, bool is_translating);

private:
    float current_vw = 0.0f;
    bool settled = false;   // 死区迟滞锁存：进死区判稳后，需误差涨过再触发阈值才重新给力
};


// ==========================================
// 1b. Stanley 式平移横向纠偏（贴路径线跟踪）
// ==========================================
// 把"朝目标点收敛"改成"沿路径线行驶 + 横向 Stanley 纠偏"：
// 沿轨方向用梯形规划器出主速度，垂直方向用 Stanley atan 律压回路径线。
// 全向底盘下横纠表现为垂直于段方向的平移速度分量，不改逆运动学。
class PathLineFollower {
public:
    inline void reset() { planner.reset(); }

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
    Trajectory planner;
};


// ==========================================
// 2. 运动学库
// ==========================================
namespace Kinematics {
    constexpr float L = SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS;

    __attribute__((always_inline))
    inline WheelSpeed4 inverse(float vx, float vy, float vw) noexcept {
        float vx_compensated = vx * tune.dynamics.kinematic_gain_x;
        float vy_compensated = vy * tune.dynamics.kinematic_gain_y; 
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

// 速度环 PID
class Speed_PosPid {
public:
    Speed_PosPid(const PidParams& p) : params(p) {}
    
    void reset() { i_out = 0.0f;}  // 断电/急停/初始化时，必须调用此函数清零历史状态

    __attribute__((always_inline))
    inline float calculate(float target, float current) {
        if (std::abs(target) < 0.1f && std::abs(current) < 0.5f) {
            i_out = 0.0f;
            return 0.0f;
        }

        float err = target - current;

        // 积分按快环节拍缩放，保持原 20ms 整定的 ki 等效（快环每秒调用更多次）
        i_out += params.ki * err * SystemConfig::SPEED_LOOP_KI_SCALE;

        // 积分抗饱和
        i_out = std::clamp(i_out, -20.0f, 20.0f);

        // 最终返回 PI 补偿占空比
        return (params.kp * err) + i_out;
    }

private:
    const PidParams& params;
    float i_out = 0.0f;
};

} // namespace Algorithm::Motion
