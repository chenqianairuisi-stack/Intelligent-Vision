#pragma once
#include "system_config.h"
#include "tuning_config.h"
#include <cmath>
#include <algorithm>

namespace Algorithm::Motion {

// ==========================================
// 1. 运动学库 (纯数学 inline，零运行时开销)
// ==========================================
namespace Kinematics {
    constexpr float L = SystemConfig::HALF_X_AXIS + SystemConfig::HALF_Y_AXIS;

    __attribute__((always_inline))
    inline WheelSpeed4 inverse(float vx, float vy, float vw) noexcept {
        return {
            vy + vx - vw * L,  // LF
            vy - vx - vw * L,  // LB
            vy - vx + vw * L,  // RF
            vy + vx + vw * L   // RB
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
// 2. PID 算法库 (Header-only 极速内联)
// ==========================================

// 增量式 PID (适合速度内环)
class IncPid {
public:
    IncPid(const PidParams& p) : params(p) {}
    
    __attribute__((always_inline)) 
    inline float calculate(float target, float current) {
        float err = target - current;
        float p_term = params.kp * (err - last_err);
        float i_term = params.ki * err;
        float d_term = params.kd * (err - 2.0f * last_err + prev_err);
        
        // 防止积分项过大导致的风车效应
        if (i_term > 2.0f) i_term = 2.0f;
        if (i_term < -2.0f) i_term = -2.0f;
        if (out >= 50.0f && i_term > 0.0f) {
            i_term = 0.0f;
        } else if (out <= -50.0f && i_term < 0.0f) {
            i_term = 0.0f;
        }

        prev_err = last_err;
        last_err = err;
        
        out += (p_term + i_term + d_term);
        return out;
    }
private:
    const PidParams& params;
    float last_err = 0.0f, prev_err = 0.0f, out = 0.0f;
};

// 位置式 PID (适合角度外环)
class PosPid { 
public:
    PosPid(const PidParams& p) : params(p) {}
    
    __attribute__((always_inline))
    inline float calculate(float target, float current) {
        float err = target - current;
        i_out += params.ki * err;  // ki为 0，可忽略
        float d_term = params.kd * (err - last_err);
        last_err = err;
        return (params.kp * err) + i_out + d_term;
    }
private:
    const PidParams& params;
    float last_err = 0.0f, i_out = 0.0f;
};

// ==========================================
// 3. 轨迹规划器 (因为逻辑较长，实现在 cpp 中)
// ==========================================
class Trajectory {
public:
    inline void reset() { current_v = 0.0f; current_a = 0.0f; }  
    Speed2D velocity_planning_1d(float dx, float dy, float dt);  // 根据当前距离，规划最优速度
private:
    float current_v = 0.0f;  // 当前速度
    float current_a = 0.0f;  // 当前加速度
};

} // namespace Algorithm::Motion
