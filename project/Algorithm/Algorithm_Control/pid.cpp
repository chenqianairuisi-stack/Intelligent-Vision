# include "pid.h"
#include <cmath>

// 增量式 PID 计算函数
__attribute__((section(".ramfunc"))) float IncPid::calculate(float target, float current) {

    if (std::abs(target) < 0.1f && std::abs(current) < 0.5f) {
        output = 0.0f;
        last_error = 0.0f;
        prev_error = 0.0f;
        return 0.0f;
    }

    float error = target - current;
    float p = params.kp * (error - last_error);
    float i = params.ki * error;
    float d = params.kd * (error - 2.0f * last_error + prev_error);
    
    const float MAX_I_STEP = 2.0f;  // 防止积分项过大导致的风车效应
    if (i > MAX_I_STEP) i = MAX_I_STEP;
    if (i < -MAX_I_STEP) i = -MAX_I_STEP;

    if (output >= 50.0f && i > 0.0f) {
        i = 0.0f; 
    } else if (output <= -50.0f && i < 0.0f) {
        i = 0.0f;
    }

    prev_error = last_error;
    last_error = error;
        
    output += ( p + i + d );

    if (output > 50.0f) output = 50.0f;
    if (output < -50.0f) output = -50.0f;

    return output;
}


// 位置式 PID 计算函数
__attribute__((section(".ramfunc"))) float PosPid::calculate(float target, float current) {
    float error = target - current;
    sum_error += error;
    float d_error = error - last_error;
    last_error = error;
        
    return params.kp * error + params.ki * sum_error + params.kd * d_error;
}