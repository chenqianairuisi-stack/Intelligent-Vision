#pragma once
#include "tuning_config.h"

// 增量式 PID (适合速度内环)
class IncPid {
public:
    IncPid(const PidParams& p) : params(p), last_error(0.0f), prev_error(0.0f), output(0.0f) {}
    float calculate(float target, float current);

private:
    const PidParams& params;
    float last_error;
    float prev_error;
    float output;
};


// 位置式 PID (适合角度外环)
class PosPid {
public:
    PosPid(const PidParams& p) : params(p), sum_error(0.0f), last_error(0.0f) {}
    float calculate(float target, float current);

private:
    const PidParams& params;
    float last_error;
    float sum_error;
};