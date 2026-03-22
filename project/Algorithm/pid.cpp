# include "pid.h"


__attribute__((section(".ramfunc"))) float IncPid::calculate(float target, float current) {
    float error = target - current;
    float p = params.kp * (error - last_error);
    float i = params.ki * error;
    float d = params.kd * (error - 2.0f * last_error + prev_error);
        
    prev_error = last_error;
    last_error = error;
        
    output += (p + i + d);

    return output;
}



__attribute__((section(".ramfunc"))) float PosPid::calculate(float target, float current) {
    float error = target - current;
    sum_error += error;
    float d_error = error - last_error;
    last_error = error;
        
    return params.kp * error + params.ki * sum_error + params.kd * d_error;
}