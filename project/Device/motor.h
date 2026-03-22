#pragma once

class Motor {
public:
    Motor(int dir_pin, int pwm_channel, bool is_inverted);
    
    void init();
    void set_duty(float duty); // -100.0f ~ 100.0f
    
private:
    int dir_pin_;             // GPIO 输出方向控制
    int pwm_channel_;         // PWM 输出通道
    bool is_inverted_;        // 是否反转（根据电机接线决定）
};