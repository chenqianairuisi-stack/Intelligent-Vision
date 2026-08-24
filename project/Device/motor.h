/// \file motor.h
/// \brief 单路直流电机硬件封装
///
/// \details
/// 保存方向引脚、PWM 通道和接线反转配置，向底盘层提供初始化与占空比控制接口

#pragma once

class Motor {
public:
    Motor(int dir_pin, int pwm_channel, bool is_inverted);
    
    void init();
    void set_duty(float duty, float max_duty);  // -100.0f ~ 100.0f
    
private:
    int dir_pin_;               // GPIO 输出方向控制
    int pwm_channel_;           // PWM 输出通道
    bool is_inverted_;          // 是否反转（根据电机接线决定）
};
