#include "zf_common_headfile.h"
#include "motor.h"

Motor::Motor(int dir_pin, int pwm_channel, bool is_inverted)
    : dir_pin_(dir_pin), pwm_channel_(pwm_channel), is_inverted_(is_inverted) {}

void Motor::init() {
    gpio_init((gpio_pin_enum)dir_pin_, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init((pwm_channel_enum)pwm_channel_, 17000, 0); 
}


// 设置电机占空比，范围 -100.0f ~ 100.0f
__attribute__((section(".ramfunc"))) void Motor::set_duty(float duty) {
    if (duty > 50.0f) duty = 50.0f;
    if (duty < -50.0f) duty = -50.0f;
    if (is_inverted_) duty = -duty;

    if (duty >= 0.0f) {
        gpio_set_level((gpio_pin_enum)dir_pin_, GPIO_HIGH); // 正转
        pwm_set_duty((pwm_channel_enum)pwm_channel_, (uint32)(duty / 100.0f * PWM_DUTY_MAX));
    } else {
        gpio_set_level((gpio_pin_enum)dir_pin_, GPIO_LOW); // 反转
        pwm_set_duty((pwm_channel_enum)pwm_channel_, (uint32)(-duty / 100.0f * PWM_DUTY_MAX));
    }
}