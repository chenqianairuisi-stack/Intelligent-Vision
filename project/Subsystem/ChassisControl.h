#pragma once

namespace Subsystem::Chassis {
    void init();

    void update_20ms_tick();  // PIT_CH1(20ms) 慢环：规划/Tracker/yaw/逆运动学，产出四轮目标速度

    void update_speed_loop_5ms();  // 5ms 快环(200Hz)：测速后跑轮速 PID 出占空比驱动电机

    void check_is_stopped();
}