#pragma once

namespace Subsystem::Chassis {
    void init();

    void update_20ms_tick();  // PIT_CH1 定时器调用，执行底盘控制算法更新

    void check_is_stopped();
}