#pragma once

namespace Subsystem::Chassis {
    void init();
    void update_20ms_tick();  // 放到 20ms 定时器中断里
    void update_20ms_tick_debug();
}