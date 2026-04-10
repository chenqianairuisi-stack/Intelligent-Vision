#pragma once
#include <cstdint>

namespace Core::Scheduler {
    void init();
    void run();
    
    // 获取系统的全局 1ms 绝对时间戳
    uint32_t get_sys_tick_ms(); 
}