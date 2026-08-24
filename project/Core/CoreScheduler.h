/// \file CoreScheduler.h
/// \brief 主循环周期任务调度接口
///
/// \details
/// 声明软件调度器的初始化、轮询执行和全局毫秒时间戳查询接口

#pragma once
#include <cstdint>

namespace Core::Scheduler {
    void init();
    void run();
    
    // 获取系统的全局 1ms 绝对时间戳
    uint32_t get_sys_tick_ms(); 
}
