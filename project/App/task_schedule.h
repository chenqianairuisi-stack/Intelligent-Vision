#pragma once
#include <cstdint>

struct Task {
    void (*task_func)();      // 任务执行的函数指针
    uint32_t interval_ms;     // 任务执行周期
    uint32_t last_run_ms;     // 上次执行的时间戳
};

class TaskScheduler {
public:
    void init();
    void run();

    // 获取系统的全局 1ms 绝对时间戳
    static uint32_t get_sys_tick_ms(); 

private:
    // 静态注册所有需要调度的任务
    static Task task_table[];
    static const uint8_t TASK_COUNT;
};

extern TaskScheduler scheduler;