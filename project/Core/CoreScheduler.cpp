#include "CoreScheduler.h"

#include "Display.h"
#include "Telemetry.h"

#include "zf_common_headfile.h"


namespace Core::Scheduler {

namespace { // 隐藏任务表实现
    struct Task {
        void (*task_func)();      
        uint32_t interval_ms;     
        uint32_t last_run_ms;     
    };

    // 全局任务注册表
    Task task_table[] = {
        {[]{ Subsystem::Telemetry::receive_and_parse_task(); }, 10,  0 },   // 上位机指令解析 (高频，10ms)
        {[]{ Subsystem::Telemetry::send_wave_data(); },         20,  0 },   // 上位机波形发送 (中频，20ms)
        {[]{ sys_menu.run(); },                                 100, 0 }    // TFT UI 渲染 (低频，100ms)
    };

    constexpr uint8_t TASK_COUNT = sizeof(task_table) / sizeof(task_table[0]);
}

// 初始化一个全局 1ms 硬件定时器 (比如 GPT_TIM_1)
void init() {
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
}

// 获取系统的全局 1ms 绝对时间戳
uint32_t get_sys_tick_ms() {
    return timer_get(GPT_TIM_1); 
}

// 任务调度器核心：遍历任务表，检查每个任务是否到达执行时间，如果是则调用对应函数并更新时间戳
__attribute__((section(".ramfunc")))
void run() {
    uint32_t current_time = get_sys_tick_ms();

    for (uint8_t i = 0; i < TASK_COUNT; ++i) {
        // 如果当前时间 - 上次执行时间 >= 设定的周期，则触发该任务
        if (current_time - task_table[i].last_run_ms >= task_table[i].interval_ms) {
            task_table[i].last_run_ms = current_time;   // 更新时间戳
            task_table[i].task_func();                  // 执行任务
        }
    }
}

} // namespace Core::Scheduler