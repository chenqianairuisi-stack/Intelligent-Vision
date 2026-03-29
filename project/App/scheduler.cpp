#include "scheduler.h"
#include "task_control.h"
#include "display.h"
#include "telemetry.h"
#include "task_vision.h"
#include "game_manage.h"
#include "zf_common_headfile.h"

TaskScheduler scheduler;

// ================= 全局任务注册表 =================
Task TaskScheduler::task_table[] = {
    // 上位机指令解析 (高频，10ms)
    {[]{telemetry.receive_and_parse_task();}, 10, 0 },  

    // 上位机波形发送 (中频，20ms)
    {[]{ telemetry.send_wave_data(); }, 20, 0 },

    // TFT UI 渲染 (低频，100ms)
    {[]{ sys_menu.run(); }, 100, 0 }
};

const uint8_t TaskScheduler::TASK_COUNT = sizeof(task_table) / sizeof(task_table[0]);

void TaskScheduler::init() {
    // 初始化一个全局 1ms 硬件定时器 (比如 GPT_TIM_1)
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
}

uint32_t TaskScheduler::get_sys_tick_ms() {
    return timer_get(GPT_TIM_1); 
}

// 极其高效的非阻塞调度器核心
__attribute__((section(".ramfunc"))) void TaskScheduler::run() {
    uint32_t current_time = get_sys_tick_ms();
    
    for (uint8_t i = 0; i < TASK_COUNT; ++i) {
        // 如果当前时间 - 上次执行时间 >= 设定的周期，则触发该任务
        if (current_time - task_table[i].last_run_ms >= task_table[i].interval_ms) {
            task_table[i].last_run_ms = current_time;  // 更新时间戳
            task_table[i].task_func();                 // 执行任务
        }
    }
}