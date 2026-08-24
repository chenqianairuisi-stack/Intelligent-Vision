/// \file CoreScheduler.cpp
/// \brief 主循环软件任务调度器实现
///
/// \details
/// 基于 GPT 产生的毫秒绝对时基，按固定周期协作式调度遥测接收、波形发送和显示刷新
/// 调度器不在中断中执行业务任务，由主循环持续调用 run 推进

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
        {[]{ Subsystem::Display::run(); },                      100, 0 }    // TFT UI 渲染 (低频，100ms)
    };

    constexpr uint8_t TASK_COUNT = sizeof(task_table) / sizeof(task_table[0]);
}

/// \brief 初始化软件调度器的全局时基
///
/// \details
/// GPT_TIM_1 被配置为 1ms 计数源，run 中所有任务周期都基于该绝对时间戳判断
///
void init() {
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
}

/// \brief 获取系统 1ms 绝对时间戳
/// \return GPT_TIM_1 当前计数值，单位 ms
///
uint32_t get_sys_tick_ms() {
    return timer_get(GPT_TIM_1); 
}

/// \brief 软件任务调度器核心
///
/// \details
/// 每次主循环调用时遍历任务表，达到周期的任务立即执行
/// 本调度器不抢占，任务函数应保持短小，避免拖慢视觉串口和比赛状态机
///
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
