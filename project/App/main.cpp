#include "zf_common_headfile.h"
#include "code_headfile.h"

extern "C" int main(void) {

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);           

    if (!imu_sensor.init()) { 
        sys_menu.halt_with_error("IMU Initialization Failed");
    }                                         // IMU 初始化 (imu)
    encoders.init();                          // 编码器初始化 (encoder)
    Storage::init();                          // 存储模块初始化，加载参数 (flash)
    telemetry.init();                         // 通信模块初始化 (wireless_uart)
    sys_menu.init();                          // 系统菜单初始化 (tft180)
    vision_manager.init();                    // 视觉模块初始化 (uart)
    chassis_task.init();                      // 控制模块初始化 (motor)
    scheduler.init();                         // 任务调度器初始化 (timer)

    pit_ms_init(PIT_CH0, 5);                  // 初始化 PIT_CH0 为 5ms 周期中断
    pit_ms_init(PIT_CH1, 20);                 // 初始化 PIT_CH1 为 20ms 周期中断
    interrupt_set_priority(PIT_IRQn, 0);      // 设置 PIT1 优先级为 0
    interrupt_global_enable(0);               // 全局使能中断


    // vision_manager.request_map_ART1();
    // game_manager.set_phase(GamePhase::WAIT_FOR_VISION);


    while(1) {
        vision_manager.update();             // 视觉地图/位置解析
        debug_manager.update();              // 游戏状态机更新（包含动画演示）
        // game_manager.update();               // 游戏状态机更新

        scheduler.run();
    }
}