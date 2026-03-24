#include "zf_common_headfile.h"
#include "code_headfile.h"
#include "storage.h"

extern "C" int main(void) {

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);           

    // encoders.init();          // 编码器初始化 (encoder)
    if (!imu_sensor.init()) { sys_menu.halt_with_error("IMU Initialization Failed");}
    Storage::init();          // 存储模块初始化，加载参数 (storage)
    telemetry.init();         // 通信模块初始化 (telemetry)
    sys_menu.init();          // 系统菜单初始化 (display)

    vision_manager.init();    // 视觉模块初始化 (uart)
    chassis_task.init();      // 控制模块初始化 (motor)
    game_manager.init();      // 游戏管理器初始化

    pit_ms_init(PIT_CH0, 5);                  // 初始化 PIT_CH0 为 5ms 周期中断
    pit_ms_init(PIT_CH1, 20);                 // 初始化 PIT_CH1 为 20ms 周期中断
    interrupt_set_priority(PIT_IRQn, 0);      // 设置 PIT1 优先级为 0
    interrupt_global_enable(0);               // 全局使能中断

    while(1) {
        telemetry.receive_and_parse_task();   // 解析上位机命令
        telemetry.send_wave_data();           // 发送波形数据给上位机
        sys_menu.run();                       // 运行系统菜单
        // vision_manager.update();              // 解析通信数据
        // game_manager.update();                // 全局游戏状态轮询
        
    }
}