#include "zf_common_headfile.h"
#include "code_headfile.h"

extern "C" int main(void) {

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);           

    imu_icm42688.init();                      // ICM42688 IMU 初始化 (spi)
    encoders.init();                          // 编码器初始化 (encoder)
    Storage::init();                          // 存储模块初始化，加载参数 (flash)
    telemetry.init();                         // 通信模块初始化 (wireless_uart)
    sys_menu.init();                          // 系统菜单初始化 (tft180)
    scheduler.init();                         // 任务调度器初始化 (timer)
    vision_manager.init();                    // 视觉模块初始化 (uart)
    chassis_task.init();                      // 控制模块初始化 (motor)
    // game_manager.init();                      // 管理模块初始化 (gpio)

    // IMU 开机静态标定，累计 500 次数据求平均，得到 gyro_z_offset
    system_delay_ms(500); // 等待imu芯片稳定
    while(!imu_sensor.calibrate_step()) {
        imu_icm42688.update_gyro_only();   // 主动提供数据给标定函数
        system_delay_ms(5);                // 模拟 5ms 等待
    }

    pit_ms_init(PIT_CH0, 5);                  // 初始化 PIT_CH0 为 5ms 周期中断
    pit_ms_init(PIT_CH1, 20);                 // 初始化 PIT_CH1 为 20ms 周期中断
    interrupt_set_priority(PIT_IRQn, 0);      // 设置 PIT1 优先级为 0
    interrupt_global_enable(0);               // 全局使能中断


    debug_manager.init();                      // 管理模块初始化 (gpio)
    debug_manager.inject_mock_semantics();     // 注入虚拟视觉标签，供没有摄像头时的调试使用

    while(1) {
        vision_manager.update();             // 视觉地图/位置解析
        debug_manager.update();              // 游戏状态机更新（包含动画演示）
        // game_manager.update();               // 游戏状态机更新

        scheduler.run();
    }
}