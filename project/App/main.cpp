/// \file main.cpp
/// \brief RT1064 固件启动入口
///
/// \details
/// 完成时钟、定位、底盘、视觉、通信、显示、参数存储和游戏管理器初始化
/// 随后启动周期中断，并在主循环中持续推进视觉、软件任务和游戏状态机

#include "zf_common_headfile.h"
#include "CoreScheduler.h"
#include "RobotState.h"
#include "GameManage.h"
#include "Vision.h"
#include "ChassisControl.h"
#include "Telemetry.h"
#include "Display.h"
#include "PoseEstimate.h"
#include "Storage.h"
#include "system_config.h"


extern "C" int main(void) {

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);           
    
    Subsystem::PoseEstimator::init();         // 定位模块初始化 (imu)
    Subsystem::Chassis::init();               // 控制模块初始化 (motor, encoder)
    Subsystem::Vision::init();                // 视觉模块初始化 (uart)
    Subsystem::Telemetry::init();             // 通信模块初始化 (wireless_uart)
    Subsystem::Display::init();               // 系统菜单初始化 (tft180) 
    
    Storage::init();                          // 存储模块初始化，加载参数 (flash)
    Core::Scheduler::init();                  // 任务调度器初始化 (timer)

    App::GameEngine::init();                  // 游戏管理器初始化 (读取拨码开关，设置初始阶段)

    // IMU 开机静态标定
    Subsystem::PoseEstimator::calibrate_gyro_step();

    pit_ms_init(PIT_CH0, SystemConfig::PIT_CH0_PERIOD_MS);
    pit_ms_init(PIT_CH1, SystemConfig::PIT_CH1_PERIOD_MS);
    pit_ms_init(PIT_CH2, SystemConfig::PIT_CH2_PERIOD_MS);   // 视觉修正专用 15ms 通道（与 CH0/CH1 共用 PIT_IRQ，串行免锁）
    interrupt_set_priority(PIT_IRQn, 0);
    interrupt_global_enable(0);
    

    while(1) {
        Subsystem::Vision::update(); 
        Core::Scheduler::run();      

        App::GameEngine::update();
    }
}
