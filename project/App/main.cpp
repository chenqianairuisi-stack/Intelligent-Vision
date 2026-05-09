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
    interrupt_set_priority(PIT_IRQn, 0);    
    interrupt_global_enable(0);
    

    while(1) {
        Subsystem::Vision::update(); 
        Core::Scheduler::run();      

        App::GameEngine::update();
    }
}