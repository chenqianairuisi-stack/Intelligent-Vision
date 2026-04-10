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
#include "Icm42688.h"


extern "C" int main(void) {

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);           
    
    Core::Scheduler::init();                  // 任务调度器初始化 (timer)
    Subsystem::Telemetry::init();             // 通信模块初始化 (wireless_uart)
    Subsystem::PoseEstimator::init();         // 定位模块初始化 (imu)
    Subsystem::Vision::init();                // 视觉模块初始化 (uart)
    Subsystem::Chassis::init();               // 控制模块初始化 (motor, encoder)

    // Storage::init();                          // 存储模块初始化，加载参数 (flash)
    sys_menu.init();                          // 系统菜单初始化 (tft180)
    debug_manager.init();                     // 游戏管理器初始化 (读取拨码开关，设置初始阶段)
    debug_manager.inject_mock_semantics();    // 注入虚拟视觉标签，供没有摄像头时的调试使用

    // IMU 开机静态标定
    system_delay_ms(500);
    Subsystem::PoseEstimator::calibrate_gyro_step();

    pit_ms_init(PIT_CH0, 5);                 
    pit_ms_init(PIT_CH1, 20);               
    interrupt_set_priority(PIT_IRQn, 0);    
    interrupt_global_enable(0);
    

    while(1) {
        Subsystem::Vision::update(); 
        Core::Scheduler::run();      

        if (App::g_state.game.is_debug_mode) {
            debug_manager.update();              // 调试模式：进入拦截器，执行动画逻辑
        } else {
            debug_manager.GameManager::update(); // 正赛模式：直接穿透到基类，执行纯物理/控制逻辑
        }
    }
}