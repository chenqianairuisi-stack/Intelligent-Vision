/// \file isr.cpp
/// \brief RT1064 外设中断入口与实时任务分发
///
/// \details
/// 在 PIT 中断中按不同周期采集 IMU 和编码器并推进姿态、速度、底盘和跟踪控制
/// 其余 UART、GPIO 与 CSI 中断转发给对应驱动或设备模块处理

#include "zf_common_headfile.h"
#include "zf_common_debug.h"

#include "RobotState.h"
#include "ChassisControl.h"
#include "PoseEstimate.h"
#include "Tracker.h"

#include "UartComm.h"
#include "Encoder.h"
#include "Icm42688.h"


extern "C" void PIT_IRQHandler(void) {

    // PIT_CH0
    if(pit_flag_get(PIT_CH0)) 
    {
        pit_flag_clear(PIT_CH0);

        imu_icm42688.update_all();  

        Subsystem::PoseEstimator::update_yaw_1ms_tick();

        static uint8_t s_speed_loop_div = 0;
        if (++s_speed_loop_div >= SystemConfig::SPEED_LOOP_PERIOD_MS) {
            s_speed_loop_div = 0;
            encoders.update_speed_5ms_tick();
            Subsystem::Chassis::update_speed_loop_5ms();
        }
    }
    
    // PIT_CH1
    if(pit_flag_get(PIT_CH1)) 
    {
        pit_flag_clear(PIT_CH1);

        encoders.update_encoders_20ms_tick();
        Subsystem::Chassis::check_is_stopped(); 
        Subsystem::PoseEstimator::update_position_20ms_tick(encoders.getAllCounts(), App::g_state.physical.pose.yaw);
        Subsystem::Chassis::update_20ms_tick();      
    }
    
    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);

        Algorithm::Tracker::vision_correction_tick();
    }
    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}


extern "C" void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        uart_cam2.rxisr();
        

    // #if DEBUG_UART_USE_INTERRUPT                  
    //     debug_interrupr_handler();                         
    // #endif                                              
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);   
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {

    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    
}

extern "C" void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        uart_cam1.rxisr();

        // flexio_camera_uart_handler();
        // gnss_uart_callback();
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        camera_uart_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);
}


extern "C" void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        if(NULL != wireless_module_uart_handler)
        {
            wireless_module_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16);
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);
    }

}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    tof_module_exti_handler();
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16);
    }
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);
    }
}



void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    __DSB();                    
}
