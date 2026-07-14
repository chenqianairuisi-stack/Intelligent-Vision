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

    // PIT_CH0 锟斤拷时锟斤拷锟叫断ｏ拷锟斤拷锟斤拷锟斤拷 SystemConfig::PIT_CH0_PERIOD_MS 锟斤拷锟矫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷讹拷取锟斤拷锟斤拷锟�
    if(pit_flag_get(PIT_CH0)) 
    {
        pit_flag_clear(PIT_CH0);

        // 锟斤拷锟斤拷锟斤拷锟斤拷锟捷讹拷取锟斤拷转锟斤拷
        imu_icm42688.update_all();  

        // yaw 锟斤拷嵌雀锟斤拷锟�
        Subsystem::PoseEstimator::update_yaw_1ms_tick();

        // 杞€熷唴鐜揩鐜細1ms 瀹氭椂鍣ㄥ垎棰戝埌 5ms(200Hz)锛屽厛娴嬮€熷啀璺戦€熷害 PID 鍑哄崰绌烘瘮
        static uint8_t s_speed_loop_div = 0;
        if (++s_speed_loop_div >= SystemConfig::SPEED_LOOP_PERIOD_MS) {
            s_speed_loop_div = 0;
            encoders.update_speed_5ms_tick();
            Subsystem::Chassis::update_speed_loop_5ms();
        }
    }
    
    // PIT_CH1 锟斤拷时锟斤拷锟叫断ｏ拷锟斤拷锟斤拷锟斤拷 SystemConfig::PIT_CH1_PERIOD_MS 锟斤拷锟矫ｏ拷锟斤拷锟斤拷锟节碉拷锟教匡拷锟斤拷锟姐法锟斤拷锟铰猴拷锟斤拷碳锟斤拷锟斤拷锟�
    if(pit_flag_get(PIT_CH1)) 
    {
        pit_flag_clear(PIT_CH1);

        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
        encoders.update_encoders_20ms_tick();

        // 锟斤拷锟铰碉拷锟斤拷锟角凤拷锟斤拷全停止锟斤拷状态
        Subsystem::Chassis::check_is_stopped(); 

        // 全锟街讹拷位锟斤拷碳锟斤拷锟斤拷锟� 
        Subsystem::PoseEstimator::update_position_20ms_tick(encoders.getAllCounts(), App::g_state.physical.pose.yaw);

        // 锟斤拷锟教匡拷锟斤拷锟姐法锟斤拷锟斤拷
        Subsystem::Chassis::update_20ms_tick();      
    }
    
    // PIT_CH2 瀹氭椂鍣ㄤ腑鏂細鍛ㄦ湡涓� SystemConfig::PIT_CH2_PERIOD_MS(15ms)锛屼笓璺戣瑙変綅濮夸慨姝ｃ€�
    // 蹇呴』鏀惧湪 CH1 鍒嗘敮涔嬪悗锛氬悓涓€娆′腑鏂噷鑻� 20ms 涔熷埌鏈燂紝鍏堣閲岀▼璁℃洿鏂� s_encoder_pose锛�
    // CH2 鍐嶆嬁鏈€鏂扮紪鐮佸櫒浣嶅Э鍘荤籂鍋忋€備笌 CH1 鍚屽睘涓€涓� PIT_IRQ銆佷覆琛屾墽琛岋紝鍐� pose.x/y 鍏嶉攣銆�
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
        // 锟斤拷锟斤拷锟叫断达拷锟斤拷锟斤拷锟斤拷锟斤拷 uart_cam1 锟斤拷锟叫断凤拷锟斤拷锟斤拷
        uart_cam2.rxisr();
        

    // #if DEBUG_UART_USE_INTERRUPT                        // 锟斤拷锟斤拷锟斤拷锟� debug 锟斤拷锟斤拷锟叫讹拷
    //     debug_interrupr_handler();                      // 锟斤拷锟斤拷 debug 锟斤拷锟节斤拷锟秸达拷锟斤拷锟斤拷锟斤拷 锟斤拷锟捷会被 debug 锟斤拷锟轿伙拷锟斤拷锟斤拷锟斤拷取
    // #endif                                              // 锟斤拷锟斤拷薷锟斤拷锟� DEBUG_UART_INDEX 锟斤拷锟斤拷未锟斤拷锟斤拷锟揭拷诺锟斤拷锟接︼拷拇锟斤拷锟斤拷卸锟饺�
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // 锟斤拷锟斤拷锟斤拷删锟斤拷
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 锟斤拷锟斤拷锟叫讹拷
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // 锟斤拷锟斤拷锟斤拷删锟斤拷
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 锟斤拷锟斤拷锟叫讹拷
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // 锟斤拷锟斤拷锟斤拷删锟斤拷
}

extern "C" void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 锟斤拷锟斤拷锟叫断达拷锟斤拷锟斤拷锟斤拷锟斤拷 uart_cam2 锟斤拷锟叫断凤拷锟斤拷锟斤拷
        uart_cam1.rxisr();

        // 锟斤拷锟斤拷锟叫讹拷 
        // flexio_camera_uart_handler();
        // gnss_uart_callback();
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // 锟斤拷锟斤拷锟斤拷删锟斤拷
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 锟斤拷锟斤拷锟叫讹拷
        camera_uart_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // 锟斤拷锟斤拷锟斤拷删锟斤拷
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 锟斤拷锟斤拷锟叫讹拷
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // 锟斤拷锟斤拷锟斤拷删锟斤拷
}


extern "C" void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 锟斤拷锟斤拷锟叫讹拷
        if(NULL != wireless_module_uart_handler)
        {
            wireless_module_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 锟斤拷锟斤拷锟斤拷删锟斤拷
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// 锟斤拷锟斤拷卸媳锟街疚�
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 锟斤拷锟斤拷卸媳锟街疚�
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// 锟斤拷锟斤拷卸媳锟街疚�
    }

}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 锟斤拷锟斤拷锟叫讹拷 预锟斤拷锟叫断达拷锟斤拷锟斤拷锟斤拷 *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT 锟斤拷锟斤拷锟叫讹拷 预锟斤拷锟叫断达拷锟斤拷锟斤拷锟斤拷 *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 锟斤拷锟斤拷卸媳锟街疚�
    }
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// 锟斤拷锟斤拷卸媳锟街疚�
    }
}



void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // 锟斤拷锟斤拷SDK锟皆达拷锟斤拷锟叫断猴拷锟斤拷 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫的回碉拷锟斤拷锟斤拷
    __DSB();                    // 锟斤拷锟斤拷同锟斤拷锟斤拷锟斤拷
}





/*
锟叫断猴拷锟斤拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟矫讹拷应锟斤拷锟杰碉拷锟叫断猴拷锟斤拷
Sample usage:锟斤拷前锟斤拷锟斤拷锟斤拷锟斤拷锟节讹拷时锟斤拷锟叫讹拷
void PIT_IRQHandler(void)
{
    //锟斤拷锟斤拷锟斤拷锟斤拷志位
    __DSB();
}
锟角得斤拷锟斤拷锟叫断猴拷锟斤拷锟斤拷锟街疚�
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/