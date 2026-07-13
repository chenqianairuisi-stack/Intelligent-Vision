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

    // PIT_CH0 ��ʱ���жϣ������� SystemConfig::PIT_CH0_PERIOD_MS ���ã����������������ݶ�ȡ�����
    if(pit_flag_get(PIT_CH0)) 
    {
        pit_flag_clear(PIT_CH0);

        // ���������ݶ�ȡ��ת��
        imu_icm42688.update_all();  

        // yaw ��Ƕȸ���
        Subsystem::PoseEstimator::update_yaw_1ms_tick();

        // 轮速内环快环：1ms 定时器分频到 5ms(200Hz)，先测速再跑速度 PID 出占空比
        static uint8_t s_speed_loop_div = 0;
        if (++s_speed_loop_div >= SystemConfig::SPEED_LOOP_PERIOD_MS) {
            s_speed_loop_div = 0;
            encoders.update_speed_5ms_tick();
            Subsystem::Chassis::update_speed_loop_5ms();
        }
    }
    
    // PIT_CH1 ��ʱ���жϣ������� SystemConfig::PIT_CH1_PERIOD_MS ���ã������ڵ��̿����㷨���º���̼�����
    if(pit_flag_get(PIT_CH1)) 
    {
        pit_flag_clear(PIT_CH1);

        // ��������������
        encoders.update_encoders_20ms_tick();

        // ���µ����Ƿ���ȫֹͣ��״̬
        Subsystem::Chassis::check_is_stopped(); 

        // ȫ�ֶ�λ��̼����� 
        Subsystem::PoseEstimator::update_position_20ms_tick(encoders.getAllCounts(), App::g_state.physical.pose.yaw);

        // ���̿����㷨����
        Subsystem::Chassis::update_20ms_tick();      
    }
    
    // PIT_CH2 定时器中断：周期为 SystemConfig::PIT_CH2_PERIOD_MS(15ms)，专跑视觉位姿修正。
    // 必须放在 CH1 分支之后：同一次中断里若 20ms 也到期，先让里程计更新 s_encoder_pose，
    // CH2 再拿最新编码器位姿去纠偏。与 CH1 同属一个 PIT_IRQ、串行执行，写 pose.x/y 免锁。
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
        // �����жϴ��������� uart_cam1 ���жϷ�����
        uart_cam2.rxisr();
        

    // #if DEBUG_UART_USE_INTERRUPT                        // ������� debug �����ж�
    //     debug_interrupr_handler();                      // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
    // #endif                                              // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // ������ɾ��
}

extern "C" void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // �����жϴ��������� uart_cam2 ���жϷ�����
        uart_cam1.rxisr();

        // �����ж� 
        // flexio_camera_uart_handler();
        // gnss_uart_callback();
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // �����ж�
        camera_uart_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // ������ɾ��
}


extern "C" void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // �����ж�
        if(NULL != wireless_module_uart_handler)
        {
            wireless_module_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // ������ɾ��
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// ����жϱ�־λ
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // ����жϱ�־λ
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// ����жϱ�־λ
    }

}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT �����ж� Ԥ���жϴ������� *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT �����ж� Ԥ���жϴ������� *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // ����жϱ�־λ
    }
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// ����жϱ�־λ
    }
}



void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // ����SDK�Դ����жϺ��� ���������������������õĻص�����
    __DSB();                    // ����ͬ������
}





/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ���ж�
void PIT_IRQHandler(void)
{
    //��������־λ
    __DSB();
}
�ǵý����жϺ������־λ
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