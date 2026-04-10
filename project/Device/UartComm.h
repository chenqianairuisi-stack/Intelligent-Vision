#pragma once
#include <cstdint>
#include "zf_common_headfile.h" 
#include "system_config.h"


class UartComm {
public:
    static constexpr uint16_t RX_BUFFER_SIZE = 512; 

    UartComm(uart_index_enum idx_, uint32_t baud_, uart_tx_pin_enum tx_, uart_rx_pin_enum rx_, IRQn_Type irq_);  // 构造函数：传入指定的串口号和引脚
    void Init();  // 初始化硬件配置与 FIFO

    bool read_byte(uint8_t& out_byte);  // 非阻塞读取 1 个字节（供 App 层状态机调用）
    void send_packet(uint8_t msg_type, const uint8_t* payload, uint8_t len);  // 打包发送数据
    void rxisr();   // 供底层的 isr.cpp 中断调用的服务函数

private:
    // 硬件配置参数
    uart_index_enum     uart_idx_;                  // 串口号
    uint32_t            baud_rate_;                 // 波特率
    uart_tx_pin_enum    tx_pin_;                    // TX 引脚
    uart_rx_pin_enum    rx_pin_;                    // RX 引脚
    IRQn_Type           irq_n_;                     // 中断号

    // FIFO 结构体与底层静态数组
    fifo_struct    rx_fifo;                         // FIFO 结构体
    uint8_t        rx_buffer[RX_BUFFER_SIZE];       // FIFO 使用的底层数组
};


extern UartComm uart_cam1;
extern UartComm uart_cam2;