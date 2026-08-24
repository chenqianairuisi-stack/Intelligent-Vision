/// \file UartComm.cpp
/// \brief 双相机 UART 接收与数据包发送实现
///
/// \details
/// 中断接收字节写入固定容量 FIFO，业务层通过无阻塞接口读取并解析
/// 发送端按 AA55 帧头、消息类型、长度、负载和累加校验组织相机协议包

#include "UartComm.h"


DTCM_DATA UartComm uart_cam2(UART_1, 115200, UART1_TX_B12,  UART1_RX_B13, LPUART1_IRQn);
DTCM_DATA UartComm uart_cam1(UART_4, 115200, UART4_TX_C16, UART4_RX_C17, LPUART4_IRQn);


UartComm::UartComm(uart_index_enum idx_, uint32_t baud_, uart_tx_pin_enum tx_, uart_rx_pin_enum rx_, IRQn_Type irq_)
    : uart_idx_(idx_), baud_rate_(baud_), tx_pin_(tx_), rx_pin_(rx_), irq_n_(irq_) {}       

/// \brief 初始化 UART 外设和接收 FIFO
///
/// \details
/// RX 使用中断写 FIFO，业务层在主循环中通过 read_byte 逐字节解析协议
///
void UartComm::Init() {
    // 初始化逐飞 FIFO，绑定定长数组 rx_buffer
    fifo_init(&rx_fifo, FIFO_DATA_8BIT, rx_buffer, RX_BUFFER_SIZE);
    
    // 初始化 UART 外设
    uart_init(uart_idx_, baud_rate_, tx_pin_, rx_pin_);
    
    // 配置中断
    uart_rx_interrupt(uart_idx_, ZF_ENABLE);
    interrupt_set_priority(irq_n_, 1);          // 串口优先级
}


/// \brief 从接收 FIFO 读取一个字节
/// \param out_byte 输出字节
/// \return FIFO 有数据时返回 true
///
/// \details
/// 该函数运行在协议解析热路径中，保持无阻塞，避免影响主循环实时性
///
__attribute__((section(".ramfunc"))) bool UartComm::read_byte(uint8_t& out_byte) {
    // 先检查 FIFO 是否有数据
    if (fifo_used(&rx_fifo) > 0) {
        uint32_t read_len = 1;
        // 读出 1 byte 并清理
        fifo_read_buffer(&rx_fifo, &out_byte, &read_len, FIFO_READ_AND_CLEAN);
        return true;
    }
    return false;
}

/// \brief UART 接收中断服务函数
///
/// \details
/// 中断里只把硬件寄存器中的一个字节搬入 FIFO，协议解析放到主循环处理
///
__attribute__((section(".ramfunc"))) void UartComm::rxisr() {
    uint8_t get_data;
    // 使用查询式读取（不会死等），有数据就塞进 FIFO
    if (uart_query_byte(uart_idx_, &get_data)) {
        fifo_write_buffer(&rx_fifo, &get_data, 1);
    }
}

/// \brief 发送一帧相机协议包
/// \param message_type 协议消息类型
/// \param payload 负载指针，可为空
/// \param len 负载长度
///
/// \details
/// 帧格式为 AA 55 + 类型 + 长度 + 负载 + 简单累加校验和
///
void UartComm::send_packet(uint8_t message_type, const uint8_t* payload, uint8_t len) {
    uint8_t header[4] = {0xAA, 0x55, message_type, len};
    uint8_t checksum = message_type + len;
    
    for (int i = 0; i < len; i++) {
        checksum += payload[i];
    }
    
    // 连续调用 UART 阻塞发送（发送不使用FIFO，因为单片机发送频率低且数据量小）
    uart_write_buffer(uart_idx_, header, 4);
    if (len > 0) {
        uart_write_buffer(uart_idx_, payload, len);
    }
    uart_write_byte(uart_idx_, checksum);
}
