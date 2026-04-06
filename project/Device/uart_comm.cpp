#include "uart_comm.h"


__attribute__((section(".dtcm_data"))) UartComm uart_cam2(UART_1, 115200, UART1_TX_B12,  UART1_RX_B13, LPUART1_IRQn);
__attribute__((section(".dtcm_data"))) UartComm uart_cam1(UART_4, 115200, UART4_TX_C16, UART4_RX_C17, LPUART4_IRQn);


UartComm::UartComm(uart_index_enum idx_, uint32_t baud_, uart_tx_pin_enum tx_, uart_rx_pin_enum rx_, IRQn_Type irq_)
    : uart_idx_(idx_), baud_rate_(baud_), tx_pin_(tx_), rx_pin_(rx_), irq_n_(irq_) {}       

void UartComm::Init() {
    // 初始化逐飞 FIFO，绑定定长数组 rx_buffer
    fifo_init(&rx_fifo, FIFO_DATA_8BIT, rx_buffer, RX_BUFFER_SIZE);
    
    // 初始化 UART 外设
    uart_init(uart_idx_, baud_rate_, tx_pin_, rx_pin_);
    
    // 配置中断
    uart_rx_interrupt(uart_idx_, ZF_ENABLE);
    interrupt_set_priority(irq_n_, 1);          // 串口优先级
}


// 高频读取函数
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

// 中断处理函数
__attribute__((section(".ramfunc"))) void UartComm::rxisr() {
    uint8_t get_data;
    // 使用查询式读取（不会死等），有数据就塞进 FIFO
    if (uart_query_byte(uart_idx_, &get_data)) {
        fifo_write_buffer(&rx_fifo, &get_data, 1);
    }
}

// 发送协议包（包头AA 55 + 类型 + 长度 + 负载 + 校验和）
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