#pragma once
#include "zf_common_headfile.h"
#include "tuning_config.h"
#include "system_config.h"
#include <stdint.h>

extern char last_rx_cmd[32];    // 全局变量，用于保存最后一次接收到的命令字符串，供 TFT 显示

class Telemetry {
public:
    void init();
    
    // 发送波形给上位机 (放入 20ms 定时器中断中，或主循环中调用)
    void send_wave_data();
    
    // 解析上位机发来的控制命令
    void receive_and_parse_task();

private:
    // VOFA+ JustFloat 协议包结构体 (必须 1 字节对齐)
    // 格式：N 个 float + 1 个 4 字节的 Tail (0x7F800000)
    #pragma pack(push, 1)
    struct VofaJustFloat {
        uint8_t head;        // 帧头，固定为 0xAA
        float target_v;      // 通道 0: 期望速度
        float actual_v;      // 通道 1: 实际速度
        float target_x;      // 通道 2: 期望 X
        float actual_x;      // 通道 3: 实际 X
        float target_y;      // 通道 4: 期望 Y
        float actual_y;      // 通道 5: 实际 Y
        uint8_t tail;        // 帧尾，固定为 0xBB
    };
    #pragma pack(pop)

    VofaJustFloat tx_packet;

    // 接收缓冲区 (应对定长/变长字符串指令)
    char rx_cmd_buf[64];
    uint8_t rx_idx = 0;

    // 解析具体的字符串指令
    void execute_command(const char* cmd);
};

extern Telemetry telemetry;
