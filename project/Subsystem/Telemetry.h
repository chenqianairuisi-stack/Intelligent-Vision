#pragma once
#include <stdint.h>


namespace Subsystem::Telemetry {
    void init();
    
    void send_wave_data();           // 发送波形给上位机
    void receive_and_parse_task();   // 解析上位机发来的控制命令
}