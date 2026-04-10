#include "Vision.h"
#include "RobotState.h"
#include "system_config.h"

#include <string.h> 
#include <array>

#include "UartComm.h"


namespace Subsystem::Vision {

// ====================================================================
// 内部私有实现
// ====================================================================

namespace {
    // 状态机解析器的状态枚举
    enum class ParseState : uint8_t {
        WAIT_HEADER1, WAIT_HEADER2, WAIT_TYPE, WAIT_LEN, RECEIVE_PAYLOAD, WAIT_CHECKSUM
    };

    // 独立的解析器结构体
    struct ProtocolParser {
        ParseState state = ParseState::WAIT_HEADER1;
        uint8_t msg_type = 0;
        uint8_t payload_len = 0;
        uint8_t payload_idx = 0;
        uint8_t payload_buf[64];
        uint8_t checksum = 0;
    };

    // 分配在极速区的独立解析器
    __attribute__((section(".dtcm_data"))) ProtocolParser parser_art1;
    __attribute__((section(".dtcm_data"))) ProtocolParser parser_art2; 

    // 协议指令定义
    constexpr uint8_t CMD_REQ_MAP      = 0x10;  // 请求地图指令 (ART1)
    constexpr uint8_t CMD_REQ_POSE     = 0x11;  // 请求定位指令 (ART1)
    constexpr uint8_t CMD_TRIG_CAPTURE = 0x30;  // 触发拍照指令 (ART2)
    constexpr uint8_t MSG_MAP_DATA     = 0x20;  // ART1 发送的地图数据
    constexpr uint8_t MSG_POSE_DATA    = 0x21;  // ART1 发送的定位数据
    constexpr uint8_t MSG_CAPTURE_ACK  = 0x40;  // ART2 发送的拍照确认ACK
    constexpr uint8_t MSG_ART2_RESULT  = 0x41;  // ART2 发送的语义识别结果

    // 通用的状态机解析器 
    __attribute__((always_inline))
    inline bool step_parser(UartComm& uart, ProtocolParser& parser) {
        uint8_t byte;
        while (uart.read_byte(byte)) {
            switch (parser.state) {
                case ParseState::WAIT_HEADER1:
                    if (byte == 0xAA) parser.state = ParseState::WAIT_HEADER2;
                    break;
                case ParseState::WAIT_HEADER2:
                    if (byte == 0x55) parser.state = ParseState::WAIT_TYPE;
                    else parser.state = ParseState::WAIT_HEADER1;
                    break;
                case ParseState::WAIT_TYPE:
                    parser.msg_type = byte;
                    parser.state = ParseState::WAIT_LEN;
                    break;
                case ParseState::WAIT_LEN:
                    parser.payload_len = byte;
                    if (parser.payload_len > sizeof(parser.payload_buf)) {
                        parser.state = ParseState::WAIT_HEADER1; 
                    } else {
                        parser.payload_idx = 0;
                        parser.checksum = parser.msg_type + parser.payload_len;
                        parser.state = (parser.payload_len > 0) ? ParseState::RECEIVE_PAYLOAD : ParseState::WAIT_CHECKSUM;
                    }
                    break;
                case ParseState::RECEIVE_PAYLOAD:
                    parser.payload_buf[parser.payload_idx++] = byte;
                    parser.checksum += byte;
                    if (parser.payload_idx >= parser.payload_len) {
                        parser.state = ParseState::WAIT_CHECKSUM;
                    }
                    break;
                case ParseState::WAIT_CHECKSUM:
                    parser.state = ParseState::WAIT_HEADER1;
                    if (byte == parser.checksum) return true; // 帧校验成功！
                    break;
            }
        }
        return false; // 当前缓冲区已空，帧未完成
    }

    // ART1 协议解包 (地图数据 + 定位数据) [包头AA 55 + 类型 + 长度 + 负载 + 校验和]
    __attribute__((section(".ramfunc"))) 
    void process_art1_packet() {
        auto& vis = App::g_state.vision;

        if (parser_art1.msg_type == MSG_MAP_DATA) {
            const uint8_t* p = parser_art1.payload_buf;
            for (uint16_t i = 0; i < 192; i++) {
                vis.map[i / 12][i % 12] = ((p[i / 8] & (1 << (i % 8))) != 0) ? 1 : 0;
            }
            
            uint8_t counts = p[24];
            vis.box_count = (counts >> 4) & 0x0F;
            vis.bomb_count = counts & 0x0F;
            
            uint8_t offset = 25;
            for (int i = 0; i < vis.box_count; i++)  { vis.boxes[i] = {(int8_t)((p[offset] >> 4) & 0x0F), (int8_t)(p[offset] & 0x0F)}; offset++; }
            for (int i = 0; i < vis.box_count; i++)  { vis.targets[i] = {(int8_t)((p[offset] >> 4) & 0x0F), (int8_t)(p[offset] & 0x0F)}; offset++; }
            for (int i = 0; i < vis.bomb_count; i++) { vis.bombs[i] = {(int8_t)((p[offset] >> 4) & 0x0F), (int8_t)(p[offset] & 0x0F)}; offset++; }
            
            vis.art1_map_ready = true;
        } 
        else if (parser_art1.msg_type == MSG_POSE_DATA && parser_art1.payload_len == 12) {        
            memcpy(&vis.art1_pose.x, &parser_art1.payload_buf[0], 4);
            memcpy(&vis.art1_pose.y, &parser_art1.payload_buf[4], 4);
            memcpy(&vis.art1_pose.yaw, &parser_art1.payload_buf[8], 4);
            vis.art1_pose_updated = true;
        }
    }

    // ART2 协议解包 (捕获确认 + 语义结果) [包头AA 55 + 类型 + 长度 + 负载 + 校验和]
    __attribute__((section(".ramfunc"))) 
    void process_art2_packet() {
        auto& vis = App::g_state.vision;

        if (parser_art2.msg_type == MSG_CAPTURE_ACK && parser_art2.payload_len == 1) {
            vis.capture_ack_received = true;
        }
        else if (parser_art2.msg_type == MSG_ART2_RESULT && parser_art2.payload_len == 2) {
            uint8_t entity_id = parser_art2.payload_buf[0];
            int8_t semantic_id = parser_art2.payload_buf[1];
            if (entity_id < SystemConfig::MAX_ENTITIES) {
                vis.semantic_labels[entity_id] = semantic_id;
            }
        }
    }
} // namespace


// ====================================================================
// 对外公开接口实现
// ====================================================================

void init() {
    uart_cam1.Init();uart_cam2.Init();
    reset_semantic_labels();
}

// 寻图前清空基于 entity_id 的语义缓存池
void reset_semantic_labels() {
    for(int i=0; i < SystemConfig::MAX_ENTITIES; ++i) 
        App::g_state.vision.semantic_labels[i] = -1;
}

// 请求地图 (MCU -> ART1)
void request_map_ART1() {
    uart_cam1.send_packet(CMD_REQ_MAP, nullptr, 0);
}

// 请求定位 (MCU -> ART1)
void request_pose_ART1() {
    uart_cam1.send_packet(CMD_REQ_POSE, nullptr, 0);
}

// 请求捕获照片 (MCU -> ART2)
void request_capture_ART2(uint8_t entity_id, bool is_box) {
    App::g_state.vision.capture_ack_received = false;
    uint8_t payload[2] = {entity_id, static_cast<uint8_t>(is_box ? 1 : 0)};
    uart_cam2.send_packet(CMD_TRIG_CAPTURE, payload, 2);
}

__attribute__((section(".ramfunc"))) void update() {
    if (step_parser(uart_cam1, parser_art1)) {
        process_art1_packet();
    }
    if (step_parser(uart_cam2, parser_art2)) {
        process_art2_packet();
    }
}

}