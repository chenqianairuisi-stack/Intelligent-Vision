/**
 * @brief RT1064 与 OpenART 通信协议总结
 * 
 * 一、 通用帧结构格式:
 *   [包头1] [包头2] [消息类型(MsgType)] [负载长度(Len)] [负载数据(Payload)...] [校验和(Checksum)]
 *   - 包头固定为: 0xAA 0x55
 *   - 校验和计算: Checksum = MsgType + Len + SUM(Payload字节)
 * 
 * 二、 RT1064 -> 视觉模块 (发送指令):
 *   1. 请求地图 (向 ART1 发送):
 *      - MsgType: CMD_REQ_MAP, Payload: 无 (长度 0)
 *   2. 请求定位 (向 ART1 发送):
 *      - MsgType: CMD_REQ_POSE, Payload: 无 (长度 0)
 *   3. 触发拍照 (向 ART2 发送):
 *      - MsgType: CMD_TRIG_CAPTURE, Payload: [实体ID (uint8_t)] [是否为箱子 (1/0)] (长度 2)
 * 
 * 三、 视觉模块 -> RT1064 (接收解析):
 *   1. 地图与目标数据 (ART1 发送, MSG_MAP_DATA):
 *      - Payload: 
 *        [0~23字节]: 12x16 位图迷宫地图 (1位=1格子，1=墙壁。按从左到右、下到上顺序压缩)。
 *        [第24字节]: 高4位(Box/Target数目), 低4位(Bomb数量)。
 *        [25+字节]: 动态坐标组，每字节存一个坐标 (高4位=X, 低4位=Y)。
 *                   顺序依次是: 箱子坐标组 -> 目标点坐标组 -> 炸弹坐标组。
 *   2. 定位数据 (ART1 发送, MSG_POSE_DATA):
 *      - Payload: [X轴 (float)] [Y轴 (float)] [Yaw角 (float)] (长度 12 字节)
 *   3. 拍照确认ACK (ART2 发送, MSG_CAPTURE_ACK):
 *      - Payload: 任意1字节 -> 用于快速放行小车
 *   4. 目标语义识别结果 (ART2 发送, MSG_ART2_RESULT):
 *      - Payload: [实体ID (uint8_t)] [语义/类别ID (int8_t)] (长度 2)
 */

#include "task_vision.h"
#include <string.h> 


// ===================================================== 实例化与初始化 =====================================================

__attribute__((section(".dtcm_data"))) VisionData     vision_data;
__attribute__((section(".dtcm_data"))) VisionManager  vision_manager;


void VisionManager::init() {
    vision_data.art1_map_ready = false;
    vision_data.art1_pose_updated = false;
    vision_data.art2_result_ready = false;

    uart_cam1.Init();
    uart_cam2.Init();
}


// ===================================================== 控制信息传递 =====================================================

// 请求地图 (MCU -> ART1)
void VisionManager::request_map_ART1() {
    uart_cam1.send_packet(CMD_REQ_MAP, nullptr, 0);
}

// 请求定位 (MCU -> ART1)
void VisionManager::request_pose_ART1() {
    uart_cam1.send_packet(CMD_REQ_POSE, nullptr, 0);
}

// 请求捕获照片 (MCU -> ART2)
void VisionManager::request_capture_ART2(uint8_t entity_id, bool is_box) {
    vision_data.capture_ack_received = false;
    uint8_t payload[2] = {entity_id, static_cast<uint8_t>(is_box ? 1 : 0)};
    uart_cam2.send_packet(CMD_TRIG_CAPTURE, payload, 2);
}


// ===================================================== 接收信息解析 =====================================================

// 高频状态机解析轮询，放在 App 层 main loop 中调用
__attribute__((section(".ramfunc"))) void VisionManager::update() {
    // 轮询 ART1
    step_parser(uart_cam1, parser_art1, &VisionManager::process_art1_packet);
    // 轮询 ART2
    step_parser(uart_cam2, parser_art2, &VisionManager::process_art2_packet);
}


// 通用的状态机解析器 
__attribute__((section(".ramfunc"))) 
void VisionManager::step_parser(UartComm& uart, ProtocolParser& parser, void(VisionManager::*process_cb)()) {
    uint8_t byte;

    while (uart.read_byte(byte)) {
        switch (parser.state) {
            case VisionParseState::WAIT_HEADER1:
                if (byte == 0xAA) parser.state = VisionParseState::WAIT_HEADER2;
                break;
            case VisionParseState::WAIT_HEADER2:
                if (byte == 0x55) parser.state = VisionParseState::WAIT_TYPE;
                else parser.state = VisionParseState::WAIT_HEADER1;
                break;
            case VisionParseState::WAIT_TYPE:
                parser.msg_type = byte;
                parser.state = VisionParseState::WAIT_LEN;
                break;
            case VisionParseState::WAIT_LEN:
                parser.payload_len = byte;
                if (parser.payload_len > sizeof(parser.payload_buf)) {
                    parser.state = VisionParseState::WAIT_HEADER1;  // 防止溢出
                } else {
                    parser.payload_idx = 0;
                    parser.checksum = parser.msg_type + parser.payload_len;
                    parser.state = (parser.payload_len > 0) ? VisionParseState::RECEIVE_PAYLOAD : VisionParseState::WAIT_CHECKSUM;
                }
                break;
            case VisionParseState::RECEIVE_PAYLOAD:
                parser.payload_buf[parser.payload_idx++] = byte;
                parser.checksum += byte;
                if (parser.payload_idx >= parser.payload_len) {
                    parser.state = VisionParseState::WAIT_CHECKSUM;
                }
                break;
            case VisionParseState::WAIT_CHECKSUM:
                if (byte == parser.checksum) {
                    // 校验成功，调用处理回调
                    (this->*process_cb)();
                }
                parser.state = VisionParseState::WAIT_HEADER1;  // 重置状态
                break;
        }
    }
}

// ART1 协议解包 (地图数据 + 定位数据) [包头AA 55 + 类型 + 长度 + 负载 + 校验和]
__attribute__((section(".ramfunc"))) void VisionManager::process_art1_packet() {
    if (parser_art1.msg_type == MSG_MAP_DATA) {
        const uint8_t* p = parser_art1.payload_buf;
        
        // 解压 12*16 地图 [前24 bytes, 每bit表示一个格子, 左->右、下->上]
        for (uint16_t i = 0; i < 192; i++) {
            uint8_t x = i % 12;
            uint8_t y = i / 12;
            bool is_wall = (p[i / 8] & (1 << (i % 8))) != 0;
            vision_data.map[y][x] = is_wall ? 1 : 0;
        }
        
        // 解析数量统计 [第25 byte]
        uint8_t counts = p[24];
        vision_data.box_count = (counts >> 4) & 0x0F;
        vision_data.bomb_count = counts & 0x0F;
        
        // 解析动态目标坐标 [1 byte = (X << 4) | Y] 
        uint8_t offset = 25;
        
        for (int i = 0; i < vision_data.box_count; i++) {
            vision_data.boxes[i].x = (p[offset] >> 4) & 0x0F;
            vision_data.boxes[i].y = p[offset] & 0x0F;
            offset++;
        }
        
        for (int i = 0; i < vision_data.box_count; i++) {
            vision_data.targets[i].x = (p[offset] >> 4) & 0x0F;
            vision_data.targets[i].y = p[offset] & 0x0F;
            offset++;
        }
        
        for (int i = 0; i < vision_data.bomb_count; i++) {
            vision_data.bombs[i].x = (p[offset] >> 4) & 0x0F;
            vision_data.bombs[i].y = p[offset] & 0x0F;
            offset++;
        }
        
        vision_data.art1_map_ready = true;
    } 
    else if (parser_art1.msg_type == MSG_POSE_DATA && parser_art1.payload_len == 12) {        
        memcpy(&vision_data.current_x, &parser_art1.payload_buf[0], 4);
        memcpy(&vision_data.current_y, &parser_art1.payload_buf[4], 4);
        memcpy(&vision_data.current_yaw, &parser_art1.payload_buf[8], 4);
        vision_data.art1_pose_updated = true;
    }
}

// ART2 协议解包
__attribute__((section(".ramfunc"))) void VisionManager::process_art2_packet() {
    // 收到 ART2 的“快门完成”确认，小车可以立刻开走
    if (parser_art2.msg_type == MSG_CAPTURE_ACK && parser_art2.payload_len == 1) {
        vision_data.capture_ack_received = true;
    }

    // 收到 ART2 的“推理完成”结果，直接写入对应实体的内存池（无需打断主流程）
    else if (parser_art2.msg_type == MSG_ART2_RESULT && parser_art2.payload_len == 2) {
        uint8_t entity_id = parser_art2.payload_buf[0];
        int8_t semantic_id = parser_art2.payload_buf[1];
        if (entity_id < SystemConfig::MAX_ENTITIES) {
            vision_data.semantic_labels[entity_id] = semantic_id;
        }
    }
}