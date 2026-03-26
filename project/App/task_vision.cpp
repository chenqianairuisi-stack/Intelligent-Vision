#include "task_vision.h"
#include <string.h> 

__attribute__((section(".dtcm_data"))) VisionData     vision_data;
__attribute__((section(".dtcm_data"))) VisionManager  vision_manager;

void VisionManager::init() {
    vision_data.art1_map_ready = false;
    vision_data.art1_pose_updated = false;
    vision_data.art2_result_ready = false;

    uart_cam1.Init();
    uart_cam2.Init();
}

// 请求地图 (MCU -> ART1)
void VisionManager::request_map_ART1() {
    uart_cam1.send_packet(CMD_REQ_MAP, nullptr, 0);
}

// 请求定位 (MCU -> ART1)
void VisionManager::request_pose_ART1() {
    uart_cam1.send_packet(CMD_REQ_POSE, nullptr, 0);
}

// 触发第一视角识别 (MCU -> ART2)
void VisionManager::trigger_ART2(bool is_box) {
    vision_data.art2_result_ready = false;
    uint8_t cmd = is_box ? CMD_TRIG_BOX : CMD_TRIG_TARGET;
    uart_cam2.send_packet(cmd, nullptr, 0);
}

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
        
        // 解压 16*12 地图 [前24 bytes, 每bit表示一个格子, 左->右、下->上]
        for (uint16_t i = 0; i < 192; i++) {
            uint8_t x = i % 16;
            uint8_t y = i / 16;
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
    else if (parser_art1.msg_type == MSG_POSE_DATA && parser_art1.payload_len == 12) {        memcpy(&vision_data.current_x, &parser_art1.payload_buf[0], 4);
        memcpy(&vision_data.current_y, &parser_art1.payload_buf[4], 4);
        memcpy(&vision_data.current_yaw, &parser_art1.payload_buf[8], 4);
        vision_data.art1_pose_updated = true;
    }
}

// ART2 协议解包 (数字识别)
__attribute__((section(".ramfunc"))) void VisionManager::process_art2_packet() {
    if (parser_art2.msg_type == MSG_ART2_RESULT && parser_art2.payload_len == 1) {
        vision_data.current_front_id = parser_art2.payload_buf[0];
        vision_data.art2_result_ready = true;
    }
}


// 加载本地 ASCII 字符测试地图
void VisionManager::load_mock_map() {
    const char* map_layout[SystemConfig::MAP_MAX_HEIGHT] = {
        "############",
        "#----------#",
        "#-######---#",
        "#-#----#-.-#",
        "#-#-##-----#",
        "#-#-.$$----#",
        "#-####---#-#",
        "#----#---#-#",
        "#----#---#-#",
        "#----#---#-#",
        "#--###-----#",
        "#--#-------#",
        "#--#-------#",
        "#-.$-------#",
        "##-#-------#",
        "############"
    };

    // 清空原有的统计数据
    vision_data.box_count = 0;
    vision_data.bomb_count = 0;
    uint8_t target_count = 0;

    // 遍历解析字符矩阵
    for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; y++) {
        for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; x++) {
            
            // 注：屏幕/字符画是自顶向下(Row 0在最上面)，而我们的地图坐标系是自底向上(Y=0在最下面)
            char ch = map_layout[SystemConfig::MAP_MAX_HEIGHT - 1 - y][x]; 
    
            vision_data.map[y][x] = 0;
            switch(ch) {
                case '#': 
                    vision_data.map[y][x] = 1; // 墙壁
                    break;
                    
                case '.': 
                    if (target_count < SystemConfig::MAX_BOXES) {
                        vision_data.targets[target_count] = {(int8_t)x, (int8_t)y};
                        target_count++;
                    }
                    break;
                    
                case '$': 
                    if (vision_data.box_count < SystemConfig::MAX_BOXES) {
                        vision_data.boxes[vision_data.box_count] = {(int8_t)x, (int8_t)y};
                        vision_data.box_count++;
                    }
                    break;
                    
                case '*': 
                    if (vision_data.bomb_count < SystemConfig::MAX_BOMBS) {
                        vision_data.bombs[vision_data.bomb_count] = {(int8_t)x, (int8_t)y};
                        vision_data.bomb_count++;
                    }
                    break;
                    
                default: 
                    // '-' 或者空格等其他字符，均视为平地(已默认为0)
                    break;
            }
        }
    }
    
    // 拉起就绪标志位，瞬间触发 GameManager 进入 PLAN_SOKOBAN 寻路状态
    vision_data.art1_map_ready = true; 
}