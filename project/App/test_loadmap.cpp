#include "test_loadmap.h"
#include "task_vision.h"
#include "uart_comm.h"


// 获取地图库中地图的总数
uint8_t TestMap::get_mock_map_count() {return MOCK_MAP_COUNT;}

// 获取地图库中指定地图的名称
const char* TestMap::get_mock_map_name(uint8_t idx) {
    if (idx < MOCK_MAP_COUNT) return mock_map_library[idx].name;
    return "Unknown";
}

// 注入本地脱机测试地图数据
void TestMap::load_mock_map(uint8_t map_idx) {
    if (map_idx >= MOCK_MAP_COUNT) return; // 越界保护
    
    // 获取选中的地图布局
    const auto& map_layout = mock_map_library[map_idx].layout;

    vision_data.box_count = 0;
    vision_data.bomb_count = 0;
    uint8_t target_count = 0;

    // 遍历解析字符矩阵
    for (int y = 0; y < SystemConfig::MAP_MAX_HEIGHT; y++) {
        for (int x = 0; x < SystemConfig::MAP_MAX_WIDTH; x++) {
            
            char ch = map_layout[SystemConfig::MAP_MAX_HEIGHT - 1 - y][x]; 
    
            vision_data.map[y][x] = 0;
            switch(ch) {
                case '#': 
                    vision_data.map[y][x] = 1; 
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
                    break;
            }
        }
    }
    
    vision_data.art1_map_ready = true; 
}



// 构造一个测试包，测试串口自环回功能
void TestMap::test_loopback_map() {

    // 长度 = 24(墙壁) + 1(数量) + 1(箱子) + 1(目标) + 1(炸弹) = 28字节
    uint8_t payload[28] = {0}; 
    payload[0] = 0x03;  // 铺设墙壁: 假设在 X=0, Y=0 (第0个bit) 和 X=1, Y=0 (第1个bit) 放墙壁
    payload[24] = 0x11; // 设置数量: 高4位是箱子(1), 低4位是炸弹(1) -> 0x11
    payload[25] = 0x34; // 箱子坐标：X=3, Y=4
    payload[26] = 0x56; // 目标坐标：X=5, Y=6
    payload[27] = 0x78; // 炸弹坐标：X=7, Y=8

    // 调用底层的串口发送函数发给自己
    uart_cam1.send_packet(0x20, payload, 28);
}