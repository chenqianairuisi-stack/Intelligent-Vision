#pragma once
#include <array>
#include "uart_comm.h"
#include "system_config.h"

struct VisionData {
    // 解压后的地图 [0为平地，1为墙]
    std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT> map{};
    
    // 识别到的箱子和目标点
    uint8_t box_count;
    uint8_t bomb_count;
    point boxes[SystemConfig::MAX_BOXES];
    point targets[SystemConfig::MAX_BOXES];
    point bombs[SystemConfig::MAX_BOMBS];

    // 全局定位坐标
    float current_x;
    float current_y;
    float current_yaw;
    
    // ART2 识别结果
    int8_t current_front_id = -1; // -1表示未识别，1-10表示有效ID

    // 业务同步标志位
    bool art1_map_ready = false;
    bool art1_pose_updated = false;
    bool art2_result_ready = false;
};

// 串口接收解析状态机枚举
enum class VisionParseState : uint8_t {
    WAIT_HEADER1,
    WAIT_HEADER2,
    WAIT_TYPE,
    WAIT_LEN,
    RECEIVE_PAYLOAD,
    WAIT_CHECKSUM
};

// 独立的解析器结构
struct ProtocolParser {
    VisionParseState state = VisionParseState::WAIT_HEADER1;
    uint8_t msg_type = 0;
    uint8_t payload_len = 0;
    uint8_t payload_idx = 0;
    uint8_t payload_buf[64];   // 最大载荷64字节足够装下极限压缩的地图
    uint8_t checksum = 0;
};

class VisionManager {
public:
    VisionManager() = default;
    void init();
    
    // 主循环调用，处理串口数据并更新状态
    void update();    

    // 动作控制接口
    void request_map_ART1();
    void request_pose_ART1();
    void trigger_ART2(bool is_box);

    // 注入本地脱机测试地图数据，供没有摄像头时的调试使用
    void load_mock_map();

private:
    // 协议指令类型 (MCU -> OpenART)
    static constexpr uint8_t CMD_REQ_MAP      = 0x10;  // 请求刷新地图
    static constexpr uint8_t CMD_REQ_POSE     = 0x11;  // 请求当前定位
    static constexpr uint8_t CMD_TRIG_BOX     = 0x30;  // 触发ART2识别箱子
    static constexpr uint8_t CMD_TRIG_TARGET  = 0x31;  // 触发ART2识别目标点

    // 协议指令类型 (OpenART -> MCU)
    static constexpr uint8_t MSG_MAP_DATA     = 0x20;  // 接收地图包
    static constexpr uint8_t MSG_POSE_DATA    = 0x21;  // 接收定位包
    static constexpr uint8_t MSG_ART2_RESULT  = 0x40;  // 接收ART2识别结果(1-10)

    ProtocolParser parser_art1;
    ProtocolParser parser_art2;

    // 内部处理解析完成的数据包
    void process_art1_packet();
    void process_art2_packet();
    // 状态机步进函数
    void step_parser(UartComm& uart, ProtocolParser& parser, void(VisionManager::*process_cb)());
};


extern VisionData vision_data;
extern VisionManager vision_manager;