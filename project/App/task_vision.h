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
    
    // ART1 业务同步标志位
    bool art1_map_ready = false;
    bool art1_pose_updated = false;

    bool art2_result_ready = false;
    bool capture_ack_received = false;    // ART2 异步流水线状态
    int8_t semantic_labels[SystemConfig::MAX_ENTITIES];            // 基于 entity_id 的语义缓存池 (-1 表示未识别/正在后台推理，1~10 表示识别到的特征数字)
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

extern VisionData vision_data;

class VisionManager {
public:
    VisionManager() = default;
    void init();
    
    // 主循环调用，处理串口数据并更新状态
    void update();    

    // 动作控制接口
    void request_map_ART1();
    void request_pose_ART1();
    void request_capture_ART2(uint8_t entity_id, bool is_box);

    // 寻图前清空缓存
    void reset_semantic_labels() { for(int i=0; i<SystemConfig::MAX_ENTITIES; ++i) vision_data.semantic_labels[i] = -1;};  
 
private:
    // 协议指令类型 (MCU -> OpenART)
    static constexpr uint8_t CMD_REQ_MAP      = 0x10;  // 请求刷新地图
    static constexpr uint8_t CMD_REQ_POSE     = 0x11;  // 请求当前定位
    static constexpr uint8_t CMD_TRIG_CAPTURE = 0x30;  // 触发ART2捕捉图片 Payload: [entity_id, is_box]

    // 协议指令类型 (OpenART -> MCU)
    static constexpr uint8_t MSG_MAP_DATA     = 0x20;  // 接收地图包
    static constexpr uint8_t MSG_POSE_DATA    = 0x21;  // 接收定位包
    static constexpr uint8_t MSG_CAPTURE_ACK  = 0x40;  // 捕获成功回报 Payload:[entity_id]
    static constexpr uint8_t MSG_ART2_RESULT  = 0x41;  // 识别结果 Payload: [entity_id, semantic_id]

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