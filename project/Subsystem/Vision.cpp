#include "Vision.h"
#include "RobotState.h"
#include "system_config.h"
#include "CoreScheduler.h"

#include <string.h> 
#include <array>
#include <cmath>

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
    DTCM_DATA ProtocolParser parser_art1;
    DTCM_DATA ProtocolParser parser_art2; 

    // 协议指令定义
    constexpr uint8_t CMD_REQ_MAP      = 0x10;  // 请求地图指令 (ART1)
    constexpr uint8_t CMD_REQ_POSE     = 0x11;  // 请求定位指令 (ART1)
    constexpr uint8_t CMD_TRIG_CAPTURE = 0x30;  // 触发拍照指令 (ART2)
    constexpr uint8_t MSG_MAP_DATA     = 0x20;  // ART1 发送的地图数据
    constexpr uint8_t MSG_POSE_DATA    = 0x21;  // ART1 发送的定位数据
    constexpr uint8_t MSG_CAPTURE_ACK  = 0x40;  // ART2 发送的拍照确认ACK
    constexpr uint8_t MSG_ART2_RESULT  = 0x41;  // ART2 发送的语义识别结果

    /// \brief 通用串口协议状态机
    /// \param uart 数据来源串口
    /// \param parser 对应串口的解析状态
    /// \return 成功解析出一帧完整且校验通过的数据时返回 true
    ///
    /// \details
    /// 解析器支持被主循环反复调用，每次尽可能消费 FIFO 中已有字节
    ///
    constexpr uint32_t ART1_POSE_STABLE_MAX_GAP_MS = 300U;
    constexpr float ART1_POSE_STABLE_MAX_STEP_CM = 60.0f;
    constexpr float ART1_POSE_STABLE_MAX_YAW_STEP_DEG = 60.0f;

    [[gnu::always_inline]] inline float abs_yaw_delta_deg(float a, float b) {
        float delta = std::fabs(a - b);
        if (delta > 180.0f) {
            delta = 360.0f - delta;
        }
        return delta;
    }

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

    /// \brief 处理 ART1 完整协议帧
    ///
    /// \details
    /// ART1 负责地图和绝对位姿，地图只接收第一帧，位姿每帧通过双缓冲发布
    ///
    __attribute__((section(".ramfunc")))
    void process_art1_packet() {
        auto& vis = App::g_state.vision;

        if (parser_art1.msg_type == MSG_MAP_DATA && vis.art1_map_ready == false) { // 只处理第一帧地图数据，后续更新由业务层控制
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
            // 先写未发布缓冲，再切换发布下标，避免业务层读到半包位姿
            uint8_t write_idx = vis.art1_pose_publish_idx ^ 1U;
            Pose2D& next_pose = vis.art1_pose_buffer[write_idx];

            memcpy(&next_pose.x, &parser_art1.payload_buf[0], 4);
            memcpy(&next_pose.y, &parser_art1.payload_buf[4], 4);
            memcpy(&next_pose.yaw, &parser_art1.payload_buf[8], 4);

            // seq 和 tick 给标定/循迹校正判断新帧与时效性
            uint32_t now = Core::Scheduler::get_sys_tick_ms();
            if (!std::isfinite(next_pose.x) ||
                !std::isfinite(next_pose.y) ||
                !std::isfinite(next_pose.yaw)) {
                vis.art1_pose_stable_count = 0;
                return;
            }

            bool stable_frame = true;
            if (vis.art1_pose_seq != 0U) {
                uint32_t gap_ms = now - vis.art1_pose_tick_ms;
                float dx = next_pose.x - vis.art1_pose.x;
                float dy = next_pose.y - vis.art1_pose.y;
                float step_sq = dx * dx + dy * dy;
                stable_frame =
                    gap_ms <= ART1_POSE_STABLE_MAX_GAP_MS &&
                    step_sq <= ART1_POSE_STABLE_MAX_STEP_CM * ART1_POSE_STABLE_MAX_STEP_CM &&
                    abs_yaw_delta_deg(next_pose.yaw, vis.art1_pose.yaw) <= ART1_POSE_STABLE_MAX_YAW_STEP_DEG;
            }

            if (stable_frame) {
                if (vis.art1_pose_stable_count < App::ART1_POSE_STABLE_REQUIRED_FRAMES) {
                    vis.art1_pose_stable_count++;
                }
            } else {
                vis.art1_pose_stable_count = 1;
            }

            vis.art1_pose = next_pose;
            vis.art1_pose_publish_idx = write_idx;
            vis.art1_pose_seq++;
            vis.art1_pose_tick_ms = now;
            vis.art1_pose_updated = true;
        }
    }

    /// \brief 处理 ART2 完整协议帧
    ///
    /// \details
    /// ART2 负责拍照 ACK 和实体语义结果，语义结果按 entity_id 写入视觉黑板
    ///
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

/// \brief 初始化视觉通信模块
///
/// \details
/// 初始化 ART1/ART2 两路串口，并清空语义标签缓存
///
void init() {
    uart_cam1.Init();uart_cam2.Init();
    reset_semantic_labels();
}

/// \brief 清空基于 entity_id 的语义缓存池
///
/// \details
/// 每次开始新地图或新一轮巡图前调用，-1 表示该实体语义未知
///
void reset_semantic_labels() {
    for(int i=0; i < SystemConfig::MAX_ENTITIES; ++i) 
        App::g_state.vision.semantic_labels[i] = -1;
}

/// \brief 请求 ART1 返回地图数据
void request_map_ART1() {
    uart_cam1.send_packet(CMD_REQ_MAP, nullptr, 0);
}

/// \brief 请求 ART1 返回绝对位姿
void request_pose_ART1() {
    uart_cam1.send_packet(CMD_REQ_POSE, nullptr, 0);
}

/// \brief 在非中断主循环中尽快请求 ART1 返回绝对位姿
void schedule_pose_request_ART1() {
    App::g_state.vision.art1_pose_request_pending = true;
}

/// \brief 请求 ART2 捕获并识别一个实体
/// \param entity_id 实体编号
/// \param is_box true 表示箱子，false 表示目标点
///
void request_capture_ART2(uint8_t entity_id, bool is_box) {
    App::g_state.vision.capture_ack_received = false;
    uint8_t payload[2] = {entity_id, static_cast<uint8_t>(is_box ? 1 : 0)};
    uart_cam2.send_packet(CMD_TRIG_CAPTURE, payload, 2);
}

/// \brief 视觉模块主循环轮询入口
///
/// \details
/// 从两路相机串口 FIFO 中解析完整帧，并将结果写入全局视觉黑板
///
__attribute__((section(".ramfunc"))) void update() {
    if (step_parser(uart_cam1, parser_art1)) {
        process_art1_packet();
    }
    if (step_parser(uart_cam2, parser_art2)) {
        process_art2_packet();
    }

    if (App::g_state.vision.art1_pose_request_pending) {
        App::g_state.vision.art1_pose_request_pending = false;
        request_pose_ART1();
    }
}

// 调试专用 ART2 ACK 环回测试
void test_loopback_art2_ack() {
    // 构造一帧完美的 ART2 ACK 数据包
    uint8_t dummy_payload[1] = {0x01}; 
    uart_cam2.send_packet(MSG_CAPTURE_ACK, dummy_payload, 1);
}


}
