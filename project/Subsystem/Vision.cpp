#include "Vision.h"
#include "RobotState.h"
#include "system_config.h"
#include "CoreScheduler.h"
#include "PoseEstimate.h"

#include <string.h> 
#include <array>
#include <cmath>
#include <cstring>

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

    struct Art2TargetEntry {
        int8_t entity_id;
        int8_t relative_x;
        int8_t relative_y;
    };

    struct Art2CameraOffset {
        int8_t x;
        int8_t y;
    };

    static int count_mask_bits(uint32_t mask) {
        int count = 0;
        while (mask != 0u) {
            count += static_cast<int>(mask & 1u);
            mask >>= 1u;
        }
        return count;
    }

    static uint32_t level_entity_mask(const SokobanLevel& level) {
        const int entity_count = static_cast<int>(level.box_count) +
                                 static_cast<int>(level.target_count);
        if (entity_count <= 0) return 0u;
        return (uint32_t{1u} << entity_count) - 1u;
    }

    // 将规划地图格差旋转到 ART2 相机坐标，X 为车身右侧，Y 为车头前方
    static Art2CameraOffset to_art2_camera_offset(point entity_grid,
                                                   point vehicle_grid,
                                                   int yaw_index) {
        static constexpr point kForward[4] = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}
        };
        const point forward = kForward[yaw_index & 3];
        // 与规划层的 yaw 定义一致，构造车辆右向轴
        const point right = {
            static_cast<int8_t>(-forward.y),
            forward.x
        };
        const int map_dx = static_cast<int>(entity_grid.x) - vehicle_grid.x;
        const int map_dy = static_cast<int>(entity_grid.y) - vehicle_grid.y;
        return {
            static_cast<int8_t>(map_dx * right.x + map_dy * right.y),
            static_cast<int8_t>(map_dx * forward.x + map_dy * forward.y)
        };
    }

    static bool camera_yaw_to_index(float camera_yaw, int& yaw_index) {
        if (!std::isfinite(camera_yaw)) return false;
        float normalized_yaw = camera_yaw;
        while (normalized_yaw < 0.0f) normalized_yaw += 360.0f;
        while (normalized_yaw >= 360.0f) normalized_yaw -= 360.0f;
        yaw_index = static_cast<int>((normalized_yaw + 45.0f) / 90.0f) & 3;
        return true;
    }

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
    [[maybe_unused]] constexpr float ART1_POSE_STABLE_MAX_YAW_STEP_DEG = 60.0f;

    [[maybe_unused]] [[gnu::always_inline]] inline float abs_yaw_delta_deg(float a, float b) {
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

            // 摄像头协议的 x/y 字段顺序与本机坐标系相反：payload[0..3] 实为物理 y、payload[4..7] 实为物理 x。
            // 上车实测：车放在物理 x≈124 / y≈14 处时，旧解析把 x/y 装反，视觉一介入位姿就错乱→四处飘→斜冲出赛道。
            memcpy(&next_pose.y, &parser_art1.payload_buf[0], 4);
            memcpy(&next_pose.x, &parser_art1.payload_buf[4], 4);
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
                // 纯视觉只用 x/y：帧稳定判定不再依赖视觉姿态角（航向全交陀螺仪）
                stable_frame =
                    gap_ms <= ART1_POSE_STABLE_MAX_GAP_MS &&
                    step_sq <= ART1_POSE_STABLE_MAX_STEP_CM * ART1_POSE_STABLE_MAX_STEP_CM;

                // 仅在稳定帧上喂入帧间位移，驱动视觉侧拐点检测（实时延时估计）
                if (stable_frame) {
                    Subsystem::PoseEstimator::notify_vision_inflection(dx, dy, gap_ms);
                }
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
            vis.art1_pose_tick_ms = now;
            vis.art1_pose_updated = true;
            // 发布顺序加固：buffer→idx→tick 全部落定后再自增 seq。读者（PIT_CH2 视觉修正）以 seq
            // 判新帧，这样一旦看到新 seq，必配套已写好的 tick/帧，取新帧黑窗按 tick 计时才可靠。
            __asm volatile("" ::: "memory");   // 编译器屏障：禁止把 seq 自增重排到上面几条 store 之前
            vis.art1_pose_seq++;
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
            if (vis.art2_expected_mask != 0u) {
                vis.capture_ack_received = true;
            }
        }
        else if (parser_art2.msg_type == MSG_ART2_RESULT && parser_art2.payload_len == 2) {
            uint8_t entity_id = parser_art2.payload_buf[0];
            int8_t semantic_id = parser_art2.payload_buf[1];
            const uint32_t entity_bit = entity_id < SystemConfig::MAX_ENTITIES
                ? (uint32_t{1u} << entity_id) : 0u;
            // ACK 到达前的结果可能属于上一批尚未排空的帧，不能计入当前观测
            if (vis.capture_ack_received &&
                semantic_id >= 0 && semantic_id <= 9 &&
                (vis.art2_expected_mask & entity_bit) != 0u) {
                vis.semantic_labels[entity_id] = semantic_id;
                vis.art2_received_mask |= entity_bit;
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
    finish_capture_ART2();
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

/// \brief 请求 ART2 完成一个宏观观测动作
/// \param level 当前逻辑地图
/// \param vehicle_grid 当前观测位的小车格点
/// \param camera_yaw 本次观测的相机朝向
/// \param active_mask 本次请求的同类别实体掩码
/// \return active_mask 合法且请求已发送时返回 true
///
/// \details
/// 箱子请求固定发送一组 [id, camera_x, camera_y]，目标点请求固定发送三组。
/// 相机坐标 X 指向车身右侧，Y 指向车头前方，目标点按 X 从小到大排列。
bool request_capture_ART2(const SokobanLevel& level,
                          point vehicle_grid,
                          float camera_yaw,
                          uint32_t active_mask) {
    const uint32_t valid_mask = level_entity_mask(level);
    const uint32_t box_mask = level.box_count == 0u
        ? 0u
        : (uint32_t{1u} << level.box_count) - 1u;
    const uint32_t target_mask = valid_mask & ~box_mask;
    const uint32_t requested_boxes = active_mask & box_mask;
    const uint32_t requested_targets = active_mask & target_mask;

    int camera_yaw_index = 0;
    if (active_mask == 0u || (active_mask & ~valid_mask) != 0u ||
        !camera_yaw_to_index(camera_yaw, camera_yaw_index) ||
        (requested_boxes != 0u && requested_targets != 0u)) {
        return false;
    }

    auto& vis = App::g_state.vision;
    // 一个观测宏动作只能持有一批请求，禁止覆盖仍在等待结果的掩码
    if (vis.art2_expected_mask != 0u) {
        return false;
    }
    vis.capture_ack_received = false;
    vis.art2_expected_mask = active_mask;
    vis.art2_received_mask = 0u;

    if (requested_boxes != 0u) {
        if (count_mask_bits(requested_boxes) != 1) {
            finish_capture_ART2();
            return false;
        }

        int box_id = 0;
        while ((requested_boxes & (uint32_t{1u} << box_id)) == 0u) ++box_id;
        const Art2CameraOffset offset = to_art2_camera_offset(
            level.boxes[box_id], vehicle_grid, camera_yaw_index);
        uint8_t payload[3] = {
            static_cast<uint8_t>(box_id),
            static_cast<uint8_t>(offset.x),
            static_cast<uint8_t>(offset.y)
        };
        uart_cam2.send_packet(CMD_TRIG_CAPTURE, payload, sizeof(payload));
        return true;
    }

    if (requested_targets == 0u || count_mask_bits(requested_targets) > 3) {
        finish_capture_ART2();
        return false;
    }

    Art2TargetEntry entries[3] = {};
    int entry_count = 0;
    for (int target_id = 0; target_id < level.target_count; ++target_id) {
        const int entity_id = static_cast<int>(level.box_count) + target_id;
        if ((requested_targets & (uint32_t{1u} << entity_id)) == 0u) continue;

        const Art2CameraOffset offset = to_art2_camera_offset(
            level.targets[target_id], vehicle_grid, camera_yaw_index);
        entries[entry_count++] = {
            static_cast<int8_t>(entity_id),
            offset.x,
            offset.y
        };
    }

    for (int i = 0; i < entry_count - 1; ++i) {
        for (int j = 0; j < entry_count - 1 - i; ++j) {
            const Art2TargetEntry& lhs = entries[j];
            const Art2TargetEntry& rhs = entries[j + 1];
            // X 正方向是相机右侧，故从左到右应按 X 从小到大发送
            if (lhs.relative_x > rhs.relative_x ||
                (lhs.relative_x == rhs.relative_x && lhs.relative_y > rhs.relative_y) ||
                (lhs.relative_x == rhs.relative_x && lhs.relative_y == rhs.relative_y &&
                 lhs.entity_id > rhs.entity_id)) {
                Art2TargetEntry temp = entries[j];
                entries[j] = entries[j + 1];
                entries[j + 1] = temp;
            }
        }
    }

    uint8_t payload[9];
    std::memset(payload, 0xFF, sizeof(payload));
    for (int i = 0; i < entry_count; ++i) {
        payload[i * 3] = static_cast<uint8_t>(entries[i].entity_id);
        payload[i * 3 + 1] = static_cast<uint8_t>(entries[i].relative_x);
        payload[i * 3 + 2] = static_cast<uint8_t>(entries[i].relative_y);
    }
    uart_cam2.send_packet(CMD_TRIG_CAPTURE, payload, sizeof(payload));
    return true;
}

void finish_capture_ART2() {
    auto& vis = App::g_state.vision;
    vis.capture_ack_received = false;
    vis.art2_expected_mask = 0u;
    vis.art2_received_mask = 0u;
}

/// \brief 视觉模块主循环轮询入口
///
/// \details
/// 从两路相机串口 FIFO 中解析完整帧，并将结果写入全局视觉黑板
///
__attribute__((section(".ramfunc"))) void update() {
    while (step_parser(uart_cam1, parser_art1)) {
        process_art1_packet();
    }
    // 目标点批量识别会连续回传多个结果帧，本轮全部取出以缩短等待窗口
    while (step_parser(uart_cam2, parser_art2)) {
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
