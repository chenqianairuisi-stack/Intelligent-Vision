#pragma once
#include <cstdint>


namespace Subsystem::Vision {
    void init();
    
    // 主循环调用，处理串口数据并更新状态
    void update();    

    // 动作控制接口
    void request_map_ART1();
    void request_pose_ART1();
    void request_capture_ART2(uint8_t entity_id, bool is_box);

    // 寻图前清空缓存
    void reset_semantic_labels();  

};

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
