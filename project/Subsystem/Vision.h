#pragma once
#include <cstdint>
#include "system_config.h"


namespace Subsystem::Vision {
    void init();
    
    // 主循环调用，处理串口数据并更新状态
    void update();    

    // 动作控制接口
    void request_map_ART1();
    void request_pose_ART1();
    void schedule_pose_request_ART1();
    /// \brief 请求 ART2 完成一次箱子或目标点批量观测
    /// \param level 当前逻辑地图，用于把实体格点转换成相对小车坐标
    /// \param vehicle_grid 当前观测时的小车格点
    /// \param camera_yaw 当前观测相机朝向，用于转换到车体相机坐标
    /// \param active_mask 本次观测要求返回语义的同类别实体掩码
    /// \return 请求参数合法且已发包时返回 true
    ///
    /// \details
    /// 相机坐标 X 轴指向车身右侧，Y 轴指向车头前方。箱子请求负载为
    /// [id, relative_x, relative_y]，目标点请求负载为三组
    /// [id, relative_x, relative_y]。目标点不足三组时用 -1 填充空槽。
    bool request_capture_ART2(const SokobanLevel& level,
                              point vehicle_grid,
                              float camera_yaw,
                              uint32_t active_mask);

    // 当前批量观测结束后失效请求掩码，忽略后续迟到结果
    void finish_capture_ART2();

    // 寻图前清空缓存
    void reset_semantic_labels();  

    // 调试函数
    void test_loopback_art2_ack();

}

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
 *   3. 触发批量观测 (向 ART2 发送):
 *      - 箱子：Payload[3] = [实体ID, 相机X, 相机Y]，X 为右侧、Y 为前方。
 *      - 目标点：Payload[9] = 三组 [实体ID, 相机X, 相机Y]，从左到右排列；
 *        不足三组的 ID、X、Y 全部填 -1。
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
 *      - Payload: 任意1字节。主控仍需等待本次请求全部实体的结果包。
 *   4. 目标语义识别结果 (ART2 发送, MSG_ART2_RESULT):
 *      - Payload: [实体ID (uint8_t)] [语义/类别ID (int8_t)] (长度 2)
 */
