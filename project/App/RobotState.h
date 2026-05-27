#pragma once
#include <array>
#include "system_config.h"
#include "Strategy.h"


namespace App {

// 全局机器人状态结构体，包含游戏状态、视觉输入、规划结果、控制指令和物理状态等信息，供各模块读写共享 
struct RobotState {

    // 1. 游戏业务层 (GameManager 写入，全局共享)
    struct {
        GamePhase phase = GamePhase::INIT_CALIBRATE;  
        bool is_advanced_stage = false;  // 是否是第二/三阶段
        bool is_debug_mode = false;      // 调试模式标志（直接注入地图数据，不等待视觉输入）
        bool is_demo_mode = false;       // 演示模式标志（强制动画演示，不进行实际控制）

        uint8_t selected_map_id = 0;     // 选定的地图 ID（由 UI 菜单设置，供调试模式使用）
        uint8_t error_stage = 0;         // 发生错误的阶段（仅在 phase == ERROR_OCCURRED 时用于定位问题）
        uint8_t action_idx = 0;          // 当前宏动作索引
    } game;

    // 2. 感知层 (视觉模块写入，其他模块读取)
    struct {
        // 地图与实体信息
        std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT> map{};
        uint8_t box_count;
        uint8_t bomb_count;
        point boxes[SystemConfig::MAX_BOXES];
        point targets[SystemConfig::MAX_BOXES];
        point bombs[SystemConfig::MAX_BOMBS];

        // ART1 定位推算
        Pose2D art1_pose = {0.0f, 0.0f, 0.0f};
        Pose2D art1_pose_buffer[2] = {};        // ART1 位姿双缓冲，避免中断写入时读到半帧数据
        uint8_t art1_pose_publish_idx = 0;      // 当前发布给业务层读取的缓冲下标
        uint32_t art1_pose_seq = 0;             // 位姿帧序号，用于判断是否有新数据
        uint32_t art1_pose_tick_ms = 0;         // 位姿帧接收时间，用于拒绝过期视觉数据
        
        // ART1 业务同步标志位
        bool art1_map_ready = false;
        bool art1_pose_updated = false;
        // ART2 业务同步标志位
        bool art2_result_ready = false;
        // ART2 异步流水线状态
        bool capture_ack_received = false;
        // 语义缓存池（-1 表示未知，0~9 表示识别到的特征数字；索引顺序为先箱子、再目标点）
        int8_t semantic_labels[SystemConfig::MAX_ENTITIES];
    } vision;

    // 3. 规划层 (Tracker 生成，UI/GameManager 读取)
    struct {
        StaticArray<point, SystemConfig::MAX_PATH_LENGTH> grid_path;          // 供 UI 渲染

        StaticArray<Point2D, SystemConfig::MAX_PATH_LENGTH> physical_path;    // 物理路径（cm级坐标），供 Tracker 追踪
        uint16_t current_wp_idx = 0;                                          // 当前正在追的航点索引

        StaticArray<BombTask, SystemConfig::MAX_BOMBS> bomb_tasks;            // 炸弹任务列表
        Point2D vision_segment_start = {0.0f, 0.0f};                          // 当前直线路径段起点，用于判断视觉校正轴向
        bool vision_correction_done = false;                                  // 当前路径段是否已经做过一次视觉校正
    } planning;

    // 4. 控制层 (Tracker 写入，底盘 Chassis 读取)
    struct {
        Pose2D current_target = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};   // 当前目标位姿 (Chassis 直接追这个)
        TrackerState tracker_state = TrackerState::NONE;                 // 当前 Tracker 状态
        ControlMode mode = ControlMode::AUTO_TRACKING;                   // 当前控制模式 (默认自动循迹)
    } control;

    // 5. 物理层 (仅允许底盘/传感器更新，其他模块只读)
    struct {
        Pose2D pose = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};          // 物理位姿 (cm/deg)
        WheelSpeed4 current_wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f};   // 当前四轮速度 (cm/s)
        bool is_stopped = true;                                       // 底盘是否完全停止 
    } physical;

    // 6. 其他调试信息
    struct {
        int telemetry_mode = -1;         // 波形显示模式（默认不显示）
        bool need_bg_redraw = true;      // UI 背景重绘请求标志
    } debug;
};

inline __attribute__((section(".dtcm_data"))) RobotState g_state;

}
