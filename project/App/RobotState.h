#pragma once
#include <array>
#include "system_config.h"

// 游戏全局状态机枚举
enum class GamePhase : uint8_t {
    // --- 发车阶段状态 ---
    INIT_CALIBRATE,         // 初始化与校准里程计
    EXIT_START_ZONE,        // 出发车区
    WAIT_FOR_VISION,        // 等待摄像头返回地图

    // --- 第二/三阶段状态 ---
    PLAN_PATROL,            // GTSP 规划巡图观测路径
    EXEC_ACTION_DISPATCH,   // 分发：判断当前动作是去观测，还是去推炸弹
    EXEC_PATROL_MOVE,       // 动作 A1：底盘移动到观测点
    EXEC_ALIGN_YAW,         // 动作 A2：底盘自旋对准目标，并发送 ART2 捕捉请求
    WAIT_ART2_CAPTURE_ACK,  // 等待截图成功
    EXEC_BOMB_PUSH,         // 动作 B：执行推炸宏动作
    UPDATE_MAP,             // 完成推炸弹，更新地图状态

    // --- 第一阶段状态 ---
    BIND_SEMANTICS,         // 巡视完毕，将识别结果绑定到底层算法
    PLAN_SOKOBAN,           // 规划推箱子路径
    EXEC_SOKOBAN,           // 执行推箱子循迹

    // --- 结束阶段状态 ---
    FINISHED,               // 比赛完成，停车
    ERROR_OCCURRED,         // 发生错误，停车

    // --- 调试专用状态 ---
    ANIMATE_PATROL_DEMO,    // 播放巡图过程动画
    ANIMATE_DEMO,           // 播放推箱子过程动画
};

enum class TrackerState : uint8_t {
    NONE,                   // 待机
    TRACKING,               // 正在循迹
    FINISHED                // 路径执行完毕
};

enum class ControlMode : uint8_t {
    MANUAL_DEBUG,           // 调试模式：上位机直接写 target_pose，不理会 Tracker
    AUTO_TRACKING           // 自动模式：听从 Tracker 生成的路径
};

namespace App {

struct RobotState {
    

    // 1. 游戏业务层 (GameManager 写入，全局共享)
    struct {
        GamePhase phase = GamePhase::INIT_CALIBRATE;
        uint8_t stage = 1;           // 当前赛段
        uint8_t action_idx = 0;      // 当前宏动作索引
        bool is_debug_mode = false;  // 调试模式标志
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
        
        // ART1 业务同步标志位
        bool art1_map_ready = false;
        bool art1_pose_updated = false;
        // ART2 业务同步标志位
        bool art2_result_ready = false;
        // ART2 异步流水线状态
        bool capture_ack_received = false;    
        // 基于 entity_id 的语义缓存池 (-1 表示未识别/正在后台推理，1~10 表示识别到的特征数字)
        int8_t semantic_labels[SystemConfig::MAX_ENTITIES];            
    } vision;

    // 3. 规划层 (Tracker 生成，UI/GameManager 读取)
    struct {
        StaticArray<point, SystemConfig::MAX_PATH_LENGTH> grid_path;          // 供 UI 渲染
        StaticArray<Point2D, SystemConfig::MAX_PATH_LENGTH> physical_path;    // 物理航点
        uint16_t current_wp_idx = 0;                                          // 当前正在追的航点索引
    } planning;

    // 4. 控制层 (Tracker 写入，底盘 Chassis 读取)
    struct {
        Pose2D current_target = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};   // 当前目标位姿 (Chassis 直接追这个)
        TrackerState tracker_state = TrackerState::NONE;                 // 当前 Tracker 状态
        ControlMode mode = ControlMode::AUTO_TRACKING;                   // 当前控制模式 (默认自动循迹)
    } control;

    // 5. 物理层 (仅允许底盘/传感器更新，其他模块只读)
    struct {
        Pose2D pose = {0.0f, 0.0f, SystemConfig::ENTRY_YAW};          // 当前物理位姿 (cm/deg)
        WheelSpeed4 current_wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f};   // 当前四轮速度 (cm/s)
    } physical;

    // 6. 调试与遥测层 (供 Telemetry 写入，UI/波形 读取)
    struct {
        int telemetry_mode = 0;       // 波形显示模式
    } debug;
};

inline __attribute__((section(".dtcm_data"))) RobotState g_state;

}