#pragma once
#include <cstdint>
#include <cmath>
#include "system_config.h"

using namespace SystemConfig;

// 路径跟踪状态机
enum class TrackerState : uint8_t {
    NONE,       // 待机
    TRACKING,   // 正在循迹
    FINISHED    // 路径执行完毕
};

class PathTracker {
public:
    PathTracker();

    void load_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path);    // 载入网格路径，并启动跟踪
    Pose2D update_and_get_target(const Point2D& current_pos);    // 更新跟踪状态并获取当前目标位姿，供控制模块调用

    TrackerState get_state() const { return state; }
    void stop() { state = TrackerState::NONE; }

private:
    StaticArray<point, MAX_PATH_LENGTH> grid_path;
    TrackerState state;
    uint16_t current_wp_idx;  // 当前正在追踪的航点索引
    Pose2D current_target;    // 内部缓存当前的目标位姿    
};

extern PathTracker path_tracker;