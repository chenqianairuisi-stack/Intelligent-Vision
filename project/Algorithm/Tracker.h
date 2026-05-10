#pragma once
#include <cstddef>
#include "system_config.h"
#include "RobotState.h"

namespace Algorithm::Tracker {

    void track_point(const Pose2D& target); // 直接追踪单个目标点

    bool check_arrival(Point2D target, float radius);  // 检查是否到达当前目标点

    // 载入网格路径，并启动跟踪
    void load_path(const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& raw_path);
    // 更新当前目标位姿，供控制模块调用
    void update_target();
    // 停止循迹
    inline void stop() { App::g_state.control.tracker_state = TrackerState::NONE; }
}