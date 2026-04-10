#pragma once
#include <cstddef>
#include "system_config.h"
#include "RobotState.h"

class PathTracker {
public:
    // 载入网格路径，并启动跟踪
    static void load_path(const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& raw_path); 

    // 更新当前目标位姿，供控制模块调用
    static void update_target();  

    // 停止循迹
    static inline void stop() {App::g_state.control.tracker_state = TrackerState::NONE;}
};