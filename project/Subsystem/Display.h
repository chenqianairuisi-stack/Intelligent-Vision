#pragma once
#include "zf_common_headfile.h"
#include <stdint.h>

// 页面枚举
enum class MenuPage {
    // ---菜单组 (可选)---
    MAIN_MENU,          // 主菜单
    MODE_SELECT,        // 模式选择
    MAP_SELECT,         // 地图选择

    // --- 状态监控组 (只读) ---
    DASHBOARD,          // 全局状态监控 (游戏进程、地图信息、规划展示等)
    ODOMETRY_DATA,      // 里程计+硬件监控 (全局位姿、编码器速度、IMU)

    // --- 参数调节组 (可编辑) ---
    TUNE_PARAMS
};

namespace Subsystem::Display {
    void init();
    void run();
};