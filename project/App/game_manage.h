#pragma once
#include <cstdint>
#include "system_config.h"
#include "task_vision.h"
#include "planning.h"
#include "tracker.h"

// 游戏全局状态机枚举
enum class GamePhase : uint8_t {
    INIT_CALIBRATE,       // 初始化与校准里程计
    EXIT_START_ZONE,      // 出发车区（向前一格）
    WAIT_FOR_VISION,      // 等待摄像头返回地图
    PLAN_SOKOBAN,         // 规划推箱子路径
    EXEC_SOKOBAN,         // 执行推箱子循迹
    PLAN_RETURN,          // 规划返程路径
    EXEC_RETURN,          // 执行返程循迹
    ENTER_START_ZONE,     // 退回发车区
    FINISHED              // 比赛完成，停车
};

class GameManager {
public:
    static constexpr int ENTRY_GRID_X = 6;
    static constexpr int ENTRY_GRID_Y = 0;

    GameManager();
    void init();
    
    GamePhase get_phase() const { return phase; }
    void update();    // 主循环调用，更新状态机

private:
    GamePhase phase;
};

extern GameManager game_manager;