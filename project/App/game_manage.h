#pragma once
#include <cstdint>
#include "system_config.h"
#include "task_vision.h"
#include "planning.h"
#include "tracker.h"

// 游戏全局状态机枚举
enum class GamePhase : uint8_t {
    INIT_CALIBRATE,       // 初始化与校准里程计
    EXIT_START_ZONE,      // 出发车区
    WAIT_FOR_VISION,      // 等待摄像头返回地图
    PLAN_SOKOBAN,         // 规划推箱子路径
    ANIMATE_DEMO,         // 播放动画演示（可选）
    EXEC_SOKOBAN,         // 执行推箱子循迹
    FINISHED              // 比赛完成，停车
};

class GameManager {
public:
    GameManager();
    
    GamePhase get_phase() const { return phase; }
    void set_phase(GamePhase new_phase) { phase = new_phase; }
    virtual void update();    // 主循环调用，更新状态机

protected:
    static constexpr float ENTRY_X = 120.0f;
    static constexpr float ENTRY_Y = 10.0f;
    static constexpr float ENTRY_YAW = 90.0f;
    static constexpr float OUT_TARGET_X = 120.0f;
    static constexpr float OUT_TARGET_Y = 50.0f;
    static constexpr int PLAN_START_X = 6;
    static constexpr int PLAN_START_Y = 3;

    GamePhase phase;
};

extern GameManager game_manager;



//---------------------------------------------------------------------------------------------
// 下面是一个专门用于调试的派生类，增加了动画演示和规划耗时记录功能
//---------------------------------------------------------------------------------------------

// 独立的动画状态机
struct DemoState {
    point player;
    point boxes[SystemConfig::MAX_BOXES];
    uint8_t box_count;
    point targets[SystemConfig::MAX_BOXES];
    uint8_t target_count;
    
    uint16_t path_idx;      // 动画播到了第几步
    uint32_t last_tick;     // 动画帧率时间戳
};

// 派生类：继承自 GameManager
class DebugGameManager : public GameManager {
public:
    DebugGameManager();
    void timer_init();
    void update() override;

    // 供 UI 读取的调试数据接口
    uint32_t get_plan_time_ms() const { return plan_time_ms; }
    const DemoState& get_demo_state() const { return demo; }

private:
    uint32_t plan_time_ms;  
    DemoState demo;         

    // 定长数组极速删除
    void demo_remove_box(point p);
    void demo_remove_target(point p);
    int  demo_find_box(point p);
};

extern DebugGameManager debug_manager;