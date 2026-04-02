#pragma once
#include <cstdint>
#include "system_config.h"
#include "tracker.h"
#include "task_vision.h"
#include "sokoban.h"
#include "exploration.h"


// 游戏全局状态机枚举
enum class GamePhase : uint8_t {
    // --- 发车阶段状态 ---
    INIT_CALIBRATE,         // 初始化与校准里程计
    EXIT_START_ZONE,        // 出发车区
    WAIT_FOR_VISION,        // 等待摄像头返回地图

    // --- 第二阶段状态 ---
    PLAN_PATROL,            // GTSP 规划巡图观测路径
    EXEC_PATROL_MOVE,       // 循迹前往下一个观测点
    ALIGN_YAW,              // 到达观测点后，底盘自旋对准目标
    WAIT_ART2_CAPTURE_ACK,  // 发送指令给 ART2，等待截图成功
    BIND_SEMANTICS,         // 巡视完毕，将识别结果绑定到底层算法

    // --- 第一阶段状态 ---
    PLAN_SOKOBAN,           // 规划推箱子路径
    EXEC_SOKOBAN,           // 执行推箱子循迹

    // --- 结束阶段状态 ---
    FINISHED,               // 比赛完成，停车

    // --- 调试专用状态 ---
    ANIMATE_PATROL_DEMO,    // 播放巡图过程动画
    ANIMATE_DEMO,           // 播放推箱子过程动画
};


class GameManager {
public:
    GameManager();
    void init();   // 初始化拨码开关引脚，并读取当前的比赛阶段

    // 对外数据接口
    GamePhase get_phase() const { return phase; }
    uint8_t get_stage() const { return competition_stage; }
    void set_phase(GamePhase new_phase) { phase = new_phase; }

    // 游戏状态机更新函数，放在 main 循环中高频调用
    virtual void update();  

protected:
    GamePhase phase;              // 当前的比赛状态
    uint8_t competition_stage;    // 当前的比赛阶段
    
    // --- 第二阶段所需状态缓存 ---
    StaticArray<ObsPoint, 32> patrol_path;    // GTSP 算出的最优观测点序列
    uint8_t patrol_idx;                       // 当前正在前往第几个观测点
    point logical_patrol_pos;                 // 当前逻辑网格位置
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
    
    uint16_t path_idx;      // 推箱子动画步数
    uint32_t last_tick;     // 动画帧率时间戳

    // --- 巡图动画专属缓存 ---
    StaticArray<point, MAX_PATH_LENGTH> segment_path;  // 两个观测点之间的网格路径
    uint16_t segment_idx;                              // 当前在网格路径的第几步
    uint8_t patrol_target_idx;                         // 当前正开往第几个观测点
};


class DebugGameManager : public GameManager {
public:
    DebugGameManager();
    void update() override;

    // 供 UI 读取的调试数据接口
    uint32_t get_patrol_plan_time_ms() const { return patrol_plan_time_ms; }
    uint32_t get_push_plan_time_ms() const { return push_plan_time_ms; }
    const StaticArray<ObsPoint, 32>& get_patrol_path() const { return patrol_path; }  // UI可以画出那些观测点
    const DemoState& get_demo_state() const { return demo; }

    // 注入虚拟视觉标签的接口
    void inject_mock_semantics();

private:
    uint32_t patrol_plan_time_ms;    // GTSP 规划巡图路径的耗时
    uint32_t push_plan_time_ms;      // IDA* 规划推箱子路径的耗时
    DemoState demo;                  // 演示状态机，记录动画演示时的虚拟小车、箱子、目标点位置等信息
    int8_t mock_truth_labels[8];     // 虚拟的上帝视角 ART2 答案

    // 辅助函数:定长数组极速删除
    void demo_remove_box(point p);
    void demo_remove_target(point p);
    int  demo_find_box(point p);
};

extern DebugGameManager debug_manager;