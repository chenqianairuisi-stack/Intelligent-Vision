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
    point logical_patrol_pos;                     // 当前小车所在逻辑网格位置
    SokobanLevel logical_level;                   // 用于跟踪物理执行/演示期间的地形变化 (墙壁销毁等)

    StaticArray<PatrolAction, 32> patrol_actions; // 3D DP 返回的宏动作序列
    uint8_t action_idx;                           // 当前正在执行第几个宏动作
    
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
    point bombs[SystemConfig::MAX_BOMBS]; 
    uint8_t bomb_count;

    uint16_t path_idx;      // 动画步数索引
    uint32_t last_tick;     // 动画帧率时间戳

    // --- 巡图动画专属缓存 ---
    SokobanLevel map_state;                            // 记录每一颗炸弹爆炸后的动态临时地图
    StaticArray<point, MAX_PATH_LENGTH> segment_path;  // 当前正在执行的微观网格路径
    uint16_t segment_idx;                              // 当前在微观网格路径的第几步
    uint8_t patrol_target_idx;                         // 正在执行第几个宏动作 (对应 patrol_actions 索引)
};


class DebugGameManager : public GameManager {
public:
    DebugGameManager();
    void update() override;

    // 供 UI 读取的调试数据接口
    uint32_t get_bomb_plan_time_ms() const { return bomb_plan_time_ms; }
    uint32_t get_patrol_plan_time_ms() const { return patrol_plan_time_ms; }
    uint32_t get_push_plan_time_ms() const { return push_plan_time_ms; }

    const DemoState& get_demo_state() const { return demo; }
    const StaticArray<PatrolAction, 32>& get_patrol_actions() const { return patrol_actions; } 

    // 供外部屏幕绘制函数获取炸弹-墙壁匹配对，以此绘制相同颜色的彩色边框
    const StaticArray<BombTask, MAX_BOMBS>& get_cached_bomb_tasks() const { return cached_bomb_tasks; }

    // 注入虚拟视觉标签的接口
    void inject_mock_semantics();

    bool force_bg_redraw = true;  // 发生爆炸摧毁墙壁时置 true，提示 UI 重绘底图废墟

private:
    uint32_t bomb_plan_time_ms;      // 生成炸弹任务的耗时
    uint32_t patrol_plan_time_ms;    // GTSP 规划巡图路径的耗时
    uint32_t push_plan_time_ms;      // IDA* 规划推箱子路径的耗时

    DemoState demo;                  // 演示状态机，记录动画演示时的虚拟小车、箱子、目标点位置等信息
    int8_t mock_truth_labels[SystemConfig::MAX_ENTITIES];     // 虚拟的上帝视角 ART2 答案
    StaticArray<BombTask, MAX_BOMBS> cached_bomb_tasks;       // 缓存炸弹结果供 UI 读取颜色对应关系

    // 辅助函数:定长数组极速删除
    void demo_remove_box(point p);
    void demo_remove_target(point p);
    void demo_remove_bomb(point p);
    int  demo_find_box(point p);
    int  demo_find_bomb(point p);
};

extern DebugGameManager debug_manager;