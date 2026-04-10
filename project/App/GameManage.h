#pragma once
#include <array>
#include <cstdint>
#include "system_config.h"
#include "RobotState.h"
#include "Sokoban.h"
#include "Exploration.h"

class GameManager {
public:
    GameManager();
    void init();   // 初始化拨码开关引脚，并读取当前的比赛阶段

    // 对外数据接口
    void set_phase(GamePhase new_phase) { App::g_state.game.phase = new_phase; }
    const SokobanLevel& get_logical_level() const { return logical_level; }

    // 游戏状态机更新函数，放在 main 循环中高频调用
    virtual void update();  

protected:    
    
    // --- 第二阶段所需状态缓存 ---
    point logical_patrol_pos;                     // 当前小车所在逻辑网格位置
    SokobanLevel logical_level;                   // 用于跟踪物理执行/演示期间的地形变化 (墙壁销毁等)

    StaticArray<PatrolAction, 32> patrol_actions; // 3D DP 返回的宏动作序列    
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

constexpr uint8_t TILE_WALL  = 1 << 0;  // 墙壁
constexpr uint8_t TILE_TGT   = 1 << 1;  // 目标点
constexpr uint8_t TILE_PATH  = 1 << 2;  // 路径蓝点
constexpr uint8_t TILE_CROSS = 1 << 3;  // 未观测叉叉
constexpr uint8_t TILE_BOX   = 1 << 4;  // 箱子
constexpr uint8_t TILE_BOMB  = 1 << 5;  // 炸弹
constexpr uint8_t TILE_CAR   = 1 << 6;  // 小车

struct RenderContext {
    // 0. 暴露给 UI 的耗时数据
    uint32_t bomb_plan_time_ms = 0;   
    uint32_t patrol_plan_time_ms = 0;
    uint32_t push_plan_time_ms = 0;
    
    // 1. 地图与实体指针
    const std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT>* map;
    const point* boxes;     uint8_t box_count;
    const point* targets;   uint8_t target_count;
    const point* bombs;     uint8_t bomb_count;
    
    // 2. 动态轨迹与小车
    point player_pos;
    const StaticArray<point, MAX_PATH_LENGTH>* path_ptr; 
    uint16_t path_start_idx;                             
    
    // 3. 宏动作与炸弹任务投影
    const StaticArray<PatrolAction, 32>* actions_ptr;
    uint8_t action_start_idx;
    const StaticArray<BombTask, MAX_BOMBS>* bomb_tasks_ptr;  // 用于画彩色炸弹框
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
    // 同步逻辑状态到 UI 的接口
    RenderContext get_render_context() const; // 获取当前帧的渲染投影

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
extern RenderContext dashboard_vm;  // 仪表盘数据模型，供 Dashboard 页面渲染使用