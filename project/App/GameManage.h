#pragma once
#include <array>
#include <cstdint>
#include <string_view>

#include "system_config.h"
#include "RobotState.h"
#include "RobotTask.h"

#include "Sokoban.h"
#include "Exploration.h"
#include "MacroPlanner.h"


namespace App::GameEngine {

// 渲染上下文：将游戏状态转换为 UI 层可直接使用的格式
struct RenderContext {
    // 规划耗时统计（ms）
    uint32_t patrol_bomb_plan_time_ms = 0;  // 寻图阶段炸弹解算耗时
    uint32_t exploration_plan_time_ms = 0;  // Exploration 参考巡图规划耗时
    uint32_t push_bomb_plan_time_ms = 0;    // 推箱阶段炸弹解算耗时
    uint32_t push_plan_time_ms = 0;         // Sokoban 推箱解算耗时

    // 地图与实体数据源：所有的墙、箱、靶、炸弹都在这一个指针里，UI 直接渲染这个数据源即可
    const std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT>* map = nullptr;

    // 小车位置
    point player_pos = {0, 0};
    
    // 当前路径
    uint16_t path_start_idx = 0;
    const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>* path_ptr = nullptr;  // 当前阶段小车路径
    
    // 宏动作序列与炸弹任务列表
    uint8_t action_start_idx = 0;
    const StaticArray<MacroAction, 32>* actions_ptr = nullptr;  // 当前阶段巡逻动作序列
    const StaticArray<BombTask, SystemConfig::MAX_BOMBS>* bomb_tasks_ptr = nullptr;  // 当前阶段炸弹任务列表（用于 UI 绘制炸弹目标框）
};


// =========================================================
// 三级状态机架构
// =========================================================
// 1. 基类：正式比赛逻辑（实车 + 实景）
class GameManager {
public:
    virtual void update();

protected:
    SokobanLevel logical_level;  // 逻辑层地图状态
    uint32_t observed_mask = 0;  // 已经观测到的实体掩码
    float current_observe_yaw = SystemConfig::ENTRY_YAW;  // 当前航向角

    StaticArray<MacroAction, 32> reference_patrol_actions; // 参考巡图动作序列
    StaticArray<MacroAction, 32> executed_patrol_actions;  // 已执行的巡图动作序列（仅供 UI 展示，调度器内部以 current_macro_action 为准）
    MacroAction current_macro_action;  // 当前正在执行的宏动作
    
    // 微观任务队列（由 current_macro_action 转换而来）
    StaticArray<RobotTask, 10> task_queue;
    size_t current_task_idx = 0;
    bool art2_capture_request_sent = false;  // 当前 ART2 请求是否已发出

    // --- 内部函数 ---
    bool plan_next_macro_action(MacroAction& out_action);
    void start_macro_action(const MacroAction& action);
    bool prepare_phase2_solver(bool dynamic_fallback);
    point current_grid_from_pose(const Pose2D& pos) const;
    uint32_t observe_mask_of(const MacroAction& action) const;
};


// 2. Mock 派生类：拦截视觉输入，注入本地地图与语义（实车 + 虚景）
class MockGameManager : public GameManager {
public:
    void update() override;

protected:
    int8_t mock_truth_labels[SystemConfig::MAX_ENTITIES];  // Mock 语义标签缓存，供注入使用
    void inject_mock_semantics();  // 将预设的语义标签写入视觉黑板
};


// 3. Demo 派生类：拦截底盘控制，纯内部动画推演（虚车 + 虚景）
class DemoGameManager : public MockGameManager {
public:
    void update() override;
    RenderContext get_render_context() const;

private:
    // 严格遵循：0=空地, 1=墙壁, 2=箱子, 3=目标点, 4=炸弹
    enum DemoTile : uint8_t {
        TILE_EMPTY  = 0, TILE_WALL   = 1, TILE_BOX    = 2, TILE_TARGET = 3, TILE_BOMB   = 4
    };

    struct DemoState {
        point player;  // 小车当前坐标）
        std::array<std::array<uint8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT> map;

        uint32_t last_tick; // 上次更新时间戳（动画推进用）

        uint16_t path_idx;  // 当前路径索引
        uint8_t patrol_target_idx;  // 当前巡逻目标索引

        StaticArray<point, SystemConfig::MAX_PATH_LENGTH> segment_path;  // 当前宏动作的细分路径
        uint16_t segment_idx;  // 当前宏动作细分路径索引
        bool has_active_macro = false; // 是否已有正在播放的宏动作
    } demo;

    uint32_t patrol_bomb_plan_time_ms = 0;
    uint32_t exploration_plan_time_ms = 0;
    uint32_t push_bomb_plan_time_ms = 0;
    uint32_t push_plan_time_ms = 0;

    mutable std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT> flattened_render_map;

    // 内部辅助函数：每当开启新动画阶段时，从基准地图同步全局状态
    void init_demo_map_from_logical();
};


// =========================================================
// 对外接口
// =========================================================
void init();  
void update(); 

/// \brief 切换连续关卡返航后的停顿时长档位
void toggle_return_home_dwell();

/// \brief 获取当前返航停顿时长
/// \return 当前选择的停顿时长 ms
uint32_t get_return_home_dwell_ms();

// Mock 模式专用接口：获取可用地图数量、地图名称列表、加载指定地图
uint8_t get_mock_map_count();
const char* get_mock_map_name(uint8_t idx);
void load_mock_map(uint8_t map_idx);

// 获取当前渲染上下文（供 UI 层调用）
RenderContext get_render_context();  

} // namespace App::GameEngine
