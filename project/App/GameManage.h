#pragma once
#include <array>
#include <cstdint>

#include "system_config.h"
#include "RobotState.h"
#include "Sokoban.h"
#include "Exploration.h"


namespace App::GameEngine {

// 渲染上下文：将游戏状态转换为 UI 层可直接使用的格式，减少 UI 层的计算负担
struct RenderContext {
    uint32_t bomb_plan_time_ms = 0;
    uint32_t patrol_plan_time_ms = 0;
    uint32_t push_plan_time_ms = 0;

    const std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT>* map = nullptr;
    const point* boxes = nullptr;     uint8_t box_count = 0;
    const point* targets = nullptr;   uint8_t target_count = 0;
    const point* bombs = nullptr;     uint8_t bomb_count = 0;

    point player_pos = {0, 0};
    uint16_t path_start_idx = 0;
    const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>* path_ptr = nullptr;  // 当前阶段小车路径
    
    uint8_t action_start_idx = 0;
    const StaticArray<PatrolAction, 32>* actions_ptr = nullptr;  // 当前阶段巡逻动作序列
    const StaticArray<BombTask, SystemConfig::MAX_BOMBS>* bomb_tasks_ptr = nullptr;  // 当前阶段炸弹任务列表（用于 UI 绘制炸弹目标框）
};
 
void init();  // 初始化游戏引擎（读取拨码开关，设置初始阶段）
void update();  // 游戏主循环（每帧调用，负责状态更新和阶段切换）
RenderContext get_render_context();  // 获取当前渲染上下文（供 UI 层调用）

} // namespace App::GameEngine