#pragma once
#include "RobotState.h"

namespace App::GameEngine {

enum class TaskType : uint8_t {
    LOAD_PATH_OBS,
    LOAD_PATH_BOMB,
    LOAD_PATH_BOX,
    WAIT_TRACKING_DONE,
    ALIGN_YAW,
    WAIT_ART2_CAPTURE,
    APPLY_BOMB_RESULT,
    UPDATE_BOX_LOGIC
    // 注：炸弹按需等待爆炸未做成独立任务，而是在 LOAD_PATH_* 加载前内联门控
    // （需先规划出路径才能判断是否穿过被炸墙），见 GameManage gate_explosion_before_path
};

// 指令结构
struct alignas(4) RobotTask {
    TaskType type;

    // 任务参数联合体，根据任务类型使用不同的字段
    struct Param {
        point target_grid;         // LOAD_PATH_OBS
        BombPushAction bomb_push;  // LOAD_PATH_BOMB / APPLY_BOMB_RESULT
        BoxPushAction box_push;    // LOAD_PATH_BOX / UPDATE_BOX_LOGIC
        float target_yaw;          // ALIGN_YAW
        struct {
            uint8_t entity_id;
            bool is_box;
        } capture;                 // WAIT_ART2_CAPTURE
    } param;

    
    // 工具函数：根据不同的任务类型构造对应的 RobotTask 实例
    static RobotTask make_path_obs(point grid) { 
        RobotTask t{}; 
        t.type = TaskType::LOAD_PATH_OBS; t.param.target_grid = grid; 
        return t; 
    }
    static RobotTask make_path_bomb(const BombPushAction& bomb_push) { 
        RobotTask t{}; 
        t.type = TaskType::LOAD_PATH_BOMB; t.param.bomb_push = bomb_push; 
        return t; 
    }
    static RobotTask make_path_box(const BoxPushAction& box_push) {
        RobotTask t{};
        t.type = TaskType::LOAD_PATH_BOX; t.param.box_push = box_push;
        return t;
    }
    static RobotTask make_wait_track() { 
        RobotTask t{}; 
        t.type = TaskType::WAIT_TRACKING_DONE; 
        return t; 
    }
    static RobotTask make_align(float yaw) { 
        RobotTask t{}; 
        t.type = TaskType::ALIGN_YAW; t.param.target_yaw = yaw; 
        return t; 
    }
    static RobotTask make_capture(uint8_t id, bool box) { 
        RobotTask t{}; 
        t.type = TaskType::WAIT_ART2_CAPTURE; t.param.capture.entity_id = id; t.param.capture.is_box = box; 
        return t; 
    }
    static RobotTask make_apply_bomb_result(const BombPushAction& bomb_push) {
        RobotTask t{};
        t.type = TaskType::APPLY_BOMB_RESULT; t.param.bomb_push = bomb_push;
        return t;
    }
    static RobotTask make_update_box(const BoxPushAction& box_push) {
        RobotTask t{};
        t.type = TaskType::UPDATE_BOX_LOGIC; t.param.box_push = box_push;
        return t;
    }
};

} // namespace App::GameEngine
