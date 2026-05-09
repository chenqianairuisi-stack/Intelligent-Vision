#pragma once
#include "RobotState.h"

namespace App::GameEngine {

enum class TaskType : uint8_t { 
    LOAD_PATH_OBS,
    LOAD_PATH_BOMB,
    WAIT_TRACKING_DONE, 
    ALIGN_YAW, 
    WAIT_ART2_CAPTURE,
    UPDATE_MAP_LOGIC
};

// 指令结构
struct alignas(4) RobotTask {
    TaskType type;
    union Param {
        point target_grid;         // LOAD_PATH_OBS
        BombTask bomb;             // LOAD_PATH_BOMB
        float target_yaw;          // ALIGN_YAW
        struct {
            uint8_t entity_id;
            bool is_box;
        } capture;                 // WAIT_ART2_CAPTURE
        struct {
            point bomb_start;
            point target_wall;
        } map_update;              // UPDATE_MAP_LOGIC
        
        Param() { std::memset(this, 0, sizeof(Param)); } // 默认构造
    } param;

    
    static RobotTask make_path_obs(point grid) { 
        RobotTask t; 
        t.type = TaskType::LOAD_PATH_OBS; t.param.target_grid = grid; 
        return t; 
    }
    static RobotTask make_path_bomb(BombTask b) { 
        RobotTask t; 
        t.type = TaskType::LOAD_PATH_BOMB; t.param.bomb = b; 
        return t; 
    }
    static RobotTask make_wait_track() { 
        RobotTask t; 
        t.type = TaskType::WAIT_TRACKING_DONE; 
        return t; 
    }
    static RobotTask make_align(float yaw) { 
        RobotTask t; 
        t.type = TaskType::ALIGN_YAW; t.param.target_yaw = yaw; 
        return t; 
    }
    static RobotTask make_capture(uint8_t id, bool box) { 
        RobotTask t; 
        t.type = TaskType::WAIT_ART2_CAPTURE; t.param.capture.entity_id = id; t.param.capture.is_box = box; 
        return t; 
    }
    static RobotTask make_update_map(point start, point wall) { 
        RobotTask t; 
        t.type = TaskType::UPDATE_MAP_LOGIC; t.param.map_update.bomb_start = start; t.param.map_update.target_wall = wall; 
        return t; 
    }
};

} // namespace App::GameEngine