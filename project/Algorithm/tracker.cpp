#include "tuning_config.h"
#include "tracker.h"
#include "odometry.h" // 依赖你的里程计获取当前物理坐标

// 放入 DTCM 加速内存访问
__attribute__((section(".dtcm_data"))) PathTracker path_tracker;

PathTracker::PathTracker() : current_wp_idx(0), state(TrackerState::NONE) {}

// 载入并压缩路径 (自动合并共线的直线段)
void PathTracker::load_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path) {
    grid_path.clear();    // 清空旧路径

    if (raw_path.size() == 0) {
        state = TrackerState::FINISHED;
        return;
    }

    // 路径点极少，直接照搬
    if (raw_path.size() <= 2) {
        grid_path = raw_path;
    } else {
        // 保留起点
        grid_path.push_back(raw_path[0]);
        
        // 遍历中间节点，提取拐点
        for (size_t i = 1; i < raw_path.size() - 1; ++i) {

            int dx1 = raw_path[i].x - raw_path[i - 1].x;
            int dy1 = raw_path[i].y - raw_path[i - 1].y;
            int dx2 = raw_path[i + 1].x - raw_path[i].x;
            int dy2 = raw_path[i + 1].y - raw_path[i].y;

            if (dx1 != dx2 || dy1 != dy2) {
                grid_path.push_back(raw_path[i]);
            }
        }
        
        // 保留终点
        grid_path.push_back(raw_path.back());
    }

    // 索引 0 是小车当前所在的起点，所以直接去追索引 1（下一个拐点或终点）
    current_wp_idx = (grid_path.size() > 1) ? 1 : 0; 
    
    state = TrackerState::TRACKING;
}


__attribute__((section(".ramfunc"))) Pose2D PathTracker::update_and_get_target(const Point2D& current_pos) {
    if (state != TrackerState::TRACKING) {
        return current_target;    // 没在循迹就返回最后一次的目标
    }

    // 获取当前格子物理坐标
    point target_grid = grid_path[current_wp_idx];
    float target_x = target_grid.x * GRID_SIZE_CM + MAP_OFFSET_X;
    float target_y = target_grid.y * GRID_SIZE_CM + MAP_OFFSET_Y;

    // 计算距离平方
    float dx = target_x - current_pos.x;
    float dy = target_y - current_pos.y;
    float dist_sq = dx * dx + dy * dy;

    bool is_last_point = (current_wp_idx == grid_path.size() - 1);
    float current_radius = is_last_point ? tune.tracker.reach_radius_min : tune.tracker.reach_radius;   // 终点 2 cm，中途切弯 6 cm

    // 状态切换判断
    if (dist_sq <= current_radius * current_radius) {
        current_wp_idx++;
        if (current_wp_idx >= grid_path.size()) {
            state = TrackerState::FINISHED;
        } else {
            target_grid = grid_path[current_wp_idx];
            target_x = target_grid.x * GRID_SIZE_CM + MAP_OFFSET_X;
            target_y = target_grid.y * GRID_SIZE_CM + MAP_OFFSET_Y;
        }
    }

    // 组装并输出纯净的数学结果（不带任何控制逻辑的位姿），供控制模块调用
    current_target.x = target_x;
    current_target.y = target_y;
    current_target.yaw = 0.0f; // 目前不需要航向信息，保持 0 即可

    return current_target;
}