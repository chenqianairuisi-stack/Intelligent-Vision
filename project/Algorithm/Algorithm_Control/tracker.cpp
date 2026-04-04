#include "tuning_config.h"
#include "tracker.h"
#include "odometry.h" // 依赖你的里程计获取当前物理坐标

// 放入 DTCM 加速内存访问
__attribute__((section(".dtcm_data"))) PathTracker path_tracker;

PathTracker::PathTracker() : current_wp_idx(0), state(TrackerState::NONE) {}

// 载入并压缩网格路径 (自动合并共线的直线段)，并转换为物理坐标系
void PathTracker::load_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path) {
    grid_path.clear();                 // 清空旧网格路径
    physical_path.clear();             // 清空旧物理路径
    state = TrackerState::FINISHED;

    if (raw_path.size() == 0) return;

    // 路径点极少，直接照搬
    if (raw_path.size() <= 2) grid_path = raw_path;
    else {
        // 保留起点
        grid_path.push_back(raw_path[0]);
        
        // 遍历中间节点，提取拐点
        for (size_t i = 1; i < raw_path.size() - 1; ++i) {

            int dx1 = raw_path[i].x - raw_path[i - 1].x;
            int dy1 = raw_path[i].y - raw_path[i - 1].y;
            int dx2 = raw_path[i + 1].x - raw_path[i].x;
            int dy2 = raw_path[i + 1].y - raw_path[i].y;

            if ((dx1 * dy2) != (dx2 * dy1)) {
                grid_path.push_back(raw_path[i]);
            }
        }
        // 保留终点
        grid_path.push_back(raw_path.back());
    }

    // 将格子转为物理坐标 Point2D
    auto to_physical =[](const point& p) -> Point2D {
        return { p.x * GRID_SIZE_CM + MAP_OFFSET_X, 
                 p.y * GRID_SIZE_CM + MAP_OFFSET_Y };
    };

    // 生成物理坐标路径
    for (size_t i = 0; i < grid_path.size(); ++i) {
        physical_path.push_back(to_physical(grid_path[i]));
    }

    // 索引 0 是小车当前所在的起点，所以直接去追索引 1（下一个拐点或终点）
    current_wp_idx = (grid_path.size() > 1) ? 1 : 0; 

    state = TrackerState::TRACKING;
}


__attribute__((section(".ramfunc"))) Pose2D PathTracker::update_and_get_target(const Point2D& current_pos) {
    // 没在循迹就返回最后一次的目标
    if (state != TrackerState::TRACKING) { return current_target;}

    // 获取当前格子物理坐标
    Point2D target_phys = physical_path[current_wp_idx];

    // 计算距离平方
    float dx = target_phys.x - current_pos.x;
    float dy = target_phys.y - current_pos.y;
    float dist_sq = dx * dx + dy * dy;

    // 切弯半径选择：终点切弯半径更小(1 cm)，保证停稳；中途切弯半径较大(10 cm)，保证平滑
    bool is_last_point = (current_wp_idx == physical_path.size() - 1);
    float current_radius = is_last_point ? tune.tracker.reach_radius_min : tune.tracker.reach_radius;

    // 状态切换判断
    if (dist_sq <= current_radius * current_radius) {
        current_wp_idx++;
        if (current_wp_idx >= physical_path.size()) {
            state = TrackerState::FINISHED;
        } else {
            target_phys = physical_path[current_wp_idx];
        }
    }

    // 组装并输出纯净的数学结果（不带任何控制逻辑的位姿），供控制模块调用
    current_target.x = target_phys.x;
    current_target.y = target_phys.y;
    current_target.yaw = ENTRY_YAW;        // 目前不需要航向信息，保持初始角度即可

    return current_target;
}