#include "tuning_config.h"
#include "Tracker.h"
#include "RobotState.h"

using namespace SystemConfig;

namespace Algorithm::Tracker {

// 载入并压缩网格路径 (自动合并共线的直线段)，并转换为物理坐标系
void load_path(const StaticArray<point, MAX_PATH_LENGTH>& raw_path) {
    auto& plan = App::g_state.planning;
    auto& ctrl = App::g_state.control;

    plan.grid_path.clear();           
    plan.physical_path.clear();      
    ctrl.tracker_state = TrackerState::FINISHED;

    if (raw_path.size() == 0) return;

    // 路径点极少，直接照搬
    if (raw_path.size() <= 2) plan.grid_path = raw_path;
    else {
        // 保留起点
        plan.grid_path.push_back(raw_path[0]);
        
        // 遍历中间节点，提取拐点
        for (size_t i = 1; i < raw_path.size() - 1; ++i) {

            int dx1 = raw_path[i].x - raw_path[i - 1].x;
            int dy1 = raw_path[i].y - raw_path[i - 1].y;
            int dx2 = raw_path[i + 1].x - raw_path[i].x;
            int dy2 = raw_path[i + 1].y - raw_path[i].y;

            // 共线且同向的点不保留，其他点都保留（包括拐点和回头点）
            if (((dx1 * dy2) != (dx2 * dy1)) || ((dx1 * dx2 + dy1 * dy2) < 0)) {
                plan.grid_path.push_back(raw_path[i]);
            }
        }
        // 保留终点
        plan.grid_path.push_back(raw_path.back());
    }

    // 将格子转为物理坐标 Point2D
    auto to_physical =[](const point& p) -> Point2D {
        return { p.x * GRID_SIZE_CM + MAP_OFFSET_X, 
                 p.y * GRID_SIZE_CM + MAP_OFFSET_Y };
    };

    // 生成物理坐标路径
    for (size_t i = 0; i < plan.grid_path.size(); ++i) {
        plan.physical_path.push_back(to_physical(plan.grid_path[i]));
    }

    // 索引 0 是小车当前所在的起点，所以直接去追索引 1（下一个拐点或终点）
    plan.current_wp_idx = 0;

    ctrl.tracker_state = TrackerState::TRACKING;
}


// 更新跟踪状态并获取当前目标位姿，供控制模块调用
__attribute__((section(".ramfunc"))) void update_target() {
    auto& plan = App::g_state.planning;
    auto& ctrl = App::g_state.control;

    // 没在循迹就返回最后一次的目标
    if (ctrl.tracker_state != TrackerState::TRACKING) return;

    // 获取当前目标物理坐标
    Point2D target_phys = plan.physical_path[plan.current_wp_idx];

    // 切弯半径选择：如果是最后一个点了，就用更小的半径要求，防止越过终点；否则用正常的半径要求
    bool is_last_point = (plan.current_wp_idx == plan.physical_path.size() - 1);
    float current_radius = is_last_point ? tune.tracker.reach_radius_min : tune.tracker.reach_radius;

    // 状态切换判断
    if (check_arrival(target_phys, current_radius)) {
        if (!is_last_point) {
            // 中间点：直接切到下一个点
            plan.current_wp_idx++;
            target_phys = plan.physical_path[plan.current_wp_idx];
        } else {
            // 终点：不仅要求距离足够近，还必须等待底盘物理静止            
            if (App::g_state.physical.is_stopped) {
                ctrl.tracker_state = TrackerState::FINISHED;
            }
        }
    }

    // 组装并输出纯净的数学结果，供控制模块调用
    ctrl.current_target.x = target_phys.x;
    ctrl.current_target.y = target_phys.y;
}


// 检查是否到达当前目标点
bool check_arrival(Point2D target, float radius) {
    auto& current_pos = App::g_state.physical.pose;

    float dx = target.x - current_pos.x;
    float dy = target.y - current_pos.y;
    float dist_sq = dx * dx + dy * dy;

    return dist_sq <= radius * radius;
}

} // namespace Algorithm::Tracker