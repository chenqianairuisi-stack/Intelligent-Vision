#pragma once
#include <cstddef>
#include "system_config.h"
#include "RobotState.h"

namespace Algorithm::Tracker {

    // 载入网格路径，并启动跟踪
    void load_path(const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& raw_path);
    // 载入网格路径，并用真实逻辑起点辅助首段压缩
    void load_path(const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& raw_path, point start_grid);
    void load_sokoban_path(const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& raw_path,
                           point start_grid,
                           const SokobanLevel& level);
    void load_box_push_path(const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& raw_path,
                            point start_grid,
                            point box_start,
                            point box_target,
                            const SokobanLevel& level);
    void load_bomb_push_path(const StaticArray<point, SystemConfig::MAX_PATH_LENGTH>& raw_path,
                             point start_grid,
                             point bomb_target,
                             const SokobanLevel& level);
    void set_box_push_final_press_cm(float press_cm);
    float get_box_push_final_press_cm();

    // 更新当前目标位姿，供控制模块调用
    void update_target();

    // 运动中的视觉辅助，POINT_TRACKING 也可复用
    void update_vision_assist(const Point2D& target);
    // 15ms 视觉修正节拍（PIT_CH2 中断调用）：停车/保持冻结、起步取新帧，内部含全部纠偏门控。
    void vision_correction_tick();
    // 炸弹爆炸窗口期屏蔽视觉修正：闪光会污染视觉坐标，此期间只靠锁位+里程计扛过
    void set_vision_correction_suppressed(bool suppressed);
    void track_point(const Pose2D& target);
    void track_point_with_vision_assist(const Pose2D& target);
    // 只改变车头方向，底盘不做位置伺服
    void track_yaw(float target_yaw);

    // 检查是否到达当前目标点
    bool check_arrival(Point2D target, float radius); 
    // 停止循迹
    inline void stop() { App::g_state.control.tracker_state = TrackerState::NONE; }
}
