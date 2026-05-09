#pragma once
#include <cstdint>

// ===========================================================================================
// 采用右手坐标系:
// X轴：车头前方，Y轴：车身左侧，Z轴：车顶上方
// gyro 确定: 右手坐标系下，右手大拇指指向 X 轴正方向，四指弯曲的方向为陀螺仪正旋转方向 (右手定则)
// accel 确定: 水平时acc_z为正，车头朝下时acc_x为负，车身左侧朝下时acc_y为负
// ===========================================================================================

namespace Subsystem::PoseEstimator {
    // 用于 Vofa+ 波形分析的 6 轴滤波上帝视角探针
    struct DebugProbes {
        float pitch_acc;      // 纯加速度推算的 Pitch 
        float pitch_gyro;     // 纯陀螺仪积分的 Pitch 
        float pitch_mahony;   // 四元数解算的 Pitch  
        float kp_adaptive;    // 动态 Kp 的当前值     
        float acc_norm;       // 加速度模长 (1G 附近) 
    };

    // 获取探针数据的接口
    const DebugProbes& get_debug_probes();

    // 初始化与标定
    void init();
    void calibrate_gyro_step(); // 返回 true 表示标定完成

    // 强行重置里程计坐标
    void set_position(float x, float y, float yaw_deg);

    // 两个高频中断钩子
    void update_yaw_1ms_tick();
    void update_position_20ms_tick(const int16_t* encoder_counts, float current_yaw_deg);
}