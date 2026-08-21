#pragma once

#include "system_config.h"

namespace Subsystem::Chassis {
    void init();

    void update_20ms_tick();  // PIT_CH1(20ms) 慢环：规划/Tracker/yaw/逆运动学，产出四轮目标速度

    void update_speed_loop_5ms();  // 5ms 快环(200Hz)：测速后跑轮速 PID 出占空比驱动电机

    void check_is_stopped();

    /// \brief 启动原地连续旋转
    /// \param degrees 旋转角度，正值为逆时针
    ///
    /// \details
    /// 底盘会累计解包后的 yaw，避免 360 度目标被最短角度控制折叠为零误差
    void start_continuous_spin(float degrees);

    /// \brief 查询连续旋转是否已完成并停稳
    bool is_continuous_spin_finished();

    // 获取慢环最近发布的四轮目标速度快照，供遥测使用
    WheelSpeed4 get_target_wheel_speeds();
}
