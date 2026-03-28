#pragma once

// 可复用 PID 参数块
struct PidParams {
    float kp;
    float ki;
    float kd;
};

// 集中管理全车所有可调参数
struct TuningConfig {
    PidParams pid_yaw;
    PidParams pid_speed;

    struct {
        float max_speed;                     // 跟踪时的最大线速度 (cm/s)
        float max_acc;                       // 跟踪时的最大加速度 (cm/s^2)
        float max_jerk;                      // 跟踪时的最大加加速度 (cm/s^3)
        float max_ang_speed;                 // 跟踪时的最大角速度 (rad/s)
    }dynamics;

    struct {
        float reach_radius;
        float reach_radius_min;
    } tracker;

};

// 全局调参实例，放在 DTCM 区域，供所有模块访问
__attribute__((section(".dtcm_data"))) inline TuningConfig tune {
    {2.5f, 0.0f, 0.8f},                        // pid_yaw (小车身 Yaw 极易受干扰，适当保留 Kd 抵抗旋转惯性)
    {1.2f, 0.5f, 0.0f},                        // pid_speed
    
    // Dynamics 动力学预测参数
    {
        100.0f,    // max_speed: 1m/s，极速过弯
        250.0f,    // max_acc: 0.25G 极限抓地力
        2500.0f,   // t_acc_jerk: 0.1秒起步柔化
        4.0f       // max_ang_speed: 约 230度/秒，旋转敏捷
    },
    
    // Tracker 几何预测参数
    {
        10.0f,     // reach_radius: 10cm 切弯
        1.5f       // reach_radius_min: 终点停稳极小宽容度
    }
};




// ### 🕹️ 第三部分：赛场 3 步调参法 (Tuning Guide)

// 有了这套极度科学的参数和底层 S 曲线算法，你的调参将不再是“玄学试错”，而是严谨的“剥洋葱法”。请严格按照以下 3 步在场地测试：

// **第一步：测定物理抓地极限 (`max_acc` 调参)**
// *   **方法**：先随便跑一段直线的路径点。把 `t_acc_jerk` 设得非常小（比如 0.01s，相当于关闭 S 曲线），`max_speed` 给 100。
// *   **操作**：不断增大 `max_acc` (从 100 开始加，每次加 50)。
// *   **观察**：当你发现小车在起步瞬间或者急刹车瞬间，**轮胎发出了“呲呲”的打滑声，或者车体有轻微侧偏**，这就说明你突破了 10x20cm 底盘的静摩擦力极限！
// *   **锁定**：把出现打滑时的值**乘以 0.85**（留出灰尘和电量衰减裕度），这就是你这台车的**天命 `max_acc`**，以后永远不要动它。

// **第二步：测定底盘悬挂柔性 (`t_acc_jerk` 调参)**
// *   **方法**：保持上一步测出的天命 `max_acc`。现在车不打滑了，但是起步/刹车会“硬邦邦”的，车身（特别是上方的摄像头）会猛烈颤抖。
// *   **操作**：逐渐增大 `t_acc_jerk` (从 0.02 往上加，0.05, 0.08, 0.10...)。
// *   **观察**：盯住车体顶部的 OpenART 摄像头。当参数加到一个临界点时，你会发现**小车起步瞬间如同德芙般丝滑，摄像头的果冻效应完全消失**。
// *   **锁定**：此时的值就是完美的柔化时间（通常在 0.08s ~ 0.15s 之间）。

// **第三步：测定几何切弯流线 (`reach_radius` 调参)**
// *   **方法**：在屏幕上画一个 90度 直角弯的 Mock 地图。
// *   **操作**：增大 `reach_radius`（从 5cm 往上加）。
// *   **观察**：这个参数直接决定了小车是“走折线”还是“跑赛车线”。半径越大，切入直角越早，速度掉得越少。如果你发现小车切弯时碰到了虚拟箱子或者偏离中心线太多，就减小它。一般 8~12cm 是最佳甜点。

