#pragma once
#include "system_config.h"
#include "CoreScheduler.h"
#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>

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

    // 视觉异步标定状态，供 GameManage 非阻塞推进
    enum class AsyncCalibState {
        IDLE,       // 空闲/刚复位
        BUSY,       // 正在采集中
        SUCCESS,    // 收敛并已覆盖位姿
        ERROR       // 超时或偏差过大
    };

    // 视觉标定器类：收集窗口数据并判断视觉结果是否稳定
    class VisionCalibrator {
    public:
        // 配置参数
        static constexpr size_t WINDOW_SIZE = 10;     // 收集10帧视觉数据
        static constexpr float MAX_VARIANCE = 1.0f;   // 允许的最大方差(cm^2)，越小要求画面越稳定

        void reset() {
            m_count = 0;
            m_start_tick = Core::Scheduler::get_sys_tick_ms();
            m_converged = false;
        }

        bool is_timed_out(uint32_t timeout_ms) const {
            return (Core::Scheduler::get_sys_tick_ms() - m_start_tick) > timeout_ms;
        }

        // 压入视觉传感器传来的绝对坐标
        __attribute__((section(".ramfunc"))) 
        void push(float x, float y, float yaw) {
            if (m_converged) return; // 收敛后不再接收，直到下次 reset
            
            m_buffer_x[m_count % WINDOW_SIZE] = x;
            m_buffer_y[m_count % WINDOW_SIZE] = y;
            m_buffer_yaw[m_count % WINDOW_SIZE] = yaw;
            m_count++;

            if (m_count >= WINDOW_SIZE) {
                check_convergence();
            }
        }

        // 重置异步函数的内部状态（在每次需要开始一次新标定时调用）

        [[nodiscard]] bool is_converged() const { return m_converged; }
        [[nodiscard]] size_t get_count() const { return m_count; }
        // 获取滤波后的最佳位姿
        [[nodiscard]] Pose2D get_optimal_pose() const { return m_optimal_pose; }

    private:
        std::array<float, WINDOW_SIZE> m_buffer_x{};
        std::array<float, WINDOW_SIZE> m_buffer_y{};
        std::array<float, WINDOW_SIZE> m_buffer_yaw{};
        
        size_t m_count = 0;
        uint32_t m_start_tick = 0; // 私有化时间戳，不被外部污染
        bool m_converged = false;
        Pose2D m_optimal_pose{};

        // 核心算法：截尾均值与方差校验
        __attribute__((section(".ramfunc"))) 
        void check_convergence() {
            // 拷贝数据以便排序（防止破坏时序窗口）
            std::array<float, WINDOW_SIZE> sorted_x = m_buffer_x;
            std::array<float, WINDOW_SIZE> sorted_y = m_buffer_y;
            
            std::sort(sorted_x.begin(), sorted_x.end());
            std::sort(sorted_y.begin(), sorted_y.end());

            // 剔除突变离群点 (最高和最低的 20%)
            constexpr size_t trim_count = WINDOW_SIZE / 5; 
            constexpr size_t valid_count = WINDOW_SIZE - 2 * trim_count;

            float sum_x = 0.0f, sum_y = 0.0f;
            for (size_t i = trim_count; i < WINDOW_SIZE - trim_count; ++i) {
                sum_x += sorted_x[i];
                sum_y += sorted_y[i];
            }

            float mean_x = sum_x / valid_count;
            float mean_y = sum_y / valid_count;

            // 计算有效数据的方差
            float var_x = 0.0f, var_y = 0.0f;
            for (size_t i = trim_count; i < WINDOW_SIZE - trim_count; ++i) {
                var_x += (sorted_x[i] - mean_x) * (sorted_x[i] - mean_x);
                var_y += (sorted_y[i] - mean_y) * (sorted_y[i] - mean_y);
            }
            var_x /= valid_count;
            var_y /= valid_count;

            // 如果方差满足要求，说明画面已经稳定，识别结果可信
            if (var_x < MAX_VARIANCE && var_y < MAX_VARIANCE) {
                m_converged = true;
                m_optimal_pose.x = mean_x;
                m_optimal_pose.y = mean_y;                
            }
        };
    };

    // 重置异步函数的内部状态（在每次需要开始一次新标定时调用）
    void reset_async_calibrate();

    // 异步非阻塞视觉标定函数
    AsyncCalibState async_calibrate_vision(uint32_t timeout_ms = 2000, float reject_threshold = 4.0f);

    // 初始化与标定
    void init();
    void calibrate_gyro_step(); // 返回 true 表示标定完成

    // 强行重置里程计坐标
    void set_position(float x, float y, float yaw_deg);

    // Apply latency-compensated vision correction for the current straight segment.
    bool apply_vision_axis_correction(const Point2D& segment_start, const Point2D& segment_end,
                                      uint32_t& last_consumed_seq,
                                      bool allow_near_target_correction = false);

    // 主循环在每个稳定视觉帧上调用，喂入帧间位移以驱动视觉侧拐点检测与实时延时估计
    void notify_vision_inflection(float dx, float dy, uint32_t gap_ms);

    struct VisionLatencyDebug {
        Pose2D raw_pose;
        Pose2D compensated_pose;
        float odom_dx;
        float odom_dy;
        float correction_x;
        float correction_y;
        uint32_t receive_tick_ms;
        uint32_t capture_tick_ms;
        bool valid;

        // 拐点对齐实时延时估计 (foundation 阶段新增)
        float est_raw_l_ms;             // 最近一次成功配对得到的原始 L
        float est_filt_l_ms;            // 滤波后的 L
        float used_l_ms;                // 本帧补偿实际采用的 L（回退梯结果）
        uint32_t est_last_match_tick_ms;// 最近一次成功配对的时刻
        uint8_t  est_pending_count;     // 待配对编码器拐点数
        bool est_locked;               // 是否已锁定过有效 L
    };

    const VisionLatencyDebug& get_vision_latency_debug();

    // 两个高频中断钩子
    void update_yaw_1ms_tick();
    void update_position_20ms_tick(const int16_t* encoder_counts, float current_yaw_deg);

    
    

}
