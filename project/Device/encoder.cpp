/// \file encoder.cpp
/// \brief 四轮正交编码器采样与轮速换算实现
///
/// \details
/// 绑定四个轮位的 QTIMER 模块、引脚和安装极性
/// 分别维护 20 ms 里程计增量与 5 ms 速度内环历史值，并把轮速写入全局物理状态

#include "zf_common_headfile.h"
#include "Encoder.h"
#include "RobotState.h"

// 配置结构体，用于绑定硬件模块和引脚
struct EncoderHWConfig {
    encoder_index_enum     module;     // 编码器模块编号
    encoder_channel1_enum  pin_A;      // A相引脚
    encoder_channel2_enum  pin_B;      // B相引脚
    int8_t                 polarity;   // 极性 (1/-1)
};

// 集中配置字典，方便统一管理和修改编码器的硬件连接和极性
static constexpr EncoderHWConfig ENC_CONFIGS[4] = {
    // 轮位映射：按照顺序，分别对应 LF, LB, RF, RB
    { QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2,  QTIMER1_ENCODER2_CH2_C24,  1 },
    { QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3,  QTIMER2_ENCODER1_CH2_C4,   1 }, 
    { QTIMER2_ENCODER2, QTIMER2_ENCODER2_CH1_C5,  QTIMER2_ENCODER2_CH2_C25, -1 },
    { QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0,  QTIMER1_ENCODER1_CH2_C1,  -1 },
};


DTCM_DATA EncoderArray encoders;


/// \brief 初始化四路正交编码器
///
/// \details
/// 初始化后立即记录当前硬件计数，避免第一次 20ms 更新时把历史残留当成真实增量
///
void EncoderArray::init() {
    for (int i = 0; i < 4; ++i) {
        encoder_quad_init(ENC_CONFIGS[i].module, ENC_CONFIGS[i].pin_A, ENC_CONFIGS[i].pin_B);
        // 初始化历史值为当前定时器实际值（里程计与测速各一份，互不干扰）
        int32_t raw = encoder_get_count(ENC_CONFIGS[i].module);
        last_raw[i] = raw;
        last_raw_speed[i] = raw;
    }
}


/// \brief 20ms 周期更新四轮编码器增量（供里程计）
///
/// \details
/// 使用 16 位补码差值处理计数器回绕，再按轮位极性修正方向
/// 只写入 counts 供里程计积分；轮速测量已移到 5ms 快环 update_speed_5ms_tick
///
__attribute__((section(".ramfunc")))
void EncoderArray::update_encoders_20ms_tick() {
    for (int i = 0; i < 4; ++i) {
        // 读取当前绝对计数值
        int32_t current_raw = encoder_get_count(ENC_CONFIGS[i].module);

        // 补码无损求差，并乘以极性 (即使 current_raw 溢出，强转为 int16_t 后差值依然是绝对正确的)
        // 仅供里程计使用；轮速已移到 5ms 快环 update_speed_5ms_tick
        counts[i] = (int16_t)(current_raw - last_raw[i]) * ENC_CONFIGS[i].polarity;

        // 更新历史绝对值
        last_raw[i] = current_raw;
    }
}


/// \brief 5ms 快环测速：用与里程计独立的历史值求 5ms 增量并转成 cm/s
///
/// \details
/// 与 update_encoders_20ms_tick 各自维护一份 last_raw，读同一个自由运行的硬件计数器互不影响
/// 结果写入全局 current_wheel_speed，供 200Hz 速度内环闭环
///
__attribute__((section(".ramfunc")))
void EncoderArray::update_speed_5ms_tick() {
    auto& wheel_speed = App::g_state.physical.current_wheel_speed;

    for (int i = 0; i < 4; ++i) {
        int32_t current_raw = encoder_get_count(ENC_CONFIGS[i].module);
        int16_t delta = (int16_t)(current_raw - last_raw_speed[i]) * ENC_CONFIGS[i].polarity;
        float speed_cm_s = (float)delta * PULSES_TO_SPEED_CM_S;
        switch (i) {
            case 0: wheel_speed.lf = speed_cm_s; break;
            case 1: wheel_speed.lb = speed_cm_s; break;
            case 2: wheel_speed.rf = speed_cm_s; break;
            case 3: wheel_speed.rb = speed_cm_s; break;
        }
        last_raw_speed[i] = current_raw;
    }
}
