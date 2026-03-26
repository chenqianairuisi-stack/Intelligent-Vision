#include "zf_common_headfile.h"
#include "encoder.h"

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
    { QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0,  QTIMER1_ENCODER1_CH2_C1,   1 },
    { QTIMER2_ENCODER2, QTIMER2_ENCODER2_CH1_C5,  QTIMER2_ENCODER2_CH2_C25,  1 },
    { QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3,  QTIMER2_ENCODER1_CH2_C4,  -1 }, 
    { QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2,  QTIMER1_ENCODER2_CH2_C24, -1 }
    
};


__attribute__((section(".dtcm_data"))) EncoderArray encoders;


void EncoderArray::init() {
    for (int i = 0; i < 4; ++i) {
        encoder_quad_init(ENC_CONFIGS[i].module, ENC_CONFIGS[i].pin_A, ENC_CONFIGS[i].pin_B);
        // 初始化历史值为当前定时器实际值
        last_raw[i] = encoder_get_count(ENC_CONFIGS[i].module);
    }
}


// 更新所有编码器的计数值，并清零计数器 (中断服务函数调用)
__attribute__((section(".ramfunc"))) void EncoderArray::update_encoders_20ms_tick() {
    
    for (int i = 0; i < 4; ++i) {
        // 读取当前绝对计数值
        int32_t current_raw = encoder_get_count(ENC_CONFIGS[i].module);
        
        // 补码无损求差，并乘以极性 (即使 current_raw 溢出，强转为 int16_t 后差值依然是绝对正确的)
        counts[i] = (int16_t)(current_raw - last_raw[i]) * ENC_CONFIGS[i].polarity;
        
        // 更新历史绝对值
        last_raw[i] = current_raw;
    }
}

// 获取指定编码器的计数值
__attribute__((section(".ramfunc"))) int16 EncoderArray::get_count(uint8_t idx) const {

    if(idx >= 4) { return 0; }
    return counts[idx];
}

// 获取指定编码器的速度值，单位 cm/s
__attribute__((section(".ramfunc"))) float EncoderArray::get_speed_cm_s(uint8_t idx) const {    

    if(idx >= 4) return 0.0f;
    return (float)counts[idx] * PULSES_TO_SPEED_CM_S;
}