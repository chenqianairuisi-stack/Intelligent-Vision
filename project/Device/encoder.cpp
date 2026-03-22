#include "zf_common_headfile.h"
#include "encoder.h"


#define ENCODER_1                   (QTIMER1_ENCODER1)
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER2_ENCODER2)
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)


__attribute__((section(".dtcm_data"))) EncoderArray encoders;


EncoderArray::EncoderArray() {
    for(uint8 i = 0; i < 4; ++i) {
        counts[i] = 0;
    }
}

void EncoderArray::init() {
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);  
}


// 获取指定编码器的计数值
__attribute__((section(".ramfunc"))) int16 EncoderArray::get_count(uint8 idx) const {

    if(idx >= 4) { return 0; }
    return counts[idx];
}


// 获取指定编码器的速度值，单位 cm/s
__attribute__((section(".ramfunc"))) float EncoderArray::get_speed_cm_s(uint8 idx) const {    

    return (float)counts[idx] * PULSES_TO_SPEED_CM_S;
}


// 更新所有编码器的计数值，并清零计数器 (中断服务函数调用)
__attribute__((section(".ramfunc"))) void EncoderArray::update_encoders() {
    
    counts[0] = encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);

    counts[1] = encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);

    counts[2] = encoder_get_count(ENCODER_3);
    encoder_clear_count(ENCODER_3);

    counts[3] = encoder_get_count(ENCODER_4);
    encoder_clear_count(ENCODER_4);  
}