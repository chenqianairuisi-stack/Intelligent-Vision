#pragma once
#include <cstdint>
#include "system_config.h"


class EncoderArray {
public:
	EncoderArray() = default;
	void init();
	
	void update_encoders_20ms_tick();
	
	int16_t get_count(uint8_t idx) const;                         // 获取指定编码器的计数值
	float get_speed_cm_s(uint8_t idx) const;                    // 获取指定编码器的速度值，单位 cm/s
	const int16_t* getAllCounts() const { return counts; }      // 获取所有编码器计数的指针，供外部使用

private:
	// 从编码器脉冲转换到 cm/s 的系数 (假设 20ms 更新一次)
	static constexpr float PULSES_TO_SPEED_CM_S = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / (SystemConfig::PULSES_PER_REV * 0.02f);  

	int16_t counts[4] = {0, 0, 0, 0};                             // 当前周期的增量计数值 (顺序 LF, LB, RF, RB, 已乘上极性)
	int32_t last_raw[4] = {0, 0, 0, 0};                         // 硬件定时器上一次的绝对计数值 (顺序 LF, LB, RF, RB)
};


extern EncoderArray encoders;