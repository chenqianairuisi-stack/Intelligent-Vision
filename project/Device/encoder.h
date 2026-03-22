#pragma once
#include <cstdint>
#include "system_config.h"

class EncoderArray {
public:
	static constexpr float PULSES_TO_SPEED_CM_S = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / (SystemConfig::PULSES_PER_REV * 0.02f);

	EncoderArray();
	void init();
	
	void update_encoders();
	
	int16 get_count(uint8 idx) const;                         // 获取指定编码器的计数值
	float get_speed_cm_s(uint8 idx) const;                    // 获取指定编码器的速度值，单位 cm/s
	const int16_t* getAllCounts() const { return counts; }    // 获取所有编码器计数的指针，供外部使用

private:
	int16 counts[4];  
};


extern EncoderArray encoders;