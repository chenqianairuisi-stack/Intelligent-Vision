#pragma once
#include <cstdint>
#include "system_config.h"


class EncoderArray {
public:
	EncoderArray() = default;
	void init();
	
	// PIT_CH1 中断调用，更新 20ms 增量计数值，供里程计使用（不再写轮速）
	void update_encoders_20ms_tick();

	// 快环(5ms)调用：用独立的 last 值算出当周期轮速，写入全局 current_wheel_speed
	void update_speed_5ms_tick();

	// 获取所有编码器计数的指针，供外部使用 (注：返回的是增量计数值)
	const int16_t* getAllCounts() const { return counts; }

private:
	// 从编码器脉冲转换到 cm/s 的系数 (按 5ms 快环周期计算)
	static constexpr float PULSES_TO_SPEED_CM_S = (2.0f * PI * SystemConfig::WHEEL_RADIUS) / (SystemConfig::PULSES_PER_REV * SystemConfig::SPEED_LOOP_DT_S);

	int16_t counts[4] = {0, 0, 0, 0};                           // 当前 20ms 周期的增量计数值 (顺序 LF, LB, RF, RB, 已乘上极性)，供里程计
	int32_t last_raw[4] = {0, 0, 0, 0};                         // 里程计用：硬件定时器上一次的绝对计数值
	int32_t last_raw_speed[4] = {0, 0, 0, 0};                   // 测速用：与里程计独立的上一次绝对计数值（5ms 节拍）
};


extern EncoderArray encoders;