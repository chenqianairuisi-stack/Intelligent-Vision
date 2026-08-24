/// \file icm42688.h
/// \brief ICM42688 惯性传感器驱动接口与寄存器配置
///
/// \details
/// 定义原始数据和物理量结构、SPI 引脚、寄存器地址及量程换算系数
/// 提供完整采样与单独陀螺仪或加速度计采样接口

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "zf_common_typedef.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#ifdef __cplusplus
}
#endif

// ===========================================================================================
// 采用右手坐标系:
// X轴：车头前方，Y轴：车身左侧，Z轴：车顶上方
// gyro 确定: 右手坐标系下，右手大拇指指向 X 轴正方向，四指弯曲的方向为陀螺仪正旋转方向 (右手定则)
// accel 确定: 水平时acc_z为正，车头朝下时acc_x为负，车身左侧朝下时acc_y为负
// ===========================================================================================

// imu 原始数据与物理量结构体
struct IMUData {
    int16_t raw_temp, raw_acc_x, raw_acc_y, raw_acc_z, raw_gyro_x, raw_gyro_y, raw_gyro_z;
    float temp;
    float acc_x, acc_y, acc_z;       // 单位: g
    float gyro_x, gyro_y, gyro_z;    // 单位: °/s
};

class Icm42688 {
public:
    IMUData data = {0};          // 存储原始数据和物理量的结构体

    Icm42688() = default;
    bool init();                 // icm42688 系统初始化
    void update_all();           // 读取所有数据
    void update_gyro_only();     // 读取陀螺仪数据 6 字节
    void update_accel_only();    // 读取加速度计数据 6 字节

private:
    inline void cs_low()  { gpio_low(cs_pin);  }
    inline void cs_high() { gpio_high(cs_pin); }

private:
    // ==== 硬件 SPI 引脚配置 ====
    static constexpr spi_index_enum    spi_n     = SPI_4;         // SPI 端口号
    static constexpr spi_sck_pin_enum  sck_pin   = SPI4_SCK_C23;  // SPI 时钟引脚
    static constexpr spi_mosi_pin_enum mosi_pin  = SPI4_MOSI_C22; // SPI 主输出从输入引脚
    static constexpr spi_miso_pin_enum miso_pin  = SPI4_MISO_C21; // SPI 主输入从输出引脚
    static constexpr gpio_pin_enum     cs_pin    = C20;           // 独立的 CS 引脚 (不使用 SPI 内置的 CS 功能)

    // ==== ICM-42688 核心寄存器地址 ====
    static constexpr uint8_t SPI_W              = 0x00;
    static constexpr uint8_t SPI_R              = 0x80;
    static constexpr uint8_t REG_DEVICE_CONFIG  = 0x11;  // 配置寄存器
    static constexpr uint8_t REG_TEMP_DATA1     = 0x1D;  // 温度寄存器 (2 bytes) [起始数据突发读取的首地址 (14 bytes 连续读取包含温度、加速度、陀螺仪数据)]
    static constexpr uint8_t REG_ACCEL_DATA_X1  = 0x1F;  // 加速度寄存器起始 (6 bytes)
    static constexpr uint8_t REG_GYRO_DATA_X1   = 0x25;  // 陀螺仪寄存器起始 (6 bytes)
    static constexpr uint8_t REG_PWR_MGMT0      = 0x4E;  // 电源管理寄存器 
    static constexpr uint8_t REG_GYRO_CONFIG0   = 0x4F;  // 陀螺仪配置寄存器
    static constexpr uint8_t REG_ACCEL_CONFIG0  = 0x50;  // 加速度计配置寄存器
    static constexpr uint8_t REG_WHO_AM_I       = 0x75;  // ID寄存器
    static constexpr uint8_t REG_BANK_SEL       = 0x76;  

    // ==== 物理量换算系数 ====
    static constexpr float ACCEL_SCALE = 1.0f / 4096.0f; // ±8g
    static constexpr float GYRO_SCALE  = 1.0f / 16.4f;   // ±2000 dps
    static constexpr float TEMP_SCALE  = 1.0f / 132.48f; // 温度系数
    static constexpr float TEMP_OFFSET = 25.0f;          // 温度偏移
};

extern Icm42688 imu_icm42688;






//加速度计配置：
// BIT NAME FUNCTION 
// 7:5 ACCEL_FS_SEL 
// Full scale select for accelerometer UI interface output 
// 000: ±16g (default) 
// 001: ±8g 
// 010: ±4g 
// 011: ±2g 
// 100: Reserved 
// 101: Reserved 
// 110: Reserved 
// 111: Reserved 
// 4 - Reserved 
// 3:0 ACCEL_ODR 
// Accelerometer ODR selection for UI interface output 
// 0000: Reserved 
// 0001: 32kHz (LN mode) 
// 0010: 16kHz (LN mode) 
// 0011: 8kHz (LN mode) 
// 0100: 4kHz (LN mode) 
// 0101: 2kHz (LN mode) 
// 0110: 1kHz (LN mode) (default) 
// 0111: 200Hz (LP or LN mode)  
// 1000: 100Hz (LP or LN mode) 
// 1001: 50Hz (LP or LN mode) 
// 1010: 25Hz (LP or LN mode) 
// 1011: 12.5Hz (LP or LN mode) 
// 1100: 6.25Hz (LP mode) 
// 1101: 3.125Hz (LP mode) 
// 1110: 1.5625Hz (LP mode) 
// 1111: 500Hz (LP or LN mode) 
// ACCEL_FS_SEL 
// Full scale select for accelerometer UI interface output 
// 000: ±16g (default) 
// 001: ±8g 
// 010: ±4g 
// 011: ±2g 
// 100: Reserved 
// 101: Reserved 
// 110: Reserved 
// 111: Reserved 
// Sensitivity Scale Factor 
// ACCEL_FS_SEL =0  2,048  LSB/g 2 
// ACCEL_FS_SEL =1  4,096  LSB/g 2 
// ACCEL_FS_SEL =2  8,192  LSB/g 2 
// ACCEL_FS_SEL =3  16,384  LSB/g 2 

//陀螺仪配置：
// BIT NAME FUNCTION 
// 7:5 GYRO_FS_SEL 
// Full scale select for gyroscope UI interface output 
// 000: ±2000dps (default) 
// 001: ±1000dps 
// 010: ±500dps 
// 011: ±250dps 
// 100: ±125dps 
// 101: ±62.5dps 
// 110: ±31.25dps 
// 111: ±15.625dps 
// 4 - Reserved 
// 3:0 GYRO_ODR 
// Gyroscope ODR selection for UI interface output 
// 0000: Reserved 
// 0001: 32kHz 
// 0010: 16kHz 
// 0011: 8kHz 
// 0100: 4kHz 
// 0101: 2kHz 
// 0110: 1kHz (default) 
// 0111: 200Hz  
// 1000: 100Hz 
// 1001: 50Hz 
// 1010: 25Hz 
// 1011: 12.5Hz 
// 1100: Reserved 
// 1101: Reserved 
// 1110: Reserved 
// 1111: 500Hz 
// GYRO_FS_SEL 
// Full scale select for gyroscope UI interface output 
// 000: ±2000dps (default) 
// 001: ±1000dps 
// 010: ±500dps 
// 011: ±250dps 
// 100: ±125dps 
// 101: ±62.5dps 
// 110: ±31.25dps 
// 111: ±15.625dps 
// Sensitivity Scale Factor 
// GYRO_FS_SEL=0  16.4  LSB/(º/s) 2 
// GYRO_FS_SEL =1  32.8  LSB/(º/s) 2 
// GYRO_FS_SEL =2  65.5  LSB/(º/s) 2 
// GYRO_FS_SEL =3  131  LSB/(º/s) 2 
// GYRO_FS_SEL =4  262  LSB/(º/s) 2 
// GYRO_FS_SEL =5  524.3  LSB/(º/s) 2 
// GYRO_FS_SEL =6  1048.6  LSB/(º/s) 2 
// GYRO_FS_SEL =7  2097.2  LSB/(º/s) 2
