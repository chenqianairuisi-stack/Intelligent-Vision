/// \file icm42688.cpp
/// \brief ICM42688 初始化、采样与物理量换算实现
///
/// \details
/// 通过 SPI 手动片选配置传感器并突发读取温度、加速度和陀螺仪数据
/// 将寄存器原始值转换到项目统一的车体右手坐标系，供姿态估计与标定使用

#include "Icm42688.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "zf_driver_delay.h"
#include "zf_common_debug.h"
#ifdef __cplusplus
}
#endif

DTCM_DATA Icm42688 imu_icm42688;


/// \brief 初始化 ICM42688
/// \return WHO_AM_I 校验和配置成功时返回 true
///
/// \details
/// 使用 SPI 手动片选，开启 Gyro/Accel Low Noise 模式，并配置 1kHz 数据输出
///
bool Icm42688::init() {
    uint8_t val = 0;
    system_delay_ms(10);

    // 1. 初始化逐飞 SPI (10MHz) 和独立的 CS 引脚
    spi_init(spi_n, SPI_MODE0, 10 * 1000 * 1000, sck_pin, mosi_pin, miso_pin, SPI_CS_NULL);
    gpio_init(cs_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // 2. 验证 WHO_AM_I
    cs_low();
    val = spi_read_8bit_register(spi_n, REG_WHO_AM_I | SPI_R);
    cs_high();
    
    if (val != 0x47) {
        debug_log_handler(0, (char *)"ICM42688 self check error!", (char *)__FILE__, __LINE__);
        return false;
    }

    // 3. 极简复位与启动序列
    // 软复位
    cs_low(); spi_write_8bit_register(spi_n, REG_DEVICE_CONFIG, 0x01); cs_high();
    system_delay_ms(10);
    
    // 切 Bank 0
    cs_low(); spi_write_8bit_register(spi_n, REG_BANK_SEL, 0x00); cs_high();
    
    // 打开 Gyro 和 Accel 的 Low Noise 模式 (0x0F)
    cs_low(); spi_write_8bit_register(spi_n, REG_PWR_MGMT0, 0x0F); cs_high();
    system_delay_ms(5);
    
    // 配置 Gyro: ±2000dps, 1kHz (0x06)
    cs_low(); spi_write_8bit_register(spi_n, REG_GYRO_CONFIG0, 0x06); cs_high();
    
    // 配置 Accel: ±8g, 1kHz (0x26)
    cs_low(); spi_write_8bit_register(spi_n, REG_ACCEL_CONFIG0, 0x26); cs_high();

    return true; 
}

/// \brief 读取温度、加速度和陀螺仪完整数据
///
/// \details
/// 通过 burst read 一次取 14 字节，减少 SPI 片选开销
/// 解码后会转换到项目统一的车体系符号约定
///
__attribute__((section(".ramfunc"))) void Icm42688::update_all() {
    uint8_t buf[14];
    
    // 极速 Burst Read
    cs_low();
    spi_read_8bit_registers(spi_n, REG_TEMP_DATA1 | SPI_R, buf, 14);
    cs_high();

    // 大端解码原始数据 (Temp -> Accel -> Gyro)
    data.raw_temp   = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    data.raw_acc_x  = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    data.raw_acc_y  = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    data.raw_acc_z  = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    data.raw_gyro_x = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    data.raw_gyro_y = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    data.raw_gyro_z = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    // 乘以预编译的常数转换为物理量,并且调整硬件坐标系
    data.temp   = static_cast<float>(data.raw_temp) * TEMP_SCALE + TEMP_OFFSET;
    data.acc_x  = - static_cast<float>(data.raw_acc_y) * ACCEL_SCALE;
    data.acc_y  = - static_cast<float>(data.raw_acc_x) * ACCEL_SCALE;
    data.acc_z  = - static_cast<float>(data.raw_acc_z) * ACCEL_SCALE;
    data.gyro_x = - static_cast<float>(data.raw_gyro_y) * GYRO_SCALE;
    data.gyro_y = - static_cast<float>(data.raw_gyro_x) * GYRO_SCALE;
    data.gyro_z = - static_cast<float>(data.raw_gyro_z) * GYRO_SCALE;
}


/// \brief 只读取陀螺仪三轴数据
///
/// \details
/// 主要用于开机静态零偏标定，减少不必要的 SPI 读取量
///
__attribute__((section(".ramfunc"))) void Icm42688::update_gyro_only() {
    uint8_t buf[6];
    
    // 从 0x25 开始只读 6 个字节
    cs_low();
    spi_read_8bit_registers(spi_n, REG_GYRO_DATA_X1 | SPI_R, buf, 6);
    cs_high();

    // 大端解码
    data.raw_gyro_x = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    data.raw_gyro_y = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    data.raw_gyro_z = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    // 转换为物理量
    data.gyro_x = - static_cast<float>(data.raw_gyro_y) * GYRO_SCALE;
    data.gyro_y = - static_cast<float>(data.raw_gyro_x) * GYRO_SCALE;
    data.gyro_z = - static_cast<float>(data.raw_gyro_z) * GYRO_SCALE;
}


/// \brief 只读取加速度计三轴数据
///
/// \details
/// 主要用于初始化调试探针或单独检查姿态参考
///
__attribute__((section(".ramfunc"))) void Icm42688::update_accel_only() {
    uint8_t buf[6];
    
    // 从 0x1F 开始只读 6 个字节
    cs_low();
    spi_read_8bit_registers(spi_n, REG_ACCEL_DATA_X1 | SPI_R, buf, 6);
    cs_high();

    // 大端解码
    data.raw_acc_x = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    data.raw_acc_y = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    data.raw_acc_z = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    // 转换为物理量
    data.acc_x = - static_cast<float>(data.raw_acc_y) * ACCEL_SCALE;
    data.acc_y = - static_cast<float>(data.raw_acc_x) * ACCEL_SCALE;
    data.acc_z = - static_cast<float>(data.raw_acc_z) * ACCEL_SCALE;
}
