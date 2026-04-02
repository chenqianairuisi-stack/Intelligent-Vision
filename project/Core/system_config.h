#pragma once
#include <cstdint>

//-----------------------------------------------------------------------------
//全局坐标系：x轴正方向为右，y轴正方向为前，逆时针为正旋转（x轴设为0度）
//全局地图：大小 240cm*320cm, 分为 12*16 格，原点(0,0)在左下角，x轴向右，y轴向上
//-----------------------------------------------------------------------------

namespace SystemConfig {
    // 机械参数
    static constexpr float WHEEL_RADIUS = 3.15f;                // 轮子半径，单位：厘米

    // 编码器硬件参数
    static constexpr float ENC_LINES = 1024.0f;                 // 编码器物理线数
    static constexpr float ENC_QUAD_FACTOR = 4.0f;              // 硬件四倍频
    static constexpr float GEAR_RATIO = 70.0f / 30.0f;          // 传动比：轮轴转一圈，编码器转多少圈
    static constexpr float PULSES_PER_REV  = ENC_LINES * ENC_QUAD_FACTOR * GEAR_RATIO;    // 轮子每转脉冲数

    // 物理常数
    static constexpr float GRID_SIZE_CM = 20.0f;                // 每格20cm
    static constexpr float MAP_OFFSET_X = 10.0f;                // 网格原点 (0,0) 对应物理世界坐标系 X 偏移量
    static constexpr float MAP_OFFSET_Y = 10.0f;                // 网格原点 (0,0) 对应物理世界坐标系 Y 偏移量

    // 规划算法参数
    static constexpr int MAP_MAX_WIDTH = 12;                    // 地图最大宽度（网格坐标）
    static constexpr int MAP_MAX_HEIGHT = 16;                   // 地图最大高度（网格坐标）
    static constexpr int PLAN_START_X = 5;                      // 规划起点 X 坐标（网格坐标）
    static constexpr int PLAN_START_Y = 2;                      // 规划起点 Y 坐标（网格坐标）
    static constexpr int MAX_BOXES = 4;                         // 最大箱子数
    static constexpr int MAX_BOMBS = 4;                         // 最大炸弹数
    static constexpr int MAX_ENTITIES = MAX_BOXES + MAX_BOMBS;  // 最大实体数（箱子+炸弹）
    static constexpr int MAX_PATH_LENGTH = 100;                 // 最大搜索步数
    
    // 其他全局常量
    static constexpr float ENTRY_X = 120.0f;                    // 入口位置 X 坐标
    static constexpr float ENTRY_Y = 10.0f;                     // 入口位置 Y 坐标
    static constexpr float ENTRY_YAW = 90.0f;                   // 入口位置航向（单位：度，0度为x轴正方向，逆时针为正）
    static constexpr float OUT_TARGET_X = 120.0f;               // 出库目标位置 X 坐标
    static constexpr float OUT_TARGET_Y = 50.0f;                // 出库目标位置 Y 坐标
}

// 四个轮子的转速结构体 (单位：cm/s)
struct WheelSpeed4 {
    float lf;      // Left Front
    float lb;      // Left Back
    float rf;      // Right Front
    float rb;      // Right Back
};

// 速度结构体 (单位：cm/s)
struct Velocity2D {
    float vy;      // 前后速度 (前为正)
    float vx;      // 左右速度 (右为正)
    float vw;      // 旋转速度 (逆时针为正)
};

// 速度结构体 (单位：cm/s)
struct Speed2D {
    float vy;      
    float vx;     
    float v_mag;   // 标量速度 (单位：cm/s)
};

// 全局坐标结构体 (现实世界坐标系中的坐标点，单位cm)
struct Point2D {
    float x;
    float y;
};

// 全局位姿结构体 (现实世界坐标系中的位姿，单位 cm 和 deg)
struct Pose2D {
    float x;
    float y;
    float yaw;
};

// 坐标结构体 (用于表示网格地图上的坐标点)
struct point {
    int8_t x, y;  

    bool operator == (const point &other) const { return x == other.x && y == other.y; }
    bool operator != (const point &other) const { return !(x == other.x && y == other.y); }
    point operator + (const point &other) const { 
        return {static_cast<int8_t>(x + other.x), static_cast<int8_t>(y + other.y)}; 
    }
    point operator - (const point &other) const { 
        return {static_cast<int8_t>(x - other.x), static_cast<int8_t>(y - other.y)}; 
    }
};

// 模版类：固定长度数组，兼容 vector 接口，避免动态内存分配
template <typename T, int MAX_LEN>
struct StaticArray {
    T data[MAX_LEN];
    int length = 0;

    // 兼容 vector 的接口
    void push_back(const T& val) { if (length < MAX_LEN) data[length++] = val; }
    void pop_back() { if (length > 0) length--; }
    void clear() { length = 0; }
    int size() const { return length; }
    bool empty() const { return length == 0; }
    
    T& operator[](int i) { return data[i]; }
    const T& operator[](int i) const { return data[i]; }

    T& back() { return data[length - 1]; }
    const T& back() const { return data[length - 1]; }

    T* begin() { return &data[0]; }
    T* end() { return &data[length]; }

    const T* begin() const { return &data[0]; }
    const T* end() const { return &data[length]; }

    // 提供一个空的 reserve 预分配函数，兼容原本代码里的 reserve
    void reserve(int n) { (void)n; } 
};


#ifndef PI
#define PI 3.1415926535898f
#endif