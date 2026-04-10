#pragma once
#include <cstdint>

//---------------------------------------------------------------------------------
//全局坐标系：x 轴正方向为右，y 轴正方向为前，逆时针为正旋转（x 轴设为 0 度）
//全局地图：大小 240cm*320cm, 分为 12*16 格，原点(0,0)在左下角，x 轴向右，y 轴向上
//---------------------------------------------------------------------------------

namespace SystemConfig {
    // 机械参数
    static constexpr float WHEEL_RADIUS = 3.15f;                // 轮子半径，单位：厘米
    static constexpr float HALF_X_AXIS = 9.0f;                  // x 轴半轴距，单位：厘米
    static constexpr float HALF_Y_AXIS = 9.8f;                  // y 轴半轴距，单位：厘米

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
    static constexpr int PLAN_START_X = 5;                      // 出库点 X 坐标（网格坐标）
    static constexpr int PLAN_START_Y = 2;                      // 出库点 Y 坐标（网格坐标）
    static constexpr int PLAN_END_X = 5;                        // 入库点 X 坐标（网格坐标）
    static constexpr int PLAN_END_Y = 1;                        // 入库点 Y 坐标（网格坐标）
    static constexpr int MAX_BOXES = 3;                         // 最大箱子数
    static constexpr int MAX_BOMBS = 3;                         // 最大炸弹数
    static constexpr int MAX_ENTITIES = 2 * MAX_BOXES;          // 最大实体数（箱子+目标点）
    static constexpr int MAX_ENTITY_MASK = 1 << MAX_ENTITIES;   // 实体访问状态总数（bitmask）
    static constexpr int MAX_OBS_POINTS = 4 * MAX_ENTITIES;     // 最大观测点数（每个实体最多4个观测点，分别对应4个朝向）
    static constexpr int MAP_CELL_COUNT = MAP_MAX_WIDTH * MAP_MAX_HEIGHT;   // 地图总格子数
    static constexpr int MAX_PATH_LENGTH = 100;                 // 最大搜索步数
    
    // 其他全局常量
    static constexpr float ENTRY_X = 110.0f;                    // 入口位置 X 坐标
    static constexpr float ENTRY_Y = 10.0f;                     // 入口位置 Y 坐标
    static constexpr float ENTRY_YAW = 90.0f;                   // 入口位置航向（单位：度，0度为x轴正方向，逆时针为正）
    static constexpr float OUT_TARGET_X = 110.0f;               // 出库目标位置 X 坐标
    static constexpr float OUT_TARGET_Y = 50.0f;                // 出库目标位置 Y 坐标

    // 数学常数
    static constexpr float DEG_TO_RAD = 3.1415926535f / 180.0f;
}

// 四个轮子的转速结构体 (单位：cm/s)
struct WheelSpeed4 { float lf; float lb; float rf; float rb;};

// 速度结构体 (cm/s)
struct Velocity2D { float vx; float vy; float vw;};
struct Speed2D { float vx; float vy;};

// 全局物理坐标结构体 (单位 cm/deg)
struct Pose2D { float x; float y; float yaw;};
struct Point2D { float x; float y;};

// 全局网格坐标结构体 (单位：格)
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

// 定长数组：保留常用 vector 风格接口，避免动态内存分配 
template <typename T, int MAX_LEN>
struct StaticArray {
    T data[MAX_LEN];
    int length = 0;

    // 与 vector 常用接口保持一致
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

    // 占位接口：保持与原调用方兼容
    void reserve(int n) { (void)n; } 
};


#ifndef PI
#define PI 3.1415926535898f
#endif