#pragma once

namespace SystemConfig {
    // 机械参数
    static constexpr float WHEEL_RADIUS = 5.0f;        // 轮子半径，单位：厘米

    // 编码器参数
    static constexpr float PULSES_PER_REV = 1024.0f;   // 编码器每转脉冲数
    static constexpr float GEAR_RATIO = 1.0f;          // 减速比

    // 物理常数
    static constexpr float GRID_SIZE_CM = 20.0f;           // 每格20cm
    static constexpr float MAP_OFFSET_X = 10.0f;           // 网格原点 (0,0) 对应物理世界坐标系 X 偏移量
    static constexpr float MAP_OFFSET_Y = 10.0f;           // 网格原点 (0,0) 对应物理世界坐标系 Y 偏移量

    // 规划算法参数
    static constexpr int MAP_MAX_WIDTH = 12;
    static constexpr int MAP_MAX_HEIGHT = 16;
    static constexpr int MAX_BOXES = 4;             // 最大箱子数放宽到4个
    static constexpr int MAX_BOMBS = 4;             // 最大炸弹数放宽到4个
    static constexpr int MAX_PATH_LENGTH = 100;     // 限制最大搜索步数
    
}

// 全局坐标结构体 (现实世界坐标系中的坐标点，单位cm)
struct Point2D {
    float x;
    float y;
};

// 全局位姿结构体 (现实世界坐标系中的位姿，单位 cm 和弧度)
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

