#pragma once
#include <cstdint>
#include <array>

//---------------------------------------------------------------------------------
//全局坐标系：x 轴正方向为右，y 轴正方向为前，逆时针为正旋转（x 轴设为 0 度）
//全局地图：大小 240cm*320cm, 分为 12*16 格，原点(0,0)在左下角，x 轴向右，y 轴向上
//---------------------------------------------------------------------------------

// =================================================================
// 全局系统配置和常量定义
// =================================================================
namespace SystemConfig {
    // 定时器参数
    static constexpr uint32_t PIT_CH0_PERIOD_MS = 1U;          // IMU 采样与姿态解算周期
    static constexpr uint32_t PIT_CH1_PERIOD_MS = 20U;         // 底盘控制与里程计周期
    static constexpr float PIT_CH0_DT_S = static_cast<float>(PIT_CH0_PERIOD_MS) * 0.001f;  // 转换为秒
    static constexpr float PIT_CH1_DT_S = static_cast<float>(PIT_CH1_PERIOD_MS) * 0.001f;  // 转换为秒

    // 机械参数
    static constexpr float WHEEL_RADIUS = 3.15f;                // 轮子半径，单位：厘米
    static constexpr float HALF_X_AXIS = 9.0f;                  // x 轴半轴距，单位：厘米
    static constexpr float HALF_Y_AXIS = 10.0f;                 // y 轴半轴距，单位：厘米

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
    static constexpr int PLAN_START_X = 4;                      // 出库点 X 坐标（网格坐标）
    static constexpr int PLAN_START_Y = 1;                      // 出库点 Y 坐标（网格坐标）
    static constexpr int PLAN_END_X = 5;                        // 入库点 X 坐标（网格坐标）
    static constexpr int PLAN_END_Y = 1;                        // 入库点 Y 坐标（网格坐标）
    static constexpr int MAX_BOXES = 10;                        // 最大箱子数
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
    static constexpr float OUT_TARGET_X = 90.0f;                // 出库目标位置 X 坐标
    static constexpr float OUT_TARGET_Y = 30.0f;                // 出库目标位置 Y 坐标

    // 数学常数
    static constexpr float DEG_TO_RAD = 0.017453292519943f;
    static constexpr float RAD_TO_DEG = 57.29577951308232f;
}


// =================================================================
// 全局状态机枚举
// =================================================================

// 游戏全局状态机枚举
enum class GamePhase : uint8_t {
    // --- 发车阶段 ---
    NONE,                   // 初始状态，等待开始
    INIT_CALIBRATE,         // 初始化与校准里程计
    EXIT_START_ZONE,        // 出发车区
    WAIT_FOR_VISION,        // 等待摄像头返回地图

    // --- 第二/三阶段 ---
    PLAN_PATROL,            // GTSP 规划巡图观测路径
    EXEC_ACTION_DISPATCH,   // 分发：判断当前动作是去观测，还是去推炸弹
    EXEC_PATROL_MOVE,       // 动作 A1：底盘移动到观测点
    EXEC_ALIGN_YAW,         // 动作 A2：底盘自旋对准目标，并发送 ART2 捕捉请求
    WAIT_ART2_CAPTURE_ACK,  // 等待截图成功
    EXEC_BOMB_PUSH,         // 动作 B：执行推炸宏动作
    UPDATE_MAP,             // 完成推炸弹，更新地图状态

    // --- 第一阶段 ---
    BIND_SEMANTICS,         // 巡视完毕，将识别结果绑定到底层算法
    PLAN_SOKOBAN,           // 规划推箱子路径
    EXEC_SOKOBAN,           // 执行推箱子循迹

    // --- 返程状态 ---
    PLAN_RETURN_HOME,       // 规划回发车区的路径
    EXEC_RETURN_HOME,       // 执行回程

    // --- 结束阶段 ---
    FINISHED,               // 比赛完成，停车
    ERROR_OCCURRED,         // 发生错误，停车

    // --- 调试专用状态 ---
    ANIMATE_PATROL_DEMO,    // 播放巡图过程动画
    ANIMATE_DEMO,           // 播放推箱子过程动画
    ANIMATE_RETURN_DEMO,    // 播放回程动画
};

// 底盘循迹状态机枚举
enum class TrackerState : uint8_t {
    NONE,                   // 待机
    TRACKING,               // 正在循迹
    FINISHED                // 路径执行完毕
};

// 控制模式枚举：手动调试 vs 自动循迹
enum class ControlMode : uint8_t {
    MANUAL_DEBUG,           // 调试模式：上位机直接写 target_pose，不理会 Tracker
    AUTO_TRACKING           // 自动模式：听从 Tracker 生成的路径
};


// =================================================================
// 数据结构定义
// =================================================================

// 四轮转速结构体 (cm/s)
struct WheelSpeed4 { float lf; float lb; float rf; float rb;};

// 速度结构体 (cm/s)
struct Velocity2D { float vx; float vy; float vw;};
struct Speed2D { float vx; float vy;};

// 全局物理坐标结构体 (cm/deg)
struct Pose2D { float x; float y; float yaw;};
struct Point2D { float x; float y;};

// 网格坐标结构体 (格)
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

// 四个移动方向：上、右、下、左
constexpr point MOVE[4] = {{0,1}, {1,0}, {0,-1}, {-1,0}};

// 地图和状态表示结构体
struct SokobanLevel {
    std::array<std::array<int8_t, SystemConfig::MAP_MAX_WIDTH>, SystemConfig::MAP_MAX_HEIGHT> map;
    point player_start;
    
    point bombs[SystemConfig::MAX_BOMBS];    uint8_t bomb_count;
    point targets[SystemConfig::MAX_BOXES];  uint8_t target_count;
    point boxes[SystemConfig::MAX_BOXES];    uint8_t box_count;

    uint8_t box_ids[SystemConfig::MAX_BOXES];  // 映射关系：box_ids[i] 表示第 i 个箱子对于应的目标点ID
};


// =================================================================
// 通用模板
// =================================================================

// 定长数组：保留常用 vector 风格接口，避免动态内存分配 
template <typename T, int MAX_LEN>
struct StaticArray {
    T data[MAX_LEN];
    int length = 0;

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

    void reserve(int n) { (void)n; } 
};


#ifndef PI
#define PI 3.1415926535898f
#endif