#pragma once
#include "system_config.h"
#include "strategy.h"
#include "sokoban.h"

// ============================================================================
// 数据结构定义
// ============================================================================

// 单个观测点结构体 (寻图物理位置与视觉目标)
struct ObsPoint {
    point pos;           // 底盘需要到达的网格坐标
    float target_yaw;    // 车头需要对齐的偏航角 (deg)
    uint8_t entity_id;   // 关联的实体 ID[0, MAX_ENTITIES)
    bool is_box;         // true: 观测的是箱子, false: 观测的是目标点
};

// 全局宏动作指令
struct PatrolAction {
    bool is_bomb_task;    // false: 常规巡图观测; true: 切换为推炸弹宏动作
    ObsPoint obs;         // 若为观测任务，读取此字段
    BombTask bomb;        // 若为推炸弹任务，读取此字段
};

// 推炸弹宏动作节点 (用于生成推炸弹的连续路径)
struct BombMacroNode {
    int8_t bx, by;          // 炸弹坐标
    uint8_t p_dir;          // 小车站在炸弹的哪一侧 (0~3，对应 MOVE[0~3] 的反方向)
    uint16_t parent_idx;    // 父节点索引 (最多 1024 个节点，uint16_t 绰绰有余)
};


// ============================================================================
// 视觉探索与巡图规划器核心类
// ============================================================================
class Exploration {
public:
    Exploration() = default;

    // --- 初始化与状态查询 ---
    void load_level(const SokobanLevel& level);
    uint8_t get_entity_count() const { return total_entities; }

    // --- 核心函数 ---
    // 【API 1】无炸弹纯净巡图：供 Phase 2 调用，返回纯净的最优观测序列
    StaticArray<ObsPoint, 32> plan_optimal_patrol(point start_pos);
    // 【API 2】多重分支 3D 巡图：供 Phase 3 调用，自动将“推炸弹”动作与“巡图”无缝交织融合
    StaticArray<PatrolAction, 32> plan_optimal_patrol(point start_pos, const StaticArray<BombTask, MAX_BOMBS>& bomb_tasks);
    
    // --- 物理底层循迹接口 ---
    // 生成真实可用的网格路径坐标数组
    bool get_grid_path(const SokobanLevel& lvl, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path);
    // 专门为推炸弹设计的路径生成，考虑炸弹推行时的特殊碰撞规则
    bool get_bomb_push_path(const SokobanLevel& lvl, point player_start, BombTask bomb_task, StaticArray<point, MAX_PATH_LENGTH>& out_path);
    
    // 【语义匹配】：融合云台识别结果，进行逻辑演绎与死锁解除
    bool match_semantics(const int8_t* semantic_labels, uint8_t* out_matched_ids) const;

private:
    // --- 内部状态缓存 ---
    StaticArray<ObsPoint, MAX_OBS_POINTS> obs_points;    // 当前地图所有合法的观测位姿
    uint8_t total_entities;                              // 实体总数 (Box + Target)
    SokobanLevel cached_level;                           // 初始时刻的地图

    // 生成合法观测点
    void generate_obs_points();                          
    
    // 寻路函数：支持多重地图分支（不同炸弹爆炸后的地形）
    uint16_t bfs_shortest_path(const SokobanLevel& lvl, point start, point end);
};

extern Exploration patrol_planner;