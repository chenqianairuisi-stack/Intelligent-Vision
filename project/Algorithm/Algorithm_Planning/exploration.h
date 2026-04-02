#pragma once
#include "system_config.h"
#include "sokoban.h"


// 观测点结构体
struct ObsPoint {
    point pos;           // 网格坐标
    float target_yaw;    // 车头角度 (deg)
    uint8_t entity_id;   // 属于哪个实体？ (0~7)
    bool is_box;         // 这是一个箱子还是目标点
};

class Exploration {
public:
    Exploration() = default;

    void load_level(const SokobanLevel& level);
    uint8_t get_entity_count() const { return total_entities; }

    // 观测路径生成：传入当前起点坐标与朝向，返回绝对最优的观测顺序
    StaticArray<ObsPoint, 32> plan_optimal_patrol(point start_pos, float start_yaw);
    // 简单寻路函数
    bool get_grid_path(point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path);

    // 匹配语义：传入识别到的乱序标签数组，输出完美匹配的箱子到目标的映射关系
    bool match_semantics(const int8_t* semantic_labels, uint8_t* out_matched_ids) const;

private:
    StaticArray<ObsPoint, 32> obs_points;    // 所有合法的观测点列表
    uint8_t total_entities;                  // 观测点对应的实体总数（箱子+目标点）
    SokobanLevel cached_level;               // 内部缓存的绝对真理地图

    float cost_matrix[32][32];               // 任意两个观测点之间的代价矩阵 (物理距离 + 朝向差的加权和)

    void generate_obs_points();                         // 生成合法观测点
    void build_cost_matrix();                           // BFS 构建全源代价矩阵
    float bfs_shortest_path(point start, point end);    // BFS 纯寻路算法 (把箱子也当成墙，因为巡图时不能撞箱子)
};

extern Exploration patrol_planner;