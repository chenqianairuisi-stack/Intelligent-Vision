#pragma once
#include "system_config.h"
#include "Sokoban.h"

// 炸弹动作定义
struct BombTask {
    point bomb_start;    // 要推的炸弹初始位置
    point target_wall;   // 要炸毁的墙壁坐标
    bool is_essential;   // 是否为救命墙（不炸就绝对无解）
    int net_profit;      // 净收益评估值
};

// 用于对合法墙壁进行排序
struct BombCandidate {
    uint8_t bomb_idx;
    int8_t x, y;
    int score;
    bool operator<(const BombCandidate& other) const {
        return score > other.score; // 分数高的排前面
    }
};

// DFS 最优结果
struct DFSResult {
    StaticArray<BombTask, MAX_BOMBS> tasks;
    int deadlocks_remaining;
    int net_profit;
};

// 战略规划器：通过 DFS 推演寻找全局最优炸弹策略
class StrategicPlanner {
public:
    StrategicPlanner() = default;

    // 对外接口：输入地图和玩家位置，输出炸弹任务列表
    StaticArray<BombTask, MAX_BOMBS> evaluate_and_assign_bombs(const SokobanLevel& level, point player_start);

private:
    SokobanLevel cached_level;

    // 递归搜索核心
    void dfs_bomb_sequence(
        const SokobanLevel& current_lvl,
        point player_start,
        StaticArray<BombTask, MAX_BOMBS> current_seq,
        int cost_so_far,
        int depth,
        DFSResult& best_res);

    // 内部物理与搜索工具函数
    void calc_player_reach(const SokobanLevel& lvl, point start_pos, point ignored_obj, point extra_obs, bool out_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]);
    void fast_push_bfs(const SokobanLevel& lvl, point start_obj, point player_start, bool is_bomb, int16_t out_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]);
    bool has_entity(const SokobanLevel& lvl, int x, int y, int ignored_bomb) const;
    bool is_obstacle(const SokobanLevel& lvl, point p, point ignored_obj) const;
};

extern StrategicPlanner strategic_planner;