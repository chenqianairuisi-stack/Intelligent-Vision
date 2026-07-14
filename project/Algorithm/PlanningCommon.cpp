#include "PlanningCommon.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace PlanningCommon {

// ============================================================================
// 麦轮底盘时间代价参数模板
// ============================================================================

namespace MotionCost {
    // 单格平移的基础时间单位，建议先按实测直线走一格耗时缩放到整数
    inline constexpr uint16_t GRID_MOVE = 1;
    // 路径方向发生变化时的停车和再启动惩罚，麦轮拐点停顿明显时应显著大于 GRID_MOVE
    inline constexpr uint16_t CORNER_STOP = 3;
    // 到达观测位后为了对准目标朝向产生的额外代价，不参与普通路径拐点统计
    inline constexpr uint16_t OBSERVE_YAW = 4;
    // uint16_t 对外接口的饱和值，和旧 BFS 不可达返回值保持一致
    inline constexpr uint16_t INF = 65535;
}

/// \brief 时间代价 Dijkstra 的父节点记录
struct TimeParent {
    int8_t x;     // 上一个格子的 x
    int8_t y;     // 上一个格子的 y
    uint8_t dir;  // 进入当前格子前的方向，255 表示从起点直接到达
};

/// \brief 时间代价最小堆节点
struct TimeHeapNode {
    uint32_t cost; // 累计时间代价
    int8_t x;      // 当前格子 x
    int8_t y;      // 当前格子 y
    uint8_t dir;   // 当前行进方向
};

// 时间代价堆容量，16x16 地图下按状态数预留 8 倍冗余
static constexpr int TIME_HEAP_CAPACITY = MAP_CELL_COUNT * 8;

/// \brief 带转向代价的网格搜索共享工作区
struct TimeSearchWorkspace {
    uint32_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4]; // 每格每方向的最小代价
    bool used[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];     // Dijkstra 已确定状态标记
    bool blocked[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];     // 墙体和动态物体占用表
    TimeHeapNode heap[TIME_HEAP_CAPACITY];           // 固定容量最小堆
};

/// \brief 普通 BFS 共享工作区
struct SimpleBfsWorkspace {
    point q[MAP_CELL_COUNT];                         // BFS 队列
    point parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];     // 路径回溯父节点
    uint16_t dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];    // 普通最短步数
    uint16_t visited_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // generation visited 标记
    bool blocked[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];     // O(1) 障碍查询表
    uint16_t current_gen = 0;                        // 当前 BFS 代数
};

// 时间搜索工作区放 OCRAM，避免挤占 DTCM
OCRAM_BSS static TimeSearchWorkspace time_ws;

// 普通 BFS 工作区放 OCRAM，消除热点函数中的局部大数组
OCRAM_BSS static SimpleBfsWorkspace simple_bfs_ws;

// 将 32 位内部代价钳制到对外 uint16_t 代价范围
static uint16_t clamp_time_cost(uint32_t cost) {
    return cost >= MotionCost::INF ? MotionCost::INF : static_cast<uint16_t>(cost);
}

// 返回两个相邻格子的移动方向
static int direction_between(point from, point to) {
    point delta = to - from;
    for (int d = 0; d < 4; ++d) {
        if (delta == MOVE[d]) return d;
    }
    return -1;
}

// 查询时间搜索中的可通行状态
static bool grid_time_passable(const bool blocked[MAP_MAX_HEIGHT][MAP_MAX_WIDTH], point p) {
    if (!in_bounds(p)) return false;
    return !blocked[p.y][p.x];
}

/// \brief 构建 O(1) 障碍占用表
/// \param lvl 当前地图状态
/// \param blocked 输出障碍矩阵
/// \param ignored_obj 需要临时忽略的动态物体坐标
/// \param extra_obs 额外加入的临时障碍
static void build_blocked_map(const SokobanLevel& lvl,
                            bool blocked[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
                            point ignored_obj = {-1, -1},
                            point extra_obs = {-1, -1}) {
    std::memset(blocked, 0, MAP_MAX_HEIGHT * MAP_MAX_WIDTH * sizeof(blocked[0][0]));
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            blocked[y][x] = (lvl.map[y][x] == 1);
        }
    }
    for (int i = 0; i < lvl.box_count; ++i) {
        point p = lvl.boxes[i];
        if (!(p == ignored_obj) && in_bounds(p)) blocked[p.y][p.x] = true;
    }
    for (int i = 0; i < lvl.bomb_count; ++i) {
        point p = lvl.bombs[i];
        if (p.x != -1 && !(p == ignored_obj) && in_bounds(p)) blocked[p.y][p.x] = true;
    }
    if (in_bounds(extra_obs)) blocked[extra_obs.y][extra_obs.x] = true;
}

// 递增普通 BFS generation，溢出时清空 visited 表
static uint16_t next_simple_bfs_gen() {
    SimpleBfsWorkspace& ws = simple_bfs_ws;
    ++ws.current_gen;
    if (ws.current_gen == 0) {
        std::memset(ws.visited_gen, 0, sizeof(ws.visited_gen));
        ws.current_gen = 1;
    }
    return ws.current_gen;
}

// 将时间代价图初始化为不可达
static void init_time_map(uint16_t out_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            out_cost[y][x] = MotionCost::INF;
        }
    }
}

/// \brief 执行带转向惩罚的网格 Dijkstra
/// \param lvl 当前地图状态
/// \param start 起点
/// \param out_cost 输出每个格子的最小时间代价
/// \param parent 可选输出每格每方向的父节点，用于回溯路径
/// \return 搜索成功完成时返回 true，堆容量不足或起点非法时返回 false
static bool run_grid_time_search(const SokobanLevel& lvl,
                                point start,
                                uint16_t out_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
                                TimeParent parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4]) {
    TimeSearchWorkspace& ws = time_ws;
    build_blocked_map(lvl, ws.blocked);
    int heap_size = 0;
    bool heap_overflow = false;

    auto heap_less = [](const TimeHeapNode& a, const TimeHeapNode& b) {
        return a.cost < b.cost;
    };
    auto heap_push = [&](TimeHeapNode node) {
        if (heap_size >= TIME_HEAP_CAPACITY) {
            heap_overflow = true;
            return;
        }
        int i = heap_size++;
        ws.heap[i] = node;
        while (i > 0) {
            int parent_idx = (i - 1) / 2;
            if (!heap_less(ws.heap[i], ws.heap[parent_idx])) break;
            TimeHeapNode temp = ws.heap[i];
            ws.heap[i] = ws.heap[parent_idx];
            ws.heap[parent_idx] = temp;
            i = parent_idx;
        }
    };
    auto heap_pop = [&]() {
        TimeHeapNode root = ws.heap[0];
        ws.heap[0] = ws.heap[--heap_size];
        int i = 0;
        while (true) {
            int left = i * 2 + 1;
            int right = left + 1;
            int best = i;
            if (left < heap_size && heap_less(ws.heap[left], ws.heap[best])) best = left;
            if (right < heap_size && heap_less(ws.heap[right], ws.heap[best])) best = right;
            if (best == i) break;
            TimeHeapNode temp = ws.heap[i];
            ws.heap[i] = ws.heap[best];
            ws.heap[best] = temp;
            i = best;
        }
        return root;
    };

    init_time_map(out_cost);
    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            for (int d = 0; d < 4; ++d) {
                ws.dist[y][x][d] = 0xFFFFFFFFU;
                ws.used[y][x][d] = false;
                if (parent) parent[y][x][d] = {-1, -1, 255};
            }
        }
    }

    if (!in_bounds(start)) return false;
    out_cost[start.y][start.x] = 0;

    for (int d = 0; d < 4; ++d) {
        point np = start + MOVE[d];
        if (!grid_time_passable(ws.blocked, np)) continue;
        ws.dist[np.y][np.x][d] = MotionCost::GRID_MOVE;
        out_cost[np.y][np.x] = MotionCost::GRID_MOVE;
        if (parent) parent[np.y][np.x][d] = {start.x, start.y, 255};
        heap_push({MotionCost::GRID_MOVE, np.x, np.y, static_cast<uint8_t>(d)});
    }

    while (heap_size > 0) {
        TimeHeapNode node = heap_pop();
        point cur{node.x, node.y};
        int cur_dir = node.dir;
        if (ws.used[cur.y][cur.x][cur_dir]) continue;
        if (node.cost != ws.dist[cur.y][cur.x][cur_dir]) continue;
        ws.used[cur.y][cur.x][cur_dir] = true;

        for (int nd = 0; nd < 4; ++nd) {
            point np = cur + MOVE[nd];
            if (!grid_time_passable(ws.blocked, np)) continue;

            uint32_t next_cost = node.cost + MotionCost::GRID_MOVE;
            if (nd != cur_dir) next_cost += MotionCost::CORNER_STOP;
            if (next_cost >= ws.dist[np.y][np.x][nd]) continue;

            ws.dist[np.y][np.x][nd] = next_cost;
            uint16_t clamped = clamp_time_cost(next_cost);
            if (clamped < out_cost[np.y][np.x]) out_cost[np.y][np.x] = clamped;
            if (parent) parent[np.y][np.x][nd] = {cur.x, cur.y, static_cast<uint8_t>(cur_dir)};
            heap_push({next_cost, np.x, np.y, static_cast<uint8_t>(nd)});
        }
    }

    if (heap_overflow) {
        init_time_map(out_cost);
        return false;
    }
    return true;
}

// ============================================================================
// 推物体宏搜索工作区
// ============================================================================

/// \brief 推箱和推炸弹宏层 BFS 节点
struct BombMacroNode {
    int8_t bx, by;        // 当前被推动物体坐标
    uint8_t p_dir;        // 玩家站在物体哪一侧推动
    uint16_t parent_idx;  // BFS 回溯父节点
};

/// \brief 推物体路径展开共享工作区
struct BombPathWorkspace {
    BombMacroNode q[1024];                         // 宏层 BFS 队列
    uint8_t visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 每格按推动方向记录访问状态
    uint16_t node_cost[1024];                      // 代价优先推炸弹搜索中的节点总代价
    uint16_t state_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4]; // 每个宏状态的最优代价
    uint16_t state_node[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4]; // 每个宏状态对应的最优节点
    uint8_t state_closed[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4]; // Dijkstra 已确定状态
    uint8_t micro_dist[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 微层步数缓存
    point micro_q[MAP_CELL_COUNT];                 // 微层玩家 BFS 队列
    uint8_t micro_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 微层 generation visited 标记
    point micro_parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];    // 微层路径回溯父节点
    uint8_t micro_gen;                             // 微层 BFS 代数
};

// 推物体工作区保留在 DTCM，提高宏微双层搜索访问速度
DTCM_DATA static BombPathWorkspace b_ws;

// ============================================================================
// 地图与实体查询
// ============================================================================

// 判断坐标是否位于固定地图边界内
bool in_bounds(point p) {
    return p.x >= 0 && p.x < MAP_MAX_WIDTH && p.y >= 0 && p.y < MAP_MAX_HEIGHT;
}

// 判断坐标是否位于可变化的地图内圈
bool is_inner_map_cell(point p) {
    return p.x > 0 && p.x < MAP_MAX_WIDTH - 1 && p.y > 0 && p.y < MAP_MAX_HEIGHT - 1;
}

// 外圈墙是永久边界，炸弹任务只能绑定内圈墙
bool is_blastable_wall(const SokobanLevel& lvl, point p) {
    return is_inner_map_cell(p) && lvl.map[p.y][p.x] == 1;
}

// 查询指定格子是否有箱子
bool has_box(const SokobanLevel& lvl, point p) {
    for (int i = 0; i < lvl.box_count; ++i) {
        if (lvl.boxes[i] == p) return true;
    }
    return false;
}

// 查询指定格子是否有炸弹，可忽略一个炸弹编号
bool has_bomb(const SokobanLevel& lvl, point p, int ignored_bomb) {
    for (int i = 0; i < lvl.bomb_count; ++i) {
        if (i == ignored_bomb) continue;
        if (lvl.bombs[i].x != -1 && lvl.bombs[i] == p) return true;
    }
    return false;
}

// 查询指定格子是否有箱子、目标或炸弹
bool has_entity(const SokobanLevel& lvl, int x, int y, int ignored_bomb) {
    point p = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
    if (has_box(lvl, p)) return true;
    for (int i = 0; i < lvl.target_count; ++i) {
        if (lvl.targets[i] == p) return true;
    }
    return has_bomb(lvl, p, ignored_bomb);
}

// 判断格子是否为墙体或动态障碍，可临时忽略一个物体
bool is_obstacle(const SokobanLevel& lvl, point p, point ignored_obj) {
    if (!in_bounds(p)) return true;
    if (lvl.map[p.y][p.x] == 1) return true;

    for (int i = 0; i < lvl.box_count; ++i) {
        if (lvl.boxes[i] == p && !(p == ignored_obj)) return true;
    }
    for (int i = 0; i < lvl.bomb_count; ++i) {
        if (lvl.bombs[i].x != -1 && lvl.bombs[i] == p && !(p == ignored_obj)) return true;
    }
    return false;
}

// ============================================================================
// 地图状态更新
// ============================================================================

// 判断任务列表中的炸弹任务是否对应本次推炸弹动作
static bool matches_bomb_push_action(const BombTask& task, const BombPushAction& action) {
    return task.bomb_start == action.bomb_start && task.target_wall == action.blast_wall;
}

// 删除已完成的炸弹任务，保持 StaticArray 连续存储
static void remove_bomb_task_at(StaticArray<BombTask, MAX_BOMBS>& tasks, int index) {
    for (int i = index; i + 1 < tasks.size(); ++i) {
        tasks[i] = tasks[i + 1];
    }
    tasks.pop_back();
}

/// \brief 应用一次推箱任务对地图造成的状态变化
/// \param lvl 需要原地更新的地图状态
/// \param task 已完成执行的推箱任务
///
/// \details
/// 函数只根据箱子起点查找并更新箱子坐标，不处理路径生成和玩家位置
void apply_box_push_task_effect(SokobanLevel& lvl, const BoxPushTask& task) {
    for (int b = 0; b < lvl.box_count; ++b) {
        if (lvl.boxes[b] == task.box_start) {
            lvl.boxes[b] = task.box_target;
            return;
        }
    }
}

/// \brief 应用一次推箱宏动作对地图造成的状态变化
/// \param lvl 需要原地更新的地图状态
/// \param action 已完成执行的推箱宏动作
///
/// \details
/// 当动作携带有效 box_id 且起点仍匹配时优先按编号更新，
/// 否则退回到按起点坐标查找，兼容未绑定编号的清障推箱动作
void apply_box_push_action_effect(SokobanLevel& lvl, const BoxPushAction& action) {
    if (action.box_id < lvl.box_count && lvl.boxes[action.box_id] == action.box_start) {
        lvl.boxes[action.box_id] = action.box_target;
        return;
    }

    apply_box_push_task_effect(lvl, make_box_push_task(action));
}

/// \brief 应用炸弹对静态墙体的爆破效果
/// \param lvl 需要原地更新的地图状态
/// \param target_wall 目标爆破墙体
///
/// \details
/// 只有内圈墙可以作为爆心，外圈边界墙不会被炸弹任务清除
void apply_blast_effect(SokobanLevel& lvl, point target_wall) {
    if (!is_blastable_wall(lvl, target_wall)) return;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            point p = {
                static_cast<int8_t>(target_wall.x + dx),
                static_cast<int8_t>(target_wall.y + dy)
            };
            if (is_inner_map_cell(p)) {
                lvl.map[p.y][p.x] = 0;
            }
        }
    }
}

/// \brief 应用一个完整炸弹任务对地图造成的状态变化
/// \param lvl 需要原地更新的地图状态
/// \param task 已确定的炸弹任务
///
/// \details
/// 该函数会依次应用推箱让路结果、移除已引爆炸弹，并把目标墙体周围 3x3 范围内的墙清成空地
void apply_bomb_task_effect(SokobanLevel& lvl, const BombTask& task) {
    if (!is_blastable_wall(lvl, task.target_wall)) return;

    // 先应用推箱让路结果
    for (int i = 0; i < task.box_pushes.size(); ++i) {
        const BoxPushTask& bp = task.box_pushes[i];
        for (int b = 0; b < lvl.box_count; ++b) {
            if (lvl.boxes[b] == bp.box_start) {
                lvl.boxes[b] = bp.box_target;
                break;
            }
        }
    }

    // 移除已引爆的炸弹
    for (int b = 0; b < lvl.bomb_count; ++b) {
        if (lvl.bombs[b].x != -1 && lvl.bombs[b] == task.bomb_start) {
            lvl.bombs[b] = {-1, -1};
            break;
        }
    }

    apply_blast_effect(lvl, task.target_wall);
}

/// \brief 应用推炸弹宏动作的地图变化
/// \param lvl 需要原地更新的地图状态
/// \param action 推炸弹动作，可能只是移动炸弹，也可能触发爆破
///
/// \details
/// 该函数只修改传入的地图快照，不同步剩余炸弹任务列表
/// 宏规划里的模拟推演应继续使用这个接口，避免误改真实任务状态
void apply_bomb_push_action_effect(SokobanLevel& lvl, const BombPushAction& action) {
    // 如果动作末尾引爆，则直接应用完整炸弹任务效果
    if (action.detonates) {
        apply_bomb_task_effect(lvl, make_bomb_task(action));
        return;
    }

    // 仅移动炸弹，不触发爆破
    for (int b = 0; b < lvl.bomb_count; ++b) {
        if (lvl.bombs[b].x != -1 && lvl.bombs[b] == action.bomb_start) {
            lvl.bombs[b] = action.bomb_target;
            return;
        }
    }
}

/// \brief 根据已执行的推炸弹动作同步剩余炸弹任务列表
/// \param tasks 需要原地更新的剩余炸弹任务列表
/// \param action 已完成执行的推炸弹宏动作
///
/// \details
/// 未引爆时只说明炸弹被推到了中途，需要把对应任务的 bomb_start 改成新的炸弹位置
/// 已引爆时说明任务已经完成，直接从剩余任务列表中删除，避免后续规划继续看到一个已完成任务
void sync_bomb_tasks_after_push(StaticArray<BombTask, MAX_BOMBS>& tasks, const BombPushAction& action) {
    for (int i = 0; i < tasks.size(); ++i) {
        if (!matches_bomb_push_action(tasks[i], action)) continue;

        if (action.detonates) {
            remove_bomb_task_at(tasks, i);
        } else {
            tasks[i].bomb_start = action.bomb_target;
            tasks[i].box_pushes.clear();
        }
        return;
    }
}

/// \brief 应用真实执行完成后的推炸弹结果
/// \param lvl 需要原地更新的真实逻辑地图
/// \param tasks 需要同步的剩余炸弹任务列表
/// \param action 已完成执行的推炸弹宏动作
///
/// \details
/// 该接口用于 GameManage / Demo 的执行结算：先更新地图里的炸弹移动或爆炸结果，再同步策略层剩余任务
/// 规划器做分支模拟时不要调用这个接口
void apply_executed_bomb_push_result(SokobanLevel& lvl,
                                    StaticArray<BombTask, MAX_BOMBS>& tasks,
                                    const BombPushAction& action) {
    apply_bomb_push_action_effect(lvl, action);
    sync_bomb_tasks_after_push(tasks, action);
}

// ============================================================================
// 普通网格寻路
// ============================================================================

/// \brief 计算普通网格最短路距离
/// \param lvl 当前地图状态
/// \param start 起点
/// \param end 终点
/// \return 可达时返回最短步数，不可达时返回 65535
uint16_t bfs_shortest_path(const SokobanLevel& lvl, point start, point end) {
    if (start == end) return 0;
    if (!in_bounds(start) || !in_bounds(end)) return 65535;

    SimpleBfsWorkspace& ws = simple_bfs_ws;
    build_blocked_map(lvl, ws.blocked);
    uint16_t gen = next_simple_bfs_gen();

    int head = 0;
    int tail = 0;
    ws.q[tail++] = start;
    ws.visited_gen[start.y][start.x] = gen;
    ws.dist[start.y][start.x] = 0;

    while (head < tail) {
        point curr = ws.q[head++];
        if (curr == end) return ws.dist[curr.y][curr.x];

        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (in_bounds(np) && ws.visited_gen[np.y][np.x] != gen && !ws.blocked[np.y][np.x]) {
                ws.visited_gen[np.y][np.x] = gen;
                ws.dist[np.y][np.x] = ws.dist[curr.y][curr.x] + 1;
                ws.q[tail++] = np;
            }
        }
    }
    return 65535;
}


/// \brief 生成普通网格移动路径
/// \param lvl 当前地图状态
/// \param start 起点
/// \param end 终点
/// \param out_path 输出路径，不包含起点，包含终点
/// \return 成功找到路径时返回 true
bool get_grid_path(const SokobanLevel& lvl, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    out_path.clear();
    if (start == end) return true;
    if (!in_bounds(start) || !in_bounds(end)) return false;

    SimpleBfsWorkspace& ws = simple_bfs_ws;
    build_blocked_map(lvl, ws.blocked);
    uint16_t gen = next_simple_bfs_gen();

    int head = 0;
    int tail = 0;
    ws.q[tail++] = start;
    ws.visited_gen[start.y][start.x] = gen;
    bool found = false;

    while (head < tail) {
        point curr = ws.q[head++];
        if (curr == end) {
            found = true;
            break;
        }

        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (in_bounds(np) && ws.visited_gen[np.y][np.x] != gen && !ws.blocked[np.y][np.x]) {
                ws.visited_gen[np.y][np.x] = gen;
                ws.parent[np.y][np.x] = curr;
                ws.q[tail++] = np;
            }
        }
    }

    if (!found) return false;

    point curr = end;
    while (!(curr == start)) {
        out_path.push_back(curr);
        curr = ws.parent[curr.y][curr.x];
    }
    std::reverse(out_path.begin(), out_path.end());
    return true;
}

// ============================================================================
// 麦轮时间代价工具
// ============================================================================

/// \brief 统计一条已生成路径的执行时间
/// \param start 路径起点
/// \param path 不包含起点但包含终点的路径
/// \return 钳制到 uint16_t 的时间代价
///
/// \details
/// 每个格子计 GRID_MOVE，方向变化额外计 CORNER_STOP
/// path 与 get_grid_path 和推箱路径展开的输出格式保持一致
uint16_t path_time_cost(point start, const StaticArray<point, MAX_PATH_LENGTH>& path) {
    if (path.empty()) return 0;

    uint32_t cost = 0;
    int prev_dir = -1;
    point prev = start;

    for (int i = 0; i < path.size(); ++i) {
        point curr = path[i];
        int dir = direction_between(prev, curr);
        int span = std::abs(static_cast<int>(curr.x) - static_cast<int>(prev.x)) +
                   std::abs(static_cast<int>(curr.y) - static_cast<int>(prev.y));
        if (span <= 0) continue;

        cost += static_cast<uint32_t>(span) * MotionCost::GRID_MOVE;
        if (dir >= 0 && prev_dir >= 0 && dir != prev_dir) {
            cost += MotionCost::CORNER_STOP;
        }

        prev_dir = dir;
        prev = curr;
    }

    return clamp_time_cost(cost);
}

/// \brief 构建从起点到所有格子的麦轮时间代价图
/// \param lvl 当前地图状态
/// \param start 起点
/// \param out_cost 输出代价图，不可达格子填 MotionCost::INF
void build_grid_time_map(const SokobanLevel& lvl, point start, uint16_t out_cost[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    if (!run_grid_time_search(lvl, start, out_cost, nullptr)) {
        init_time_map(out_cost);
    }
}

/// \brief 查询两点间最短麦轮时间代价
/// \param lvl 当前地图状态
/// \param start 起点
/// \param end 终点
/// \return 可达时返回时间代价，不可达时返回 MotionCost::INF
uint16_t shortest_grid_time_cost(const SokobanLevel& lvl, point start, point end) {
    if (start == end) return 0;
    uint16_t cost_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    build_grid_time_map(lvl, start, cost_map);
    if (!in_bounds(end)) return MotionCost::INF;
    return cost_map[end.y][end.x];
}

/// \brief 生成带麦轮转向代价的最短时间路径
/// \param lvl 当前地图状态
/// \param start 起点
/// \param end 终点
/// \param out_path 输出路径，不包含起点，包含终点
/// \return 成功找到路径时返回 true
bool get_grid_time_path(const SokobanLevel& lvl, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    out_path.clear();
    if (!in_bounds(start) || !in_bounds(end)) return false;
    if (start == end) return true;

    uint16_t cost_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static TimeParent parent[MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];
    if (!run_grid_time_search(lvl, start, cost_map, parent)) return false;
    if (cost_map[end.y][end.x] == MotionCost::INF) return false;

    int best_dir = -1;
    uint32_t best = 0xFFFFFFFFU;
    for (int d = 0; d < 4; ++d) {
        TimeParent p = parent[end.y][end.x][d];
        if (p.x == -1) continue;

        StaticArray<point, MAX_PATH_LENGTH> probe;
        point curr = end;
        int curr_dir = d;
        bool ok = true;
        while (!(curr == start)) {
            probe.push_back(curr);
            TimeParent pp = parent[curr.y][curr.x][curr_dir];
            if (pp.x == -1) {
                ok = false;
                break;
            }
            curr = {pp.x, pp.y};
            if (pp.dir == 255) break;
            curr_dir = pp.dir;
        }
        if (!ok) continue;

        std::reverse(probe.begin(), probe.end());
        uint32_t c = path_time_cost(start, probe);
        if (c < best) {
            best = c;
            best_dir = d;
        }
    }

    if (best_dir < 0) return false;

    point curr = end;
    int curr_dir = best_dir;
    while (!(curr == start)) {
        out_path.push_back(curr);
        TimeParent p = parent[curr.y][curr.x][curr_dir];
        if (p.x == -1) {
            out_path.clear();
            return false;
        }
        curr = {p.x, p.y};
        if (p.dir == 255) break;
        curr_dir = p.dir;
    }

    std::reverse(out_path.begin(), out_path.end());
    return true;
}

/// \brief 计算两次观测朝向之间的转向代价
/// \param from_yaw 起始偏航角，负值表示忽略
/// \param to_yaw 目标偏航角，负值表示忽略
/// \return 需要明显转向时返回 OBSERVE_YAW，否则返回 0
uint16_t yaw_turn_time_cost(float from_yaw, float to_yaw) {
    if (from_yaw < 0.0f || to_yaw < 0.0f) return 0;
    float diff = std::abs(from_yaw - to_yaw);
    if (diff > 180.0f) diff = 360.0f - diff;
    return diff > 1.0f ? MotionCost::OBSERVE_YAW : 0;
}

// 判断玩家能否从起点走到目标点，可额外忽略一个动态物体并加入一个临时障碍
bool can_player_reach(const SokobanLevel& lvl, point start_pos, point target_pos, point ignored_obj, point extra_obs) {
    if (start_pos == target_pos) return true;

    OCRAM_BSS static uint16_t vis_gen[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];
    static uint16_t cur_vis_gen = 0;
    cur_vis_gen++;
    if (cur_vis_gen == 0) {
        std::memset(vis_gen, 0, sizeof(vis_gen));
        cur_vis_gen = 1;
    }

    OCRAM_BSS static point q[MAP_CELL_COUNT];
    int head = 0;
    int tail = 0;

    q[tail++] = start_pos;
    vis_gen[start_pos.y][start_pos.x] = cur_vis_gen;

    while (head < tail) {
        point curr = q[head++];
        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (np == target_pos) return true;
            if (np == extra_obs) continue;

            if (in_bounds(np) && vis_gen[np.y][np.x] != cur_vis_gen && !is_obstacle(lvl, np, ignored_obj)) {
                vis_gen[np.y][np.x] = cur_vis_gen;
                q[tail++] = np;
            }
        }
    }
    return false;
}

/// \brief 计算玩家可达区域
/// \param lvl 当前地图状态
/// \param start_pos 玩家起点
/// \param ignored_obj 可忽略的动态物体坐标
/// \param extra_obs 额外临时障碍坐标
/// \param out_visited 输出可达矩阵
void calc_player_reach(const SokobanLevel& lvl, point start_pos, point ignored_obj, point extra_obs, bool out_visited[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
    if (!in_bounds(start_pos)) {
        for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
            for (int x = 0; x < MAP_MAX_WIDTH; ++x) out_visited[y][x] = false;
        }
        return;
    }

    SimpleBfsWorkspace& ws = simple_bfs_ws;
    build_blocked_map(lvl, ws.blocked, ignored_obj, extra_obs);
    uint16_t gen = next_simple_bfs_gen();

    int head = 0;
    int tail = 0;

    ws.q[tail++] = start_pos;
    ws.visited_gen[start_pos.y][start_pos.x] = gen;

    while (head < tail) {
        point curr = ws.q[head++];
        for (int d = 0; d < 4; ++d) {
            point np = curr + MOVE[d];
            if (in_bounds(np) && ws.visited_gen[np.y][np.x] != gen && !ws.blocked[np.y][np.x]) {
                ws.visited_gen[np.y][np.x] = gen;
                ws.q[tail++] = np;
            }
        }
    }

    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            out_visited[y][x] = (ws.visited_gen[y][x] == gen);
        }
    }
}

// ============================================================================
// 推箱让路路径生成
// ============================================================================

// 判断单格是否是静态角落死锁位置
bool is_static_deadlock_cell(const SokobanLevel& lvl, point p) {
    if (!in_bounds(p) || lvl.map[p.y][p.x] == 1) return true;
    for (int t = 0; t < lvl.target_count; ++t) {
        if (lvl.targets[t] == p) return false;
    }

    point up_p = p + MOVE[2];
    point down_p = p + MOVE[0];
    point left_p = p + MOVE[3];
    point right_p = p + MOVE[1];
    bool up = !in_bounds(up_p) || lvl.map[up_p.y][up_p.x] == 1;
    bool down = !in_bounds(down_p) || lvl.map[down_p.y][down_p.x] == 1;
    bool left = !in_bounds(left_p) || lvl.map[left_p.y][left_p.x] == 1;
    bool right = !in_bounds(right_p) || lvl.map[right_p.y][right_p.x] == 1;

    return (up || down) && (left || right);
}

// 判断某个箱子位置是否形成无目标点的 2x2 死锁块
bool is_2x2_box_deadlock(const SokobanLevel& lvl, point p) {
    for (int oy = -1; oy <= 0; ++oy) {
        for (int ox = -1; ox <= 0; ++ox) {
            int solid_count = 0;
            bool has_target = false;
            for (int dy = 0; dy <= 1; ++dy) {
                for (int dx = 0; dx <= 1; ++dx) {
                    point q = {
                        static_cast<int8_t>(p.x + ox + dx),
                        static_cast<int8_t>(p.y + oy + dy)
                    };
                    if (!in_bounds(q) || lvl.map[q.y][q.x] == 1 || has_box(lvl, q)) ++solid_count;
                    for (int t = 0; t < lvl.target_count; ++t) {
                        if (lvl.targets[t] == q) has_target = true;
                    }
                }
            }
            if (solid_count == 4 && !has_target) return true;
        }
    }
    return false;
}

// 判断箱子是否仍存在通向候选目标集合的拓扑路径
bool box_has_candidate_target_path(const SokobanLevel& lvl, uint8_t box_id, uint16_t candidate_mask) {
    if (box_id >= lvl.box_count || candidate_mask == 0) return false;
    for (int t = 0; t < lvl.target_count; ++t) {
        if (!(candidate_mask & (1U << t))) continue;
        if (bfs_shortest_path(lvl, lvl.boxes[box_id], lvl.targets[t]) != 65535) return true;
    }
    return false;
}

// 判断箱子当前位置是否安全，不在静态死锁且仍能到达候选目标
bool is_box_position_safe(const SokobanLevel& lvl, uint8_t box_id, uint16_t candidate_mask) {
    if (box_id >= lvl.box_count) return false;
    point p = lvl.boxes[box_id];
    bool on_candidate_target = false;
    for (int t = 0; t < lvl.target_count; ++t) {
        if ((candidate_mask & (1U << t)) && lvl.targets[t] == p) {
            on_candidate_target = true;
            break;
        }
    }
    if (!on_candidate_target && is_static_deadlock_cell(lvl, p)) return false;
    if (!on_candidate_target && is_2x2_box_deadlock(lvl, p)) return false;
    return on_candidate_target || box_has_candidate_target_path(lvl, box_id, candidate_mask);
}

/// \brief 将一个推箱让路任务展开成底层移动路径
/// \param lvl 输入并原地更新的地图状态
/// \param player_pos 输入并原地更新的玩家位置
/// \param task 推箱让路任务
/// \param out_path 追加输出底层路径
/// \return 成功生成路径时返回 true
///
/// \details
/// 搜索分为宏层和微层
/// - 宏层搜索箱子的坐标和推动方向
/// - 微层搜索玩家如何绕到下一次发力点
/// 成功后会同步更新箱子位置和玩家位置
bool append_box_push_path(SokobanLevel& lvl, point& player_pos, const BoxPushTask& task, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    int moving_box = -1;
    for (int i = 0; i < lvl.box_count; ++i) {
        if (lvl.boxes[i] == task.box_start) {
            moving_box = i;
            break;
        }
    }
    if (moving_box == -1) {
        for (int i = 0; i < lvl.box_count; ++i) {
            if (lvl.boxes[i] == task.box_target) return true;
        }
        return false;
    }
    if (task.box_start == task.box_target) return true;

    std::memset(b_ws.visited, 0, sizeof(b_ws.visited));

    auto is_passable = [&](int x, int y) {
        if (x < 0 || x >= MAP_MAX_WIDTH || y < 0 || y >= MAP_MAX_HEIGHT) return false;
        if (lvl.map[y][x] == 1) return false;
        for (int i = 0; i < lvl.box_count; ++i) {
            if (i != moving_box && lvl.boxes[i].x == x && lvl.boxes[i].y == y) return false;
        }
        for (int i = 0; i < lvl.bomb_count; ++i) {
            if (lvl.bombs[i].x != -1 && lvl.bombs[i].x == x && lvl.bombs[i].y == y) return false;
        }
        return true;
    };

    auto check_micro_reachable = [&](point start, point end, point obstacle_box) {
        if (start == end) return true;
        if (!is_passable(end.x, end.y)) return false;

        b_ws.micro_gen++;
        if (b_ws.micro_gen == 0) {
            std::memset(b_ws.micro_visited, 0, sizeof(b_ws.micro_visited));
            b_ws.micro_gen = 1;
        }

        int h = 0, t = 0;
        b_ws.micro_q[t++] = start;
        b_ws.micro_visited[start.y][start.x] = b_ws.micro_gen;

        while (h < t) {
            point curr = b_ws.micro_q[h++];
            if (curr == end) return true;
            for (int d = 0; d < 4; ++d) {
                point np = curr + MOVE[d];
                if (np.x < 0 || np.x >= MAP_MAX_WIDTH || np.y < 0 || np.y >= MAP_MAX_HEIGHT) continue;
                if (b_ws.micro_visited[np.y][np.x] == b_ws.micro_gen) continue;
                if (is_passable(np.x, np.y) && !(np == obstacle_box)) {
                    b_ws.micro_visited[np.y][np.x] = b_ws.micro_gen;
                    b_ws.micro_q[t++] = np;
                }
            }
        }
        return false;
    };

    int head = 0, tail = 0;
    int target_node_idx = -1;

    for (int d = 0; d < 4; ++d) {
        point push_pos = {
            static_cast<int8_t>(task.box_start.x - MOVE[d].x),
            static_cast<int8_t>(task.box_start.y - MOVE[d].y)
        };
        if (is_passable(push_pos.x, push_pos.y) && check_micro_reachable(player_pos, push_pos, task.box_start)) {
            b_ws.visited[task.box_start.y][task.box_start.x] |= (1 << d);
            b_ws.q[tail++] = {task.box_start.x, task.box_start.y, (uint8_t)d, 65535};
        }
    }

    while (head < tail) {
        int curr_idx = head++;
        BombMacroNode curr = b_ws.q[curr_idx];

        if (curr.bx == task.box_target.x && curr.by == task.box_target.y) {
            target_node_idx = curr_idx;
            break;
        }

        point curr_p = {
            static_cast<int8_t>(curr.bx - MOVE[curr.p_dir].x),
            static_cast<int8_t>(curr.by - MOVE[curr.p_dir].y)
        };

        int nbx = curr.bx + MOVE[curr.p_dir].x;
        int nby = curr.by + MOVE[curr.p_dir].y;
        if (is_passable(nbx, nby)) {
            if (!(b_ws.visited[nby][nbx] & (1 << curr.p_dir))) {
                b_ws.visited[nby][nbx] |= (1 << curr.p_dir);
                b_ws.q[tail++] = {(int8_t)nbx, (int8_t)nby, curr.p_dir, (uint16_t)curr_idx};
            }
        }

        for (int d = 0; d < 4; ++d) {
            if (d == curr.p_dir) continue;
            point adj_p = {
                static_cast<int8_t>(curr.bx - MOVE[d].x),
                static_cast<int8_t>(curr.by - MOVE[d].y)
            };

            if (is_passable(adj_p.x, adj_p.y)) {
                if (!(b_ws.visited[curr.by][curr.bx] & (1 << d))) {
                    if (check_micro_reachable(curr_p, adj_p, {curr.bx, curr.by})) {
                        b_ws.visited[curr.by][curr.bx] |= (1 << d);
                        b_ws.q[tail++] = {curr.bx, curr.by, (uint8_t)d, (uint16_t)curr_idx};
                    }
                }
            }
        }
    }

    if (target_node_idx == -1) return false;

    StaticArray<BombMacroNode, 256> macro_path;
    int curr_idx = target_node_idx;
    while (curr_idx != 65535) {
        macro_path.push_back(b_ws.q[curr_idx]);
        curr_idx = b_ws.q[curr_idx].parent_idx;
    }
    std::reverse(macro_path.begin(), macro_path.end());

    auto append_micro_path = [&](point start, point end, point obstacle_box) -> bool {
        if (start == end) return true;
        b_ws.micro_gen++;
        if (b_ws.micro_gen == 0) {
            std::memset(b_ws.micro_visited, 0, sizeof(b_ws.micro_visited));
            b_ws.micro_gen = 1;
        }

        int h = 0, t = 0;
        bool found = false;
        b_ws.micro_q[t++] = start;
        b_ws.micro_visited[start.y][start.x] = b_ws.micro_gen;

        while (h < t) {
            point c = b_ws.micro_q[h++];
            if (c == end) {
                found = true;
                break;
            }
            for (int d = 0; d < 4; ++d) {
                point np = c + MOVE[d];
                if (np.x < 0 || np.x >= MAP_MAX_WIDTH || np.y < 0 || np.y >= MAP_MAX_HEIGHT) continue;
                if (b_ws.micro_visited[np.y][np.x] == b_ws.micro_gen) continue;
                if (is_passable(np.x, np.y) && !(np == obstacle_box)) {
                    b_ws.micro_visited[np.y][np.x] = b_ws.micro_gen;
                    b_ws.micro_parent[np.y][np.x] = c;
                    b_ws.micro_q[t++] = np;
                }
            }
        }
        if (!found) return false;

        StaticArray<point, 256> temp;
        point curr_p = end;
        while (!(curr_p == start)) {
            temp.push_back(curr_p);
            curr_p = b_ws.micro_parent[curr_p.y][curr_p.x];
        }
        for (int i = temp.size() - 1; i >= 0; --i) out_path.push_back(temp[i]);
        return true;
    };

    point current_car_pos = player_pos;
    point first_push_pos = {
        static_cast<int8_t>(macro_path[0].bx - MOVE[macro_path[0].p_dir].x),
        static_cast<int8_t>(macro_path[0].by - MOVE[macro_path[0].p_dir].y)
    };
    if (!append_micro_path(current_car_pos, first_push_pos, task.box_start)) return false;
    current_car_pos = first_push_pos;

    for (int i = 0; i < macro_path.size() - 1; ++i) {
        BombMacroNode c_node = macro_path[i];
        BombMacroNode n_node = macro_path[i + 1];

        if (c_node.bx != n_node.bx || c_node.by != n_node.by) {
            point step_into = {c_node.bx, c_node.by};
            out_path.push_back(step_into);
            current_car_pos = step_into;
        } else {
            point target_face = {
                static_cast<int8_t>(n_node.bx - MOVE[n_node.p_dir].x),
                static_cast<int8_t>(n_node.by - MOVE[n_node.p_dir].y)
            };
            if (!append_micro_path(current_car_pos, target_face, {c_node.bx, c_node.by})) return false;
            current_car_pos = target_face;
        }
    }

    point final_car_pos = {
        static_cast<int8_t>(macro_path.back().bx - MOVE[macro_path.back().p_dir].x),
        static_cast<int8_t>(macro_path.back().by - MOVE[macro_path.back().p_dir].y)
    };
    if (out_path.empty() || out_path.back() != final_car_pos) out_path.push_back(final_car_pos);

    lvl.boxes[moving_box] = task.box_target;
    player_pos = final_car_pos;
    return true;
}

/// \brief 代价优先搜索推炸弹宏路径
/// \details
/// 宏状态是“炸弹坐标 + 小车发力方向”。旧 BFS 把推一步和绕到另一侧发力都当成同代价边，
/// 会偏向宏步数短但小车实际绕行很长的方案。这里用 Dijkstra 把换发力位的微层步数计入边权，
/// 让策略评分和真实路径展开共享同一套“顺手程度”判断
static bool find_bomb_macro_path(
    const SokobanLevel& lvl,
    point player_start,
    point bomb_start,
    point target,
    StaticArray<BombMacroNode, 256>& out_macro_path,
    uint16_t& out_cost,
    point& out_final_player)
{
    out_macro_path.clear();
    out_cost = 0;
    out_final_player = player_start;

    if (!in_bounds(bomb_start) || !in_bounds(target)) return false;
    if (bomb_start == target) return true;
    if (lvl.map[target.y][target.x] == 1 && !is_blastable_wall(lvl, target)) return false;

    uint8_t blocked[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    for (int i = 0; i < lvl.box_count; ++i) {
        point p = lvl.boxes[i];
        if (in_bounds(p)) blocked[p.y][p.x] = 1;
    }
    for (int i = 0; i < lvl.bomb_count; ++i) {
        point p = lvl.bombs[i];
        if (!in_bounds(p) || p == bomb_start) continue;
        blocked[p.y][p.x] = 1;
    }

    auto is_passable = [&](int x, int y, bool is_bomb_moving) {
        point p = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
        if (!in_bounds(p)) return false;
        if (lvl.map[y][x] == 1) {
            if (!(is_bomb_moving && p == target && is_blastable_wall(lvl, p))) return false;
        }
        if (blocked[y][x]) return false;
        return true;
    };

    auto micro_distance = [&](point start, point end, point obstacle_bomb) {
        if (start == end) return 0;
        if (!is_passable(end.x, end.y, false)) return 9999;

        b_ws.micro_gen++;
        if (b_ws.micro_gen == 0) {
            std::memset(b_ws.micro_visited, 0, sizeof(b_ws.micro_visited));
            b_ws.micro_gen = 1;
        }

        int h = 0;
        int t = 0;
        b_ws.micro_q[t++] = start;
        b_ws.micro_visited[start.y][start.x] = b_ws.micro_gen;
        b_ws.micro_dist[start.y][start.x] = 0;

        while (h < t) {
            point curr = b_ws.micro_q[h++];
            if (curr == end) return static_cast<int>(b_ws.micro_dist[curr.y][curr.x]);
            for (int d = 0; d < 4; ++d) {
                point np = curr + MOVE[d];
                if (!in_bounds(np)) continue;
                if (b_ws.micro_visited[np.y][np.x] == b_ws.micro_gen) continue;
                if (is_passable(np.x, np.y, false) && !(np == obstacle_bomb)) {
                    b_ws.micro_visited[np.y][np.x] = b_ws.micro_gen;
                    b_ws.micro_dist[np.y][np.x] = static_cast<uint8_t>(b_ws.micro_dist[curr.y][curr.x] + 1);
                    b_ws.micro_q[t++] = np;
                }
            }
        }
        return 9999;
    };

    std::memset(b_ws.state_cost, 0xFF, sizeof(b_ws.state_cost));
    std::memset(b_ws.state_node, 0xFF, sizeof(b_ws.state_node));
    std::memset(b_ws.state_closed, 0, sizeof(b_ws.state_closed));

    int node_count = 0;
    bool overflow = false;
    uint16_t heap[1024];
    int heap_size = 0;

    auto heap_less = [&](uint16_t a, uint16_t b) {
        return b_ws.node_cost[a] < b_ws.node_cost[b];
    };
    auto heap_push = [&](uint16_t idx) {
        if (heap_size >= 1024) {
            overflow = true;
            return;
        }
        int i = heap_size++;
        heap[i] = idx;
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (!heap_less(heap[i], heap[parent])) break;
            uint16_t tmp = heap[i];
            heap[i] = heap[parent];
            heap[parent] = tmp;
            i = parent;
        }
    };
    auto heap_pop = [&]() -> int {
        if (heap_size <= 0) return -1;
        uint16_t root = heap[0];
        heap[0] = heap[--heap_size];
        int i = 0;
        while (true) {
            int left = i * 2 + 1;
            int right = left + 1;
            int best = i;
            if (left < heap_size && heap_less(heap[left], heap[best])) best = left;
            if (right < heap_size && heap_less(heap[right], heap[best])) best = right;
            if (best == i) break;
            uint16_t tmp = heap[i];
            heap[i] = heap[best];
            heap[best] = tmp;
            i = best;
        }
        return static_cast<int>(root);
    };

    auto keep_state = [&](point bomb_pos, uint8_t dir, uint16_t cost, uint16_t parent_idx) {
        if (!in_bounds(bomb_pos) || dir >= 4) return;
        uint16_t& best = b_ws.state_cost[bomb_pos.y][bomb_pos.x][dir];
        if (cost >= best) return;
        if (node_count >= 1024) {
            overflow = true;
            return;
        }
        int idx = node_count++;
        b_ws.q[idx] = {bomb_pos.x, bomb_pos.y, dir, parent_idx};
        b_ws.node_cost[idx] = cost;
        best = cost;
        b_ws.state_node[bomb_pos.y][bomb_pos.x][dir] = static_cast<uint16_t>(idx);
        heap_push(static_cast<uint16_t>(idx));
    };

    for (int d = 0; d < 4; ++d) {
        point push_pos = bomb_start - MOVE[d];
        int walk = micro_distance(player_start, push_pos, bomb_start);
        if (walk != 9999) {
            keep_state(bomb_start, static_cast<uint8_t>(d), static_cast<uint16_t>(walk), 65535);
        }
    }

    int target_node_idx = -1;
    while (!overflow && heap_size > 0) {
        int best_idx = heap_pop();
        if (best_idx < 0) break;
        BombMacroNode curr = b_ws.q[best_idx];
        if (b_ws.state_closed[curr.by][curr.bx][curr.p_dir]) continue;
        if (b_ws.node_cost[best_idx] != b_ws.state_cost[curr.by][curr.bx][curr.p_dir]) continue;
        uint16_t best_cost = b_ws.node_cost[best_idx];
        b_ws.state_closed[curr.by][curr.bx][curr.p_dir] = 1;
        point curr_bomb = {curr.bx, curr.by};

        if (curr_bomb == target) {
            target_node_idx = best_idx;
            break;
        }

        point forward = curr_bomb + MOVE[curr.p_dir];
        if (is_passable(forward.x, forward.y, true) && best_cost < UINT16_MAX) {
            uint32_t next_cost = static_cast<uint32_t>(best_cost) + 1;
            if (next_cost < UINT16_MAX) {
                keep_state(forward, curr.p_dir, static_cast<uint16_t>(next_cost), static_cast<uint16_t>(best_idx));
            }
        }

        point curr_car = curr_bomb - MOVE[curr.p_dir];
        for (int d = 0; d < 4; ++d) {
            if (d == curr.p_dir) continue;
            point next_face = curr_bomb - MOVE[d];
            if (!is_passable(next_face.x, next_face.y, false)) continue;
            int walk = micro_distance(curr_car, next_face, curr_bomb);
            if (walk == 9999) continue;
            uint32_t next_cost = static_cast<uint32_t>(best_cost) + static_cast<uint32_t>(walk);
            if (next_cost < UINT16_MAX) {
                keep_state(curr_bomb, static_cast<uint8_t>(d), static_cast<uint16_t>(next_cost), static_cast<uint16_t>(best_idx));
            }
        }
    }

    if (overflow || target_node_idx < 0) return false;

    int curr_idx = target_node_idx;
    while (curr_idx != 65535) {
        out_macro_path.push_back(b_ws.q[curr_idx]);
        curr_idx = b_ws.q[curr_idx].parent_idx;
    }
    std::reverse(out_macro_path.begin(), out_macro_path.end());
    if (out_macro_path.empty()) return false;

    out_cost = b_ws.node_cost[target_node_idx];
    BombMacroNode last = out_macro_path.back();
    out_final_player = {
        static_cast<int8_t>(last.bx - MOVE[last.p_dir].x),
        static_cast<int8_t>(last.by - MOVE[last.p_dir].y)
    };
    return true;
}

bool get_direct_bomb_push_path_cost(
    const SokobanLevel& lvl,
    point player_start,
    const BombTask& task,
    uint16_t& out_cost,
    point& out_final_player) {
    out_cost = 0;
    out_final_player = {-1, -1};
    if (task.bomb_start.x == -1 || task.target_wall.x == -1) return false;
    if (task.box_pushes.size() > 0) return false;
    StaticArray<BombMacroNode, 256> macro_path;
    return find_bomb_macro_path(
        lvl,
        player_start,
        task.bomb_start,
        task.target_wall,
        macro_path,
        out_cost,
        out_final_player);
}

// ============================================================================
// 推炸弹路径生成
// ============================================================================

/// \brief 将一个炸弹宏任务展开成底层移动路径
/// \param lvl 当前地图状态
/// \param player_start 玩家起点
/// \param task 炸弹宏任务，可能包含推箱让路子任务
/// \param out_path 输出完整底层路径
/// \return 成功生成路径时返回 true
///
/// \details
/// 函数会先顺序执行 task.box_pushes 中的推箱让路任务，
/// 然后用与推箱类似的宏/微双层搜索，把炸弹推到 target_wall
bool get_bomb_push_path(const SokobanLevel& lvl, point player_start, const BombTask& task, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    out_path.clear();
    SokobanLevel working_lvl = lvl;
    point working_player = player_start;
    if (!in_bounds(task.bomb_start) || !in_bounds(task.target_wall)) return false;
    for (int i = 0; i < task.box_pushes.size(); ++i) {
        if (!append_box_push_path(working_lvl, working_player, task.box_pushes[i], out_path)) {
            out_path.clear();
            return false;
        }
    }
    if (working_lvl.map[task.target_wall.y][task.target_wall.x] == 1 &&
        !is_blastable_wall(working_lvl, task.target_wall)) {
        out_path.clear();
        return false;
    }

    StaticArray<BombMacroNode, 256> macro_path;
    uint16_t macro_cost = 0;
    point final_player = {-1, -1};
    if (!find_bomb_macro_path(
            working_lvl,
            working_player,
            task.bomb_start,
            task.target_wall,
            macro_path,
            macro_cost,
            final_player)) {
        out_path.clear();
        return false;
    }
    if (macro_path.empty()) return true;

    const SokobanLevel& cur_lvl = working_lvl;
    auto is_passable = [&](int x, int y, bool is_bomb_moving) {
        point p = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
        if (!in_bounds(p)) return false;
        if (cur_lvl.map[y][x] == 1) {
            if (!(is_bomb_moving && p == task.target_wall && is_blastable_wall(cur_lvl, p))) return false;
        }
        for (int i = 0; i < cur_lvl.box_count; ++i) {
            if (cur_lvl.boxes[i].x == x && cur_lvl.boxes[i].y == y) return false;
        }
        for (int i = 0; i < cur_lvl.bomb_count; ++i) {
            if (cur_lvl.bombs[i].x != -1 && cur_lvl.bombs[i].x == x && cur_lvl.bombs[i].y == y) {
                if (cur_lvl.bombs[i] == task.bomb_start) continue;
                return false;
            }
        }
        return true;
    };

    auto append_micro_path = [&](point start, point end, point obstacle_bomb) -> bool {
        if (start == end) return true;
        if (!is_passable(end.x, end.y, false)) return false;
        b_ws.micro_gen++;
        if (b_ws.micro_gen == 0) {
            std::memset(b_ws.micro_visited, 0, sizeof(b_ws.micro_visited));
            b_ws.micro_gen = 1;
        }

        int h = 0, t = 0;
        b_ws.micro_q[t++] = start;
        b_ws.micro_visited[start.y][start.x] = b_ws.micro_gen;

        bool found = false;
        while (h < t) {
            point c = b_ws.micro_q[h++];
            if (c == end) {
                found = true;
                break;
            }
            for (int d = 0; d < 4; ++d) {
                point np = c + MOVE[d];
                if (!in_bounds(np)) continue;
                if (b_ws.micro_visited[np.y][np.x] != b_ws.micro_gen) {
                    if (is_passable(np.x, np.y, false) && !(np == obstacle_bomb)) {
                        b_ws.micro_visited[np.y][np.x] = b_ws.micro_gen;
                        b_ws.micro_parent[np.y][np.x] = c;
                        b_ws.micro_q[t++] = np;
                    }
                }
            }
        }
        if (!found) return false;

        StaticArray<point, 256> temp;
        point curr_p = end;
        while (!(curr_p == start)) {
            temp.push_back(curr_p);
            curr_p = b_ws.micro_parent[curr_p.y][curr_p.x];
        }
        for (int i = temp.size() - 1; i >= 0; --i) out_path.push_back(temp[i]);
        return true;
    };

    point current_car_pos = working_player;
    point first_push_pos = {
        static_cast<int8_t>(macro_path[0].bx - MOVE[macro_path[0].p_dir].x),
        static_cast<int8_t>(macro_path[0].by - MOVE[macro_path[0].p_dir].y)
    };
    if (!append_micro_path(current_car_pos, first_push_pos, task.bomb_start)) {
        out_path.clear();
        return false;
    }
    current_car_pos = first_push_pos;

    for (int i = 0; i < macro_path.size() - 1; ++i) {
        BombMacroNode c_node = macro_path[i];
        BombMacroNode n_node = macro_path[i + 1];

        if (c_node.bx != n_node.bx || c_node.by != n_node.by) {
            point step_into = {c_node.bx, c_node.by};
            out_path.push_back(step_into);
            current_car_pos = step_into;
        } else {
            point target_face = {
                static_cast<int8_t>(n_node.bx - MOVE[n_node.p_dir].x),
                static_cast<int8_t>(n_node.by - MOVE[n_node.p_dir].y)
            };
            if (!append_micro_path(current_car_pos, target_face, {c_node.bx, c_node.by})) {
                out_path.clear();
                return false;
            }
            current_car_pos = target_face;
        }
    }

    point final_car_pos = {
        static_cast<int8_t>(macro_path.back().bx - MOVE[macro_path.back().p_dir].x),
        static_cast<int8_t>(macro_path.back().by - MOVE[macro_path.back().p_dir].y)
    };
    if (out_path.empty() || out_path.back() != final_car_pos) out_path.push_back(final_car_pos);

    return true;
}

}
