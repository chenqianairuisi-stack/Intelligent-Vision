/// \file Sokoban.h
/// \brief C++ 宏动作候选与 IDA* 混合推箱子求解器
///
/// \details
/// 定义推箱搜索状态、换位表、性能统计和求解器接口
/// 求解器接收逻辑地图、语义绑定和炸弹任务，输出固定容量的最终执行路径

#pragma once

#include <array>
#include <cstring>
#include "PlanningCommon.h"
#include "system_config.h"

using namespace SystemConfig;


// ============================================================================
// IDA* 搜索状态
// ============================================================================

struct GameState {
    point player;              // 玩家当前位置

    int8_t box_x[MAX_BOXES];   // 箱子 x 坐标
    int8_t box_y[MAX_BOXES];   // 箱子 y 坐标
    uint8_t box_semantics[MAX_BOXES]; // 当前箱子的语义编号
    uint8_t num_boxes;          // 当前仍未归位的箱子数量
    uint16_t target_mask;       // 尚未完成的目标集合，bit=1 表示该目标仍需处理

    int8_t bomb_x[MAX_BOMBS];  // 炸弹 x 坐标
    int8_t bomb_y[MAX_BOMBS];  // 炸弹 y 坐标
    uint8_t num_bombs;         // 当前地图中的炸弹数量
    uint8_t blown_mask;        // 已引爆炸弹集合，bit=1 表示对应炸弹已经清墙

    uint32_t hash;             // Zobrist 哈希
};

// ============================================================================
// 置换表
// ============================================================================

constexpr uint32_t TT_SIZE = 32768;

struct alignas(4) TTEntry {
    uint16_t sig;              // hash 高 16 位签名
    uint16_t value;            // 该状态已知不可行的剩余阈值下界
};

// ============================================================================
// 推箱子核心求解器
// ============================================================================

// ============================================================================
// Sokoban 搜索性能探针
// ============================================================================

struct SokobanProfile {
    uint32_t expanded_nodes = 0;          // 真正展开并生成动作的搜索节点数
    uint32_t generated_moves = 0;         // 所有节点累计生成的候选推动作数量
    uint32_t tt_hits = 0;                 // 置换表命中并剪枝的次数
    uint32_t heuristic_dead_prunes = 0;   // 启发式判断不可达/死锁而剪枝的次数
    uint32_t threshold_prunes = 0;        // IDA* f 值超过当前阈值的剪枝次数
    uint32_t path_cycle_prunes = 0;       // 当前递归路径内状态重复的剪枝次数
    uint32_t static_deadlock_prunes = 0;  // 推入静态死锁格或目标距离场不可达的剪枝次数
    uint32_t block_2x2_prunes = 0;        // 2x2 团块死锁剪枝次数
    uint16_t max_depth = 0;               // 本次搜索达到的最大递归深度
    uint16_t threshold_iterations = 0;    // IDA* 阈值迭代轮数
    uint16_t final_threshold = 0;         // 搜索结束时的阈值，便于对比启发式强弱
};

enum class SokobanHeuristicMode : uint8_t {
    PURE,       // 纯推箱使用保守等权启发
    SEMANTIC    // 语义推箱按活跃实体数提高权重
};

class Sokoban {
public:
    // --- 生命周期与外部输入 ---
    // 导入当前地图快照、炸弹任务和本轮启发式模式
    bool load_from_vision(
        const SokobanLevel& level,
        const BombTask* tasks,
        int count,
        SokobanHeuristicMode heuristic_mode);
    // 校验已导入的箱子/目标点语义，移除已落位箱子，并统一完成预计算
    bool bind_semantics();

    // --- 求解入口与结果读取 ---
    // 按统一语义规则运行 IDA*；成功后可通过 get_result_path 读取完整执行路径
    bool solve();
    // 只用宏层快速验证当前候选任务能否接上语义推箱，不进入完整 IDA* 失败证明
    bool solve_macro_candidate();
    // 返回最近一次成功求解得到的玩家路径，包含行走和推动过程
    const StaticArray<point, MAX_PATH_LENGTH>& get_result_path() const { return final_path; }
    // 返回需要在爆破落脚点等待的原始路径索引
    const StaticArray<uint16_t, MAX_BOMBS>& get_explosion_wait_indices() const {
        return explosion_wait_indices;
    }
    // 返回最近一次求解的搜索统计，用于分析阈值、剪枝和扩展规模
    const SokobanProfile& get_profile() const { return profile; }
    bool profile_enabled() const;

private:
    // 每个搜索节点最多保留的普通一格推动作数量；限制栈空间和排序开销
    static constexpr int MAX_NODE_MOVES = 24;
    // 每个节点最多生成的炸弹宏动作数量，最多对应当前地图中的炸弹数
    static constexpr int MAX_NODE_MACROS = MAX_BOMBS;
    // 普通动作与宏动作合并排序后的候选上限
    static constexpr int MAX_NODE_ACTIONS = MAX_NODE_MOVES + MAX_NODE_MACROS;

    // 单个普通推动作：只记录“推哪个实体、往哪推、玩家需走多远”等增量信息
    struct TinyMove {
        uint8_t entity_idx;         // [0, num_boxes) 为箱子；其后为炸弹编号偏移
        uint8_t dir;                // 推动方向，对应 MOVE[dir]
        uint8_t walk_dist;          // 玩家从当前位置走到推位的距离
        uint8_t slide_dist;         // 隧道自动滑行的额外格数
        bool triggers_explosion;    // 本次推动是否直接触发炸弹清墙
    };

    // 统一排序条目：move_idx 指向 TinyMove 或 MacroMove，由 is_macro 区分
    struct EvalMove {
        uint8_t move_idx;           // 对应动作数组中的原始下标
        uint8_t is_macro;           // 0=普通推动作，1=炸弹宏动作
        int16_t sort_key;           // 越小越优先尝试
    };

    // 可选炸弹宏动作：把“推炸弹直到完成爆破任务”折叠为一条 successor 边
    // 普通推箱/推炸弹动作仍然保留，宏动作只作为额外候选参与同一套排序
    struct MacroMove {
        uint8_t bomb_idx;           // 当前要完成的炸弹任务编号
        uint8_t entity_idx;         // 与普通动作排序/递归记忆共用的实体编号
        uint8_t final_push_dir;     // 宏动作最后一次把炸弹推入目标墙的方向
        uint16_t path_cost;         // 展开后的真实网格步数
        int16_t sort_key;           // 越小越优先尝试
        GameState next_state;       // 完成爆破后的搜索状态
    };

    static constexpr int MACRO_COST_CACHE_SIZE = 512;
    struct MacroCostCacheEntry {
        uint32_t hash;
        uint16_t cost;
        int8_t final_x;
        int8_t final_y;
        uint8_t bomb_idx;
        uint8_t final_dir;
        uint8_t flags;              // bit0=valid, bit1=success
        uint8_t reserved;
    };

    // 单节点动态占用表：以 192B 栈空间换掉热路径内反复线性扫描箱子/炸弹数组
    // cell 中 0..15 表示箱子下标，BOMB_BASE 之后表示炸弹下标，-1 表示空格
    struct NodeOccupancy {
        static constexpr int8_t EMPTY = -1;
        static constexpr int8_t BOMB_BASE = 16;
        int8_t cell[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];

        inline void build(const GameState& state) {
            std::memset(cell, EMPTY, sizeof(cell));
            for (int i = 0; i < state.num_boxes; ++i) {
                cell[state.box_y[i]][state.box_x[i]] = static_cast<int8_t>(i);
            }
            for (int b = 0; b < state.num_bombs; ++b) {
                if (!(state.blown_mask & (1 << b))) {
                    cell[state.bomb_y[b]][state.bomb_x[b]] = static_cast<int8_t>(BOMB_BASE + b);
                }
            }
        }

        inline int box_at(point p) const {
            if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return -1;
            int v = cell[p.y][p.x];
            return (v >= 0 && v < BOMB_BASE) ? v : -1;
        }

        inline int bomb_at(point p) const {
            if (p.x < 0 || p.x >= MAP_MAX_WIDTH || p.y < 0 || p.y >= MAP_MAX_HEIGHT) return -1;
            int v = cell[p.y][p.x];
            return (v >= BOMB_BASE) ? (v - BOMB_BASE) : -1;
        }
    };
    // --- 求解入口与首解修复 ---
    // IDA* 外层驱动：初始化根状态、阈值和置换表，并逐轮加深
    bool solve_internal();
    // 纯推箱先取任务级宏首解，再按固定节点预算运行严格 IDA* 改进
    bool try_pure_box_hybrid_solution();
    // 高级语义模式先按炸弹任务和箱子任务构造有界首解
    bool try_semantic_task_solution();
    // 运行一次 IDA*，strict_cost=true 时按真实步数做受限修复搜索
    bool run_ida_search(
        bool strict_cost,
        int max_threshold,
        StaticArray<point, MAX_PATH_LENGTH>& out_path,
        uint32_t node_budget = 0);
    // 使用当前首解作为上界，只搜索真实步数更短的路径，避免修复阶段把预算耗在低阈值证明上
    bool run_bounded_strict_search(
        int max_cost,
        StaticArray<point, MAX_PATH_LENGTH>& out_path,
        uint32_t node_budget);
    // 用快速首解作为上界，尝试在小预算内找真实步数更短的解
    bool try_strict_cost_repair(const StaticArray<point, MAX_PATH_LENGTH>& candidate_path);

    // --- 纯推箱任务级宏搜索 ---
    // 纯推箱按“完成一个箱子任务”为宏边快速搜索候选解
    bool try_box_task_candidate(StaticArray<point, MAX_PATH_LENGTH>& out_path) const;
    // 从指定状态按固定分支预算搜索箱子任务候选解
    bool try_box_task_candidate_from_state(
        const GameState& state,
        uint32_t branch_budget,
        bool include_entry_walk,
        StaticArray<point, MAX_PATH_LENGTH>& out_path) const;
    // 有界枚举剩余炸弹任务顺序并衔接箱子任务候选解
    bool search_semantic_task_candidate(
        const GameState& state,
        const StaticArray<point, MAX_PATH_LENGTH>& path,
        uint32_t& bomb_branch_budget,
        StaticArray<point, MAX_PATH_LENGTH>& out_path) const;
    // 递归搜索首个可行的箱子完成顺序和目标分配
    bool search_box_task_candidate(
        const SokobanLevel& level,
        point player,
        uint16_t remaining_targets,
        const StaticArray<point, MAX_PATH_LENGTH>& path,
        StaticArray<point, MAX_PATH_LENGTH>& out_path,
        uint32_t& branch_budget,
        bool include_entry_walk) const;
    // 回放并校验一条箱子任务宏边符合求解器的落位规则
    bool validate_box_task_segment(
        const SokobanLevel& level,
        point player,
        int moving_box,
        uint8_t semantic_id,
        uint16_t remaining_targets,
        uint16_t target_bit,
        int segment_begin,
        const StaticArray<point, MAX_PATH_LENGTH>& path) const;

    // --- IDA* 搜索热路径 ---
    // 单次阈值内的递归搜索；返回 -1 表示找到解，其它值用于推进下一轮阈值
    int ida_star_search(
        const GameState& state,
        int g,
        int depth,
        int threshold,
        StaticArray<point, MAX_PATH_LENGTH>& path,
        int last_entity = -1,
        uint8_t last_push_dir = 4,
        int known_h = -1,
        int known_active_bombs = -1,
        bool tt_already_clear = false);
    // 统计尚未完成且有目标墙的炸弹任务数
    int count_active_bomb_tasks(const GameState& state) const;
    // 仅在仍有炸弹任务时估计箱子推动压力，用于动作排序
    int box_push_lb_sum_if_needed(const GameState& state, int active_bombs) const;
    // 返回 IDA* 启发式权重的分子；分母在 Sokoban.cpp 的参数面板中配置
    int heuristic_weight_num(int active_entities) const;
    // 查询置换表中“该状态在当前剩余阈值下已失败”的记录
    int probe_transposition(uint32_t hash, int g, int threshold);
    // 记录当前失败分支推导出的下一轮最小阈值，供后续等价状态剪枝
    void store_transposition(uint32_t hash, int g, int min_next_threshold);
    // 查找格子是否命中当前箱子的同语义有效目标
    int find_active_target_index(const GameState& state, uint16_t target_mask, point p, int box_idx) const;
    // 判断格子是否是当前箱子的有效目标；包装 find_active_target_index，供热路径死锁判定使用
    bool is_active_target_cell(const GameState& state, point p, int box_idx, int& out_idx) const;
    // 判断当前位置沿推动方向是否处于动态隧道，用于自动滑行压缩推动作
    bool is_tunnel_dynamic(point p, int dir, uint8_t blown_mask) const;
    // 基于当前玩家可达区生成普通推箱/推炸弹动作
    int generate_moves(
        const GameState& state,
        const NodeOccupancy& occupancy,
        int active_bombs,
        TinyMove moves[MAX_NODE_MOVES]);
    // 生成可选炸弹宏动作；只在宏动作开关启用时产生候选
    int generate_bomb_macros(
        const GameState& state,
        const NodeOccupancy& occupancy,
        int h_before,
        int active_entities,
        int g,
        int threshold,
        MacroMove macros[MAX_NODE_MACROS]) const;
    // 把搜索状态还原成临时关卡，供炸弹路径规划复用通用接口
    void build_level_from_state(const GameState& state, SokobanLevel& out_level) const;
    // 计算某个炸弹宏动作的完整玩家路径，并做长度/合法性检查
    bool build_bomb_macro_path(
        const GameState& state,
        int bomb_idx,
        StaticArray<point, MAX_PATH_LENGTH>& out_path) const;
    // 将已验证的复合炸弹任务同步到搜索状态
    bool build_bomb_task_successor_state(
        const GameState& state,
        int bomb_idx,
        const BombTask& task,
        point final_player,
        GameState& out_state) const;
    // 根据宏路径末端玩家位置反推最后一次推炸弹的方向；4 表示无法推向目标墙
    uint8_t infer_final_bomb_push_dir(point final_player, point target_wall) const;
    // 成功求解后优化同箱连续推动轨迹和玩家行走段转弯
    void optimize_final_path_turns();
    void build_explosion_wait_indices();
    // 对边界状态一致的同箱连续推动片段做代价优先重规划
    void optimize_final_box_push_runs();
    // 为相邻推动作之间的一段行走重新寻路，优先减少转弯并限制额外步数
    bool append_optimized_walk_segment(
        const GameState& state,
        point start,
        point goal,
        uint8_t prev_dir,
        uint8_t next_dir,
        int max_steps,
        StaticArray<point, MAX_PATH_LENGTH>& out_path,
        uint8_t& out_dir) const;

    // --- 启发式与哈希 ---
    // 计算当前状态的乐观剩余代价；9999 表示启发式已判定不可解
    int get_heuristic(const GameState& state) const;
    // 估计多任务之间玩家切换所需的最小行走下界
    int task_route_walk_lower_bound(
        point player,
        const point starts[MAX_BOXES + MAX_BOMBS],
        const point ends[MAX_BOXES + MAX_BOMBS],
        int task_count) const;
    // 返回箱子到当前仍未完成的同语义目标点的最短反向推动距离
    int nearest_active_target_distance(const GameState& state, uint8_t semantic_id, point box_pos) const;
    // 小规模最小权匹配
    template <size_t N>
    int min_weight_assignment(int cost[N][N], int n) const;
    // 按语义规则计算状态哈希，纳入箱子语义、炸弹位置和 blown_mask
    uint32_t compute_hash(const GameState& state) const;

    // --- 空间预计算 ---
    // 初始化 Zobrist 随机表
    void init_zobrist();
    // 预计算每个目标到所有格子的反向推箱距离
    void precompute_target_distances();
    // 预计算单箱推到目标的乐观总代价下界
    void precompute_box_target_costs();
    // 解出忽略其他动态实体的单箱参考路径
    void precompute_box_reference_paths();
    // 预计算每个炸弹到目标墙的反向推动距离
    void precompute_bomb_distances();
    void precompute_bomb_macro_costs();
    void precompute_solid_masks();
    // 预计算静态地图上的任意两格行走距离下界
    void precompute_walk_distances();
    // 标记静态死锁格，供动作生成时快速剪枝
    void precompute_deadlocks();

    // --- 高频物理查询 ---
    // 越界检查
    inline bool is_overstep(point p) const;
    // 动态实体墙检查：已被炸开的墙不再阻挡
    inline bool is_solid(point p, uint8_t blown_mask) const;
    // 查询初始炸弹列表中是否存在该格；用于兼容旧逻辑和预计算
    inline bool is_bomb(point p) const;
    // 在线性状态数组中查找箱子编号
    inline int find_box_id(const GameState& state, point p) const;
    // 查询炸弹编号
    inline int get_bomb_id(const GameState& state, point p) const;
    // 静态行走距离表查询
    inline int walk_dist_between(point from, point to) const;
    // 玩家走到物体相邻推位的乐观下界
    inline int walk_to_push_stand_lower_bound(point from, point obj) const;

    // --- 静态地图、初始状态与求解结果 ---
    std::array<std::array<int8_t, MAP_MAX_WIDTH>, MAP_MAX_HEIGHT> map{}; // 静态地图：0=空地，1=墙
    GameState initial_state;                                             // 搜索根状态，会随语义绑定/炸弹任务更新
    StaticArray<point, MAX_BOXES> initial_targets;                       // 初始目标点列表
    uint8_t target_semantics[MAX_BOXES] = {};                            // 每个目标点的语义编号
    StaticArray<point, MAX_BOMBS> initial_bombs;                         // 初始炸弹位置列表
    StaticArray<point, MAX_PATH_LENGTH> final_path;                      // 最近一次成功求解的完整玩家路径
    StaticArray<uint16_t, MAX_BOMBS> explosion_wait_indices;            // 后续路径需穿新清墙时的爆破落脚点
    StaticArray<BombTask, MAX_BOMBS> bomb_tasks;                         // 按初始炸弹编号索引的清墙任务
    uint8_t num_bomb_tasks = 0;                                          // 有效炸弹任务/炸弹槽数量
    uint8_t b_macro_cost[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH][4];   // 炸弹宏动作乐观路径下界，255 表示不可达
    MacroCostCacheEntry macro_cost_cache[MACRO_COST_CACHE_SIZE] = {};
    uint8_t solid_mask[1 << MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};
    uint16_t current_threshold_iteration = 0;                            // 当前 IDA* 阈值轮次，供硬图后段策略启用
    SokobanHeuristicMode heuristic_mode = SokobanHeuristicMode::SEMANTIC; // 当前求解使用的启发权重组
    bool strict_cost_search = false;                                     // 当前是否处于真实步数修复搜索
    uint32_t search_node_budget = 0;                                     // 本轮搜索节点预算，0 表示不限制
    uint32_t search_node_count = 0;                                      // 本轮已展开节点数
    bool search_aborted = false;                                         // 节点预算耗尽时中止本轮搜索

    // --- Zobrist 随机表与路径环路检测 ---
    uint32_t ZOBRIST_SPECIFIC_BOX[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];   // 带语义编号的箱子哈希
    uint32_t ZOBRIST_PLAYER[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];                    // 玩家规范位置哈希
    uint32_t ZOBRIST_TARGET[MAX_BOXES];                                        // 未完成目标集合哈希
    uint32_t ZOBRIST_BOMB[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];           // 未爆炸炸弹位置哈希
    uint32_t ZOBRIST_BLOWN_MASK[1 << MAX_BOMBS];                               // 已爆炸集合哈希
    uint32_t path_hashes[MAX_PATH_LENGTH];                                     // 当前递归路径的规范状态哈希，用于环路剪枝

    // --- 启发式距离场与死锁缓存 ---
    int16_t t_dist[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];           // target_id -> cell 的反向推箱距离，-1 表示不可达
    uint8_t box_target_cost[MAX_BOXES][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];   // target_id -> cell 的单箱乐观总代价，255 表示不可达
    uint8_t relaxed_walk_dist[MAP_CELL_COUNT][MAP_CELL_COUNT];          // 静态地图上的压缩行走距离，255 表示不可达
    bool is_dead[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];                        // 静态死锁格缓存
    uint16_t target_semantic_mask[10] = {};
    uint16_t target_cell_mask[10][MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = {};

    int16_t b_dist[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];           // bomb_id -> cell 的反向推炸弹距离，-1 表示不可达
    uint8_t wall_clear_mask[MAP_MAX_HEIGHT][MAP_MAX_WIDTH];             // 每个墙格会被哪些炸弹任务清除的 bitmask
    SokobanProfile profile;                                             // 最近一次 solve 的性能统计
};

extern TTEntry TT[TT_SIZE]; // 全局置换表，求解开始时清空
extern Sokoban solver;      // 全局求解器实例，供现有规划流程复用
