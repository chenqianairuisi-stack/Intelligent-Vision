#include "Exploration.h"
#include "Strategy.h"
#include <cmath>
#include <cstring>
#include <algorithm>

__attribute__((section(".dtcm_data"))) Exploration patrol_planner;

// ============================================================================
// 参数面板：巡图启发式代价与观测配置
// ============================================================================
static constexpr uint16_t MIN_GRID_TIME_LOWER_BOUND = 2;    // DFS 剪枝下界：单个未观测实体至少还要多少基础代价 [越大剪枝越激进]
static constexpr int32_t  BONUS_FOR_BOMB = 8;               // 炸弹动作奖励
static constexpr uint16_t BOMB_ROUTE_COST_DIVISOR = 100;   // 推必炸炸弹本体路径的摊销系数
static constexpr uint16_t FIRST_BOMB_LOCALITY_WEIGHT = 8;   // 巡图炸弹排序：优先处理当前附近的炸弹
static constexpr uint16_t FINAL_NEAR_BOX_RADIUS = 4;        // 收尾位置靠近箱子的判定半径
static constexpr uint16_t FINAL_NO_BOX_PENALTY = 10;        // 收尾位置远离箱子的时间惩罚
static constexpr uint16_t COST_INFINITY = 65535;            // 不可达代价哨兵值
static constexpr int32_t SEARCH_COST_INFINITY = 1000000000; // DFS 有符号代价上界，允许炸弹奖励产生负代价
static constexpr int MAX_BOMB_APPROACH_OBS_BRANCHES = 4;    // 每个炸弹宏动作最多展开的顺路观测组合分支数

// 根据剩余实体数量动态计算网格时间下界，更激进地剪枝多实体状态空间
static constexpr int GRID_TIME_CACHE_SLOTS = 16;            // 小型 LRU 距离图缓存槽数，16 槽约 6KB

// 根据初始实体数量返回巡图 DFS 使用的乐观距离下界
static constexpr uint16_t dynamic_grid_time_lower_bound(int initial_entities) {
    if (initial_entities <= 6) return MIN_GRID_TIME_LOWER_BOUND;
    if (initial_entities <= 8) return MIN_GRID_TIME_LOWER_BOUND + 8;
    if (initial_entities <= 10) return MIN_GRID_TIME_LOWER_BOUND + 10;
    return MIN_GRID_TIME_LOWER_BOUND + 16;
}

namespace VisionConfig {
    constexpr bool ENABLE_FACE_TO_FACE = true;     // 正面贴近观测
    constexpr bool ENABLE_OPTIMAL_DIST = false;    // 远一格的最佳视距观测
    constexpr bool ENABLE_DIAGONAL     = false;    // 近斜角观测
    constexpr bool ENABLE_FAR_DIAGONAL = false;    // 远斜角观测
}

// ============================================================================
// 热点工作区
// ============================================================================

// 在忽略所有动态物体的软地图上生成时间代价路径
static bool build_soft_grid_path(const SokobanLevel& lvl, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    SokobanLevel soft = lvl;
    soft.box_count = 0;
    soft.bomb_count = 0;
    return PlanningCommon::get_grid_time_path(soft, start, end, out_path);
}

// 返回路径上遇到的第一个箱子编号
static int find_first_box_on_path(const SokobanLevel& lvl, const StaticArray<point, MAX_PATH_LENGTH>& path) {
    for (int i = 0; i < path.size(); ++i) {
        for (int b = 0; b < lvl.box_count; ++b) {
            if (lvl.boxes[b] == path[i]) return b;
        }
    }
    return -1;
}

// 判断指定格子是否被其他箱子或炸弹占用
static bool has_other_dynamic_entity(const SokobanLevel& lvl, point p, int ignored_box) {
    for (int b = 0; b < lvl.box_count; ++b) {
        if (b != ignored_box && lvl.boxes[b] == p) return true;
    }
    for (int b = 0; b < lvl.bomb_count; ++b) {
        if (lvl.bombs[b].x != -1 && lvl.bombs[b] == p) return true;
    }
    return false;
}


// ============================================================================
// 模块 1：地图加载与观测位姿预生成
// ============================================================================

// 缓存当前巡图地图快照
void Exploration::load_level(const SokobanLevel& level) {
    this->cached_level = level; 
}

/// \brief 构建当前地图的去重观测位姿集合
/// \param level 当前地图状态
/// \return 已按位置和朝向合并后的候选观测位姿
StaticArray<ViewPose, MAX_OBS_POINTS> Exploration::build_current_views(const SokobanLevel& level) {
    this->cached_level = level;
    total_entities = level.box_count + level.target_count;
    SokobanLevel maps[1];
    maps[0] = level;
    build_entity_views(maps, 0);

    StaticArray<ViewPose, MAX_OBS_POINTS> out;
    for (int e = 0; e < total_entities; ++e) {
        for (int i = 0; i < entity_views[e].size(); ++i) {
            const ViewPose& vp = entity_views[e][i];
            bool exists = false;
            for (int j = 0; j < out.size(); ++j) {
                if (out[j].pos == vp.pos && out[j].target_yaw == vp.target_yaw) {
                    out[j].mask[0] |= vp.mask[0];
                    if (vp.penalty[0] < out[j].penalty[0]) out[j].penalty[0] = vp.penalty[0];
                    exists = true;
                    break;
                }
            }
            if (!exists) out.push_back(vp);
        }
    }
    return out;
}


/// \brief 为所有箱子和目标点预生成候选观测位姿
/// \param multi_maps 炸弹执行前后形成的地图快照数组
/// \param B 炸弹阶段数量
///
/// \details
/// 每个候选位姿会记录在不同炸弹阶段下可观测到的实体掩码和惩罚
/// 后续巡图 DFS 只需查询 entity_views，即可快速枚举能看到某实体的位姿
void Exploration::build_entity_views(const SokobanLevel* multi_maps, int B) {
    for (int e = 0; e < total_entities; ++e) entity_views[e].clear();

    // 将地图坐标转换成全局实体编号：箱子在前，目标点在后
    auto get_entity_id = [&](point p, const SokobanLevel& lvl) -> int {
        for (int i = 0; i < lvl.box_count; ++i) 
            if (lvl.boxes[i] == p) return i;
        for (int i = 0; i < lvl.target_count; ++i) 
            if (lvl.targets[i] == p) return lvl.box_count + i;
        return -1;
    };

    for (int y = 0; y < MAP_MAX_HEIGHT; ++y) {
        for (int x = 0; x < MAP_MAX_WIDTH; ++x) {
            point p = {(int8_t)x, (int8_t)y};

            // 枚举车体 4 个朝向，生成该驻留点可能看到的实体集合
            for (int d = 0; d < 4; ++d) {
                point F = { (int8_t)-MOVE[d].x, (int8_t)-MOVE[d].y }; 
                
                // F 是从车位指向观测实体的前向向量，yaw 必须按车头方向计算
                point view_dir = F;
                float true_yaw = 0.0f;
                if (view_dir.x == 1 && view_dir.y == 0) true_yaw = 0.0f;        
                else if (view_dir.x == 0 && view_dir.y == 1) true_yaw = 90.0f;  
                else if (view_dir.x == -1 && view_dir.y == 0) true_yaw = 180.0f;
                else if (view_dir.x == 0 && view_dir.y == -1) true_yaw = 270.0f;

                point R = { (int8_t)-F.y, (int8_t)F.x }; 
                point L = { (int8_t)F.y, (int8_t)-F.x };

                struct ViewGrid { point p; uint16_t pen; bool needs_los; bool enabled; };
                ViewGrid v_pts[6] = {
                    { p + F,         1, false, VisionConfig::ENABLE_FACE_TO_FACE },
                    { p + F + F,     0, true,  VisionConfig::ENABLE_OPTIMAL_DIST },
                    { p + F + L,     4, false, VisionConfig::ENABLE_DIAGONAL     },
                    { p + F + R,     4, false, VisionConfig::ENABLE_DIAGONAL     },
                    { p + F + F + L, 5, true,  VisionConfig::ENABLE_FAR_DIAGONAL },
                    { p + F + F + R, 5, true,  VisionConfig::ENABLE_FAR_DIAGONAL } 
                };

                bool valid_any = false;
                uint32_t masks[MAX_BOMBS + 1] = {0};
                uint16_t pens[MAX_BOMBS + 1];
                for (int k = 0; k <= B; ++k) pens[k] = COST_INFINITY;

                // 在每个炸弹阶段的地图快照上评估这个观测位姿
                for (int k = 0; k <= B; ++k) {
                    const SokobanLevel& lvl = multi_maps[k];
                    
                    // 车不能停在墙、箱子、目标点或未引爆炸弹上
                    if (lvl.map[y][x] == 1) continue;
                    bool occupied = false;
                    for(int b=0; b<lvl.box_count; ++b) if(lvl.boxes[b] == p) occupied = true;
                    for(int b=0; b<lvl.bomb_count; ++b) if(lvl.bombs[b].x != -1 && lvl.bombs[b] == p) occupied = true;
                    if (occupied) continue;

                    // 视线遮挡判定：中间格被墙或实体挡住时，远视距观测失效
                    bool los_blocked = false;
                    point mid = p + F;
                    if (mid.x >= 0 && mid.x < MAP_MAX_WIDTH && mid.y >= 0 && mid.y < MAP_MAX_HEIGHT) {
                        if (lvl.map[mid.y][mid.x] == 1) los_blocked = true;
                        for(int b=0; b<lvl.box_count; ++b) if(lvl.boxes[b] == mid) los_blocked = true;
                        for(int b=0; b<lvl.bomb_count; ++b) if(lvl.bombs[b].x != -1 && lvl.bombs[b] == mid) los_blocked = true;
                    } else los_blocked = true; 

                    uint32_t mask = 0;
                    uint16_t total_penalty = 0;
                    uint8_t count = 0;

                    // 将候选视野投影到实体掩码上，得到本位姿能观测到的实体集合
                    for (int i = 0; i < 6; ++i) {
                        if (!v_pts[i].enabled) continue; 
                        if (v_pts[i].needs_los && los_blocked) continue;
                        
                        int eid = get_entity_id(v_pts[i].p, lvl);
                        if (eid != -1 && !(mask & (1UL << eid))) {
                            mask |= (1UL << eid);
                            count++;
                            uint16_t actual_penalty = v_pts[i].pen;
                            if (i == 0 && eid >= lvl.box_count) actual_penalty = 0; // 目标点正面观测不额外惩罚
                            total_penalty += actual_penalty;
                        }
                    }

                    if (mask > 0) {
                        masks[k] = mask;
                        pens[k] = total_penalty / count;
                        valid_any = true;
                    }
                }

                if (valid_any) {
                    ViewPose vp;
                    vp.pos = p; 
                    vp.target_yaw = true_yaw;
                    for (int k = 0; k <= B; ++k) { 
                        vp.mask[k] = masks[k]; 
                        vp.penalty[k] = pens[k]; 
                    }
                    
                    // 将该位姿挂到所有可见实体的候选列表中，后续 DFS 可 O(1) 查询
                    for (int e = 0; e < total_entities; ++e) {
                        bool can_see_e = false;
                        for (int k = 0; k <= B; ++k) {
                            if (masks[k] & (1UL << e)) { can_see_e = true; break; }
                        }
                        if (can_see_e && entity_views[e].size() < 40) {
                            entity_views[e].push_back(vp);
                        }
                    }
                }
            }
        }
    }
}



// 转发炸弹任务对地图的爆破效果
void Exploration::apply_macro_bomb_effect(SokobanLevel& lvl, const BombTask& task) const {
    PlanningCommon::apply_bomb_task_effect(lvl, task);
}


/// \brief 根据真实可执行代价重排炸弹任务顺序
/// \param initial_lvl 初始地图状态
/// \param start_pos 玩家起点
/// \param raw_tasks 策略层给出的原始炸弹任务序列
/// \return 重排后的炸弹任务序列
///
/// \details
/// 炸弹数量很小，因此直接 DFS 枚举排列
/// 若任务已经包含推箱让路子任务，则保守返回原序列，避免重排破坏依赖关系
StaticArray<BombTask, MAX_BOMBS> Exploration::optimize_bomb_timeline(
    const SokobanLevel& initial_lvl, point start_pos, const StaticArray<BombTask, MAX_BOMBS>& raw_tasks) 
{
    if (raw_tasks.size() == 0) return raw_tasks;
    for (int i = 0; i < raw_tasks.size(); ++i) {
        if (raw_tasks[i].box_pushes.size() > 0) return raw_tasks;
    }
    StaticArray<BombTask, MAX_BOMBS> best_seq = raw_tasks;
    uint32_t best_cost = 0xFFFFFFFF; 
    uint32_t best_first_step_cost = 0xFFFFFFFF;
    bool found_complete_sequence = false;
    bool used[MAX_BOMBS] = {false};
    StaticArray<BombTask, MAX_BOMBS> current_seq;

    // 暴力枚举炸弹执行顺序；MAX_BOMBS 很小，直接 DFS 更稳
    auto dfs_perm = [&](auto& self, const SokobanLevel& lvl, point current_pos, uint32_t cost) -> void {
        if (current_seq.size() == raw_tasks.size()) {
            StaticArray<point, MAX_PATH_LENGTH> first_path;
            uint32_t first_step_cost = 0xFFFFFFFF;
            if (PlanningCommon::get_bomb_push_path(initial_lvl, start_pos, current_seq[0], first_path)) {
                first_step_cost = PlanningCommon::path_time_cost(start_pos, first_path);
            }

            uint32_t score = cost + first_step_cost * FIRST_BOMB_LOCALITY_WEIGHT;
            uint32_t best_score = best_cost + best_first_step_cost * FIRST_BOMB_LOCALITY_WEIGHT;
            if (score < best_score || (score == best_score && first_step_cost < best_first_step_cost)) {
                best_cost = cost;
                best_first_step_cost = first_step_cost;
                best_seq = current_seq;
                found_complete_sequence = true;
            }
            return;
        }
        if (cost >= best_cost) return; // 当前排列已经不可能更优，剪枝

        for (int i = 0; i < raw_tasks.size(); ++i) {
            if (!used[i]) {
                StaticArray<point, MAX_PATH_LENGTH> dummy_path;
                // 只有物理上能推到目标墙的炸弹任务才参与排序
                if (PlanningCommon::get_bomb_push_path(lvl, current_pos, raw_tasks[i], dummy_path)) {
                    used[i] = true;
                    current_seq.push_back(raw_tasks[i]);

                    uint16_t dist = PlanningCommon::path_time_cost(current_pos, dummy_path);
                    point next_pos = dummy_path.empty() ? current_pos : dummy_path.back();

                    SokobanLevel next_lvl = lvl;
                    apply_macro_bomb_effect(next_lvl, raw_tasks[i]);
                    for (int b = 0; b < next_lvl.bomb_count; ++b) {
                        if (next_lvl.bombs[b].x != -1 && next_lvl.bombs[b] == raw_tasks[i].bomb_start) {
                            next_lvl.bombs[b] = {-1, -1}; break; 
                        }
                    }
                    point tw = raw_tasks[i].target_wall;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (tw.y+dy > 0 && tw.y+dy < MAP_MAX_HEIGHT-1 && tw.x+dx > 0 && tw.x+dx < MAP_MAX_WIDTH-1) {
                                next_lvl.map[tw.y+dy][tw.x+dx] = 0;
                            }
                        }
                    }

                    self(self, next_lvl, next_pos, cost + dist);
                    current_seq.pop_back();
                    used[i] = false;
                }
            }
        }
    };
    dfs_perm(dfs_perm, initial_lvl, start_pos, 0);
    if (found_complete_sequence) return best_seq;

    StaticArray<BombTask, MAX_BOMBS> executable_seq;
    SokobanLevel work = initial_lvl;
    point current_pos = start_pos;
    bool consumed[MAX_BOMBS] = {false};

    for (int picked = 0; picked < raw_tasks.size(); ++picked) {
        int best_idx = -1;
        uint16_t best_step_cost = COST_INFINITY;
        StaticArray<point, MAX_PATH_LENGTH> best_path;

        for (int i = 0; i < raw_tasks.size(); ++i) {
            if (consumed[i]) continue;

            StaticArray<point, MAX_PATH_LENGTH> path;
            if (!PlanningCommon::get_bomb_push_path(work, current_pos, raw_tasks[i], path)) continue;

            uint16_t step_cost = PlanningCommon::path_time_cost(current_pos, path);
            if (step_cost < best_step_cost) {
                best_step_cost = step_cost;
                best_idx = i;
                best_path = path;
            }
        }

        if (best_idx < 0) break;

        consumed[best_idx] = true;
        executable_seq.push_back(raw_tasks[best_idx]);
        current_pos = best_path.empty() ? current_pos : best_path.back();
        apply_macro_bomb_effect(work, raw_tasks[best_idx]);
    }

    return executable_seq;
}


// ============================================================================
// 模块 2：多阶段动态状态空间搜索（EDSS）
// ============================================================================

/// \brief 巡图 DFS 的全局边界上下文
struct BoundingContext {
    int32_t best_cost;                       // 当前找到的最低总代价
    StaticArray<MacroAction, 32> best_path;  // 当前最优动作序列
    StaticArray<MacroAction, 32> current_path; // DFS 正在展开的动作序列
    uint32_t ops_limit;                      // 搜索节点预算
    uint32_t ops_count;                      // 已展开节点数
};

/// \brief 未观测实体的最近可达观测估价
struct PatrolEntityEval {
    int id;              // 实体编号
    uint16_t min_cost;   // 当前阶段的最近观测代价
};

/// \brief 单个观测位姿的动态排序结果
struct PatrolDynamicEval {
    int vp_idx;             // entity_views 中的位姿下标
    uint16_t actual_cost;   // 移动加转向后的真实估计代价
    int32_t score;          // 排序分数，越小越优先
};

/// \brief 推炸弹途中插入观测的候选分支
struct PatrolApproachObsCandidate {
    ViewPose vp;                 // 插入的观测位姿
    uint32_t newly_seen;         // 本次观测新增实体掩码
    int32_t total_cost;          // 推炸弹路径和观测转向合并代价
    int32_t score;               // 排序分数，越小越优先
    int observe_after_push_count; // 在第几个推箱前置任务后插入观测
};

// 巡图 DFS 最大递归层数，用于适配 32 KiB 栈
static constexpr int PATROL_DFS_FRAME_LIMIT = 16;

/// \brief 巡图 DFS 单层复用工作区
struct PatrolDfsFrame {
    uint16_t dist_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 当前状态到所有格子的时间代价
    PatrolEntityEval ordered_entities[32];            // 按最近观测代价排序的实体
    PatrolDynamicEval top_poses_by_yaw[4];            // 每个朝向保留一个最佳位姿
    PatrolDynamicEval sorted_evals[4];                // 当前实体待展开位姿
    StaticArray<point, MAX_PATH_LENGTH> macro_path;   // 炸弹宏任务底层路径
    StaticArray<point, MAX_PATH_LENGTH> support_path; // 推箱前置任务路径
    StaticArray<point, MAX_PATH_LENGTH> prefix_path;  // 顺路观测前缀路径
    StaticArray<point, MAX_PATH_LENGTH> suffix_path;  // 顺路观测后缀路径
    StaticArray<PatrolApproachObsCandidate, MAX_BOMB_APPROACH_OBS_BRANCHES> approach_obs_candidates; // 顺路观测候选
    int support_prefix_len[9];                        // 每个前置推箱任务结束时的路径长度
};

/// \brief 巡图规划共享 scratch，放 OCRAM 避免栈溢出和 DTCM 压力
struct PatrolScratchWorkspace {
    uint16_t micro_tt_sig[8192];       // 小型置换表签名
    int32_t micro_tt_cost[8192];       // 小型置换表最低代价
    bool materialize_checked[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 子任务补全尝试标记
    bool materialize_ok[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH];      // 子任务补全成功标记
    BombTask materialize_cache[MAX_BOMBS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 补全后的炸弹任务缓存
    uint16_t grid_time_cache[GRID_TIME_CACHE_SLOTS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 时间距离图缓存
    uint8_t grid_time_cache_valid[GRID_TIME_CACHE_SLOTS]; // 缓存槽有效标记
    uint8_t grid_time_cache_stage[GRID_TIME_CACHE_SLOTS]; // 缓存对应炸弹阶段
    point grid_time_cache_pos[GRID_TIME_CACHE_SLOTS];     // 缓存对应起点
    uint16_t grid_time_cache_stamp[GRID_TIME_CACHE_SLOTS]; // LRU 时间戳
    uint16_t grid_time_cache_clock = 0;                   // LRU 逻辑时钟
    BoundingContext ctx;                                  // DFS 全局边界上下文
    StaticArray<MacroAction, 32> seed_path;               // 贪心上界路径
    uint16_t seed_dist_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 贪心阶段距离图
    StaticArray<point, MAX_PATH_LENGTH> seed_macro_path;  // 贪心阶段炸弹路径
    StaticArray<MacroAction, 32> fallback_path;           // DFS 失败后的兜底路径
    uint16_t fallback_dist_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 兜底阶段距离图
    StaticArray<point, MAX_PATH_LENGTH> fallback_macro_path; // 兜底炸弹路径
    StaticArray<point, MAX_PATH_LENGTH> fallback_soft_path;  // 兜底软路径
    StaticArray<point, MAX_PATH_LENGTH> fallback_push_path;  // 兜底推箱路径
    StaticArray<point, MAX_PATH_LENGTH> fallback_clear_path; // 兜底清障路径
    PatrolDfsFrame dfs_frames[PATROL_DFS_FRAME_LIMIT];       // 按递归深度复用的帧数组
};

// 巡图大工作区放 OCRAM，链接脚本必须将 .ocram_bss 放到非 DTCM 区域
static MCU_OCRAM_BSS PatrolScratchWorkspace patrol_ws;


/// \brief 搜索最优巡图动作序列
/// \param start_pos 巡图起点
/// \param raw_bomb_tasks 可插入的炸弹宏任务序列
/// \param start_yaw 起始朝向，负值表示忽略初始转向代价
/// \return 巡图动作序列，包含普通观测动作和推炸弹宏动作
///
/// \details
/// 算法使用多阶段动态状态空间搜索：
/// - 每个炸弹执行阶段对应一张地图快照
/// - 状态由当前位置、朝向、已观测实体 mask、炸弹阶段 k 组成
/// - 搜索过程中可选择观测动作，也可执行下一个炸弹宏动作切换到下一阶段
StaticArray<MacroAction, 32> Exploration::plan_optimal_patrol(
    point start_pos, const StaticArray<BombTask, MAX_BOMBS>& raw_bomb_tasks, float start_yaw, uint32_t start_mask) 
{
    total_entities = cached_level.box_count + cached_level.target_count;
    if (total_entities == 0) return StaticArray<MacroAction, 32>();

    int B = raw_bomb_tasks.size();
    static SokobanLevel multi_maps[MAX_BOMBS + 1];
    multi_maps[0] = cached_level; 

    // 先按可执行代价调整炸弹顺序，再构造每次爆破后的地图快照
    auto bomb_tasks = optimize_bomb_timeline(cached_level, start_pos, raw_bomb_tasks);
    B = bomb_tasks.size();
    for (int k = 0; k < B; ++k) {
        multi_maps[k + 1] = multi_maps[k];
        apply_macro_bomb_effect(multi_maps[k + 1], bomb_tasks[k]);
        point t_wall = bomb_tasks[k].target_wall;
        
        for (int b = 0; b < multi_maps[k+1].bomb_count; ++b) {
            if (multi_maps[k+1].bombs[b].x != -1 && multi_maps[k+1].bombs[b] == bomb_tasks[k].bomb_start) {
                multi_maps[k+1].bombs[b] = {-1, -1};
                break;
            }
        }
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int ny = t_wall.y + dy, nx = t_wall.x + dx;
                if (ny > 0 && ny < MAP_MAX_HEIGHT - 1 && nx > 0 && nx < MAP_MAX_WIDTH - 1) {
                    multi_maps[k + 1].map[ny][nx] = 0; 
                }
            }
        }
    }

    build_entity_views(multi_maps, B);

    // 麦轮底盘平移代价低，转向单独计入代价
    auto get_turn_cost = [](float yaw1, float yaw2) -> uint16_t {
        return PlanningCommon::yaw_turn_time_cost(yaw1, yaw2);
    };

    auto get_bomb_route_cost = [](point start_pos, const StaticArray<point, MAX_PATH_LENGTH>& path, const BombTask& task) -> uint32_t {
        int approach_len = path.size();
        for (int i = 0; i < path.size(); ++i) {
            if (path[i] == task.bomb_start) {
                approach_len = i;
                break;
            }
        }

        StaticArray<point, MAX_PATH_LENGTH> approach_path;
        StaticArray<point, MAX_PATH_LENGTH> push_path;
        for (int i = 0; i < approach_len; ++i) approach_path.push_back(path[i]);
        for (int i = approach_len; i < path.size(); ++i) push_path.push_back(path[i]);

        point push_start = approach_path.empty() ? start_pos : approach_path.back();
        uint32_t approach_cost = PlanningCommon::path_time_cost(start_pos, approach_path);
        uint32_t push_cost = PlanningCommon::path_time_cost(push_start, push_path);
        return approach_cost + push_cost / BOMB_ROUTE_COST_DIVISOR;
    };

    auto apply_bomb_reward = [](uint32_t raw_cost) -> int32_t {
        return static_cast<int32_t>(raw_cost) - BONUS_FOR_BOMB;
    };

    auto get_bomb_action_cost = [&](point start_pos, const StaticArray<point, MAX_PATH_LENGTH>& path, const BombTask& task) -> int32_t {
        return apply_bomb_reward(get_bomb_route_cost(start_pos, path, task));
    };

    auto bomb_approach_order = [](point pos, point start_pos, const StaticArray<point, MAX_PATH_LENGTH>& path, const BombTask& task) -> int {
        if (pos == start_pos) return 0;

        int bomb_move_order = path.size() + 1;
        for (int i = 0; i < path.size(); ++i) {
            if (path[i] == task.bomb_start) {
                bomb_move_order = i + 1;
                break;
            }
        }

        for (int i = 0; i < path.size() && i + 1 < bomb_move_order; ++i) {
            if (path[i] == pos) return i + 1;
        }
        return -1;
    };

    auto prefix_path_to_order = [](const StaticArray<point, MAX_PATH_LENGTH>& path, int order) {
        StaticArray<point, MAX_PATH_LENGTH> prefix;
        for (int i = 0; i < order && i < path.size(); ++i) prefix.push_back(path[i]);
        return prefix;
    };

    auto suffix_path_from_order = [](const StaticArray<point, MAX_PATH_LENGTH>& path, int order) {
        StaticArray<point, MAX_PATH_LENGTH> suffix;
        for (int i = order; i < path.size(); ++i) suffix.push_back(path[i]);
        return suffix;
    };

    auto find_box_id_at = [](const SokobanLevel& level, point box_pos) -> int {
        for (int b = 0; b < level.box_count; ++b) {
            if (level.boxes[b] == box_pos) return b;
        }
        return -1;
    };

    auto make_flat_bomb = [](const BombTask& task) {
        BombTask flat = task;
        flat.box_pushes.clear();
        return flat;
    };

    auto append_flat_bomb_actions = [&](StaticArray<MacroAction, 32>& out,
                                        const SokobanLevel& stage,
                                        point player,
                                        const BombTask& task,
                                        int observe_after_push_count,
                                        const MacroAction* inserted_observe) -> bool {
        int action_count = task.box_pushes.size() + 1 + (inserted_observe ? 1 : 0);
        if (out.size() + action_count > 32) return false;

        SokobanLevel work = stage;
        point work_player = player;

        auto append_observe_if_needed = [&](int push_count_done) -> bool {
            if (!inserted_observe || push_count_done != observe_after_push_count) return true;

            StaticArray<point, MAX_PATH_LENGTH> obs_path;
            if (!PlanningCommon::get_grid_time_path(work, work_player, inserted_observe->observe.view.pos, obs_path)) return false;
            out.push_back(*inserted_observe);
            work_player = inserted_observe->observe.view.pos;
            return true;
        };

        if (!append_observe_if_needed(0)) return false;

        for (int i = 0; i < task.box_pushes.size(); ++i) {
            const BoxPushTask& push = task.box_pushes[i];
            int box_id = find_box_id_at(work, push.box_start);
            if (box_id < 0) return false;

            StaticArray<point, MAX_PATH_LENGTH> path;
            point before_player = work_player;
            if (!PlanningCommon::append_box_push_path(work, work_player, push, path)) return false;

            MacroAction push_action = make_box_push_macro_action(
                push,
                static_cast<uint8_t>(box_id),
                PlanningCommon::path_time_cost(before_player, path)
            );
            out.push_back(push_action);

            if (!append_observe_if_needed(i + 1)) return false;
        }

        MacroAction bomb_action = make_bomb_push_macro_action(make_flat_bomb(task));
        out.push_back(bomb_action);
        return true;
    };

    auto pop_flat_bomb_actions = [](StaticArray<MacroAction, 32>& out, const BombTask& task, bool has_observe) {
        int count = task.box_pushes.size() + 1 + (has_observe ? 1 : 0);
        while (count-- > 0) out.pop_back();
    };

    auto validate_reference_plan = [&](const StaticArray<MacroAction, 32>& plan) -> bool {
        SokobanLevel work = cached_level;
        point work_player = start_pos;

        for (int i = 0; i < plan.size(); ++i) {
            const MacroAction& action = plan[i];
            StaticArray<point, MAX_PATH_LENGTH> path;

            if (action.kind == MacroActionKind::OBSERVE) {
                if (!PlanningCommon::get_grid_time_path(work, work_player, action.observe.view.pos, path)) return false;
                work_player = action.observe.view.pos;
                continue;
            }

            if (action.kind == MacroActionKind::PUSH_BOX) {
                if (!PlanningCommon::append_box_push_path(work, work_player, macro_box_task(action), path)) return false;
                continue;
            }

            if (action.kind == MacroActionKind::PUSH_BOMB) {
                // Reference 输出必须是扁平原子任务，炸弹动作不再暗藏推箱清障
                BombTask bomb = macro_bomb_task(action);
                if (!PlanningCommon::get_bomb_push_path(work, work_player, bomb, path)) return false;
                if (!path.empty()) work_player = path.back();
                PlanningCommon::apply_bomb_task_effect(work, bomb);
                continue;
            }
        }

        return true;
    };

    // 小型置换表：记录同一状态下已知的最低代价，避免 DFS 环路和重复展开
    PatrolScratchWorkspace& ws = patrol_ws;
    std::fill(ws.micro_tt_sig, ws.micro_tt_sig + 8192, static_cast<uint16_t>(0xFFFF));
    std::fill(ws.micro_tt_cost, ws.micro_tt_cost + 8192, SEARCH_COST_INFINITY);

    BoundingContext& ctx = ws.ctx;
    ctx.best_cost = SEARCH_COST_INFINITY;
    ctx.best_path.clear();
    ctx.current_path.clear();
    ctx.ops_limit = 200000;
    ctx.ops_count = 0;
    std::memset(ws.materialize_checked, 0, sizeof(ws.materialize_checked));
    std::memset(ws.materialize_ok, 0, sizeof(ws.materialize_ok));
    std::memset(ws.grid_time_cache_valid, 0, sizeof(ws.grid_time_cache_valid));
    ws.grid_time_cache_clock = 0;
    auto build_grid_time_map_cached = [&](int stage, point pos, uint16_t out[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
        ++ws.grid_time_cache_clock;

        for (int i = 0; i < GRID_TIME_CACHE_SLOTS; ++i) {
            if (ws.grid_time_cache_valid[i] &&
                ws.grid_time_cache_stage[i] == stage &&
                ws.grid_time_cache_pos[i] == pos) {
                ws.grid_time_cache_stamp[i] = ws.grid_time_cache_clock;
                std::memcpy(out, ws.grid_time_cache[i], sizeof(ws.grid_time_cache[i]));
                return;
            }
        }

        int slot = 0;
        for (int i = 0; i < GRID_TIME_CACHE_SLOTS; ++i) {
            if (!ws.grid_time_cache_valid[i]) {
                slot = i;
                break;
            }
            if (ws.grid_time_cache_stamp[i] < ws.grid_time_cache_stamp[slot]) slot = i;
        }

        PlanningCommon::build_grid_time_map(multi_maps[stage], pos, ws.grid_time_cache[slot]);
        ws.grid_time_cache_valid[slot] = 1;
        ws.grid_time_cache_stage[slot] = static_cast<uint8_t>(stage);
        ws.grid_time_cache_pos[slot] = pos;
        ws.grid_time_cache_stamp[slot] = ws.grid_time_cache_clock;
        std::memcpy(out, ws.grid_time_cache[slot], sizeof(ws.grid_time_cache[slot]));
    };

    int req_boxes = cached_level.box_count - 1;       
    int req_targets = cached_level.target_count - 1;  
    if (req_boxes < 0) req_boxes = 0;
    if (req_targets < 0) req_targets = 0;
    const uint16_t patrol_lower_bound = dynamic_grid_time_lower_bound(total_entities);
    auto mask_has_required_counts = [&](uint32_t semantic_mask) -> bool {
        int seen_boxes = 0;
        int seen_targets = 0;
        for (int b = 0; b < cached_level.box_count; ++b) {
            if (semantic_mask & (1UL << b)) ++seen_boxes;
        }
        for (int t = 0; t < cached_level.target_count; ++t) {
            if (semantic_mask & (1UL << (cached_level.box_count + t))) ++seen_targets;
        }
        return seen_boxes >= req_boxes && seen_targets >= req_targets;
    };

    auto seed_greedy_upper_bound = [&]() -> void {
        StaticArray<MacroAction, 32>& seed_path = ws.seed_path;
        seed_path.clear();
        uint32_t mask = start_mask;
        point curr_pos = start_pos;
        float curr_yaw = start_yaw;
        int k = 0;
        int32_t current_cost = 0;

        for (int guard = 0; guard < 32; ++guard) {
            int box_seen = 0;
            int target_seen = 0;
            for (int e = 0; e < total_entities; ++e) {
                if (mask & (1UL << e)) {
                    if (e < cached_level.box_count) ++box_seen;
                    else ++target_seen;
                }
            }

            if (mask_has_required_counts(mask)) {
                int32_t final_cost = current_cost;
                if (box_seen < cached_level.box_count || target_seen < cached_level.target_count) {
                    bool near_box = false;
                    for (int b = 0; b < multi_maps[k].box_count; ++b) {
                        int dx = std::abs(curr_pos.x - multi_maps[k].boxes[b].x);
                        int dy = std::abs(curr_pos.y - multi_maps[k].boxes[b].y);
                        if (dx + dy <= FINAL_NEAR_BOX_RADIUS) { near_box = true; break; }
                    }
                    if (!near_box) final_cost += FINAL_NO_BOX_PENALTY;
                }
                if (final_cost < ctx.best_cost && validate_reference_plan(seed_path)) {
                    ctx.best_cost = final_cost;
                    ctx.best_path = seed_path;
                }
                return;
            }

            int remain_b = req_boxes - box_seen; if (remain_b < 0) remain_b = 0;
            int remain_t = req_targets - target_seen; if (remain_t < 0) remain_t = 0;
            auto newly_covers_needed_category = [&](uint32_t newly_seen) -> bool {
                if (remain_b > 0) {
                    for (int b = 0; b < cached_level.box_count; ++b) {
                        if (newly_seen & (1UL << b)) return true;
                    }
                }
                if (remain_t > 0) {
                    for (int t = 0; t < cached_level.target_count; ++t) {
                        if (newly_seen & (1UL << (cached_level.box_count + t))) return true;
                    }
                }
                return false;
            };

            uint16_t (&dist_map)[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = ws.seed_dist_map;
            build_grid_time_map_cached(k, curr_pos, dist_map);

            int best_entity = -1;
            int best_view = -1;
            int best_score = 0x7FFFFFFF;
            int best_pop = 0;
            int reachable_remaining_boxes = 0;
            int reachable_remaining_targets = 0;

            for (int e = 0; e < total_entities; ++e) {
                if (mask & (1UL << e)) continue;
                bool entity_reachable = false;
                for (int i = 0; i < entity_views[e].size(); ++i) {
                    const ViewPose& vp = entity_views[e][i];
                    uint32_t newly_seen = vp.mask[k] & ~mask;
                    if (newly_seen == 0) continue;
                    if (!newly_covers_needed_category(newly_seen)) continue;

                    uint16_t dist = dist_map[vp.pos.y][vp.pos.x];
                    if (dist == 0xFFFF) continue;
                    entity_reachable = true;

                    int pop = 0;
                    for (int bit = 0; bit < 20; ++bit) {
                        if (newly_seen & (1UL << bit)) ++pop;
                    }
                    int score = dist + get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k] * 4 - pop * 12;
                    if (score < best_score || (score == best_score && pop > best_pop)) {
                        best_score = score;
                        best_entity = e;
                        best_view = i;
                        best_pop = pop;
                    }
                }
                if (entity_reachable) {
                    if (e < cached_level.box_count) ++reachable_remaining_boxes;
                    else ++reachable_remaining_targets;
                }
            }

            if (best_entity >= 0) {
                const ViewPose& vp = entity_views[best_entity][best_view];
                uint32_t newly_seen = vp.mask[k] & ~mask;
                uint16_t dist = dist_map[vp.pos.y][vp.pos.x];
                MacroAction act = make_observe_macro_action(vp, vp.mask[k]);
                seed_path.push_back(act);
                current_cost += dist + get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k];
                curr_pos = vp.pos;
                curr_yaw = vp.target_yaw;
                mask |= newly_seen;
                continue;
            }

            bool needs_stage_unlock = (reachable_remaining_boxes < remain_b) || (reachable_remaining_targets < remain_t);
            if (k < B && needs_stage_unlock) {
                StaticArray<point, MAX_PATH_LENGTH>& macro_path = ws.seed_macro_path;
                macro_path.clear();
                BombTask executable_task = bomb_tasks[k];
                if (!PlanningCommon::get_bomb_push_path(multi_maps[k], curr_pos, executable_task, macro_path)) return;
                if (!append_flat_bomb_actions(seed_path, multi_maps[k], curr_pos, executable_task, -1, nullptr)) return;
                current_cost += get_bomb_action_cost(curr_pos, macro_path, executable_task);
                curr_pos = macro_path.empty() ? curr_pos : macro_path.back();
                ++k;
                continue;
            }

            return;
        }
    };
    seed_greedy_upper_bound();

    auto dfs = [&](auto& self, point curr_pos, float curr_yaw, int k, uint32_t mask, int32_t current_cost, int depth) -> void {
        if (depth >= PATROL_DFS_FRAME_LIMIT) return;
        PatrolDfsFrame& frame = ws.dfs_frames[depth];
        if (ctx.ops_count++ > ctx.ops_limit) return;
        int32_t remaining_bomb_credit = static_cast<int32_t>(B - k) * BONUS_FOR_BOMB;
        if (current_cost - remaining_bomb_credit >= ctx.best_cost) return;

        // 状态压缩：观测掩码 + 位置 + 朝向 + 炸弹阶段
        int yaw_idx = 0;
        if (curr_yaw >= 0.0f) {
            int int_yaw = (int)(curr_yaw + 0.5f) % 360;
            if (int_yaw < 0) int_yaw += 360;
            yaw_idx = (int_yaw / 90) % 4;
        }

        uint64_t safe_state = ((uint64_t)mask << 24) | ((uint64_t)curr_pos.y << 16) | ((uint64_t)curr_pos.x << 8) | (yaw_idx << 4) | k;
        uint32_t hash = (safe_state ^ (safe_state >> 32)) * 2654435761U; 
        uint16_t tt_idx = hash & 8191;       
        uint16_t tt_sig = hash >> 16;        
        if (ws.micro_tt_sig[tt_idx] == tt_sig && ws.micro_tt_cost[tt_idx] <= current_cost) return;
        ws.micro_tt_sig[tt_idx] = tt_sig;
        ws.micro_tt_cost[tt_idx] = current_cost;

        // 统计当前已经观测到的箱子和目标点数量
        int box_seen = 0, target_seen = 0;
        for (int e = 0; e < total_entities; ++e) {
            if (mask & (1UL << e)) {
                if (e < cached_level.box_count) box_seen++;
                else target_seen++;
            }
        }

        int remain_b = req_boxes - box_seen; if(remain_b < 0) remain_b = 0;
        int remain_t = req_targets - target_seen; if(remain_t < 0) remain_t = 0;
        int remain_total = remain_b + remain_t;
        auto newly_covers_needed_category = [&](uint32_t newly_seen) -> bool {
            if (remain_b > 0) {
                for (int b = 0; b < cached_level.box_count; ++b) {
                    if (newly_seen & (1UL << b)) return true;
                }
            }
            if (remain_t > 0) {
                for (int t = 0; t < cached_level.target_count; ++t) {
                    if (newly_seen & (1UL << (cached_level.box_count + t))) return true;
                }
            }
            return false;
        };
        if (remain_total > 0) {
            // 每个未覆盖实体至少需要一次平移代价，用作乐观下界剪枝
            if (current_cost + remain_total * patrol_lower_bound - remaining_bomb_credit >= ctx.best_cost) return;
        }

        // N-1 观测目标达成后即可收尾，避免为最后一个实体消耗过多时间
        if (mask_has_required_counts(mask)) {
            int32_t final_cost = current_cost;
            if (box_seen < cached_level.box_count || target_seen < cached_level.target_count) {
                bool near_box = false;
                for (int b = 0; b < multi_maps[k].box_count; ++b) {
                    int dx = std::abs(curr_pos.x - multi_maps[k].boxes[b].x);
                    int dy = std::abs(curr_pos.y - multi_maps[k].boxes[b].y);
                    if (dx + dy <= FINAL_NEAR_BOX_RADIUS) { near_box = true; break; }
                }
                if (!near_box) final_cost += FINAL_NO_BOX_PENALTY;
            }
            if (final_cost < ctx.best_cost) { ctx.best_cost = final_cost; ctx.best_path = ctx.current_path; }
            return;
        }

        uint16_t (&dist_map)[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = frame.dist_map;
        build_grid_time_map_cached(k, curr_pos, dist_map);

        int unvisited_count = 0;

        // 为每个未观测实体估计最近可达观测位姿，用于排序展开
        for (int e = 0; e < total_entities; ++e) {
            if (mask & (1UL << e)) continue; 
            uint16_t best_cost_for_e = COST_INFINITY;
            
            for (int i = 0; i < entity_views[e].size(); ++i) {
                const auto& vp = entity_views[e][i];
                uint32_t newly_seen = vp.mask[k] & ~mask;
                if (newly_seen == 0) continue;
                if (!newly_covers_needed_category(newly_seen)) continue;

                uint16_t dist = dist_map[vp.pos.y][vp.pos.x]; 
                if (dist == 0xFFFF) continue;

                uint16_t cost = dist + get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k];
                if (cost < best_cost_for_e) { best_cost_for_e = cost; }
            }
            frame.ordered_entities[unvisited_count++] = {e, best_cost_for_e};
        }

        for (int i = 0; i < unvisited_count - 1; ++i) {
            for (int j = 0; j < unvisited_count - 1 - i; ++j) {
                if (frame.ordered_entities[j].min_cost > frame.ordered_entities[j+1].min_cost) {
                    auto temp = frame.ordered_entities[j];
                    frame.ordered_entities[j] = frame.ordered_entities[j+1];
                    frame.ordered_entities[j+1] = temp;
                }
            }
        }
        int reachable_remaining_boxes = 0;
        int reachable_remaining_targets = 0;
        for (int i = 0; i < unvisited_count; ++i) {
            if (frame.ordered_entities[i].min_cost != COST_INFINITY) {
                if (frame.ordered_entities[i].id < cached_level.box_count) ++reachable_remaining_boxes;
                else ++reachable_remaining_targets;
            }
        }
        bool needs_stage_unlock = (reachable_remaining_boxes < remain_b) || (reachable_remaining_targets < remain_t);

        // 优先展开代价低、一次能覆盖更多实体的观测动作
        for (int idx = 0; idx < unvisited_count; ++idx) {
            int e = frame.ordered_entities[idx].id;
            if (frame.ordered_entities[idx].min_cost == COST_INFINITY) continue; 

            for(int i=0; i<4; i++) frame.top_poses_by_yaw[i] = {-1, COST_INFINITY, 0x7FFFFFFF};

            for (int i = 0; i < entity_views[e].size(); ++i) {
                const auto& vp = entity_views[e][i];
                uint32_t newly_seen = vp.mask[k] & ~mask;
                if (newly_seen == 0) continue; 
                if (!newly_covers_needed_category(newly_seen)) continue;

                uint16_t dist = dist_map[vp.pos.y][vp.pos.x]; 
                if (dist == 0xFFFF) continue;

                uint16_t turn_cost = get_turn_cost(curr_yaw, vp.target_yaw);
                uint16_t total_cost = dist + turn_cost + vp.penalty[k];

                int pop = 0;
                for(int bit = 0; bit < 16; ++bit) { if(newly_seen & (1UL << bit)) pop++; }
                
                // 低代价优先，同时略微奖励“一次看到多个实体”的观测点
                int32_t score = (total_cost * 10) - pop;

                int vp_yaw_idx = 0;
                int int_yaw = (int)(vp.target_yaw + 0.5f) % 360;
                if (int_yaw < 0) int_yaw += 360;
                vp_yaw_idx = (int_yaw / 90) % 4;

                if (score < frame.top_poses_by_yaw[vp_yaw_idx].score) {
                    frame.top_poses_by_yaw[vp_yaw_idx] = {i, total_cost, score};
                }
            }

            int valid_eval_count = 0;
            for (int i = 0; i < 4; ++i) {
                if (frame.top_poses_by_yaw[i].vp_idx != -1) {
                    frame.sorted_evals[valid_eval_count++] = frame.top_poses_by_yaw[i];
                }
            }

            for (int i = 0; i < valid_eval_count - 1; ++i) {
                for (int j = 0; j < valid_eval_count - 1 - i; ++j) {
                    if (frame.sorted_evals[j].score > frame.sorted_evals[j+1].score) {
                        auto temp = frame.sorted_evals[j];
                        frame.sorted_evals[j] = frame.sorted_evals[j+1];
                        frame.sorted_evals[j+1] = temp;
                    }
                }
            }

            for (int i = 0; i < valid_eval_count; ++i) {
                const auto& best_vp = entity_views[e][frame.sorted_evals[i].vp_idx];
                MacroAction act = make_observe_macro_action(best_vp, best_vp.mask[k]);
                
                uint32_t next_mask = mask | best_vp.mask[k];

                ctx.current_path.push_back(act);
                self(self, best_vp.pos, best_vp.target_yaw, k, next_mask, current_cost + frame.sorted_evals[i].actual_cost, depth + 1);
                ctx.current_path.pop_back();
            }
        }

        // 如果还有炸弹任务，可以在当前阶段插入推炸弹宏动作，跳转到下一张地图快照
        if (k < B) {
            StaticArray<point, MAX_PATH_LENGTH>& macro_path = frame.macro_path;
            macro_path.clear();
            BombTask executable_task = bomb_tasks[k];
            bool can_execute_bomb = PlanningCommon::get_bomb_push_path(multi_maps[k], curr_pos, executable_task, macro_path);
            if (!can_execute_bomb && executable_task.box_pushes.size() == 0) {
                // 当前阶段可达观测不足以完成目标时，尝试补全“推箱让路”子任务
                if (!ws.materialize_checked[k][curr_pos.y][curr_pos.x]) {
                    ws.materialize_checked[k][curr_pos.y][curr_pos.x] = true;
                    BombTask materialized_task;
                    if (strategic_planner.materialize_bomb_task(multi_maps[k], curr_pos, executable_task, materialized_task)) {
                        ws.materialize_ok[k][curr_pos.y][curr_pos.x] = true;
                        ws.materialize_cache[k][curr_pos.y][curr_pos.x] = materialized_task;
                    }
                }
                if (ws.materialize_ok[k][curr_pos.y][curr_pos.x]) {
                    executable_task = ws.materialize_cache[k][curr_pos.y][curr_pos.x];
                    can_execute_bomb = PlanningCommon::get_bomb_push_path(multi_maps[k], curr_pos, executable_task, macro_path);
                }
            }
            if (can_execute_bomb) {
                int* support_prefix_len = frame.support_prefix_len;
                support_prefix_len[0] = 0;
                bool support_prefix_ok = true;
                {
                    SokobanLevel support_stage = multi_maps[k];
                    point support_player = curr_pos;
                    StaticArray<point, MAX_PATH_LENGTH>& support_path = frame.support_path;
                    support_path.clear();
                    for (int i = 0; i < executable_task.box_pushes.size(); ++i) {
                        if (!PlanningCommon::append_box_push_path(support_stage, support_player, executable_task.box_pushes[i], support_path)) {
                            support_prefix_ok = false;
                            break;
                        }
                        support_prefix_len[i + 1] = support_path.size();
                    }
                }

                auto observe_push_prefix_count = [&](int path_order) -> int {
                    if (!support_prefix_ok) return -1;
                    int support_count = executable_task.box_pushes.size();
                    if (path_order >= support_prefix_len[support_count]) return support_count;

                    // 暂时只在完整清障推箱任务边界插入观测，不拆半个推箱动作
                    for (int i = 0; i <= support_count; ++i) {
                        if (path_order == support_prefix_len[i]) return i;
                    }
                    return -1;
                };

                auto& approach_obs_candidates = frame.approach_obs_candidates;
                approach_obs_candidates.clear();

                // 推炸弹是一个宏动作，但它的接近路径上可能正好经过未执行的观测点
                // 这里只保留去重后的少量高收益候选，避免 DFS 反复展开同一类组合分支
                for (int e = 0; e < total_entities; ++e) {
                    if (mask & (1UL << e)) continue;
                    for (int i = 0; i < entity_views[e].size(); ++i) {
                        const ViewPose& vp = entity_views[e][i];
                        uint32_t newly_seen = vp.mask[k] & ~mask;
                        if (newly_seen == 0) continue;
                        if (!newly_covers_needed_category(newly_seen)) continue;

                        int order = bomb_approach_order(vp.pos, curr_pos, macro_path, executable_task);
                        if (order < 0) continue;
                        int observe_after_push_count = observe_push_prefix_count(order);
                        if (observe_after_push_count < 0) continue;

                        StaticArray<point, MAX_PATH_LENGTH>& prefix = frame.prefix_path;
                        StaticArray<point, MAX_PATH_LENGTH>& suffix = frame.suffix_path;
                        prefix.clear();
                        suffix.clear();
                        for (int pi = 0; pi < order && pi < macro_path.size(); ++pi) prefix.push_back(macro_path[pi]);
                        for (int si = order; si < macro_path.size(); ++si) suffix.push_back(macro_path[si]);
                        uint32_t prefix_cost = PlanningCommon::path_time_cost(curr_pos, prefix);
                        uint32_t observe_turn_cost = get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k];
                        uint32_t combined_bomb_route_cost =
                            prefix_cost + get_bomb_route_cost(vp.pos, suffix, executable_task);
                        int32_t combined_cost =
                            apply_bomb_reward(combined_bomb_route_cost) + static_cast<int32_t>(observe_turn_cost);

                        bool duplicated = false;
                        for (int c = 0; c < approach_obs_candidates.size(); ++c) {
                            const PatrolApproachObsCandidate& old = approach_obs_candidates[c];
                            if (old.vp.pos == vp.pos &&
                                old.vp.target_yaw == vp.target_yaw &&
                                old.newly_seen == newly_seen) {
                                duplicated = true;
                                break;
                            }
                        }
                        if (duplicated) continue;

                        int pop = 0;
                        for (int bit = 0; bit < 20; ++bit) {
                            if (newly_seen & (1UL << bit)) ++pop;
                        }
                        int32_t score = static_cast<int32_t>(combined_cost * 10) - pop;
                        PatrolApproachObsCandidate candidate{
                            vp,
                            newly_seen,
                            combined_cost,
                            score,
                            observe_after_push_count
                        };

                        int insert_pos = approach_obs_candidates.size();
                        for (int c = 0; c < approach_obs_candidates.size(); ++c) {
                            if (candidate.score < approach_obs_candidates[c].score) {
                                insert_pos = c;
                                break;
                            }
                        }
                        if (insert_pos >= MAX_BOMB_APPROACH_OBS_BRANCHES) continue;

                        if (approach_obs_candidates.size() < MAX_BOMB_APPROACH_OBS_BRANCHES) {
                            approach_obs_candidates.push_back(candidate);
                        }
                        for (int c = approach_obs_candidates.size() - 1; c > insert_pos; --c) {
                            approach_obs_candidates[c] = approach_obs_candidates[c - 1];
                        }
                        approach_obs_candidates[insert_pos] = candidate;
                    }
                }

                for (int c = 0; c < approach_obs_candidates.size(); ++c) {
                    const PatrolApproachObsCandidate& candidate = approach_obs_candidates[c];

                    MacroAction obs_act = make_observe_macro_action(candidate.vp, candidate.vp.mask[k]);

                    if (!append_flat_bomb_actions(ctx.current_path,
                                                  multi_maps[k],
                                                  curr_pos,
                                                  executable_task,
                                                  candidate.observe_after_push_count,
                                                  &obs_act)) {
                        continue;
                    }

                    point next_pos_after_obs = macro_path.empty() ? candidate.vp.pos : macro_path.back();
                    // Movement and push/bomb macros do not reset observe yaw in execution
                    // Keep DFS yaw consistent so the next observation pays the real turn cost
                    self(self,
                         next_pos_after_obs,
                         candidate.vp.target_yaw,
                         k + 1,
                         mask | candidate.newly_seen,
                         current_cost + candidate.total_cost,
                         depth + 1);

                    pop_flat_bomb_actions(ctx.current_path, executable_task, true);
                }

                if (!append_flat_bomb_actions(ctx.current_path,
                                              multi_maps[k],
                                              curr_pos,
                                              executable_task,
                                              -1,
                                              nullptr)) {
                    return;
                }

                point next_pos = macro_path.empty() ? curr_pos : macro_path.back();
                int32_t bomb_cost = get_bomb_action_cost(curr_pos, macro_path, executable_task);
                // No observation happened inside this macro branch, so carry the previous observe yaw
                self(self, next_pos, curr_yaw, k + 1, mask, current_cost + bomb_cost, depth + 1);
                pop_flat_bomb_actions(ctx.current_path, executable_task, false);
            }
        }
    };

    dfs(dfs, start_pos, start_yaw, 0, start_mask, 0, 0);

    if (!ctx.best_path.empty() && validate_reference_plan(ctx.best_path)) return ctx.best_path;

    // DFS 未找到完整巡图序列时，使用有限贪心兜底
    // 兜底逻辑优先做可达观测，若无观测可走则尝试执行下一颗炸弹打开地图
    StaticArray<MacroAction, 32>& fallback_path = ws.fallback_path;
    fallback_path.clear();
    uint32_t mask = start_mask;
    point curr_pos = start_pos;
    float curr_yaw = start_yaw;
    int k = 0;

    for (int guard = 0; guard < 32; ++guard) {
        const SokobanLevel& stage_lvl = multi_maps[k];
        int box_seen = 0;
        int target_seen = 0;
        for (int e = 0; e < total_entities; ++e) {
            if (mask & (1UL << e)) {
                if (e < stage_lvl.box_count) ++box_seen;
                else ++target_seen;
            }
        }
        if (mask_has_required_counts(mask)) break;

        int remain_b = req_boxes - box_seen;
        int remain_t = req_targets - target_seen;
        if (remain_b < 0) remain_b = 0;
        if (remain_t < 0) remain_t = 0;
        auto newly_covers_needed_category = [&](uint32_t newly_seen) -> bool {
            if (remain_b > 0) {
                for (int b = 0; b < stage_lvl.box_count; ++b) {
                    if (newly_seen & (1UL << b)) return true;
                }
            }
            if (remain_t > 0) {
                for (int t = 0; t < stage_lvl.target_count; ++t) {
                    if (newly_seen & (1UL << (stage_lvl.box_count + t))) return true;
                }
            }
            return false;
        };

        uint16_t (&dist_map)[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] = ws.fallback_dist_map;
        PlanningCommon::build_grid_time_map(stage_lvl, curr_pos, dist_map);

        int best_entity = -1;
        int best_view = -1;
        int best_score = 0x7FFFFFFF;
        int best_newly_seen = 0;
        int reachable_remaining_boxes = 0;
        int reachable_remaining_targets = 0;
        for (int e = 0; e < total_entities; ++e) {
            if (mask & (1UL << e)) continue;
            bool entity_reachable = false;
            for (int i = 0; i < entity_views[e].size(); ++i) {
                const ViewPose& vp = entity_views[e][i];
                uint32_t newly_seen = vp.mask[k] & ~mask;
                if (newly_seen == 0) continue;
                if (!newly_covers_needed_category(newly_seen)) continue;
                uint16_t dist = dist_map[vp.pos.y][vp.pos.x];
                if (dist == 0xFFFF) continue;
                entity_reachable = true;

                int pop = 0;
                for (int bit = 0; bit < 20; ++bit) {
                    if (newly_seen & (1UL << bit)) ++pop;
                }
                int score = dist + get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k] * 4 - pop * 12;
                if (score < best_score || (score == best_score && pop > best_newly_seen)) {
                    best_score = score;
                    best_entity = e;
                    best_view = i;
                    best_newly_seen = pop;
                }
            }
            if (entity_reachable) {
                if (e < stage_lvl.box_count) ++reachable_remaining_boxes;
                else ++reachable_remaining_targets;
            }
        }

        bool needs_stage_unlock = (reachable_remaining_boxes < remain_b) || (reachable_remaining_targets < remain_t);

        if (k < B && needs_stage_unlock) {
            StaticArray<point, MAX_PATH_LENGTH>& macro_path = ws.fallback_macro_path;
            macro_path.clear();
            BombTask executable_task = bomb_tasks[k];
            bool can_execute_bomb = PlanningCommon::get_bomb_push_path(stage_lvl, curr_pos, executable_task, macro_path);
            if (!can_execute_bomb && executable_task.box_pushes.size() == 0) {
                BombTask materialized_task;
                if (strategic_planner.materialize_bomb_task(stage_lvl, curr_pos, executable_task, materialized_task)) {
                    executable_task = materialized_task;
                    can_execute_bomb = PlanningCommon::get_bomb_push_path(stage_lvl, curr_pos, executable_task, macro_path);
                }
            }
            if (can_execute_bomb) {
                if (!append_flat_bomb_actions(fallback_path, stage_lvl, curr_pos, executable_task, -1, nullptr)) break;
                curr_pos = macro_path.empty() ? curr_pos : macro_path.back();
                ++k;
                continue;
            }
        }

        // 这里如果继续硬凑观测，容易把巡图拖成多余观测序列；
        // 先尝试把挡路箱子软推开，再交给后续轮次重新评估
        if (needs_stage_unlock) {
            struct SoftClearCandidate {
                MacroAction action;
                int score = 0x7FFFFFFF;
                bool valid = false;
            };
            SoftClearCandidate best_clear;
            uint16_t full_target_mask = 0;
            for (int t = 0; t < stage_lvl.target_count; ++t) full_target_mask |= (1U << t);

            for (int e = 0; e < total_entities; ++e) {
                if (mask & (1UL << e)) continue;

                bool entity_reachable = false;
                for (int i = 0; i < entity_views[e].size(); ++i) {
                    const ViewPose& vp = entity_views[e][i];
                    uint32_t newly_seen = vp.mask[k] & ~mask;
                    if (newly_seen == 0) continue;
                    if (!newly_covers_needed_category(newly_seen)) continue;
                    uint16_t dist = dist_map[vp.pos.y][vp.pos.x];
                    if (dist != 0xFFFF) {
                        entity_reachable = true;
                        break;
                    }
                }
                if (entity_reachable) continue;

                for (int i = 0; i < entity_views[e].size(); ++i) {
                    const ViewPose& vp = entity_views[e][i];
                    uint32_t newly_seen = vp.mask[k] & ~mask;
                    if (newly_seen == 0) continue;
                    if (!newly_covers_needed_category(newly_seen)) continue;
                    if (dist_map[vp.pos.y][vp.pos.x] != 0xFFFF) continue;

                    StaticArray<point, MAX_PATH_LENGTH>& soft_path = ws.fallback_soft_path;
                    soft_path.clear();
                    if (!build_soft_grid_path(stage_lvl, curr_pos, vp.pos, soft_path)) continue;
                    int blocking_box = find_first_box_on_path(stage_lvl, soft_path);
                    if (blocking_box < 0) continue;

                    int pop = 0;
                    for (int bit = 0; bit < 20; ++bit) {
                        if (newly_seen & (1UL << bit)) ++pop;
                    }

                    for (int d = 0; d < 4; ++d) {
                        for (int step = 1; step <= 2; ++step) {
                            point target = {
                                static_cast<int8_t>(stage_lvl.boxes[blocking_box].x + MOVE[d].x * step),
                                static_cast<int8_t>(stage_lvl.boxes[blocking_box].y + MOVE[d].y * step)
                            };
                            if (!PlanningCommon::in_bounds(target) || stage_lvl.map[target.y][target.x] == 1) continue;
                            if (has_other_dynamic_entity(stage_lvl, target, blocking_box)) continue;

                            BoxPushTask task{stage_lvl.boxes[blocking_box], target};
                            StaticArray<point, MAX_PATH_LENGTH>& push_path = ws.fallback_push_path;
                            push_path.clear();
                            SokobanLevel probe = stage_lvl;
                            point probe_player = curr_pos;
                            if (!PlanningCommon::append_box_push_path(probe, probe_player, task, push_path)) continue;
                            if (!PlanningCommon::is_box_position_safe(probe, static_cast<uint8_t>(blocking_box), full_target_mask)) continue;

                            uint16_t after_direct = PlanningCommon::shortest_grid_time_cost(probe, probe_player, vp.pos);
                            if (after_direct == 65535) continue;

                            uint16_t push_cost = PlanningCommon::path_time_cost(curr_pos, push_path);
                            int score = static_cast<int>(push_cost) +
                                        static_cast<int>(after_direct) / 2 +
                                        vp.penalty[k] * 3 -
                                        pop * 20;

                            if (!best_clear.valid || score < best_clear.score) {
                                best_clear.valid = true;
                                best_clear.score = score;
                                best_clear.action = make_box_push_macro_action(
                                    task,
                                    static_cast<uint8_t>(blocking_box),
                                    push_cost
                                );
                            }
                        }
                    }
                }
            }

            if (best_clear.valid) {
                StaticArray<point, MAX_PATH_LENGTH>& clear_path = ws.fallback_clear_path;
                clear_path.clear();
                SokobanLevel probe = multi_maps[k];
                point probe_player = curr_pos;
                if (PlanningCommon::append_box_push_path(probe, probe_player, macro_box_task(best_clear.action), clear_path)) {
                    fallback_path.push_back(best_clear.action);
                    curr_pos = probe_player;

                    for (int s = k; s <= B; ++s) {
                        if (best_clear.action.box_push.box_id < multi_maps[s].box_count) {
                            multi_maps[s].boxes[best_clear.action.box_push.box_id] = best_clear.action.box_push.box_target;
                        }
                    }
                    build_entity_views(multi_maps, B);
                    continue;
                }
            }
        }

        if (best_entity != -1) {
            const ViewPose& vp = entity_views[best_entity][best_view];
            MacroAction act = make_observe_macro_action(vp, vp.mask[k]);
            fallback_path.push_back(act);
            mask |= vp.mask[k];
            curr_pos = vp.pos;
            curr_yaw = vp.target_yaw;
            continue;
        }

        if (k >= B || !needs_stage_unlock) break;

        StaticArray<point, MAX_PATH_LENGTH>& macro_path = ws.fallback_macro_path;
        macro_path.clear();
        BombTask executable_task = bomb_tasks[k];
        bool can_execute_bomb = PlanningCommon::get_bomb_push_path(multi_maps[k], curr_pos, executable_task, macro_path);
        if (!can_execute_bomb && executable_task.box_pushes.size() == 0) {
            BombTask materialized_task;
            if (strategic_planner.materialize_bomb_task(multi_maps[k], curr_pos, executable_task, materialized_task)) {
                executable_task = materialized_task;
                can_execute_bomb = PlanningCommon::get_bomb_push_path(multi_maps[k], curr_pos, executable_task, macro_path);
            }
        }
        if (!can_execute_bomb) break;

        if (!append_flat_bomb_actions(fallback_path, multi_maps[k], curr_pos, executable_task, -1, nullptr)) break;
        curr_pos = macro_path.empty() ? curr_pos : macro_path.back();
        ++k;
    }

    if (mask_has_required_counts(mask) && validate_reference_plan(fallback_path)) return fallback_path;
    return StaticArray<MacroAction, 32>();
}

