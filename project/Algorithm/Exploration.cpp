/// \file exploration.cpp
/// \brief 巡图候选观测位姿与实体可见性规划实现

#include "Exploration.h"
#include "Strategy.h"
#include <cmath>
#include <cstring>
#include <algorithm>

OCRAM_BSS Exploration patrol_planner;

// ============================================================================
// 参数面板：巡图启发式代价与观测配置
// ============================================================================

namespace ExplorationConfig {
    // ------------------------------------------------------------------------
    // 巡图 DFS 代价参数
    // ------------------------------------------------------------------------
    inline constexpr uint16_t OBSERVE_ACTION_COST = 32;         // 一次观测约等于跨图移动，优先合并可同时识别的实体
    inline constexpr int32_t BONUS_FOR_BOMB = 8;                // 完成必推炸弹的固定收益，避免把解锁动作无限推迟
    inline constexpr uint16_t BOMB_ROUTE_COST_DIVISOR = 100;    // 推炸弹本体路径按摊销代价参与观测顺序排序
    inline constexpr uint16_t FIRST_BOMB_LOCALITY_WEIGHT = 8;   // 巡图炸弹排序：优先处理当前附近的炸弹
    inline constexpr uint16_t FINAL_NEAR_BOX_RADIUS = 2;        // 收尾位置靠近箱子的判定半径
    inline constexpr uint16_t FINAL_NO_BOX_PENALTY = 10;        // 收尾位置远离箱子的时间惩罚
    inline constexpr uint16_t COST_INFINITY = 65535;            // 不可达代价哨兵值
    inline constexpr int32_t SEARCH_COST_INFINITY = 1000000000; // DFS 有符号代价上界，允许炸弹奖励产生负代价

    // ------------------------------------------------------------------------
    // 搜索缓存与分支上限
    // ------------------------------------------------------------------------
    inline constexpr int MAX_BOMB_APPROACH_OBS_BRANCHES = 4;    // 每个炸弹宏动作最多展开的顺路观测组合分支数
    inline constexpr int OBS_POSES_PER_YAW = 1;                 // 每个朝向保留一个候选；关键斜角由独立几何候选直接参与排序
    inline constexpr int OBS_POSE_BRANCHES = 4 * OBS_POSES_PER_YAW;
    inline constexpr int GRID_TIME_CACHE_SLOTS = 32;            // 小型 LRU 距离图缓存槽数，含进入方向约 12KB
    inline constexpr int PATROL_DFS_FRAME_LIMIT = 16;           // DFS 递归帧复用数组深度上限
    inline constexpr uint32_t PATROL_DFS_OPS_LIMIT = 15000;      // 巡图参考解只保留有限搜索预算
    inline constexpr int OBS_SUCCESSOR_HASH_SLOTS = 128;         // 单层观测后继去重哈希槽数
    inline constexpr int FALLBACK_CLEAR_MAX_STEPS = 5;          // 巡图兜底开通单格瓶颈允许的连续推送距离
    inline constexpr int SEED_POSES_PER_YAW = 2;                // 种子每个朝向保留两个候选做精确一步前瞻
    inline constexpr int SEED_POSE_BRANCHES = 4 * SEED_POSES_PER_YAW;

}

using namespace ExplorationConfig;

static_assert((OBS_SUCCESSOR_HASH_SLOTS & (OBS_SUCCESSOR_HASH_SLOTS - 1)) == 0,
              "Observation successor hash size must be a power of two");
static_assert(MAX_ENTITIES * OBS_POSE_BRANCHES < OBS_SUCCESSOR_HASH_SLOTS,
              "Observation successor hash table must retain one empty slot");

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

// 将车头朝向映射为 MOVE 下标，供时间寻路计算跨宏动作的首个拐点。
static int yaw_to_move_direction(float yaw) {
    if (!std::isfinite(yaw) || yaw < 0.0f) return -1;
    float normalized = yaw;
    while (normalized < 0.0f) normalized += 360.0f;
    while (normalized >= 360.0f) normalized -= 360.0f;
    const int yaw_index = static_cast<int>((normalized + 45.0f) / 90.0f) & 3;
    // 0 度为右，90 度为上，180 度为左，270 度为下。
    static constexpr int YAW_TO_MOVE[4] = {1, 0, 3, 2};
    return YAW_TO_MOVE[yaw_index];
}

// 返回路径最后一段的实际平移方向；空路径时保留调用方已有方向。
static int path_last_move_direction(point start,
                                    const StaticArray<point, MAX_PATH_LENGTH>& path,
                                    int fallback_dir = -1) {
    point previous = start;
    int last_dir = fallback_dir;
    for (int i = 0; i < path.size(); ++i) {
        const point current = path[i];
        const point delta = current - previous;
        for (int d = 0; d < 4; ++d) {
            if (delta == MOVE[d]) {
                last_dir = d;
                break;
            }
        }
        previous = current;
    }
    return last_dir;
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
    const uint32_t box_mask = level.box_count == 0u
        ? 0u
        : (uint32_t{1u} << level.box_count) - 1u;
    for (int e = 0; e < total_entities; ++e) {
        for (int i = 0; i < entity_views[e].size(); ++i) {
            const ViewPose& vp = entity_views[e][i];
            // 同一物理位姿的箱子候选与目标点候选是两个独立观测动作，不能合并
            const bool is_box_view = (vp.mask[0] & box_mask) != 0u;
            bool exists = false;
            for (int j = 0; j < out.size(); ++j) {
                const bool out_is_box_view = (out[j].mask[0] & box_mask) != 0u;
                if (out[j].pos == vp.pos && out[j].target_yaw == vp.target_yaw &&
                    out_is_box_view == is_box_view) {
                    if (is_box_view) {
                        out[j].mask[0] |= vp.mask[0];
                        if (vp.penalty[0] < out[j].penalty[0]) out[j].penalty[0] = vp.penalty[0];
                        exists = true;
                        break;
                    }
                    // 目标点同一物理位姿可能有多排候选，只有同一排 mask 才允许去重
                    if (out[j].mask[0] == vp.mask[0]) {
                        exists = true;
                        break;
                    }
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

                bool valid_any = false;
                uint32_t masks[MAX_BOMBS + 1] = {0};
                uint16_t pens[MAX_BOMBS + 1];
                PlanningCommon::TargetObserveSlots target_slots[MAX_BOMBS + 1] = {};
                for (int k = 0; k <= B; ++k) {
                    pens[k] = COST_INFINITY;
                }

                // 在每个炸弹阶段的地图快照上评估这个观测位姿
                for (int k = 0; k <= B; ++k) {
                    const SokobanLevel& lvl = multi_maps[k];
                    
                    uint32_t mask = 0u;
                    uint16_t penalty = 0u;
                    if (PlanningCommon::evaluate_observe_pose(
                            lvl, p, true_yaw, mask, penalty, &target_slots[k])) {
                        masks[k] = mask;
                        pens[k] = penalty;
                        valid_any = true;
                    }
                }

                if (valid_any) {
                    const uint32_t box_mask = cached_level.box_count == 0u
                        ? 0u
                        : (uint32_t{1u} << cached_level.box_count) - 1u;
                    const uint32_t target_mask =
                        ((uint32_t{1u} << total_entities) - 1u) & ~box_mask;

                    // 箱子和目标点共用物理位姿，但在巡图候选中保持两套
                    // 独立掩码，避免一次箱子观测顺带提交目标点，反之亦然。
                    auto append_category_views = [&](uint32_t category_mask,
                                                      bool is_box_category,
                                                      const uint32_t* view_masks,
                                                      const uint16_t* view_pens) {
                        ViewPose vp;
                        vp.pos = p;
                        vp.target_yaw = true_yaw;
                        bool category_valid = false;
                        for (int k = 0; k <= B; ++k) {
                            vp.mask[k] = view_masks[k] & category_mask;
                            vp.penalty[k] = is_box_category ? 0u : view_pens[k];
                            category_valid = category_valid || vp.mask[k] != 0u;
                        }
                        if (!category_valid) return;

                        auto target_view_rank = [&](const ViewPose& view) {
                            int max_coverage = 0;
                            uint16_t min_penalty = COST_INFINITY;
                            for (int stage = 0; stage <= B; ++stage) {
                                const int coverage = __builtin_popcount(
                                    view.mask[stage] & category_mask);
                                if (coverage > max_coverage) max_coverage = coverage;
                                if (view.mask[stage] != 0u &&
                                    view.penalty[stage] < min_penalty) {
                                    min_penalty = view.penalty[stage];
                                }
                            }
                            return max_coverage * 1024 -
                                   static_cast<int>(min_penalty == COST_INFINITY
                                       ? COST_INFINITY : min_penalty);
                        };

                        // 将该位姿挂到所有可见实体的候选列表中，后续 DFS 可 O(1) 查询
                        for (int e = 0; e < total_entities; ++e) {
                            bool can_see_e = false;
                            for (int k = 0; k <= B; ++k) {
                                if (vp.mask[k] & (1UL << e)) {
                                    can_see_e = true;
                                    break;
                                }
                            }
                            if (!can_see_e) continue;
                            if (entity_views[e].size() < 40) {
                                entity_views[e].push_back(vp);
                                continue;
                            }
                            if (is_box_category) continue;

                            // 目标候选容量满后保留联合覆盖能力更强的位姿，
                            // 避免地图扫描顺序把一次看三个的候选静默截掉。
                            int worst = 0;
                            int worst_rank = target_view_rank(entity_views[e][0]);
                            for (int candidate = 1; candidate < entity_views[e].size(); ++candidate) {
                                const int rank = target_view_rank(entity_views[e][candidate]);
                                if (rank < worst_rank) {
                                    worst = candidate;
                                    worst_rank = rank;
                                }
                            }
                            if (target_view_rank(vp) > worst_rank) {
                                entity_views[e][worst] = vp;
                            }
                        }
                    };

                    append_category_views(box_mask, true, masks, pens);

                    auto append_target_pattern = [&](int slot0, int slot1, int slot2) {
                        uint32_t pattern_masks[MAX_BOMBS + 1] = {};
                        uint16_t pattern_pens[MAX_BOMBS + 1];
                        const int pattern_slots[3] = {slot0, slot1, slot2};
                        for (int k = 0; k <= B; ++k) {
                            bool pattern_valid = true;
                            uint32_t pattern_mask = 0u;
                            uint16_t pattern_penalty = 0u;
                            for (int member = 0; member < 3; ++member) {
                                const int slot = pattern_slots[member];
                                if (slot < 0) continue;
                                if (target_slots[k].mask[slot] == 0u) {
                                    pattern_valid = false;
                                    break;
                                }
                                pattern_mask |= target_slots[k].mask[slot];
                                pattern_penalty = std::max(
                                    pattern_penalty,
                                    target_slots[k].penalty[slot]);
                            }
                            pattern_masks[k] = pattern_valid ? pattern_mask : 0u;
                            pattern_pens[k] = pattern_valid ? pattern_penalty : COST_INFINITY;
                        }
                        append_category_views(target_mask, false, pattern_masks, pattern_pens);
                    };

                    // 单目标只使用原有五种位置，F1 斜角不得单独形成观测动作
                    for (int slot = 0;
                         slot < PlanningCommon::ObservationConfig::TARGET_SINGLE_SLOT_COUNT;
                         ++slot) {
                        append_target_pattern(slot, -1, -1);
                    }

                    const int core = PlanningCommon::ObservationConfig::TARGET_SLOT_F2_CORE;
                    const int f2_left = PlanningCommon::ObservationConfig::TARGET_SLOT_F2_LEFT;
                    const int f2_right = PlanningCommon::ObservationConfig::TARGET_SLOT_F2_RIGHT;
                    const int f1_left = PlanningCommon::ObservationConfig::TARGET_SLOT_F1_LEFT;
                    const int f1_right = PlanningCommon::ObservationConfig::TARGET_SLOT_F1_RIGHT;

                    if constexpr (PlanningCommon::ObservationConfig::ENABLE_TARGET_JOINT_F2_DIAGONAL) {
                        // F2 斜角开关控制两个双目标模式和同排三目标模式
                        append_target_pattern(core, f2_left, -1);
                        append_target_pattern(core, f2_right, -1);
                        append_target_pattern(core, f2_left, f2_right);
                    }
                    if constexpr (PlanningCommon::ObservationConfig::ENABLE_TARGET_JOINT_F1_DIAGONAL) {
                        // F1 斜角开关控制两个双目标模式和近排三目标模式
                        append_target_pattern(core, f1_left, -1);
                        append_target_pattern(core, f1_right, -1);
                        append_target_pattern(core, f1_left, f1_right);
                    }
                    if constexpr (
                        PlanningCommon::ObservationConfig::ENABLE_TARGET_JOINT_F2_DIAGONAL &&
                        PlanningCommon::ObservationConfig::ENABLE_TARGET_JOINT_F1_DIAGONAL) {
                        // 混合三目标只允许左右交叉组合，对应图中的另外两种情况
                        append_target_pattern(core, f2_left, f1_right);
                        append_target_pattern(core, f2_right, f1_left);
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
    const SokobanLevel& initial_lvl, point start_pos, const StaticArray<BombTask, MAX_BOMBS>& raw_tasks) {
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

/// \brief 贪心种子的精确一步前瞻短名单项
struct PatrolSeedCandidate {
    int entity;              // entity_views 的实体下标
    int view;                // entity_views 的位姿下标
    int32_t shortlist_score; // 当前真实代价加剩余观测下界
    int32_t access_cost;     // 从当前状态执行本次观测的真实代价
    uint8_t coverage;        // 本次新增观测实体数量
    int8_t next_move_dir;    // 到达候选位后的实际末段移动方向
};

/// \brief 推炸弹途中插入观测的候选分支
struct PatrolApproachObsCandidate {
    ViewPose vp;                 // 插入的观测位姿
    uint32_t newly_seen;         // 本次观测新增实体掩码
    int32_t total_cost;          // 推炸弹路径和观测转向合并代价
    int32_t score;               // 排序分数，越小越优先
    int observe_after_push_count; // 在第几个推箱前置任务后插入观测
};

/// \brief 巡图 DFS 单层复用工作区
struct PatrolDfsFrame {
    uint16_t dist_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 当前状态到所有格子的时间代价
    uint8_t final_dir_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 到达各格子的最短路径末段方向
    PatrolEntityEval ordered_entities[32];            // 按最近观测代价排序的实体
    PatrolDynamicEval top_poses_by_yaw[OBS_POSE_BRANCHES]; // 每个朝向保留若干低代价位姿
    PatrolDynamicEval sorted_evals[OBS_POSE_BRANCHES];     // 当前实体待展开位姿
    uint64_t observe_successor_keys[OBS_SUCCESSOR_HASH_SLOTS]; // 本层已展开观测后继键
    uint16_t observe_successor_costs[OBS_SUCCESSOR_HASH_SLOTS]; // 等价后继的最低动作代价
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
    uint8_t grid_time_cache_final_dir[GRID_TIME_CACHE_SLOTS][MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 距离图对应的末段方向
    uint8_t grid_time_cache_valid[GRID_TIME_CACHE_SLOTS]; // 缓存槽有效标记
    uint8_t grid_time_cache_stage[GRID_TIME_CACHE_SLOTS]; // 缓存对应炸弹阶段
    point grid_time_cache_pos[GRID_TIME_CACHE_SLOTS];     // 缓存对应起点
    int8_t grid_time_cache_initial_dir[GRID_TIME_CACHE_SLOTS]; // 缓存对应进入起点前的行驶方向
    uint16_t grid_time_cache_stamp[GRID_TIME_CACHE_SLOTS]; // LRU 时间戳
    uint16_t grid_time_cache_clock = 0;                   // LRU 逻辑时钟
    BoundingContext ctx;                                  // DFS 全局边界上下文
    StaticArray<MacroAction, 32> seed_path;               // 贪心上界路径
    uint16_t seed_dist_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 贪心阶段距离图
    uint8_t seed_final_dir_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 贪心阶段末段方向图
    PatrolSeedCandidate seed_candidates[SEED_POSE_BRANCHES]; // 贪心阶段精确前瞻短名单
    StaticArray<point, MAX_PATH_LENGTH> seed_macro_path;  // 贪心阶段炸弹路径
    StaticArray<MacroAction, 32> fallback_path;           // DFS 失败后的兜底路径
    uint16_t fallback_dist_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 兜底阶段距离图
    uint8_t fallback_final_dir_map[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]; // 兜底阶段末段方向图
    StaticArray<point, MAX_PATH_LENGTH> fallback_macro_path; // 兜底炸弹路径
    StaticArray<point, MAX_PATH_LENGTH> fallback_soft_path;  // 兜底软路径
    StaticArray<point, MAX_PATH_LENGTH> fallback_push_path;  // 兜底推箱路径
    StaticArray<point, MAX_PATH_LENGTH> fallback_clear_path; // 兜底清障路径
    PatrolDfsFrame dfs_frames[PATROL_DFS_FRAME_LIMIT];       // 按递归深度复用的帧数组
};

// 巡图大工作区放 OCRAM，链接脚本必须将 .ocram_bss 放到非 DTCM 区域
OCRAM_BSS static PatrolScratchWorkspace patrol_ws;


/// \brief 搜索最优巡图动作序列
/// \param start_pos 巡图起点
/// \param raw_bomb_tasks 可插入的炸弹宏任务序列
/// \param start_yaw 起始朝向，负值表示忽略初始转向代价
/// \return 巡图动作序列，包含普通观测动作和推炸弹宏动作
///
/// \details
/// 算法使用多阶段动态状态空间搜索：
/// - 每个炸弹执行阶段对应一张地图快照
/// - 状态由当前位置、观测朝向、上一段实际移动方向、已观测实体 mask、炸弹阶段 k 组成
/// - 搜索过程中可选择观测动作，也可执行下一个炸弹宏动作切换到下一阶段
StaticArray<MacroAction, 32> Exploration::plan_optimal_patrol(
    point start_pos, const StaticArray<BombTask, MAX_BOMBS>& raw_bomb_tasks, float start_yaw, uint32_t start_mask) {
    total_entities = cached_level.box_count + cached_level.target_count;
    if (total_entities == 0) return StaticArray<MacroAction, 32>();

    int B = raw_bomb_tasks.size();
    OCRAM_BSS static SokobanLevel multi_maps[MAX_BOMBS + 1];
    multi_maps[0] = cached_level; 

    // 在地图快照生成前按真实推炸路径重排炸弹，优先处理当前可达且能尽早解锁地图的任务。
    auto bomb_tasks = optimize_bomb_timeline(cached_level, start_pos, raw_bomb_tasks);
    B = bomb_tasks.size();
    for (int k = 0; k < B; ++k) {
        multi_maps[k + 1] = multi_maps[k];
        apply_macro_bomb_effect(multi_maps[k + 1], bomb_tasks[k]);
    }

    build_entity_views(multi_maps, B);

    // 麦轮底盘平移代价低，转向单独计入代价
    auto get_turn_cost = [](float yaw1, float yaw2) -> uint16_t {
        return PlanningCommon::yaw_turn_time_cost(yaw1, yaw2);
    };

    auto get_bomb_route_cost = [](point start_pos,
                                  const StaticArray<point, MAX_PATH_LENGTH>& path,
                                  const BombTask& task,
                                  int initial_dir) -> uint32_t {
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
        uint32_t approach_cost = PlanningCommon::path_time_cost(start_pos, approach_path, initial_dir);
        const int push_initial_dir = path_last_move_direction(
            start_pos, approach_path, initial_dir);
        uint32_t push_cost = PlanningCommon::path_time_cost(
            push_start, push_path, push_initial_dir);
        return approach_cost + push_cost / BOMB_ROUTE_COST_DIVISOR;
    };

    auto apply_bomb_reward = [](uint32_t raw_cost) -> int32_t {
        return static_cast<int32_t>(raw_cost) - BONUS_FOR_BOMB;
    };

    auto get_bomb_action_cost = [&](point start_pos,
                                    const StaticArray<point, MAX_PATH_LENGTH>& path,
                                    const BombTask& task,
                                    int initial_dir) -> int32_t {
        return apply_bomb_reward(get_bomb_route_cost(start_pos, path, task, initial_dir));
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
                                        const MacroAction* inserted_observe,
                                        int initial_move_dir) -> bool {
        int action_count = task.box_pushes.size() + 1 + (inserted_observe ? 1 : 0);
        if (out.size() + action_count > 32) return false;

        SokobanLevel work = stage;
        point work_player = player;
        int work_move_dir = initial_move_dir;

        auto append_observe_if_needed = [&](int push_count_done) -> bool {
            if (!inserted_observe || push_count_done != observe_after_push_count) return true;

            StaticArray<point, MAX_PATH_LENGTH> obs_path;
            const point before_observe = work_player;
            if (!PlanningCommon::get_grid_time_path(
                    work, before_observe, inserted_observe->observe.view.pos, obs_path, work_move_dir)) {
                return false;
            }
            out.push_back(*inserted_observe);
            work_player = inserted_observe->observe.view.pos;
            work_move_dir = path_last_move_direction(before_observe, obs_path, work_move_dir);
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
                PlanningCommon::path_time_cost(before_player, path, work_move_dir)
            );
            out.push_back(push_action);
            work_move_dir = path_last_move_direction(before_player, path, work_move_dir);

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
    ctx.ops_limit = PATROL_DFS_OPS_LIMIT;
    ctx.ops_count = 0;
    std::memset(ws.materialize_checked, 0, sizeof(ws.materialize_checked));
    std::memset(ws.materialize_ok, 0, sizeof(ws.materialize_ok));
    std::memset(ws.grid_time_cache_valid, 0, sizeof(ws.grid_time_cache_valid));
    ws.grid_time_cache_clock = 0;
    auto build_grid_time_map_cached = [&](int stage,
                                          point pos,
                                          int initial_dir,
                                          uint16_t out[MAP_MAX_HEIGHT][MAP_MAX_WIDTH],
                                          uint8_t out_final_dir[MAP_MAX_HEIGHT][MAP_MAX_WIDTH]) {
        ++ws.grid_time_cache_clock;

        for (int i = 0; i < GRID_TIME_CACHE_SLOTS; ++i) {
            if (ws.grid_time_cache_valid[i] &&
                ws.grid_time_cache_stage[i] == stage &&
                ws.grid_time_cache_pos[i] == pos &&
                ws.grid_time_cache_initial_dir[i] == initial_dir) {
                ws.grid_time_cache_stamp[i] = ws.grid_time_cache_clock;
                std::memcpy(out, ws.grid_time_cache[i], sizeof(ws.grid_time_cache[i]));
                if (out_final_dir) {
                    std::memcpy(out_final_dir,
                                ws.grid_time_cache_final_dir[i],
                                sizeof(ws.grid_time_cache_final_dir[i]));
                }
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

        PlanningCommon::build_grid_time_map(
            multi_maps[stage],
            pos,
            ws.grid_time_cache[slot],
            initial_dir,
            ws.grid_time_cache_final_dir[slot]);
        ws.grid_time_cache_valid[slot] = 1;
        ws.grid_time_cache_stage[slot] = static_cast<uint8_t>(stage);
        ws.grid_time_cache_pos[slot] = pos;
        ws.grid_time_cache_initial_dir[slot] = static_cast<int8_t>(initial_dir);
        ws.grid_time_cache_stamp[slot] = ws.grid_time_cache_clock;
        std::memcpy(out, ws.grid_time_cache[slot], sizeof(ws.grid_time_cache[slot]));
        if (out_final_dir) {
            std::memcpy(out_final_dir,
                        ws.grid_time_cache_final_dir[slot],
                        sizeof(ws.grid_time_cache_final_dir[slot]));
        }
    };

    int req_boxes = cached_level.box_count - 1;       
    int req_targets = cached_level.target_count - 1;  
    if (req_boxes < 0) req_boxes = 0;
    if (req_targets < 0) req_targets = 0;
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
    auto minimum_remaining_observations = [&](uint32_t semantic_mask) -> int {
        int seen_boxes = 0;
        int seen_targets = 0;
        for (int e = 0; e < total_entities; ++e) {
            if ((semantic_mask & (1UL << e)) == 0u) continue;
            if (e < cached_level.box_count) ++seen_boxes;
            else ++seen_targets;
        }
        const int remain_boxes = std::max(0, req_boxes - seen_boxes);
        const int remain_targets = std::max(0, req_targets - seen_targets);
        return remain_boxes +
            (remain_targets + PlanningCommon::ObservationConfig::MAX_TARGETS_PER_OBSERVATION - 1) /
                PlanningCommon::ObservationConfig::MAX_TARGETS_PER_OBSERVATION;
    };

    auto seed_greedy_upper_bound = [&]() -> void {
        StaticArray<MacroAction, 32>& seed_path = ws.seed_path;
        seed_path.clear();
        uint32_t mask = start_mask;
        point curr_pos = start_pos;
        float curr_yaw = start_yaw;
        int curr_move_dir = yaw_to_move_direction(start_yaw);
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
            build_grid_time_map_cached(
                k, curr_pos, curr_move_dir, dist_map, ws.seed_final_dir_map);

            int best_entity = -1;
            int best_view = -1;
            int best_score = 0x7FFFFFFF;
            int best_pop = 0;
            int reachable_remaining_boxes = 0;
            int reachable_remaining_targets = 0;

            for (int i = 0; i < SEED_POSE_BRANCHES; ++i) {
                ws.seed_candidates[i] = {-1, -1, 0x7FFFFFFF, COST_INFINITY, 0u, -1};
            }

            auto seed_candidate_better = [&](const PatrolSeedCandidate& lhs,
                                             const PatrolSeedCandidate& rhs) {
                if (rhs.entity < 0) return true;
                if (lhs.shortlist_score != rhs.shortlist_score) {
                    return lhs.shortlist_score < rhs.shortlist_score;
                }
                const ViewPose& lhs_view = entity_views[lhs.entity][lhs.view];
                const ViewPose& rhs_view = entity_views[rhs.entity][rhs.view];
                if (lhs_view.penalty[k] != rhs_view.penalty[k]) {
                    return lhs_view.penalty[k] < rhs_view.penalty[k];
                }
                return lhs.coverage > rhs.coverage;
            };

            auto insert_seed_candidate = [&](const PatrolSeedCandidate& candidate,
                                             int yaw_index) {
                for (int i = 0; i < SEED_POSE_BRANCHES; ++i) {
                    const PatrolSeedCandidate& old = ws.seed_candidates[i];
                    if (old.entity < 0) continue;
                    const ViewPose& old_view = entity_views[old.entity][old.view];
                    const ViewPose& new_view = entity_views[candidate.entity][candidate.view];
                    if (old_view.pos == new_view.pos &&
                        old_view.target_yaw == new_view.target_yaw &&
                        old_view.mask[k] == new_view.mask[k]) {
                        if (seed_candidate_better(candidate, old)) ws.seed_candidates[i] = candidate;
                        return;
                    }
                }

                const int begin = yaw_index * SEED_POSES_PER_YAW;
                for (int rank = 0; rank < SEED_POSES_PER_YAW; ++rank) {
                    const int slot = begin + rank;
                    if (!seed_candidate_better(candidate, ws.seed_candidates[slot])) continue;
                    for (int shift = SEED_POSES_PER_YAW - 1; shift > rank; --shift) {
                        ws.seed_candidates[begin + shift] = ws.seed_candidates[begin + shift - 1];
                    }
                    ws.seed_candidates[slot] = candidate;
                    return;
                }
            };

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

                    const int access_cost = OBSERVE_ACTION_COST + dist +
                        get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k];
                    int pop = 0;
                    for (int bit = 0; bit < 20; ++bit) {
                        if (newly_seen & (1UL << bit)) ++pop;
                    }
                    int yaw_index = static_cast<int>((vp.target_yaw + 45.0f) / 90.0f) & 3;
                    const PatrolSeedCandidate candidate{
                        e,
                        i,
                        access_cost + minimum_remaining_observations(mask | newly_seen) * OBSERVE_ACTION_COST,
                        access_cost,
                        static_cast<uint8_t>(pop),
                        static_cast<int8_t>(ws.seed_final_dir_map[vp.pos.y][vp.pos.x])
                    };
                    insert_seed_candidate(candidate, yaw_index);
                }
                if (entity_reachable) {
                    if (e < cached_level.box_count) ++reachable_remaining_boxes;
                    else ++reachable_remaining_targets;
                }
            }

            // 只对每个朝向的少量候选建立真实距离图，避免曼哈顿前瞻把绕路关系排错
            for (int i = 0; i < SEED_POSE_BRANCHES; ++i) {
                const PatrolSeedCandidate& candidate = ws.seed_candidates[i];
                if (candidate.entity < 0) continue;
                const ViewPose& vp = entity_views[candidate.entity][candidate.view];
                const uint32_t next_mask = mask | vp.mask[k];
                const int next_move_dir =
                    candidate.next_move_dir >= 0 && candidate.next_move_dir < 4
                        ? candidate.next_move_dir : curr_move_dir;
                int next_box_seen = 0;
                int next_target_seen = 0;
                for (int entity = 0; entity < total_entities; ++entity) {
                    if ((next_mask & (1UL << entity)) == 0u) continue;
                    if (entity < cached_level.box_count) ++next_box_seen;
                    else ++next_target_seen;
                }
                const bool needs_box = next_box_seen < req_boxes;
                const bool needs_target = next_target_seen < req_targets;
                int next_access = 0;
                if (needs_box || needs_target) {
                    uint16_t (&lookahead_map)[MAP_MAX_HEIGHT][MAP_MAX_WIDTH] =
                        ws.dfs_frames[0].dist_map;
                    build_grid_time_map_cached(
                        k, vp.pos, next_move_dir, lookahead_map, nullptr);
                    next_access = 0x7FFFFFFF;
                    for (int entity = 0; entity < total_entities; ++entity) {
                        if (next_mask & (1UL << entity)) continue;
                        if ((entity < cached_level.box_count) ? !needs_box : !needs_target) continue;
                        for (int view = 0; view < entity_views[entity].size(); ++view) {
                            const ViewPose& next_view = entity_views[entity][view];
                            if ((next_view.mask[k] & ~next_mask) == 0u) continue;
                            const uint16_t next_dist = lookahead_map[next_view.pos.y][next_view.pos.x];
                            if (next_dist == COST_INFINITY) continue;
                            const int estimate = OBSERVE_ACTION_COST + next_dist +
                                get_turn_cost(vp.target_yaw, next_view.target_yaw) +
                                next_view.penalty[k];
                            if (estimate < next_access) next_access = estimate;
                        }
                    }
                    if (next_access == 0x7FFFFFFF) next_access = 0;
                }
                const int score = candidate.access_cost + next_access;
                if (score < best_score ||
                    (score == best_score && candidate.coverage > best_pop)) {
                    best_score = score;
                    best_entity = candidate.entity;
                    best_view = candidate.view;
                    best_pop = candidate.coverage;
                }
            }

            if (best_entity >= 0) {
                const ViewPose& vp = entity_views[best_entity][best_view];
                uint32_t newly_seen = vp.mask[k] & ~mask;
                uint16_t dist = dist_map[vp.pos.y][vp.pos.x];
                int next_move_dir = ws.seed_final_dir_map[vp.pos.y][vp.pos.x];
                if (next_move_dir < 0 || next_move_dir >= 4) next_move_dir = curr_move_dir;
                MacroAction act = make_observe_macro_action(vp, vp.mask[k]);
                seed_path.push_back(act);
                current_cost += OBSERVE_ACTION_COST + dist +
                    get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k];
                curr_pos = vp.pos;
                curr_yaw = vp.target_yaw;
                curr_move_dir = next_move_dir;
                mask |= newly_seen;
                continue;
            }

            bool needs_stage_unlock = (reachable_remaining_boxes < remain_b) || (reachable_remaining_targets < remain_t);
            if (k < B && needs_stage_unlock) {
                StaticArray<point, MAX_PATH_LENGTH>& macro_path = ws.seed_macro_path;
                macro_path.clear();
                BombTask executable_task = bomb_tasks[k];
                if (!PlanningCommon::get_bomb_push_path(multi_maps[k], curr_pos, executable_task, macro_path)) return;
                if (!append_flat_bomb_actions(
                        seed_path, multi_maps[k], curr_pos, executable_task, -1, nullptr, curr_move_dir)) {
                    return;
                }
                current_cost += get_bomb_action_cost(
                    curr_pos, macro_path, executable_task, curr_move_dir);
                curr_move_dir = path_last_move_direction(curr_pos, macro_path, curr_move_dir);
                curr_pos = macro_path.empty() ? curr_pos : macro_path.back();
                ++k;
                continue;
            }

            return;
        }
    };
    seed_greedy_upper_bound();

    auto dfs = [&](auto& self,
                   point curr_pos,
                   float curr_yaw,
                   int curr_move_dir,
                   int k,
                   uint32_t mask,
                   int32_t current_cost,
                   int depth) -> void {
        if (depth >= PATROL_DFS_FRAME_LIMIT) return;
        PatrolDfsFrame& frame = ws.dfs_frames[depth];
        if (ctx.ops_count >= ctx.ops_limit) return;
        ++ctx.ops_count;
        int32_t remaining_bomb_credit = static_cast<int32_t>(B - k) * BONUS_FOR_BOMB;
        if (current_cost - remaining_bomb_credit >= ctx.best_cost) return;

        // 状态压缩：观测掩码 + 位置 + 观测朝向 + 上一段实际移动方向 + 炸弹阶段
        int yaw_idx = 0;
        if (curr_yaw >= 0.0f) {
            int int_yaw = (int)(curr_yaw + 0.5f) % 360;
            if (int_yaw < 0) int_yaw += 360;
            yaw_idx = (int_yaw / 90) % 4;
        }

        const uint64_t move_dir_code = static_cast<uint64_t>(curr_move_dir + 1) & 0x07u;
        uint64_t safe_state = (static_cast<uint64_t>(mask) << 32) |
                              (static_cast<uint64_t>(curr_pos.y) << 24) |
                              (static_cast<uint64_t>(curr_pos.x) << 16) |
                              (static_cast<uint64_t>(yaw_idx) << 12) |
                              (move_dir_code << 8) |
                              static_cast<uint64_t>(k & 0xFF);
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
        // 箱子候选一次只提交一个箱子；目标候选最多合并三个目标点。
        // 每个动作都必付停车观测开销，因此这是不会高估的观测次数下界。
        const int remain_observations = minimum_remaining_observations(mask);
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
        if (current_cost + remain_observations * OBSERVE_ACTION_COST - remaining_bomb_credit >=
            ctx.best_cost) {
            return;
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
        build_grid_time_map_cached(
            k, curr_pos, curr_move_dir, dist_map, frame.final_dir_map);

        int unvisited_count = 0;
        uint16_t nearest_observe_extra = COST_INFINITY;

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
                if (cost < best_cost_for_e) best_cost_for_e = cost;
                if (cost < nearest_observe_extra) nearest_observe_extra = cost;
            }
            frame.ordered_entities[unvisited_count++] = {e, best_cost_for_e};
        }

        // 最后炸弹阶段地图不再变化，最近观测的接近代价可安全加入下界
        if (k == B) {
            if (nearest_observe_extra == COST_INFINITY) return;
            const int32_t final_stage_lower_bound = current_cost +
                remain_observations * OBSERVE_ACTION_COST + nearest_observe_extra;
            if (final_stage_lower_bound >= ctx.best_cost) return;
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
        // 候选必含新增观测位，因此后继键非零，可用零值表示空槽
        std::fill(frame.observe_successor_keys,
                  frame.observe_successor_keys + OBS_SUCCESSOR_HASH_SLOTS,
                  uint64_t{0});

        // 每个实体先保留各朝向最佳候选，再按后继状态去重
        for (int idx = 0; idx < unvisited_count; ++idx) {
            int e = frame.ordered_entities[idx].id;
            if (frame.ordered_entities[idx].min_cost == COST_INFINITY) continue; 

            for (int i = 0; i < OBS_POSE_BRANCHES; ++i) {
                frame.top_poses_by_yaw[i] = {-1, COST_INFINITY, 0x7FFFFFFF};
            }

            for (int i = 0; i < entity_views[e].size(); ++i) {
                const auto& vp = entity_views[e][i];
                uint32_t newly_seen = vp.mask[k] & ~mask;
                if (newly_seen == 0) continue; 
                if (!newly_covers_needed_category(newly_seen)) continue;

                uint16_t dist = dist_map[vp.pos.y][vp.pos.x]; 
                if (dist == 0xFFFF) continue;

                uint16_t turn_cost = get_turn_cost(curr_yaw, vp.target_yaw);
                uint16_t total_cost = OBSERVE_ACTION_COST + dist + turn_cost + vp.penalty[k];

                // 当前动作和剩余观测下界使用同一总代价，覆盖收益不再重复计权
                int32_t score = total_cost +
                    minimum_remaining_observations(mask | newly_seen) * OBSERVE_ACTION_COST;

                int vp_yaw_idx = 0;
                int int_yaw = (int)(vp.target_yaw + 0.5f) % 360;
                if (int_yaw < 0) int_yaw += 360;
                vp_yaw_idx = (int_yaw / 90) % 4;

                const int yaw_begin = vp_yaw_idx * OBS_POSES_PER_YAW;
                for (int rank = 0; rank < OBS_POSES_PER_YAW; ++rank) {
                    const int slot = yaw_begin + rank;
                    if (score >= frame.top_poses_by_yaw[slot].score) continue;
                    for (int shift = OBS_POSES_PER_YAW - 1; shift > rank; --shift) {
                        frame.top_poses_by_yaw[yaw_begin + shift] =
                            frame.top_poses_by_yaw[yaw_begin + shift - 1];
                    }
                    frame.top_poses_by_yaw[slot] = {i, total_cost, score};
                    break;
                }
            }

            int valid_eval_count = 0;
            for (int i = 0; i < OBS_POSE_BRANCHES; ++i) {
                if (frame.top_poses_by_yaw[i].vp_idx != -1) {
                    frame.sorted_evals[valid_eval_count++] = frame.top_poses_by_yaw[i];
                }
            }

            for (int i = 0; i < valid_eval_count - 1; ++i) {
                for (int j = 0; j < valid_eval_count - 1 - i; ++j) {
                    if (frame.sorted_evals[j].score > frame.sorted_evals[j + 1].score) {
                        auto temp = frame.sorted_evals[j];
                        frame.sorted_evals[j] = frame.sorted_evals[j + 1];
                        frame.sorted_evals[j + 1] = temp;
                    }
                }
            }

            for (int i = 0; i < valid_eval_count; ++i) {
                const PatrolDynamicEval& eval = frame.sorted_evals[i];

                const ViewPose& vp = entity_views[e][eval.vp_idx];
                const uint32_t next_mask = mask | vp.mask[k];
                int next_move_dir = frame.final_dir_map[vp.pos.y][vp.pos.x];
                if (next_move_dir < 0 || next_move_dir >= 4) next_move_dir = curr_move_dir;

                int int_yaw = static_cast<int>(vp.target_yaw + 0.5f) % 360;
                if (int_yaw < 0) int_yaw += 360;
                const uint64_t successor_key = static_cast<uint64_t>(next_mask) |
                    (static_cast<uint64_t>(static_cast<uint8_t>(vp.pos.x)) << 20) |
                    (static_cast<uint64_t>(static_cast<uint8_t>(vp.pos.y)) << 24) |
                    (static_cast<uint64_t>((int_yaw / 90) & 3) << 28) |
                    (static_cast<uint64_t>(next_move_dir + 1) << 30);
                uint32_t hash = static_cast<uint32_t>(
                    (successor_key ^ (successor_key >> 32)) * 2654435761U);
                int slot = hash & (OBS_SUCCESSOR_HASH_SLOTS - 1);
                while (frame.observe_successor_keys[slot] != 0u &&
                       frame.observe_successor_keys[slot] != successor_key) {
                    slot = (slot + 1) & (OBS_SUCCESSOR_HASH_SLOTS - 1);
                }

                // 同一后继只保留更低动作代价，保持原有实体和位姿展开顺序
                if (frame.observe_successor_keys[slot] == successor_key &&
                    frame.observe_successor_costs[slot] <= eval.actual_cost) {
                    continue;
                }
                frame.observe_successor_keys[slot] = successor_key;
                frame.observe_successor_costs[slot] = eval.actual_cost;

                MacroAction act = make_observe_macro_action(vp, vp.mask[k]);
                ctx.current_path.push_back(act);
                self(self,
                     vp.pos,
                     vp.target_yaw,
                     next_move_dir,
                     k,
                     next_mask,
                     current_cost + eval.actual_cost,
                     depth + 1);
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
                        uint32_t prefix_cost = PlanningCommon::path_time_cost(
                            curr_pos, prefix, curr_move_dir);
                        uint32_t observe_turn_cost = OBSERVE_ACTION_COST +
                            get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k];
                        uint32_t combined_bomb_route_cost =
                            prefix_cost + get_bomb_route_cost(
                                vp.pos,
                                suffix,
                                executable_task,
                                yaw_to_move_direction(vp.target_yaw));
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

                        int32_t score = combined_cost +
                            minimum_remaining_observations(mask | newly_seen) * OBSERVE_ACTION_COST;
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
                                                  &obs_act,
                                                  curr_move_dir)) {
                        continue;
                    }

                    point next_pos_after_obs = macro_path.empty() ? candidate.vp.pos : macro_path.back();
                    const int next_move_dir = path_last_move_direction(
                        curr_pos, macro_path, curr_move_dir);
                    // Movement and push/bomb macros do not reset observe yaw in execution
                    // Keep DFS yaw consistent so the next observation pays the real turn cost
                    self(self,
                         next_pos_after_obs,
                         candidate.vp.target_yaw,
                         next_move_dir,
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
                                              nullptr,
                                              curr_move_dir)) {
                    return;
                }

                point next_pos = macro_path.empty() ? curr_pos : macro_path.back();
                int32_t bomb_cost = get_bomb_action_cost(
                    curr_pos, macro_path, executable_task, curr_move_dir);
                const int next_move_dir = path_last_move_direction(
                    curr_pos, macro_path, curr_move_dir);
                // No observation happened inside this macro branch, so carry the previous observe yaw
                self(self,
                     next_pos,
                     curr_yaw,
                     next_move_dir,
                     k + 1,
                     mask,
                     current_cost + bomb_cost,
                     depth + 1);
                pop_flat_bomb_actions(ctx.current_path, executable_task, false);
            }
        }
    };

    dfs(dfs,
        start_pos,
        start_yaw,
        yaw_to_move_direction(start_yaw),
        0,
        start_mask,
        0,
        0);

    if (!ctx.best_path.empty() && validate_reference_plan(ctx.best_path)) {
        return ctx.best_path;
    }

    // DFS 未找到完整巡图序列时，使用有限贪心兜底
    // 兜底逻辑优先做可达观测，若无观测可走则尝试执行下一颗炸弹打开地图
    StaticArray<MacroAction, 32>& fallback_path = ws.fallback_path;
    fallback_path.clear();
    uint32_t mask = start_mask;
    point curr_pos = start_pos;
    float curr_yaw = start_yaw;
    int curr_move_dir = yaw_to_move_direction(start_yaw);
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
        PlanningCommon::build_grid_time_map(
            stage_lvl, curr_pos, dist_map, curr_move_dir, ws.fallback_final_dir_map);

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
                int score = dist + get_turn_cost(curr_yaw, vp.target_yaw) + vp.penalty[k] +
                    minimum_remaining_observations(mask | newly_seen) * OBSERVE_ACTION_COST;
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
                if (!append_flat_bomb_actions(
                        fallback_path, stage_lvl, curr_pos, executable_task, -1, nullptr, curr_move_dir)) {
                    break;
                }
                curr_move_dir = path_last_move_direction(curr_pos, macro_path, curr_move_dir);
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

            auto single_box_can_reach_target = [](const SokobanLevel& level,
                                                  point player,
                                                  uint8_t box_id,
                                                  uint8_t target_id) -> bool {
                if (box_id >= level.box_count || target_id >= level.target_count) return false;
                if (level.boxes[box_id] == level.targets[target_id]) return true;

                SokobanLevel soft = level;
                soft.box_count = 1;
                soft.boxes[0] = level.boxes[box_id];
                soft.box_semantics[0] = level.box_semantics[box_id];
                soft.bomb_count = 0;

                point soft_player = player;
                StaticArray<point, MAX_PATH_LENGTH> path;
                BoxPushTask task{soft.boxes[0], level.targets[target_id]};
                return PlanningCommon::append_box_push_path(soft, soft_player, task, path);
            };

            auto clear_keeps_all_target_options = [&](const SokobanLevel& level, point player, uint8_t box_id) -> bool {
                if (!PlanningCommon::is_box_position_safe(level, box_id, full_target_mask)) return false;

                // 巡图兜底清障发生在语义绑定前，不能提前把箱子锁进只适配某个目标的单向位置
                for (int t = 0; t < level.target_count; ++t) {
                    if (!single_box_can_reach_target(level, player, box_id, static_cast<uint8_t>(t))) return false;
                }
                return true;
            };

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

                    for (int d = 0; d < 4; ++d) {
                        for (int step = 1; step <= FALLBACK_CLEAR_MAX_STEPS; ++step) {
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
                            if (!clear_keeps_all_target_options(probe, probe_player, static_cast<uint8_t>(blocking_box))) continue;

                            uint16_t after_direct = PlanningCommon::shortest_grid_time_cost(
                                probe, probe_player, vp.pos);
                            if (after_direct == 65535) continue;

                            uint16_t push_cost = PlanningCommon::path_time_cost(curr_pos, push_path);
                            int score = static_cast<int>(push_cost) +
                                static_cast<int>(after_direct) / 2 + vp.penalty[k] +
                                minimum_remaining_observations(mask | newly_seen) *
                                    OBSERVE_ACTION_COST;

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
                    curr_move_dir = path_last_move_direction(curr_pos, clear_path, curr_move_dir);
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
            int next_move_dir = ws.fallback_final_dir_map[vp.pos.y][vp.pos.x];
            if (next_move_dir < 0 || next_move_dir >= 4) next_move_dir = curr_move_dir;
            MacroAction act = make_observe_macro_action(vp, vp.mask[k]);
            fallback_path.push_back(act);
            mask |= vp.mask[k];
            curr_pos = vp.pos;
            curr_yaw = vp.target_yaw;
            curr_move_dir = next_move_dir;
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

        if (!append_flat_bomb_actions(
                fallback_path, multi_maps[k], curr_pos, executable_task, -1, nullptr, curr_move_dir)) {
            break;
        }
        curr_move_dir = path_last_move_direction(curr_pos, macro_path, curr_move_dir);
        curr_pos = macro_path.empty() ? curr_pos : macro_path.back();
        ++k;
    }

    if (mask_has_required_counts(mask) && validate_reference_plan(fallback_path)) return fallback_path;
    return StaticArray<MacroAction, 32>();
}

