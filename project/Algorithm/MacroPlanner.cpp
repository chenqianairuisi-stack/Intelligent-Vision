#include "MacroPlanner.h"
#include "Strategy.h"
#include <cstring>

DTCM_DATA MacroPlanner macro_planner;

// ============================================================================
// 参数面板
// ============================================================================

namespace MacroConfig {
    // ------------------------------------------------------------------------
    // 槽位功能开关
    // ------------------------------------------------------------------------
    inline constexpr bool ENABLE_COMPLETION_PUSH_SLOT = false;       // 是否启用完成式推箱候选位
    inline constexpr bool ENABLE_APPROACH_PUSH_SLOT = false;         // 是否启用接近式推箱候选位
    inline constexpr bool ENABLE_OPPORTUNISTIC_BOMB_SLOT = false;    // 是否启用机会性炸弹候选位
    inline constexpr bool ENABLE_SUPPLEMENTAL_OBSERVE_SLOT = false;  // 是否启用补充式观测候选位

    // ------------------------------------------------------------------------
    // 完成式推箱候选位
    // ------------------------------------------------------------------------
    // score =
    //     follow_gain      * COMPLETION_FOLLOW_WEIGHT
    //   + deferred_return  * COMPLETION_RETURN_WEIGHT
    //   + pressure_reward
    //   - player_to_box    * COMPLETION_DISTANCE_WEIGHT
    //   + COMPLETION_PRESSURE_REWARD
    inline constexpr int COMPLETION_SCORE_THRESHOLD = 0;    // 执行阈值，给镜像地图的路径拐点差异留出余量
    inline constexpr int COMPLETION_FOLLOW_WEIGHT = 3;      // 推箱后接回下一参考任务的时间收益权重 [调大：更偏好顺路完成]
    inline constexpr int COMPLETION_RETURN_WEIGHT = 1;      // 未来少一次回到该箱子附近的时间收益权重 [调大：更愿意提前处理远处箱子]
    inline constexpr int COMPLETION_DISTANCE_WEIGHT = 1;    // 小车到箱子可站位时间代价的惩罚权重 [调大：更偏向离车近的箱子]
    inline constexpr int COMPLETION_PRESSURE_REWARD = 14;   // 箱子直接落到目标点后，对第二阶段 IDA* 搜索压力下降的基础时间奖励
    inline constexpr int COMPLETION_MAX_PUSH_PATH = 16;     // 完成式推箱展开路径长度上限

    // ------------------------------------------------------------------------
    // 参考候选位清障
    // ------------------------------------------------------------------------
    inline constexpr int REFERENCE_CLEAR_MAX_STEPS = 2;    //沿四方向最多尝试把挡路箱子推开几格
    inline constexpr int REFERENCE_CLEAR_MAX_PATH = 20;    // 单次清障推箱允许的最大展开路径长度
    inline constexpr int REFERENCE_MATERIALIZED_CLEAR_BONUS = 950; // Strategy 已验证清障序列的优先级奖励

    // ------------------------------------------------------------------------
    // 补充式观测候选位
    // ------------------------------------------------------------------------
    inline constexpr int SUPPLEMENTAL_NON_BOX_PENALTY = 8; // 补看目标点而不是箱子时的收尾惩罚
}

static constexpr int kInfScore = 1000000;


// ============================================================================
// 模块 1：在线调度入口
// ============================================================================

/// \brief 按固定候选位选择下一条宏动作
/// \param ctx 当前地图、位置、朝向和炸弹任务上下文
/// \param out_action 输出本轮要执行的宏动作
/// \return 成功选出动作时返回 true；参考主线不可推进且无法清障时返回 false
///
/// \details
/// 调度顺序为：先处理上一轮留下的 followup，再构造参考候选位，
/// 然后依次评估完成式推箱、接近式推箱、机会性炸弹三个插入槽位。
/// 当前只有完成式推箱启用，其余槽位保留接口。
bool MacroPlanner::plan_next_action(const MacroPlanContext& ctx, MacroAction& out_action) {
    sync_semantics(semantic_labels);

    if (pending_cursor < pending_actions.size()) {
        MacroAction pending = pending_actions[pending_cursor++];
        MacroAction prepared;
        out_action = prepare_reference_action(ctx, pending, prepared) ? prepared : pending;
        if (pending_cursor >= pending_actions.size()) {
            pending_actions.clear();
            pending_cursor = 0;
        }
        return true;
    }

    SlotCandidate reference_slot;
    if (!build_reference_slot(ctx, reference_slot)) {
        SlotCandidate supplemental_slot;
        if (build_supplemental_observe_slot(ctx, supplemental_slot)) {
            out_action = supplemental_slot.action;
            pending_actions = supplemental_slot.followups;
            pending_cursor = 0;
            return true;
        }
        force_supplemental_fallback(ctx.level);
        return false;
    }

    SlotCandidate best_optional;
    best_optional.score = -kInfScore;

    SlotCandidate completion_slot;
    if (build_completion_push_slot(ctx, reference_slot.action, completion_slot)) {
        best_optional = completion_slot;
    }

    SlotCandidate approach_slot;
    if (build_approach_push_slot(ctx, reference_slot.action, approach_slot)) {
        if (!best_optional.valid || approach_slot.score > best_optional.score) best_optional = approach_slot;
    }

    SlotCandidate bomb_slot;
    if (build_opportunistic_bomb_slot(ctx, reference_slot.action, bomb_slot)) {
        if (!best_optional.valid || bomb_slot.score > best_optional.score) best_optional = bomb_slot;
    }

    SlotCandidate chosen = best_optional.valid ? best_optional : reference_slot;
    out_action = chosen.action;
    pending_actions = chosen.followups;
    pending_cursor = 0;
    if (chosen.consumes_reference && chosen.slot == SlotKind::REFERENCE) {
        ++reference_cursor;
    }

    return true;
}

// ============================================================================
// 模块 2：文件内轻量工具
// ============================================================================

// 整数绝对值，避免在小工具里引入额外重载
static int abs_i(int v) {
    return v < 0 ? -v : v;
}

static int manhattan(point a, point b) {
    return abs_i(a.x - b.x) + abs_i(a.y - b.y);
}

static bool is_valid_semantic_id(int8_t label) {
    return label >= 0 && label <= 9;
}

static int first_unused_semantic_id(const int box_counts[10], const int target_counts[10]) {
    for (int s = 0; s < 10; ++s) {
        if (box_counts[s] == 0 && target_counts[s] == 0) return s;
    }
    return 0;
}

static bool mask_contains_box(uint32_t mask, int box_count) {
    for (int b = 0; b < box_count; ++b) {
        if (mask & (1UL << b)) return true;
    }
    return false;
}

// 判断参考炸弹是否还没执行；炸弹被使用后会在地图里标记为 {-1, -1}
static bool bomb_still_present(const SokobanLevel& level, const BombTask& task) {
    for (int i = 0; i < level.bomb_count; ++i) {
        if (level.bombs[i].x != -1 && level.bombs[i] == task.bomb_start) return true;
    }
    return false;
}

// 检查某格是否被其他箱子或尚未引爆的炸弹占用，供清障推箱枚举使用
static bool has_dynamic_entity(const SokobanLevel& level, point p, int ignored_box = -1) {
    for (int b = 0; b < level.box_count; ++b) {
        if (b != ignored_box && level.boxes[b] == p) return true;
    }
    for (int b = 0; b < level.bomb_count; ++b) {
        if (level.bombs[b].x != -1 && level.bombs[b] == p) return true;
    }
    return false;
}

// 忽略动态实体走一条软路径，用来判断参考动作大概被哪个箱子挡住
static bool build_soft_grid_path(const SokobanLevel& level, point start, point end, StaticArray<point, MAX_PATH_LENGTH>& out_path) {
    SokobanLevel soft = level;
    soft.box_count = 0;
    soft.bomb_count = 0;
    return PlanningCommon::get_grid_time_path(soft, start, end, out_path);
}

// 找出软路径上遇到的第一个箱子，作为参考位清障的目标
static int first_box_on_path(const SokobanLevel& level, const StaticArray<point, MAX_PATH_LENGTH>& path) {
    for (int i = 0; i < path.size(); ++i) {
        for (int b = 0; b < level.box_count; ++b) {
            if (level.boxes[b] == path[i]) return b;
        }
    }
    return -1;
}

// 计算相邻两格的移动方向，非相邻时返回 -1
static int first_step_direction(point from, point to) {
    point delta = to - from;
    for (int d = 0; d < 4; ++d) {
        if (delta == MOVE[d]) return d;
    }
    return -1;
}

// 粗略估计某箱子到目标集合的静态可达距离，用于完成式推箱的压力奖励
static int best_box_to_target_distance(const SokobanLevel& level, uint8_t box_id, uint16_t target_mask) {
    if (box_id >= level.box_count || target_mask == 0) return kInfScore;
    int best = kInfScore;
    for (int t = 0; t < level.target_count; ++t) {
        if (!(target_mask & (1U << t))) continue;
        uint16_t d = PlanningCommon::shortest_grid_time_cost(level, level.boxes[box_id], level.targets[t]);
        if (d != 65535 && d < best) best = d;
    }
    return best;
}

// 小车不能站在箱子格上，因此用箱子四邻域可达距离估算“离箱子近”
static int player_to_box_access_distance(const SokobanLevel& level, point player, point box) {
    int best = kInfScore;
    for (int d = 0; d < 4; ++d) {
        point stand = box - MOVE[d];
        if (!PlanningCommon::in_bounds(stand)) continue;
        if (level.map[stand.y][stand.x] == 1) continue;
        if (has_dynamic_entity(level, stand)) continue;

        uint16_t dist = PlanningCommon::shortest_grid_time_cost(level, player, stand);
        if (dist != 65535 && dist < best) best = dist;
    }
    if (best == kInfScore) best = manhattan(player, box) + 8;
    return best;
}

// 参考动作的接入点：观测看观测位，炸弹看炸弹初始位，推箱看箱子初始位
static point action_anchor(const MacroAction& action) {
    if (action.kind == MacroActionKind::OBSERVE) return action.observe.view.pos;
    if (action.kind == MacroActionKind::PUSH_BOMB) return action.bomb_push.bomb_start;
    return action.box_push.box_start;
}

// 将相邻移动转换成 MOVE 下标，供炸弹路径状态回放使用
static int move_dir_index(point from, point to) {
    point delta = to - from;
    for (int d = 0; d < 4; ++d) {
        if (delta == MOVE[d]) return d;
    }
    return -1;
}

// 回放前 order 个路径点，推算炸弹在中途观测分裂点的位置
static bool bomb_state_after_path_order(point player_start,
                                        point bomb_start,
                                        const StaticArray<point, MAX_PATH_LENGTH>& path,
                                        int order,
                                        point& out_bomb_pos,
                                        bool& out_bomb_moved) {
    point car = player_start;
    out_bomb_pos = bomb_start;
    out_bomb_moved = false;

    for (int i = 0; i < order && i < path.size(); ++i) {
        point next_car = path[i];
        if (next_car == out_bomb_pos) {
            int dir = move_dir_index(car, next_car);
            if (dir < 0) return false;
            out_bomb_pos = out_bomb_pos + MOVE[dir];
            out_bomb_moved = true;
        }
        car = next_car;
    }
    return true;
}

// 构造推炸弹分段动作，保留原始炸弹动作的策略属性
static BombPushAction make_bomb_push_segment(point bomb_start,
                                            point bomb_target,
                                            point blast_wall,
                                            bool detonates,
                                            const BombPushAction& source) {
    BombPushAction action{};
    action.bomb_start = bomb_start;
    action.bomb_target = bomb_target;
    action.blast_wall = blast_wall;
    action.detonates = detonates;
    action.is_essential = source.is_essential;
    action.net_profit = source.net_profit;
    return action;
}

// ============================================================================
// 模块 3：生命周期
// ============================================================================

uint16_t MacroPlanner::default_candidate_mask() const {
    uint16_t mask = 0;
    for (int t = 0; t < target_count; ++t) mask |= (1U << t);
    return mask;
}

/// \brief 重置 MacroPlanner 的在线状态
/// \param level 当前逻辑地图
void MacroPlanner::reset(const SokobanLevel& level) {
    knowledge_state.observed_mask = 0;
    reference_plan.clear();
    pending_actions.clear();
    reference_cursor = 0;
    pending_cursor = 0;
    box_count = level.box_count;
    target_count = level.target_count;

    for (int i = 0; i < MAX_ENTITIES; ++i) semantic_labels[i] = -1;
    for (int i = 0; i < MAX_ENTITIES; ++i) knowledge_state.inferred_semantics[i] = -1;
    knowledge_state.semantics_ready = false;
    knowledge_state.needs_supplemental_observe = false;
    for (int i = 0; i < MAX_BOXES; ++i) {
        knowledge_state.candidate_targets[i] = default_candidate_mask();
        knowledge_state.bound_target[i] = 0;
        knowledge_state.is_bound[i] = false;
    }
}

/// \brief 装入 Exploration 生成的参考序列
/// \param plan 离线巡图参考动作序列
void MacroPlanner::set_reference_plan(const StaticArray<MacroAction, 32>& plan) {
    reference_plan = plan;
    reference_cursor = 0;
    pending_actions.clear();
    pending_cursor = 0;
}

// ============================================================================
// 模块 4：语义知识维护
// ============================================================================

bool MacroPlanner::has_required_observations(const SokobanLevel& level) const {
    int seen_boxes = 0;
    int seen_targets = 0;
    for (int b = 0; b < level.box_count; ++b) {
        if (knowledge_state.observed_mask & (1UL << b)) ++seen_boxes;
    }
    for (int t = 0; t < level.target_count; ++t) {
        if (knowledge_state.observed_mask & (1UL << (level.box_count + t))) ++seen_targets;
    }

    // Macro 阶段只要求 n-1 个箱子和 n-1 个目标点被观测，最后一个由语义配对兜底补全
    int req_boxes = level.box_count - 1;
    int req_targets = level.target_count - 1;
    if (req_boxes < 0) req_boxes = 0;
    if (req_targets < 0) req_targets = 0;
    return seen_boxes >= req_boxes && seen_targets >= req_targets;
}

// 检查参考序列中剩余动作是否都已完成或已不再需要执行
bool MacroPlanner::reference_sequence_done(const SokobanLevel& level) const {
    int cursor = reference_cursor;
    while (cursor < reference_plan.size()) {
        MacroAction action = reference_plan[cursor];
        if (action.kind == MacroActionKind::OBSERVE) {
            if ((action.observe.active_mask & ~knowledge_state.observed_mask) == 0) {
                ++cursor;
                continue;
            }
            break;
        }

        if (action.kind == MacroActionKind::PUSH_BOMB) {
            if (bomb_still_present(level, macro_bomb_task(action))) break;
            ++cursor;
            continue;
        }

        if (action.kind == MacroActionKind::PUSH_BOX) {
            bool box_at_start = false;
            bool box_at_target = false;
            for (int b = 0; b < level.box_count; ++b) {
                if (level.boxes[b] == action.box_push.box_start) box_at_start = true;
                if (level.boxes[b] == action.box_push.box_target) box_at_target = true;
            }
            if (box_at_start && !box_at_target) break;
            ++cursor;
            continue;
        }

        ++cursor;
    }
    return cursor >= reference_plan.size();
}

/// \brief 根据观测标签推断完整实体语义
/// \param level 当前逻辑地图
/// \param labels 视觉输出的语义标签，前 box_count 项为箱子，后 target_count 项为目标点
/// \param out_labels 输出完整语义标签，允许同一侧存在重复语义
/// \return 语义推断状态
///
/// \details
/// Macro 现在只推断语义，不把语义直接等价成“箱子绑定某个固定目标点”。
/// 已知标签必须满足每个语义在箱子侧和目标侧最终数量一致。若只剩一箱一目标未知，
/// 则优先用两侧计数缺口推断最后语义；如果缺口为空，说明最后一组可能是任意重复语义，
/// 这时需要补充式观测。补充式观测关闭时，才把这一组补成 0-9 中未出现的新语义。
MacroPlanner::SemanticInferenceStatus MacroPlanner::infer_semantics(const SokobanLevel& level,
                                                                    const int8_t* labels,
                                                                    int8_t* out_labels,
                                                                    bool allow_supplemental_observe) const {
    if (!labels || !out_labels) return SemanticInferenceStatus::INVALID;
    if (level.box_count != level.target_count) return SemanticInferenceStatus::INVALID;

    const int entity_count = level.box_count + level.target_count;
    for (int i = 0; i < entity_count; ++i) out_labels[i] = labels[i];

    int box_counts[10] = {0};
    int target_counts[10] = {0};
    int hidden_box = -1;
    int hidden_target = -1;
    int hidden_box_count = 0;
    int hidden_target_count = 0;

    for (int b = 0; b < level.box_count; ++b) {
        int8_t label = labels[b];
        if (label == -1) {
            hidden_box = b;
            ++hidden_box_count;
            continue;
        }
        if (!is_valid_semantic_id(label)) return SemanticInferenceStatus::INVALID;
        ++box_counts[label];
    }

    for (int t = 0; t < level.target_count; ++t) {
        int8_t label = labels[level.box_count + t];
        if (label == -1) {
            hidden_target = t;
            ++hidden_target_count;
            continue;
        }
        if (!is_valid_semantic_id(label)) return SemanticInferenceStatus::INVALID;
        ++target_counts[label];
    }

    int req_boxes = level.box_count - 1;
    int req_targets = level.target_count - 1;
    if (req_boxes < 0) req_boxes = 0;
    if (req_targets < 0) req_targets = 0;
    if (level.box_count - hidden_box_count < req_boxes ||
        level.target_count - hidden_target_count < req_targets) {
        return SemanticInferenceStatus::INSUFFICIENT;
    }

    if (hidden_box_count > 1 || hidden_target_count > 1) return SemanticInferenceStatus::INSUFFICIENT;

    int box_needs[10] = {0};
    int target_needs[10] = {0};
    int box_need_total = 0;
    int target_need_total = 0;
    int box_need_sem = -1;
    int target_need_sem = -1;

    for (int s = 0; s < 10; ++s) {
        if (box_counts[s] > target_counts[s]) {
            target_needs[s] = box_counts[s] - target_counts[s];
            target_need_total += target_needs[s];
            if (target_needs[s] == 1) target_need_sem = s;
        } else if (target_counts[s] > box_counts[s]) {
            box_needs[s] = target_counts[s] - box_counts[s];
            box_need_total += box_needs[s];
            if (box_needs[s] == 1) box_need_sem = s;
        }
    }

    if (box_need_total > hidden_box_count || target_need_total > hidden_target_count) {
        return SemanticInferenceStatus::INVALID;
    }

    if (hidden_box_count == 0 && hidden_target_count == 0) {
        return (box_need_total == 0 && target_need_total == 0)
            ? SemanticInferenceStatus::INFERRED
            : SemanticInferenceStatus::INVALID;
    }

    if (hidden_box_count == 1 && hidden_target_count == 0) {
        if (box_need_total != 1 || target_need_total != 0 || box_need_sem < 0) {
            return SemanticInferenceStatus::INVALID;
        }
        out_labels[hidden_box] = static_cast<int8_t>(box_need_sem);
        return SemanticInferenceStatus::INFERRED;
    }

    if (hidden_box_count == 0 && hidden_target_count == 1) {
        if (target_need_total != 1 || box_need_total != 0 || target_need_sem < 0) {
            return SemanticInferenceStatus::INVALID;
        }
        out_labels[level.box_count + hidden_target] = static_cast<int8_t>(target_need_sem);
        return SemanticInferenceStatus::INFERRED;
    }

    if (box_need_total == 1 && target_need_total == 1 &&
        box_need_sem >= 0 && target_need_sem >= 0) {
        out_labels[hidden_box] = static_cast<int8_t>(box_need_sem);
        out_labels[level.box_count + hidden_target] = static_cast<int8_t>(target_need_sem);
        return SemanticInferenceStatus::INFERRED;
    }

    if (box_need_total == 0 && target_need_total == 0) {
        if constexpr (MacroConfig::ENABLE_SUPPLEMENTAL_OBSERVE_SLOT) {
            if (!allow_supplemental_observe) {
                int fallback_sem = first_unused_semantic_id(box_counts, target_counts);
                out_labels[hidden_box] = static_cast<int8_t>(fallback_sem);
                out_labels[level.box_count + hidden_target] = static_cast<int8_t>(fallback_sem);
                return SemanticInferenceStatus::INFERRED;
            }
            return SemanticInferenceStatus::NEED_SUPPLEMENTAL;
        }

        int fallback_sem = first_unused_semantic_id(box_counts, target_counts);
        out_labels[hidden_box] = static_cast<int8_t>(fallback_sem);
        out_labels[level.box_count + hidden_target] = static_cast<int8_t>(fallback_sem);
        return SemanticInferenceStatus::INFERRED;
    }

    return SemanticInferenceStatus::INVALID;
}

// 将推断出的完整语义写回 Macro 状态，并同步到 semantic_labels。
// 这样后续 sync_semantics 再次运行时，不会把补充观测失败后的回退结果冲掉。
bool MacroPlanner::apply_inferred_semantics(const SokobanLevel& level, const int8_t* inferred_labels) {
    if (!inferred_labels) return false;

    for (int i = 0; i < MAX_ENTITIES; ++i) knowledge_state.inferred_semantics[i] = -1;
    for (int b = 0; b < box_count; ++b) {
        knowledge_state.candidate_targets[b] = default_candidate_mask();
        knowledge_state.bound_target[b] = 0;
        knowledge_state.is_bound[b] = false;
    }

    const int entity_count = level.box_count + level.target_count;
    for (int i = 0; i < entity_count; ++i) {
        if (!is_valid_semantic_id(inferred_labels[i])) return false;
        knowledge_state.inferred_semantics[i] = inferred_labels[i];
        semantic_labels[i] = inferred_labels[i];
    }

    knowledge_state.semantics_ready = true;
    knowledge_state.needs_supplemental_observe = false;

    // 完成式推箱若重新打开，需要一份临时自然顺序目标；最终 Sokoban 只看语义组
    bool used_targets[MAX_BOXES] = {false};
    for (int b = 0; b < box_count; ++b) {
        int chosen = -1;
        int8_t box_sem = knowledge_state.inferred_semantics[b];
        for (int t = 0; t < target_count; ++t) {
            int8_t target_sem = knowledge_state.inferred_semantics[box_count + t];
            if (used_targets[t] || target_sem != box_sem) continue;
            chosen = t;
            break;
        }
        if (chosen < 0) return false;

        used_targets[chosen] = true;
        knowledge_state.candidate_targets[b] = (1U << chosen);
        knowledge_state.bound_target[b] = static_cast<uint8_t>(chosen);
        knowledge_state.is_bound[b] = true;
    }

    return true;
}

bool MacroPlanner::force_supplemental_fallback(const SokobanLevel& level) {
    if (!needs_supplemental_observation(level)) return false;

    SokobanLevel semantic_level{};
    semantic_level.box_count = box_count;
    semantic_level.target_count = target_count;

    int8_t inferred[MAX_ENTITIES];
    SemanticInferenceStatus status = infer_semantics(semantic_level, semantic_labels, inferred, false);
    if (status != SemanticInferenceStatus::INFERRED) return false;

    // 补充观测点不可达时，退回到“补充观测关闭”的语义策略：
    // 给最后一组未观测箱子/目标点分配一个前面没有出现过的新语义。
    return apply_inferred_semantics(semantic_level, inferred);
}

/// \brief 写入一次观测结果
/// \param level 当前逻辑地图
/// \param mask 本次观测覆盖的实体掩码
void MacroPlanner::apply_observation(const SokobanLevel& level, uint32_t mask) {
    (void)level;
    knowledge_state.observed_mask |= mask;
}

/// \brief 同步视觉语义池并更新内部配对状态
/// \param labels ART2 语义标签数组，-1 表示未知
///
/// \details
/// 函数先同步 vision 语义池，再按“同语义数量相等”的规则做实体语义推断。
/// 语义完整后只额外生成完成式推箱候选位需要的临时自然顺序映射。
void MacroPlanner::sync_semantics(const int8_t* labels) {
    if (!labels) return;
    for (int i = 0; i < MAX_ENTITIES; ++i) {
        if (labels[i] != -1) semantic_labels[i] = labels[i];
    }

    knowledge_state.semantics_ready = false;
    knowledge_state.needs_supplemental_observe = false;
    for (int i = 0; i < MAX_ENTITIES; ++i) knowledge_state.inferred_semantics[i] = -1;
    for (int b = 0; b < box_count; ++b) {
        knowledge_state.candidate_targets[b] = default_candidate_mask();
        knowledge_state.bound_target[b] = 0;
        knowledge_state.is_bound[b] = false;
    }

    SokobanLevel semantic_level{};
    semantic_level.box_count = box_count;
    semantic_level.target_count = target_count;

    int8_t inferred[MAX_ENTITIES];
    SemanticInferenceStatus status = infer_semantics(semantic_level, semantic_labels, inferred);
    if (status == SemanticInferenceStatus::NEED_SUPPLEMENTAL) {
        knowledge_state.needs_supplemental_observe = true;
        return;
    }
    if (status != SemanticInferenceStatus::INFERRED) return;
    apply_inferred_semantics(semantic_level, inferred);
}

/// \brief 判断 Macro 阶段是否可以交给 Sokoban
/// \param level 当前逻辑地图
/// \return 参考序列完成且已达到 N-1 观测要求时返回 true
bool MacroPlanner::ready_for_sokoban(const SokobanLevel& level) const {
    if (pending_cursor < pending_actions.size()) return false;
    if (!reference_sequence_done(level)) return false;
    if (!has_required_observations(level)) return false;
    if (needs_supplemental_observation(level)) return false;

    // 语义是否完整交给 BIND_SEMANTICS 阶段判定
    // 否则最后一个观测点完成后若绑定尚未收敛，会继续请求下一条宏动作并误报 error1
    return true;
}

/// \brief 将 Macro 推断出的完整实体语义写回关卡
/// \param level 当前逻辑地图
/// \return 语义推断完整时返回 true
bool MacroPlanner::apply_semantics_to_level(SokobanLevel& level) const {
    if (!semantics_ready()) return false;
    if (level.box_count != box_count || level.target_count != target_count) return false;

    for (int b = 0; b < box_count; ++b) {
        int8_t sem = knowledge_state.inferred_semantics[b];
        if (!is_valid_semantic_id(sem)) return false;
        level.box_semantics[b] = static_cast<uint8_t>(sem);
    }
    for (int t = 0; t < target_count; ++t) {
        int8_t sem = knowledge_state.inferred_semantics[box_count + t];
        if (!is_valid_semantic_id(sem)) return false;
        level.target_semantics[t] = static_cast<uint8_t>(sem);
    }
    return true;
}

bool MacroPlanner::semantics_ready() const {
    return knowledge_state.semantics_ready;
}

bool MacroPlanner::needs_supplemental_observation(const SokobanLevel& level) const {
    return reference_sequence_done(level) &&
           has_required_observations(level) &&
           knowledge_state.needs_supplemental_observe &&
           !knowledge_state.semantics_ready;
}

// ============================================================================
// 模块 5：参考主线推进
// ============================================================================

// 刷新观测动作在当前地图下的可见掩码，剔除已经观测过的实体
bool MacroPlanner::refresh_observe_action(const SokobanLevel& level, MacroAction& action) const {
    if (action.kind != MacroActionKind::OBSERVE) return true;

    auto views = patrol_planner.build_current_views(level);
    for (int i = 0; i < views.size(); ++i) {
        if (views[i].pos == action.observe.view.pos && views[i].target_yaw == action.observe.view.target_yaw) {
            action.observe.view = views[i];
            action.observe.active_mask = views[i].mask[0] & ~knowledge_state.observed_mask;
            return true;
        }
    }

    // Exploration 输出的参考观测属于主线任务，Macro 不能按新增 mask 多少跳过它
    // 如果观测位路径不可达，prepare_reference_action 会失败并触发清障
    action.observe.active_mask = action.observe.view.mask[0] & ~knowledge_state.observed_mask;
    return true;
}

// 验证参考动作在当前地图和当前位置下是否可执行，并刷新真实代价
bool MacroPlanner::prepare_reference_action(const MacroPlanContext& ctx, const MacroAction& raw_action, MacroAction& prepared_action) const {
    prepared_action = raw_action;

    if (prepared_action.kind == MacroActionKind::OBSERVE) {
        if (!refresh_observe_action(ctx.level, prepared_action)) return false;
        StaticArray<point, MAX_PATH_LENGTH> path;
        if (!PlanningCommon::get_grid_time_path(ctx.level, ctx.player, prepared_action.observe.view.pos, path)) return false;
        prepared_action.real_cost =
            PlanningCommon::path_time_cost(ctx.player, path) +
            PlanningCommon::yaw_turn_time_cost(ctx.yaw, prepared_action.observe.view.target_yaw);
        return true;
    }

    if (prepared_action.kind == MacroActionKind::PUSH_BOX) {
        StaticArray<point, MAX_PATH_LENGTH> path;
        SokobanLevel probe = ctx.level;
        point probe_player = ctx.player;
        if (!PlanningCommon::append_box_push_path(probe, probe_player, macro_box_task(prepared_action), path)) return false;
        prepared_action.real_cost = PlanningCommon::path_time_cost(ctx.player, path);
        return true;
    }

    if (prepared_action.kind == MacroActionKind::PUSH_BOMB) {
        BombTask bomb = macro_bomb_task(prepared_action);
        if (!bomb_still_present(ctx.level, bomb)) return false;

        StaticArray<point, MAX_PATH_LENGTH> path;
        if (!PlanningCommon::get_bomb_push_path(ctx.level, ctx.player, bomb, path)) return false;
        prepared_action.real_cost = PlanningCommon::path_time_cost(ctx.player, path);
        return true;
    }

    return false;
}

/// \brief 将推炸弹参考动作拆成“推到中途、观测、继续引爆”
/// \param ctx 当前在线调度上下文
/// \param bomb_action 原始推炸弹参考动作
/// \param prefix_action 输出前半段推炸弹动作
/// \param observe_action 输出中途观测动作
/// \param suffix_action 输出后半段推炸弹动作
/// \return 找到顺路观测分裂点时返回 true
///
/// \details
/// 该逻辑只在炸弹路径经过后续参考观测位时触发，
/// 用于减少“先推炸弹再回头观测”的来回路程。
bool MacroPlanner::build_bomb_observe_split(const MacroPlanContext& ctx,
                                            const MacroAction& bomb_action,
                                            MacroAction& prefix_action,
                                            MacroAction& observe_action,
                                            MacroAction& suffix_action) const {
    if (bomb_action.kind != MacroActionKind::PUSH_BOMB || !bomb_action.bomb_push.detonates) return false;

    BombTask full_task = macro_bomb_task(bomb_action);
    StaticArray<point, MAX_PATH_LENGTH> full_path;
    if (!PlanningCommon::get_bomb_push_path(ctx.level, ctx.player, full_task, full_path)) return false;

    for (int look = reference_cursor + 1; look < reference_plan.size(); ++look) {
        const MacroAction& raw_observe = reference_plan[look];
        if (raw_observe.kind != MacroActionKind::OBSERVE) break;
        uint32_t raw_newly_seen = raw_observe.observe.active_mask & ~knowledge_state.observed_mask;
        if (raw_newly_seen == 0) continue;

        for (int order = 1; order <= full_path.size(); ++order) {
            if (full_path[order - 1] != raw_observe.observe.view.pos) continue;

            point pause_bomb_pos;
            bool bomb_moved = false;
            if (!bomb_state_after_path_order(ctx.player,
                                             full_task.bomb_start,
                                             full_path,
                                             order,
                                             pause_bomb_pos,
                                             bomb_moved)) {
                continue;
            }
            if (!bomb_moved || pause_bomb_pos == full_task.target_wall) continue;

            BombPushAction prefix_push = make_bomb_push_segment(
                full_task.bomb_start,
                pause_bomb_pos,
                full_task.target_wall,
                false,
                bomb_action.bomb_push
            );
            MacroAction prefix = make_bomb_push_macro_action(prefix_push);

            StaticArray<point, MAX_PATH_LENGTH> prefix_path;
            if (!PlanningCommon::get_bomb_push_path(ctx.level, ctx.player, macro_bomb_task(prefix), prefix_path)) continue;
            point prefix_end = prefix_path.empty() ? ctx.player : prefix_path.back();

            SokobanLevel pause_level = ctx.level;
            PlanningCommon::apply_bomb_push_action_effect(pause_level, prefix_push);

            MacroAction observe = raw_observe;
            MacroAction refreshed_observe = observe;
            refresh_observe_action(pause_level, refreshed_observe);
            if (refreshed_observe.observe.active_mask != 0) {
                observe = refreshed_observe;
                observe.observe.active_mask |= raw_newly_seen;
            } else {
                observe.observe.active_mask = raw_newly_seen;
            }

            StaticArray<point, MAX_PATH_LENGTH> observe_path;
            if (!PlanningCommon::get_grid_time_path(pause_level, prefix_end, observe.observe.view.pos, observe_path)) continue;

            BombPushAction suffix_push = make_bomb_push_segment(
                pause_bomb_pos,
                full_task.target_wall,
                full_task.target_wall,
                true,
                bomb_action.bomb_push
            );
            MacroAction suffix = make_bomb_push_macro_action(suffix_push);

            StaticArray<point, MAX_PATH_LENGTH> suffix_path;
            if (!PlanningCommon::get_bomb_push_path(pause_level, observe.observe.view.pos, macro_bomb_task(suffix), suffix_path)) continue;

            prefix.real_cost = PlanningCommon::path_time_cost(ctx.player, prefix_path);
            observe.real_cost =
                PlanningCommon::path_time_cost(prefix_end, observe_path) +
                PlanningCommon::yaw_turn_time_cost(ctx.yaw, observe.observe.view.target_yaw);
            suffix.real_cost = PlanningCommon::path_time_cost(observe.observe.view.pos, suffix_path);

            prefix_action = prefix;
            observe_action = observe;
            suffix_action = suffix;
            return true;
        }
    }

    return false;
}

// 跳过已经完成的参考动作，把 reference_cursor 推进到下一条待处理动作
void MacroPlanner::advance_reference_cursor(const SokobanLevel& level) {
    while (reference_cursor < reference_plan.size()) {
        MacroAction action = reference_plan[reference_cursor];
        if (action.kind == MacroActionKind::OBSERVE) {
            if ((action.observe.active_mask & ~knowledge_state.observed_mask) == 0) {
                ++reference_cursor;
                continue;
            }
            break;
        }

        if (action.kind == MacroActionKind::PUSH_BOMB) {
            if (bomb_still_present(level, macro_bomb_task(action))) break;
            ++reference_cursor;
            continue;
        }

        if (action.kind == MacroActionKind::PUSH_BOX) {
            bool box_at_start = false;
            bool box_at_target = false;
            for (int b = 0; b < level.box_count; ++b) {
                if (level.boxes[b] == action.box_push.box_start) box_at_start = true;
                if (level.boxes[b] == action.box_push.box_target) box_at_target = true;
            }
            if (box_at_start && !box_at_target) break;
            ++reference_cursor;
            continue;
        }

        ++reference_cursor;
    }
}

/// \brief 为当前不可达参考动作构造短推清障动作
/// \param ctx 当前在线调度上下文
/// \param reference_action 当前无法直接执行的参考动作
/// \param slot 输出清障候选位
/// \return 找到可执行清障动作时返回 true
///
/// \details
/// 优先沿用 Strategy materialize_bomb_task 已验证的移障序列；
/// 若没有可用序列，则根据忽略动态物体的软路径找第一个挡路箱子，
/// 尝试把它短推离参考路径。
bool MacroPlanner::build_reference_clearance(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const {
    SlotCandidate best;
    best.score = -kInfScore;

    auto consider_box_push = [&](uint8_t box_id, point target, int bonus) {
        if (box_id >= ctx.level.box_count) return;
        if (!PlanningCommon::in_bounds(target)) return;
        if (ctx.level.map[target.y][target.x] == 1) return;
        if (has_dynamic_entity(ctx.level, target, box_id)) return;

        BoxPushTask task{ctx.level.boxes[box_id], target};
        SokobanLevel probe = ctx.level;
        point probe_player = ctx.player;
        StaticArray<point, MAX_PATH_LENGTH> path;
        if (!PlanningCommon::append_box_push_path(probe, probe_player, task, path)) return;
        if (path.size() > MacroConfig::REFERENCE_CLEAR_MAX_PATH) return;
        uint16_t push_cost = PlanningCommon::path_time_cost(ctx.player, path);

        uint16_t safe_mask = default_candidate_mask();
        if (box_id < box_count && knowledge_state.is_bound[box_id]) {
            safe_mask = (1U << knowledge_state.bound_target[box_id]);
        }
        if (!PlanningCommon::is_box_position_safe(probe, box_id, safe_mask)) return;

        int after_access = reference_access_cost(probe, probe_player, ctx.yaw, reference_action);
        if (after_access >= kInfScore) return;

        // 清障评分只回答一个问题：短推后参考动作是否恢复可执行且代价可接受
        int score = bonus - static_cast<int>(push_cost) - after_access * 2;
        if (!best.valid || score > best.score) {
            MacroAction action = make_box_push_macro_action(task, box_id, push_cost);

            best.action = action;
            best.slot = SlotKind::REFERENCE;
            best.score = score;
            best.valid = true;
            best.consumes_reference = false;
        }
    };

    StaticArray<point, MAX_PATH_LENGTH> soft_path;
    point anchor = action_anchor(reference_action);
    int blocking_box = -1;
    int blocked_path_dir = -1;
    if (build_soft_grid_path(ctx.level, ctx.player, anchor, soft_path)) {
        blocking_box = first_box_on_path(ctx.level, soft_path);
        if (blocking_box >= 0) {
            for (int i = 0; i + 1 < soft_path.size(); ++i) {
                if (soft_path[i] == ctx.level.boxes[blocking_box]) {
                    blocked_path_dir = first_step_direction(soft_path[i], soft_path[i + 1]);
                    break;
                }
            }
        }
    }

    auto find_box_id = [&](point box_pos) -> int {
        for (int b = 0; b < ctx.level.box_count; ++b) {
            if (ctx.level.boxes[b] == box_pos) return b;
        }
        return -1;
    };

    auto consider_materialized_sequence = [&](const BombTask& materialized, int bonus) {
        if (materialized.box_pushes.empty()) return;

        const BoxPushTask& first_task = materialized.box_pushes[0];
        int box_id = find_box_id(first_task.box_start);
        if (box_id < 0) return;

        SokobanLevel first_probe = ctx.level;
        point first_player = ctx.player;
        StaticArray<point, MAX_PATH_LENGTH> first_path;
        if (!PlanningCommon::append_box_push_path(first_probe, first_player, first_task, first_path)) return;
        if (first_path.size() > MacroConfig::REFERENCE_CLEAR_MAX_PATH) return;

        SokobanLevel seq_probe = first_probe;
        point seq_player = first_player;
        StaticArray<point, MAX_PATH_LENGTH> rest_path;
        for (int i = 1; i < materialized.box_pushes.size(); ++i) {
            if (!PlanningCommon::append_box_push_path(seq_probe, seq_player, materialized.box_pushes[i], rest_path)) return;
        }

        int after_access = reference_access_cost(seq_probe, seq_player, ctx.yaw, reference_action);
        if (after_access >= kInfScore) return;

        uint16_t push_cost = PlanningCommon::path_time_cost(ctx.player, first_path);
        int score = bonus - static_cast<int>(push_cost) - after_access;
        if (!best.valid || score > best.score) {
            MacroAction action = make_box_push_macro_action(
                first_task,
                static_cast<uint8_t>(box_id),
                push_cost
            );

            best.action = action;
            best.slot = SlotKind::REFERENCE;
            best.score = score;
            best.valid = true;
            best.consumes_reference = false;
        }
    };

    // 如果参考动作被箱子挡住，优先沿用 Strategy 已验证过的移障序列。
    if (reference_action.kind == MacroActionKind::PUSH_BOMB) {
        BombTask materialized;
        if (strategic_planner.materialize_bomb_task(ctx.level, ctx.player, macro_bomb_task(reference_action), materialized)) {
            consider_materialized_sequence(materialized, MacroConfig::REFERENCE_MATERIALIZED_CLEAR_BONUS);
        }
    } else if (ctx.bomb_tasks) {
        for (int i = 0; i < ctx.bomb_tasks->size(); ++i) {
            const BombTask& bomb = (*ctx.bomb_tasks)[i];
            if (!bomb_still_present(ctx.level, bomb)) continue;
            BombTask materialized;
            if (strategic_planner.materialize_bomb_task(ctx.level, ctx.player, bomb, materialized)) {
                consider_materialized_sequence(materialized, MacroConfig::REFERENCE_MATERIALIZED_CLEAR_BONUS);
            }
        }
    }

    // 通用兜底：只把挡路箱子推离参考软路径，避免继续顺着通道往前顶导致死局。
    if (blocking_box >= 0) {
        for (int d = 0; d < 4; ++d) {
            if (d == blocked_path_dir) continue;
            for (int step = 1; step <= MacroConfig::REFERENCE_CLEAR_MAX_STEPS; ++step) {
                point target{
                    static_cast<int8_t>(ctx.level.boxes[blocking_box].x + MOVE[d].x * step),
                    static_cast<int8_t>(ctx.level.boxes[blocking_box].y + MOVE[d].y * step)
                };
                consider_box_push(static_cast<uint8_t>(blocking_box), target, 600);
            }
        }
    }

    if (!best.valid) return false;
    slot = best;
    return true;
}

/// \brief 构造当前参考候选位
/// \param ctx 当前在线调度上下文
/// \param slot 输出参考候选位
/// \return 成功得到参考动作或清障动作时返回 true
bool MacroPlanner::build_reference_slot(const MacroPlanContext& ctx, SlotCandidate& slot) {
    advance_reference_cursor(ctx.level);

    while (reference_cursor < reference_plan.size()) {
        MacroAction prepared;
        const MacroAction& raw = reference_plan[reference_cursor];
        if (prepare_reference_action(ctx, raw, prepared)) {
            if (prepared.kind == MacroActionKind::PUSH_BOMB) {
                MacroAction prefix;
                MacroAction observe;
                MacroAction suffix;
                if (build_bomb_observe_split(ctx, prepared, prefix, observe, suffix)) {
                    slot.action = prefix;
                    slot.followups.clear();
                    slot.followups.push_back(observe);
                    slot.followups.push_back(suffix);
                    slot.slot = SlotKind::REFERENCE;
                    slot.score = 0;
                    slot.valid = true;
                    slot.consumes_reference = true;
                    return true;
                }
            }

            slot.action = prepared;
            slot.slot = SlotKind::REFERENCE;
            slot.score = 0;
            slot.valid = true;
            slot.consumes_reference = true;
            return true;
        }

        if (raw.kind == MacroActionKind::PUSH_BOMB || raw.kind == MacroActionKind::OBSERVE) {
            if (build_reference_clearance(ctx, raw, slot)) return true;
        }

        if (raw.kind == MacroActionKind::PUSH_BOMB && !bomb_still_present(ctx.level, macro_bomb_task(raw))) {
            ++reference_cursor;
            continue;
        }

        // 参考观测和仍存在的必推炸弹不能被 Macro 擅自跳过
        return false;
    }

    return false;
}

// ============================================================================
// 模块 6：动作验证与评分辅助
// ============================================================================

// 在临时地图上执行一个宏动作，供候选动作评分使用
bool MacroPlanner::simulate_action(const SokobanLevel& level, point player, uint32_t observed_mask, const MacroAction& action,
                                   SokobanLevel& out_level, point& out_player, uint32_t& out_observed, int& out_cost) const {
    out_level = level;
    out_player = player;
    out_observed = observed_mask;
    out_cost = 0;

    StaticArray<point, MAX_PATH_LENGTH> path;
    if (action.kind == MacroActionKind::OBSERVE) {
        if (!PlanningCommon::get_grid_time_path(out_level, out_player, action.observe.view.pos, path)) return false;
        out_cost = PlanningCommon::path_time_cost(out_player, path);
        out_player = action.observe.view.pos;
        out_observed |= action.observe.active_mask;
        return true;
    }

    if (action.kind == MacroActionKind::PUSH_BOX) {
        if (!PlanningCommon::append_box_push_path(out_level, out_player, macro_box_task(action), path)) return false;
        out_cost = PlanningCommon::path_time_cost(player, path);
        return true;
    }

    if (action.kind == MacroActionKind::PUSH_BOMB) {
        BombTask bomb = macro_bomb_task(action);
        if (!PlanningCommon::get_bomb_push_path(out_level, out_player, bomb, path)) return false;
        out_cost = PlanningCommon::path_time_cost(player, path);
        if (!path.empty()) out_player = path.back();
        PlanningCommon::apply_bomb_push_action_effect(out_level, action.bomb_push);
        return true;
    }

    return false;
}

// 估计从当前位置接入参考动作的代价，不修改地图
int MacroPlanner::reference_access_cost(const SokobanLevel& level, point player, float observe_yaw, const MacroAction& reference_action) const {
    if (reference_action.kind == MacroActionKind::OBSERVE) {
        uint16_t d = PlanningCommon::shortest_grid_time_cost(level, player, reference_action.observe.view.pos);
        if (d == 65535) return kInfScore;
        return d + PlanningCommon::yaw_turn_time_cost(observe_yaw, reference_action.observe.view.target_yaw);
    }

    StaticArray<point, MAX_PATH_LENGTH> path;
    if (reference_action.kind == MacroActionKind::PUSH_BOMB) {
        if (!PlanningCommon::get_bomb_push_path(level, player, macro_bomb_task(reference_action), path)) return kInfScore;
        return PlanningCommon::path_time_cost(player, path);
    }

    if (reference_action.kind == MacroActionKind::PUSH_BOX) {
        SokobanLevel probe = level;
        point probe_player = player;
        if (!PlanningCommon::append_box_push_path(probe, probe_player, macro_box_task(reference_action), path)) return kInfScore;
        return PlanningCommon::path_time_cost(player, path);
    }

    return kInfScore;
}

// 估计提前完成某箱子后，未来少回到该箱子附近一次的粗略收益
int MacroPlanner::estimate_deferred_return_cost(const SokobanLevel& level, uint8_t box_id, uint8_t target_id) const {
    if (box_id >= level.box_count || target_id >= level.target_count) return 0;

    int best = kInfScore;
    for (int t = 0; t < level.target_count; ++t) {
        if (t == target_id) continue;
        uint16_t d = PlanningCommon::shortest_grid_time_cost(level, level.targets[t], level.boxes[box_id]);
        if (d != 65535 && d < best) best = d;
    }

    if (best == kInfScore) {
        best = manhattan(level.boxes[box_id], level.targets[target_id]);
    }

    return best;
}

// 验证某个箱子能否直接推到绑定目标点，并生成完成式推箱动作
bool MacroPlanner::validate_completion_push(const SokobanLevel& level, point player, uint8_t box_id, uint8_t target_id,
                                            MacroAction& action, SokobanLevel& after_level, point& after_player,
                                            int& push_path_cost) const {
    if (box_id >= level.box_count || target_id >= level.target_count) return false;
    if (level.boxes[box_id] == level.targets[target_id]) return false;

    BoxPushTask task{level.boxes[box_id], level.targets[target_id]};
    after_level = level;
    after_player = player;
    StaticArray<point, MAX_PATH_LENGTH> path;
    if (!PlanningCommon::append_box_push_path(after_level, after_player, task, path)) return false;
    if (path.size() > MacroConfig::COMPLETION_MAX_PUSH_PATH) return false;
    uint16_t push_cost = PlanningCommon::path_time_cost(player, path);

    uint16_t target_mask = (1U << target_id);
    if (!PlanningCommon::is_box_position_safe(after_level, box_id, target_mask)) return false;

    action = make_box_push_macro_action(task, box_id, push_cost);
    push_path_cost = push_cost;
    return true;
}

// ============================================================================
// 模块 7：插入候选槽位
// ============================================================================

/// \brief 构造完成式推箱候选位
/// \param ctx 当前在线调度上下文
/// \param reference_action 当前参考动作，用于评估推箱后是否更容易接回主线
/// \param slot 输出候选位
/// \return 候选评分超过阈值时返回 true
///
/// \details
/// 完成式推箱只考虑已完成语义绑定且箱子/目标均已观测的实体。
/// 评分不惩罚自身推箱路径，因为这部分动作第二阶段早晚也要做；
/// 重点评估接回参考主线收益、未来返程收益和降低 Sokoban 搜索压力。
bool MacroPlanner::build_completion_push_slot(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const {
    if constexpr (!MacroConfig::ENABLE_COMPLETION_PUSH_SLOT) {
        (void)ctx;
        (void)reference_action;
        (void)slot;
        return false;
    }

    SlotCandidate best;
    best.score = -kInfScore;
    int best_player_to_box = kInfScore;

    int before_ref_cost = reference_access_cost(ctx.level, ctx.player, ctx.yaw, reference_action);

    for (int b = 0; b < ctx.level.box_count; ++b) {
        if (!knowledge_state.is_bound[b]) continue;

        uint8_t target_id = knowledge_state.bound_target[b];
        if (target_id >= ctx.level.target_count) continue;

        uint32_t box_bit = (1UL << b);
        uint32_t target_bit = (1UL << (ctx.level.box_count + target_id));
        if (!(knowledge_state.observed_mask & box_bit)) continue;
        if (!(knowledge_state.observed_mask & target_bit)) continue;

        MacroAction action;
        SokobanLevel after_level;
        point after_player;
        int ignored_push_cost = 0;
        if (!validate_completion_push(ctx.level, ctx.player, static_cast<uint8_t>(b), target_id,
                                    action, after_level, after_player, ignored_push_cost)) {
            continue;
        }

        int after_ref_cost = reference_access_cost(after_level, after_player, ctx.yaw, reference_action);
        if (after_ref_cost >= kInfScore) continue;

        int follow_gain = 0;
        if (before_ref_cost >= kInfScore) follow_gain = 20;
        else follow_gain = before_ref_cost - after_ref_cost;

        int deferred_return = estimate_deferred_return_cost(ctx.level, static_cast<uint8_t>(b), target_id);
        int pressure_reward = MacroConfig::COMPLETION_PRESSURE_REWARD + ctx.level.box_count * 2;
        int player_to_box = player_to_box_access_distance(ctx.level, ctx.player, ctx.level.boxes[b]);

        // 完成式推箱不计自身推箱路径成本，只评估插入后的净收益
        // follow_gain：推完箱后当前位置接回下一参考动作是更近还是更远
        // deferred_return：提前完成后，未来少一次回到该箱子附近的粗略收益
        // pressure_reward：箱子落位会降低第二阶段 IDA* 的搜索压力
        // player_to_box：多对候选时优先处理离小车更近的箱子
        int score =
            follow_gain * MacroConfig::COMPLETION_FOLLOW_WEIGHT +
            deferred_return * MacroConfig::COMPLETION_RETURN_WEIGHT +
            pressure_reward -
            player_to_box * MacroConfig::COMPLETION_DISTANCE_WEIGHT;

        if (best_box_to_target_distance(ctx.level, static_cast<uint8_t>(b), (1U << target_id)) >= kInfScore) {
            score += MacroConfig::COMPLETION_PRESSURE_REWARD;
        }

        if (!best.valid || score > best.score || (score == best.score && player_to_box < best_player_to_box)) {
            best.action = action;
            best.slot = SlotKind::COMPLETION_PUSH;
            best.score = score;
            best.valid = true;
            best.consumes_reference = false;
            best_player_to_box = player_to_box;
        }
    }

    if (!best.valid || best.score < MacroConfig::COMPLETION_SCORE_THRESHOLD) return false;
    slot = best;
    return true;
}

// 构造接近式推箱候选位
bool MacroPlanner::build_approach_push_slot(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const {
    if constexpr (!MacroConfig::ENABLE_APPROACH_PUSH_SLOT) {
        (void)ctx;
        (void)reference_action;
        (void)slot;
        return false;
    }

    // 预留：后续只在已观测箱子上做短推，不改变参考主线
    (void)ctx;
    (void)reference_action;
    (void)slot;
    return false;
}

// 构造机会炸弹候选位
bool MacroPlanner::build_opportunistic_bomb_slot(const MacroPlanContext& ctx, const MacroAction& reference_action, SlotCandidate& slot) const {
    if constexpr (!MacroConfig::ENABLE_OPPORTUNISTIC_BOMB_SLOT) {
        (void)ctx;
        (void)reference_action;
        (void)slot;
        return false;
    }

    // 预留：后续只选择 Strategy 中未进入参考序列的近距离炸弹
    (void)ctx;
    (void)reference_action;
    (void)slot;
    return false;
}

/// \brief 构造补充式观测候选位
/// \param ctx 当前在线调度上下文
/// \param slot 输出补充观测动作
/// \return 找到可执行补充观测时返回 true
///
/// \details
/// 补充式观测只在参考序列已经结束、N-1 数量达标但最后一组语义仍无法唯一推断时启用。
/// 评分沿用“剩余那个近看哪个”的原则：优先距离和转向小的位姿；如果候选没有看到未知箱子，
/// 则加入非箱子惩罚，避免收尾时停在离箱子很远的目标点旁边。
bool MacroPlanner::build_supplemental_observe_slot(const MacroPlanContext& ctx, SlotCandidate& slot) const {
    if constexpr (!MacroConfig::ENABLE_SUPPLEMENTAL_OBSERVE_SLOT) {
        (void)ctx;
        (void)slot;
        return false;
    }

    if (!needs_supplemental_observation(ctx.level)) return false;

    StaticArray<ViewPose, MAX_OBS_POINTS> views = patrol_planner.build_current_views(ctx.level);
    SlotCandidate best;
    best.score = kInfScore;

    for (int i = 0; i < views.size(); ++i) {
        uint32_t newly_seen = views[i].mask[0] & ~knowledge_state.observed_mask;
        if (newly_seen == 0) continue;

        StaticArray<point, MAX_PATH_LENGTH> path;
        if (!PlanningCommon::get_grid_time_path(ctx.level, ctx.player, views[i].pos, path)) continue;

        int score =
            PlanningCommon::path_time_cost(ctx.player, path) +
            PlanningCommon::yaw_turn_time_cost(ctx.yaw, views[i].target_yaw) +
            views[i].penalty[0];

        if (!mask_contains_box(newly_seen, ctx.level.box_count)) {
            score += MacroConfig::SUPPLEMENTAL_NON_BOX_PENALTY;
        }

        if (!best.valid || score < best.score) {
            MacroAction action = make_observe_macro_action(views[i], newly_seen);
            action.real_cost =
                PlanningCommon::path_time_cost(ctx.player, path) +
                PlanningCommon::yaw_turn_time_cost(ctx.yaw, views[i].target_yaw);

            best.action = action;
            best.slot = SlotKind::SUPPLEMENTAL_OBSERVE;
            best.score = score;
            best.valid = true;
            best.consumes_reference = false;
        }
    }

    if (!best.valid) return false;
    slot = best;
    return true;
}
