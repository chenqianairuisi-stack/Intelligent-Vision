#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "Exploration.h"
#include "MacroPlanner.h"
#include "Sokoban.h"
#include "Strategy.h"
#include "RobotTask.h"
#include "tuning_config.h"

using namespace SystemConfig;

static const char* SOLVER_BUILD_TAG = __DATE__ " " __TIME__;
// PC 地图从文本顶部向下读取，规划坐标原点在左下角，入场方向为正 Y

namespace {

bool parse_switch_arg(const std::string& arg, const char* option, float& target) {
    const std::string prefix = std::string(option) + "=";
    if (arg.rfind(prefix, 0) != 0) return false;

    const std::string value = arg.substr(prefix.size());
    if (value == "0") {
        target = 0.0f;
        return true;
    }
    if (value == "1") {
        target = 1.0f;
        return true;
    }
    return false;
}

bool apply_simulation_switch(const std::string& arg) {
    // 命令行开关只覆盖当前仿真进程中的运行时参数
    return parse_switch_arg(arg, "--diagonal-move", tune.planning_extra.diagonal_move_enable) ||
           parse_switch_arg(arg, "--box-extra-observe", tune.planning_extra.box_extra_observe_enable) ||
           parse_switch_arg(arg, "--target-extra-observe", tune.planning_extra.target_extra_observe_enable);
}

long long elapsed_ms(std::chrono::high_resolution_clock::time_point begin,
                     std::chrono::high_resolution_clock::time_point end) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
}

point to_grid_coord(int file_x, int file_y) {
    return {
        static_cast<int8_t>(file_x),
        static_cast<int8_t>(MAP_MAX_HEIGHT - 1 - file_y)
    };
}

std::string point_to_output_string(point p) {
    return std::to_string(static_cast<int>(p.x)) + " " + std::to_string(static_cast<int>(p.y));
}

void write_output_point(std::ofstream& out, point p) {
    out << point_to_output_string(p);
}

const char* strategy_phase1_repair_reject_name(StrategyPhase1RepairReject reason) {
    switch (reason) {
        case StrategyPhase1RepairReject::NONE: return "none";
        case StrategyPhase1RepairReject::NOT_RUN: return "not_run";
        case StrategyPhase1RepairReject::MATERIALIZE_FAILED: return "materialize_failed";
        case StrategyPhase1RepairReject::UNRESOLVED_OBLIGATION: return "unresolved_obligation";
        case StrategyPhase1RepairReject::RESIDUAL_DEADLOCKS: return "residual_deadlocks";
        case StrategyPhase1RepairReject::RESIDUAL_UNREACHABLE: return "residual_unreachable";
        case StrategyPhase1RepairReject::NOT_BETTER_THAN_HARD: return "not_better_than_hard";
    }
    return "unknown";
}

const char* strategy_clear_method_name(StrategyClearMethod method) {
    switch (method) {
        case StrategyClearMethod::NONE: return "none";
        case StrategyClearMethod::DIRECT_BOMB_PATH: return "direct_bomb_path";
        case StrategyClearMethod::SOFT_ROUTE_CLEAR: return "soft_route_clear";
        case StrategyClearMethod::REAL_CLEAR_SEARCH: return "real_clear_search";
    }
    return "unknown";
}

const char* strategy_clear_reason_name(StrategyClearReason reason) {
    switch (reason) {
        case StrategyClearReason::NONE: return "none";
        case StrategyClearReason::BOMB_CORRIDOR_BLOCKER: return "bomb_corridor";
        case StrategyClearReason::BOMB_REAL_PATH_BLOCKER: return "bomb_real_path";
        case StrategyClearReason::PUSH_STAND_NEARBY: return "push_stand_nearby";
        case StrategyClearReason::ROUTE_NEARBY: return "route_nearby";
        case StrategyClearReason::RECURSIVE_BOX_BLOCKER: return "recursive_box";
        case StrategyClearReason::REAL_CLEAR_SUPPORT: return "real_clear_support";
    }
    return "unknown";
}

const char* strategy_clear_parking_name(StrategyClearParking parking) {
    switch (parking) {
        case StrategyClearParking::UNKNOWN: return "unknown";
        case StrategyClearParking::DIRECT_SAFE: return "direct_safe";
        case StrategyClearParking::THEORETICAL_RESCUE: return "theoretical_rescue";
        case StrategyClearParking::OPEN_PATH_ONLY: return "open_path_only";
        case StrategyClearParking::DEAD_PARKING: return "dead_parking";
    }
    return "unknown";
}

const char* strategy_rescue_obligation_name(StrategyRescueObligationKind obligation) {
    switch (obligation) {
        case StrategyRescueObligationKind::NONE: return "none";
        case StrategyRescueObligationKind::EXPLICIT_PHASE1_TASK: return "phase1_task";
        case StrategyRescueObligationKind::EXPLICIT_FUTURE_BOMB: return "future_bomb";
        case StrategyRescueObligationKind::UNRESOLVED: return "unresolved";
    }
    return "unknown";
}

void write_bomb_task_record(std::ofstream& out, const BombTask& task) {
    write_output_point(out, task.bomb_start);
    out << " ";
    write_output_point(out, task.target_wall);
    out << " " << task.box_pushes.size();
    for (int p = 0; p < task.box_pushes.size(); ++p) {
        const BoxPushTask& bp = task.box_pushes[p];
        out << " ";
        write_output_point(out, bp.box_start);
        out << " ";
        write_output_point(out, bp.box_target);
    }
}

int bounded_box_count(const SokobanLevel& level) {
    return level.box_count < MAX_BOXES ? level.box_count : MAX_BOXES;
}

int bounded_target_count(const SokobanLevel& level) {
    return level.target_count < MAX_BOXES ? level.target_count : MAX_BOXES;
}

bool is_integer_token(const std::string& token) {
    if (token.empty()) return false;
    int start = token[0] == '-' ? 1 : 0;
    if (start >= static_cast<int>(token.size())) return false;
    for (int i = start; i < static_cast<int>(token.size()); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(token[i]))) return false;
    }
    return true;
}

void set_default_semantic_labels(const SokobanLevel& level, int8_t out_labels[MAX_ENTITIES]) {
    for (int i = 0; i < MAX_ENTITIES; ++i) out_labels[i] = -1;

    const int box_count = bounded_box_count(level);
    const int target_count = bounded_target_count(level);
    for (int i = 0; i < box_count; ++i) out_labels[i] = static_cast<int8_t>(i % 10);
    for (int i = 0; i < target_count; ++i) out_labels[box_count + i] = static_cast<int8_t>(i % 10);
}

void set_uniform_semantic_labels(const SokobanLevel& level,
                                 int8_t out_labels[MAX_ENTITIES],
                                 int8_t semantic_id) {
    for (int i = 0; i < MAX_ENTITIES; ++i) out_labels[i] = -1;

    const int box_count = bounded_box_count(level);
    const int target_count = bounded_target_count(level);
    for (int i = 0; i < box_count; ++i) out_labels[i] = semantic_id;
    for (int i = 0; i < target_count; ++i) out_labels[box_count + i] = semantic_id;
}

void build_labels_from_legacy_mapping(const SokobanLevel& level,
                                      const uint8_t matched_ids[MAX_BOXES],
                                      int8_t out_labels[MAX_ENTITIES]) {
    for (int i = 0; i < MAX_ENTITIES; ++i) out_labels[i] = -1;

    const int box_count = bounded_box_count(level);
    const int target_count = bounded_target_count(level);
    for (int box_id = 0; box_id < box_count; ++box_id) {
        uint8_t target_id = matched_ids[box_id];
        int8_t sem = static_cast<int8_t>(box_id % 10);
        out_labels[box_id] = sem;
        if (target_id < target_count && box_count + target_id < MAX_ENTITIES) {
            out_labels[box_count + target_id] = sem;
        }
    }
    for (int target_id = 0; target_id < target_count; ++target_id) {
        if (out_labels[box_count + target_id] == -1) {
            out_labels[box_count + target_id] = static_cast<int8_t>(target_id % 10);
        }
    }
}

bool apply_semantic_labels_to_level(SokobanLevel& level, const int8_t labels[MAX_ENTITIES]) {
    const int box_count = bounded_box_count(level);
    const int target_count = bounded_target_count(level);

    for (int i = 0; i < box_count; ++i) {
        if (labels[i] < 0 || labels[i] > 9) return false;
        level.box_semantics[i] = static_cast<uint8_t>(labels[i]);
    }
    for (int t = 0; t < target_count; ++t) {
        int idx = box_count + t;
        if (labels[idx] < 0 || labels[idx] > 9) return false;
        level.target_semantics[t] = static_cast<uint8_t>(labels[idx]);
    }
    return true;
}

void build_compatible_matched_ids(const SokobanLevel& level, uint8_t out_ids[MAX_BOXES]) {
    bool used[MAX_BOXES] = {};
    const int box_count = bounded_box_count(level);
    const int target_count = bounded_target_count(level);

    for (int b = 0; b < box_count; ++b) {
        int chosen = -1;
        for (int t = 0; t < target_count; ++t) {
            if (used[t]) continue;
            if (level.target_semantics[t] != level.box_semantics[b]) continue;
            chosen = t;
            break;
        }
        if (chosen == -1) chosen = b < target_count ? b : 0;
        out_ids[b] = static_cast<uint8_t>(chosen);
        if (chosen >= 0 && chosen < target_count) used[chosen] = true;
    }
}

bool read_semantic_values(std::ifstream& map_file, int count, int8_t* out_values) {
    for (int i = 0; i < count; ++i) {
        std::string token;
        if (!(map_file >> token)) return false;
        if (static_cast<int>(token.size()) == count) {
            for (int j = 0; j < count; ++j) {
                if (!std::isdigit(static_cast<unsigned char>(token[j]))) return false;
                out_values[j] = static_cast<int8_t>(token[j] - '0');
            }
            return true;
        }
        if (!is_integer_token(token)) return false;
        int value = std::atoi(token.c_str());
        if (value < 0 || value > 9) return false;
        out_values[i] = static_cast<int8_t>(value);
    }
    return true;
}

bool read_level_from_file(std::ifstream& map_file,
                          SokobanLevel& level,
                          int8_t truth_semantic_labels[MAX_ENTITIES],
                          bool& has_semantic_labels) {
    level = SokobanLevel{};
    level.box_count = 0;
    level.target_count = 0;
    level.bomb_count = 0;
    level.player_start = {static_cast<int8_t>(PLAN_START_X), static_cast<int8_t>(PLAN_START_Y)};
    set_default_semantic_labels(level, truth_semantic_labels);

    for (int file_y = 0; file_y < MAP_MAX_HEIGHT; ++file_y) {
        std::string line;
        if (!(map_file >> line)) return false;

        for (int file_x = 0; file_x < MAP_MAX_WIDTH; ++file_x) {
            char c = file_x < static_cast<int>(line.size()) ? line[file_x] : '-';
            point p = to_grid_coord(file_x, file_y);
            level.map[p.y][p.x] = (c == '#') ? 1 : 0;

            if (c == '@') {
                level.player_start = p;
            } else if (c == '$' && level.box_count < MAX_BOXES) {
                level.boxes[level.box_count++] = p;
            } else if (c == '.' && level.target_count < MAX_BOXES) {
                level.targets[level.target_count++] = p;
            } else if (c == '*' && level.bomb_count < MAX_BOMBS) {
                level.bombs[level.bomb_count++] = p;
            }
        }
    }

    std::string token;
    if (map_file >> token) {
        if (token == "START" || token == "PLAYER" || token == "CAR") {
            int start_x = PLAN_START_X;
            int start_y = PLAN_START_Y;
            if (!(map_file >> start_x >> start_y)) return false;
            level.player_start = {
                static_cast<int8_t>(start_x),
                static_cast<int8_t>(start_y)
            };
            if (!(map_file >> token)) {
                has_semantic_labels = false;
                set_default_semantic_labels(level, truth_semantic_labels);
                return true;
            }
        }
    }

    const int box_count = bounded_box_count(level);
    const int target_count = bounded_target_count(level);
    has_semantic_labels = true;
    for (int i = 0; i < MAX_ENTITIES; ++i) truth_semantic_labels[i] = -1;

    if (token == "SEMANTICS") {
        std::string section;
        if (!(map_file >> section)) return false;
        if (section != "BOXES" && section != "BOX_SEM") return false;
        if (!read_semantic_values(map_file, box_count, truth_semantic_labels)) return false;

        if (!(map_file >> section)) return false;
        if (section != "TARGETS" && section != "TARGET_SEM") return false;
        return read_semantic_values(map_file, target_count, truth_semantic_labels + box_count);
    }

    if (token == "BOXES" || token == "BOX_SEM") {
        if (!read_semantic_values(map_file, box_count, truth_semantic_labels)) return false;

        std::string section;
        if (!(map_file >> section)) return false;
        if (section != "TARGETS" && section != "TARGET_SEM") return false;
        return read_semantic_values(map_file, target_count, truth_semantic_labels + box_count);
    }

    // 兼容旧格式：尾部只有 box_count 个数字，含义为 box -> target_id。
    if (is_integer_token(token)) {
        uint8_t legacy_matched_ids[MAX_BOXES] = {};
        legacy_matched_ids[0] = static_cast<uint8_t>(std::atoi(token.c_str()));
        for (int i = 1; i < box_count; ++i) {
            int target_id = 0;
            if (!(map_file >> target_id)) return false;
            legacy_matched_ids[i] = static_cast<uint8_t>(target_id);
        }
        build_labels_from_legacy_mapping(level, legacy_matched_ids, truth_semantic_labels);
        return true;
    }

    has_semantic_labels = false;
    set_default_semantic_labels(level, truth_semantic_labels);
    return true;
}

void append_path_strings(const StaticArray<point, MAX_PATH_LENGTH>& segment,
                         std::vector<std::string>& out) {
    for (int i = 0; i < segment.size(); ++i) {
        out.push_back(point_to_output_string(segment[i]));
    }
}

int direction_between(point from, point to) {
    for (int d = 0; d < 4; ++d) {
        if (from + MOVE[d] == to) return d;
    }
    return -1;
}

int active_box_at(const point box_pos[MAX_BOXES], const bool active[MAX_BOXES], int box_count, point p) {
    for (int i = 0; i < box_count; ++i) {
        if (active[i] && box_pos[i] == p) return i;
    }
    return -1;
}

int matching_active_target_at(const SokobanLevel& level, const bool target_done[MAX_BOXES], int box_id, point p) {
    if (box_id < 0 || box_id >= level.box_count) return -1;
    uint8_t sem = level.box_semantics[box_id];
    for (int t = 0; t < level.target_count; ++t) {
        if (target_done[t]) continue;
        if (level.target_semantics[t] == sem && level.targets[t] == p) return t;
    }
    return -1;
}

void write_sokoban_replay_diag(std::ofstream& out,
                               const SokobanLevel& level,
                               const StaticArray<point, MAX_PATH_LENGTH>& path) {
    point box_pos[MAX_BOXES];
    bool box_active[MAX_BOXES] = {};
    bool target_done[MAX_BOXES] = {};
    int box_push_count[MAX_BOXES] = {};
    int box_last_target[MAX_BOXES];
    for (int i = 0; i < MAX_BOXES; ++i) box_last_target[i] = -1;

    const int box_count = bounded_box_count(level);
    const int target_count = bounded_target_count(level);
    for (int i = 0; i < box_count; ++i) {
        box_pos[i] = level.boxes[i];
        box_active[i] = true;
    }

    out << "SOKOBAN_REPLAY_BEGIN\n";
    out << "REPLAY_BOX_COUNT " << box_count << "\n";
    out << "REPLAY_TARGET_COUNT " << target_count << "\n";

    int push_index = 0;
    int complete_index = 0;
    int run_index = 0;
    int run_box = -1;
    int run_dir = -1;
    int run_len = 0;
    int run_start_push = 0;
    point run_start{};
    point run_end{};
    auto flush_run = [&](int complete_target) {
        if (run_len <= 0) return;
        out << "REPLAY_RUN "
            << run_index++ << " "
            << run_box << " "
            << run_dir << " "
            << static_cast<int>(run_start.x) << " "
            << static_cast<int>(run_start.y) << " "
            << static_cast<int>(run_end.x) << " "
            << static_cast<int>(run_end.y) << " "
            << run_len << " "
            << run_start_push << " "
            << complete_target << "\n";
        run_len = 0;
        run_box = -1;
        run_dir = -1;
    };

    for (int i = 1; i < path.size(); ++i) {
        point from = path[i - 1];
        point to = path[i];
        int dir = direction_between(from, to);
        if (dir < 0) continue;

        int box_id = active_box_at(box_pos, box_active, box_count, to);
        if (box_id < 0) continue;

        point next_box = to + MOVE[dir];
        int target_id = matching_active_target_at(level, target_done, box_id, next_box);

        out << "REPLAY_PUSH "
            << push_index++ << " "
            << box_id << " "
            << static_cast<int>(level.box_semantics[box_id]) << " "
            << static_cast<int>(box_pos[box_id].x) << " "
            << static_cast<int>(box_pos[box_id].y) << " "
            << static_cast<int>(next_box.x) << " "
            << static_cast<int>(next_box.y) << " "
            << dir << " "
            << target_id << "\n";

        box_pos[box_id] = next_box;
        ++box_push_count[box_id];

        if (target_id != -1) {
            if (run_len > 0 && (run_box != box_id || run_dir != dir)) {
                flush_run(-1);
            }
            if (run_len == 0) {
                run_box = box_id;
                run_dir = dir;
                run_len = 1;
                run_start_push = push_index - 1;
                run_start = to;
                run_end = next_box;
            } else {
                ++run_len;
                run_end = next_box;
            }
            flush_run(target_id);

            target_done[target_id] = true;
            box_active[box_id] = false;
            box_last_target[box_id] = target_id;
            out << "REPLAY_COMPLETE "
                << complete_index++ << " "
                << box_id << " "
                << target_id << " "
                << static_cast<int>(next_box.x) << " "
                << static_cast<int>(next_box.y) << " "
                << push_index << "\n";
        } else {
            if (run_len > 0 && (run_box != box_id || run_dir != dir)) {
                flush_run(-1);
            }
            if (run_len == 0) {
                run_box = box_id;
                run_dir = dir;
                run_len = 1;
                run_start_push = push_index - 1;
                run_start = to;
                run_end = next_box;
            } else {
                ++run_len;
                run_end = next_box;
            }
        }
    }
    flush_run(-1);

    out << "REPLAY_SUMMARY " << push_index << " " << complete_index << "\n";
    for (int i = 0; i < box_count; ++i) {
        out << "REPLAY_BOX "
            << i << " "
            << static_cast<int>(level.box_semantics[i]) << " "
            << box_push_count[i] << " "
            << box_last_target[i] << " "
            << static_cast<int>(box_pos[i].x) << " "
            << static_cast<int>(box_pos[i].y) << " "
            << (box_active[i] ? 1 : 0) << "\n";
    }
    out << "SOKOBAN_REPLAY_END\n";
}

bool load_semantic_solver(SokobanLevel& level,
                          const int8_t semantic_labels[MAX_ENTITIES],
                          const StaticArray<BombTask, MAX_BOMBS>& bombs) {
    if (!apply_semantic_labels_to_level(level, semantic_labels)) return false;
    solver.load_from_vision(level, bombs.size() > 0 ? bombs.begin() : nullptr, bombs.size());
    if (!solver.bind_semantics()) return false;
    return true;
}

} // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (!apply_simulation_switch(argv[i])) return -4;
    }

    std::ifstream map_file("map_input.txt");
    if (!map_file.is_open()) return -1;

    SokobanLevel logical_level;
    bool is_advanced_stage = false;
    int8_t truth_semantic_labels[MAX_ENTITIES];
    if (!read_level_from_file(map_file, logical_level, truth_semantic_labels, is_advanced_stage)) return -2;
    map_file.close();

    int8_t observed_semantic_labels[MAX_ENTITIES];
    for (int i = 0; i < MAX_ENTITIES; ++i) observed_semantic_labels[i] = -1;

    std::ofstream out_file("path_output.txt");
    if (!out_file.is_open()) return -3;

    strategic_planner.reset_profile();

    long long time1 = 0;
    long long time2 = 0;
    long long time3 = 0;
    long long time4 = 0;

    StaticArray<BombTask, MAX_BOMBS> phase1_bombs;
    StaticArray<BombTask, MAX_BOMBS> phase1_bombs_for_output;
    StaticArray<BombTask, MAX_BOMBS> phase2_bombs;
    StaticArray<MacroAction, 32> reference_patrol_actions;
    StaticArray<MacroAction, 32> patrol_actions;
    std::vector<std::string> patrol_out;
    std::vector<std::string> mcu_trace;
    uint32_t observed_mask = 0;
    float current_observe_yaw = SystemConfig::ENTRY_YAW;
    bool patrol_failed = false;
    bool success = false;
    uint8_t planner_matched_ids[MAX_BOXES] = {};
    int8_t planner_semantic_labels[MAX_ENTITIES];
    for (int i = 0; i < MAX_ENTITIES; ++i) planner_semantic_labels[i] = -1;
    bool has_planner_matching = false;

    if (is_advanced_stage) {
        auto t1_start = std::chrono::high_resolution_clock::now();
        phase1_bombs = strategic_planner.plan_phase1_bombs(logical_level);
        phase1_bombs_for_output = phase1_bombs;
        auto t1_end = std::chrono::high_resolution_clock::now();
        time1 = elapsed_ms(t1_start, t1_end);

        auto t2_start = std::chrono::high_resolution_clock::now();
        patrol_planner.load_level(logical_level);
        reference_patrol_actions = patrol_planner.plan_optimal_patrol(
            logical_level.player_start,
            phase1_bombs,
            SystemConfig::ENTRY_YAW,
            0
        );
        if (reference_patrol_actions.empty()) patrol_failed = true;

        macro_planner.reset(logical_level);
        macro_planner.set_reference_plan(reference_patrol_actions);

        enum class McuSimPhase : uint8_t {
            EXEC_ACTION_DISPATCH,
            EXEC_TASK_QUEUE,
            BIND_SEMANTICS,
            DONE
        };

        using App::GameEngine::RobotTask;
        using App::GameEngine::TaskType;
        McuSimPhase sim_phase = McuSimPhase::EXEC_ACTION_DISPATCH;
        StaticArray<RobotTask, 10> task_queue;
        size_t current_task_idx = 0;
        MacroAction current_macro_action;
        int macro_action_count = 0;
        int exploration_replan_count = 0;
        int loop_guard = 0;

        // 按 MCU start_macro_action() 的规则把宏动作拆成微任务
        auto start_macro_action = [&](const MacroAction& action) {
            current_macro_action = action;
            task_queue.clear();
            current_task_idx = 0;
            if (action.kind == MacroActionKind::PUSH_BOMB) {
                task_queue.push_back(RobotTask::make_path_bomb(action.bomb_push));
                task_queue.push_back(RobotTask::make_wait_track());
                task_queue.push_back(RobotTask::make_apply_bomb_result(action.bomb_push));
            } else if (action.kind == MacroActionKind::PUSH_BOX) {
                task_queue.push_back(RobotTask::make_path_box(action.box_push));
                task_queue.push_back(RobotTask::make_wait_track());
                task_queue.push_back(RobotTask::make_update_box(action.box_push));
            } else {
                const ViewPose& view = action.observe.view;
                task_queue.push_back(RobotTask::make_path_obs(view.pos));
                task_queue.push_back(RobotTask::make_wait_track());
                task_queue.push_back(RobotTask::make_align(view.target_yaw));
                task_queue.push_back(RobotTask::make_capture(action.observe.active_mask));
            }
        };

        // 模拟 Vision.update() 收到 ACK 后逐实体收到 0x41 结果
        auto run_art2_capture = [&](uint32_t requested_mask) {
            uint32_t received_mask = 0u;
            bool capture_ack_received = false;
            mcu_trace.push_back("ART2_CAPTURE " + std::to_string(requested_mask));

            // ART2 必须先回 ACK，主控才会接收本批次的结果
            capture_ack_received = true;
            mcu_trace.push_back("ART2_ACK");
            for (int entity_id = 0; entity_id < MAX_ENTITIES; ++entity_id) {
                const uint32_t entity_bit = 1UL << entity_id;
                if ((requested_mask & entity_bit) == 0u) continue;
                const int8_t semantic_id = truth_semantic_labels[entity_id];
                if (semantic_id < 0 || semantic_id > 9) return false;
                observed_semantic_labels[entity_id] = semantic_id;
                received_mask |= entity_bit;
                // PATROL 段保留观测完成事件，visualizer 可按真实 ART2 回传顺序逐个揭示语义
                const bool is_box = entity_id < logical_level.box_count;
                // 观测结果必须携带本次 ALIGN_YAW 的真实朝向，供 visualizer 统计旋转次数
                const int observe_yaw = static_cast<int>(
                    current_macro_action.observe.view.target_yaw + 0.5f) % 360;
                patrol_out.push_back(
                    "OBSERVE " + std::to_string(entity_id) + " " + (is_box ? "1" : "0") +
                    " " + std::to_string(observe_yaw)
                );
                mcu_trace.push_back(
                    "ART2_RESULT " + std::to_string(entity_id) + " " +
                    std::to_string(static_cast<int>(semantic_id))
                );
            }
            return capture_ack_received &&
                   (received_mask & requested_mask) == requested_mask;
        };

        while (loop_guard++ < 4096 && !patrol_failed && sim_phase != McuSimPhase::DONE) {
            if (sim_phase == McuSimPhase::EXEC_ACTION_DISPATCH) {
                macro_planner.sync_semantics(observed_semantic_labels);
                if (macro_planner.ready_for_sokoban(logical_level)) {
                    sim_phase = McuSimPhase::BIND_SEMANTICS;
                    continue;
                }

                MacroPlanContext ctx;
                ctx.level = logical_level;
                ctx.player = logical_level.player_start;
                ctx.yaw = current_observe_yaw;
                ctx.bomb_tasks = &phase1_bombs;

                MacroAction action;
                if (!macro_planner.plan_next_action(ctx, action)) {
                    if (macro_planner.needs_exploration_replan() && exploration_replan_count < 4) {
                        patrol_planner.load_level(logical_level);
                        StaticArray<MacroAction, 32> replanned_actions =
                            patrol_planner.plan_optimal_patrol(
                                logical_level.player_start,
                                phase1_bombs,
                                current_observe_yaw,
                                macro_planner.observed_mask()
                            );
                        if (!replanned_actions.empty()) {
                            reference_patrol_actions = replanned_actions;
                            macro_planner.set_reference_plan(reference_patrol_actions);
                            ++exploration_replan_count;
                            continue;
                        }
                    }
                    patrol_failed = true;
                    continue;
                }
                if (++macro_action_count > 32) {
                    patrol_failed = true;
                    continue;
                }
                patrol_actions.push_back(action);
                start_macro_action(action);
                sim_phase = McuSimPhase::EXEC_TASK_QUEUE;
                continue;
            }

            if (sim_phase == McuSimPhase::EXEC_TASK_QUEUE) {
                if (current_task_idx >= task_queue.size()) {
                    sim_phase = McuSimPhase::EXEC_ACTION_DISPATCH;
                    continue;
                }

                const RobotTask& task = task_queue[current_task_idx];
                bool task_done = false;
                StaticArray<point, MAX_PATH_LENGTH> segment;
                switch (task.type) {
                    case TaskType::LOAD_PATH_OBS:
                        task_done = PlanningCommon::get_optimized_observe_path(
                            logical_level, logical_level.player_start, task.param.target_grid, segment);
                        if (task_done) {
                            append_path_strings(segment, patrol_out);
                            if (!segment.empty()) logical_level.player_start = segment.back();
                        }
                        break;
                    case TaskType::LOAD_PATH_BOMB:
                        task_done = PlanningCommon::get_bomb_push_path(
                            logical_level, logical_level.player_start,
                            make_bomb_task(task.param.bomb_push), segment);
                        if (task_done) {
                            append_path_strings(segment, patrol_out);
                            if (!segment.empty()) logical_level.player_start = segment.back();
                        }
                        break;
                    case TaskType::LOAD_PATH_BOX:
                        task_done = PlanningCommon::append_box_push_path(
                            logical_level, logical_level.player_start,
                            make_box_push_task(task.param.box_push), segment);
                        if (task_done) append_path_strings(segment, patrol_out);
                        break;
                    case TaskType::WAIT_TRACKING_DONE:
                        task_done = true;
                        break;
                    case TaskType::ALIGN_YAW:
                        current_observe_yaw = task.param.target_yaw;
                        task_done = true;
                        break;
                    case TaskType::WAIT_ART2_CAPTURE: {
                        const uint32_t requested_mask = task.param.capture.active_mask;
                        if (run_art2_capture(requested_mask)) {
                            observed_mask |= requested_mask;
                            macro_planner.sync_semantics(observed_semantic_labels);
                            macro_planner.apply_observation(logical_level, requested_mask);
                            task_done = true;
                        }
                        break;
                    }
                    case TaskType::APPLY_BOMB_RESULT:
                        PlanningCommon::apply_executed_bomb_push_result(
                            logical_level, phase1_bombs, task.param.bomb_push);
                        task_done = true;
                        break;
                    case TaskType::UPDATE_BOX_LOGIC:
                        PlanningCommon::apply_box_push_action_effect(logical_level, task.param.box_push);
                        task_done = true;
                        break;
                    default:
                        break;
                }
                if (!task_done) {
                    patrol_failed = true;
                    continue;
                }
                ++current_task_idx;
                continue;
            }

            if (sim_phase == McuSimPhase::BIND_SEMANTICS) {
                sim_phase = McuSimPhase::DONE;
            }
        }
        if (loop_guard >= 4096) patrol_failed = true;
        auto t2_end = std::chrono::high_resolution_clock::now();
        time2 = elapsed_ms(t2_start, t2_end);

        if (!patrol_failed) {
            bool all_done = true;
            for (uint8_t entity_id = 0; entity_id < MAX_ENTITIES; ++entity_id) {
                if ((observed_mask & (1UL << entity_id)) && observed_semantic_labels[entity_id] == -1) {
                    all_done = false;
                    break;
                }
            }

            macro_planner.sync_semantics(observed_semantic_labels);
            bool semantic_ready = macro_planner.fill_semantic_labels(planner_semantic_labels);
            if (!all_done || !semantic_ready || !apply_semantic_labels_to_level(logical_level, planner_semantic_labels)) {
                patrol_failed = true;
            } else {
                has_planner_matching = true;
                build_compatible_matched_ids(logical_level, planner_matched_ids);

                auto t3_start = std::chrono::high_resolution_clock::now();
                phase2_bombs = strategic_planner.plan_phase2_bombs(logical_level, phase1_bombs);
                auto t3_end = std::chrono::high_resolution_clock::now();
                time3 += elapsed_ms(t3_start, t3_end);

                if (load_semantic_solver(logical_level, planner_semantic_labels, phase2_bombs)) {
                    auto t4_start = std::chrono::high_resolution_clock::now();
                    success = solver.solve();
                    auto t4_end = std::chrono::high_resolution_clock::now();
                    time4 += elapsed_ms(t4_start, t4_end);
                }
            }
        }
    } else {
        set_uniform_semantic_labels(logical_level, truth_semantic_labels, 0);
        if (!apply_semantic_labels_to_level(logical_level, truth_semantic_labels)) return -4;
        solver.load_from_vision(logical_level, nullptr, 0);
        if (!solver.bind_semantics()) return -5;

        auto t4_start = std::chrono::high_resolution_clock::now();
        success = solver.solve();
        auto t4_end = std::chrono::high_resolution_clock::now();
        time4 = elapsed_ms(t4_start, t4_end);
    }

    out_file << "TIMES " << time1 << " " << time2 << " " << time3 << " " << time4 << "\n";
    out_file << "BUILD " << SOLVER_BUILD_TAG << "\n";
    out_file << "COST_MODEL "
             << PlanningCommon::MotionCostConfig::MOVE_STEP << " "
             << PlanningCommon::MotionCostConfig::STOP_NODE << " "
             << PlanningCommon::MotionCostConfig::OBSERVE_EXTRA << " "
             << PlanningCommon::MotionCostConfig::TURN_EXTRA << "\n";
    out_file << "SEMANTICS";
    for (int i = 0; i < logical_level.box_count + logical_level.target_count; ++i) {
        int label = has_planner_matching ? planner_semantic_labels[i] : truth_semantic_labels[i];
        out_file << " " << label;
    }
    out_file << "\n";
    out_file << "MATCHED_IDS";
    for (int i = 0; i < logical_level.box_count; ++i) {
        out_file << " " << static_cast<int>(has_planner_matching ? planner_matched_ids[i] : i);
    }
    out_file << "\n";
    if (solver.profile_enabled()) {
        const SokobanProfile& sp = solver.get_profile();
        unsigned long long nps = 0;
        if (time4 > 0) {
            nps = (static_cast<unsigned long long>(sp.expanded_nodes) * 1000ULL) /
                  static_cast<unsigned long long>(time4);
        }
        out_file << "PROFILE "
                 << sp.expanded_nodes << " "
                 << sp.generated_moves << " "
                 << sp.tt_hits << " "
                 << sp.heuristic_dead_prunes << " "
                 << sp.threshold_prunes << " "
                 << sp.path_cycle_prunes << " "
                 << sp.static_deadlock_prunes << " "
                 << sp.block_2x2_prunes << " "
                 << sp.max_depth << " "
                 << sp.threshold_iterations << " "
                 << sp.final_threshold << " "
                 << nps << "\n";
    }
    if constexpr (StrategyConfig::ENABLE_PROFILE) {
        const StrategyProfile& stp = strategic_planner.get_profile();
        out_file << "STRATEGY_PROFILE "
                 << static_cast<int>(stp.eval_count) << " "
                 << stp.dropped_evals << "\n";
        for (int e = 0; e < stp.eval_count; ++e) {
            const StrategyEvalProfile& ev = stp.evals[e];
            out_file << "STRATEGY_EVAL "
                     << e << " "
                     << static_cast<int>(ev.mode) << " "
                     << static_cast<int>(ev.selected_pass) << " "
                     << ev.selected_deadlocks << " "
                     << ev.selected_profit << " "
                     << static_cast<int>(ev.selected_tasks) << "\n";
            for (int p = 0; p < 3; ++p) {
                const StrategyPassProfile& pass = ev.passes[p];
                out_file << "STRATEGY_PASS "
                         << e << " "
                         << p << " "
                         << pass.result_deadlocks << " "
                         << pass.result_profit << " "
                         << static_cast<int>(pass.result_tasks) << " "
                         << pass.root_candidates << " "
                         << static_cast<int>(pass.root_branch_limit) << " "
                         << pass.local_clear_calls << " "
                         << pass.local_clear_successes << " "
                         << pass.materialize_calls << " "
                         << pass.materialize_successes << " "
                         << static_cast<int>(pass.top_count) << "\n";
                out_file << "STRATEGY_TOP " << e << " " << p << " " << static_cast<int>(pass.top_count);
                for (int i = 0; i < pass.top_count; ++i) {
                    const StrategyCandidateProfile& top = pass.top[i];
                    out_file << " "
                             << static_cast<int>(top.bomb_x) << " "
                             << static_cast<int>(top.bomb_y) << " "
                             << static_cast<int>(top.wall_x) << " "
                             << static_cast<int>(top.wall_y) << " "
                             << top.score;
                }
                out_file << "\n";
                out_file << "STRATEGY_STATS "
                         << e << " "
                         << p << " "
                         << pass.dfs_nodes << " "
                         << pass.fast_bfs_calls << " "
                         << pass.candidate_evals << " "
                         << pass.candidate_kept << " "
                         << pass.child_branches << " "
                         << static_cast<int>(pass.logic_builds) << "\n";
            }
        }
        const StrategyPhase1RepairProfile& repair = stp.phase1_repair;
        out_file << "STRATEGY_PHASE1_REPAIR "
                 << static_cast<int>(repair.valid) << " "
                 << static_cast<int>(repair.selected_soft) << " "
                 << static_cast<int>(repair.source_pass) << " "
                 << static_cast<int>(repair.repaired_ok) << " "
                 << static_cast<int>(repair.beats_hard) << " "
                 << static_cast<int>(repair.repaired_outstanding_obligations) << " "
                 << static_cast<int>(repair.hard_outstanding_obligations) << " "
                 << strategy_phase1_repair_reject_name(repair.reject_reason) << " "
                 << repair.soft_deadlocks << " "
                 << repair.soft_unreachable << " "
                 << repair.soft_profit << " "
                 << repair.hard_deadlocks << " "
                 << repair.hard_unreachable << " "
                 << repair.hard_distance << " "
                 << repair.repaired_deadlocks << " "
                 << repair.repaired_unreachable << " "
                 << repair.repaired_distance << " "
                 << repair.repaired_cost << " "
                 << repair.raw_tasks.size() << " "
                 << repair.repaired_tasks.size() << " "
                 << static_cast<int>(repair.step_count) << "\n";

        out_file << "STRATEGY_PHASE1_REPAIR_RAW " << repair.raw_tasks.size() << "\n";
        for (int i = 0; i < repair.raw_tasks.size(); ++i) {
            out_file << "STRATEGY_PHASE1_REPAIR_RAW_TASK " << i << " ";
            write_bomb_task_record(out_file, repair.raw_tasks[i]);
            out_file << "\n";
        }

        out_file << "STRATEGY_PHASE1_REPAIR_FIXED " << repair.repaired_tasks.size() << "\n";
        for (int i = 0; i < repair.repaired_tasks.size(); ++i) {
            out_file << "STRATEGY_PHASE1_REPAIR_FIXED_TASK " << i << " ";
            write_bomb_task_record(out_file, repair.repaired_tasks[i]);
            out_file << "\n";
        }

        out_file << "STRATEGY_PHASE1_REPAIR_STEPS " << static_cast<int>(repair.step_count) << "\n";
        for (int i = 0; i < repair.step_count && i < MAX_BOMBS; ++i) {
            const StrategyPhase1RepairStepProfile& step = repair.steps[i];
            out_file << "STRATEGY_PHASE1_REPAIR_STEP "
                     << static_cast<int>(step.index) << " "
                     << static_cast<int>(step.direct_executable) << " "
                     << static_cast<int>(step.materialized) << " "
                     << static_cast<int>(step.apply_ok) << " "
                     << static_cast<int>(step.outstanding_obligations) << " "
                     << step.deadlocks << " "
                     << step.unreachable << " "
                     << step.distance << " "
                     << step.sequence_cost << " "
                     << static_cast<int>(step.player.x) << " "
                     << static_cast<int>(step.player.y) << " ";
            write_bomb_task_record(out_file, step.task);
            out_file << "\n";
        }
    }
#if STRATEGY_ENABLE_HOT_PROFILE
    if constexpr (StrategyConfig::ENABLE_HOT_PROFILE) {
        const StrategyHotProfile& hot = strategic_planner.get_profile().hot;
        out_file << "STRATEGY_HOT_PROFILE "
                 << hot.fast_bfs_calls << " "
                 << hot.fast_bfs_us << " "
                 << hot.fast_bfs_player_reach_calls << " "
                 << hot.fast_bfs_state_pops << " "
                 << hot.fast_bfs_max_queue << " "
                 << hot.macro_soft_calls << " "
                 << hot.macro_soft_us << " "
                 << hot.macro_soft_state_pops << " "
                 << hot.macro_soft_max_queue << " "
                 << hot.local_clear_calls << " "
                 << hot.local_clear_successes << " "
                 << hot.local_clear_us << " "
                 << hot.soft_route_builds << " "
                 << hot.soft_route_successes << " "
                 << hot.box_push_checks << " "
                 << hot.box_push_successes << " "
                 << hot.bomb_path_checks << " "
                 << hot.bomb_path_successes << " "
                 << hot.player_path_checks << " "
                 << hot.real_clear_nodes << " "
                 << hot.real_clear_candidate_total << " "
                 << hot.real_clear_try_total << " "
                 << hot.real_clear_max_depth << "\n";
    }
#endif
#if STRATEGY_ENABLE_SHADOW_CLEAR_CLASSIFIER
    if constexpr (StrategyConfig::ENABLE_SHADOW_CLEAR_CLASSIFIER) {
        const StrategyShadowClearProfile& shadow = strategic_planner.get_profile().shadow_clear;
        out_file << "STRATEGY_SHADOW_CLEAR "
                 << shadow.route_clear_attempts << " "
                 << shadow.route_clear_successes << " "
                 << shadow.route_clear_failed_no_blocker << " "
                 << shadow.blocker_bomb_corridor << " "
                 << shadow.blocker_bomb_real_path << " "
                 << shadow.blocker_push_stand_nearby << " "
                 << shadow.blocker_push_stand_exact << " "
                 << shadow.blocker_push_stand_near_only << " "
                 << shadow.blocker_route_nearby << " "
                 << shadow.blocker_recursive << " "
                 << shadow.blocker_real_support << " "
                 << shadow.accepted_direct_safe << " "
                 << shadow.accepted_theoretical_rescue << " "
                 << shadow.accepted_open_path_only << " "
                 << shadow.accepted_dead_parking << " "
                 << shadow.accepted_exact_reason << " "
                 << shadow.accepted_nearby_reason << " "
                 << shadow.real_nodes << " "
                 << shadow.real_source_exact << " "
                 << shadow.real_source_near << " "
                 << shadow.real_source_far << " "
                 << shadow.real_push_candidates << " "
                 << shadow.real_push_executable << " "
                 << shadow.real_opens_path << " "
                 << shadow.real_parking_checks << " "
                 << shadow.real_parking_direct_safe << " "
                 << shadow.real_parking_theoretical << " "
                 << shadow.real_parking_dead << " "
                 << shadow.real_parking_rejected << " "
                 << shadow.decide_keep_exact_blocker << " "
                 << shadow.decide_deprioritize_near_stand << " "
                 << shadow.decide_deprioritize_route_near << " "
                 << shadow.decide_keep_recursive << " "
                 << shadow.decide_validate_real_exact << " "
                 << shadow.decide_validate_real_near << " "
                 << shadow.decide_deprioritize_real_far << " "
                 << shadow.decide_accept_direct_safe << " "
                 << shadow.decide_require_theory_proof << " "
                 << shadow.decide_require_open_path << " "
                 << shadow.decide_reject_dead_parking << " "
                 << shadow.decide_reject_no_blocker << "\n";

        const uint32_t near_only = shadow.blocker_push_stand_near_only;
        const uint32_t real_validation =
            shadow.real_source_exact + shadow.real_source_near + shadow.real_source_far;
        const uint32_t theory_work =
            shadow.accepted_theoretical_rescue + shadow.real_parking_theoretical;
        const uint32_t dead_reject = shadow.real_parking_rejected;
        const uint32_t open_only = shadow.accepted_open_path_only;

        auto pressure_level = [](uint32_t value, uint32_t warn, uint32_t high) -> uint8_t {
            if (value >= high) return 2;
            if (value >= warn) return 1;
            return 0;
        };

        // 影子分类只输出未来决策压力，不参与当前清障验收
        uint8_t near_level = pressure_level(near_only, 25, 50);
        uint8_t real_level = pressure_level(real_validation, 350, 900);
        uint8_t theory_level = pressure_level(theory_work, 200, 300);
        uint8_t dead_level = pressure_level(dead_reject, 100, 1000);
        uint8_t open_level = pressure_level(open_only, 1, 5);
        uint8_t no_blocker_level = shadow.route_clear_failed_no_blocker > 0 ? 1 : 0;
        uint8_t timeout_like =
            (near_level == 2 || real_level == 2 || theory_level == 2 || dead_level == 2) ? 1 : 0;
        uint16_t risk_score =
            static_cast<uint16_t>(near_level + open_level + no_blocker_level +
                                  real_level * 2 + theory_level * 2 + dead_level * 2);

        out_file << "STRATEGY_SHADOW_CLASS "
                 << static_cast<int>(near_level) << " "
                 << static_cast<int>(real_level) << " "
                 << static_cast<int>(theory_level) << " "
                 << static_cast<int>(dead_level) << " "
                 << static_cast<int>(open_level) << " "
                 << static_cast<int>(no_blocker_level) << " "
                 << static_cast<int>(timeout_like) << " "
                 << risk_score << " "
                 << near_only << " "
                 << real_validation << " "
                 << theory_work << " "
                 << dead_reject << " "
                 << open_only << "\n";
    }
#endif
#if STRATEGY_ENABLE_CLEAR_DIAG
    if constexpr (StrategyConfig::ENABLE_CLEAR_DIAG) {
        const StrategyProfile& stp = strategic_planner.get_profile();
        out_file << "STRATEGY_CLEAR_PROFILE "
                 << static_cast<int>(stp.clear_diag_count) << " "
                 << stp.dropped_clear_diags << "\n";
        for (int i = 0; i < stp.clear_diag_count; ++i) {
            const StrategyClearRouteProfile& diag = stp.clear_diags[i];
            out_file << "STRATEGY_CLEAR "
                     << i << " "
                     << static_cast<int>(diag.eval_index) << " "
                     << static_cast<int>(diag.pass) << " "
                     << static_cast<int>(diag.success) << " "
                     << strategy_clear_method_name(diag.method) << " "
                     << static_cast<int>(diag.phase2_specific) << " "
                     << static_cast<int>(diag.include_player_access_clear) << " "
                     << static_cast<int>(diag.bomb_start.x) << " "
                     << static_cast<int>(diag.bomb_start.y) << " "
                     << static_cast<int>(diag.target_wall.x) << " "
                     << static_cast<int>(diag.target_wall.y) << " "
                     << diag.cost << " "
                     << static_cast<int>(diag.route_len) << " "
                     << static_cast<int>(diag.blocker_count) << " "
                     << static_cast<int>(diag.push_count) << "\n";
            for (int p = 0; p < diag.push_count && p < StrategyConfig::CLEAR_DIAG_PUSH_LIMIT; ++p) {
                const StrategyClearPushProfile& push = diag.pushes[p];
                out_file << "STRATEGY_CLEAR_PUSH "
                         << i << " "
                         << p << " "
                         << static_cast<int>(push.box_id) << " "
                         << strategy_clear_reason_name(push.reason) << " "
                         << strategy_clear_parking_name(push.parking) << " "
                         << static_cast<int>(push.box_start.x) << " "
                         << static_cast<int>(push.box_start.y) << " "
                         << static_cast<int>(push.box_target.x) << " "
                         << static_cast<int>(push.box_target.y) << " "
                         << static_cast<int>(push.depth) << " "
                         << static_cast<int>(push.opens_bomb_path) << " "
                         << static_cast<int>(push.safe_without_open_path) << " "
                         << push.score << " "
                         << strategy_rescue_obligation_name(push.obligation) << " "
                         << static_cast<int>(push.owner_task_index) << " "
                         << static_cast<int>(push.owner_bomb_start.x) << " "
                         << static_cast<int>(push.owner_bomb_start.y) << " "
                         << static_cast<int>(push.owner_target_wall.x) << " "
                         << static_cast<int>(push.owner_target_wall.y) << "\n";
            }
        }
    }
#endif

    out_file << "BOMB_TASKS_1 " << phase1_bombs_for_output.size() << "\n";
    for (int i = 0; i < phase1_bombs_for_output.size(); ++i) {
        write_output_point(out_file, phase1_bombs_for_output[i].bomb_start);
        out_file << " ";
        write_output_point(out_file, phase1_bombs_for_output[i].target_wall);
        out_file << " " << phase1_bombs_for_output[i].box_pushes.size();
        for (int p = 0; p < phase1_bombs_for_output[i].box_pushes.size(); ++p) {
            const BoxPushTask& bp = phase1_bombs_for_output[i].box_pushes[p];
            out_file << " ";
            write_output_point(out_file, bp.box_start);
            out_file << " ";
            write_output_point(out_file, bp.box_target);
        }
        out_file << "\n";
    }

    out_file << "BOMB_TASKS_2 " << phase2_bombs.size() << "\n";
    for (int i = 0; i < phase2_bombs.size(); ++i) {
        write_output_point(out_file, phase2_bombs[i].bomb_start);
        out_file << " ";
        write_output_point(out_file, phase2_bombs[i].target_wall);
        out_file << " " << phase2_bombs[i].box_pushes.size();
        for (int p = 0; p < phase2_bombs[i].box_pushes.size(); ++p) {
            const BoxPushTask& bp = phase2_bombs[i].box_pushes[p];
            out_file << " ";
            write_output_point(out_file, bp.box_start);
            out_file << " ";
            write_output_point(out_file, bp.box_target);
        }
        out_file << "\n";
    }

    int obs_count = 0;
    for (int i = 0; i < patrol_actions.size(); ++i) {
        if (patrol_actions[i].kind == MacroActionKind::OBSERVE) ++obs_count;
    }
    out_file << "CHOSEN_OBS " << obs_count << "\n";
    for (int i = 0; i < patrol_actions.size(); ++i) {
        if (patrol_actions[i].kind == MacroActionKind::OBSERVE) {
            out_file << point_to_output_string(patrol_actions[i].observe.view.pos) << "\n";
        }
    }

    int obs_clear_push_count = 0;
    for (int i = 0; i < reference_patrol_actions.size(); ++i) {
        if (reference_patrol_actions[i].kind != MacroActionKind::OBSERVE) continue;
        int start = i;
        while (start > 0 && reference_patrol_actions[start - 1].kind == MacroActionKind::PUSH_BOX) --start;
        obs_clear_push_count += i - start;
    }
    out_file << "OBS_CLEAR_PUSHES " << obs_clear_push_count << "\n";
    for (int i = 0; i < reference_patrol_actions.size(); ++i) {
        if (reference_patrol_actions[i].kind != MacroActionKind::OBSERVE) continue;
        int start = i;
        while (start > 0 && reference_patrol_actions[start - 1].kind == MacroActionKind::PUSH_BOX) --start;
        for (int p = start; p < i; ++p) {
            write_output_point(out_file, reference_patrol_actions[p].box_push.box_start);
            out_file << " ";
            write_output_point(out_file, reference_patrol_actions[p].box_push.box_target);
            out_file << "\n";
        }
    }

    out_file << "PATROL\n";
    for (const auto& s : patrol_out) out_file << s << "\n";

    out_file << "MCU_TRACE\n";
    for (const auto& s : mcu_trace) out_file << s << "\n";

    bool sokoban_path_valid = false;
    if (success && !patrol_failed) {
        const auto& path = solver.get_result_path();
        // 定长路径溢出时 StaticArray 会静默丢弃前段，必须先校验起点再对外报告成功
        sokoban_path_valid = !path.empty() && path[0] == logical_level.player_start;
        if (sokoban_path_valid) {
            write_sokoban_replay_diag(out_file, logical_level, path);
        } else {
            out_file << "SOKOBAN_PATH_INVALID\n";
        }
    }

    out_file << "SOKOBAN\n";
    if (success && !patrol_failed && sokoban_path_valid) {
        const auto& path = solver.get_result_path();
        for (int i = 0; i < path.size(); ++i) {
            out_file << point_to_output_string(path[i]) << "\n";
        }
    } else {
        out_file << "FAILED\n";
    }

    out_file.close();
    return 0;
}
