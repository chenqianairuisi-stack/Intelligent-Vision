/// \file Telemetry.cpp
/// \brief 无线遥测、在线调参和调试命令实现

#include "Telemetry.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "ChassisControl.h"
#include "Icm42688.h"
#include "MotionControl.h"
#include "PoseEstimate.h"
#include "RobotState.h"
#include "Storage.h"
#include "Tracker.h"
#include "Vision.h"
#include "system_config.h"
#include "tuning_config.h"

#include "zf_common_headfile.h"

namespace Subsystem::Telemetry {
namespace {

// ============================================================================
// 模块 1：协议状态与文本工具
// ============================================================================

constexpr std::size_t COMMAND_CAPACITY = 96u;
constexpr std::size_t WHEEL_COUNT = TUNING_WHEEL_COUNT;

char command_buffer[COMMAND_CAPACITY] = {};
std::size_t command_size = 0u;
bool command_overflow = false;
std::uint8_t selected_wheel = static_cast<std::uint8_t>(WHEEL_COUNT);

bool movement_monitoring = false;
ControlMode movement_mode = ControlMode::POINT_TRACKING;

bool semantic_monitoring = false;
int8_t cached_semantic_labels[SystemConfig::MAX_ENTITIES] = {};

const char* skip_spaces(const char* text) {
    while (*text == ' ' || *text == '\t') ++text;
    return text;
}

char upper_ascii(char value) {
    return value >= 'a' && value <= 'z'
               ? static_cast<char>(value - 'a' + 'A')
               : value;
}

bool same_word(const char* lhs, const char* rhs) {
    while (*lhs != '\0' && *rhs != '\0' &&
           upper_ascii(*lhs) == upper_ascii(*rhs)) {
        ++lhs;
        ++rhs;
    }
    return *lhs == '\0' && *rhs == '\0';
}

bool read_word(const char*& text, char* output, std::size_t capacity) {
    text = skip_spaces(text);
    std::size_t length = 0u;
    while ((*text >= 'A' && *text <= 'Z') ||
           (*text >= 'a' && *text <= 'z')) {
        if (length + 1u < capacity) output[length++] = upper_ascii(*text);
        ++text;
    }
    output[length] = '\0';
    text = skip_spaces(text);
    return length != 0u;
}

bool read_float(const char*& text, float& value) {
    text = skip_spaces(text);
    char* end = nullptr;
    value = std::strtof(text, &end);
    if (end == text || !std::isfinite(value)) return false;
    text = skip_spaces(end);
    return true;
}

bool read_int(const char*& text, int& value) {
    text = skip_spaces(text);
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text) return false;
    value = static_cast<int>(parsed);
    text = skip_spaces(end);
    return true;
}

void send_text(const char* text) {
    wireless_uart_send_buffer(
        reinterpret_cast<const uint8_t*>(text),
        static_cast<uint32_t>(std::strlen(text)));
}

void send_format(const char* format, ...) {
    char output[256];
    va_list args;
    va_start(args, format);
    int length = std::vsnprintf(output, sizeof(output), format, args);
    va_end(args);
    if (length <= 0) return;
    if (length >= static_cast<int>(sizeof(output))) {
        length = static_cast<int>(sizeof(output)) - 1;
    }
    wireless_uart_send_buffer(
        reinterpret_cast<const uint8_t*>(output),
        static_cast<uint32_t>(length));
}

void send_error(const char* group, const char* reason) {
    send_format("[%s] ERR: %s\r\n", group, reason);
}

bool debug_mode_active() {
    return App::g_state.game.is_debug_mode || App::g_state.game.is_demo_mode;
}

Pose2D pose_snapshot() {
    const std::uint32_t primask = interrupt_global_disable();
    const Pose2D pose = App::g_state.physical.pose;
    interrupt_global_enable(primask);
    return pose;
}

// ============================================================================
// 模块 2：波形、参数和语义输出
// ============================================================================

void send_wave_csv(const float (&data)[8]) {
    send_format("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                static_cast<double>(data[0]), static_cast<double>(data[1]),
                static_cast<double>(data[2]), static_cast<double>(data[3]),
                static_cast<double>(data[4]), static_cast<double>(data[5]),
                static_cast<double>(data[6]), static_cast<double>(data[7]));
}

const char* wheel_name(std::size_t index) {
    constexpr const char* names[WHEEL_COUNT] = {"LF", "LB", "RF", "RB"};
    return index < WHEEL_COUNT ? names[index] : "ALL";
}

void send_tune_snapshot() {
    send_format("[TUNE] wheel=%u(%s)\r\n",
                static_cast<unsigned>(selected_wheel), wheel_name(selected_wheel));
    send_format("[TUNE] MD=%.3f MV=%.3f MA=%.3f AV=%.3f AA=%.3f BL=%.3f\r\n",
                static_cast<double>(tune.dynamics.max_duty),
                static_cast<double>(tune.dynamics.max_vel),
                static_cast<double>(tune.dynamics.max_acc),
                static_cast<double>(tune.dynamics.max_ang_vel),
                static_cast<double>(tune.dynamics.max_ang_acc),
                static_cast<double>(tune.dynamics.brake_limit));
    // 上车调纵向时要能一眼看到实际生效的刹车能力：brake_limit×max_acc 与 BC 取小
    send_format("[TUNE] BC=%.1f brake_acc=%.1f | VN=%.0f VF=%.2f VG=%.2f VS=%.2f VP=%.2f VJ=%.2f\r\n",
                static_cast<double>(tune.dynamics.brake_acc_ceiling),
                static_cast<double>(std::min(tune.dynamics.max_acc * tune.dynamics.brake_limit,
                                             tune.dynamics.brake_acc_ceiling)),
                static_cast<double>(tune.vision_long.enable),
                static_cast<double>(tune.vision_long.freeze_floor_cm),
                static_cast<double>(tune.vision_long.latency_window_gain),
                static_cast<double>(tune.vision_long.max_step_cm),
                static_cast<double>(tune.vision_long.push_max_step_cm),
                static_cast<double>(tune.vision_long.reject_dist_cm));
    send_format("[TUNE] VE=%.0f VA=%.2f VD=%.1f SM=%.2f SX=%.2f\r\n",
                static_cast<double>(tune.vision_long.scale_learn_enable),
                static_cast<double>(tune.vision_long.scale_learn_alpha),
                static_cast<double>(tune.vision_long.scale_sample_min_cm),
                static_cast<double>(tune.vision_long.scale_min),
                static_cast<double>(tune.vision_long.scale_max));
    send_format("[TUNE] VR=%.2f HS=%.2f HG=%.2f\r\n",
                static_cast<double>(tune.tracker.vision_reject_dist),
                static_cast<double>(tune.vision_lateral.max_step_cm),
                static_cast<double>(tune.vision_lateral.gain));
    for (std::size_t index = 0u; index < WHEEL_COUNT; ++index) {
        const auto& wheel = tune.wheels[index];
        send_format("[%s] KP=%.3f KI=%.3f KD=%.4f KV=%.3f KA=%.3f KB=%.3f KS=%.3f\r\n",
                    wheel_name(index),
                    static_cast<double>(wheel.pid.kp),
                    static_cast<double>(wheel.pid.ki),
                    static_cast<double>(wheel.pid.kd),
                    static_cast<double>(wheel.kv),
                    static_cast<double>(wheel.ka),
                    static_cast<double>(wheel.kb),
                    static_cast<double>(wheel.ks));
    }
}

void send_tune_list() {
    for (std::size_t index = 0u; index < TuningRegistry::screen_count(); ++index) {
        const auto& item = TuningRegistry::screen_param(index);
        send_format("[PARAM] %s=%.5f range[%.3f,%.3f]\r\n",
                    item.key, static_cast<double>(*item.value),
                    static_cast<double>(item.minimum),
                    static_cast<double>(item.maximum));
    }
}

void dump_semantic_cache() {
    char output[256];
    int offset = std::snprintf(output, sizeof(output), "[SEMANTIC] ");
    for (int index = 0; index < SystemConfig::MAX_ENTITIES; ++index) {
        if (offset >= 0 && offset < static_cast<int>(sizeof(output)) - 16) {
            offset += std::snprintf(output + offset, sizeof(output) - offset,
                                    "ID%d:%d ", index,
                                    App::g_state.vision.semantic_labels[index]);
        }
    }
    if (offset >= 0 && offset < static_cast<int>(sizeof(output)) - 2) {
        output[offset++] = '\r';
        output[offset++] = '\n';
        wireless_uart_send_buffer(reinterpret_cast<const uint8_t*>(output),
                                  static_cast<uint32_t>(offset));
    }
}

void update_semantic_monitor() {
    if (!semantic_monitoring) return;

    bool changed = false;
    for (int index = 0; index < SystemConfig::MAX_ENTITIES; ++index) {
        const int8_t current = App::g_state.vision.semantic_labels[index];
        if (current != cached_semantic_labels[index]) {
            cached_semantic_labels[index] = current;
            changed = true;
        }
    }
    if (changed) dump_semantic_cache();
}

// ============================================================================
// 模块 3：调试运动与参数写入
// ============================================================================

void start_movement_monitoring(ControlMode mode) {
    movement_monitoring = true;
    movement_mode = mode;
    send_text("[MOVE] START\r\n");
}

void stop_at_current_pose() {
    movement_monitoring = false;
    Algorithm::Tracker::track_point(pose_snapshot());
}

void check_movement_completion() {
    if (!movement_monitoring) return;

    bool done = false;
    if (movement_mode == ControlMode::AUTO_TRACKING) {
        done = App::g_state.control.tracker_state == TrackerState::FINISHED;
    } else {
        const Pose2D pose = pose_snapshot();
        const Pose2D target = App::g_state.control.current_target;
        const float dx = target.x - pose.x;
        const float dy = target.y - pose.y;
        float dyaw = target.yaw - pose.yaw;
        while (dyaw > 180.0f) dyaw -= 360.0f;
        while (dyaw < -180.0f) dyaw += 360.0f;
        done = std::sqrt(dx * dx + dy * dy) < 0.8f &&
               std::fabs(dyaw) < 2.0f && App::g_state.physical.is_stopped;
    }

    if (done) {
        movement_monitoring = false;
        send_text("[MOVE] DONE\r\n");
    }
}

TuningRegistry::SetResult set_wheel_param(char code, float value) {
    const std::size_t first = selected_wheel < WHEEL_COUNT ? selected_wheel : 0u;
    const std::size_t last = selected_wheel < WHEEL_COUNT
                                 ? selected_wheel + 1u : WHEEL_COUNT;
    for (std::size_t index = first; index < last; ++index) {
        char key[3] = {static_cast<char>('0' + index), upper_ascii(code), '\0'};
        const auto result = TuningRegistry::set_by_key(key, value);
        if (result != TuningRegistry::SetResult::OK) return result;
    }
    return TuningRegistry::SetResult::OK;
}

bool is_wheel_param_code(char code) {
    const char normalized = upper_ascii(code);
    return normalized == 'P' || normalized == 'I' || normalized == 'D' ||
           normalized == 'V' || normalized == 'A' || normalized == 'B' ||
           normalized == 'S';
}

void process_tune_assignment(const char* text) {
    text = skip_spaces(text);
    if (text[0] == '\0' || text[1] == '\0') {
        send_error("TUNE", "bad key");
        return;
    }

    char key[3] = {upper_ascii(text[0]), upper_ascii(text[1]), '\0'};
    text = skip_spaces(text + 2);
    if (*text == ':' || *text == '=') text = skip_spaces(text + 1);

    float value = 0.0f;
    if (!read_float(text, value) || *text != '\0') {
        send_error("TUNE", "bad value");
        return;
    }

    TuningRegistry::SetResult result = TuningRegistry::SetResult::UNKNOWN_KEY;
    if (key[0] == 'W' && key[1] == 'H') {
        const int wheel = static_cast<int>(value);
        if (std::fabs(value - static_cast<float>(wheel)) < 0.001f &&
            wheel >= 0 && wheel <= static_cast<int>(WHEEL_COUNT)) {
            selected_wheel = static_cast<std::uint8_t>(wheel);
            result = TuningRegistry::SetResult::OK;
        } else {
            result = TuningRegistry::SetResult::OUT_OF_RANGE;
        }
    } else if (key[0] == 'K' && is_wheel_param_code(key[1])) {
        result = set_wheel_param(key[1], value);
    } else {
        result = TuningRegistry::set_by_key(key, value);
    }

    if (result == TuningRegistry::SetResult::OK) {
        send_format("[TUNE] %s=%.5f\r\n", key, static_cast<double>(value));
    } else if (result == TuningRegistry::SetResult::OUT_OF_RANGE) {
        send_error("TUNE", "value out of range");
    } else {
        send_error("TUNE", "unknown key");
    }
}

// ============================================================================
// 模块 4：控制命令解析
// ============================================================================

void process_tel_command(const char* text) {
    char operation[12] = {};
    if (!read_word(text, operation, sizeof(operation))) {
        send_error("TEL", "use Q, MODE <0..4>, or OFF");
    } else if (same_word(operation, "Q")) {
        send_format("[TEL] mode=%d (0=wheel 1=pose 2=imu 3=latency 4=mileage)\r\n",
                    App::g_state.debug.telemetry_mode);
    } else if (same_word(operation, "OFF")) {
        App::g_state.debug.telemetry_mode = -1;
        send_text("[TEL] OFF\r\n");
    } else if (same_word(operation, "MODE")) {
        int mode = -1;
        if (!read_int(text, mode) || *text != '\0' || mode < 0 || mode > 4) {
            send_error("TEL", "mode must be 0..4");
        } else {
            App::g_state.debug.telemetry_mode = mode;
            send_format("[TEL] mode=%d\r\n", mode);
        }
    } else {
        send_error("TEL", "unknown op");
    }
}

void process_tune_command(const char* text) {
    char operation[12] = {};
    if (!read_word(text, operation, sizeof(operation))) {
        send_error("TUNE", "use ON, OFF, Q, or LIST");
    } else if (same_word(operation, "Q")) {
        send_tune_snapshot();
    } else if (same_word(operation, "LIST")) {
        send_tune_list();
    } else if (!debug_mode_active()) {
        send_error("TUNE", "debug mode only");
    } else if (same_word(operation, "ON")) {
        selected_wheel = static_cast<std::uint8_t>(WHEEL_COUNT);
        stop_at_current_pose();
        App::g_state.debug.telemetry_mode = 0;
        send_text("[TUNE] ON\r\n");
        send_tune_snapshot();
    } else if (same_word(operation, "OFF")) {
        stop_at_current_pose();
        App::g_state.debug.telemetry_mode = -1;
        send_text("[TUNE] OFF\r\n");
    } else {
        send_error("TUNE", "unknown op");
    }
}

void process_move_command(const char* text) {
    if (!debug_mode_active()) {
        send_error("MOVE", "debug mode only");
        return;
    }

    char operation[12] = {};
    if (!read_word(text, operation, sizeof(operation))) {
        send_error("MOVE", "use POS, REL, HOME, or STOP");
        return;
    }

    Pose2D target = pose_snapshot();
    if (same_word(operation, "HOME") || same_word(operation, "STOP")) {
        stop_at_current_pose();
        send_text("[MOVE] STOP\r\n");
    } else if (same_word(operation, "POS")) {
        if (!read_float(text, target.x) || !read_float(text, target.y) ||
            !read_float(text, target.yaw) || *text != '\0') {
            send_error("MOVE", "use POS <x_cm> <y_cm> <yaw_deg>");
            return;
        }
        Algorithm::Tracker::track_point(target);
        start_movement_monitoring(ControlMode::POINT_TRACKING);
    } else if (same_word(operation, "REL")) {
        float dx = 0.0f;
        float dy = 0.0f;
        float dyaw = 0.0f;
        if (!read_float(text, dx) || !read_float(text, dy) ||
            !read_float(text, dyaw) || *text != '\0') {
            send_error("MOVE", "use REL <dx_cm> <dy_cm> <dyaw_deg>");
            return;
        }
        target.x += dx;
        target.y += dy;
        target.yaw += dyaw;
        Algorithm::Tracker::track_point(target);
        start_movement_monitoring(ControlMode::POINT_TRACKING);
    } else {
        send_error("MOVE", "unknown op");
    }
}

void process_path_command(const char* text) {
    if (!debug_mode_active()) {
        send_error("PATH", "debug mode only");
        return;
    }

    char operation[12] = {};
    if (!read_word(text, operation, sizeof(operation))) {
        send_error("PATH", "use CLEAR, ADD, or EXEC");
    } else if (same_word(operation, "CLEAR")) {
        auto& plan = App::g_state.planning;
        plan.physical_path.clear();
        plan.force_stop_at_wp.clear();
        plan.current_wp_idx = 0u;
        send_text("[PATH] CLEAR\r\n");
    } else if (same_word(operation, "ADD")) {
        float x = 0.0f;
        float y = 0.0f;
        if (!read_float(text, x) || !read_float(text, y) || *text != '\0') {
            send_error("PATH", "use ADD <x_cm> <y_cm>");
        } else {
            auto& plan = App::g_state.planning;
            if (plan.physical_path.size() >= SystemConfig::MAX_PATH_LENGTH) {
                send_error("PATH", "path full");
            } else {
                plan.physical_path.push_back({x, y});
                plan.force_stop_at_wp.push_back(0u);
                send_format("[PATH] ADD %.2f %.2f\r\n",
                            static_cast<double>(x), static_cast<double>(y));
            }
        }
    } else if (same_word(operation, "EXEC")) {
        App::g_state.control.mode = ControlMode::AUTO_TRACKING;
        App::g_state.control.tracker_state = TrackerState::TRACKING;
        start_movement_monitoring(ControlMode::AUTO_TRACKING);
    } else {
        send_error("PATH", "unknown op");
    }
}

void process_vision_command(const char* text) {
    char operation[12] = {};
    if (!read_word(text, operation, sizeof(operation))) {
        send_error("VISION", "use MAP, POSE, or LAG ON/OFF");
    } else if (same_word(operation, "MAP")) {
        Subsystem::Vision::request_map_ART1();
        send_text("[VISION] MAP REQUESTED\r\n");
    } else if (same_word(operation, "POSE")) {
        Subsystem::Vision::request_pose_ART1();
        send_text("[VISION] POSE REQUESTED\r\n");
    } else if (same_word(operation, "LAG")) {
        char state[8] = {};
        if (!read_word(text, state, sizeof(state)) ||
            (!same_word(state, "ON") && !same_word(state, "OFF"))) {
            send_error("VISION", "use LAG ON or OFF");
        } else {
            tune.latency.enable_estimation = same_word(state, "ON") ? 1.0f : 0.0f;
            send_text(tune.latency.enable_estimation > 0.5f
                          ? "[VISION] LAG ON\r\n" : "[VISION] LAG OFF\r\n");
        }
    } else {
        send_error("VISION", "unknown op");
    }
}

void process_semantic_command(const char* text) {
    char operation[8] = {};
    if (!read_word(text, operation, sizeof(operation))) {
        send_error("SEM", "use ON, OFF, or Q");
    } else if (same_word(operation, "ON")) {
        semantic_monitoring = true;
        std::memset(cached_semantic_labels, -2, sizeof(cached_semantic_labels));
        send_text("[SEM] ON\r\n");
    } else if (same_word(operation, "OFF")) {
        semantic_monitoring = false;
        send_text("[SEM] OFF\r\n");
    } else if (same_word(operation, "Q")) {
        dump_semantic_cache();
    } else {
        send_error("SEM", "unknown op");
    }
}

void process_command(const char* text) {
    char group[12] = {};
    if (!read_word(text, group, sizeof(group))) return;

    if (same_word(group, "TEL")) process_tel_command(text);
    else if (same_word(group, "TUNE")) process_tune_command(text);
    else if (same_word(group, "MOVE")) process_move_command(text);
    else if (same_word(group, "PATH")) process_path_command(text);
    else if (same_word(group, "VISION")) process_vision_command(text);
    else if (same_word(group, "SEM")) process_semantic_command(text);
    else if (same_word(group, "SAVE")) {
        if (!debug_mode_active()) send_error("SAVE", "debug mode only");
        else send_text(Storage::save_params() ? "[SAVE] OK\r\n" : "[SAVE] FAILED\r\n");
    } else if (same_word(group, "LOAD")) {
        if (!debug_mode_active()) send_error("LOAD", "debug mode only");
        else send_text(Storage::load_params() ? "[LOAD] OK\r\n" : "[LOAD] FAILED\r\n");
    } else if (same_word(group, "RESET")) {
        if (!debug_mode_active()) send_error("RESET", "debug mode only");
        else {
            Storage::reset_params();
            send_text("[RESET] OK\r\n");
        }
    } else {
        send_error("CMD", "unknown group");
    }
}

void finish_command() {
    if (command_overflow) {
        send_error("CMD", "line too long");
    } else if (command_size != 0u) {
        command_buffer[command_size] = '\0';
        const char* line = skip_spaces(command_buffer);
        if (*line == '!') process_command(line + 1);
        else if (*line != '\0') process_tune_assignment(line);
    }
    command_size = 0u;
    command_overflow = false;
}

} // namespace

// ============================================================================
// 对外接口
// ============================================================================

void init() {
    wireless_uart_init();
    command_size = 0u;
    command_overflow = false;
    selected_wheel = static_cast<std::uint8_t>(WHEEL_COUNT);
    App::g_state.debug.telemetry_mode = -1;
}

void receive_and_parse_task() {
    uint8_t bytes[32];
    const uint32_t count = wireless_uart_read_buffer(bytes, sizeof(bytes));
    for (uint32_t index = 0u; index < count; ++index) {
        const uint8_t byte = bytes[index];
        if (byte == '\r' || byte == '\n') {
            finish_command();
        } else if (!command_overflow) {
            if (command_size + 1u < COMMAND_CAPACITY) {
                command_buffer[command_size++] = static_cast<char>(byte);
            } else {
                command_overflow = true;
            }
        }
    }
    check_movement_completion();
}

void send_wave_data() {
    float data[8] = {};
    const int mode = App::g_state.debug.telemetry_mode;
    if (mode == 0) {
        const WheelSpeed4 target = Subsystem::Chassis::get_target_wheel_speeds();
        const std::uint32_t primask = interrupt_global_disable();
        const WheelSpeed4 current = App::g_state.physical.current_wheel_speed;
        interrupt_global_enable(primask);
        data[0] = target.lf; data[1] = target.lb;
        data[2] = target.rf; data[3] = target.rb;
        data[4] = current.lf; data[5] = current.lb;
        data[6] = current.rf; data[7] = current.rb;
        send_wave_csv(data);
    } else if (mode == 1) {
        const std::uint32_t primask = interrupt_global_disable();
        const Pose2D target = App::g_state.control.current_target;
        const Pose2D fused = App::g_state.physical.pose;
        const Pose2D vision = App::g_state.vision.art1_pose;
        interrupt_global_enable(primask);
        const Pose2D encoder = Subsystem::PoseEstimator::get_encoder_pose();
        data[0] = target.x; data[1] = fused.x;
        data[2] = vision.x; data[3] = encoder.x;
        data[4] = target.y; data[5] = fused.y;
        data[6] = vision.y; data[7] = encoder.y;
        send_wave_csv(data);
    } else if (mode == 2) {
        const auto& probes = Subsystem::PoseEstimator::get_debug_probes();
        data[0] = probes.pitch_acc;
        data[1] = probes.pitch_gyro;
        data[2] = probes.pitch_mahony;
        data[3] = probes.kp_adaptive;
        data[4] = probes.acc_norm;
        data[5] = App::g_state.physical.yaw_rate;
        data[6] = imu_icm42688.data.gyro_x;
        data[7] = App::g_state.physical.pose.yaw;
        send_wave_csv(data);
    } else if (mode == 3) {
        const auto& latency = Subsystem::PoseEstimator::get_vision_latency_debug();
        const Pose2D pose = pose_snapshot();
        data[0] = latency.est_raw_l_ms;
        data[1] = latency.est_filt_l_ms;
        data[2] = latency.used_l_ms;
        data[3] = static_cast<float>(latency.est_pending_count);
        data[4] = latency.compensated_pose.x - pose.x;
        data[5] = latency.compensated_pose.y - pose.y;
        data[6] = latency.correction_x;
        data[7] = latency.correction_y;
        send_wave_csv(data);
    } else if (mode == 4) {
        const auto& mileage = Subsystem::PoseEstimator::get_mileage_scale_debug();
        data[0] = mileage.scale_x;
        data[1] = mileage.scale_y;
        data[2] = mileage.last_sample;
        data[3] = mileage.last_sample_distance_cm;
        data[4] = static_cast<float>(mileage.last_axis);
        data[5] = static_cast<float>(mileage.consistent_x);
        data[6] = static_cast<float>(mileage.consistent_y);
        data[7] = mileage.anchor_valid ? 1.0f : 0.0f;
        send_wave_csv(data);
    }

    update_semantic_monitor();
}

void log_vision_calibration(float vision_x, float vision_y, float vision_yaw,
                            float odom_x, float odom_y, float odom_yaw,
                            bool accepted) {
    float yaw_error = std::fabs(vision_yaw - odom_yaw);
    if (yaw_error > 180.0f) yaw_error = 360.0f - yaw_error;
    send_format("[VIS_CALIB] %s Vis(%.1f,%.1f,%.1f) Odom(%.1f,%.1f,%.1f) Err(%.1f,%.1f,%.1f)\r\n",
                accepted ? "ACCEPT" : "REJECT",
                static_cast<double>(vision_x), static_cast<double>(vision_y),
                static_cast<double>(vision_yaw), static_cast<double>(odom_x),
                static_cast<double>(odom_y), static_cast<double>(odom_yaw),
                static_cast<double>(std::fabs(vision_x - odom_x)),
                static_cast<double>(std::fabs(vision_y - odom_y)),
                static_cast<double>(yaw_error));
}

} // namespace Subsystem::Telemetry
