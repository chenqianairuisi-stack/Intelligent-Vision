#include "Telemetry.h"
#include "RobotState.h"
#include "tuning_config.h"
#include "system_config.h"
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "CoreScheduler.h"
#include "Vision.h"
#include "MotionControl.h"
#include "Tracker.h"
#include "PoseEstimate.h"

#include "zf_common_headfile.h"
#include "Encoder.h"
#include "UartComm.h"


namespace Subsystem::Telemetry {

// ====================================================================
// 内部变量
// ====================================================================
namespace {
    #pragma pack(push, 1)
    struct VofaJustFloat {
        uint8_t head = 0xAA;
        float data_1 = 0; float data_2 = 0; float data_3 = 0; 
        float data_4 = 0; float data_5 = 0; float data_6 = 0;
        uint8_t tail = 0xBB;
    };
    #pragma pack(pop)
    VofaJustFloat tx_packet;

    char rx_cmd_buf[64];
    uint8_t rx_idx = 0;

    // --- 运动时间监测 ---
    bool is_timing_movement = false;   // 测时器状态标志
    uint32_t movement_start_time = 0;  // 运动开始的系统时间戳 (ms)
    ControlMode current_timing_mode = ControlMode::POINT_TRACKING;  // 当前测时器关联的控制模式

    // --- 语义监测 ---
    bool is_monitoring_semantics = false;                            // 语义监测开关
    int8_t cached_semantic_labels[SystemConfig::MAX_ENTITIES] = {0}; // 语义缓存，用于比对差异

    // --- 内部函数声明 ---
    void start_movement_timing(ControlMode mode);  // 触发测时器
    void check_movement_completion();              // 在后台循环中检查测时器状态，计算并发送结果
    void dump_semantic_cache();                    // 发送语义缓存内容
    void execute_command(const char* cmd);         // 命令解析函数声明
}


// ====================================================================
// 公共接口
// ====================================================================

/// \brief 初始化无线遥测串口
void init() {
    wireless_uart_init();  
}

/// \brief 输出一次视觉标定日志
/// \param v_x 视觉 X
/// \param v_y 视觉 Y
/// \param v_yaw 视觉 yaw
/// \param o_x 里程计 X
/// \param o_y 里程计 Y
/// \param o_yaw 里程计 yaw
/// \param accepted 本次标定是否被接受
///
void log_vision_calibration(float v_x, float v_y, float v_yaw, 
                            float o_x, float o_y, float o_yaw, bool accepted) {
    char msg[128];
    // 计算偏差
    float dx = std::abs(v_x - o_x);
    float dy = std::abs(v_y - o_y);
    float dyaw = std::abs(v_yaw - o_yaw);
    if (dyaw > 180.0f) dyaw = 360.0f - dyaw;

    snprintf(msg, sizeof(msg),
             "[VIS_CALIB] %s | Vis(%.1f, %.1f, %.1f) Odom(%.1f, %.1f, %.1f) Err(%.1f, %.1f, %.1f)\r\n",
             accepted ? "ACCEPT" : "REJECT",
             v_x, v_y, v_yaw, o_x, o_y, o_yaw, dx, dy, dyaw);
             
    wireless_uart_send_buffer((uint8_t*)msg, strlen(msg));
}

/// \brief 周期发送遥测数据
///
/// \details
/// 根据 telemetry_mode 输出不同波形数据，同时在语义监测开启时增量发送语义缓存变化
///
void send_wave_data() {

    // 1. 常规高频波形推送
    if (App::g_state.debug.telemetry_mode != -1) {
        if (App::g_state.debug.telemetry_mode == 0) {
            // 【模式 0】：底盘动力学监控
            const auto& current_pose = App::g_state.physical.pose;
            const auto& wheel_speed = App::g_state.physical.current_wheel_speed;
            Pose2D  target_pos  = App::g_state.control.current_target;  
            
            Velocity2D avg_speed = Algorithm::Motion::Kinematics::forward(
                wheel_speed.lf, wheel_speed.lb,
                wheel_speed.rf, wheel_speed.rb
            );
            float avg_speed_mag = std::sqrt(avg_speed.vx * avg_speed.vx + avg_speed.vy * avg_speed.vy);

            tx_packet.data_1 = 0;            
            tx_packet.data_2 = avg_speed_mag;              
            tx_packet.data_3 = target_pos.x;               
            tx_packet.data_4 = current_pose.x;
            tx_packet.data_5 = target_pos.y;               
            tx_packet.data_6 = current_pose.y;
            
        } 
        else if (App::g_state.debug.telemetry_mode == 1) {
            // 【模式 1】：定位数据监控
            const auto& current_pose = App::g_state.physical.pose;
            const auto& vision_pose = App::g_state.vision.art1_pose;

            tx_packet.data_1 = current_pose.x;
            tx_packet.data_2 = current_pose.y;
            tx_packet.data_3 = vision_pose.x;
            tx_packet.data_4 = vision_pose.y;
            tx_packet.data_5 = 0;
            tx_packet.data_6 = 0;
        }
        else if (App::g_state.debug.telemetry_mode == 2) {
            // 【模式 2】：IMU 四元数融合监控
            const auto& probes = Subsystem::PoseEstimator::get_debug_probes();
            const auto& cur_pose = App::g_state.physical.pose;

            tx_packet.data_1 = probes.pitch_acc;     // 通道1: 假信号(红线)
            tx_packet.data_2 = probes.pitch_gyro;    // 通道2: 积分信号(蓝线)
            tx_packet.data_3 = probes.pitch_mahony;  // 通道3: 融合结果(绿线)
            
            // 为了在同一张图里看清，把 Kp(0~1) 放大 10 倍显示
            tx_packet.data_4 = probes.kp_adaptive * 10.0f; // 通道4: 动态 Kp(放大10倍) 
            
            // 加速度模长减去 1G，再放大 10 倍，方便和 0 度基准线对比
            tx_packet.data_5 = (probes.acc_norm - 1.0f) * 10.0f; 

            // 附带监控我们最关心的 Yaw，看看它有没有漂移
            tx_packet.data_6 = cur_pose.yaw;
        }
        else if (App::g_state.debug.telemetry_mode == 3) {
            // 【模式 3】：视觉延时估计 & 纯视觉定位监控
            const auto& lat = Subsystem::PoseEstimator::get_vision_latency_debug();
            const auto& cur_pose = App::g_state.physical.pose;

            tx_packet.data_1 = lat.est_raw_l_ms;    // 通道1: 单次配对原始 L (ms)
            tx_packet.data_2 = lat.est_filt_l_ms;   // 通道2: 滤波后 L (ms)
            tx_packet.data_3 = lat.used_l_ms;       // 通道3: 本帧实际采用 L (回退梯结果)
            // 通道4: 待配对编码器拐点数 (放大10倍便于和 L 同图区分)
            tx_packet.data_4 = (float)lat.est_pending_count * 10.0f;
            // 通道5/6: 补偿后视觉 X/Y 与当前 pose X/Y 的偏差，看纯视觉闭环贴合程度
            tx_packet.data_5 = lat.compensated_pose.x - cur_pose.x;
            tx_packet.data_6 = lat.compensated_pose.y - cur_pose.y;
        }
        else if (App::g_state.debug.telemetry_mode == 4) {
            // 【模式 4】：Stanley 横纠贴线监控
            const auto& pose = App::g_state.physical.pose;
            const auto& ctrl = App::g_state.control;
            float sx = ctrl.segment_start.x, sy = ctrl.segment_start.y;
            float dx = ctrl.current_target.x - sx;
            float dy = ctrl.current_target.y - sy;
            float len = sqrtf(dx * dx + dy * dy);
            float e_ct = 0.0f, s_rem = 0.0f;
            if (len > 1e-3f) {
                float ax = dx / len, ay = dy / len;       // 沿轨单位向量
                float nx = -ay, ny = ax;                  // 法向
                e_ct = (pose.x - sx) * nx + (pose.y - sy) * ny;          // 横向偏差 cm
                s_rem = (ctrl.current_target.x - pose.x) * ax +
                        (ctrl.current_target.y - pose.y) * ay;           // 沿轨剩余 cm
            }
            tx_packet.data_1 = e_ct;                  // 通道1: 横向偏差 (压线目标=0)
            tx_packet.data_2 = s_rem;                 // 通道2: 沿轨剩余距离
            tx_packet.data_3 = pose.x;                // 通道3: pose X
            tx_packet.data_4 = pose.y;                // 通道4: pose Y
            tx_packet.data_5 = ctrl.current_target.x; // 通道5: 目标 X
            tx_packet.data_6 = ctrl.current_target.y; // 通道6: 目标 Y
        }
        wireless_uart_send_buffer((uint8*)&tx_packet, sizeof(VofaJustFloat));
    }

    // 2. 增量式语义监测 (仅在监测开启，且数据有实质变更时发送)
    if (is_monitoring_semantics) {
        bool has_new_info = false;
        const auto& current_labels = App::g_state.vision.semantic_labels;
        
        // 极速内存比对 (M7 执行这段循环耗时 < 0.1us)
        for (int i = 0; i < SystemConfig::MAX_ENTITIES; ++i) {
            if (current_labels[i] != cached_semantic_labels[i]) {
                has_new_info = true;
                cached_semantic_labels[i] = current_labels[i]; // 更新影子缓存
            }
        }
        
        // 只有当捕捉到“标签翻转”的跳变沿时，才组合字符串下发
        if (has_new_info) {
            dump_semantic_cache();
        }
    }
    
}

/// \brief 接收并解析上位机指令
///
/// \details
/// 无线串口按行拼装命令，收到换行后调用 execute_command
/// 每次轮询末尾也会检查调试运动是否完成并回传耗时
///
void receive_and_parse_task() {
    uint8_t temp_buf[32]; 
    
    // 读取接收缓冲区，返回实际读取到的数据个数
    uint8_t data_len = wireless_uart_read_buffer(temp_buf, 32);
    
    if (data_len > 0) {
        // 遍历读出的每一个字节，拼装字符串命令
        for (uint8_t i = 0; i < data_len; i++) {
            uint8_t byte = temp_buf[i];
            
            // 遇到回车或换行符，认为一条命令接收完毕
            if (byte == '\n' || byte == '\r') {
                if (rx_idx > 0) {
                    rx_cmd_buf[rx_idx] = '\0';     // 加上字符串结束符
                    
                    execute_command(rx_cmd_buf);   // 解析并执行命令
                    rx_idx = 0;                    // 清空缓冲区，准备下一次接收
                }
            } else {
                if (rx_idx < sizeof(rx_cmd_buf) - 1) {
                    rx_cmd_buf[rx_idx++] = byte;
                }
            }
        }
    }

    // 每次轮询解析完串口数据后，顺便检查一下运动是否已经完成
    check_movement_completion(); 
}

// ====================================================================
// 内部实现细节：命令解析与执行
// ====================================================================
namespace {

    /// \brief 启动一次运动耗时统计
    /// \param mode 本次运动使用的控制模式
    ///
    void start_movement_timing(ControlMode mode) {
        is_timing_movement = true;
        current_timing_mode = mode;
        movement_start_time = Core::Scheduler::get_sys_tick_ms(); 
        
        const char* msg = "[SYS] Movement Started.\r\n";
        wireless_uart_send_buffer((uint8_t*)msg, strlen(msg));
    }

    /// \brief 检查当前运动是否完成并输出耗时
    ///
    /// \details
    /// AUTO_TRACKING 直接使用 Tracker 状态，POINT_TRACKING 通过位置、角度和停稳状态综合判断
    ///
    void check_movement_completion() {
        if (!is_timing_movement) return;

        bool is_done = false;
        const auto& ctrl = App::g_state.control;
        const auto& phys = App::g_state.physical;

        if (current_timing_mode == ControlMode::AUTO_TRACKING) {
            // 路径循迹模式：听从 Tracker 的状态报告
            if (ctrl.tracker_state == TrackerState::FINISHED) {
                is_done = true;
            }
        } else {
            // 单点调试模式：根据物理状态反馈闭环判断
            float dx = ctrl.current_target.x - phys.pose.x;
            float dy = ctrl.current_target.y - phys.pose.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            
            // 航向误差归一化 [-180, 180]
            float dyaw = ctrl.current_target.yaw - phys.pose.yaw;
            while (dyaw > 180.0f) dyaw -= 360.0f;
            while (dyaw < -180.0f) dyaw += 360.0f;

            // 严苛结束条件：位置误差 < 0.8cm，角度误差 < 2度，且底层速度环判定彻底静止
            if (dist < 0.8f && std::abs(dyaw) < 2.0f && phys.is_stopped) {
                is_done = true;
            }
        }

        // 触发完成，结算耗时
        if (is_done) {
            uint32_t elapsed_ms = Core::Scheduler::get_sys_tick_ms() - movement_start_time;
            is_timing_movement = false;
            
            char msg[64];
            int len = snprintf(msg, sizeof(msg), "[SYS] Target Reached! Time: %lu ms\r\n", elapsed_ms);
            wireless_uart_send_buffer((uint8_t*)msg, len);
        }
    }

    /// \brief 将当前语义缓存池发送给上位机
    ///
    /// \details
    /// 字符串长度受 dump_buf 限制，追加时预留余量防止越界
    ///
    void dump_semantic_cache() {
        static char dump_buf[256];
        const auto& labels = App::g_state.vision.semantic_labels;
        
        // 组装前缀
        int offset = snprintf(dump_buf, sizeof(dump_buf), "[SEMANTIC_DUMP] ");
        
        // 遍历整个语义池，将每个实体的 ID 和标签值追加到字符串中
        for (int i = 0; i < SystemConfig::MAX_ENTITIES; ++i) {
            // 安全追加字符串，防止越界
            if (offset < sizeof(dump_buf) - 16) {
                offset += snprintf(dump_buf + offset, sizeof(dump_buf) - offset, 
                                    "ID%d:%d ", i, labels[i]);
            }
        }
        
        // 添加回车换行，方便 VOFA+ 终端显示
        offset += snprintf(dump_buf + offset, sizeof(dump_buf) - offset, "\r\n");
        
        // 阻塞/异步推入发送缓冲
        wireless_uart_send_buffer(reinterpret_cast<uint8_t*>(dump_buf), offset);
    }

    /// \brief 解析并执行上位机字符串命令
    /// \param cmd 以 ! 开头的一行命令
    ///
    /// \details
    /// 支持参数调节、视觉请求、路径调试和点位移动
    /// 格式示例：!S Q 1.5 表示设置 yaw Kp，!M P x y yaw 表示移动到绝对位姿
    ///
    void execute_command(const char* cmd) {

        if (cmd[0] != '!') return;     // 命令必须以 ! 开头
        char type = cmd[1];            // 主类型: 'S' 设置参数, 'M' 移动指令
        char sub  = cmd[2];            // 子类型
        float value = atof(&cmd[3]);   // 把后面的字符串转为浮点数 (跳过空格自动处理)

        const auto& cur_pose = App::g_state.physical.pose;
        Point2D cur_pos = {cur_pose.x, cur_pose.y};
        float cur_yaw_deg = cur_pose.yaw;

        switch (type) {
            case 'S':  // 参数设置指令
                switch (sub) {
                    case 'Q': tune.pid_yaw.kp = value; break;
                    case 'W': tune.pid_yaw.ki = value; break;
                    case 'E': tune.pid_yaw.kd = value; break;
                    case 'R': tune.pid_speed.kp = value; break;
                    case 'T': tune.pid_speed.ki = value; break;
                    case 'Y': tune.pid_speed.kd = value; break;
                    case 'U': tune.ff.kv = value; break;
                    case 'I': tune.ff.ka = value; break;
                    case 'O': tune.ff.k_stiction = value; break;
                    
                    case 'A': tune.dynamics.max_duty = value; break;
                    case 'S':
                        tune.dynamics.max_vel = value;
                        (void)TuningDefaults::clamp_if_outside(tune.dynamics.max_vel,
                                                                TuningDefaults::MIN_DYNAMICS_MAX_VEL,
                                                                TuningDefaults::MAX_DYNAMICS_MAX_VEL,
                                                                TuningDefaults::DEFAULT_DYNAMICS_MAX_VEL);
                        break;
                    case 'D': tune.dynamics.max_acc = value; break;
                    case 'F': tune.dynamics.max_ang_vel = value; break;
                    case 'H': tune.dynamics.max_ang_acc = value; break;
                    case 'J': tune.dynamics.kinematic_gain_x = value; break;
                    case 'K': tune.dynamics.kinematic_gain_y = value; break;
                    case 'L': tune.dynamics.brake_limit = value; break;

                    case 'Z': tune.tracker.reach_radius = value; break;
                    case 'X': tune.tracker.reach_radius_min = value; break;
                    // 过弯和视觉校正常用参数，便于现场用无线串口快速调试
                    case 'B': tune.tracker.corner_pass_speed = value; break;
                    case 'N': tune.tracker.corner_switch_window = value; break;
                    case 'M': tune.tracker.corner_line_tolerance = value; break;
                    case 'G': tune.tracker.vision_reject_dist = value; break;
                    case 'C': tune.tracker.ang_tolerance = value; break;
                    case 'V': tune.estimate.mahony_kp = value; break;
                    case '1': tune.latency.encoder_latency_gain = value; break;
                    case '2': tune.latency.vision_latency_ms = value; break;
                    case '3': Algorithm::Tracker::set_box_push_final_press_cm(value); break;

                    case 'P': App::g_state.debug.telemetry_mode = (int)value; break; 
                    default: return;
                }
                break;
            
            case 'V':  // 视觉控制类指令
                switch (sub) {
                    case 'R': {
                        if (value == 1) Subsystem::Vision::request_map_ART1();
                        else if (value == 2) Subsystem::Vision::request_pose_ART1();
                        break;
                    }
                    case 'E': {
                        // !V E 1 开启实时延时估计，!V E 0 关闭(回退固定 vision_latency_ms)
                        tune.latency.enable_estimation = (value > 0.5f);
                        wireless_uart_send_buffer(
                            tune.latency.enable_estimation ?
                                (uint8_t*)"[SYS] Latency Est: ON\r\n" :
                                (uint8_t*)"[SYS] Latency Est: OFF\r\n",
                            tune.latency.enable_estimation ? 23 : 24);
                        break;
                    }
                    case 'G': {
                        // !V G 1 开启监测，!V G 0 关闭监测
                        if (value > 0.5f) {
                            is_monitoring_semantics = true;
                            // 将语义缓存全刷为 -2 (无效标签)，强制下一次轮询立即发送一次全量数据
                            memset(cached_semantic_labels, -2, sizeof(cached_semantic_labels));
                            wireless_uart_send_buffer((uint8_t*)"[SYS] Semantic Monitor: ON\r\n", 28);
                        } else {
                            is_monitoring_semantics = false;
                            wireless_uart_send_buffer((uint8_t*)"[SYS] Semantic Monitor: OFF\r\n", 29);
                        }
                        break;
                    }
                    default: return;
                }
                break;

            case 'L':  // 拐点延时估计调参指令 (latency estimator)
                switch (sub) {
                    case 'T': tune.latency.turn_thresh_deg = value; break;  // 拐点触发转角阈值 deg
                    case 'E': tune.latency.enc_v_min = value; break;        // 编码器速度门 cm/20ms
                    case 'V': tune.latency.vis_v_min = value; break;        // 视觉速度门 cm/frame
                    case 'R': tune.latency.refractory_ms = value; break;    // 去抖间隔 ms
                    case 'N': tune.latency.l_min_ms = value; break;         // L 接受下限 ms
                    case 'X': tune.latency.l_max_ms = value; break;         // L 接受上限 ms
                    case 'D': tune.latency.dtheta_tol_deg = value; break;   // 转角量级容差 deg
                    case 'A': tune.latency.lowpass_alpha = value; break;    // L 低通系数
                    case 'S': tune.latency.l_stale_ms = value; break;       // L 过期时间 ms
                    default: return;
                }
                break;

            case 'T':  // 运动控制调参（Stanley横纠 / 拐点略停 / 炸弹等待）
                switch (sub) {
                    case 'E': tune.stanley.enable = (value > 0.5f); break;  // Stanley 开关
                    case 'K': tune.stanley.k_ct = value; break;            // 横向误差增益
                    case 'F': tune.stanley.k_soft = value; break;          // 软化速度 cm/s
                    case 'L': tune.stanley.v_lat_max = value; break;       // 横纠速度上限 cm/s
                    case 'P': tune.tracker.corner_pause_speed = value; break; // 拐点略停阈值 cm/s
                    case 'B': tune.bomb.explosion_wait_ms = value; break;  // 炸弹引信等待 ms
                    case 'G': tune.feel.brake_hold_gain = value; break;    // 主动刹车前馈增益（刹更狠锁更死）
                    case 'A': tune.feel.corner_turn_acc = value; break;    // 切向方向变化加速度限 cm/s^2（大=切更直不磨圆）
                    // !T S 已移除：stop_approach_band_cm 字段保留占位但无任何行为（2026-07-15 删 StopBand）
                    case 'H': tune.stop_approach_brake_gain = value; break; // 停车接近区刹车倍率（大=到点抖减速更狠进点更慢过冲更小，1=旧行为）
                    case 'D': tune.short_seg_len_cm = value; break;        // 短段判定阈值 cm（段全长<=此值走高加速起步；设1≈关闭）
                    case 'V': tune.short_seg_accel = value; break;         // 短段起步加速度 cm/s^2（只压加速斜坡，不动刹车）
                    case 'Z': tune.approach_zone_cm = value; break;        // 停车接近区提前刹车距离上限 cm（默认40；0.5≈关闭回纯sqrt）
                    case 'R': tune.approach_zone_ratio = value; break;     // 接近区占段全长比例（默认0.25：20cm段→5cm）
                    case 'C': tune.approach_brake_acc = value; break;      // 接近区缓减速度 cm/s^2（默认15，越小末端越慢越准）
                    case 'N': tune.approach_enable = (value > 0.5f) ? 1.0f : 0.0f; break; // 接近区总开关（!T N 1 开 / !T N 0 关，一键切）
                    default: return;
                }
                break;

            case 'W': {  // 每轮轮速环调参：!W <轮 0-3> <字段> <值>，字段 kp/ki/kd/kv/ka/kb/ks
                int wi = -1; char field[8] = {0}; float v = 0.0f;
                if (sscanf(&cmd[2], "%d %7s %f", &wi, field, &v) == 3 && wi >= 0 && wi < 4) {
                    WheelControlParams& w = tune.wheels[wi];
                    if      (strcmp(field, "kp") == 0) w.pid.kp = v;
                    else if (strcmp(field, "ki") == 0) w.pid.ki = v;
                    else if (strcmp(field, "kd") == 0) w.pid.kd = v;
                    else if (strcmp(field, "kv") == 0) w.kv = v;
                    else if (strcmp(field, "ka") == 0) w.ka = v;
                    else if (strcmp(field, "kb") == 0) w.kb = v;
                    else if (strcmp(field, "ks") == 0) w.ks = v;
                }
                break;
            }

            case 'P': { // 多点路径下发指令
                auto& plan = App::g_state.planning;
                if (sub == 'C') {
                    // !P C -> 清空航点列表
                    plan.physical_path.clear();
                    plan.force_stop_at_wp.clear();
                    plan.current_wp_idx = 0;
                    wireless_uart_send_buffer((uint8_t*)"[PATH] Cleared.\r\n", 17);
                } 
                else if (sub == 'A') { 
                    // !P A 10.5 20.0 -> 添加航点
                    float px, py;
                    if (sscanf(&cmd[3], "%f %f", &px, &py) == 2) {
                        if (plan.physical_path.size() < SystemConfig::MAX_PATH_LENGTH) {
                            plan.physical_path.push_back({px, py});
                            plan.force_stop_at_wp.push_back(0U);
                        }
                    }
                }
                else if (sub == 'E') { 
                    // !P E -> 切换模式，强制执行路径
                    App::g_state.control.mode = ControlMode::AUTO_TRACKING;
                    App::g_state.control.tracker_state = TrackerState::TRACKING;
                    start_movement_timing(ControlMode::AUTO_TRACKING);
                }
                break;
            }

            case 'M': {  // 全局移动与位移指令
                float value = atof(&cmd[3]);
                // 强制切入调试模式，不再理会 Tracker
                App::g_state.control.mode = ControlMode::POINT_TRACKING;
                Pose2D target = App::g_state.control.current_target; 

                switch (sub) {
                    case 'P': {
                        // !M P 10.0 20.0 90.0 -> 直接设置目标位姿
                        float tx, ty, tyaw;
                        if (sscanf(&cmd[3], "%f %f %f", &tx, &ty, &tyaw) == 3) {
                            target.x = tx;
                            target.y = ty;
                            target.yaw = tyaw;
                        }
                        break;
                    }

                    case 'W': target.y += value; break;                  // 前进 (绝对坐标系 +Y 方向)
                    case 'S': target.y -= value; break;                  // 后退 (绝对坐标系 -Y 方向)
                    case 'A': target.x -= value; break;                  // 向左 (绝对坐标系 -X 方向)
                    case 'D': target.x += value; break;                  // 向右 (绝对坐标系 +X 方向)
                    case 'T': target.yaw = cur_yaw_deg + value; break;   // 转向 (逆时针为正)
                    case 'M': target = { cur_pos.x, cur_pos.y, cur_yaw_deg }; break;
                    default: return;
                }

                // 通过 Tracker 进入点跟踪，顺带清掉旧路径和视觉校正状态
                Algorithm::Tracker::track_point(target);
                start_movement_timing(ControlMode::POINT_TRACKING);
                break;
            }
        }
    }
    } // namespace (anonymous)

} // namespace Subsystem::Telemetry
