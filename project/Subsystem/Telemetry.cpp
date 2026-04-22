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

#include "zf_common_headfile.h"
#include "Encoder.h"
#include "UartComm.h"


namespace Subsystem::Telemetry {

// ====================================================================
// 内部隐蔽变量：VOFA 协议体和接收缓存
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

    bool is_timing_movement = false;   // 测时器状态标志
    uint32_t movement_start_time = 0;  // 运动开始的系统时间戳 (ms)
    ControlMode current_timing_mode = ControlMode::MANUAL_DEBUG;  // 当前测时器关联的控制模式

    void start_movement_timing(ControlMode mode);  // 触发测时器
    void check_movement_completion();              // 在后台循环中检查测时器状态，计算并发送结果
    void dump_semantic_cache();                    // 发送语义缓存内容
    void execute_command(const char* cmd);         // 命令解析函数声明
}

// ====================================================================
// 公共接口
// ====================================================================

void init() {
    wireless_uart_init();  // 初始化无线串口 (波特率默认115200)
}


// 发送波形数据
void send_wave_data() {

    if (App::g_state.debug.telemetry_mode == -1) return; 

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
        auto& vision_data = App::g_state.vision;
        // 【模式 1】：ART1 视觉定位与地图监控
        tx_packet.data_1 = vision_data.art1_pose.x;    // CH1: 视觉 X 坐标
        tx_packet.data_2 = vision_data.art1_pose.y;    // CH2: 视觉 Y 坐标
        tx_packet.data_3 = vision_data.box_count;      // CH4: 解析出的箱子总数
        tx_packet.data_4 = vision_data.bomb_count;     // CH5: 解析出的炸弹总数
        tx_packet.data_5 = 0.0f; 
        tx_packet.data_6 = 0.0f;         
    } 

    wireless_uart_send_buffer((uint8*)&tx_packet, sizeof(VofaJustFloat));
}

// 接收上位机命令
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

    // 触发测时器
    void start_movement_timing(ControlMode mode) {
        is_timing_movement = true;
        current_timing_mode = mode;
        movement_start_time = Core::Scheduler::get_sys_tick_ms(); 
        
        const char* msg = "[SYS] Movement Started.\r\n";
        wireless_uart_send_buffer((uint8_t*)msg, strlen(msg));
    }

    // 在后台循环中检查运动是否结束
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

    // 语义缓存池内容发送给上位机
    void dump_semantic_cache() {
        static char dump_buf[256];
        const auto& labels = App::g_state.vision.semantic_labels;
        
        // 组装前缀
        int offset = snprintf(dump_buf, sizeof(dump_buf), "[SEMANTIC_DUMP] ");
        
        // 遍历整个语义池（假设 MAX_ENTITIES 不会过大，否则需要分包）
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

    // 解析并执行上位机发来的字符串命令 (格式示例: "!S Q 1.5" 表示设置速度环 KP=1.5)
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
                    
                    case 'A': tune.dynamics.max_duty = value; break;
                    case 'S': tune.dynamics.max_speed = value; break;
                    case 'D': tune.dynamics.max_acc = value; break;
                    case 'F': tune.dynamics.max_jerk = value; break;
                    case 'G': tune.dynamics.max_ang_speed = value; break;

                    case 'Z': tune.dynamics.kinematic_gain_x = value; break;
                    case 'X': tune.dynamics.kinematic_gain_y = value; break;
                    case 'C': tune.tracker.reach_radius = value; break;
                    case 'V': tune.tracker.reach_radius_min = value; break;

                    case 'P': App::g_state.debug.telemetry_mode = (int)value; break; 
                    default: return;
                }
                break;
            
            case 'C':  // 视觉控制类指令
                switch (sub) {
                    case 'R': {
                        if (value == 1) Subsystem::Vision::request_map_ART1();
                        else if (value == 2) Subsystem::Vision::request_pose_ART1();
                        break;
                    }
                    case 'G': dump_semantic_cache(); break;
                    default: return;
                }
                break;

            case 'P': { // 多点路径下发指令
                auto& plan = App::g_state.planning;
                if (sub == 'C') { 
                    // !P C -> 清空航点列表
                    plan.physical_path.clear();
                    plan.current_wp_idx = 0;
                    wireless_uart_send_buffer((uint8_t*)"[PATH] Cleared.\r\n", 17);
                } 
                else if (sub == 'A') { 
                    // !P A 10.5 20.0 -> 添加航点
                    float px, py;
                    if (sscanf(&cmd[3], "%f %f", &px, &py) == 2) {
                        if (plan.physical_path.size() < SystemConfig::MAX_PATH_LENGTH) {
                            plan.physical_path.push_back({px, py});
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
                App::g_state.control.mode = ControlMode::MANUAL_DEBUG;
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

                // 下发并重置定时器
                App::g_state.control.current_target = target;
                start_movement_timing(ControlMode::MANUAL_DEBUG);
                break;
            }
        }
    }
    } // namespace (anonymous)

} // namespace Subsystem::Telemetry