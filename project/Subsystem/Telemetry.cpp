#include "Telemetry.h"
#include "RobotState.h"
#include "tuning_config.h"
#include "system_config.h"
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "Vision.h"
#include "TestMap.h"
#include "MotionControl.h"

#include "zf_common_headfile.h"
#include "Encoder.h"
#include "UartComm.h"


namespace Subsystem::Telemetry {

// ====================================================================
// 内部隐蔽变量：VOFA 协议体和接收缓存 (绝对不污染外部环境)
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

    void execute_command(const char* cmd);
}

// ====================================================================
// 公共接口
// ====================================================================

void init() {
    wireless_uart_init();  // 初始化无线串口 (波特率默认115200)
}

// 发送波形数据
void send_wave_data() {
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
        // CH5: 地图就绪脉冲信号 (视觉模块每次解析出新地图时会置位一次，发送后立即清零)
        tx_packet.data_5 = vision_data.art1_map_ready ? 10.0f : 0.0f; 
        vision_data.art1_map_ready = false;
        // CH6: 定位更新脉冲信号
        tx_packet.data_6 = vision_data.art1_pose_updated ? 10.0f : 0.0f; 
        vision_data.art1_pose_updated = false;
        
    } 
    else if (App::g_state.debug.telemetry_mode == 2) {
        auto& vision_data = App::g_state.vision;
        // 【模式 2】：ART2 语义识别监控
        tx_packet.data_1 = (float)vision_data.semantic_labels[0]; // CH1: 实体 0 的标签
        tx_packet.data_2 = (float)vision_data.semantic_labels[1]; // CH2: 实体 1 的标签
        tx_packet.data_3 = (float)vision_data.semantic_labels[2]; // CH3: 实体 2 的标签
        tx_packet.data_4 = (float)vision_data.semantic_labels[3]; // CH4: 实体 3 的标签
        tx_packet.data_5 = (float)vision_data.semantic_labels[4]; // CH5: 实体 4 的标签
        tx_packet.data_6 = (float)vision_data.semantic_labels[5]; // CH6: 实体 5 的标签
        vision_data.capture_ack_received = false;
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
}


namespace {
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
                    case 'Q': tune.pid_speed.kp = value; break;
                    case 'W': tune.pid_speed.ki = value; break;
                    case 'E': tune.pid_yaw.kp = value; break;
                    case 'R': tune.pid_yaw.kd = value; break;

                    case 'A': tune.dynamics.max_speed = value; break;
                    case 'S': tune.dynamics.max_acc = value; break;
                    case 'D': tune.dynamics.max_jerk = value; break;
                    case 'F': tune.dynamics.max_ang_speed = value; break;

                    case 'Z': tune.tracker.reach_radius = value; break;
                    case 'X': tune.tracker.reach_radius_min = value; break;

                    case 'V': App::g_state.debug.telemetry_mode = (int)value; break; 
                    default: return;
                }
                break;
            
            case 'C':  // 视觉控制类指令
                switch (sub) {
                    case 'M': Subsystem::Vision::request_map_ART1();  break;
                    case 'P': Subsystem::Vision::request_pose_ART1(); break;
                    case 'T': Subsystem::Vision::request_capture_ART2((uint8_t)value, true); break;
                }
                break;

            case 'M': {  // 全局移动类指令
                Pose2D target = { cur_pos.x, cur_pos.y, cur_yaw_deg };

                switch (sub) {
                    case 'W': target.y += value; break;                  // 前进 (绝对坐标系 +Y 方向)
                    case 'S': target.y -= value; break;                  // 后退 (绝对坐标系 -Y 方向)
                    case 'A': target.x -= value; break;                  // 向左 (绝对坐标系 -X 方向)
                    case 'D': target.x += value; break;                  // 向右 (绝对坐标系 +X 方向)
                    case 'T': target.yaw = cur_yaw_deg + value; break;   // 转向 (逆时针为正)
                    case 'M': target = { cur_pos.x, cur_pos.y, cur_yaw_deg }; break;
                    default: return;
                }
                App::g_state.control.current_target = target;
                break;
            }
        }
    }
    } // namespace (anonymous)
} // namespace Subsystem::Telemetry