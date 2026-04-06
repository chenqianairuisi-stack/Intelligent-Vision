#include "telemetry.h"
#include "task_control.h" 
#include "task_vision.h"
#include "odometry.h"
#include "encoder.h"
#include "imu_process.h"
#include <cstdlib>
#include <cstring>
#include <cmath>

Telemetry telemetry;
float planned_v_debug = 0.0f;     // 当前规划的速度大小，供 telemetry 模块发送波形数据
float current_local_x = 0.0f;     // 当前局部 X 位移，供调试用
float current_local_y = 0.0f;     // 当前局部 Y 位移，供调试用     

void Telemetry::init() {
    // 初始化无线串口 (波特率默认115200)
    wireless_uart_init();

    // 初始化发送数据包的帧头和帧尾
    tx_packet.head = 0xAA;
    tx_packet.tail = 0xBB;
}

// 发送波形数据
void Telemetry::send_wave_data() {
    if (telemetry_display_mode == 0) {
        // 【模式 0】：底盘动力学监控
        Point2D current_pos = chassis_odometry.get_position();
        Pose2D  target_pos  = chassis_task.get_target_pose();  
        
        Velocity2D avg_speed = Kinematics::forward_kinematics(
            encoders.get_speed_cm_s(0), encoders.get_speed_cm_s(1),
            encoders.get_speed_cm_s(2), encoders.get_speed_cm_s(3)
        );
        float avg_speed_mag = std::sqrt(avg_speed.vx * avg_speed.vx + avg_speed.vy * avg_speed.vy);

        tx_packet.data_1 = planned_v_debug;            
        tx_packet.data_2 = avg_speed_mag;              
        tx_packet.data_3 = target_pos.x;               
        tx_packet.data_4 = current_pos.x;              
        tx_packet.data_5 = target_pos.y;               
        tx_packet.data_6 = current_pos.y;              
        
    } 
    else if (telemetry_display_mode == 1) {
        // 【模式 1】：ART1 视觉定位与地图监控
        tx_packet.data_1 = vision_data.current_x;      // CH1: 视觉 X 坐标
        tx_packet.data_2 = vision_data.current_y;      // CH2: 视觉 Y 坐标
        tx_packet.data_3 = vision_data.box_count;      // CH4: 解析出的箱子总数
        tx_packet.data_4 = vision_data.bomb_count;     // CH5: 解析出的炸弹总数
        // CH5: 地图就绪脉冲信号 (视觉模块每次解析出新地图时会置位一次，发送后立即清零)
        tx_packet.data_5 = vision_data.art1_map_ready ? 10.0f : 0.0f; 
        vision_data.art1_map_ready = false;
        // CH6: 定位更新脉冲信号
        tx_packet.data_6 = vision_data.art1_pose_updated ? 10.0f : 0.0f; 
        vision_data.art1_pose_updated = false;
        
    } 
    else if (telemetry_display_mode == 2) {
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
void Telemetry::receive_and_parse_task() {
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



// 解析并执行上位机发来的字符串命令 (格式示例: "!S Q 1.5" 表示设置速度环 KP=1.5)
void Telemetry::execute_command(const char* cmd) {

    if (cmd[0] != '!') return;     // 命令必须以 ! 开头
    char type = cmd[1];            // 主类型: 'S' 设置参数, 'M' 移动指令
    char sub  = cmd[2];            // 子类型
    float value = atof(&cmd[3]);   // 把后面的字符串转为浮点数 (跳过空格自动处理)

    Point2D cur_pos = chassis_odometry.get_position();
    float cur_yaw_deg = imu_sensor.get_yaw();

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

                case 'V': telemetry_display_mode = (int)value; break; 
                default: return;
            }
            break;
        
        case 'C':  // 视觉控制类指令
            switch (sub) {
                case 'M': vision_manager.request_map_ART1();  break;
                case 'P': vision_manager.request_pose_ART1(); break;
                case 'T': vision_manager.request_capture_ART2((uint8_t)value, true); break;
                case 'L': vision_manager.test_loopback_map(); break;
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
            chassis_task.set_target_pose(target);
            break;
        }

        case 'L': {  // 局部移动类指令
            Pose2D target = { current_local_x, current_local_y, 90.0f }; 

            switch (sub) {
                case 'W': target.y += value; break;                  // 前进 (绝对坐标系 +Y 方向)
                case 'S': target.y -= value; break;                  // 后退 (绝对坐标系 -Y 方向)
                case 'A': target.x -= value; break;                  // 向左 (绝对坐标系 -X 方向)
                case 'D': target.x += value; break;                  // 向右 (绝对坐标系 +X 方向)
                case 'M': target = { current_local_x, current_local_y, 90.0f }; break;
                default: return;
            }
            chassis_task.set_target_pose(target);
            break;
        }
    }
}

