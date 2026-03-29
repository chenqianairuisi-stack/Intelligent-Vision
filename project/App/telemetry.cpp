#include "zf_common_headfile.h"
#include "telemetry.h"
#include "task_control.h" 
#include "task_vision.h"
#include "odometry.h"
#include "encoder.h"
#include "imu.h"
#include <cstdlib>
#include <cstring>
#include <cmath>

Telemetry telemetry;
float planned_v_debug = 0.0f;     // 当前规划的速度大小，供 telemetry 模块发送波形数据

char last_rx_cmd[32] = "WAITING CMD...";

void Telemetry::init() {
    // 初始化无线串口 (波特率默认115200)
    wireless_uart_init();

    // 初始化发送数据包的帧头和帧尾
    tx_packet.head = 0xAA;
    tx_packet.tail = 0xBB;
}

// 发送波形数据 (主循环)
void Telemetry::send_wave_data() {
    Point2D current_pos = chassis_odometry.get_position();
    Pose2D  target_pos  = chassis_task.get_target_pose();  
    
    Velocity2D avg_speed = Kinematics::forward_kinematics(
        encoders.get_speed_cm_s(0),
        encoders.get_speed_cm_s(1),
        encoders.get_speed_cm_s(2),
        encoders.get_speed_cm_s(3)
    );
    float avg_speed_mag = std::sqrt(avg_speed.vx * avg_speed.vx + avg_speed.vy * avg_speed.vy);

    // 填充数据通道
    tx_packet.target_v = planned_v_debug;            // 梯形规划的当前标量速度
    tx_packet.actual_v = avg_speed_mag;              // 轮子实际反馈速度
    tx_packet.target_x = target_pos.x;               // 目标点 X
    tx_packet.actual_x = current_pos.x;              // 实际里程计 X
    tx_packet.target_y = target_pos.y;               // 目标点 Y
    tx_packet.actual_y = current_pos.y;              // 实际里程计 Y

    wireless_uart_send_buffer((uint8*)&tx_packet, sizeof(VofaJustFloat));
}

// 接收并解析上位机命令 (主循环)
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

                    strncpy(last_rx_cmd, rx_cmd_buf, sizeof(last_rx_cmd) - 1);  // 更新全局变量供 TFT 显示
                    last_rx_cmd[sizeof(last_rx_cmd) - 1] = '\0';
                    
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
                default: return;
            }
            break;
        
        case 'C':  // 视觉控制类指令
            switch (sub) {
                case 'M': vision_manager.request_map_ART1();  break;
                case 'P': vision_manager.request_pose_ART1(); break;
                case 'T': vision_manager.trigger_ART2(true);  break;
            }
            break;

        case 'M': {  // 移动类指令
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
    }
}