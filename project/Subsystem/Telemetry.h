#pragma once

/// \file Telemetry.h
/// \brief 无线串口遥测、在线调参和调试控制协议
///
/// \details
/// 通信使用 UART8 无线串口，波特率 115200，命令以 CR 或 LF 结束
/// 单行最多 95 个字符，不含结尾的空字符
/// 控制命令必须以 `!` 开头，参数赋值不加 `!`
///
/// 控制命令：
/// ```text
/// !TEL Q                         查询当前波形模式
/// !TEL MODE <0..3>               开启指定波形模式
/// !TEL OFF                       关闭波形输出
/// !TUNE ON                       进入速度环调参并输出当前参数
/// !TUNE OFF                      退出速度环调参
/// !TUNE Q                        输出速度环和运动参数摘要
/// !TUNE LIST                     输出全部可调参数、当前值和范围
/// !SAVE                          保存当前参数到 Flash
/// !LOAD                          从 Flash 载入参数
/// !RESET                         恢复默认参数，不自动保存
/// !MOVE POS <x_cm> <y_cm> <yaw_deg>
/// !MOVE REL <dx_cm> <dy_cm> <dyaw_deg>
/// !MOVE HOME                     停在当前位置
/// !MOVE STOP                     停在当前位置
/// !PATH CLEAR                    清空调试路径
/// !PATH ADD <x_cm> <y_cm>         添加路径点
/// !PATH EXEC                     执行调试路径
/// !VISION MAP                    请求 ART1 地图
/// !VISION POSE                   请求 ART1 位姿
/// !VISION LAG ON                 开启视觉延迟估计
/// !VISION LAG OFF                关闭视觉延迟估计
/// !SEM ON                        开启语义变化上报
/// !SEM OFF                       关闭语义变化上报
/// !SEM Q                         立即输出语义标签
/// ```
///
/// 参数赋值支持 `MV 150`、`MV=150`、`MV:150` 三种格式，修改立即生效
/// 修改后需要执行 `!SAVE` 才会写入 Flash，越界值和未知键会返回错误
/// `WH 0/1/2/3/4` 选择 LF/LB/RF/RB/全部车轮，随后可用以下速度环键：
/// ```text
/// KP KI KD KV KA KB KS            修改当前选中车轮
/// 0P..0S  1P..1S  2P..2S  3P..3S 直接修改 LF/LB/RF/RB
/// P=Kp I=Ki D=Kd V=Kv A=Ka B=Kb S=Ks
/// ```
///
/// 通用参数键：
/// ```text
/// MD 最大占空比       MV 最大线速度       MA 最大线加速度
/// AV 最大角速度       AA 最大角加速度     KX/KY 运动学增益
/// BL 制动限幅         TR 到点半径         TM 最小到点半径
/// CV 转角通过速度     CW 转角切换窗口     CT 转角直线容差
/// VI 视觉请求间隔     VR 视觉拒绝距离     AT 角度到点容差
/// CP 转角暂停速度     MK Mahony Kp        EG 编码器延迟增益
/// VL 视觉延迟         LE 延迟估计开关     LT 转弯判定角
/// EV 编码器速度门     VV 视觉速度门       LR 延迟估计冷却时间
/// LN 最小延迟         LX 最大延迟         LD 角度匹配容差
/// LA 延迟低通系数     LS 延迟失效时间     BW 爆炸等待时间
/// YB 偏航线性区       YD 偏航微分         YG 平移偏航增益
/// YT 平移偏航微分     YS 偏航静摩擦补偿   TB 停车制动增益
/// SL 短段长度         SA 短段加速度       AZ 接近区长度
/// AR 接近速度比例     AC 接近区加速度     AE 接近区使能
/// ```
///
/// 波形每帧为 8 个逗号分隔的浮点数：
/// ```text
/// 模式 0  目标 LF/LB/RF/RB，实测 LF/LB/RF/RB 轮速
/// 模式 1  目标/融合/视觉/编码器 X，目标/融合/视觉/编码器 Y
/// 模式 2  加速度/陀螺仪/Mahony 俯仰角，自适应 Kp，加速度模，偏航角速度，陀螺仪 X，偏航角
/// 模式 3  原始/滤波/采用延迟，待匹配转角数，补偿位姿 X/Y 误差，视觉修正 X/Y
/// ```
///
/// `!TUNE ON/OFF`、`!MOVE`、`!PATH`、`!SAVE`、`!LOAD`、`!RESET`
/// 只允许在 Debug 或 Demo 模式使用，其余查询、遥测、视觉、语义和参数赋值不受此限制
/// 运动命令开始时返回 `[MOVE] START`，到达目标后只返回 `[MOVE] DONE`

namespace Subsystem::Telemetry {
    void init();

    void send_wave_data();           // 发送 8 通道 CSV 波形
    void receive_and_parse_task();   // 解析无线串口按行命令

    void log_vision_calibration(float v_x, float v_y, float v_yaw, float o_x, float o_y, float o_yaw, bool accepted);
}
