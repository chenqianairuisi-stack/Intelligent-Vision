# 第21届全国大学生智能汽车竞赛 - 智能视觉组推箱子主控工程

本仓库是 RT1064 主控侧完整工程，面向智能视觉组推箱子赛题，覆盖视觉通信、巡图规划、推箱求解、底盘控制、姿态与里程计、遥测与调参显示等核心链路。

## 1. 硬件与平台

- 主控：NXP i.MX RT1064（Cortex-M7, 600MHz）
- 存储布局：ITCM / DTCM / OCRAM 分区优化
- 底盘：四轮麦克纳姆轮（编码器闭环）
- 视觉：双 OpenART（UART）
- 惯导：IMU 模块（工程内含 ICM42688 与通用 IMU 封装）
- 显示：TFT 菜单与状态可视化

## 2. 工程结构

```text
.
├── README.md
├── README_PLANNING.md
├── libraries/                        # SDK、驱动、组件库（第三方与平台层）
│   ├── sdk/
│   ├── zf_common/
│   ├── zf_driver/
│   ├── zf_device/
│   ├── zf_components/
│   ├── components/
│   └── doc/
├── project/                          # 业务主工程
    ├── App/                          # 主循环、全局状态、业务状态机
    │   ├── main.cpp
    │   ├── GameManage.cpp/.h
    │   ├── RobotState.h
    │   └── RobotTask.h
    ├── Algorithm/                    # 算法层（规划 + 控制）
    │   ├── PlanningCommon.cpp/.h
    │   ├── Strategy.cpp/.h
    │   ├── MotionControl.cpp/.h
    │   ├── Tracker.cpp/.h
    │   ├── Exploration.cpp/.h
    │   ├── MacroPlanner.cpp/.h
    │   └── Sokoban.cpp/.h
    ├── Subsystem/                    # 业务子系统编排层
    │   ├── Vision.cpp/.h
    │   ├── ChassisControl.cpp/.h
    │   ├── PoseEstimate.cpp/.h
    │   ├── Telemetry.cpp/.h
    │   └── Display.cpp/.h
    ├── Device/                       # 设备抽象封装层
    │   ├── Motor.cpp/.h
    │   ├── Encoder.cpp/.h
    │   ├── Icm42688.cpp/.h
    │   ├── UartComm.cpp/.h
    │   └── Storage.cpp/.h
    ├── Core/                         # 调度、中断、系统与调参配置
    │   ├── CoreScheduler.cpp/.h
    │   ├── isr.cpp
    │   ├── system_config.h
    │   └── tuning_config.h
    └── mdk/                          # Keil 工程与构建输出
        ├── rt1064.uvprojx
        ├── rt1064.uvoptx
        ├── Objects/
        ├── Listings/
        ├── scf/
        └── MDK删除临时文件.bat
└── simulation/                        # 桌面仿真入口与地图/回归工具
    ├── main.cpp                       # 按 MCU GameManage 微任务流程运行
    ├── map/                            # 地图与地图生成器
    ├── test/                           # 批量回归脚本
    └── visualizer/                     # PC 可视化工具
```

## 3. 分层职责

| 目录 | 职责 | 说明 |
| :--- | :--- | :--- |
| project/App | 主循环与比赛状态机 | 负责流程编排，不直接操作底层寄存器 |
| project/Algorithm | 纯算法逻辑 | 包含轨迹、控制、巡图、推箱求解 |
| project/Subsystem | 子系统协调层 | 连接算法层和设备层，承接业务接口 |
| project/Device | 设备抽象层 | 封装电机、编码器、IMU、串口、存储 |
| project/Core | 系统核心层 | 中断入口、调度器、系统参数与调参项 |
| simulation | 桌面仿真层 | 直接链接 project/Algorithm，模拟 GameManage、RobotTask 和 ART2 回传 |
| libraries | 平台/第三方库 | SDK、驱动、组件与文档 |

## 4. 运行流程概览

启动关键顺序（见 main.cpp）：

1. 时钟与调试初始化
2. 调度器、遥测、姿态估计、视觉、底盘初始化
3. 参数存储与 TFT 菜单初始化
4. 游戏管理器初始化，按开关选择实车、Mock 或 Demo 流程
5. IMU 开机静态标定
6. 启动 5ms/20ms 周期中断
7. 主循环执行：视觉更新 + 任务调度 + 业务状态机

调度器当前任务（见 CoreScheduler.cpp）：

| 任务 | 周期 |
| :--- | :--- |
| 上位机指令解析 | 10ms |
| 波形数据发送 | 20ms |
| TFT UI 刷新 | 100ms |

### 4.3 状态机流程图（GamePhase）

```mermaid
flowchart TD
   A[INIT_CALIBRATE] --> B[EXIT_START_ZONE]
   B -->|到达出发目标点| C[WAIT_FOR_VISION]

   C -->|地图就绪 且 阶段2/3| D[PLAN_PATROL]
   C -->|地图就绪 且 阶段1| K[PLAN_SOKOBAN]

   D --> E[EXEC_ACTION_DISPATCH]
   E -->|语义与参考序列满足条件| J[BIND_SEMANTICS]
   E -->|Macro 观测动作| F[EXEC_PATROL_MOVE]
   E -->|Macro 推炸弹动作| H[EXEC_BOMB_PUSH]
   E -->|Macro 完成式推箱动作| P[EXEC_BOX_PUSH]
   E -->|路径失败| Z[ERROR_OCCURRED]

   F -->|到达观测点| G[EXEC_ALIGN_YAW]
   G -->|对准后触发ART2抓拍| G2[WAIT_ART2_CAPTURE]
   G2 -->|语义结果与ACK均到齐| E

   H -->|动作完成| I[UPDATE_MAP]
   I --> E
   P -->|动作完成| E

   J -->|语义绑定完成| K

   K -->|求解成功| L[EXEC_SOKOBAN]
   K -->|求解失败 重新感知| C

   L -->|路径执行完成| M[FINISHED]

   subgraph Mock / Demo 拦截分支
      G2 -->|Mock注入语义并直接结算抓拍| E
      D --> D1[ANIMATE_PATROL_DEMO]
      D1 -->|Demo动画完成并同步语义| J
      K --> K1[ANIMATE_DEMO]
      K1 -->|推箱动画结束| M
   end
```

说明：

- 主流程状态机位于 `GameManager::update()`。
- Mock 模式只拦截地图与 ART2 抓拍输入，仍使用真实底盘执行路径。
- Demo 模式在 Mock 语义基础上拦截底盘动作，用内部动画推演巡图与推箱。
- `ERROR_OCCURRED` 与 `FINISHED` 均会将底盘目标锁定为当前位置，进入停车保持。

## 5. 视觉通信协议摘要

主控与视觉模块采用统一帧格式：

```text
[0xAA][0x55][MsgType][Len][Payload...][Checksum]
Checksum = MsgType + Len + Sum(Payload)
```

关键消息方向：

- 主控 -> ART1：请求地图、请求定位
- 主控 -> ART2：触发箱子单实体观测或目标点三实体批量观测
- ART1 -> 主控：地图包、定位包
- ART2 -> 主控：抓拍 ACK、语义识别结果

ART2 语义缓存规则：

- `semantic_labels[i] = -1` 表示未知；`0~9` 表示识别到的图案类别。
- 实体索引按 ART1 地图顺序排列：`0..box_count-1` 为箱子，`box_count..box_count+target_count-1` 为目标点。
- `relative_x`、`relative_y` 为相机坐标格差：X 指向车身右侧，Y 指向车头前方，按本次观测 yaw 从地图坐标旋转得到。
- 箱子请求发送 `[id, relative_x, relative_y]` 三字节负载。
- 目标点请求发送三组 `[id, relative_x, relative_y]`，共九字节；按相机 X 从小到大，即从左到右排列，空槽全部填 `-1`。
- ART2 仍逐实体回传语义结果，实车任务在 ACK 与本次请求 mask 内全部实体的语义结果都到齐后才结算。
- Mock 模式在自身流程中注入同格式语义，不修改基类真实视觉请求逻辑。

语义匹配采用 N-1 规则：先用已观测的同语义箱子和目标点直接配对；若只剩一个未识别目标点，则把已识别但未找到同标签目标的箱子补到它；若只剩一个未识别箱子，则把已识别但未找到同标签箱子的目标反向补到它；最后只允许唯一剩余箱子和唯一剩余目标互补。

## 6. 算法与性能说明

- 推箱求解：IDA* + 剪枝策略（含哈希与置换表）
- 巡图规划：Exploration 生成参考序列，MacroPlanner 在线调度观测、推炸弹与完成式推箱
- 语义绑定：MacroPlanner 同步视觉语义池，满足 N-1 规则后导出 box -> target 配对供 Sokoban 使用
- 控制模块：轨迹规划 + 运动学解算 + PID 闭环
- 规划模块与内存说明参考：`README_PLANNING.md`

## 7. 开发约束建议

- 以静态内存为主，避免运行期动态分配
- 高频逻辑优先放入合适段（如 ramfunc）
- C/C++ 混编接口显式处理链接边界
- 中断逻辑尽量短小，将复杂逻辑下放到任务或模块函数

## 8. 参考文档

- 规划模块说明：README_PLANNING.md
- 第三方许可与版本：libraries/doc 与 libraries/LICENSE
