# 第21届全国大学生智能汽车竞赛 - 智能视觉组 (推箱子) 

本仓库包含了智能视觉组推箱子赛题的底层主控源码。项目采用 C++17(gnu17) 编写，深度优化了 RT1064 的存储性能，并实现了基于 IDA* 的寻路算法与高精度底盘控制系统。

## 🏎️ 硬件环境
- **主控**: NXP i.MX RT1064 (Cortex-M7, 600MHz)
- **FlexRAM 配置**: 64KB ITCM / 448KB DTCM / 488KB OCRAM
- **底盘**: 四轮麦克纳姆轮 (带增量式编码器)
- **传感器**: IMU660RA 陀螺仪
- **视觉**: 双 OpenART 方案 (UART DMA 通信)
- **显示**: 1.8寸 TFT (用于实时监控与调参)

## 🏗️ 软件架构规范 (协作必读)
本项目严格遵守**四层解耦架构**，任何新增代码必须归属于以下目录之一：

| 目录 | 功能描述 | 协作要求 |
| :--- | :--- | :--- |
| `App/` | 主循环、业务状态机、底盘控制、通信 | 严禁包含底层寄存器操作 |
| `Algorithm/` | **纯逻辑层**: IDA* 寻路、运动学解算、轨迹规划、PID等 | 必须跨平台兼容，不依赖特定硬件宏 |
| `Device/` | 外设 C++ 封装 (Motor, Encoder, Imu, Flash, Uart 等) | 隐藏硬件库细节，提供标准接口 |
| `Core/` | 中断分发 (isr.cpp)、系统配置、全局调参黑板 | 仅允许定义全局硬件句柄 |

### 🛠️ 开发黄金准则
1. **内存零分配**: 禁止使用 `new`/`malloc` 及 `std` 动态容器。所有数组需使用静态定长数组。
2. **性能加速**: 核心运算函数（如 PID、轨迹规划）必须使用 `AT_ITCM_SECTION_INIT` 放入 ITCM。
3. **参数黑板**: 调参变量统一放在 `g_tune` (TuningConfig) 结构体中，支持 Flash 掉电保存。
4. **C/C++ 混编**: 中断服务函数必须带 `extern "C"`。

## 🚀 核心算法模块
- **Sokoban Solver**: 基于 **IDA* (Iterative Deepening A*)** 算法。
  - 支持宏动作 (Macro-moves) 与连推判定。
  - 使用 Zobrist 哈希与置换表 (TT) 实现极速剪枝。
- **Trajectory Planner**: 标量速度**梯形加减速规划**。
  - 自动分解 2D 直线速度，解决麦轮启动打滑与停车不准问题。
- **Chassis Control**: 级联闭环系统。
  - 外环：位置/轨迹跟踪。
  - 内环：基于增量式 PID 的四轮转速控制。
 
## 📁 目录结构
```text
.
├── libraries/              # 逐飞科技底层库与 NXP SDK
├── project/
    ├── Algorithm/          # 算法库：寻路、PID、轨迹规划
    ├── App/                # 业务逻辑：状态机、TFT菜单、视觉处理
    ├── Device/             # 硬件驱动封装
    ├── Core/               # 中断、系统配置、存储管理
    ├── mdk/                # Keil 工程文件 (uvprojx)
    └── 
```

## 💡 提示：
在 GitHub 协作时，建议你在 `.gitignore` 文件中忽略以下 Keil 产生的临时文件，避免提交冲突：
```gitignore
*.bak
*.dep
*.py_bak
*.uvgui.*
*.uvguix.*
JLinkLog.txt
*.lst
*.obj
*.o
*.d
*.crf
*.lnp
*.axf
*.htm
*.sct
*.map
```
