# Planning 核心模块说明

本文档面向 RT1064 单片机移植场景，说明 `planning` 目录中的核心算法文件、PC 端验证文件，以及当前 RAM/DTCM/栈内存占用估算。

## 1. 需要保留的核心文件

移植到 MCU 时，核心规划链路由以下文件组成：

- `system_config.h`
- `PlanningCommon.h` / `PlanningCommon.cpp`
- `Strategy.h` / `Strategy.cpp`
- `Exploration.h` / `Exploration.cpp`
- `Sokoban.h` / `Sokoban.cpp`
- `MacroPlanner.h` / `MacroPlanner.cpp`

`main.cpp` 只是 PC 端测试入口，负责读取 `map_input.txt`、运行完整流程并写出 `path_output.txt`。`solver.exe`、`map_input.txt`、`path_output.txt` 和可视化脚本都不需要放进 MCU 工程。

## 2. 模块职责

- `PlanningCommon`：地图/实体查询、普通路径搜索、麦轮时间代价、推箱/推炸弹路径展开。
- `Strategy`：炸弹任务筛选、收益评估、任务排序、局部清障回退。
- `Exploration`：巡图观测规划、观测点生成、观测顺序安排、巡图阶段炸弹插入。
- `Sokoban`：语义绑定后的最终推箱求解，核心为 IDA* 搜索。
- `MacroPlanner`：串联巡图、观测、炸弹和最终求解，做在线宏观调度。

## 3. 数据结构约束

核心代码使用 `StaticArray<T, N>` 和固定上限数组，避免 `new` / `malloc` / `std::vector` 进入 MCU 热路径。

当前配置：

- 地图：`MAP_MAX_WIDTH = 12`，`MAP_MAX_HEIGHT = 16`
- 箱子：`MAX_BOXES = 10`
- 炸弹：`MAX_BOMBS = 3`
- 路径：`MAX_PATH_LENGTH = 200`
- 置换表：`TT_SIZE = 32768`

调整这些上限时必须同步检查 RAM、DTCM 和栈占用。

## 4. 本轮嵌入式优化

- `TT_SIZE` 已从 `65536` 降到 `32768`，节省 128 KiB DTCM。
- 普通 BFS / 路径回溯工作区改为共享 scratch，不再在 `get_grid_path`、`bfs_shortest_path`、`calc_player_reach` 中申请大局部数组。
- BFS 热路径使用 occupancy map 做 O(1) 障碍查询，避免反复线性扫描 `has_box` / `has_bomb`。
- BFS visited 改为 generation counter，减少热点 `memset`。
- `run_grid_time_search` 堆容量从 `MAP_CELL_COUNT * 24` 降为 `MAP_CELL_COUNT * 8`，并在堆溢出时安全返回失败。
- `BombTask` 参数改为 `const BombTask&`，避免反复拷贝大结构体。
- `yaw_turn_time_cost` 去掉 `% 360`，改为浮点角差和 1 度容差。
- 巡图/策略的大 scratch 不再进入显式 DTCM 区，默认留在 OCRAM，避免挤占 DTCM 和栈。
- 巡图 DFS 深度硬限制为 `PATROL_DFS_FRAME_LIMIT = 16`，用于适配 32 KiB 栈。

## 5. RT1064 内存统计

目标约束：

- DTCM 物理容量：448 KiB，`0x70000`
- `RW_m_data` 区域最大值：408 KiB，`0x66000`
- `ARM_LIB_HEAP`：8 KiB，`0x2000`
- `ARM_LIB_STACK`：32 KiB，`0x8000`

统计方法：按当前源码中的固定数组、全局对象和 section 标注统计，并用 `g++ -m32` 生成目标文件复核，使指针尺寸更接近 RT1064。桌面 GCC 只能用于估算 section 大小，最终上板前仍需以 Keil/armclang map 文件为准。

### 5.1 显式 DTCM 占用

| 模块 | 主要对象 | 占用 |
| --- | --- | ---: |
| `PlanningCommon.cpp` | `b_ws` | 7.16 KiB |
| `Strategy.cpp` | `strategic_planner`、DFS 距离场、第一阶段候选距离场 | 41.84 KiB |
| `Exploration.cpp` | `patrol_planner` | 40.41 KiB |
| `Sokoban.cpp` | `solver` + `TT[32768]` | 183.72 KiB |
| `MacroPlanner.cpp` | `macro_planner` | 2.38 KiB |
| **规划核心合计** |  | **275.50 KiB / `0x44E00`** |

规划核心显式 DTCM 合计约 275.50 KiB。加上 `core_engine`、`g_state`、`tune`、底盘、姿态估计、视觉串口等当前全局对象后，整机显式 DTCM 估算约 285.44 KiB / `0x475C0`。再计入 8 KiB heap 和 32 KiB stack，DTCM 物理占用约 325.44 KiB / `0x515C0`，距离 `RW_m_data` 408 KiB 上限和 448 KiB 物理容量均剩余约 122.56 KiB / `0x1EA40`。

### 5.2 OCRAM / 普通静态 RAM

未标记 `DTCM_DATA`、默认留在 OCRAM 的静态区：

| 模块 | 主要对象 | 占用 |
| --- | --- | ---: |
| `PlanningCommon.cpp` | 时间搜索工作区、普通 BFS 工作区、`.dtcm_bss` 命名静态区 | 20.66 KiB / `0x52A4` |
| `Strategy.cpp` | DFS 候选队列 scratch、匹配 DP、局部清障/距离场临时区 | 71.10 KiB / `0x11C68` |
| `Exploration.cpp` | 巡图 DFS / cache / materialize scratch | 134.41 KiB / `0x219A0` |
| `Sokoban.cpp` | 普通静态数据 | 7.32 KiB / `0x1D48` |
| `MacroPlanner.cpp` | 普通静态数据 | 0 KiB |
| **规划核心合计** |  | **233.49 KiB / `0x3A5F4`** |

其中源码里仍有若干 `__attribute__((section(".dtcm_bss")))` 静态对象。当前 scatter 文件只显式收集 `*(.dtcm_data)` 到 `RW_m_data`，这些 `.dtcm_bss` 命名 section 会由 OCRAM 区的 `.ANY(+ZI)` / `.ANY(+RW)` 收走，因此本次统计按 OCRAM 计算。后续如果真要放入 DTCM，需要同时修改 scatter；如果只是普通 scratch，建议逐步去掉这个容易误导的 section 名。

关键结论：只有高频小数据显式使用 `DTCM_DATA`，大型 scratch 保持普通静态数据并默认留在 OCRAM。当前规划核心静态数据合计约 508.99 KiB，其中显式 DTCM 约 275.50 KiB，普通/OCRAM 约 233.49 KiB。如果普通静态 RAM 重新落入 DTCM，会同时超过 `RW_m_data` 上限和 448 KiB 物理 DTCM。

## 6. 栈内存统计

用 `-fstack-usage` 粗测当前 PC 编译结果，32 KiB 栈已经避开了原先的巨型局部数组风险，但仍建议上板后用 watermark 或 map 文件复核。

| 函数/路径 | 当前栈占用 | 结论 |
| --- | ---: | --- |
| `Strategy::local_clear_bomb_route` | 最大约 3.69 KiB | 单帧可控 |
| `Strategy::evaluate_and_assign_bombs` | 最大约 3.38 KiB | 单帧可控 |
| `PlanningCommon::get_bomb_push_path` | 约 2.53 KiB | 单帧可控 |
| `Strategy::dfs_bomb_sequence` | 最大约 2.39 KiB/层 | 深度最多 `MAX_BOMBS=3`，可控 |
| `Exploration::plan_optimal_patrol` | 外层约 1.70 KiB，DFS 约 1.63 KiB/层 | 已限制 DFS 深度 16，32 KiB 栈下偏紧但可控 |
| `Sokoban::ida_star_search` | 最大约 1.45 KiB/层 | 仍是深解关卡的主要栈风险 |

如果最终推箱 IDA* 经常超过 20 层，建议把 `ARM_LIB_STACK` 提到 64 KiB，或者把 `ida_star_search` 改为显式搜索栈/外部 scratch。当前优先级上，必须先保证大型 scratch 不被误标为 `DTCM_DATA`。

## 7. PC 端验证

PC 端仍可用 `main.cpp` 做桌面验证：

```powershell
g++ -O3 planning\main.cpp planning\Sokoban.cpp planning\Exploration.cpp planning\PlanningCommon.cpp planning\Strategy.cpp planning\MacroPlanner.cpp -o planning\solver.exe
```

本轮 smoke test 使用当前 `map_input.txt` 通过，输出包含 `SOKOBAN`，未出现 `FAILED`。
