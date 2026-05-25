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
- 巡图/策略的大 scratch 通过 `MCU_OCRAM_BSS` 放入 `.ocram_bss`，避免挤占 DTCM 和栈。
- 巡图 DFS 深度硬限制为 `PATROL_DFS_FRAME_LIMIT = 16`，用于适配 32 KiB 栈。

## 5. RT1064 内存统计

目标约束：

- DTCM 物理容量：448 KiB，`0x70000`
- `RW_m_data` 区域最大值：408 KiB，`0x66000`
- `ARM_LIB_HEAP`：8 KiB，`0x2000`
- `ARM_LIB_STACK`：32 KiB，`0x8000`

统计方法：按当前源码中的固定数组、全局对象和 section 标注统计，并用 GCC 13.1 目标文件复核。最终上板前仍需以 Keil/armclang map 文件为准。

### 5.1 显式 DTCM 占用

| 模块 | 主要对象 | 占用 |
| --- | --- | ---: |
| `Sokoban.cpp` | `TT[32768]` + `solver` | 182.22 KiB |
| `PlanningCommon.cpp` | `b_ws`、`can_player_reach` 工作区 | 7.91 KiB |
| `Strategy.cpp` | `strategic_planner`、DFS 距离场/DP 缓存 | 33.75 KiB |
| `Exploration.cpp` | `patrol_planner` | 40.38 KiB |
| `MacroPlanner.cpp` | `macro_planner` | 2.34 KiB |
| **合计** |  | **266.59 KiB / `0x42A60`** |

显式 DTCM 合计 266.59 KiB，低于 `RW_m_data` 408 KiB 上限，剩余约 141.41 KiB。加上 8 KiB heap 和 32 KiB stack 后，DTCM 物理占用约 306.59 KiB / `0x4CA60`，距离 448 KiB 物理容量仍剩余约 141.41 KiB。

### 5.2 OCRAM / 普通静态 RAM

已显式标到 `.ocram_bss` 的大工作区：

| 模块 | 主要对象 | 占用 |
| --- | --- | ---: |
| `PlanningCommon.cpp` | 时间搜索工作区、普通 BFS 工作区、时间路径 parent 表 | 19.91 KiB |
| `Strategy.cpp` | DFS 候选队列 scratch | 8.03 KiB |
| `Exploration.cpp` | 巡图 DFS / cache / materialize scratch | 133.38 KiB |
| **合计** |  | **161.31 KiB / `0x28540`** |

未显式标到 `.dtcm_*` / `.ocram_bss` 的普通静态区约 36.72 KiB / `0x92E0`。因此非 DTCM 静态 RAM 合计约 198.03 KiB / `0x31820`。

关键结论：必须在 scatter/linker 里把 `.ocram_bss` 放到 OCRAM/普通 SRAM，不要放进 `RW_m_data`。如果所有静态 RAM 都落入 DTCM，静态占用会变成 464.63 KiB，已经超过 408 KiB 的 `RW_m_data` 上限约 56.63 KiB；再加 heap/stack 后约 504.63 KiB，也超过 448 KiB 物理 DTCM。

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

如果最终推箱 IDA* 经常超过 20 层，建议把 `ARM_LIB_STACK` 提到 64 KiB，或者把 `ida_star_search` 改为显式搜索栈/外部 scratch。当前优先级上，必须先保证 `.ocram_bss` 正确放到 OCRAM。

## 7. PC 端验证

PC 端仍可用 `main.cpp` 做桌面验证：

```powershell
g++ -O3 planning\main.cpp planning\Sokoban.cpp planning\Exploration.cpp planning\PlanningCommon.cpp planning\Strategy.cpp planning\MacroPlanner.cpp -o planning\solver.exe
```

本轮 smoke test 使用当前 `map_input.txt` 通过，输出包含 `SOKOBAN`，未出现 `FAILED`。
