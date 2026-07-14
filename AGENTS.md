# AGENTS.md

这份文件是给后续代码助手看的项目级说明。进入本仓库后，应先参考这里的规则，再动代码。

## 编码规则

- 本仓库源码是 UTF-8。
- 在 PowerShell 里读取带中文注释的源码时，必须使用：

```powershell
Get-Content -Encoding UTF8
```

- Windows 默认编码可能是 GB2312/936，省略 `-Encoding UTF8` 时，中文注释可能会显示成乱码。
- 使用 `apply_patch` 修改文件时，优先用英文函数名、变量名、`case GamePhase::...` 等 ASCII 代码行作为上下文锚点，不要依赖中文注释做精确匹配。
- 不要为了修编码或换行符而整文件重写。

## 写代码习惯

- 保持改动范围小，只改和当前任务相关的代码。
- 代码前的缩进尽量为4的倍数。
- 不要回退用户已有改动，也不要顺手重构无关文件。
- 手动改源码优先使用 `apply_patch`。
- 嵌入式规划代码里优先使用已有的 `StaticArray` 和固定容量数组，避免 `new` / `malloc` / `std::vector` 进入热路径。
- 新增/改动代码时要写注释，优先使用中文，不要写句号，方便项目维护者阅读。
- 但不要随便堆注释；只在逻辑不直观、状态机跳转复杂、硬件/内存约束容易踩坑时加简洁中文注释。
- 核心函数、重要接口、较长函数使用 Doxygen 风格段注释：

```cpp
/// \brief 载入策略层给出的炸弹任务
/// \param tasks 炸弹任务数组，可为空
/// \param count 有效任务数量
///
/// \details
/// 函数会把任务绑定到初始炸弹编号，并建立 wall_clear_mask，
/// 使搜索中可通过 blown_mask 快速判断墙体是否已被炸平。
```

- 小的辅助函数可使用 `//` 单行注释，简要说明重要功能即可。

## 编译和验证

- 本项目目标是 RT1064 + Keil/armclang。
- 桌面 `g++` 只能用于算法模块语法检查，不能完整替代 Keil 编译。
- `GameManage.cpp` 这类文件会包含 RT1064 SDK 头，例如 `zf_common_headfile.h`，桌面环境可能无法完整编译。

常用本地语法检查命令：

```powershell
g++ -std=c++17 -fsyntax-only -Iproject/Core -Iproject/App -Iproject/Algorithm project/Algorithm/PlanningCommon.cpp project/Algorithm/StrategyCommon.cpp project/Algorithm/StrategyPhase1.cpp project/Algorithm/StrategyPhase2.cpp project/Algorithm/Exploration.cpp project/Algorithm/Sokoban.cpp project/Algorithm/MacroPlanner.cpp
g++ -std=c++17 -fsyntax-only -Iproject/Core -Iproject/App -Iproject/Algorithm -Iproject/Subsystem -Iproject/Device -Iproject/Driver -Iproject project/App/GameManageDemo.cpp project/App/GameManageMock.cpp
```

最终仍然要用 Keil 编译，并检查 map 文件里的 ITCM / DTCM / OCRAM 占用。

## Planning 模块职责

核心规划文件：

- `PlanningCommon.*`：网格寻路、推箱/推炸弹路径展开、地图状态更新。
- `Strategy.*`：第一阶段和第二阶段的炸弹任务选择。
- `Exploration.*`：巡图观测参考序列生成、语义匹配。
- `MacroPlanner.*`：在线宏动作调度，负责根据参考序列、观测状态和语义知识决定下一步宏动作。
- `Sokoban.*`：最终推箱求解器。

`MacroPlanner` 不直接持有真实世界状态。真实状态以 `GameManager::logical_level`、当前位置、当前 yaw、当前炸弹任务为准。

`GameManager::plan_next_macro_action()` 负责把这些真实状态填入 `MacroPlanContext`，再调用：

```cpp
macro_planner.plan_next_action(ctx, action)
```

这样可以避免 `GameManager` 和 `MacroPlanner` 各自维护一份地图而不同步。

## GameManage 主流程

高级阶段的整体流程：

```text
ART1 地图
 -> Strategy 计算第一阶段炸弹任务
 -> Exploration 生成参考巡图序列
 -> MacroPlanner 在线选择下一条宏动作
 -> GameManage 把宏动作转换为 RobotTask 队列
 -> Tracker / Vision 执行移动和识别
 -> MacroPlanner 根据观测结果更新知识状态
 -> 语义绑定
 -> Strategy 计算第二阶段炸弹任务
 -> Sokoban 求最终推箱路径
 -> Tracker 执行最终路径
```

`GameManage.cpp` 里的关键 helper：

- `plan_next_macro_action()`：构造 `MacroPlanContext`，调用 `MacroPlanner`，失败时 fallback 到 `Exploration`。
- `start_macro_action()`：记录当前 `MacroAction`，并拆成底层 `RobotTask` 队列。
- `prepare_phase2_solver()`：重新计算第二阶段炸弹任务、绑定语义、重新加载 Sokoban 求解器；也用于第二阶段求解失败后的 dynamic fallback 重试。

## 内存注意事项

- `MAX_PATH_LENGTH` 当前应为 200。
- 只有明确需要放入 DTCM 的数据才使用 `DTCM_DATA` 标记。
- 大型规划 scratch buffer 不要标记 `DTCM_DATA`，默认留在 OCRAM，避免挤占 DTCM。
- 大型规划函数不要随便放进 `.ramfunc` / ITCM，除非明确有性能理由并确认 ITCM 空间足够。
