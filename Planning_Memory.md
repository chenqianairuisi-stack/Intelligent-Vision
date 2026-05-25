# Algorithm_Planning 内存消耗统计分析（实测更新版）

更新时间：2026-04-13  
统计口径：优先采用 Keil 链接器 map 实测值，其次为源码结构推导。

---

## 1. 数据来源

- 配置常量：`project/Core/system_config.h`
- 链接器映射：`project/mdk/Listings/rt1064.map`
- 构建日志：`project/mdk/.vscode/uv4.log`

---

## 2. 当前项目配置参数（源码）

| 参数 | 当前值 |
|---|---|
| 地图尺寸 | 12 x 16 |
| 地图总格子 | 192 |
| 最大箱子数 `MAX_BOXES` | 10 |
| 最大炸弹数 `MAX_BOMBS` | 3 |
| 最大实体数 `MAX_ENTITIES` | 20 |
| 实体掩码总数 `MAX_ENTITY_MASK` | 1048576 |
| 最大观测点 `MAX_OBS_POINTS` | 80 |
| 最大路径长度 `MAX_PATH_LENGTH` | 100 |
| 置换表大小 `TT_SIZE` | 65536 |

---

## 3. 链接器实测：Algorithm_Planning 关键符号

以下大小全部来自 `rt1064.map` 的符号表。

### 3.1 Sokoban 模块

| 符号 | 大小(字节) | 说明 |
|---|---:|---|
| `solver` | 14164 | `sokoban.o(.dtcm_data)` 主对象 |
| `TT` | 262144 | 置换表（`TTEntry TT[TT_SIZE]`） |
| `current_gen` | 2 | `sokoban.o(.bss)` |
| `bfs_visited_gen` | 384 | `sokoban.o(.bss)` |
| `bfs_q` | 384 | `sokoban.o(.bss)` |
| `bfs_dist` | 192 | `sokoban.o(.bss)` |
| `temp_parent` (Mode0) | 384 | `ida_star_search` 模板静态区 |
| `temp_parent` (Mode1) | 384 | `ida_star_search` 模板静态区 |
| **小计** | **278038** | **271.52 KiB** |

### 3.2 Exploration 模块

| 符号 | 大小(字节) | 说明 |
|---|---:|---|
| `patrol_planner` | 1220 | `exploration.o(.dtcm_data)` 对象本体 |
| `p_ws` | 52872 | 规划工作区（距离矩阵 + BFS 队列） |
| `b_ws` | 7298 | 推炸弹路径工作区 |
| `dist` | 192 | `bfs_shortest_path` 内部静态数组 |
| `multi_maps` | 1012 | `plan_optimal_patrol` 内部静态多分支地图 |
| **小计** | **62594** | **61.13 KiB** |

### 3.3 Strategy 模块（同属 Algorithm_Planning）

| 符号 | 大小(字节) | 说明 |
|---|---:|---|
| `strategic_planner` | 253 | `strategy.o(.dtcm_data)` 对象本体 |
| `dfs_player_vis` | 768 | DFS 缓冲 |
| `dfs_dist_box` | 15360 | 盒子推演距离表 |
| `dfs_dist_bomb` | 4608 | 炸弹推演距离表 |
| `state_cost` | 1536 | `fast_push_bfs` 静态数组 |
| `q` | 6144 | `fast_push_bfs` 静态队列 |
| **小计** | **28669** | **28.00 KiB** |

---

## 4. 模块总占用汇总（Algorithm_Planning）

| 子模块 | 字节 | KiB |
|---|---:|---:|
| Sokoban | 278038 | 271.52 |
| Exploration | 62594 | 61.13 |
| Strategy | 28669 | 28.00 |
| **总计** | **369301** | **360.65** |

说明：

- 以上为 Algorithm_Planning 目录三大核心模块的静态数据总和（含对象 + 文件级静态 + 函数内静态）。
- 与旧版相比，本次主要变化来自 `TT_SIZE` 扩大及 Exploration 工作区符号由 `dp_ram` 调整为 `p_ws`。

---

## 5. 与 DTCM 容量关系

### 5.1 硬件与链接配置

- DTCM 物理容量：448 KiB（`0x70000`）
- `RW_m_data` 区域最大值：408 KiB（`0x66000`）
- 额外执行区：
  - `ARM_LIB_HEAP`：8 KiB（`0x2000`）
  - `ARM_LIB_STACK`：32 KiB（`0x8000`）

### 5.2 占比

- 对 `RW_m_data`（408 KiB）占比：
  $$369301 / 417792 \approx 88.4\%$$
- 对物理 DTCM（448 KiB）占比：
  $$369301 / 458752 \approx 80.5\%$$

结论：当前 Algorithm_Planning 内存占用仍可运行，但余量已明显收紧，后续扩展需重点关注 DTCM 预算。

---

## 6. 与本次构建结果对应

来自 `uv4.log`：

- `Code = 135324`
- `RO-data = 9596`
- `RW-data = 363680`
- `ZI-data = 58168`
- 编译结果：`0 Error(s), 0 Warning(s)`

说明：该组数字为整工程镜像维度；第 5 章是 Algorithm_Planning 子模块维度，两者统计口径不同。

---

## 7. 主要结论与优化建议

### 7.1 主要结论

1. 当前 Algorithm_Planning 实测总占用约 360.65 KiB。
2. 内存大头依次为：
   - `TT`（256.00 KiB）
   - `p_ws`（51.63 KiB）
   - `dfs_dist_box`（15.00 KiB）

### 7.2 优化优先级（按收益排序）

1. **下调 TT_SIZE（Sokoban）**
   - 当前 `TT_SIZE = 65536`，`TT` 占 262144 字节。
   - 若降到 32768，可理论节省约 128 KiB；降到 16384，可节省约 192 KiB。
2. **压缩 Exploration 的 `p_ws` 工作区**
   - `p_ws` 当前 52872 字节，是第二大块。
   - 可考虑按任务规模动态裁剪距离矩阵维度，或对不可达状态采用稀疏存储。
3. **继续复用 Strategy 的临时队列**
   - `state_cost + q` 合计 7680 字节。
   - 可与其它阶段工作区做时分复用，进一步降低峰值占用。

---

## 8. 更新备注

- 本文档后续请优先以 `rt1064.map` 的符号大小作为基准，而不是手工估算。
- 每次算法结构调整（尤其数组维度、`static`/`section` 变更）后，建议同步刷新本文件。
- 若仅通过终端表格查看 map 行，可能出现字段截断，建议直接读取 map 原文行核对字节数。
