# Algorithm_Planning 内存消耗统计分析（实测更新版）

更新时间：2026-04-06  
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
| 最大箱子数 `MAX_BOXES` | 3 |
| 最大炸弹数 `MAX_BOMBS` | 3 |
| 最大实体数 `MAX_ENTITIES` | 6 |
| 实体掩码总数 `MAX_ENTITY_MASK` | 64 |
| 最大观测点 `MAX_OBS_POINTS` | 24 |
| 最大路径长度 `MAX_PATH_LENGTH` | 100 |
| 置换表大小 `TT_SIZE` | 16384 |

---

## 3. 链接器实测：Algorithm_Planning 关键符号

以下大小全部来自 `rt1064.map` 的符号表。

### 3.1 Sokoban 模块

| 符号 | 大小(字节) | 说明 |
|---|---:|---|
| `solver` | 72152 | `sokoban.o(.dtcm_data)` 主对象 |
| `current_gen` | 2 | `sokoban.o(.bss)` |
| `bfs_visited_gen` | 384 | `sokoban.o(.bss)` |
| `bfs_q` | 384 | `sokoban.o(.bss)` |
| `bfs_dist` | 192 | `sokoban.o(.bss)` |
| `temp_parent` (Mode0) | 384 | `ida_star_search` 模板静态区 |
| `temp_parent` (Mode1) | 384 | `ida_star_search` 模板静态区 |
| **小计** | **73882** | **72.15 KiB** |

### 3.2 Exploration 模块

| 符号 | 大小(字节) | 说明 |
|---|---:|---|
| `patrol_planner` | 512 | `exploration.o(.dtcm_data)` 对象本体 |
| `dp_ram` | 30984 | DP 工作区（含 dist/dp/parent/bfs_queue） |
| `b_ws` | 7298 | 推炸弹路径工作区 |
| `dist` | 192 | `bfs_shortest_path` 内部静态数组 |
| `multi_maps` | 872 | 多分支地图缓存 |
| **小计** | **39858** | **38.92 KiB** |

### 3.3 Strategy 模块（同属 Algorithm_Planning）

| 符号 | 大小(字节) | 说明 |
|---|---:|---|
| `strategic_planner` | 218 | `strategy.o(.dtcm_data)` 对象本体 |
| `dfs_player_vis` | 768 | DFS 缓冲 |
| `dfs_dist_box` | 4608 | 盒子推演距离表 |
| `dfs_dist_bomb` | 4608 | 炸弹推演距离表 |
| `state_cost` | 1536 | `fast_push_bfs` 静态数组 |
| `q` | 6144 | `fast_push_bfs` 静态队列 |
| **小计** | **17882** | **17.46 KiB** |

---

## 4. 模块总占用汇总（Algorithm_Planning）

| 子模块 | 字节 | KiB |
|---|---:|---:|
| Sokoban | 73882 | 72.15 |
| Exploration | 39858 | 38.92 |
| Strategy | 17882 | 17.46 |
| **总计** | **131622** | **128.54** |

说明：

- 以上为 Algorithm_Planning 目录三大核心模块的静态数据总和（含对象 + 文件级静态 + 函数内静态）。
- 旧文档仅统计 Sokoban + Exploration 时，总量约 111.07 KiB；本版补入 Strategy 后更完整。

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
  $$131622 / 417792 \approx 31.5\%$$
- 对物理 DTCM（448 KiB）占比：
  $$131622 / 458752 \approx 28.7\%$$

结论：当前 Algorithm_Planning 内存占用安全，仍有较大扩展余量。

---

## 6. 与本次构建结果对应

来自 `uv4.log`：

- `Code = 136830`
- `RO-data = 8162`
- `RW-data = 126972`
- `ZI-data = 57956`
- 编译结果：`0 Error(s), 0 Warning(s)`

说明：该组数字为整工程镜像维度；第 5 章是 Algorithm_Planning 子模块维度，两者统计口径不同。

---

## 7. 主要结论与优化建议

### 7.1 主要结论

1. 当前 Algorithm_Planning 实测总占用约 128.54 KiB。
2. 内存大头依旧来自：
   - `solver`（72.15 KiB）
   - `dp_ram`（30.27 KiB）
   - Strategy 的 BFS/DFS 工作区（17.46 KiB）

### 7.2 优化优先级（按收益排序）

1. **调整 TT_SIZE（Sokoban）**
   - 当前 `TT_SIZE = 16384`。
   - 可试 8192，理论上可显著降低 `solver` 占用。
2. **压缩 DP 维度（Exploration）**
   - `dp_ram` 占用约 30 KiB，是第二大块。
   - 可按赛段动态裁剪状态维度（例如减少无效 mask 与观测点空间）。
3. **复用 Strategy 的临时队列**
   - `state_cost + q` 合计 7680 字节，可考虑与其它阶段工作区时分复用。

---

## 8. 更新备注

- 本文档后续请优先以 `rt1064.map` 的符号大小作为基准，而不是手工估算。
- 每次算法结构调整（尤其数组维度、`static`/`section` 变更）后，建议同步刷新本文件。
