# Algorithm_Planning 内存消耗统计分析

## 项目配置参数
- **地图尺寸**: 12×16 格
- **最大箱子数**: 4 个
- **最大炸弹数**: 4 个  
- **最大路径长度**: 100 步
- **置换表大小**: 16384 条目

## 数据结构大小

### 基础类型大小
- `point` (网格坐标): 2 字节 (2×int8_t)
- `int8_t`: 1 字节
- `uint8_t`: 1 字节
- `uint16_t`: 2 字节
- `uint32_t`: 4 字节
- `float`: 4 字节
- `bool`: 1 字节

## 一、Sokoban 求解器内存统计

### 1.1 成员变量（class Sokoban 内部）

| 字段名 | 数据类型 | 大小计算 | 内存占用 |
|--------|---------|---------|---------|
| `map` | `std::array<std::array<int8_t, 12>, 16>` | 16×12×1 | **192 字节** |
| `player_start` | `point` | 2 | **2 字节** |
| `initial_bombs` | `StaticArray<point, 4>` | 4×2+4 | **12 字节** |
| `initial_targets` | `StaticArray<point, 4>` | 4×2+4 | **12 字节** |
| `final_path` | `StaticArray<point, 100>` | 100×2+4 | **204 字节** |
| `initial_state` | `GameState 结构体` | 见下表 | **29 字节** |
| `ZOBRIST_BOX` | `uint32_t[16][12]` | 16×12×4 | **768 字节** |
| `ZOBRIST_SPECIFIC_BOX` | `uint32_t[4][16][12]` | 4×16×12×4 | **3,072 字节** |
| `ZOBRIST_PLAYER` | `uint32_t[16][12]` | 16×12×4 | **768 字节** |
| `ZOBRIST_TARGET` | `uint32_t[4]` | 4×4 | **16 字节** |
| `TT` (置换表) | `TTEntry[16384]` | 16384×4* | **65,536 字节** |
| `path_hashes` | `uint32_t[100]` | 100×4 | **400 字节** |
| `is_dead` | `bool[16][12]` | 16×12×1 | **192 字节** |
| `specific_is_dead` | `bool[4][16][12]` | 4×16×12×1 | **768 字节** |
| `t_dist` | `int16_t[4][16][12]` | 4×16×12×2 | **1,536 字节** |

*TTEntry 结构体: `uint16_t sig` (2字节) + `uint8_t remaining` (1字节) = 3字节，编译器可能对齐到4字节

#### GameState 结构体详解
```cpp
struct GameState {
    point player;              // 8 字节
    int8_t box_x[4];          // 4 字节
    int8_t box_y[4];          // 4 字节
    uint8_t box_ids[4];       // 4 字节
    uint8_t num_boxes;        // 1 字节
    uint8_t target_mask;      // 1 字节
    uint32_t hash;            // 4 字节
    (对齐填充)               // ~2 字节
};
```
**GameState 总计: 29 字节**

### 1.2 全局静态变量（sokoban.cpp）

| 变量名 | 数据类型 | 大小计算 | 内存占用 |
|--------|---------|---------|---------|
| `solver` | `Sokoban` 对象 | 上表汇总 | **73,758 字节** |
| `bfs_visited_gen` | `uint16_t[16][12]` | 16×12×2 | **384 字节** |
| `current_gen` | `uint16_t` | 1 | **2 字节** |
| `bfs_q` | `point[256]` | 256×2 | **512 字节** |
| `bfs_dist` | `int8_t[16][12]` | 16×12×1 | **192 字节** |

**Sokoban 模块全局变量总计: 74,858 字节**

---

## 二、Exploration 巡图规划内存统计

### 2.1 成员变量（class Exploration 内部）

| 字段名 | 数据类型 | 大小计算 | 内存占用 |
|--------|---------|---------|---------|
| `obs_points` | `StaticArray<ObsPoint, 32>` | 32×16+4** | **516 字节** |
| `total_entities` | `uint8_t` | 1 | **1 字节** |
| `cached_level` | `SokobanLevel 结构体` | 见下表 | **333 字节** |
| `cost_matrix` | `float[32][32]` | 32×32×4 | **4,096 字节** |

**ObsPoint 结构体详解**
```cpp
struct ObsPoint {
    point pos;           // 2 字节
    float target_yaw;    // 4 字节
    uint8_t entity_id;   // 1 字节
    bool is_box;         // 1 字节
    (对齐填充)          // 8 字节 (对齐到16字节边界)
};
```
**ObsPoint 总计: 16 字节 (可能对齐)**

#### SokobanLevel 结构体详解
```cpp
struct SokobanLevel {
    std::array<std::array<int8_t, 12>, 16> map;  // 192 字节
    point player_start;                          // 2 字节
    point boxes[4];                              // 8 字节
    uint8_t box_count;                           // 1 字节
    uint8_t box_ids[4];                          // 4 字节
    point targets[4];                            // 8 字节
    uint8_t target_count;                        // 1 字节
    point bombs[4];                              // 8 字节
    uint8_t bomb_count;                          // 1 字节
    (对齐填充)                                  // ~7 字节
};
```
**SokobanLevel 总计: 333 字节**

### 2.2 全局对象成员（class Exploration 内部）

| 变量名 | 数据类型 | 大小计算 | 内存占用 |
|--------|---------|---------|---------|
| `patrol_planner` | `Exploration` 对象 | 上表汇总 | **4,946 字节** |

**Exploration 类全局对象大小: 4,946 字节**

### 2.3 函数内部静态变量（静态存储区，非栈空间）

| 变量名 | 位置 | 数据类型 | 大小计算 | 内存占用 | 存储位置 |
|--------|------|---------|---------|---------|---------|
| `dp` | `plan_optimal_patrol()` 中 | `static float[256][32]` | 256×32×4 | **32,768 字节** | DTCM 静态区 |
| `parent` | `plan_optimal_patrol()` 中 | `static uint8_t[256][32]` | 256×32×1 | **8,192 字节** | DTCM 静态区 |
| `dist` | `bfs_shortest_path()` 中 | `static int8_t[16][12]` | 16×12×1 | **192 字节** | DTCM 静态区 |

⚠️ **重要说明**：虽然这些变量在函数内部，但因为用了 `static` 关键字，它们存储在**静态数据段**，**不占用函数栈空间**。这是代码注释"放在 DTCM 防止局部数组爆栈"的原因。

**Exploration 模块全局+函数内静态变量总计: 46,098 + 40,960 = 87,058 字节**

⚠️ **注解**：合并后的 87 KB 包括全局成员变量 (5 KB) 和函数内部的 `static` 数组 (41 KB)

---

## 三、存储位置分析

根据代码中的标记：
```cpp
__attribute__((section(".dtcm_data"))) Sokoban solver;           // 放在 DTCM 中
__attribute__((section(".dtcm_data"))) Exploration patrol_planner; // 放在 DTCM 中
__attribute__((section(".ramfunc"))) ... ida_star_search();       // 函数放在 DTCM 中执行
__attribute__((section(".ramfunc"))) ... plan_optimal_patrol();  // 函数放在 DTCM 中执行
```

### DTCM (紧耦合内存) 分布
- `solver` 对象: **~73.8 KB**
- `patrol_planner` 对象: **~4.9 KB**
- 函数内 `static` 变量 (`dp`, `parent`, `dist`): **~41 KB**
- 其他全局静态变量 (bfs_* 等): **~1.1 KB**
- **小计: ~120.8 KB**

### 栈空间（函数运行时临时变量）
- **不包含 `static` 变量**（这些在静态区）
- 接收参数、局部变量、返回地址等
- 预留空间: **32 KB** (系统配置)
- **当前 Algorithm_Planning 栈占用**: 极小（无大型栈临时数组）

---

## 四、内存消耗汇总

| 模块 | 静态存储 (KB) | 注释 |
|------|---------------|------|
| **Sokoban 求解器** | 73.8 | 含 Zobrist 表、置换表、死锁表等 |
| **Exploration 规划器** | 4.9 | 含缓存地图、代价矩阵等 |
| **函数内 static 变量** | 41.0 | `dp[256][32]`、`parent[256][32]` 等（在 DTCM 静态区，非栈空间） |
| **其他全局变量** | 1.1 | BFS 队列、访问标记等 |
| **总计** | **120.8 KB** | **全部存储在 DTCM 静态存储区** |

**关键说明**：
- ✅ **零栈占用**：所有大数组都用 `static` 修饰，存储在静态区，不会爆栈
- ✅ **32 KB 栈预留**：系统为函数调用栈预留 32 KB，当前 Algorithm_Planning 模块栈占用极小

### 关键数据结构内存占比
```
DTCM 静态存储区分布 (总 120.8 KB):
├─ Sokoban 模块
│  ├─ 置换表 TT[16384]        : 65.5 KB  (54.2%)
│  ├─ Zobrist 哈希表          : 4.6 KB   (3.8%)
│  ├─ 死锁预计算表            : 2.5 KB   (2.1%)
│  └─ 其他                    : 1.2 KB   (1.0%)
│
├─ Exploration 模块
│  ├─ 函数内 dp & parent      : 40.8 KB  (33.8%) 
│  ├─ 代价矩阵                : 4.0 KB   (3.3%)
│  └─ 其他                    : 1.0 KB   (0.8%)
│
└─ 其他全局变量               : 1.1 KB   (0.9%)
```

---

## 五、性能优化建议

### 高优先级优化

1. **置换表大小优化** ⭐⭐⭐
   - 当前: `TT_SIZE = 16384` (65.5 KB)
   - 建议: 考虑减小到 8192 (32.8 KB) 或 4096 (16.4 KB) 根据实际命中率
   - 预期效果: 可节省 32-49 KB

2. **动态规划表优化** ⭐⭐⭐
   - 当前: `dp[256][32]` 和 `parent[256][32]` (40 KB)
   - 建议: 根据实际 entity 数量动态分配，最多 8 个实体 (2^8=256)
   - 预期效果: 实际可能用不到全部行，可按需降低

### 中等优先级优化

3. **Zobrist 表结构优化** ⭐⭐
   - 合并 `ZOBRIST_SPECIFIC_BOX` 的某些空间
   - 使用更小的哈希函数
   - 预期效果: 可节省 0.5-1 KB

4. **使用 `#pragma pack` 减少对齐填充**
   - 可能节省 100-200 字节

### 低优先级优化

5. **BFS 队列大小** ⭐
   - 当前: 256 个点，实际需要最多 192 个 (12×16)
   - 预期效果: 节省 ~128 字节

---

## 六、内存映射总览

### DTCM 物理配置 （RT1064 芯片）
📍 **物理地址范围**: `0x20000000` ~ `0x20070000`  
📊 **物理大小**: `0x70000` = **448 KB**  
⚙️ **默认堆栈配置**:
- 栈 (Stack): 0x8000 = **32 KB**
- 堆 (Heap): 0x2000 = **8 KB**
- **实际可用**: 448 - 32 - 8 = **408 KB**

### Algorithm_Planning 占用情况
```
DTCM 空间分配 (总 408 KB 可用):
├─ solver (Sokoban)           73.8 KB  ▓▓▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
├─ patrol_planner (Exploration) 4.9 KB  ▓░░░░
├─ 函数内 static 变量         41.0 KB  ▓▓▓▓▓░░░░
├─ 其他全局变量               1.1 KB   ░░
├─ 栈/堆预留空间               40 KB   ▓▓▓▓░░
└─ 剩余可用                   ~247 KB  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░

占用百分比: 120.8 KB / 408 KB = 29.6% ✅ 充足
```

### 栈空间配置
```
预留栈空间 (32 KB):
├─ Algorithm_Planning 栈占用: ~1 KB  (极小，无大型临时数组)
└─ 剩余栈空间: ~31 KB         (充足)
```

---

## 七、总结

**Algorithm_Planning 模块总内存占用**: **~120.8 KB**

### 与 DTCM 的关系
- **DTCM 物理大小**: **448 KB** (0x70000 字节)
- **DTCM 可用大小**: **408 KB** (扣除 32 KB 栈 + 8 KB 堆)
- **当前占用率**: 120.8 KB / 408 KB = **29.6%** ✅ **充足安全**
- **剩余空间**: ~**287 KB** 可用于扩展

### 栈空间评估
- **栈预留**: 32 KB
- **Algorithm_Planning 栈占用**: ~1 KB（极小）
- **栈溢出风险**: ✅ **无风险**（所有大数组都用 `static` 修饰）

### 设计亮点
- 代码作者通过使用 `static` 关键字，将大型 DP 表放在静态存储区
- 源代码注释明确说明："放在 DTCM 防止局部数组爆栈"
- 这是一个 **零栈占用设计**，非常安全高效

### 性能优化优先级
- **优化前瓶颈**: 置换表占用了总内存的 54%，DP 表占用 27%
- **建议首先优化**: 根据实际算法命中率，调整置换表大小（可节省 32-49 KB）
- **内存充足有余**: 当前配置完全满足需求，不存在内存压力
