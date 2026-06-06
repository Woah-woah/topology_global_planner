# topology_global_planner

`topology_global_planner` 是一个面向 RoboMaster / Nav2 场景的 **弱约束拓扑全局规划器插件**。

它不是重新写一个 A*，也不是替代 Nav2 原来的全局规划器；它更像是一个 **wrapper planner**：

```text
RViz/Nav2 Goal/BasicNavigator
        ↓
Nav2 planner_server
        ↓
TopoPlanner
        ↓
根据人工标注的 topology.yaml 做 Region-Connector 拓扑选路
        ↓
在 connector portal 上采样多个候选通过点
        ↓
调用原来的 InnerPlanner，例如 Navfn/Smac，计算真实 costmap 路径
        ↓
选择总代价最低的一组 through-points
        ↓
拼接成完整 nav_msgs/Path 返回给 Nav2
```

所以这个包的核心思想是：

> **拓扑地图负责“语义上该从哪几个区域走”，原来的 Nav2 全局规划器负责“几何上具体怎么走”。**

这适合处理 RM 场地里普通 2D costmap 难以表达的规则，例如：

- 单向通道，例如只能下不能上的台阶；
- 有多个区域和多个连接口时，先选一条逻辑路线；
- 希望全局路径经过某些人工标注的 connector；
- 避免普通 A* 在有捷径时反向穿越不该走的区域；
- 用 portal 线段表示真正的“门”，而不是一个固定死点。

---

## 1. 包的定位

这个 planner 的定位可以这样理解：

```text
普通 Nav2：
start -> costmap global planner -> goal

本包：
start -> topology route -> portal optimizer -> inner planner segments -> goal
```

它不会改 local planner，也不会直接控制底盘。

它只负责 planner_server 的 `nav2_core::GlobalPlanner` 插件部分，最后输出的仍然是标准的：

```cpp
nav_msgs::msg::Path
```

因此下游仍然可以继续使用 Nav2 的 controller，例如 DWB、TEB、MPPI、RegulatedPurePursuit 等。

---

## 2. 适用场景

比较适合：

- RMUC/RMUL 哨兵机器人全局导航；
- 有明确区域划分的比赛场地；
- 需要表达单向 connector、特殊通道、推荐路线；
- 想保留原始 Nav2 全局规划器能力，但在前面加一层拓扑搜索；
- 想让 RViz 直接点 Nav2 Goal 时也能走拓扑逻辑，而不是只在决策树里手动打点。

不太适合：

- 完全未知环境；
- 没有人工拓扑地图的纯自主探索；
- 动态障碍很多、主要靠局部避障解决的问题；
- 想让 planner 直接完成复杂战术决策。

这个包只负责“全局路径怎么选”，不负责“什么时候进攻、什么时候回补给区、什么时候逃跑”。这些仍然更适合放在行为树或决策树里。

---

## 3. 文件结构

当前版本已经把原来堆在一个 `.cpp` 里的逻辑拆成了多个模块：

```text
topology_global_planner/
├── CMakeLists.txt
├── package.xml
├── topology_global_planner_plugin.xml
├── README.md
├── MODULARIZATION.md
├── config/
│   ├── nav2_params_snippet.yaml
│   ├── topology.yaml
│   └── topo.yaml
├── include/
│   └── topology_global_planner/
│       └── topology_global_planner.hpp
└── src/
    ├── planner_lifecycle.cpp
    ├── topology_io.cpp
    ├── geometry_utils.cpp
    ├── topology_search.cpp
    ├── portal_optimizer.cpp
    └── path_planning.cpp
```

各文件职责如下：

| 文件 | 主要职责 |
|---|---|
| `planner_lifecycle.cpp` | Nav2 插件生命周期、参数读取、InnerPlanner 加载、`createPlan()` 主流程 |
| `topology_io.cpp` | 读取 `topology.yaml`，解析 regions/connectors，构建拓扑图 |
| `geometry_utils.cpp` | 点在多边形内判断、边界判断、区域查找、路径是否留在 region 内 |
| `topology_search.cpp` | 在 region graph 上做拓扑搜索，得到 region path 和 connector 序列 |
| `portal_optimizer.cpp` | portal 采样、粗采样/细采样、调用 InnerPlanner 评估候选点组合 |
| `path_planning.cpp` | 调用 InnerPlanner、拼接路径、fallback、frame 转换、yaw 处理 |
| `MODULARIZATION.md` | 分模块说明，主要给后续维护者看 |

---

## 4. 核心数据结构

主要结构体在：

```text
include/topology_global_planner/topology_global_planner.hpp
```

### 4.1 Region

```cpp
struct Region
{
  std::string id;
  std::string name;
  std::vector<Point2D> polygon;
  Point2D centroid;
};
```

`Region` 表示一个人工标注的大区域，例如：

```text
己方基地附近区域
中央高地区域
公路区域
敌方半场区域
补给区区域
```

它不是 costmap 里的障碍物区域，而是高层语义区域。

### 4.2 Connector

```cpp
struct Connector
{
  std::string id;
  std::string from;
  std::string to;
  std::string mode;

  bool has_portal;
  Point2D portal_start;
  Point2D portal_end;

  bool has_waypoint;
  Point2D waypoint;

  double cost;
};
```

`Connector` 表示两个 region 之间的连接关系。

新版推荐使用 `portal`：

```yaml
portal:
  start: {x: 2.0, y: -0.8}
  end:   {x: 2.0, y:  0.8}
```

这表示一扇“门线”。planner 会在门线上采多个候选点，不再强行只走一个中点。

旧版也兼容 `waypoint`：

```yaml
waypoint: {x: 2.15, y: 0.0}
```

但 waypoint 本质上是一个固定死点，不如 portal 灵活。

---

## 5. 算法流程

一次 `createPlan(start, goal)` 大致流程如下：

```text
1. 把 start 和 goal 转到 global_frame
2. 判断 start 在哪个 region
3. 判断 goal 在哪个 region
4. 如果 start_region == goal_region：
     直接调用 InnerPlanner 从 start 到 goal
5. 如果不在同一区域：
     在拓扑图上搜索 region route
6. 得到 connector 序列
7. 对每个 connector 的 portal 采样多个候选 through-point
8. 对候选点之间的每一段调用 InnerPlanner
9. 用 InnerPlanner 返回路径长度作为真实几何代价
10. 用动态规划选择总代价最低的候选点组合
11. 拼接所有 segment
12. 返回完整 nav_msgs::Path
```

可以理解为两层规划：

```text
拓扑层：
R1 -> R2 -> R3 -> R5

几何层：
start -> portal_C12_best_point -> portal_C23_best_point -> portal_C35_best_point -> goal
```

---

## 6. Portal 采样逻辑

如果 connector 是 portal 线段：

```yaml
connectors:
  - id: C12
    from: R1
    to: R2
    mode: two_way
    portal:
      start: {x: 2.0, y: -0.8}
      end:   {x: 2.0, y:  0.8}
    cost: 1.0
```

假设：

```yaml
portal_sample_count: 5
```

planner 不会只取中点，而是会在 portal 线上采样 5 个候选点。

默认不会采精确端点，避免贴墙、贴边、卡在多边形边界上。

如果开启：

```yaml
portal_adaptive_sampling: true
```

则会先粗采样，再围绕最优粗采样点做一次局部细采样。

这样比“暴力密集采样整个 portal”更省性能，也比“固定中点”更灵活。

---

## 7. Region 约束

普通 InnerPlanner 只看 costmap，它可能会为了路径更短而抄近路。

例如拓扑层希望：

```text
R3 -> R5 -> R4
```

但是 InnerPlanner 在计算 `C35 -> C54` 这一段时，可能走出 R5，绕回 R3 或直接穿到 R4。

为了解决这个问题，本包提供：

```yaml
enforce_region_constraint: true
region_constraint_tolerance: 0.15
```

每一段 path 会有一个 expected region：

```text
start -> C12    expected R1
C12   -> C23    expected R2
C23   -> C35    expected R3
C35   -> C54    expected R5
C54   -> goal   expected R4
```

如果某段 path 大部分跑出了 expected region，这个候选 segment 会被 reject。

注意：这个功能是“规划结果检查”，不是 costmap mask。

也就是说，它不会强行让 InnerPlanner 在 region 内搜索，只会检查 InnerPlanner 算出来的路径是否合法。

如果某个 region 内部本身被障碍物堵死，所有候选都可能失败。调试时可以临时设置：

```yaml
enforce_region_constraint: false
```

---

## 8. 安装与编译

建议放到：

```bash
~/origin_nav2026_developing-jazzy/src/navigation/topology_global_planner
```

编译：

```bash
cd ~/origin_nav2026_developing-jazzy
colcon build --packages-select topology_global_planner
source install/setup.bash
```

如果你在 Humble 工程里使用，也同样是：

```bash
colcon build --packages-select topology_global_planner
```

---

## 9. Nav2 参数配置

把 `config/nav2_params_snippet.yaml` 合并到你的 Nav2 参数文件里。

核心写法：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["TopoPlanner"]

    TopoPlanner:
      plugin: "topology_global_planner::TopologyGlobalPlanner"
      topology_yaml: "/home/hexinyi/origin_nav2026_developing-jazzy/src/navigation/topology_global_planner/config/topology.yaml"
      use_topology: true
      fallback_to_inner_planner: true

      inner_planner_name: "InnerPlanner"
      inner_planner_plugin: "nav2_navfn_planner::NavfnPlanner"

    InnerPlanner:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: false
```

重点：

```yaml
planner_plugins: ["TopoPlanner"]
```

不要把 `InnerPlanner` 也写进 `planner_plugins`。

`InnerPlanner` 是被 `TopoPlanner` 内部加载的。如果你把它也写进 `planner_plugins`，Nav2 会把它当成另一个可直接调用的全局 planner，而不是内部 planner。

---

## 10. 测试切换 route mode 服务

如果已经启动 Nav2，并且 `TopologyGlobalPlanner` 已经成功加载，可以这样测试切换 route mode：

```bash
ros2 service call /switch_route_mode topology_global_planner/srv/SwitchRouteMode "{}"
```

---

## 11. 如果原来用 SmacPlanner2D

如果你的原始全局规划器是 Smac，可以这样改：

```yaml
TopoPlanner:
  plugin: "topology_global_planner::TopologyGlobalPlanner"
  topology_yaml: "/绝对路径/topology.yaml"
  inner_planner_name: "InnerPlanner"
  inner_planner_plugin: "nav2_smac_planner::SmacPlanner2D"

InnerPlanner:
  plugin: "nav2_smac_planner::SmacPlanner2D"
  tolerance: 0.25
  downsample_costmap: false
  allow_unknown: false
  max_iterations: 1000000
  max_on_approach_iterations: 1000
  max_planning_time: 2.0
  motion_model_for_search: "MOORE"
  cost_travel_multiplier: 2.0
  use_final_approach_orientation: false
```

`InnerPlanner` 下面继续写你原来 Smac 的参数即可。

---

## 12. 主要参数说明

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `topology_yaml` | 空 | 拓扑地图 YAML 的绝对路径 |
| `use_topology` | `true` | 是否启用拓扑规划 |
| `fallback_to_inner_planner` | `true` | 拓扑失败时是否退回普通 InnerPlanner |
| `inner_planner_name` | `InnerPlanner` | 内部 planner 的名字 |
| `inner_planner_plugin` | `nav2_navfn_planner::NavfnPlanner` | 内部 planner 插件类型 |
| `allow_nearest_region_fallback` | `false` | 点不在任何 region 时，是否强行归到最近 region |
| `max_nearest_region_distance` | `1.0` | 最近 region fallback 的最大允许距离 |
| `region_boundary_tolerance` | `0.03` | 判断点是否在 region 边界上的容差 |
| `transform_tolerance` | `0.1` | TF 转换容差 |
| `duplicate_pose_tolerance` | `0.02` | 拼接 path 时去重的距离阈值 |
| `portal_sample_count` | `5` | portal 粗采样点数 |
| `portal_adaptive_sampling` | `true` | 是否启用 portal 局部细采样 |
| `portal_refine_sample_count` | `5` | 细采样点数 |
| `enforce_region_constraint` | `true` | 是否检查分段路径必须留在 expected region 内 |
| `region_constraint_tolerance` | `0.15` | region path 检查容差 |

---

## 13. topology.yaml 格式

### 13.1 regions

```yaml
regions:
  - id: R1
    name: home_area
    polygon:
      - {x: -2.0, y: -2.0}
      - {x:  2.0, y: -2.0}
      - {x:  2.0, y:  2.0}
      - {x: -2.0, y:  2.0}
```

说明：

- `id` 必须唯一；
- `name` 只是给人看的；
- `polygon` 是多边形顶点；
- 顶点顺时针或逆时针都可以；
- 多边形最好不要自交；
- region 之间可以轻微重叠，但不建议大面积重叠。

### 12.2 connectors: portal 推荐写法

```yaml
connectors:
  - id: C12
    from: R1
    to: R2
    mode: two_way
    portal:
      start: {x: 2.0, y: -0.8}
      end:   {x: 2.0, y:  0.8}
    cost: 1.0
```

说明：

- `from` 和 `to` 必须引用已存在的 region id；
- `mode: two_way` 表示双向；
- `mode: one_way` 表示只允许 `from -> to`；
- `cost` 是拓扑层代价，越小越优先；
- `portal` 表示两个区域之间的一条可通过门线。

### 12.3 connectors: waypoint 兼容写法

```yaml
connectors:
  - id: C12_old
    from: R1
    to: R2
    mode: two_way
    waypoint: {x: 2.15, y: 0.0}
    cost: 1.0
```

这个写法能用，但不推荐长期使用。

因为 waypoint 只是一个点，不能表达“这是一扇门”。

---

## 13. 单向通道怎么表达

例如台阶只能从 R_high 下到 R_low，不能反向上去：

```yaml
connectors:
  - id: C_step_down
    from: R_high
    to: R_low
    mode: one_way
    portal:
      start: {x: 4.2, y: 1.0}
      end:   {x: 4.2, y: 1.8}
    cost: 5.0
```

这样拓扑搜索只会生成：

```text
R_high -> R_low
```

不会生成：

```text
R_low -> R_high
```

如果你想让某条路只有紧急逃跑才走，可以先把 `cost` 设得高一些。

后续更高级的做法是接入决策树模式，例如 normal / escape 两套 cost，但当前版本还没有动态切换 connector cost。

---

## 14. fallback 逻辑

当拓扑规划失败时，如果：

```yaml
fallback_to_inner_planner: true
```

planner 会退回普通 InnerPlanner，也就是直接：

```text
start -> goal
```

常见 fallback 原因：

- `topology_yaml` 路径错误；
- YAML 格式错误；
- 起点不在任何 region 内；
- 终点不在任何 region 内；
- start region 和 goal region 之间没有拓扑连通；
- 某个 portal segment 的 InnerPlanner 失败；
- region constraint 太严格，所有候选路径都被 reject。

调试早期建议：

```yaml
fallback_to_inner_planner: true
allow_nearest_region_fallback: false
enforce_region_constraint: true
```

这样既不会直接动不了，又能暴露 topology.yaml 哪里画得不合理。

---

## 15. 日志检查

启动 Nav2 后，在 RViz 里正常点 `Nav2 Goal`。

如果拓扑生效，`planner_server` 日志里应该能看到类似：

```text
Topology route: R1 -> R2 -> R3
```

如果没有看到，优先检查：

```bash
ros2 param get /planner_server planner_plugins
```

应该能看到：

```text
[TopoPlanner]
```

也可以检查 topology yaml 路径：

```bash
ros2 param get /planner_server TopoPlanner.topology_yaml
```

---

## 16. 常见问题

### 16.1 planner_server 报 `GridBased is not a valid planner`

说明 BT 或参数里还在请求旧 planner id，例如：

```text
GridBased
```

但你的 planner_server 里只加载了：

```text
TopoPlanner
```

解决方法：

- 把 BT XML 里的 planner id 改成 `TopoPlanner`；
- 或者 Nav2 参数里保留对应名字；
- 或者确保默认 planner id 和 `planner_plugins` 一致。

### 16.2 起点/终点不在 region 里

表现：拓扑规划失败，然后 fallback。

解决方法：

- 检查 `topology.yaml` 的 region polygon 有没有覆盖实际可行驶区域；
- 检查地图坐标系是否一致；
- 先不要开 `allow_nearest_region_fallback`，否则容易把地图画漏的问题掩盖掉。

### 16.3 明明有 connector，但规划说不连通

检查：

- `from` / `to` 的 region id 有没有拼错；
- `mode: one_way` 方向是不是反了；
- connector 引用的 region 是否真的存在；
- YAML 缩进是否正确。

### 16.4 路径穿出了指定 region

如果开启：

```yaml
enforce_region_constraint: true
```

候选 segment 会被 reject。

解决方法：

- 增大 `region_constraint_tolerance`；
- 把 region polygon 画得更贴合实际可走区域；
- 检查 portal 是否放在两个 region 的合理交界处；
- 临时关闭 `enforce_region_constraint`，确认是不是约束导致失败。

### 16.5 portal 采样太慢

可以降低：

```yaml
portal_sample_count: 3
portal_refine_sample_count: 3
```

或者关闭细采样：

```yaml
portal_adaptive_sampling: false
```

### 16.6 portal 选点不理想

可以尝试：

- 增大 `portal_sample_count`；
- 检查 portal 线段是否覆盖了真正可通过的门宽；
- 不要把 portal 端点放得太贴障碍物；
- 确保 costmap 上 portal 附近确实可通行。

---

## 17. 调试建议

推荐按这个顺序调：

```text
1. 先关 region constraint，只验证 topology route 能不能跑通
2. 再打开 region constraint，看有没有 segment 被 reject
3. 再调整 portal_sample_count 和 portal_refine_sample_count
4. 最后再调 connector cost，让逻辑路径符合战术偏好
```

对应参数：

```yaml
enforce_region_constraint: false
portal_sample_count: 5
portal_adaptive_sampling: true
portal_refine_sample_count: 5
```

确认基本能跑后，再改回：

```yaml
enforce_region_constraint: true
```

---

## 18. 设计取舍

这个包采用的是“弱约束拓扑规划”，不是“强约束 costmap mask”。

优点：

- 对 Nav2 原架构侵入小；
- 能继续复用 Navfn/Smac 等成熟全局规划器；
- 可以直接接 RViz Nav2 Goal；
- YAML 可读性比较好；
- portal 比固定 waypoint 更接近真实门。

缺点：

- 不是严格数学意义上的区域内搜索；
- region constraint 是事后检查，不是搜索空间裁剪；
- topology.yaml 需要人工标注；
- 动态障碍主要还是依赖 local planner / costmap 更新。

---

## 19. 后续可以继续增强的方向

可以继续做的增强：

- 增加 topology map 可视化编辑器；
- 发布 Marker，把 regions/connectors/portal 显示到 RViz；
- 支持不同模式下 connector cost 动态变化，例如 normal / escape / defend；
- 把 region constraint 从“结果检查”升级为“costmap mask 约束”；
- 给特殊 connector 加类型，例如 `step_down`、`tunnel`、`bumpy_area`；
- 和行为树/决策树联动，根据战术状态切换拓扑代价。

---

## 20. 一句话总结

这个包做的事情是：

> **在 Nav2 原有全局规划器前面加一层人工拓扑地图，让机器人先选对区域路线，再用原来的 A*/Smac 算每一段真实路径。**

它的重点不是替代 Nav2，而是让 Nav2 更懂 RM 场地里的“区域、门、单向通道和推荐路线”。
