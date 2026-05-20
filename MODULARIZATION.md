# topology_global_planner 模块拆分说明

这版采用低风险拆分：不改变 `TopologyGlobalPlanner` 类的外部接口、不改参数名、不改 YAML 格式，只把原来 1200+ 行的 `src/topology_global_planner.cpp` 按职责拆成多个 `.cpp`。

## 文件职责

- `src/planner_lifecycle.cpp`
  - Nav2 插件生命周期：构造、configure、cleanup、activate、deactivate
  - `createPlan()` 总入口
  - pluginlib 导出宏

- `src/topology_io.cpp`
  - `loadTopologyYaml()`：读取 regions/connectors/portal/waypoint/cost/mode
  - `buildGraph()`：把 connector 转成有向图/双向图

- `src/geometry_utils.cpp`
  - region 判断：`findRegion()`、`pointInPolygon()`、边界容差判断
  - 点到线段距离、质心、欧氏距离
  - region constraint 所需的 path-in-region 判断

- `src/topology_search.cpp`
  - `searchTopology()`：Dijkstra 搜索 region 逻辑路径
  - `buildTopologyWaypoints()`：旧 weak topology 模式兼容辅助函数

- `src/portal_optimizer.cpp`
  - portal/waypoint 采样
  - 粗采样 + 自适应细采样
  - DP 选择总 A* 路径长度最短的 portal 点组合

- `src/path_planning.cpp`
  - 调用 inner planner
  - 分段路径拼接
  - fallback
  - frame normalize / TF transform
  - 中间点 yaw 设置

## 为什么先这样拆

这是最稳的第一步，因为没有改变类成员、参数和算法行为。你现在的 planner 还在迭代 portal、region mask、约束逻辑，如果一上来把类也拆碎，调 bug 会更难。等逻辑稳定后，再进一步把 `TopologyMap`、`PortalOptimizer`、`RegionConstraintChecker` 独立成类。

## 后续更彻底的拆法

下一步可以把 `include/topology_global_planner/topology_global_planner.hpp` 里的数据结构和功能类拆成：

- `types.hpp`：Point2D / Region / Connector / TopologySearchResult
- `topology_map.hpp/cpp`：YAML 读取、region 查询、graph 构建
- `topology_router.hpp/cpp`：Dijkstra
- `portal_optimizer.hpp/cpp`：采样 + DP
- `path_stitcher.hpp/cpp`：路径拼接、yaw、长度计算
- `topology_global_planner.hpp/cpp`：只保留 Nav2 plugin 壳子

但是这个第二步会改变类之间的数据流，建议等现有功能验证稳定后再做。
