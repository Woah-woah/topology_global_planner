#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>
#include <utility>

namespace topology_global_planner::grid
{
struct EikonalField
{
  std::vector<float> values;
  size_t expansions{0}, updates{0}, reachable{0};
  double max_residual{0};
  bool complete{false};
};

// |grad T| = cost。数值内核始终按到达势值因果排序，不混入 A* 启发项。
// A* 用于外层图搜索及 GoalPolicy；完整 FMM 使两种配置拥有相同可靠插值域。
// boundary 可提供精确亚栅格目标周围的局部 Dirichlet 边界值。
EikonalField propagateEikonal(int width, int height, double resolution,
  const std::vector<float> & costs, const std::vector<uint8_t> & traversable,
  int seed, const std::function<void()> & check_cancel,
  const std::vector<std::pair<int, float>> & boundary = {});
}  // namespace topology_global_planner::grid
