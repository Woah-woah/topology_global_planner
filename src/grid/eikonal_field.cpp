#include "topology_global_planner/grid/eikonal_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace topology_global_planner::grid
{
namespace
{
constexpr float INF = std::numeric_limits<float>::infinity();

float upwind(double a, double b, double step)
{
  if (a > b) {std::swap(a, b);}
  if (!std::isfinite(a)) {return INF;}
  if (!std::isfinite(b) || b - a >= step) {return a + step;}
  // 单调一阶 Godunov 上风格式，选择满足 T >= max(a,b) 的二次方程根。
  return 0.5 * (a + b + std::sqrt(std::max(0.0, 2 * step * step - (a - b) * (a - b))));
}
}  // namespace

EikonalField propagateEikonal(int width, int height, double resolution,
  const std::vector<float> & costs, const std::vector<uint8_t> & traversable,
  int seed, const std::function<void()> & check_cancel,
  const std::vector<std::pair<int, float>> & boundary)
{
  const size_t count = static_cast<size_t>(width) * height;
  if (width < 1 || height < 1 || count > 100000000 || costs.size() != count ||
    traversable.size() != count || seed < 0 || seed >= static_cast<int>(count) ||
    !traversable[seed] ||
    !std::isfinite(resolution) || resolution <= 0)
  {throw std::invalid_argument("Invalid Eikonal input");}
  for (size_t i = 0; i < count; ++i) {
    if ((i & 1023) == 0) {check_cancel();}
    if (!std::isfinite(costs[i]) || costs[i] <= 0)
    {throw std::invalid_argument("Eikonal costs must be finite and positive");}
  }
  EikonalField result;
  result.values.assign(count, INF);
  std::vector<uint8_t> accepted(count, 0), fixed(count, 0);
  struct Entry
  {
    double priority;
    float value;
    int index;
    bool operator>(const Entry & other) const
    {
      if (priority != other.priority) {return priority > other.priority;}
      return index > other.index;
    }
  };
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  const auto neighbors = [&](int i, auto visit) {
      const int x = i % width, y = i / width;
      if (x > 0) {visit(i - 1);}
      if (x + 1 < width) {visit(i + 1);}
      if (y > 0) {visit(i - width);}
      if (y + 1 < height) {visit(i + width);}
    };
  const auto update = [&](int i, bool causal) {
      const int x = i % width, y = i / width;
      const auto at = [&](int j) {
          return traversable[j] && (!causal || accepted[j]) ? result.values[j] : INF;
        };
      const float a = std::min(x > 0 ? at(i - 1) : INF, x + 1 < width ? at(i + 1) : INF);
      const float b = std::min(y > 0 ? at(i - width) : INF, y + 1 < height ? at(i + width) : INF);
      return upwind(a, b, resolution * costs[i]);
    };
  const auto initialize = [&](int i, float value) {
      if (i < 0 || i >= static_cast<int>(count) || !traversable[i] ||
        !std::isfinite(value) || value < 0) {throw std::invalid_argument("Invalid Eikonal boundary");}
      if (fixed[i] && value >= result.values[i]) {return;}
      result.values[i] = value; fixed[i] = 1;
      queue.push({value, value, i});
    };
  if (boundary.empty()) {initialize(seed, 0);}
  else {for (const auto & entry : boundary) {initialize(entry.first, entry.second);}}
  while (!queue.empty()) {
    if ((result.expansions++ & 255) == 0) {check_cancel();}
    const auto entry = queue.top(); queue.pop();
    const int i = entry.index;
    if (entry.value != result.values[i] || accepted[i]) {continue;}
    accepted[i] = 1;
    neighbors(i, [&](int next) {
      if (!traversable[next] || fixed[next] || accepted[next]) {return;}
      const float value = update(next, true);
      if (value < result.values[next]) {
        result.values[next] = value;
        ++result.updates;
        queue.push({value, value, next});
      }
    });
  }
  // 完整性包含局部离散方程残差；不可达格保留 inf，禁止伪造可插值的自由势值。
  for (size_t i = 0; i < count; ++i) {
    if ((i & 1023) == 0) {check_cancel();}
    if (!traversable[i] || !std::isfinite(result.values[i])) {continue;}
    ++result.reachable;
    if (fixed[i]) {continue;}
    const double residual = std::abs(result.values[i] - update(i, false));
    result.max_residual = std::max(result.max_residual, residual);
    if (residual > 1e-4 * std::max(1.0f, result.values[i]))
    {throw std::runtime_error("Eikonal field did not converge");}
  }
  result.complete = true;
  check_cancel();
  return result;
}
}  // namespace topology_global_planner::grid
