#include "topology_global_planner/grid/grid_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include "map_server/distance_field.hpp"
#include "topology_global_planner/grid/eikonal_field.hpp"

namespace topology_global_planner::grid
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr float INF = std::numeric_limits<float>::infinity();
struct Point {double x, y;};
double squared(double x) {return x * x;}
double distance(Point a, Point b) {return std::hypot(a.x - b.x, a.y - b.y);}
double pointSegmentSquared(Point p, Point a, Point b)
{
  const double norm = squared(b.x - a.x) + squared(b.y - a.y);
  const double t = norm > 0 ? std::clamp(((p.x - a.x) * (b.x - a.x) +
    (p.y - a.y) * (b.y - a.y)) / norm, 0.0, 1.0) : 0;
  return squared(p.x - a.x - t * (b.x - a.x)) + squared(p.y - a.y - t * (b.y - a.y));
}

// 线段与闭矩形的距离，精确覆盖擦角和短于采样间隔的障碍。
double rectangleDistanceSquared(Point a, Point b, double x, double y, double resolution)
{
  double first = 0, last = 1;
  bool intersects = true;
  for (const auto & axis : {std::array<double, 3>{a.x, b.x - a.x, x},
    std::array<double, 3>{a.y, b.y - a.y, y}})
  {
    if (std::abs(axis[1]) < 1e-15) {
      if (axis[0] < axis[2] || axis[0] > axis[2] + resolution) {intersects = false;}
    } else {
      double lo = (axis[2] - axis[0]) / axis[1];
      double hi = (axis[2] + resolution - axis[0]) / axis[1];
      if (lo > hi) {std::swap(lo, hi);}
      first = std::max(first, lo); last = std::min(last, hi);
    }
  }
  if (intersects && first <= last) {return 0;}
  const auto point_box = [&](Point p) {
      return squared(std::max({x - p.x, 0.0, p.x - x - resolution})) +
        squared(std::max({y - p.y, 0.0, p.y - y - resolution}));
    };
  double result = std::min(point_box(a), point_box(b));
  for (const auto corner : {Point{x, y}, Point{x + resolution, y},
    Point{x, y + resolution}, Point{x + resolution, y + resolution}})
  {result = std::min(result, pointSegmentSquared(corner, a, b));}
  return result;
}

class SnapshotGeometry
{
public:
  explicit SnapshotGeometry(const map_server::Costmap2D & map)
  : map_(map), field_(map.distanceField()), width(map.getSizeInCellsX()),
    height(map.getSizeInCellsY()), resolution(map.getResolution()) {}

  int cell(Point p) const
  {
    unsigned int x, y;
    return map_.worldToMap(p.x, p.y, x, y) ? static_cast<int>(y * width + x) : -1;
  }
  Point center(int i) const
  {
    Point p;
    map_.mapToWorld(i % width, i / width, p.x, p.y);
    return p;
  }
  bool pointAllowed(Point p) const
  {
    const int i = cell(p);
    if (i < 0 || map_.getCost(i) >= map_server::INSCRIBED_INFLATED_OBSTACLE ||
      field_->obstacles[i] >= map_server::LETHAL_OBSTACLE) {return false;}
    const double r = field_->robot_radius;
    return p.x - r >= map_.getOriginX() && p.y - r >= map_.getOriginY() &&
      p.x + r < map_.getOriginX() + width * resolution &&
      p.y + r < map_.getOriginY() + height * resolution;
  }
  bool edgeAllowed(Point a, Point b) const
  {
    if (!pointAllowed(a) || !pointAllowed(b)) {return false;}
    const double radius = field_->robot_radius;
    const int x0 = std::max(0, static_cast<int>(std::floor(
      (std::min(a.x, b.x) - radius - map_.getOriginX()) / resolution)));
    const int y0 = std::max(0, static_cast<int>(std::floor(
      (std::min(a.y, b.y) - radius - map_.getOriginY()) / resolution)));
    const int x1 = std::min(width - 1, static_cast<int>(std::floor(
      (std::max(a.x, b.x) + radius - map_.getOriginX()) / resolution)));
    const int y1 = std::min(height - 1, static_cast<int>(std::floor(
      (std::max(a.y, b.y) + radius - map_.getOriginY()) / resolution)));
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const int i = y * width + x;
        const auto raw = field_->obstacles[i];
        if (raw < map_server::LETHAL_OBSTACLE &&
          map_.getCost(i) < map_server::INSCRIBED_INFLATED_OBSTACLE) {continue;}
        const double d2 = rectangleDistanceSquared(a, b, map_.getOriginX() + x * resolution,
          map_.getOriginY() + y * resolution, resolution);
        // 车体半径只施加到未膨胀原始障碍；253 硬掩码只检查中心线。
        if (d2 <= 1e-20 || (raw >= map_server::LETHAL_OBSTACLE && d2 <= radius * radius))
        {return false;}
      }
    }
    return true;
  }
  bool pathAllowed(const nav_msgs::msg::Path & path, const std::function<void()> & check) const
  {
    if (path.poses.empty()) {return false;}
    Point previous{path.poses.front().pose.position.x, path.poses.front().pose.position.y};
    for (const auto & pose : path.poses) {
      check();
      Point p{pose.pose.position.x, pose.pose.position.y};
      if (!edgeAllowed(previous, p)) {return false;}
      previous = p;
    }
    return true;
  }
  const map_server::Costmap2D & map_;
  std::shared_ptr<const map_server::DistanceField> field_;
  int width, height;
  double resolution;
};

struct FieldSample {double value{0}; Point gradient{0, 0};};

// 与旧 NavFn 相同的梯度节点插值思路；屏蔽障碍/不可达节点，禁止 inf 参与运算。
bool samplePotential(const SnapshotGeometry & geometry, const std::vector<float> & values,
  Point point, FieldSample & result)
{
  if (geometry.cell(point) < 0) {return false;}
  const double ux = (point.x - geometry.map_.getOriginX()) / geometry.resolution - 0.5;
  const double uy = (point.y - geometry.map_.getOriginY()) / geometry.resolution - 0.5;
  const double cx = std::clamp(ux, 0.0, static_cast<double>(geometry.width - 1));
  const double cy = std::clamp(uy, 0.0, static_cast<double>(geometry.height - 1));
  const int x0 = std::floor(cx), y0 = std::floor(cy);
  double weight_sum = 0;
  result = {};
  const auto at = [&](int x, int y) -> double {
      if (x < 0 || y < 0 || x >= geometry.width || y >= geometry.height) {return INF;}
      return values[y * geometry.width + x];
    };
  for (int dy = 0; dy <= 1; ++dy) {
    for (int dx = 0; dx <= 1; ++dx) {
      const int x = std::min(x0 + dx, geometry.width - 1);
      const int y = std::min(y0 + dy, geometry.height - 1);
      const double value = at(x, y);
      const double weight = (dx ? cx - x0 : 1 - cx + x0) * (dy ? cy - y0 : 1 - cy + y0);
      if (!std::isfinite(value) || weight <= 0) {continue;}
      const auto derivative = [&](double low, double high) {
          if (std::isfinite(low) && std::isfinite(high)) {return 0.5 * (high - low);}
          if (std::isfinite(high)) {return high - value;}
          if (std::isfinite(low)) {return value - low;}
          return 0.0;
        };
      Point gradient{derivative(at(x - 1, y), at(x + 1, y)),
        derivative(at(x, y - 1), at(x, y + 1))};
      const double norm = std::hypot(gradient.x, gradient.y);
      if (norm > 1e-12) {gradient.x /= norm; gradient.y /= norm;}
      result.value += weight * value;
      result.gradient.x += weight * gradient.x;
      result.gradient.y += weight * gradient.y;
      weight_sum += weight;
    }
  }
  if (weight_sum <= 1e-12) {return false;}
  result.value /= weight_sum;
  result.gradient.x /= weight_sum; result.gradient.y /= weight_sum;
  return std::isfinite(result.value) && std::isfinite(result.gradient.x) &&
    std::isfinite(result.gradient.y);
}

nav_msgs::msg::Path extractGradient(const SnapshotGeometry & geometry,
  const std::vector<float> & values, Point start, Point goal, const std_msgs::msg::Header & header,
  const std::function<void()> & check, std::string & failure, size_t & projected)
{
  nav_msgs::msg::Path path;
  path.header = header;
  const auto append = [&](Point p) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header; pose.pose.orientation.w = 1;
      pose.pose.position.x = p.x; pose.pose.position.y = p.y;
      path.poses.push_back(pose);
    };
  append(start);
  Point current = start;
  const size_t limit = values.size() * 8;
  for (size_t iteration = 0; iteration < limit; ++iteration) {
    check();
    if (distance(current, goal) <= 0.6 * geometry.resolution && geometry.edgeAllowed(current, goal)) {
      if (distance(current, goal) > 1e-10) {append(goal);}
      return path;
    }
    FieldSample sample;
    if (!samplePotential(geometry, values, current, sample)) {failure = "当前位置周围没有有效的势场插值数据"; break;}
    const double norm = std::hypot(sample.gradient.x, sample.gradient.y);
    if (norm < 1e-10) {failure = "当前位置的导航梯度为零，无法确定前进方向"; break;}
    Point direction{-sample.gradient.x / norm, -sample.gradient.y / norm};
    // 中点积分减小特征线转弯处的一阶步进误差；仍由同一势场提供方向。
    const Point midpoint{current.x + 0.25 * geometry.resolution * direction.x,
      current.y + 0.25 * geometry.resolution * direction.y};
    FieldSample middle;
    if (samplePotential(geometry, values, midpoint, middle)) {
      const double middle_norm = std::hypot(middle.gradient.x, middle.gradient.y);
      if (middle_norm > 1e-10) {direction = {-middle.gradient.x / middle_norm, -middle.gradient.y / middle_norm};}
    }
    bool advanced = false;
    // 先沿连续梯度推进；边界附近在局部可行方向上投影，始终要求势值下降。
    for (int reduction = 0; reduction < 5 && !advanced; ++reduction) {
      const double step = std::ldexp(0.5 * geometry.resolution, -reduction);
      for (double angle : {0.0, 0.13, -0.13, 0.26, -0.26, 0.52, -0.52, 0.79, -0.79, 1.05, -1.05, 1.57, -1.57}) {
        Point candidate{current.x + step * (std::cos(angle) * direction.x - std::sin(angle) * direction.y),
          current.y + step * (std::sin(angle) * direction.x + std::cos(angle) * direction.y)};
        FieldSample next;
        if (!geometry.edgeAllowed(current, candidate) ||
          !samplePotential(geometry, values, candidate, next) ||
          next.value >= sample.value - 1e-5 * step) {continue;}
        if (angle != 0) {++projected;}
        current = candidate; append(current); advanced = true;
        break;
      }
    }
    if (!advanced) {failure = "找不到既降低势值又不碰撞的前进方向"; break;}
    if (path.poses.size() > 16) {
      const auto & old = path.poses[path.poses.size() - 16].pose.position;
      if (distance(current, {old.x, old.y}) < 0.02 * geometry.resolution)
      {failure = "路径提取在局部停滞，无法继续前进"; break;}
    }
  }
  if (failure.empty()) {failure = "连续路径提取超过允许步数";}
  return nav_msgs::msg::Path();
}
}  // namespace

GridPlanner::GridPlanner(map_server::Costmap2D * costmap, const std::string & global_frame,
  rclcpp::Clock::SharedPtr clock, Options options)
: costmap_(costmap), global_frame_(global_frame), clock_(std::move(clock)), options_(options)
{
  if (!costmap_ || !clock_ || global_frame_.empty() ||
    !std::isfinite(options_.tolerance) || options_.tolerance < 0)
  {throw std::invalid_argument("栅格规划参数无效");}
  if (options_.allow_unknown)
  {throw std::invalid_argument("不允许穿越未知区域：后端碰撞地图将未知区域视为不可通行");}
}

nav_msgs::msg::Path GridPlanner::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, GoalPolicy policy,
  std::function<bool()> cancelled, PathValidator topology_valid)
{
  const auto began = Clock::now();
  const auto ms = [](Clock::time_point time) {
      return std::chrono::duration<double, std::milli>(Clock::now() - time).count();
    };
  diagnostics_ = PlanningDiagnostics();
  const auto check = [&]() {
      if (cancelled && cancelled()) {throw std::runtime_error("规划已取消、超时或使用的地图快照已过期");}
    };
  check();
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
  {throw std::invalid_argument("栅格规划的起点或目标点坐标系与地图不一致");}
  const map_server::Costmap2D map(*costmap_);
  const auto field = map.distanceField();
  if (!field || !field->matches(map)) {throw std::runtime_error("缺少与当前地图配套的浮点距离场");}
  const SnapshotGeometry geometry(map);
  const int width = geometry.width, height = geometry.height;
  const size_t count = static_cast<size_t>(width) * height;
  if (width < 3 || height < 3 || field->distances.size() != count || field->obstacles.size() != count)
  {throw std::runtime_error("当前地图快照的尺寸或距离场数据无效");}
  Point begin{start.pose.position.x, start.pose.position.y};
  Point end{goal.pose.position.x, goal.pose.position.y};
  const int source = geometry.cell(begin), target = geometry.cell(end);
  if (source < 0 || target < 0) {throw std::runtime_error("起点或目标点超出地图范围");}
  std::vector<float> costs(count), potential(count, INF);
  std::vector<uint8_t> walkable(count, 0), closed(count, 0);
  for (size_t i = 0; i < count; ++i) {
    if ((i & 1023) == 0) {check();}
    const float d = field->distances[i];
    if (!std::isfinite(d) || d < 0) {throw std::runtime_error("距离场含无效或负数距离，拒绝用于搜索");}
    costs[i] = 1 + 4 * map_server::distancePenalty(d, field->robot_radius,
      field->inflation_radius, field->cost_scaling_factor).cost;
    const auto p = geometry.center(i);
    walkable[i] = geometry.pointAllowed(p) && geometry.edgeAllowed(p, p);
  }
  if (!walkable[source] || !geometry.edgeAllowed(begin, geometry.center(source)))
  {throw std::runtime_error("规划起点被占用，或起点周围没有足够空间容纳车体");}
  const double tolerance = policy == GoalPolicy::EXACT ? 0 : options_.tolerance;
  if (tolerance == 0 && (!walkable[target] || !geometry.edgeAllowed(geometry.center(target), end)))
  {throw std::runtime_error("精确目标点被占用，或目标周围没有足够空间容纳车体");}
  std_msgs::msg::Header header;
  header.frame_id = global_frame_; header.stamp = clock_->now();
  const auto make_path = [&](const std::vector<Point> & points) {
      nav_msgs::msg::Path path; path.header = header;
      for (Point p : points) {
        if (!path.poses.empty()) {
          const auto & last = path.poses.back().pose.position;
          if (distance(p, {last.x, last.y}) < 1e-10) {continue;}
        }
        auto pose = start; pose.header = header;
        pose.pose.position.x = p.x; pose.pose.position.y = p.y;
        path.poses.push_back(pose);
      }
      return path;
    };
  const auto valid = [&](const nav_msgs::msg::Path & path) {
      check();
      return geometry.pathAllowed(path, check) && (!topology_valid || topology_valid(path));
    };
  if (distance(begin, end) < 1e-12) {
    auto path = make_path({begin});
    if (!valid(path)) {throw std::runtime_error("起终点重合，但该位置不满足拓扑或车体净空要求");}
    if (!options_.use_final_approach_orientation) {path.poses.back().pose.orientation = goal.pose.orientation;}
    diagnostics_.extraction = "coincident";
    return path;
  }
  struct Entry
  {
    float priority, value; int index;
    bool operator>(const Entry & other) const {return priority > other.priority;}
  };
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const auto heuristic = [&](int i) -> float {
      return options_.use_astar ? geometry.resolution *
        std::hypot(i % width - target % width, i / width - target / width) : 0;
    };
  // 八邻域 Dijkstra/A* 仅确定可达目标；路径统一由连续 Eikonal 势场提取。
  potential[source] = 0; open.push({heuristic(source), 0, source});
  size_t expanded = 0;
  while (!open.empty()) {
    if ((expanded++ & 255) == 0) {check();}
    const auto current = open.top(); open.pop();
    const int i = current.index;
    if (closed[i] || current.value != potential[i]) {continue;}
    closed[i] = 1;
    if (i == target && walkable[target]) {break;}
    const int x = i % width, y = i / width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((!dx && !dy) || x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) {continue;}
        const int next = i + dy * width + dx;
        if (!walkable[next] || closed[next] ||
          (dx && dy && (!walkable[i + dx] || !walkable[i + dy * width]))) {continue;}
        const float candidate = potential[i] + geometry.resolution * (dx && dy ? std::sqrt(2.0) : 1.0) *
          0.5f * (costs[i] + costs[next]);
        if (candidate < potential[next]) {
          potential[next] = candidate;
          open.push({candidate + heuristic(next), candidate, next});
        }
      }
    }
  }
  int selected = walkable[target] && closed[target] &&
    geometry.edgeAllowed(geometry.center(target), end) ? target : -1;
  if (selected < 0 && tolerance > 0) {
    double best_distance = INF;
    for (size_t i = 0; i < count; ++i) {
      if ((i & 1023) == 0) {check();}
      if (!walkable[i] || !closed[i]) {continue;}
      const auto p = geometry.center(i);
      const double error = distance(p, {goal.pose.position.x, goal.pose.position.y});
      if (error <= tolerance + 1e-12 && error < best_distance) {
        best_distance = error; selected = i; end = p;
      }
    }
  }
  if (selected < 0) {throw std::runtime_error("未找到满足要求且可到达的目标点");}
  diagnostics_.graph_ms = ms(began);
  const auto field_began = Clock::now();
  // 精确目标作为点源：以可见的周围栅格中心初始化局部边界，避免末端绕向格心。
  std::vector<std::pair<int, float>> boundary;
  const int x0 = std::floor((end.x - map.getOriginX()) / geometry.resolution - 0.5);
  const int y0 = std::floor((end.y - map.getOriginY()) / geometry.resolution - 0.5);
  for (int dy = 0; dy <= 1; ++dy) {
    for (int dx = 0; dx <= 1; ++dx) {
      const int x = x0 + dx, y = y0 + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {continue;}
      const int i = y * width + x;
      if (walkable[i] && geometry.edgeAllowed(geometry.center(i), end)) {
        boundary.emplace_back(i, distance(geometry.center(i), end) * 0.5 * (costs[i] + costs[selected]));
      }
    }
  }
  auto navigation = propagateEikonal(width, height, geometry.resolution, costs, walkable,
    selected, check, boundary);
  diagnostics_.field_ms = ms(field_began);
  diagnostics_.field_complete = navigation.complete;
  diagnostics_.field_expansions = navigation.expansions;
  diagnostics_.field_reachable = navigation.reachable;
  diagnostics_.max_residual = navigation.max_residual;
  if (!std::isfinite(navigation.values[source]))
  {throw std::runtime_error("连续势场无法连接搜索已判可达的起点，本次规划失败");}
  const auto trace_began = Clock::now();
  std::string failure;
  auto continuous = extractGradient(geometry, navigation.values, begin, end, header,
    check, failure, diagnostics_.projected_steps);
  if (!continuous.poses.empty() && !valid(continuous)) {
    failure = "连续势场路径未通过碰撞检查或区域/通道约束检查";
    continuous.poses.clear();
  }
  diagnostics_.extraction_ms = ms(trace_began);
  if (options_.collect_diagnostics) {
    diagnostics_.eikonal_path = continuous;
    diagnostics_.eikonal_potential = std::move(navigation.values);
  }
  diagnostics_.extraction = "eikonal";
  diagnostics_.failure_reason = failure;
  // 连续提取失败必须向上层报告，禁止切换为父节点或图势值折线路径。
  if (continuous.poses.empty()) {
    if (diagnostics_.failure_reason.empty()) {
      diagnostics_.failure_reason = "连续势场提取没有生成路径";
    }
    throw std::runtime_error("连续势场路径提取失败：" + diagnostics_.failure_reason);
  }
  auto path = std::move(continuous);
  if (!valid(path)) {throw std::runtime_error("规划输出未满足车体净空或拓扑约束");}
  // 路径方向由实际亚栅格切线确定，精确端点位置及 GoalPolicy 保持不变。
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    auto & q = path.poses[i].pose.orientation;
    q.x = 0; q.y = 0; q.z = std::sin(yaw / 2); q.w = std::cos(yaw / 2);
  }
  path.poses.front().pose.orientation = start.pose.orientation;
  if (!options_.use_final_approach_orientation) {path.poses.back().pose.orientation = goal.pose.orientation;}
  diagnostics_.total_ms = ms(began);
  check();
  return path;
}
}  // namespace topology_global_planner::grid
