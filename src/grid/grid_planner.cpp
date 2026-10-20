
#include "topology_global_planner/grid/grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <utility>
#include <vector>
#include <limits>

#include "map_server/cost_values.hpp"

namespace topology_global_planner::grid
{

GridPlanner::GridPlanner(map_server::Costmap2D * costmap,
  const std::string & global_frame,
  rclcpp::Clock::SharedPtr clock,
  Options options)
    : costmap_(costmap), global_frame_(global_frame), clock_(std::move(clock)), options_(options)
{
  if (!costmap_ || !clock_) {
    throw std::invalid_argument("GridPlanner: null costmap or clock");
  }
  if (global_frame_.empty() || !std::isfinite(options_.tolerance) || options_.tolerance < 0.0) {
    throw std::invalid_argument("GridPlanner: invalid configuration");
  }
}

nav_msgs::msg::Path GridPlanner::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  GoalPolicy goal_policy,
  std::function<bool()> cancel_checker)
{
  if (!cancel_checker) {
    cancel_checker = []() {
      return false;
    };
  }
  if (cancel_checker()) {
    throw std::runtime_error("Planning cancelled");
  }
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw std::invalid_argument("GridPlanner: frame mismatch");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  // 复制地图快照
  unsigned int nx, ny;
  double resolution, ox, oy;
  std::vector<unsigned char> map_data;

  {
    std::unique_lock<map_server::Costmap2D::mutex_t> lock(*costmap_->getMutex());

    nx = costmap_->getSizeInCellsX();
    ny = costmap_->getSizeInCellsY();
    resolution = costmap_->getResolution();
    ox = costmap_->getOriginX();
    oy = costmap_->getOriginY();

    if (nx < 3 || ny < 3 || resolution <= 0.0) {
      throw std::runtime_error("GridPlanner: invalid costmap");
    }

    const auto * data = costmap_->getCharMap();
    map_data.assign(data, data + static_cast<size_t>(nx) * ny);
  }

  // 世界坐标转为搜索栅格
  // 取整方式与原版栅格规划器一致
  auto to_cell = [&](double wx, double wy, int & mx, int & my) {
    if (!std::isfinite(wx) || !std::isfinite(wy)) {
      return false;
    }

    const double x = (wx - ox) / resolution;
    const double y = (wy - oy) / resolution;

    if (x < 0.0 || y < 0.0) {
      return false;
    }

    const double rx = std::round(x);
    const double ry = std::round(y);

    if (rx >= nx || ry >= ny) {
      return false;
    }

    mx = static_cast<int>(rx);
    my = static_cast<int>(ry);
    return true;
  };

  int sx, sy, gx, gy;
  if (!to_cell(start.pose.position.x, start.pose.position.y, sx, sy)) {
    throw std::runtime_error("GridPlanner: start outside map");
  }
  if (!to_cell(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    throw std::runtime_error("GridPlanner: goal outside map");
  }

  // 搜索按四舍五入取格
  // 实际占用按向下取整判断
  // 终点还要检查实际所在格是否可通行
  const auto traversable = [&](double wx, double wy) {
    const double mx = std::floor((wx - ox) / resolution), my = std::floor((wy - oy) / resolution);
    if (!std::isfinite(mx) || !std::isfinite(my) || mx < 0.0 || my < 0.0 || mx >= nx || my >= ny) {
      return false;
    }
    const auto cost = map_data[static_cast<size_t>(my) * nx + static_cast<size_t>(mx)];
    return cost < map_server::INSCRIBED_INFLATED_OBSTACLE ||
           (options_.allow_unknown && cost == map_server::NO_INFORMATION);
  };
  const double tolerance = goal_policy == GoalPolicy::EXACT ? 0.0 : options_.tolerance;
  const auto goal_error = [&](const std::string & reason) {
    std::ostringstream message;
    message
      << "GridPlanner: " << reason << " at (" << goal.pose.position.x << ", "
      << goal.pose.position.y << "), cell=(" << gx << ", " << gy
      << "), cost=" << static_cast<unsigned>(map_data[static_cast<size_t>(gy) * nx + gx])
      << ", physical_cost="
      << static_cast<unsigned>(
           map_data[static_cast<size_t>(std::floor((goal.pose.position.y - oy) / resolution)) * nx +
                    static_cast<size_t>(std::floor((goal.pose.position.x - ox) / resolution))])
      << ", tolerance=" << tolerance;
    if (goal_policy == GoalPolicy::EXACT) {
      message << " (fixed connector endpoint)";
    }
    return std::runtime_error(message.str());
  };
  if (tolerance == 0.0 && !traversable(goal.pose.position.x, goal.pose.position.y)) {
    throw goal_error("goal occupied");
  }

  // 起终点重合时直接返回当前位姿
  if (start.pose.position.x == goal.pose.position.x &&
      start.pose.position.y == goal.pose.position.y)
  {
    auto pose = start;
    pose.header = path.header;

    if (!options_.use_final_approach_orientation) {
      pose.pose.orientation = goal.pose.orientation;
    }

    path.poses.push_back(pose);
    return path;
  }

  // 创建或调整势场尺寸
  if (!navfn_) {
    navfn_ = std::make_unique<NavFn>(nx, ny);
  } else if (navfn_->nx != static_cast<int>(nx) || navfn_->ny != static_cast<int>(ny)) {
    navfn_->setNavArr(nx, ny);
  }

  // 只清除快照中的起点格
  // 不修改原始代价地图
  map_data[static_cast<size_t>(sy) * nx + sx] = map_server::FREE_SPACE;

  navfn_->setCostmap(map_data.data(), true, options_.allow_unknown);

  int map_start[2] = {sx, sy};
  int map_goal[2] = {gx, gy};

  // 势场从机器人位置向外传播
  // 搜索起终点按原版的反向约定设置
  navfn_->setStart(map_goal);
  navfn_->setGoal(map_start);

  // 计算势场
  if (options_.use_astar) {
    navfn_->calcNavFnAstar(cancel_checker);
  } else {
    navfn_->calcNavFnDijkstra(cancel_checker, true);
  }

  // 在允许的容差内选择可达终点
  auto potential = [&](double wx, double wy) {
    int mx, my;
    if (!to_cell(wx, wy, mx, my)) {
      return static_cast<double>(POT_HIGH);
    }
    return static_cast<double>(navfn_->potarr[static_cast<size_t>(my) * nx + mx]);
  };

  geometry_msgs::msg::Pose best_goal = goal.pose;
  bool found = traversable(goal.pose.position.x, goal.pose.position.y) &&
               potential(goal.pose.position.x, goal.pose.position.y) < POT_HIGH;

  if (!found && tolerance > 0.0) {
    double best_dist = std::numeric_limits<double>::max();

    for (double y = goal.pose.position.y - tolerance; y <= goal.pose.position.y + tolerance;
         y += resolution)
    {
      for (double x = goal.pose.position.x - tolerance; x <= goal.pose.position.x + tolerance;
           x += resolution)
      {
        if (!traversable(x, y) || potential(x, y) >= POT_HIGH) {
          continue;
        }

        const double dx = x - goal.pose.position.x;
        const double dy = y - goal.pose.position.y;
        const double dist = dx * dx + dy * dy;

        if (dist <= tolerance * tolerance + 1e-12 && dist < best_dist) {
          best_dist = dist;
          best_goal.position.x = x;
          best_goal.position.y = y;
          found = true;
        }
      }
    }
  }

  if (!found) {
    throw goal_error("no reachable goal");
  }

  // 从势场提取路径
  int bx, by;
  if (!to_cell(best_goal.position.x, best_goal.position.y, bx, by)) {
    throw std::runtime_error("GridPlanner: invalid selected goal");
  }

  int best_cell[2] = {bx, by};
  navfn_->setStart(best_cell);

  const int max_cycles = 4 * static_cast<int>(std::max(nx, ny));

  if (navfn_->calcPath(max_cycles) <= 0) {
    throw std::runtime_error("GridPlanner: path extraction failed");
  }

  const float * px = navfn_->getPathX();
  const float * py = navfn_->getPathY();

  // 将回溯结果反转为起点到终点
  for (int i = navfn_->getPathLen() - 1; i >= 0; --i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = ox + px[i] * resolution;
    pose.pose.position.y = oy + py[i] * resolution;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  // 衔接选定的终点
  auto dist2 = [](const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) {
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    return dx * dx + dy * dy;
  };

  bool replace_last = false;
  if (path.poses.size() >= 2) {
    const auto & prev = path.poses[path.poses.size() - 2].pose;
    const auto & last = path.poses.back().pose;

    replace_last = dist2(last, prev) > dist2(best_goal, prev) || dist2(last, best_goal) < 1e-6;
  }

  if (replace_last) {
    path.poses.back().pose = best_goal;
  } else {
    geometry_msgs::msg::PoseStamped final_pose;
    final_pose.header = path.header;
    final_pose.pose = best_goal;
    path.poses.push_back(final_pose);
  }

  // 按配置确定终点朝向
  if (options_.use_final_approach_orientation) {
    if (path.poses.size() == 1) {
      path.poses.back().pose.orientation = start.pose.orientation;
    } else {
      const auto & last = path.poses.back().pose.position;
      auto prev = path.poses[path.poses.size() - 2].pose.position;

      if (std::abs(last.x - prev.x) < 1e-4 && std::abs(last.y - prev.y) < 1e-4 &&
          path.poses.size() > 2)
      {
        prev = path.poses[path.poses.size() - 3].pose.position;
      }

      const double yaw = std::atan2(last.y - prev.y, last.x - prev.x);
      auto & q = path.poses.back().pose.orientation;
      q.x = 0.0;
      q.y = 0.0;
      q.z = std::sin(yaw / 2.0);
      q.w = std::cos(yaw / 2.0);
    }
  }

  if (cancel_checker()) {
    throw std::runtime_error("Planning cancelled");
  }

  return path;
}

}
