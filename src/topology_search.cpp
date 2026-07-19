#include "topology_global_planner/topology_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

namespace topology_global_planner
{

TopologySearchResult TopologyGlobalPlanner::searchTopology(
  const std::string & start_region, 
  const std::string & goal_region,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal
) const {
  TopologySearchResult result;

  struct QueueItem
  {
    double cost;
    std::string region;
    bool operator>(const QueueItem & other) const {    // operator后的>表示重载>运算符
      return cost > other.cost;                        // other是和当前对象比较的另一个对象 const指不改变当前对象的值
    }
  };

  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open; // 优先队列: std::priority_queue<元素类型, 底层容器, 比较规则> 变量名;
  std::unordered_map<std::string, double> dist;                                         // std::greater<QueueItem>小根堆 使用“大于”比较 open.top()弹出的是最小值
  std::unordered_map<std::string, std::string> parent_region;
  std::unordered_map<std::string, int> parent_connector;

  for (const auto & r : regions_) {
    dist[r.id] = std::numeric_limits<double>::infinity();       //e.g. d["R3"] = 10.0 到每个region的cost
  }
  dist[start_region] = 0.0;
  open.push(QueueItem{0.0, start_region});

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();                             // 弹出堆中最小对象

    if (current.cost > dist[current.region]) {
      continue;
    }
    if (current.region == goal_region) {
      break;
    }

    const auto it = graph_.find(current.region);
    if (it == graph_.end()) {
      continue;
    }

    for (const auto & edge : it->second) {
      const double new_cost = current.cost + edge.cost;
      if (!dist.count(edge.to) || new_cost < dist[edge.to]) {
        dist[edge.to] = new_cost;
        parent_region[edge.to] = current.region;
        parent_connector[edge.to] = edge.connector_index;
        open.push(QueueItem{new_cost, edge.to});
      }
    }
  }

  if (!dist.count(goal_region) || !std::isfinite(dist[goal_region])) {
    return result;
  }

  std::vector<std::string> reversed_regions;
  std::vector<int> reversed_connectors;
  std::string cur = goal_region;
  reversed_regions.push_back(cur);

  while (cur != start_region) {
    if (!parent_region.count(cur) || !parent_connector.count(cur)) {
      return result;
    }
    reversed_connectors.push_back(parent_connector[cur]);
    cur = parent_region[cur];
    reversed_regions.push_back(cur);
  }

  std::reverse(reversed_regions.begin(), reversed_regions.end());
  std::reverse(reversed_connectors.begin(), reversed_connectors.end());

  result.success = true;
  result.region_path = reversed_regions;

  // 旧版本只保留一条路，如果C1 C2都可以走且cost相等，但是buildgraph的时候先从yaml中识别到C1,那么就算C2更优，
  const auto optimized_connectors = optimizeConnectorsForRegionPath(reversed_regions, start, goal);

  if (optimized_connectors.size() == reversed_connectors.size()) {
    result.connector_indices = optimized_connectors;
  } else {
    result.connector_indices = reversed_connectors;
  }

  return result;
}


// 适用于区域间多 connector 的情况
std::vector<int> TopologyGlobalPlanner::optimizeConnectorsForRegionPath(
  const std::vector<std::string> & region_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (region_path.size() < 2) {
    return {};
  }

  const size_t step_count = region_path.size() - 1;
  std::vector<std::vector<int>> layers;
  layers.reserve(step_count);

  // 每段区域转移收集 cost 最低的候选 Connector
  for (size_t step = 0; step < step_count; ++step) {
    const std::string & from_region = region_path[step];
    const std::string & to_region = region_path[step + 1];

    double min_connector_cost = std::numeric_limits<double>::infinity();
    std::vector<int> candidates;

    for (size_t i = 0; i < connectors_.size(); ++i) {
      const auto & c = connectors_[i];

      const bool forward = c.from == from_region && c.to == to_region;
      const bool reverse = c.mode == "two_way" && c.from == to_region && c.to == from_region;
      if (!forward && !reverse) {
        continue;
      }

      if (c.cost < min_connector_cost - 1e-6) {
        min_connector_cost = c.cost;
        candidates.clear();
        candidates.push_back(static_cast<int>(i));
      } else if (std::abs(c.cost - min_connector_cost) <= 1e-6) {
        candidates.push_back(static_cast<int>(i));
      }
    }

    if (candidates.empty()) {
      return {};
    }

    layers.push_back(candidates);
  }

  // 根据本次区域通过方向获得 wait 和 exit
  auto get_wait_exit = [this, &region_path](size_t step, int idx, Point2D & wait, Point2D & exit) -> bool
  {
    const auto & c = connectors_[static_cast<size_t>(idx)];
    const auto & current_region = region_path[step];
    const auto & next_region = region_path[step + 1];

    if (c.from == current_region && c.to == next_region) {
      wait = c.portal_start;
      exit = c.portal_end;
      return true;
    }

    if (c.mode == "two_way" && c.to == current_region && c.from == next_region) {
      wait = c.portal_end;
      exit = c.portal_start;
      return true;
    }

    return false;
  };

  auto point_dist = [this](const Point2D & a, const Point2D & b) {
    return euclidean(a.x, a.y, b.x, b.y);
  };

  auto pose_point_dist = [this](
    const geometry_msgs::msg::PoseStamped & pose,
    const Point2D & point)
  {
    return euclidean(pose.pose.position.x, pose.pose.position.y, point.x, point.y);
  };

  const size_t layer_count = layers.size();

  std::vector<std::vector<double>> dp(layer_count);
  std::vector<std::vector<int>> parent(layer_count);

  for (size_t layer = 0; layer < layer_count; ++layer) {
    dp[layer].assign(layers[layer].size(), std::numeric_limits<double>::infinity());
    parent[layer].assign(layers[layer].size(), -1);
  }

  // 第一层：start -> wait -> exit
  for (size_t j = 0; j < layers[0].size(); ++j) {
    Point2D wait, exit;
    if (!get_wait_exit(0, layers[0][j], wait, exit)) {
      continue;
    }

    dp[0][j] = pose_point_dist(start, wait) + point_dist(wait, exit);
  }

  // 中间层：上一个 exit -> 当前 wait -> 当前 exit
  for (size_t layer = 1; layer < layer_count; ++layer) {
    for (size_t j = 0; j < layers[layer].size(); ++j) {
      const int cur_idx = layers[layer][j];

      Point2D cur_wait, cur_exit;
      if (!get_wait_exit(layer, cur_idx, cur_wait, cur_exit)) {
        continue;
      }

      for (size_t k = 0; k < layers[layer - 1].size(); ++k) {
        if (!std::isfinite(dp[layer - 1][k])) {
          continue;
        }

        const int prev_idx = layers[layer - 1][k];
        Point2D prev_wait, prev_exit;

        if (!get_wait_exit(layer - 1, prev_idx, prev_wait, prev_exit)) {
          continue;
        }

        const double candidate = dp[layer - 1][k] + point_dist(prev_exit, cur_wait) + point_dist(cur_wait, cur_exit);

        if (candidate < dp[layer][j]) {
          dp[layer][j] = candidate;
          parent[layer][j] = static_cast<int>(k);
        }
      }
    }
  }

  // 最后一层：最后一个 exit -> goal
  double best_total = std::numeric_limits<double>::infinity();
  int best_last = -1;
  const size_t last_layer = layer_count - 1;

  for (size_t j = 0; j < layers[last_layer].size(); ++j) {
    if (!std::isfinite(dp[last_layer][j])) {
      continue;
    }

    Point2D wait, exit;
    if (!get_wait_exit(last_layer, layers[last_layer][j], wait, exit)) {
      continue;
    }

    const double total = dp[last_layer][j] + pose_point_dist(goal, exit);

    if (total < best_total) {
      best_total = total;
      best_last = static_cast<int>(j);
    }
  }

  if (best_last < 0) {
    return {};
  }

  // 回溯选中的 Connector
  std::vector<int> selected(layer_count, -1);
  int cur = best_last;

  for (int layer = static_cast<int>(layer_count) - 1; layer >= 0; --layer) {
    selected[static_cast<size_t>(layer)] = layers[static_cast<size_t>(layer)][static_cast<size_t>(cur)];
    cur = parent[static_cast<size_t>(layer)][static_cast<size_t>(cur)];

    if (layer > 0 && cur < 0) {
      return {};
    }
  }

  return selected;
}

}  // namespace topology_global_planner
