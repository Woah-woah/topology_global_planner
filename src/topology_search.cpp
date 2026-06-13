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

TopologySearchResult TopologyGlobalPlanner::searchTopology(const std::string & start_region, const std::string & goal_region) const {
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
  result.connector_indices = reversed_connectors;
  return result;
}


}  // namespace topology_global_planner
