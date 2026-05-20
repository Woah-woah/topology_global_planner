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
  const std::string & goal_region) const
{
  TopologySearchResult result;

  struct QueueItem
  {
    double cost;
    std::string region;
    bool operator>(const QueueItem & other) const {return cost > other.cost;}
  };

  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  std::unordered_map<std::string, double> dist;
  std::unordered_map<std::string, std::string> parent_region;
  std::unordered_map<std::string, int> parent_connector;

  for (const auto & r : regions_) {
    dist[r.id] = std::numeric_limits<double>::infinity();
  }
  dist[start_region] = 0.0;
  open.push(QueueItem{0.0, start_region});

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();

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

std::vector<geometry_msgs::msg::PoseStamped> TopologyGlobalPlanner::buildTopologyWaypoints(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const TopologySearchResult & topo_result) const
{
  // Compatibility helper for the old weak topology path mode.
  // Portal connectors are reduced to the middle sample here. The preferred path
  // builder is makePortalOptimizedTopologyPath(), which optimizes over all samples.
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  auto fixed_start = start;
  auto fixed_goal = goal;
  fixed_start.header.frame_id = global_frame_;
  fixed_goal.header.frame_id = global_frame_;
  const auto plan_stamp = fixed_start.header.stamp.sec == 0 && fixed_start.header.stamp.nanosec == 0 && clock_ ?
    clock_->now() : rclcpp::Time(fixed_start.header.stamp);
  fixed_start.header.stamp = plan_stamp;
  fixed_goal.header.stamp = plan_stamp;

  waypoints.push_back(fixed_start);
  for (const int idx : topo_result.connector_indices) {
    if (idx >= 0 && static_cast<size_t>(idx) < connectors_.size()) {
      const auto samples = sampleConnectorPoses(connectors_[idx], plan_stamp);
      if (!samples.empty()) {
        waypoints.push_back(samples[samples.size() / 2]);
      }
    }
  }
  waypoints.push_back(fixed_goal);

  std::vector<geometry_msgs::msg::PoseStamped> cleaned;
  for (const auto & wp : waypoints) {
    if (cleaned.empty() ||
      euclidean(
        cleaned.back().pose.position.x, cleaned.back().pose.position.y,
        wp.pose.position.x, wp.pose.position.y) > duplicate_pose_tolerance_)
    {
      cleaned.push_back(wp);
    }
  }

  assignIntermediateOrientations(cleaned);
  return cleaned;
}


}  // namespace topology_global_planner
