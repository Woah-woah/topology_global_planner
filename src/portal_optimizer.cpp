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

geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::makePoseFromPoint(
  const Point2D & point,
  const rclcpp::Time & stamp) const                 // 把二维点，变成 ROS/Nav2 能用的 pose
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = stamp;
  pose.pose.position.x = point.x;
  pose.pose.position.y = point.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::Path TopologyGlobalPlanner::makeStraightConnectorPath( //把waitpose到exitpose输出成直线
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double resolution) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_ ? clock_->now() : rclcpp::Time(start.header.stamp);

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= 1e-9) {
    path.poses.push_back(start);
    path.poses.push_back(goal);
    return path;
  }

  resolution = std::max(0.01, resolution);

  const int segment_count = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
  const double yaw = std::atan2(dy, dx);

  path.poses.reserve(static_cast<size_t>(segment_count + 1));

  for (int i = 0; i <= segment_count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(segment_count);

    auto pose = start;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = path.header.stamp;

    pose.pose.position.x = start.pose.position.x + t * dx;
    pose.pose.position.y = start.pose.position.y + t * dy;
    pose.pose.position.z = start.pose.position.z + t * (goal.pose.position.z - start.pose.position.z);

    setYaw(pose, yaw);
    path.poses.push_back(pose);
  }

  return path;
}

nav_msgs::msg::Path TopologyGlobalPlanner::makePortalOptimizedTopologyPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const TopologySearchResult & topo_result)
{
  auto makeEmptyPath = [this]() -> nav_msgs::msg::Path {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = global_frame_;
    empty_path.header.stamp = clock_ ? clock_->now() : rclcpp::Time(0);
    return empty_path;
  };

  nav_msgs::msg::Path full_path;
  full_path.header.frame_id = global_frame_;
  full_path.header.stamp = clock_ ? clock_->now() : rclcpp::Time(0);

  if (
    topo_result.region_path.empty() ||
    topo_result.connector_indices.size() + 1 !=
    topo_result.region_path.size())
  {
    RCLCPP_ERROR(
      logger_,
      "Invalid topology result: regions=%zu connectors=%zu",
      topo_result.region_path.size(),
      topo_result.connector_indices.size());

    return makeEmptyPath();
  }

  geometry_msgs::msg::PoseStamped current_pose = start;

  for (size_t step = 0; step < topo_result.connector_indices.size(); ++step)
  {
    const int connector_index = topo_result.connector_indices[step];

    if (connector_index < 0 || static_cast<size_t>(connector_index) >= connectors_.size())
    {
      RCLCPP_ERROR(
        logger_,
        "Invalid connector index: %d",
        connector_index);

      return makeEmptyPath();
    }

    const auto & connector = connectors_[static_cast<size_t>(connector_index)];

    const std::string & current_region = topo_result.region_path[step];
    const std::string & next_region = topo_result.region_path[step + 1];

    Point2D wait_point;
    Point2D exit_point;

    // 正向通过：from -> to
    if (connector.from == current_region && connector.to == next_region)
    {
      wait_point = connector.portal_start;
      exit_point = connector.portal_end;
    }
    // 反向通过：to -> from
    else if (connector.mode == "two_way" && connector.to == current_region && connector.from == next_region)
    {
      wait_point = connector.portal_end;
      exit_point = connector.portal_start;
    }
    else {
      RCLCPP_ERROR(
        logger_,
        "Connector '%s' does not match transition '%s' -> '%s'",
        connector.id.c_str(),
        current_region.c_str(),
        next_region.c_str());

      return makeEmptyPath();
    }

    auto wait_pose = makePoseFromPoint(wait_point, full_path.header.stamp);
    auto exit_pose = makePoseFromPoint(exit_point, full_path.header.stamp);

    const double connector_yaw = std::atan2(exit_point.y - wait_point.y, exit_point.x - wait_point.x);

    setYaw(wait_pose, connector_yaw);
    setYaw(exit_pose, connector_yaw);

    // ① 当前点 -> wait_pose：InnerPlanner
    auto path_to_wait = makeInnerPlannerPath(current_pose, wait_pose);

    if (path_to_wait.poses.empty()) {
      RCLCPP_WARN(logger_, "Failed to plan to wait pose of connector '%s'", connector.id.c_str());
      return makeEmptyPath();
    }

    if (enforce_region_constraint_ && !pathInsideRegionWithTolerance(path_to_wait, current_region, region_constraint_tolerance_))
    {
      RCLCPP_WARN(logger_, "Path to connector '%s' leaves region '%s'", connector.id.c_str(), current_region.c_str());
      return makeEmptyPath();
    }

    appendSegment(full_path, path_to_wait);

    // ② wait_pose -> exit_pose：固定直线
    auto connector_straight_path = makeStraightConnectorPath(wait_pose, exit_pose, no_action_line_resolution_);

    if (connector_straight_path.poses.empty()) {
      RCLCPP_WARN(logger_, "Failed to generate straight path for connector '%s'", connector.id.c_str());
      return makeEmptyPath();
    }

    appendSegment(full_path, connector_straight_path);

    // 下一段从当前 Connector 出口开始
    current_pose = exit_pose;
  }

  // ③ 最后一个 exit_pose -> 最终 goal
  auto path_to_goal = makeInnerPlannerPath(current_pose, goal);

  if (path_to_goal.poses.empty()) {
    RCLCPP_WARN(logger_, "Failed to plan final segment to goal");
    return makeEmptyPath();
  }

  if (enforce_region_constraint_ && !pathInsideRegionWithTolerance(
      path_to_goal,
      topo_result.region_path.back(),
      region_constraint_tolerance_))
  {
    RCLCPP_WARN(
      logger_,
      "Final segment leaves region '%s'",
      topo_result.region_path.back().c_str());

    return makeEmptyPath();
  }

  appendSegment(full_path, path_to_goal);

  return full_path;
}


}  // namespace topology_global_planner
