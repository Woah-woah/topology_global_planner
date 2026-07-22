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

  geometry_msgs::msg::PoseStamped current_pose = start;
  bool handled_active_connector = false;

  if (is_on_connector_) {
    const bool is_in_next_region = pointInRegionWithTolerance(
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      active_region_id_,
      region_constraint_tolerance_);

    if (is_in_next_region) {
      handled_active_connector = true;

      const double vx = active_exit_point_.x - active_wait_point_.x;
      const double vy = active_exit_point_.y - active_wait_point_.y;
      const double length_sq = vx * vx + vy * vy;

      if (length_sq <= 1e-9) {
        RCLCPP_ERROR(logger_, "Invalid active connector geometry");
        is_on_connector_ = false;
        active_region_id_.clear();
        return makeEmptyPath();
      }

      const double px = current_pose.pose.position.x - active_wait_point_.x;
      const double py = current_pose.pose.position.y - active_wait_point_.y;

      const double t = (px * vx + py * vy) / length_sq;

      const double proj_x = active_wait_point_.x + t * vx;
      const double proj_y = active_wait_point_.y + t * vy;

      const double distance_to_exit = euclidean(
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        active_exit_point_.x,
        active_exit_point_.y);

      if (t >= 1.0 || distance_to_exit <= 0.15) {
        RCLCPP_INFO(logger_, "Robot has completed active connector traversal");
        is_on_connector_ = false;
        active_region_id_.clear();
      } else {
        const double connector_yaw = std::atan2(vy, vx);

        auto straight_start = makePoseFromPoint(active_wait_point_, full_path.header.stamp);
        straight_start.pose.position.x = proj_x;
        straight_start.pose.position.y = proj_y;
        setYaw(straight_start, connector_yaw);

        auto active_exit_pose = makePoseFromPoint(active_exit_point_, full_path.header.stamp);
        setYaw(active_exit_pose, connector_yaw);

        auto approach_path = makeStraightConnectorPath(current_pose, straight_start, no_action_line_resolution_);
        if (approach_path.poses.empty()) {
          return makeEmptyPath();
        }
        appendSegment(full_path, approach_path);

        auto exit_path = makeStraightConnectorPath(straight_start, active_exit_pose, no_action_line_resolution_);
        if (exit_path.poses.empty()) {
          return makeEmptyPath();
        }
        appendSegment(full_path, exit_path);

        current_pose = active_exit_pose;
      }
    }
  }


  if (topo_result.region_path.empty()) {
    // 前面已经补完 active connector
    if (!full_path.poses.empty()) {
      auto path_to_goal = makeInnerPlannerPath(current_pose, goal);
      if (path_to_goal.poses.empty()) {
        RCLCPP_WARN(logger_, "Failed to plan from active connector exit to goal");
        return makeEmptyPath();
      }
      appendSegment(full_path, path_to_goal);

      return full_path;
    }

    RCLCPP_INFO(logger_, "No topology result");
    return makeEmptyPath();
  }

  if (topo_result.connector_indices.size() + 1 != topo_result.region_path.size())
  {
    RCLCPP_WARN(logger_, "Invalid topology search result");
    return makeEmptyPath();
  }


  for (size_t step = 0; step < topo_result.connector_indices.size(); ++step)
  {
    const int connector_index = topo_result.connector_indices[step];

    if (connector_index < 0 || static_cast<size_t>(connector_index) >= connectors_.size())
    {
      RCLCPP_ERROR(logger_, "Invalid connector index: %d", connector_index);
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

    auto wait_pose = makePoseFromPoint(wait_point, full_path.header.stamp);
    auto exit_pose = makePoseFromPoint(exit_point, full_path.header.stamp);

    const double connector_yaw = std::atan2(exit_point.y - wait_point.y, exit_point.x - wait_point.x);

    setYaw(wait_pose, connector_yaw);
    setYaw(exit_pose, connector_yaw);
    
    // 第一段 Connector 重规划时，判断机器人是否已经越过 wait_pose
    bool entered_connector = false;
    auto straight_start = wait_pose;
    double proj_x = 0.0;
    double proj_y = 0.0;

    if (step == 0) {
      const double vx = exit_point.x - wait_point.x;
      const double vy = exit_point.y - wait_point.y;
      const double length_sq = vx * vx + vy * vy;

      if (length_sq > 1e-9) {
        const double px = current_pose.pose.position.x - wait_point.x;
        const double py = current_pose.pose.position.y - wait_point.y;
        const double t = (px * vx + py * vy) / length_sq;

        proj_x = wait_point.x + t * vx;
        proj_y = wait_point.y + t * vy;
        const double lateral_distance = euclidean(
          current_pose.pose.position.x,
          current_pose.pose.position.y,
          proj_x,
          proj_y);

        entered_connector = t > 0.02 && t < 1.0 && lateral_distance < 0.30;
      }
    }

    if (!entered_connector) {
      // 还没进入 Connector：当前位置 -> wait_pose
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
    } else {
      is_on_connector_ = true;
      active_wait_point_ = wait_point;
      active_exit_point_ = exit_point;
      active_region_id_ = next_region;

      // 已经越过 wait_pose：直接从机器人当前投影位置走直线到 exit_pose
      auto proj_pose = makePoseFromPoint(Point2D{proj_x, proj_y}, full_path.header.stamp);

      straight_start = proj_pose;
      straight_start.header.frame_id = global_frame_;
      straight_start.header.stamp = full_path.header.stamp;
      setYaw(straight_start, connector_yaw);

      auto connector_approach_straight_path = makeStraightConnectorPath(current_pose, straight_start, no_action_line_resolution_);
      if (connector_approach_straight_path.poses.empty()) {
        RCLCPP_WARN(logger_, "Failed to generate straight path for connector '%s'", connector.id.c_str());
        return makeEmptyPath();
      }
      appendSegment(full_path, connector_approach_straight_path);
    }

    // wait_pose -> exit_pose，或者机器人当前位置 -> exit_pose
    auto connector_straight_path = makeStraightConnectorPath(straight_start, exit_pose, no_action_line_resolution_);

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
    RCLCPP_WARN(logger_, "Final segment leaves region '%s'", topo_result.region_path.back().c_str());
    return makeEmptyPath();
  }

  appendSegment(full_path, path_to_goal);

  return full_path;
}

void TopologyGlobalPlanner::publishNeedAction()
{
  std_msgs::msg::Bool msg;
  msg.data = need_action_;
  need_action_pub_->publish(msg);
}
}  // namespace topology_global_planner
