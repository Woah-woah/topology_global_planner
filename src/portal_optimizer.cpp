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

void TopologyGlobalPlanner::clearPlannedConnector()
{
  planned_connector_id_.clear();
  planned_next_region_id_.clear();
  has_planned_connector_geometry_ = false;
}

void TopologyGlobalPlanner::updateConnectorEntryLatch(
  const geometry_msgs::msg::PoseStamped & current_pose)
{
  if (is_on_connector_) {
    const double vx = active_exit_point_.x - active_wait_point_.x;
    const double vy = active_exit_point_.y - active_wait_point_.y;
    const double length_sq = vx * vx + vy * vy;

    if (length_sq > 1e-9) {
      const double px = current_pose.pose.position.x - active_wait_point_.x;
      const double py = current_pose.pose.position.y - active_wait_point_.y;
      const double t = (px * vx + py * vy) / length_sq;

      if (t <= -0.1) {
        RCLCPP_INFO(
          logger_, "Robot exited active connector '%s' from the entrance side",
          active_connector_id_.c_str());
        is_on_connector_ = false;
        active_region_id_.clear();
        active_connector_id_.clear();
        clearPlannedConnector();
      }
    }
    return;
  }

  if (planned_connector_id_.empty() || !has_planned_connector_geometry_)
  {
    return;
  }

  const double vx = planned_exit_point_.x - planned_wait_point_.x;
  const double vy = planned_exit_point_.y - planned_wait_point_.y;
  const double length_sq = vx * vx + vy * vy;

  if (length_sq <= 1e-9) {
    RCLCPP_ERROR(
      logger_, "Invalid planned connector geometry for '%s'",
      planned_connector_id_.c_str());
    clearPlannedConnector();
    return;
  }

  const double px = current_pose.pose.position.x - planned_wait_point_.x;
  const double py = current_pose.pose.position.y - planned_wait_point_.y;
  const double t = (px * vx + py * vy) / length_sq;
  const double projection_x = planned_wait_point_.x + t * vx;
  const double projection_y = planned_wait_point_.y + t * vy;
  const double lateral_distance = euclidean(
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    projection_x,
    projection_y);

  if (t >= 0.8) {
    // 两次重规划之间已经直接越过出口，不再锁存旧 Connector。
    clearPlannedConnector();
    return;
  }

  if (t <= -0.2 || lateral_distance >= 0.80) {
    return;
  }

  is_on_connector_ = true;
  active_wait_point_ = planned_wait_point_;
  active_exit_point_ = planned_exit_point_;
  active_region_id_ = planned_next_region_id_;
  active_connector_id_ = planned_connector_id_;
  planned_next_region_id_.clear();
  has_planned_connector_geometry_ = false;

  RCLCPP_INFO(
    logger_, "Latched planned connector '%s' as active at t=%.3f",
    active_connector_id_.c_str(), t);
}

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

  if (is_on_connector_) {
    const bool is_in_next_region = pointInRegionWithTolerance(
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      active_region_id_,
      region_constraint_tolerance_);

    if (is_in_next_region) {
      const double vx = active_exit_point_.x - active_wait_point_.x;
      const double vy = active_exit_point_.y - active_wait_point_.y;
      const double length_sq = vx * vx + vy * vy;

      if (length_sq <= 1e-9) {
        RCLCPP_ERROR(logger_, "Invalid active connector geometry");
        is_on_connector_ = false;
        active_region_id_.clear();
        active_connector_id_.clear();
        clearPlannedConnector();
        return makeEmptyPath();
      }

      const double px = current_pose.pose.position.x - active_wait_point_.x;
      const double py = current_pose.pose.position.y - active_wait_point_.y;

      const double t = (px * vx + py * vy) / length_sq;

      // Region 边界可能位于 Connector 中部，进入下一 Region 不等于已通过。
      // 只有到达或越过 exit_point 后才释放 active_connector_id_ 锁存。
      if (t >= 1.0) {
        RCLCPP_INFO(logger_, "Robot has completed active connector traversal");
        is_on_connector_ = false;
        active_region_id_.clear();
        active_connector_id_.clear();
        clearPlannedConnector();
      } else {
        const double connector_yaw = std::atan2(vy, vx);

        auto connector_start = makePoseFromPoint(active_wait_point_, full_path.header.stamp);
        connector_start = current_pose;

        auto active_exit_pose = makePoseFromPoint(active_exit_point_, full_path.header.stamp);
        setYaw(active_exit_pose, connector_yaw);

        auto exit_path = makeInnerPlannerPath(connector_start, active_exit_pose);
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
    else
    {
      RCLCPP_ERROR(
        logger_, "Connector '%s' does not match transition '%s' -> '%s'",
        connector.id.c_str(), current_region.c_str(), next_region.c_str());
      return makeEmptyPath();
    }

    auto wait_pose = makePoseFromPoint(wait_point, full_path.header.stamp);
    auto exit_pose = makePoseFromPoint(exit_point, full_path.header.stamp);

    const double connector_yaw = std::atan2(exit_point.y - wait_point.y, exit_point.x - wait_point.x);

    setYaw(wait_pose, connector_yaw);
    setYaw(exit_pose, connector_yaw);
    
    // 第一段 Connector 重规划时，判断机器人是否已经越过 wait_pose
    bool entered_connector = false;
    auto connector_start = wait_pose;
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

        entered_connector = t > -0.2 && t < 1.0 && lateral_distance < 0.80;
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
      active_connector_id_ = connector.id;
      planned_connector_id_ = connector.id;
      planned_next_region_id_.clear();
      has_planned_connector_geometry_ = false;

      // 已经越过 wait_pose：直接从机器人当前位置走到 exit_pose

      connector_start = current_pose;
      connector_start.header.frame_id = global_frame_;
      connector_start.header.stamp = full_path.header.stamp;
      setYaw(connector_start, connector_yaw);
    }

    // wait_pose -> exit_pose，或者机器人当前位置 -> exit_pose
    auto connector_path = makeInnerPlannerPath(connector_start, exit_pose);

    if (connector_path.poses.empty()) {
      RCLCPP_WARN(logger_, "Failed to plan path through connector '%s'", connector.id.c_str());
      return makeEmptyPath();
    }
    if (!pathInsideConnectorStrip(connector_path, wait_point, exit_point, connector_path_half_width_))
    {
      RCLCPP_WARN(logger_, "Connector '%s' path makes an excessive detour", connector.id.c_str());
      return makeEmptyPath();
    }
    appendSegment(full_path, connector_path);

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
  if (!need_action_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = is_on_connector_ && !active_connector_id_.empty() ?
    active_connector_id_ : planned_connector_id_;
  need_action_pub_->publish(msg);
}
}  // namespace topology_global_planner
