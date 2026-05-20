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

void TopologyGlobalPlanner::assignIntermediateOrientations(
  std::vector<geometry_msgs::msg::PoseStamped> & waypoints) const
{
  if (waypoints.size() < 3) {
    return;
  }

  // Keep start pose orientation and final goal orientation unchanged.
  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    const auto & current = waypoints[i].pose.position;
    const auto & next = waypoints[i + 1].pose.position;
    double dx = next.x - current.x;
    double dy = next.y - current.y;

    if (std::hypot(dx, dy) <= duplicate_pose_tolerance_) {
      // Degenerate case: look backward instead of producing an arbitrary yaw.
      const auto & prev = waypoints[i - 1].pose.position;
      dx = current.x - prev.x;
      dy = current.y - prev.y;
    }

    if (std::hypot(dx, dy) > 1e-6) {
      setYaw(waypoints[i], std::atan2(dy, dx));
    }
  }
}

void TopologyGlobalPlanner::setYaw(geometry_msgs::msg::PoseStamped & pose, double yaw) const
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
}

nav_msgs::msg::Path TopologyGlobalPlanner::makeInnerPlannerPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto path = inner_planner_->createPlan(start, goal);
  if (!path.poses.empty()) {
    path.header.frame_id = path.header.frame_id.empty() ? global_frame_ : path.header.frame_id;
    if (clock_) {
      path.header.stamp = clock_->now();
    }
  }
  return path;
}

nav_msgs::msg::Path TopologyGlobalPlanner::makeWeakTopologyPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
{
  nav_msgs::msg::Path full_path;
  full_path.header.frame_id = global_frame_;
  full_path.header.stamp = clock_ ? clock_->now() : rclcpp::Time(0);

  if (waypoints.size() < 2) {
    return full_path;
  }

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    auto segment = makeInnerPlannerPath(waypoints[i], waypoints[i + 1]);
    if (segment.poses.empty()) {
      RCLCPP_WARN(
        logger_, "Inner planner segment %zu/%zu failed in weak topology path",
        i + 1, waypoints.size() - 1);
      full_path.poses.clear();
      return full_path;
    }
    appendSegment(full_path, segment);
  }

  return full_path;
}

nav_msgs::msg::Path TopologyGlobalPlanner::fallbackDirectPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & reason)
{
  RCLCPP_WARN(logger_, "%s. %s", reason.c_str(),
    fallback_to_inner_planner_ ? "Using direct inner planner." : "Fallback disabled; returning empty path.");

  if (!fallback_to_inner_planner_) {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = global_frame_;
    empty.header.stamp = clock_ ? clock_->now() : rclcpp::Time(0);
    return empty;
  }

  return makeInnerPlannerPath(start, goal);
}

geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::normalizePoseFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped normalized = pose;
  if (normalized.header.frame_id.empty()) {
    normalized.header.frame_id = global_frame_;
  }

  // Do not overwrite a meaningful timestamp. Nav2 usually supplies poses in the global
  // frame already; preserving the stamp avoids asking TF for a transform at the wrong time.
  if (normalized.header.frame_id == global_frame_ || !tf_) {
    if (normalized.header.stamp.sec == 0 && normalized.header.stamp.nanosec == 0 && clock_) {
      normalized.header.stamp = clock_->now();
    }
    return normalized;
  }

  try {
    geometry_msgs::msg::PoseStamped transformed;
    tf_->transform(
      normalized, transformed, global_frame_,
      tf2::durationFromSec(std::max(0.0, transform_tolerance_)));
    if (transformed.header.stamp.sec == 0 && transformed.header.stamp.nanosec == 0 && clock_) {
      transformed.header.stamp = clock_->now();
    }
    return transformed;
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to transform pose from '%s' to '%s': %s. Using original pose.",
      normalized.header.frame_id.c_str(), global_frame_.c_str(), e.what());
    return normalized;
  }
}

void TopologyGlobalPlanner::appendSegment(nav_msgs::msg::Path & full_path, const nav_msgs::msg::Path & segment) const
{
  if (segment.poses.empty()) {
    return;
  }

  if (full_path.header.frame_id.empty()) {
    full_path.header = segment.header;
  }

  size_t start_index = 0;
  if (!full_path.poses.empty()) {
    const auto & last = full_path.poses.back().pose.position;
    const auto & first = segment.poses.front().pose.position;
    if (euclidean(last.x, last.y, first.x, first.y) <= duplicate_pose_tolerance_) {
      start_index = 1;
    }
  }

  for (size_t i = start_index; i < segment.poses.size(); ++i) {
    full_path.poses.push_back(segment.poses[i]);
  }
}


}  // namespace topology_global_planner
