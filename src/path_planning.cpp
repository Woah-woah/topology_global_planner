#include "topology_global_planner/topology_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

namespace topology_global_planner
{

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
  const geometry_msgs::msg::PoseStamped & goal,
  grid::GoalPolicy goal_policy)
{
  if (!grid_planner_) {
    throw std::runtime_error("GridPlanner is not initialized");
  }
  auto path = grid_planner_->plan(start, goal, goal_policy, cancel_checker_);

  if (!path.poses.empty()) {
    if (path.header.frame_id.empty()) {
      path.header.frame_id = global_frame_;
    }
    if (clock_) {
      path.header.stamp = clock_->now();
    }
  }
  if (!path.poses.empty()) {
    const auto end = path.poses.back().pose.position;
    const double offset = std::hypot(end.x - goal.pose.position.x, end.y - goal.pose.position.y);
    if (goal_policy == grid::GoalPolicy::EXACT && offset > 1e-6) {
      throw std::runtime_error("GridPlanner did not reach the exact segment endpoint");
    }
    path.poses.insert(path.poses.begin(), start);
    if (offset > 1e-6) {
      RCLCPP_INFO(logger_,
        "Final goal (%.3f, %.3f) uses reachable endpoint (%.3f, %.3f), offset=%.3fm",
        goal.pose.position.x,
        goal.pose.position.y,
        end.x,
        end.y,
        offset);
    }
    // 保留容差内的可达终点
    // 直接改回原目标可能落入障碍
    path.poses.back().pose.orientation = goal.pose.orientation;
  }
  return path;
}

// 将输入位姿统一到规划坐标系
geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::normalizePoseFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped normalized = pose;
  if (normalized.header.frame_id.empty()) {
    normalized.header.frame_id = global_frame_;
  }

  // 已有时间戳保持不变
  // 避免查询到错误时刻的坐标变换
  if (normalized.header.frame_id == global_frame_ || !tf_) {
    if (normalized.header.stamp.sec == 0 && normalized.header.stamp.nanosec == 0 && clock_) {
      normalized.header.stamp = clock_->now();
    }
    return normalized;
  }

  try {
    geometry_msgs::msg::PoseStamped transformed;
    tf_->transform(normalized,
      transformed,
      global_frame_,
      tf2::durationFromSec(std::max(0.0, transform_tolerance_)));
    if (transformed.header.stamp.sec == 0 && transformed.header.stamp.nanosec == 0 && clock_) {
      transformed.header.stamp = clock_->now();
    }
    return transformed;
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_,
      "Failed to transform pose from '%s' to '%s': %s. Planning aborted.",
      normalized.header.frame_id.c_str(),
      global_frame_.c_str(),
      e.what());
    throw;
  }
}

// 按执行顺序拼接完整路径
void TopologyGlobalPlanner::appendSegment(
  nav_msgs::msg::Path & full_path, const nav_msgs::msg::Path & segment) const
{
  if (segment.poses.empty()) {
    return;
  }

  if (full_path.header.frame_id.empty()) {
    full_path.header = segment.header;
  }

  // 保留分段交界处的两个位姿及各自朝向
  full_path.poses.insert(full_path.poses.end(), segment.poses.begin(), segment.poses.end());
}

}
