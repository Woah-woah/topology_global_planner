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


// createPlan 前的输入 pose 标准化函数
geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::normalizePoseFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped normalized = pose;
  if (normalized.header.frame_id.empty()) {
    normalized.header.frame_id = global_frame_;                  // frame id空的话，补成global_frame_
  }

  // Do not overwrite a meaningful timestamp. Nav2 usually supplies poses in the global
  // frame already; preserving the stamp avoids asking TF for a transform at the wrong time.
  if (normalized.header.frame_id == global_frame_ || !tf_) {
    if (normalized.header.stamp.sec == 0 && normalized.header.stamp.nanosec == 0 && clock_) {
      normalized.header.stamp = clock_->now();
    }
    return normalized;                                           // stamp 空的话，补成当前时间
  }

  try {
    geometry_msgs::msg::PoseStamped transformed;
    tf_->transform(
      normalized, transformed, global_frame_,
      tf2::durationFromSec(std::max(0.0, transform_tolerance_)));
    if (transformed.header.stamp.sec == 0 && transformed.header.stamp.nanosec == 0 && clock_) {
      transformed.header.stamp = clock_->now();
    }                                                            // 把 normalized 这个 pose 从它自己 header 里的坐标系
    return transformed;                                          // 转换到 global_frame_，并且最多等待 transform_tolerance_ 这么长时间来查 TF
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Failed to transform pose from '%s' to '%s': %s. Using original pose.",
      normalized.header.frame_id.c_str(), global_frame_.c_str(), e.what());
    return normalized;
  }
}

// 把小段路径拼成full_path
void TopologyGlobalPlanner::appendSegment(nav_msgs::msg::Path & full_path, const nav_msgs::msg::Path & segment) const
{
  if (segment.poses.empty()) {
    return;
  }

  if (full_path.header.frame_id.empty()) {
    full_path.header = segment.header;
  }

  size_t start_index = 0;
  if (!full_path.poses.empty()) {                                   // fullpath非空, 拿fullpath的最后一个点，去和接进来的小段的第一个点作比较，如果很近，就认为是一个点
    const auto & last = full_path.poses.back().pose.position;
    const auto & first = segment.poses.front().pose.position;
    if (euclidean(last.x, last.y, first.x, first.y) <= duplicate_pose_tolerance_) {
      start_index = 1;                                              // 放弃第一个点，从下一个点开始拼
    }
  }

  for (size_t i = start_index; i < segment.poses.size(); i++) {
    full_path.poses.push_back(segment.poses[i]);                    // 把小段的点放进fullpath
  }
}


}  // namespace topology_global_planner
