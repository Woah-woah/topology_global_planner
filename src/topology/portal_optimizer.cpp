#include "topology_global_planner/topology_global_planner.hpp"

#include <cmath>
#include <stdexcept>

namespace topology_global_planner
{
geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::makePoseFromPoint(
  const Point2D & point, const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = stamp;
  pose.pose.position.x = point.x;
  pose.pose.position.y = point.y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

TopologyPlan TopologyGlobalPlanner::makePortalOptimizedTopologyPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const TopologySearchResult & topology)
{
  if (topology.region_path.empty() ||
      topology.connector_indices.size() + 1 != topology.region_path.size())
  {
    throw std::runtime_error("Invalid topology search result");
  }

  TopologyPlan plan;
  auto current = start;

  const auto add_normal = [&](const geometry_msgs::msg::PoseStamped & target,
                            const std::string & region,
                            grid::GoalPolicy goal_policy) {
    Segment segment;
    segment.path = makeInnerPlannerPath(current, target, goal_policy,
      [this, &region](const auto & candidate) {
        return pathInsideRegionWithTolerance(candidate, region, region_constraint_tolerance_);
      });
    if (segment.path.poses.empty() ||
        !pathInsideRegionWithTolerance(segment.path, region, region_constraint_tolerance_))
    {
      throw std::runtime_error("Normal segment leaves region " + region);
    }
    appendSegment(plan.full_path, segment.path);
    plan.segments.push_back(std::move(segment));
  };

  for (size_t step = 0; step < topology.connector_indices.size(); ++step) {
    if (cancel_checker_()) {
      throw std::runtime_error("Planning cancelled");
    }

    const auto index = topology.connector_indices[step];
    if (index < 0 || static_cast<size_t>(index) >= connectors_.size()) {
      throw std::runtime_error("Invalid connector index");
    }

    const auto & connector = connectors_[index];
    Segment segment;
    segment.type = SegmentType::CONNECTOR;
    segment.connector_id = connector.id;
    segment.action = connector.action;
    segment.from_region = topology.region_path[step];
    segment.to_region = topology.region_path[step + 1];
    segment.corridor_half_width = connector_path_half_width_;

    const bool forward = connector.from == segment.from_region && connector.to == segment.to_region;
    const bool reverse = connector.mode == "two_way" && connector.to == segment.from_region &&
                         connector.from == segment.to_region;
    if (!forward && !reverse) {
      throw std::runtime_error("Connector direction mismatch");
    }

    const auto & wait = forward ? connector.portal_start : connector.portal_end;
    const auto & exit = forward ? connector.portal_end : connector.portal_start;
    segment.wait_pose = makePoseFromPoint(wait, clock_->now());
    segment.exit_pose = makePoseFromPoint(exit, clock_->now());
    const double yaw = std::atan2(exit.y - wait.y, exit.x - wait.x);
    setYaw(segment.wait_pose, yaw);
    setYaw(segment.exit_pose, yaw);

    add_normal(segment.wait_pose, segment.from_region, grid::GoalPolicy::EXACT);
    segment.path = makeConnectorPath(segment.wait_pose, segment);
    appendSegment(plan.full_path, segment.path);
    plan.segments.push_back(segment);
    current = segment.exit_pose;
  }

  add_normal(goal, topology.region_path.back(), grid::GoalPolicy::WITHIN_TOLERANCE);
  return plan;
}

nav_msgs::msg::Path TopologyGlobalPlanner::makeConnectorPath(
  const geometry_msgs::msg::PoseStamped & start, const Segment & segment)
{
  const auto & wait = segment.wait_pose.pose.position;
  const auto & exit = segment.exit_pose.pose.position;
  const double length = std::hypot(exit.x - wait.x, exit.y - wait.y);
  if (length <= 1e-6) {
    throw std::runtime_error("通道长度无效");
  }

  const double ux = (exit.x - wait.x) / length, uy = (exit.y - wait.y) / length;
  const double progress =
    (start.pose.position.x - wait.x) * ux + (start.pose.position.y - wait.y) * uy;
  if (progress < -connector_entry_clearance_ || progress > length + 0.05) {
    throw std::runtime_error("重新规划起点超出已锁定通道的前后边界：通道=" + segment.connector_id +
      "，沿通道位置=" + std::to_string(progress) + "米，允许范围=[" +
      std::to_string(-connector_entry_clearance_) + "," + std::to_string(length + 0.05) +
      "]米。为遵守通道方向约束，本次规划被拒绝");
  }

  nav_msgs::msg::Path path;
  auto current = start;
  if (progress < 0.0) {
    auto entry = makeInnerPlannerPath(current, segment.wait_pose, grid::GoalPolicy::EXACT,
      [this, &segment](const auto & candidate) {
        return pathInsideRegionWithTolerance(candidate, segment.from_region, region_constraint_tolerance_);
      });
    if (!pathInsideRegionWithTolerance(entry, segment.from_region, region_constraint_tolerance_)) {
      throw std::runtime_error("Unsafe committed connector approach");
    }
    appendSegment(path, entry);
    current = segment.wait_pose;
  }

  const Point2D begin{
    wait.x - ux * connector_entry_clearance_, wait.y - uy * connector_entry_clearance_};
  const Point2D end{exit.x, exit.y};
  auto traversal = makeInnerPlannerPath(current, segment.exit_pose, grid::GoalPolicy::EXACT,
    [this, &begin, &end](const auto & candidate) {
      return pathInsideConnectorStrip(candidate, begin, end, connector_path_half_width_);
    });
  appendSegment(path, traversal);

  if (path.poses.empty() || !pathInsideConnectorStrip(path, begin, end, connector_path_half_width_))
  {
    throw std::runtime_error("Unsafe directed connector or exit clearance path");
  }
  return path;
}
}
