#include "topology_global_planner/topology_global_planner.hpp"

#include <cmath>

#include "geometry_msgs/msg/point.hpp"

#include "visualization_msgs/msg/marker.hpp"

namespace topology_global_planner
{

namespace // 此service用的一些函数，无需声明
{

Point2D subtractPoints(const Point2D & lhs, const Point2D & rhs)
{
  return Point2D{lhs.x - rhs.x, lhs.y - rhs.y};
}

double pointNorm(const Point2D & point)
{
  return std::hypot(point.x, point.y);
}

Point2D normalizePoint(const Point2D & point)
{
  const double norm = pointNorm(point);
  if (norm <= 1e-9) {
    return Point2D{0.0, 0.0};
  }

  return Point2D{point.x / norm, point.y / norm};
}

Point2D connectorTraversalDirection(const Connector & connector, const std::string & approach_region_id)
{
  Point2D direction = normalizePoint(subtractPoints(connector.portal_end, connector.portal_start));

  if (approach_region_id == connector.to) {
    direction.x *= -1.0;
    direction.y *= -1.0;
  }

  return direction;
}

geometry_msgs::msg::Point toPointMsg(const Point2D & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = 0.0;
  return msg;
}

}  // namespace


// 算洞口中点
geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::computeConnectorCenterPose(
  const Connector & connector,
  const rclcpp::Time & stamp) const
{
  Point2D center;
  center.x = 0.5 * (connector.portal_start.x + connector.portal_end.x);      // 取connector中点，转成nav2可用pose
  center.y = 0.5 * (connector.portal_start.y + connector.portal_end.y);
  return makePoseFromPoint(center, stamp);
}


// 算洞前缩头点：基于洞口中点，往“来车方向”反退一段距离，得到一个让车先停下来、准备缩头的点。
geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::computeConnectorWaitPose(
  const Connector & connector,
  const std::string & approach_region_id,
  const rclcpp::Time & stamp) const
{
  auto connector_pose = computeConnectorCenterPose(connector, stamp);

  Point2D portal_center{
    0.5 * (connector.portal_start.x + connector.portal_end.x),
    0.5 * (connector.portal_start.y + connector.portal_end.y)
  };
  Point2D traversal_direction = connectorTraversalDirection(connector, approach_region_id);

  Point2D wait_point{
    portal_center.x - traversal_direction.x * connector.wait_offset,
    portal_center.y - traversal_direction.y * connector.wait_offset};
  auto wait_pose = makePoseFromPoint(wait_point, stamp);
  const double yaw = std::atan2(traversal_direction.y, traversal_direction.x);
  setYaw(wait_pose, yaw);

  return wait_pose;
}


// 算洞后抬头点
geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::computeConnectorExitPose(
  const Connector & connector,
  const std::string & approach_region_id,
  const rclcpp::Time & stamp) const
{
  auto connector_pose = computeConnectorCenterPose(connector, stamp);

  Point2D portal_center{
    0.5 * (connector.portal_start.x + connector.portal_end.x),
    0.5 * (connector.portal_start.y + connector.portal_end.y)};
  Point2D traversal_direction = connectorTraversalDirection(connector, approach_region_id);

  Point2D exit_point{
    portal_center.x + traversal_direction.x * connector.exit_offset,
    portal_center.y + traversal_direction.y * connector.exit_offset};
  auto exit_pose = makePoseFromPoint(exit_point, stamp);
  const double yaw = std::atan2(traversal_direction.y, traversal_direction.x);
  setYaw(exit_pose, yaw);

  return exit_pose;
}

void TopologyGlobalPlanner::handleQueryTopologyRoute(
  const std::shared_ptr<topology_global_planner::srv::QueryTopologyRoute::Request> request,
  std::shared_ptr<topology_global_planner::srv::QueryTopologyRoute::Response> response)
{
  publishConnectorDebugMarkers();

  response->success = false;
  response->need_action = false;

  const auto start = normalizePoseFrame(request->start);
  const auto goal = normalizePoseFrame(request->goal);
  response->final_goal = goal;

  if (!use_topology_ || regions_.empty() || connectors_.empty()) {
    response->success = true;
    response->message = "Topology disabled. Direct navigation.";
    return;
  }

  const auto start_region = findRegion(start.pose.position.x, start.pose.position.y);
  const auto goal_region = findRegion(goal.pose.position.x, goal.pose.position.y);
  response->from_region = start_region;
  response->to_region = goal_region;

  RCLCPP_WARN(
    logger_,
    "[TopoQuery] start=(%.2f, %.2f), goal=(%.2f, %.2f), start_region='%s', goal_region='%s'",
    start.pose.position.x,
    start.pose.position.y,
    goal.pose.position.x,
    goal.pose.position.y,
    start_region.c_str(),
    goal_region.c_str());

  if (start_region.empty() || goal_region.empty()) {
    if (fallback_to_inner_planner_) {
      response->success = true;
      response->message = "Start or goal is outside topology regions. Direct navigation fallback.";
      RCLCPP_WARN(logger_, "[TopoQuery] need_action=0: %s", response->message.c_str());
    } else {
      response->message = "Start or goal is outside topology regions.";
    }
    return;
  }

  if (start_region == goal_region) {
    response->success = true;
    response->message = "Start and goal are in same region.";
    RCLCPP_WARN(logger_, "[TopoQuery] need_action=0: %s", response->message.c_str());
    return;
  }

  const auto topo_result = searchTopology(start_region, goal_region);
  if (!topo_result.success) {
    if (fallback_to_inner_planner_) {
      response->success = true;
      response->message = "No topology route found. Direct navigation fallback.";
      RCLCPP_WARN(logger_, "[TopoQuery] need_action=0: %s", response->message.c_str());
    } else {
      response->message = "No topology route found.";
    }
    return;
  }

  const auto stamp = clock_ ? clock_->now() : node_->now();
  for (size_t i = 0; i < topo_result.connector_indices.size(); ++i) {
    const int idx = topo_result.connector_indices[i];
    if (idx < 0 || static_cast<size_t>(idx) >= connectors_.size()) {
      continue;
    }

    const auto & connector = connectors_[static_cast<size_t>(idx)];
    RCLCPP_WARN(
      logger_,
      "[TopoQuery] route connector: id='%s', from='%s', to='%s', has_action=%d, action_type='%s'",
      connector.id.c_str(),
      connector.from.c_str(),
      connector.to.c_str(),
      connector.has_action,
      connector.action_type.c_str());
    if (!connector.has_action) {
      continue;
    }

    const std::string approach_region_id =
      i < topo_result.region_path.size() ? topo_result.region_path[i] : start_region;
    const std::string target_region_id =
      i + 1 < topo_result.region_path.size() ? topo_result.region_path[i + 1] : goal_region;
    response->success = true;
    response->need_action = true;
    response->connector_id = connector.id;
    response->action_type = connector.action_type;
    response->from_region = approach_region_id;
    response->to_region = target_region_id;
    response->wait_pose = computeConnectorWaitPose(connector, approach_region_id, stamp);
    response->connector_pose = computeConnectorCenterPose(connector, stamp);
    response->exit_pose = computeConnectorExitPose(connector, approach_region_id, stamp);
    response->down_timeout = connector.down_timeout;
    response->up_monitor_timeout = connector.up_monitor_timeout;
    response->cancel_policy = connector.cancel_policy;
    response->message = "Action connector found.";
    RCLCPP_WARN(
      logger_,
      "[TopoQuery] need_action=1: connector='%s', action='%s', from='%s', to='%s'",
      response->connector_id.c_str(),
      response->action_type.c_str(),
      response->from_region.c_str(),
      response->to_region.c_str());
    return;
  }

  response->success = true;
  response->message = "No action connector in route.";
  RCLCPP_WARN(logger_, "[TopoQuery] need_action=0: %s", response->message.c_str());
}

void TopologyGlobalPlanner::publishConnectorDebugMarkers()
{
  if (!connector_debug_markers_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto stamp = clock_ ? clock_->now() : node_->now();
  int marker_id = 0;

  auto make_marker = [&](const std::string & ns, int type) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = stamp;
      marker.ns = ns;
      marker.id = marker_id++;
      marker.type = type;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.lifetime = rclcpp::Duration::from_seconds(0.0);
      return marker;
    };

  for (const auto & connector : connectors_) {
    const auto center_pose = computeConnectorCenterPose(connector, stamp);
    const auto wait_from_pose = computeConnectorWaitPose(connector, connector.from, stamp);
    const auto wait_to_pose = computeConnectorWaitPose(connector, connector.to, stamp);
    const auto exit_from_pose = computeConnectorExitPose(connector, connector.from, stamp);
    const auto exit_to_pose = computeConnectorExitPose(connector, connector.to, stamp);

    auto portal_marker = make_marker("connector_portal", visualization_msgs::msg::Marker::LINE_STRIP);
    portal_marker.scale.x = 0.08;
    portal_marker.color.a = 1.0F;
    portal_marker.color.r = 0.1F;
    portal_marker.color.g = 0.9F;
    portal_marker.color.b = 0.1F;
    portal_marker.points.push_back(toPointMsg(connector.portal_start));
    portal_marker.points.push_back(toPointMsg(connector.portal_end));
    marker_array.markers.push_back(portal_marker);

    auto center_marker = make_marker("connector_center", visualization_msgs::msg::Marker::SPHERE);
    center_marker.scale.x = 0.22;
    center_marker.scale.y = 0.22;
    center_marker.scale.z = 0.22;
    center_marker.color.a = 1.0F;
    center_marker.color.r = 1.0F;
    center_marker.color.g = 0.9F;
    center_marker.color.b = 0.1F;
    center_marker.pose = center_pose.pose;
    marker_array.markers.push_back(center_marker);

    auto wait_from_marker = make_marker("connector_wait_from", visualization_msgs::msg::Marker::SPHERE);
    wait_from_marker.scale.x = 0.18;
    wait_from_marker.scale.y = 0.18;
    wait_from_marker.scale.z = 0.18;
    wait_from_marker.color.a = 1.0F;
    wait_from_marker.color.r = 0.95F;
    wait_from_marker.color.g = 0.2F;
    wait_from_marker.color.b = 0.2F;
    wait_from_marker.pose = wait_from_pose.pose;
    marker_array.markers.push_back(wait_from_marker);

    auto wait_to_marker = make_marker("connector_wait_to", visualization_msgs::msg::Marker::SPHERE);
    wait_to_marker.scale.x = 0.18;
    wait_to_marker.scale.y = 0.18;
    wait_to_marker.scale.z = 0.18;
    wait_to_marker.color.a = 1.0F;
    wait_to_marker.color.r = 0.2F;
    wait_to_marker.color.g = 0.4F;
    wait_to_marker.color.b = 1.0F;
    wait_to_marker.pose = wait_to_pose.pose;
    marker_array.markers.push_back(wait_to_marker);

    auto direction_from_marker = make_marker("connector_direction_from", visualization_msgs::msg::Marker::ARROW);
    direction_from_marker.scale.x = 0.06;
    direction_from_marker.scale.y = 0.12;
    direction_from_marker.scale.z = 0.16;
    direction_from_marker.color.a = 1.0F;
    direction_from_marker.color.r = 1.0F;
    direction_from_marker.color.g = 0.2F;
    direction_from_marker.color.b = 0.2F;
    direction_from_marker.points.push_back(wait_from_pose.pose.position);
    direction_from_marker.points.push_back(exit_from_pose.pose.position);
    marker_array.markers.push_back(direction_from_marker);

    auto direction_to_marker = make_marker("connector_direction_to", visualization_msgs::msg::Marker::ARROW);
    direction_to_marker.scale.x = 0.06;
    direction_to_marker.scale.y = 0.12;
    direction_to_marker.scale.z = 0.16;
    direction_to_marker.color.a = 1.0F;
    direction_to_marker.color.r = 0.2F;
    direction_to_marker.color.g = 0.4F;
    direction_to_marker.color.b = 1.0F;
    direction_to_marker.points.push_back(wait_to_pose.pose.position);
    direction_to_marker.points.push_back(exit_to_pose.pose.position);
    marker_array.markers.push_back(direction_to_marker);
  }

  connector_debug_markers_pub_->publish(marker_array);
}

}  // namespace topology_global_planner
