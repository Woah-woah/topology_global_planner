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

namespace
{

uint8_t regionIdToUInt8(const std::string & region_id)
{
  if (region_id.size() < 2 || region_id.front() != 'R') {
    return 0;
  }

  unsigned int region_number = 0;
  for (size_t i = 1; i < region_id.size(); ++i) {
    const char character = region_id[i];
    if (character < '0' || character > '9') {
      return 0;
    }

    const unsigned int digit = static_cast<unsigned int>(character - '0');
    if (region_number > (255U - digit) / 10U) {
      return 0;
    }
    region_number = region_number * 10U + digit;
  }

  if (region_number == 0U) {
    return 0;
  }
  return static_cast<uint8_t>(region_number);
}

}

namespace topology_global_planner
{

void TopologyGlobalPlanner::initialize(rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf,
  map_server::MapView * costmap,
  std::string global_frame,
  std::string robot_base_frame,
  std::string name, double clearance_radius)
{
  if (!node || !tf || !costmap || global_frame.empty() || robot_base_frame.empty()) {
    throw std::invalid_argument("拓扑规划器初始化参数无效");
  }

  node_ = std::move(node);
  tf_ = std::move(tf);
  costmap_ = costmap;
  global_frame_ = std::move(global_frame);
  robot_base_frame_ = std::move(robot_base_frame);
  name_ = std::move(name);
  clock_ = node_->get_clock();
  logger_ = node_->get_logger();

  // 参数统一加上规划器名称前缀
  auto declare = [this](const std::string & key, const rclcpp::ParameterValue & default_value) {
    const auto full_name = name_ + "." + key;
    if (!node_->has_parameter(full_name)) {
      node_->declare_parameter(full_name, default_value);
    }
  };

  declare("topology_yaml", rclcpp::ParameterValue(std::string("")));
  declare("use_topology", rclcpp::ParameterValue(true));
  declare("allow_nearest_region_fallback", rclcpp::ParameterValue(false));
  declare("region_boundary_tolerance", rclcpp::ParameterValue(0.03));
  declare("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare("max_nearest_region_distance", rclcpp::ParameterValue(1.0));
  declare("region_constraint_tolerance", rclcpp::ParameterValue(0.15));
  declare("connector_path_half_width", rclcpp::ParameterValue(0.8));
  declare("connector_entry_clearance", rclcpp::ParameterValue(0.5));

  // 栅格规划参数
  declare("grid.tolerance", rclcpp::ParameterValue(0.2));
  declare("grid.use_astar", rclcpp::ParameterValue(false));
  declare("grid.allow_unknown", rclcpp::ParameterValue(false));
  declare("grid.use_final_approach_orientation", rclcpp::ParameterValue(true));

  node_->get_parameter(name_ + ".topology_yaml", topology_yaml_);
  node_->get_parameter(name_ + ".use_topology", use_topology_);
  node_->get_parameter(name_ + ".allow_nearest_region_fallback", allow_nearest_region_fallback_);
  node_->get_parameter(name_ + ".region_boundary_tolerance", region_boundary_tolerance_);
  node_->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node_->get_parameter(name_ + ".max_nearest_region_distance", max_nearest_region_distance_);
  node_->get_parameter(name_ + ".region_constraint_tolerance", region_constraint_tolerance_);
  node_->get_parameter(name_ + ".connector_path_half_width", connector_path_half_width_);
  node_->get_parameter(name_ + ".connector_entry_clearance", connector_entry_clearance_);
  if (!std::isfinite(connector_entry_clearance_) || connector_entry_clearance_ <= 0.0 ||
      !std::isfinite(connector_path_half_width_) || connector_path_half_width_ <= 0.0 ||
      !std::isfinite(region_constraint_tolerance_) || region_constraint_tolerance_ < 0.0)
  {
    throw std::invalid_argument("拓扑通道宽度或区域容差无效");
  }

  grid::GridPlanner::Options options;
  options.clearance_radius = clearance_radius;
  node_->get_parameter(name_ + ".grid.tolerance", options.tolerance);
  node_->get_parameter(name_ + ".grid.use_astar", options.use_astar);
  node_->get_parameter(name_ + ".grid.allow_unknown", options.allow_unknown);
  node_->get_parameter(
    name_ + ".grid.use_final_approach_orientation", options.use_final_approach_orientation);

  grid_planner_ = std::make_unique<grid::GridPlanner>(costmap_, global_frame_, clock_, options);
  RCLCPP_INFO(logger_, "栅格规划器：搜索方式=%s，路径提取=连续势场梯度，势场传播=快速行进法",
    options.use_astar ? "A*" : "Dijkstra");

  connector_debug_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "TopoPlanner/connector_debug_markers", rclcpp::QoS(1).transient_local().reliable());

  current_region_pub_ =
    node_->create_publisher<std_msgs::msg::UInt8>("/TopoPlanner/current_region", rclcpp::QoS(10));

  current_region_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TopologyGlobalPlanner::publishCurrentRegion, this));

  if (use_topology_) {
    if (topology_yaml_.empty() || !loadTopologyYaml(topology_yaml_) || regions_.empty() ||
        connectors_.empty())
    {
      throw std::runtime_error("已启用拓扑规划，但拓扑配置无效");
    }
    buildGraph();
    publishConnectorDebugMarkers();
  }
}

TopologyGlobalPlanner::~TopologyGlobalPlanner()
{
  current_region_timer_.reset();
  grid_planner_.reset();
}

void TopologyGlobalPlanner::publishCurrentRegion()
{
  if (!current_region_pub_ || !tf_) {
    return;
  }

  std_msgs::msg::UInt8 msg;
  msg.data = 0;

  try {
    const auto transform = tf_->lookupTransform(global_frame_,
      robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));

    const auto & position = transform.transform.translation;
    const std::string region = findRegion(position.x, position.y);

    msg.data = regionIdToUInt8(region);

  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "无法取得机器人当前位置：%s", e.what());
  }

  current_region_pub_->publish(msg);
}

TopologyPlan TopologyGlobalPlanner::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::optional<Segment> & active_connector,
  std::function<bool()> cancel_checker)
{
  if (!grid_planner_) {
    throw std::runtime_error("栅格规划器尚未初始化");
  }

  cancel_checker_ = cancel_checker ? std::move(cancel_checker) : []() {
    return false;
  };
  const auto start_global = normalizePoseFrame(start);
  const auto goal_global = normalizePoseFrame(goal);

  TopologyPlan prefix;
  auto route_start = start_global;
  std::string start_region;

  if (active_connector) {
    // 已进入的通道先按原方向走到出口
    // 换目标和重规划都不能绕过这一段
    const auto & active = *active_connector;
    const auto it = std::find_if(connectors_.begin(), connectors_.end(), [&](const Connector & c) {
      return c.id == active.connector_id;
    });
    if (it == connectors_.end() || active.type != SegmentType::CONNECTOR) {
      throw std::runtime_error("当前锁定的通道不存在");
    }

    const bool forward = it->from == active.from_region && it->to == active.to_region;
    const bool reverse =
      it->mode == "two_way" && it->to == active.from_region && it->from == active.to_region;
    if (!forward && !reverse) {
      throw std::runtime_error("当前锁定通道的通行方向无效");
    }

    const auto & wait = forward ? it->portal_start : it->portal_end;
    const auto & exit = forward ? it->portal_end : it->portal_start;

    auto segment = active;
    segment.wait_pose = makePoseFromPoint(wait, clock_->now());
    segment.exit_pose = makePoseFromPoint(exit, clock_->now());
    const double yaw = std::atan2(exit.y - wait.y, exit.x - wait.x);
    setYaw(segment.wait_pose, yaw);
    setYaw(segment.exit_pose, yaw);

    segment.path = makeConnectorPath(start_global, segment);
    prefix.segments.push_back(segment);
    appendSegment(prefix.full_path, segment.path);
    route_start = segment.exit_pose;
    start_region = segment.to_region;
  } else {
    start_region = findRegion(route_start.pose.position.x, route_start.pose.position.y);
  }

  if (!use_topology_) {
    auto path = makeInnerPlannerPath(route_start, goal_global, grid::GoalPolicy::WITHIN_TOLERANCE);
    Segment segment;
    segment.path = path;
    prefix.segments.push_back(std::move(segment));
    appendSegment(prefix.full_path, path);
    return prefix;
  }

  const auto goal_region = findRegion(goal_global.pose.position.x, goal_global.pose.position.y);
  if (start_region.empty() || goal_region.empty()) {
    throw std::runtime_error("起点或目标点不在任何拓扑区域内");
  }

  const auto topology = searchTopology(start_region, goal_region, route_start, goal_global);
  if (!topology.success) {
    throw std::runtime_error("按允许的通行方向找不到拓扑路线");
  }

  auto suffix = makePortalOptimizedTopologyPath(route_start, goal_global, topology);
  appendSegment(prefix.full_path, suffix.full_path);
  prefix.segments.insert(prefix.segments.end(), suffix.segments.begin(), suffix.segments.end());
  return prefix;
}
}
