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

TopologyGlobalPlanner::TopologyGlobalPlanner() : inner_planner_loader_("nav2_core", "nav2_core::GlobalPlanner")
{
}

void TopologyGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
){
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock lifecycle node in TopologyGlobalPlanner::configure");
  }

  switch_route_mode_srv_ = node_->create_service<topology_global_planner::srv::SwitchRouteMode>(
    "/switch_route_mode",
    std::bind(&TopologyGlobalPlanner::switchRouteModeCallback, this, std::placeholders::_1, std::placeholders::_2)
  ); // request和response各要一个占位符

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  clock_ = node_->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".topology_yaml", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_topology", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fallback_to_inner_planner", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_nearest_region_fallback", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".region_boundary_tolerance", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".duplicate_pose_tolerance", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_nearest_region_distance", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".portal_sample_count", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".portal_adaptive_sampling", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".portal_refine_sample_count", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".enforce_region_constraint", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".region_constraint_tolerance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".inner_planner_plugin", rclcpp::ParameterValue(inner_planner_plugin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".inner_planner_name", rclcpp::ParameterValue(inner_planner_name_));

  node_->get_parameter(name_ + ".topology_yaml", topology_yaml_);
  node_->get_parameter(name_ + ".use_topology", use_topology_);
  node_->get_parameter(name_ + ".fallback_to_inner_planner", fallback_to_inner_planner_);
  node_->get_parameter(name_ + ".allow_nearest_region_fallback", allow_nearest_region_fallback_);
  node_->get_parameter(name_ + ".region_boundary_tolerance", region_boundary_tolerance_);
  node_->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node_->get_parameter(name_ + ".duplicate_pose_tolerance", duplicate_pose_tolerance_);
  node_->get_parameter(name_ + ".max_nearest_region_distance", max_nearest_region_distance_);
  node_->get_parameter(name_ + ".portal_sample_count", portal_sample_count_);
  node_->get_parameter(name_ + ".portal_adaptive_sampling", portal_adaptive_sampling_);
  node_->get_parameter(name_ + ".portal_refine_sample_count", portal_refine_sample_count_);
  node_->get_parameter(name_ + ".enforce_region_constraint", enforce_region_constraint_);
  node_->get_parameter(name_ + ".region_constraint_tolerance", region_constraint_tolerance_);
  node_->get_parameter(name_ + ".inner_planner_plugin", inner_planner_plugin_);
  node_->get_parameter(name_ + ".inner_planner_name", inner_planner_name_);

  
  try {
    inner_planner_ = inner_planner_loader_.createUniqueInstance(inner_planner_plugin_);
    inner_planner_->configure(parent, inner_planner_name_, tf_, costmap_ros_);
    inner_planner_configured_ = true;
  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Failed to load/configure inner planner '") +
            inner_planner_plugin_ + "': " + e.what());
  }

  if (use_topology_ && !topology_yaml_.empty()) {
    if (loadTopologyYaml(topology_yaml_)) {
      buildGraph();
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "Failed to load topology yaml. Topology layer will be bypassed; inner planner will be used directly.");
      use_topology_ = false;
    }
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "Topology disabled or topology_yaml is empty. Inner planner will be used directly.");
  }
}

void TopologyGlobalPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up TopologyGlobalPlanner");
  if (inner_planner_configured_ && inner_planner_) {
    if (inner_planner_active_) {
      inner_planner_->deactivate();
      inner_planner_active_ = false;
    }
    inner_planner_->cleanup();
  }
  inner_planner_.reset();
  inner_planner_configured_ = false;
  regions_.clear();
  connectors_.clear();
  graph_.clear();
}

void TopologyGlobalPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating TopologyGlobalPlanner");
  if (inner_planner_configured_ && inner_planner_ && !inner_planner_active_) {
    inner_planner_->activate();
    inner_planner_active_ = true;
  }
}

void TopologyGlobalPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating TopologyGlobalPlanner");
  if (inner_planner_configured_ && inner_planner_ && inner_planner_active_) {
    inner_planner_->deactivate();
    inner_planner_active_ = false;
  }
}

// 重要：规划路线
nav_msgs::msg::Path TopologyGlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  if (!inner_planner_) {
    throw std::runtime_error("Inner planner is null in TopologyGlobalPlanner::createPlan");
  }

  const auto start_global = normalizePoseFrame(start);
  const auto goal_global = normalizePoseFrame(goal);

  if (!use_topology_ || regions_.empty() || connectors_.empty()) {
    return makeInnerPlannerPath(start_global, goal_global);
  }

  // 判断起终点在哪个区域
  const std::string start_region = findRegion(start_global.pose.position.x, start_global.pose.position.y);
  const std::string goal_region = findRegion(goal_global.pose.position.x, goal_global.pose.position.y);

  // 起点终点有一个不在region内就回退
  if (start_region.empty() || goal_region.empty()) {
    return fallbackDirectPlan(
      start_global, goal_global,
      "start or goal is outside all topology regions: start_region='" + start_region +
      "', goal_region='" + goal_region + "'");
  }

  // 起终点在同一个region内就回退
  if (start_region == goal_region) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Start and goal are in the same region '%s'. Using inner planner directly.",
      start_region.c_str());
    return makeInnerPlannerPath(start_global, goal_global);
  }

  // 找不到拓扑可通行路径就回退
  const auto topo_result = searchTopology(start_region, goal_region);
  if (!topo_result.success) {
    return fallbackDirectPlan(
      start_global, goal_global,
      "no topology route from '" + start_region + "' to '" + goal_region + "'");
  }

  std::string region_log;
  for (const auto & r : topo_result.region_path) {
    if (!region_log.empty()) {
      region_log += " -> ";
    }
    region_log += r;
  }
  RCLCPP_INFO(node_->get_logger(), "Topology route: %s", region_log.c_str());

  auto path = makePortalOptimizedTopologyPath(start_global, goal_global, topo_result);

  if (path.poses.empty() && fallback_to_inner_planner_) {
    RCLCPP_WARN(
      node_->get_logger(), "Segmented topology planning failed. Falling back to direct inner planner from start to goal.");
    return makeInnerPlannerPath(start_global, goal_global);
  }

  return path;
}


/*-------------------------SwitchRouteMode server---------------------------*/
// server callback：切换cost mode
void TopologyGlobalPlanner::switchRouteModeCallback(
  const std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Request> request,
  std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Response> response
){
  (void)request; // request为空，占位声明，消除编译器的“未使用参数”警告
  route_mode_ = 1 - route_mode_; //将要切换的mode转化成与当前不同的mode

  //应用更改
  applyModeCost();

  response->success = true;
}

void TopologyGlobalPlanner::applyModeCost(){
  if(route_mode_ == 0){
    RCLCPP_INFO(node_->get_logger(), "mode: 0");
  } else {
    RCLCPP_INFO(node_->get_logger(), "mode: 1");
  }
}

}  // namespace topology_global_planner

PLUGINLIB_EXPORT_CLASS(topology_global_planner::TopologyGlobalPlanner, nav2_core::GlobalPlanner)
