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

}  // namespace

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
    node_, name_ + ".enforce_region_constraint", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".region_constraint_tolerance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".blocked_goal_reset_distance", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".blocked_connector_cost", rclcpp::ParameterValue(999.0));
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
  node_->get_parameter(name_ + ".enforce_region_constraint", enforce_region_constraint_);
  node_->get_parameter(name_ + ".region_constraint_tolerance", region_constraint_tolerance_);
  node_->get_parameter(name_ + ".blocked_goal_reset_distance", blocked_goal_reset_distance_);
  node_->get_parameter(name_ + ".blocked_connector_cost", blocked_connector_cost_);
  node_->get_parameter(name_ + ".inner_planner_plugin", inner_planner_plugin_);
  node_->get_parameter(name_ + ".inner_planner_name", inner_planner_name_);

  connector_debug_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "TopoPlanner/connector_debug_markers",
    rclcpp::QoS(1).transient_local().reliable()
  );

  need_action_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "TopoPlanner/need_action",
    rclcpp::QoS(1).transient_local().reliable()
  );

  current_region_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
    "/TopoPlanner/current_region",
    rclcpp::QoS(10)
  );

  need_action_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TopologyGlobalPlanner::publishNeedAction, this));

  current_region_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TopologyGlobalPlanner::publishCurrentRegion, this));

  block_cmd_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/TopoPlanner/block_cmd",
    rclcpp::QoS(1).reliable(),
    std::bind(&TopologyGlobalPlanner::blockCmdCallback, this, std::placeholders::_1));

  try {
    inner_planner_ = inner_planner_loader_.createUniqueInstance(inner_planner_plugin_);
    inner_planner_->configure(parent, inner_planner_name_, tf_, costmap_ros_);
    inner_planner_configured_ = true;
  } catch (const std::exception & e) {
    throw std::runtime_error(
            std::string("Failed to load/configure inner planner '") + inner_planner_plugin_ + "': " + e.what());
  }

  if (use_topology_ && !topology_yaml_.empty()) {
    if (loadTopologyYaml(topology_yaml_)) {            // 从yaml中读取regions connectors 失败回退
      buildGraph();
      publishConnectorDebugMarkers();
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to load topology yaml. Topology layer will be bypassed; inner planner will be used directly.");
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
  need_action_timer_.reset();
  current_region_timer_.reset();
  block_cmd_sub_.reset();
  connector_debug_markers_pub_.reset();
  need_action_pub_.reset();
  current_region_pub_.reset();
  inner_planner_.reset();
  inner_planner_configured_ = false;
  regions_.clear();
  connectors_.clear();
  graph_.clear();
  is_on_connector_ = false;
  active_region_id_.clear();
  active_connector_id_.clear();
  clearPlannedConnector();
  has_last_goal_ = false;
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

void TopologyGlobalPlanner::publishCurrentRegion()
{
  if (!current_region_pub_ || !costmap_ros_) {
    return;
  }

  std_msgs::msg::UInt8 current_region_msg;
  current_region_msg.data = 0;

  geometry_msgs::msg::PoseStamped robot_pose;
  if (costmap_ros_->getRobotPose(robot_pose)) {
    const std::string current_region = findRegion(
      robot_pose.pose.position.x, robot_pose.pose.position.y);
    current_region_msg.data = regionIdToUInt8(current_region);
  }

  current_region_pub_->publish(current_region_msg);
}

// 重要：规划路线
nav_msgs::msg::Path TopologyGlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  if (!inner_planner_) {
    throw std::runtime_error("Inner planner is null in TopologyGlobalPlanner::createPlan");
  }

  const auto start_global = normalizePoseFrame(start);
  const auto goal_global = normalizePoseFrame(goal);

  // 新拓扑搜索前，先根据位置更新 planned -> active 或退回入口的 active 解除。
  updateConnectorEntryLatch(start_global);
  
  if (!use_topology_ || regions_.empty() || connectors_.empty()) {
    // Connector 通过状态已经锁存时，直接规划也不能提前清除 Connector ID。
    if (!is_on_connector_) {
      clearPlannedConnector();
    }
    publishNeedAction();
    return makeInnerPlannerPath(start_global, goal_global);
  }

  bool new_goal = !has_last_goal_;
  if (has_last_goal_) {
    const double goal_change_distance = std::hypot(
      goal_global.pose.position.x - last_goal_.pose.position.x,
      goal_global.pose.position.y - last_goal_.pose.position.y);
    new_goal = goal_change_distance > blocked_goal_reset_distance_;
  }

  if (new_goal) {
    restoreAllConnectorCosts();
  }

  last_goal_ = goal_global;
  has_last_goal_ = true;
  
  // 判断起终点在哪个区域
  const std::string start_region = findRegion(start_global.pose.position.x, start_global.pose.position.y);
  const std::string goal_region = findRegion(goal_global.pose.position.x, goal_global.pose.position.y);

  // // 起点终点有一个不在region内就回退
  // if (start_region.empty() || goal_region.empty()) {
  //   planned_connector_id_.clear();
  //   return fallbackDirectPlan(
  //     start_global, goal_global,
  //     "start or goal is outside all topology regions: start_region='" + start_region +
  //     "', goal_region='" + goal_region + "'");
  // }

  // 起终点在同一个region内且当前不在connector上就回退
  if (start_region == goal_region && !is_on_connector_) {
    clearPlannedConnector();
    publishNeedAction();
    RCLCPP_DEBUG(
      node_->get_logger(), "Start and goal are in the same region '%s'. Using inner planner directly.",
      start_region.c_str());
    return makeInnerPlannerPath(start_global, goal_global);
  }

  // 找不到拓扑可通行路径就回退
  const auto topo_result = searchTopology(start_region, goal_region, start_global, goal_global);
  if (!topo_result.success) {
    // 搜索失败不代表已经通过 Connector，保留正在通过时的 ID 锁存。
    if (!is_on_connector_) {
      clearPlannedConnector();
    }
    publishNeedAction();
    return fallbackDirectPlan(
      start_global, goal_global,
      "no topology route from '" + start_region + "' to '" + goal_region + "'");
  }

  auto update_need_action = [this, &topo_result]() {
    // 一旦进入 Connector，publishNeedAction() 优先发布 active_connector_id_。
    // 即使新目标落回当前 region、拓扑结果不再包含 Connector，也不能覆盖当前 ID。
    if (is_on_connector_) {
      publishNeedAction();
      return;
    }

    // 尚未进入 Connector 时，拓扑路径中的第一个 Connector 就是当前要通过的 Connector。
    clearPlannedConnector();
    if (!topo_result.connector_indices.empty() && topo_result.region_path.size() >= 2) {
      const int connector_index = topo_result.connector_indices.front();
      if (connector_index >= 0 &&
        static_cast<size_t>(connector_index) < connectors_.size())
      {
        const auto & connector = connectors_[static_cast<size_t>(connector_index)];
        const auto & current_region = topo_result.region_path[0];
        const auto & next_region = topo_result.region_path[1];

        if (connector.from == current_region && connector.to == next_region) {
          planned_wait_point_ = connector.portal_start;
          planned_exit_point_ = connector.portal_end;
        } else if (
          connector.mode == "two_way" && connector.to == current_region &&
          connector.from == next_region)
        {
          planned_wait_point_ = connector.portal_end;
          planned_exit_point_ = connector.portal_start;
        } else {
          publishNeedAction();
          return;
        }

        planned_connector_id_ = connector.id;
        planned_next_region_id_ = next_region;
        has_planned_connector_geometry_ = true;
      }
    }

    publishNeedAction();
  };

  // A* 分段规划可能耗时，先发布本轮选中的第一个 Connector ID。
  update_need_action();

  
  std::string region_log;
  for (const auto & r : topo_result.region_path) {
    if (!region_log.empty()) {
      region_log += " -> ";
    }
    region_log += r;
  }
  RCLCPP_INFO(node_->get_logger(), "Topology route: %s", region_log.c_str());
  
  auto path = makePortalOptimizedTopologyPath(start_global, goal_global, topo_result);

  // 路径生成过程中可能进入、完成或放弃 Connector，按最新锁存状态再发布一次。
  update_need_action();

  if (path.poses.empty() && fallback_to_inner_planner_) {  // fallback始终为true的情况
    RCLCPP_WARN(node_->get_logger(), "Segmented topology planning failed. Falling back to direct inner planner from start to goal.");
    // 未进入 Connector 时，回退路径不再经过计划中的 Connector。
    // 如果已经进入，则继续发布 active_connector_id_ 直到真正通过。
    if (!is_on_connector_) {
      clearPlannedConnector();
    }
    publishNeedAction();
    return makeInnerPlannerPath(start_global, goal_global);
  }
  return path;
}
  
}  // namespace topology_global_planner

PLUGINLIB_EXPORT_CLASS(topology_global_planner::TopologyGlobalPlanner, nav2_core::GlobalPlanner)
