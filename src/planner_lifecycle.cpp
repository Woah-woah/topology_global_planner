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

}  // namespace

namespace topology_global_planner
{

void TopologyGlobalPlanner::initialize(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf,
  map_server::Costmap2D * costmap,
  std::string global_frame,
  std::string robot_base_frame,
  std::string name)
{
  if (!node || !tf || !costmap || global_frame.empty() || robot_base_frame.empty()) {
    throw std::invalid_argument("TopologyGlobalPlanner: invalid initialization arguments");}

  node_ = std::move(node);
  tf_ = std::move(tf);
  costmap_ = costmap;
  global_frame_ = std::move(global_frame);
  robot_base_frame_ = std::move(robot_base_frame);
  name_ = std::move(name);
  clock_ = node_->get_clock();
  logger_ = node_->get_logger();

  // 声明参数的函数
  auto declare = [this](
      const std::string & key,
      const rclcpp::ParameterValue & default_value)
  {
    const auto full_name = name_ + "." + key;
    if (!node_->has_parameter(full_name)) {
      node_->declare_parameter(full_name, default_value);
    }
  };

  declare("topology_yaml", rclcpp::ParameterValue(std::string("")));
  declare("use_topology", rclcpp::ParameterValue(true));
  declare("fallback_to_inner_planner", rclcpp::ParameterValue(true));
  declare("allow_nearest_region_fallback", rclcpp::ParameterValue(false));
  declare("region_boundary_tolerance", rclcpp::ParameterValue(0.03));
  declare("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare("duplicate_pose_tolerance", rclcpp::ParameterValue(0.02));
  declare("max_nearest_region_distance", rclcpp::ParameterValue(1.0));
  declare("enforce_region_constraint", rclcpp::ParameterValue(true));
  declare("region_constraint_tolerance", rclcpp::ParameterValue(0.15));
  declare("blocked_goal_reset_distance", rclcpp::ParameterValue(0.50));
  declare("blocked_connector_cost", rclcpp::ParameterValue(999.0));

  // 独立 GridPlanner 参数
  declare("grid.tolerance", rclcpp::ParameterValue(0.2));
  declare("grid.use_astar", rclcpp::ParameterValue(false));
  declare("grid.allow_unknown", rclcpp::ParameterValue(true));
  declare("grid.use_final_approach_orientation", rclcpp::ParameterValue(true));

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

  grid::GridPlanner::Options options;
  node_->get_parameter(name_ + ".grid.tolerance", options.tolerance);
  node_->get_parameter(name_ + ".grid.use_astar", options.use_astar);
  node_->get_parameter(name_ + ".grid.allow_unknown", options.allow_unknown);
  node_->get_parameter(name_ + ".grid.use_final_approach_orientation", options.use_final_approach_orientation);

  grid_planner_ = std::make_unique<grid::GridPlanner>(costmap_, global_frame_, clock_, options);
  RCLCPP_INFO(logger_, "Standalone GridPlanner initialized");

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

TopologyGlobalPlanner::~TopologyGlobalPlanner()
{
  need_action_timer_.reset();
  current_region_timer_.reset();
  block_cmd_sub_.reset();
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
    const auto transform = tf_->lookupTransform(
      global_frame_,
      robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));

    const auto & position = transform.transform.translation;
    const std::string region = findRegion(position.x, position.y);

    msg.data = regionIdToUInt8(region);

  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Failed to get robot pose: %s", e.what());
  }

  current_region_pub_->publish(msg);
}

// 重要：规划路线
nav_msgs::msg::Path TopologyGlobalPlanner::plan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  if (!grid_planner_) {throw std::runtime_error("GridPlanner is not initialized");}
    cancel_checker_ = cancel_checker ? std::move(cancel_checker) : std::function<bool()>([]() { return false; });

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

  // 已经进入 Connector 后，后续拓扑必须从当前 Connector 的出口 Region 开始。
  // 机器人当下可能仍被 findRegion() 判在入口 Region，但路径会先强制补到 exit。
  std::string topology_start_region = start_region;
  auto topology_start_pose = start_global;
  if (is_on_connector_ && !active_region_id_.empty()) {
    topology_start_region = active_region_id_;
    topology_start_pose.pose.position.x = active_exit_point_.x;
    topology_start_pose.pose.position.y = active_exit_point_.y;
  }

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
  const auto topo_result = searchTopology(
    topology_start_region, goal_region, topology_start_pose, goal_global);
  if (!topo_result.success) {
    // 搜索失败不代表已经通过 Connector，保留正在通过时的 ID 锁存。
    if (!is_on_connector_) {
      clearPlannedConnector();
    }
    publishNeedAction();
    return fallbackDirectPlan(
      start_global, goal_global,
      "no topology route from '" + topology_start_region + "' to '" + goal_region + "'");
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

  // 路径生成过程中可能进入或完成 Connector。此时只发布当前状态，
  // 不再从本轮的旧 topo_result 首段重建 planned connector。
  publishNeedAction();

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
