#pragma once

#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace topology_global_planner
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Region
{
  std::string id;
  std::string name;
  std::vector<Point2D> polygon;
  Point2D centroid;
};

struct Connector
{
  std::string id;
  std::string from;
  std::string to;
  std::string mode{"two_way"};
  Point2D portal_start;
  Point2D portal_end;
  double cost{1.0};
  std::string action;
};

struct TopologySearchResult
{
  bool success{false};
  std::vector<std::string> region_path;        // 拓扑路径
  std::vector<int> connector_indices;          // connector索引
};


class TopologyGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  TopologyGlobalPlanner();
  ~TopologyGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  ) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
  ) override;

private:
  bool loadTopologyYaml(const std::string & yaml_path);
  void buildGraph();
  bool hasRegion(const std::string & id) const;

  std::string findRegion(double x, double y) const;
  std::string chooseBestRegionByCentroid(double x, double y, const std::vector<std::string> & candidates) const;
  bool pointInPolygon(double x, double y, const std::vector<Point2D> & polygon) const;
  bool pointOnPolygonBoundary(double x, double y, const std::vector<Point2D> & polygon) const;
  bool pointOnPolygonBoundary(double x, double y, const std::vector<Point2D> & polygon, double tolerance) const;
  const Region * getRegionById(const std::string & id) const;
  bool pointInRegionWithTolerance(double x, double y, const std::string & region_id, double tolerance) const;
  bool pathInsideRegionWithTolerance(const nav_msgs::msg::Path & path, const std::string & region_id, double tolerance) const;
  bool pathInsideConnectorStrip(const nav_msgs::msg::Path & path, const Point2D & wait, const Point2D & exit, double half_width) const;
  double distancePointToSegment(const Point2D & p, const Point2D & a, const Point2D & b) const;
  Point2D computeCentroid(const std::vector<Point2D> & polygon) const;


  nav_msgs::msg::Path makePortalOptimizedTopologyPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const TopologySearchResult & topo_result
  );

  TopologySearchResult searchTopology(
    const std::string & start_region,
    const std::string & goal_region,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
  ) const;

  std::vector<int> optimizeConnectorsForRegionPath(
    const std::vector<std::string> & region_path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
  ) const;

  nav_msgs::msg::Path makeInnerPlannerPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
  );

  geometry_msgs::msg::PoseStamped makePoseFromPoint(
    const Point2D & point,
    const rclcpp::Time & stamp) const;

  nav_msgs::msg::Path fallbackDirectPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & reason);

  geometry_msgs::msg::PoseStamped normalizePoseFrame(
    const geometry_msgs::msg::PoseStamped & pose) const;

  void setYaw(geometry_msgs::msg::PoseStamped & pose, double yaw) const;
  void appendSegment(nav_msgs::msg::Path & full_path, const nav_msgs::msg::Path & segment) const;
  double euclidean(double x1, double y1, double x2, double y2) const;

  struct DirectedEdge
  {
    std::string to;
    int connector_index{-1};
    double cost{1.0};
  };

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("TopologyGlobalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr connector_debug_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr need_action_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr current_region_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr block_cmd_sub_;
  rclcpp::TimerBase::SharedPtr need_action_timer_;
  rclcpp::TimerBase::SharedPtr current_region_timer_;
  void publishConnectorDebugMarkers();
  void publishNeedAction();
  void publishCurrentRegion();
  void clearPlannedConnector();
  void updateConnectorEntryLatch(const geometry_msgs::msg::PoseStamped & current_pose);
  void blockCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void restoreAllConnectorCosts();

  std::string name_;
  std::string global_frame_;

  std::string topology_yaml_;
  bool use_topology_{true};
  bool fallback_to_inner_planner_{true};
  bool allow_nearest_region_fallback_{false};
  double region_boundary_tolerance_{0.03};
  double transform_tolerance_{0.1};
  double duplicate_pose_tolerance_{0.02};
  double max_nearest_region_distance_{1.0};
  bool enforce_region_constraint_{true};
  double region_constraint_tolerance_{0.15};
  double connector_path_half_width_{0.8};
  double blocked_goal_reset_distance_{0.50};
  double blocked_connector_cost_{999.0};

  std::string inner_planner_plugin_{"nav2_navfn_planner/NavfnPlanner"};
  std::string inner_planner_name_{"InnerPlanner"};
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> inner_planner_loader_;
  nav2_core::GlobalPlanner::Ptr inner_planner_;
  bool inner_planner_configured_{false};
  bool inner_planner_active_{false};

  std::vector<Region> regions_;
  std::vector<Connector> connectors_;
  std::unordered_map<std::string, std::vector<DirectedEdge>> graph_;

  Point2D active_wait_point_;
  Point2D active_exit_point_;
  bool is_on_connector_{false};
  std::string active_region_id_;
  std::string active_connector_id_;
  std::string planned_connector_id_;
  Point2D planned_wait_point_;
  Point2D planned_exit_point_;
  std::string planned_next_region_id_;
  bool has_planned_connector_geometry_{false};

  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_last_goal_{false};
};

}  // namespace topology_global_planner
