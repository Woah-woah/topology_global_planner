#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "map_server/costmap_2d.hpp"
#include "topology_global_planner/grid/grid_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "topology_global_planner/topology_plan.hpp"
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
  std::vector<std::string> region_path; // 依次经过的区域
  std::vector<int> connector_indices;   // 每次区域转移使用的通道索引
};

class TopologyGlobalPlanner
{
public:
  TopologyGlobalPlanner() = default;
  ~TopologyGlobalPlanner();

  void initialize(rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf,
    map_server::Costmap2D * costmap,
    std::string global_frame,
    std::string robot_base_frame,
    std::string name = "TopoPlanner");

  TopologyPlan plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::optional<Segment> & active_connector,
    std::function<bool()> cancel_checker = []() {
      return false;
    });

  // 复用当前拓扑配置复检平滑结果，不重新搜索或改变通道选择。
  bool validateSmoothedPlan(const TopologyPlan & reference, const TopologyPlan & candidate) const;

private:
  // 配置读取与区域查询
  bool loadTopologyYaml(const std::string & yaml_path);
  void buildGraph();

  std::string findRegion(double x, double y) const;
  std::string chooseBestRegionByCentroid(
    double x, double y, const std::vector<std::string> & candidates) const;
  bool pointInPolygon(double x, double y, const std::vector<Point2D> & polygon) const;
  bool pointOnPolygonBoundary(
    double x, double y, const std::vector<Point2D> & polygon, double tolerance) const;
  const Region * getRegionById(const std::string & id) const;
  bool pathInsideRegionWithTolerance(
    const nav_msgs::msg::Path & path, const std::string & region_id, double tolerance) const;
  bool pathInsideConnectorStrip(const nav_msgs::msg::Path & path,
    const Point2D & wait,
    const Point2D & exit,
    double half_width) const;
  double distancePointToSegment(const Point2D & p, const Point2D & a, const Point2D & b) const;
  Point2D computeCentroid(const std::vector<Point2D> & polygon) const;

  // 路线搜索与分段路径生成
  TopologyPlan makePortalOptimizedTopologyPath(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const TopologySearchResult & topo_result);

  TopologySearchResult searchTopology(const std::string & start_region,
    const std::string & goal_region,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  std::vector<int> optimizeConnectorsForRegionPath(const std::vector<std::string> & region_path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  nav_msgs::msg::Path makeInnerPlannerPath(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    grid::GoalPolicy goal_policy);

  nav_msgs::msg::Path makeConnectorPath(
    const geometry_msgs::msg::PoseStamped & start, const Segment & segment);

  // 位姿处理与路径拼接
  geometry_msgs::msg::PoseStamped makePoseFromPoint(
    const Point2D & point, const rclcpp::Time & stamp) const;

  geometry_msgs::msg::PoseStamped normalizePoseFrame(
    const geometry_msgs::msg::PoseStamped & pose) const;

  void setYaw(geometry_msgs::msg::PoseStamped & pose, double yaw) const;
  void appendSegment(nav_msgs::msg::Path & full_path, const nav_msgs::msg::Path & segment) const;

  struct DirectedEdge
  {
    std::string to;
    int connector_index{-1};
    double cost{1.0};
  };

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  map_server::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("TopologyGlobalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr connector_debug_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr current_region_pub_;
  rclcpp::TimerBase::SharedPtr current_region_timer_;
  void publishConnectorDebugMarkers();
  void publishCurrentRegion();

  std::string name_;
  std::string global_frame_;
  std::string robot_base_frame_;

  std::string topology_yaml_;
  bool use_topology_{true};
  bool allow_nearest_region_fallback_{false};
  double region_boundary_tolerance_{0.03};
  double transform_tolerance_{0.1};
  double max_nearest_region_distance_{1.0};
  double region_constraint_tolerance_{0.15};
  double connector_path_half_width_{0.8};
  double connector_entry_clearance_{0.5};

  std::unique_ptr<grid::GridPlanner> grid_planner_;
  std::function<bool()> cancel_checker_{[]() {
    return false;
  }};

  std::vector<Region> regions_;
  std::vector<Connector> connectors_;
  std::unordered_map<std::string, std::vector<DirectedEdge>> graph_;
};

}
