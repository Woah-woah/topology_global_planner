#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "map_server/costmap_2d.hpp"
#include "rclcpp/rclcpp.hpp"

namespace topology_global_planner::grid
{
enum class GoalPolicy {EXACT, WITHIN_TOLERANCE};

struct PlanningDiagnostics
{
  nav_msgs::msg::Path eikonal_path;
  std::vector<float> eikonal_potential;
  std::string extraction, failure_reason;
  double graph_ms{0}, field_ms{0}, extraction_ms{0};
  double total_ms{0}, max_residual{0};
  size_t field_expansions{0}, field_reachable{0}, projected_steps{0};
  bool field_complete{false};
};

class GridPlanner
{
public:
  // 校验器逐段检查连续路径的 Region/Connector 约束。
  using PathValidator = std::function<bool(const nav_msgs::msg::Path &)>;
  struct Options
  {
    double tolerance{0.2};
    bool use_astar{false};
    bool allow_unknown{false};
    bool use_final_approach_orientation{true};
    bool collect_diagnostics{false};
  };

  GridPlanner(map_server::Costmap2D * costmap, const std::string & global_frame,
    rclcpp::Clock::SharedPtr clock, Options options);
  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, GoalPolicy goal_policy,
    std::function<bool()> cancel_checker, PathValidator topology_valid = {});
  const PlanningDiagnostics & lastDiagnostics() const {return diagnostics_;}

private:
  map_server::Costmap2D * costmap_;
  std::string global_frame_;
  rclcpp::Clock::SharedPtr clock_;
  Options options_;
  PlanningDiagnostics diagnostics_;
};
}  // namespace topology_global_planner::grid
