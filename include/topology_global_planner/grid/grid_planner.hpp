// 读取代价地图并生成栅格路径
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "map_server/costmap_2d.hpp"
#include "rclcpp/rclcpp.hpp"

#include "topology_global_planner/grid/navfn.hpp"

namespace topology_global_planner::grid
{

enum class GoalPolicy
{
  EXACT,
  WITHIN_TOLERANCE
};

class GridPlanner
{
public:
  struct Options
  {
    double tolerance{0.2};
    bool use_astar{false};
    bool allow_unknown{true};
    bool use_final_approach_orientation{true};
  };

  GridPlanner(map_server::Costmap2D * costmap,
    const std::string & global_frame,
    rclcpp::Clock::SharedPtr clock,
    Options options);

  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    GoalPolicy goal_policy,
    std::function<bool()> cancel_checker);

private:
  map_server::Costmap2D * costmap_;
  std::string global_frame_;
  rclcpp::Clock::SharedPtr clock_;
  Options options_;

  std::unique_ptr<NavFn> navfn_;
};

}