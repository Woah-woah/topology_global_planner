#pragma once

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace topology_global_planner
{
enum class SegmentType
{
  NORMAL,
  CONNECTOR
};

struct Segment
{
  SegmentType type{SegmentType::NORMAL};
  nav_msgs::msg::Path path;
  std::string connector_id;
  std::string action;
  geometry_msgs::msg::PoseStamped wait_pose;
  geometry_msgs::msg::PoseStamped exit_pose;
  // 本次通行方向及两端区域
  std::string from_region;
  std::string to_region;
  double corridor_half_width{0.8};
};

struct TopologyPlan
{
  // 控制器跟踪完整路径
  // 分段数据用于进度管理和通道行为判断
  nav_msgs::msg::Path full_path = nav_msgs::msg::Path();
  std::vector<Segment> segments;
};
}
