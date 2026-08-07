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

bool TopologyGlobalPlanner::loadTopologyYaml(const std::string & yaml_path)
{
  regions_.clear();
  connectors_.clear();

  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);

    for (const auto & region_node : root["regions"]) {
      Region region;
      region.id = region_node["id"].as<std::string>();
      region.name = region_node["name"] ? region_node["name"].as<std::string>() : region.id;

      const YAML::Node polygon_node = region_node["polygon"];
      for (const auto & p : polygon_node) {
        Point2D pt;
        pt.x = p["x"].as<double>();
        pt.y = p["y"].as<double>();
        region.polygon.push_back(pt);
      }
      region.centroid = computeCentroid(region.polygon);
      regions_.push_back(region);
    }

    for (const auto & connector_node : root["connectors"]) {
      Connector connector;
      connector.id = connector_node["id"].as<std::string>();
      connector.from = connector_node["from"].as<std::string>();
      connector.to = connector_node["to"].as<std::string>();
      connector.cost = connector_node["cost"] ? connector_node["cost"].as<double>() : 1.0;
      connector.action = connector_node["action"].as<std::string>();

      if (connector.action != "down" && connector.action != "step") {
        RCLCPP_WARN(logger_, "Connector '%s' has unknown action '%s'.",
          connector.id.c_str(), connector.action.c_str());
      }

      if (!hasRegion(connector.from) || !hasRegion(connector.to)) {
        RCLCPP_ERROR(
          logger_, "Connector '%s' references unknown region: from='%s', to='%s'",
          connector.id.c_str(), connector.from.c_str(), connector.to.c_str());          // from & to的region不存在返回failure
        return false;
      }

      connector.mode = connector_node["mode"].as<std::string>();
      if (!(connector.mode == "two_way" || connector.mode == "one_way")) {
        RCLCPP_ERROR(
          logger_, "Connector '%s' has invalid mode '%s'. Use two_way or one_way.",
          connector.id.c_str(), connector.mode.c_str());
        return false;
      }                                                                                  // mode必须用two_way或者one_way


      if (connector_node["portal"]) {
        const auto portal = connector_node["portal"];
        const auto portal_start = portal["start"];
        const auto portal_end = portal["end"];

        connector.portal_start.x = portal_start["x"].as<double>();
        connector.portal_start.y = portal_start["y"].as<double>();
        connector.portal_end.x = portal_end["x"].as<double>();
        connector.portal_end.y = portal_end["y"].as<double>();

      } else {
        RCLCPP_ERROR(logger_, "Connector '%s' needs portal", connector.id.c_str());
        return false;
      }

      connectors_.push_back(connector);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception while loading topology yaml '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  return true;
}

void TopologyGlobalPlanner::publishConnectorDebugMarkers()
{
  if (!connector_debug_markers_pub_) return;

  visualization_msgs::msg::MarkerArray array;
  const auto stamp = clock_ ? clock_->now() : rclcpp::Time(0);
  int id = 0;

  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  auto addArrow = [&](const Point2D & start, const Point2D & end,
                      double offset_x, double offset_y,
                      float r, float g, float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = stamp;
    marker.ns = "connectors";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p1, p2;
    p1.x = start.x + offset_x;
    p1.y = start.y + offset_y;
    p1.z = 0.1;
    p2.x = end.x + offset_x;
    p2.y = end.y + offset_y;
    p2.z = 0.1;

    marker.points = {p1, p2};
    marker.scale.x = 0.04;
    marker.scale.y = 0.10;
    marker.scale.z = 0.12;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    array.markers.push_back(marker);
  };

  for (const auto & connector : connectors_) {
    const double dx = connector.portal_end.x - connector.portal_start.x;
    const double dy = connector.portal_end.y - connector.portal_start.y;
    const double length = std::hypot(dx, dy);

    if (length < 1e-6) continue;

    if (connector.mode == "two_way") {
      addArrow(connector.portal_start, connector.portal_end, 0.0, 0.0, 0.0F, 1.0F, 0.0F);
      addArrow(connector.portal_end, connector.portal_start, 0.0, 0.0, 0.0F, 1.0F, 0.0F);
    } else if (connector.mode == "one_way") {
      addArrow(connector.portal_start, connector.portal_end, 0.0, 0.0, 1.0F, 0.45F, 0.0F);
    }
  }

  connector_debug_markers_pub_->publish(array);
}

void TopologyGlobalPlanner::buildGraph()                      // 根据from to 转化成邻接图
{
  graph_.clear();
  for (size_t i = 0; i < connectors_.size(); i++) {
    const auto & c = connectors_[i];
    graph_[c.from].push_back(DirectedEdge{c.to, static_cast<int>(i), c.cost});
    if (c.mode == "two_way") {
      graph_[c.to].push_back(DirectedEdge{c.from, static_cast<int>(i), c.cost});
    }
  }
}

void TopologyGlobalPlanner::blockCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg || !msg->data || !is_on_connector_ || active_connector_id_.empty()) {
    return;
  }

  const auto connector_it = std::find_if(
    connectors_.begin(), connectors_.end(),
    [this](const Connector & connector) {
      return connector.id == active_connector_id_;
    });

  if (connector_it == connectors_.end()) {
    RCLCPP_WARN(
      logger_, "Cannot block active connector '%s': connector was not found",
      active_connector_id_.c_str());
    return;
  }

  if (connector_it->cost == blocked_connector_cost_) {
    return;
  }

  connector_it->cost = blocked_connector_cost_;
  buildGraph();

  RCLCPP_WARN(
    logger_, "Blocked connector '%s' with cost %.3f until a new goal is received",
    connector_it->id.c_str(), connector_it->cost);
}

void TopologyGlobalPlanner::restoreAllConnectorCosts()
{
  for (auto & connector : connectors_) {
    if (connector.cost == 1.0) {
      continue;
    }

    connector.cost = 1.0;
  }
  buildGraph();
}


}  // namespace topology_global_planner
