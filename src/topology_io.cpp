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


}  // namespace topology_global_planner
