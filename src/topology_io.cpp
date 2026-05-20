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

    if (!root["regions"] || !root["regions"].IsSequence()) {
      RCLCPP_ERROR(logger_, "topology.yaml must contain a sequence field named 'regions'");
      return false;
    }

    for (const auto & region_node : root["regions"]) {
      Region region;
      if (!region_node["id"]) {
        RCLCPP_ERROR(logger_, "Every region needs an 'id'");
        return false;
      }
      region.id = region_node["id"].as<std::string>();
      region.name = region_node["name"] ? region_node["name"].as<std::string>() : region.id;

      if (hasRegion(region.id)) {
        RCLCPP_ERROR(logger_, "Duplicate region id '%s' in topology.yaml", region.id.c_str());
        return false;
      }

      const YAML::Node polygon_node = region_node["polygon"] ? region_node["polygon"] : region_node["points"];
      if (!polygon_node || !polygon_node.IsSequence() || polygon_node.size() < 3) {
        RCLCPP_ERROR(logger_, "Region '%s' needs polygon/points with at least 3 vertices", region.id.c_str());
        return false;
      }

      for (const auto & p : polygon_node) {
        if (!p["x"] || !p["y"]) {
          RCLCPP_ERROR(logger_, "Region '%s' has a polygon point without x/y", region.id.c_str());
          return false;
        }
        Point2D pt;
        pt.x = p["x"].as<double>();
        pt.y = p["y"].as<double>();
        region.polygon.push_back(pt);
      }
      region.centroid = computeCentroid(region.polygon);
      regions_.push_back(region);
    }

    if (!root["connectors"] || !root["connectors"].IsSequence()) {
      RCLCPP_ERROR(logger_, "topology.yaml must contain a sequence field named 'connectors'");
      return false;
    }

    int unnamed_index = 0;
    for (const auto & connector_node : root["connectors"]) {
      Connector connector;
      connector.id = connector_node["id"] ? connector_node["id"].as<std::string>() :
        "C" + std::to_string(++unnamed_index);
      if (!connector_node["from"] || !connector_node["to"]) {
        RCLCPP_ERROR(logger_, "Connector '%s' needs from/to", connector.id.c_str());
        return false;
      }
      connector.from = connector_node["from"].as<std::string>();
      connector.to = connector_node["to"].as<std::string>();
      connector.cost = connector_node["cost"] ? connector_node["cost"].as<double>() : 1.0;

      if (!hasRegion(connector.from) || !hasRegion(connector.to)) {
        RCLCPP_ERROR(
          logger_, "Connector '%s' references unknown region: from='%s', to='%s'",
          connector.id.c_str(), connector.from.c_str(), connector.to.c_str());
        return false;
      }

      if (connector_node["mode"]) {
        connector.mode = connector_node["mode"].as<std::string>();
      } else if (connector_node["bidirectional"]) {
        connector.mode = connector_node["bidirectional"].as<bool>() ? "two_way" : "one_way";
      }
      if (connector.mode == "bidirectional") {
        connector.mode = "two_way";
      }
      if (!(connector.mode == "two_way" || connector.mode == "one_way")) {
        RCLCPP_ERROR(
          logger_, "Connector '%s' has invalid mode '%s'. Use two_way or one_way.",
          connector.id.c_str(), connector.mode.c_str());
        return false;
      }

      if (connector_node["portal"]) {
        const auto portal = connector_node["portal"];
        if (!portal["start"] || !portal["end"]) {
          RCLCPP_ERROR(logger_, "Connector '%s' portal needs start/end", connector.id.c_str());
          return false;
        }

        const auto portal_start = portal["start"];
        const auto portal_end = portal["end"];
        if (!portal_start["x"] || !portal_start["y"] || !portal_end["x"] || !portal_end["y"]) {
          RCLCPP_ERROR(logger_, "Connector '%s' portal start/end need x/y", connector.id.c_str());
          return false;
        }

        connector.portal_start.x = portal_start["x"].as<double>();
        connector.portal_start.y = portal_start["y"].as<double>();
        connector.portal_end.x = portal_end["x"].as<double>();
        connector.portal_end.y = portal_end["y"].as<double>();
        connector.has_portal = true;

        if (euclidean(
            connector.portal_start.x, connector.portal_start.y,
            connector.portal_end.x, connector.portal_end.y) <= 1e-6)
        {
          RCLCPP_ERROR(logger_, "Connector '%s' portal length is too small", connector.id.c_str());
          return false;
        }
      } else if (connector_node["waypoint"]) {
        const auto wp = connector_node["waypoint"];
        if (!wp["x"] || !wp["y"]) {
          RCLCPP_ERROR(logger_, "Connector '%s' waypoint needs x/y", connector.id.c_str());
          return false;
        }
        connector.waypoint.x = wp["x"].as<double>();
        connector.waypoint.y = wp["y"].as<double>();
        connector.has_waypoint = true;
        if (wp["yaw"]) {
          RCLCPP_WARN(
            logger_,
            "Connector '%s' waypoint yaw is ignored. Connectors are through-points; "
            "intermediate yaw is generated from route direction.",
            connector.id.c_str());
        }
      } else {
        RCLCPP_ERROR(logger_, "Connector '%s' needs either portal or waypoint", connector.id.c_str());
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

void TopologyGlobalPlanner::buildGraph()
{
  graph_.clear();
  for (size_t i = 0; i < connectors_.size(); ++i) {
    const auto & c = connectors_[i];
    graph_[c.from].push_back(DirectedEdge{c.to, static_cast<int>(i), std::max(0.001, c.cost)});
    if (c.mode == "two_way") {
      graph_[c.to].push_back(DirectedEdge{c.from, static_cast<int>(i), std::max(0.001, c.cost)});
    }
  }
}


}  // namespace topology_global_planner
