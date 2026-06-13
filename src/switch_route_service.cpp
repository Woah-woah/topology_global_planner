#include "topology_global_planner/topology_global_planner.hpp"



namespace topology_global_planner
{

void TopologyGlobalPlanner::switchRouteModeCallback(
  const std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Request> request,
  std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Response> response
){
  const std::string &connector_id = request->connector_id;

  for(auto &connector : connectors_){
    if(connector.id == connector_id){
      setConnectorCost(connector_id, 999.0);
      has_blocked_connector_ = true;
      break;
    }
  }
  buildGraph();

  response->success = true;
}


void TopologyGlobalPlanner::setConnectorCost(const std::string &connector_id, double new_cost){
  for(auto &connector : connectors_){
    if(connector.id == connector_id){
      connector.cost = new_cost;
      return;
    }
  }
  return;
}

}// namespace topology_global_planner