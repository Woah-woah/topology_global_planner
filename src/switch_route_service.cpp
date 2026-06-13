#include "topology_global_planner/topology_global_planner.hpp"



namespace topology_global_planner
{

void TopologyGlobalPlanner::switchRouteModeCallback(
  const std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Request> request,
  std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Response> response
){
  blocked_connector_id = request->connector_id;

  for(auto &connector : connectors_){
    if(connector.id == blocked_connector_id){
      connector.cost = 999.0;
      break;
    }
  }
  buildGraph();

  response->success = true;
}

void TopologyGlobalPlanner::reConnectorCostCallback(
  const std::shared_ptr<topology_global_planner::srv::RestoreConnectorCost::Request> request,
  std::shared_ptr<topology_global_planner::srv::RestoreConnectorCost::Response> response
){
  (void)request;

  if(blocked_connector_id.empty()){
    response->success = false;
    return;
  }

  for(auto &connector : connectors_){         //假设只有两条路的情况下，所以只会block一条
    if(connector.id == blocked_connector_id){
      connector.cost = 1.0;
      blocked_connector_id = "";
      break;
    }
  }
  buildGraph();
  response->success = true;
}



}// namespace topology_global_planner