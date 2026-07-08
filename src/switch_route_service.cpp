#include "topology_global_planner/topology_global_planner.hpp"



namespace topology_global_planner
{

void TopologyGlobalPlanner::switchRouteModeCallback(
  const std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Request> request,
  std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Response> response
){
  if(!blocked_connector_id.empty()){        // 在单个导航任务中每次运行switch之前初始化，避免在只有两条路可走的时候不能来回切换
    for(auto &connector : connectors_){
      connector.cost = 1.0;
    }
  }

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
    response->success = true;
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