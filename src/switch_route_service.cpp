#include "topology_global_planner/topology_global_planner.hpp"



namespace topology_global_planner
{

void TopologyGlobalPlanner::switchRouteModeCallback(
  const std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Request> request,
  std::shared_ptr<topology_global_planner::srv::SwitchRouteMode::Response> response
){
  (void)request; // request为空，占位声明，消除编译器的“未使用参数”警告
  route_mode_ = 1 - route_mode_; //将要切换的mode转化成与当前不同的mode

  //应用更改
  applyModeCost();
  buildGraph();

  response->success = true;
}

void TopologyGlobalPlanner::applyModeCost(){
  if(route_mode_ == 1){
    setConnectorCost("C54", 1.0);
    setConnectorCost("C43", 999.0);
  } else {
    setConnectorCost("C54", 999.0);
    setConnectorCost("C43", 1.0);
  }
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