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

geometry_msgs::msg::PoseStamped TopologyGlobalPlanner::makePoseFromPoint(
  const Point2D & point,
  const rclcpp::Time & stamp) const                 // 把二维点，变成 ROS/Nav2 能用的 pose
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = stamp;
  pose.pose.position.x = point.x;
  pose.pose.position.y = point.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}


// 取portal均分粗采样点
std::vector<geometry_msgs::msg::PoseStamped> TopologyGlobalPlanner::sampleConnectorPoses(
  const Connector & connector,
  const rclcpp::Time & stamp) const
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  const int n = std::max(1, portal_sample_count_);
  poses.reserve(static_cast<size_t>(n));

  for (int i = 0; i < n; i++) {
    // Coarse samples. Do not sample exact endpoints; they are often close to walls or region boundaries.
    // Example: n=5 -> t = 1/6, 2/6, ..., 5/6.
    const double t = static_cast<double>(i + 1) / static_cast<double>(n + 1);

    Point2D p;
    p.x = connector.portal_start.x + t * (connector.portal_end.x - connector.portal_start.x);
    p.y = connector.portal_start.y + t * (connector.portal_end.y - connector.portal_start.y);

    poses.push_back(makePoseFromPoint(p, stamp));
  }
  return poses;

}

std::vector<geometry_msgs::msg::PoseStamped> TopologyGlobalPlanner::sampleConnectorPosesAround(
  const Connector & connector,
  const rclcpp::Time & stamp,
  double center_t,
  double half_width,
  int sample_count) const
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  const int n = std::max(1, sample_count);
  poses.reserve(static_cast<size_t>(n));

  // Keep a tiny margin from endpoints to avoid choosing points exactly on walls / region corners.
  constexpr double endpoint_margin = 1e-3;
  center_t = std::clamp(center_t, endpoint_margin, 1.0 - endpoint_margin);
  half_width = std::max(half_width, endpoint_margin);

  const double low = std::clamp(center_t - half_width, endpoint_margin, 1.0 - endpoint_margin);
  const double high = std::clamp(center_t + half_width, endpoint_margin, 1.0 - endpoint_margin);

  for (int i = 0; i < n; i++) {
    double t = center_t;
    if (n > 1 && high > low) {
      t = low + (high - low) * static_cast<double>(i) / static_cast<double>(n - 1);
    }
    t = std::clamp(t, endpoint_margin, 1.0 - endpoint_margin);

    Point2D p;
    p.x = connector.portal_start.x + t * (connector.portal_end.x - connector.portal_start.x);
    p.y = connector.portal_start.y + t * (connector.portal_end.y - connector.portal_start.y);

    poses.push_back(makePoseFromPoint(p, stamp));
  }

  return poses;
}

double TopologyGlobalPlanner::projectionTOnConnector(
  const Connector & connector,
  const geometry_msgs::msg::PoseStamped & pose) const
{

  const double ax = connector.portal_start.x;
  const double ay = connector.portal_start.y;
  const double bx = connector.portal_end.x;
  const double by = connector.portal_end.y;
  const double vx = bx - ax;
  const double vy = by - ay;
  const double len2 = vx * vx + vy * vy;

  if (len2 <= 1e-12) {
    return 0.5;
  }

  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  const double t = ((px - ax) * vx + (py - ay) * vy) / len2;
  return std::clamp(t, 0.0, 1.0);
}

double TopologyGlobalPlanner::pathLength(const nav_msgs::msg::Path & path) const
{
  if (path.poses.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }

  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    length += euclidean(a.x, a.y, b.x, b.y);
  }

  return length;
}

nav_msgs::msg::Path TopologyGlobalPlanner::makePortalOptimizedTopologyPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const TopologySearchResult & topo_result)
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = global_frame_;
  empty.header.stamp = clock_ ? clock_->now() : rclcpp::Time(0);

  if (topo_result.connector_indices.empty()) {
    return makeInnerPlannerPath(start, goal);
  }

  const auto plan_stamp =
    start.header.stamp.sec == 0 && start.header.stamp.nanosec == 0 && clock_ ?
    clock_->now() : rclcpp::Time(start.header.stamp);

  struct SolveResult
  {
    bool success{false};
    double cost{std::numeric_limits<double>::infinity()};
    nav_msgs::msg::Path path;
    std::vector<int> selected_indices;
    int planner_calls{0};
  };

  auto solve_layers = [this](
    const std::vector<std::vector<geometry_msgs::msg::PoseStamped>> & layers,
    const std::vector<std::string> & expected_regions) -> SolveResult
    {
      SolveResult result;
      result.path.header.frame_id = global_frame_;
      result.path.header.stamp = clock_ ? clock_->now() : rclcpp::Time(0);

      if (layers.size() < 2) {
        return result;
      }

      struct Cell
      {
        double cost{std::numeric_limits<double>::infinity()};
        int prev_index{-1};
        nav_msgs::msg::Path segment;
      };

      std::vector<std::vector<Cell>> dp(layers.size());
      dp[0].resize(layers[0].size());
      if (dp[0].empty()) {
        return result;
      }
      dp[0][0].cost = 0.0;

      for (size_t layer = 1; layer < layers.size(); ++layer) {
        dp[layer].resize(layers[layer].size());

        for (size_t cur = 0; cur < layers[layer].size(); ++cur) {
          for (size_t prev = 0; prev < layers[layer - 1].size(); ++prev) {
            if (!std::isfinite(dp[layer - 1][prev].cost)) {
              continue;
            }

            auto seg_start = layers[layer - 1][prev];
            auto seg_goal = layers[layer][cur];

            // Connector samples are through-points, not final poses. Give orientation-aware
            // planners a route-direction yaw, but preserve the real start and final goal yaw.
            const double dx = seg_goal.pose.position.x - seg_start.pose.position.x;
            const double dy = seg_goal.pose.position.y - seg_start.pose.position.y;
            if (std::hypot(dx, dy) > 1e-6) {
              const double yaw = std::atan2(dy, dx);

              if (layer - 1 != 0) {
                setYaw(seg_start, yaw);
              }
              if (layer + 1 != layers.size()) {
                setYaw(seg_goal, yaw);
              }
            }

            auto segment = makeInnerPlannerPath(seg_start, seg_goal);
            result.planner_calls++;
            if (segment.poses.empty()) {
              continue;
            }

            if (enforce_region_constraint_) {
              const size_t expected_index = layer - 1;
              if (expected_index >= expected_regions.size()) {
                RCLCPP_WARN(
                  logger_,
                  "Missing expected region for topology segment %zu; rejecting this candidate",
                  expected_index);
                continue;
              }

              const auto & expected_region = expected_regions[expected_index];
              if (!pathInsideRegionWithTolerance(
                  segment, expected_region, region_constraint_tolerance_))
              {
                RCLCPP_DEBUG(
                  logger_,
                  "Reject topology segment %zu because inner path leaves expected region '%s'",
                  expected_index, expected_region.c_str());
                continue;
              }
            }

            const double edge_cost = pathLength(segment);
            if (!std::isfinite(edge_cost)) {
              continue;
            }

            const double total_cost = dp[layer - 1][prev].cost + edge_cost;
            if (total_cost < dp[layer][cur].cost) {
              dp[layer][cur].cost = total_cost;
              dp[layer][cur].prev_index = static_cast<int>(prev);
              dp[layer][cur].segment = segment;
            }
          }
        }
      }

      const size_t final_layer = layers.size() - 1;
      int best_final_index = -1;
      double best_cost = std::numeric_limits<double>::infinity();

      for (size_t i = 0; i < dp[final_layer].size(); ++i) {
        if (dp[final_layer][i].cost < best_cost) {
          best_cost = dp[final_layer][i].cost;
          best_final_index = static_cast<int>(i);
        }
      }

      if (best_final_index < 0 || !std::isfinite(best_cost)) {
        return result;
      }

      std::vector<int> selected_indices(layers.size(), 0);
      std::vector<nav_msgs::msg::Path> reversed_segments;
      int cur_index = best_final_index;

      for (size_t layer = final_layer; layer > 0; --layer) {
        selected_indices[layer] = cur_index;
        const auto & cell = dp[layer][cur_index];
        if (cell.prev_index < 0 || cell.segment.poses.empty()) {
          return result;
        }

        reversed_segments.push_back(cell.segment);
        cur_index = cell.prev_index;
      }
      selected_indices[0] = cur_index;

      std::reverse(reversed_segments.begin(), reversed_segments.end());

      nav_msgs::msg::Path full_path;
      full_path.header.frame_id = global_frame_;
      full_path.header.stamp = clock_ ? clock_->now() : rclcpp::Time(0);

      for (const auto & segment : reversed_segments) {
        appendSegment(full_path, segment);
      }

      result.success = !full_path.poses.empty();
      result.cost = best_cost;
      result.path = full_path;
      result.selected_indices = selected_indices;
      return result;
    };

  auto build_coarse_layers = [&]() {
      std::vector<std::vector<geometry_msgs::msg::PoseStamped>> layers;
      layers.push_back({start});

      for (const int idx : topo_result.connector_indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= connectors_.size()) {
          RCLCPP_WARN(logger_, "Invalid connector index in topology result");
          return std::vector<std::vector<geometry_msgs::msg::PoseStamped>>{};
        }

        auto samples = sampleConnectorPoses(connectors_[idx], plan_stamp);
        if (samples.empty()) {
          RCLCPP_WARN(logger_, "Connector '%s' has no valid samples", connectors_[idx].id.c_str());
          return std::vector<std::vector<geometry_msgs::msg::PoseStamped>>{};
        }

        layers.push_back(samples);
      }

      layers.push_back({goal});
      return layers;
    };

  const auto coarse_layers = build_coarse_layers();
  if (coarse_layers.empty()) {
    return empty;
  }

  const auto expected_regions = topo_result.region_path;
  if (expected_regions.size() + 1 != coarse_layers.size()) {
    RCLCPP_WARN(
      logger_,
      "Region constraint metadata mismatch: regions=%zu, layers=%zu. Returning empty path.",
      expected_regions.size(), coarse_layers.size());
    return empty;
  }

  auto coarse_result = solve_layers(coarse_layers, expected_regions);
  if (!coarse_result.success) {
    RCLCPP_WARN(logger_, "Portal optimized topology path failed: no valid coarse candidate chain");
    return empty;
  }

  if (!portal_adaptive_sampling_) {
    RCLCPP_INFO(
      logger_,
      "Portal optimized topology path success: %zu connector(s), sample_count=%d, cost=%.3f, planner_calls=%d",
      topo_result.connector_indices.size(), portal_sample_count_, coarse_result.cost,
      coarse_result.planner_calls);
    return coarse_result.path;
  }

  // Adaptive refinement:
  // 1) First solve a coarse candidate graph.
  // 2) Around each selected portal position, sample a smaller local window.
  // 3) Re-solve with the refined candidate graph.
  // This gives a result close to using many samples, but with far fewer InnerPlanner calls.
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> refined_layers;
  refined_layers.push_back({start});

  const double coarse_half_width = 1.0 / static_cast<double>(portal_sample_count_ + 1);

  for (size_t i = 0; i < topo_result.connector_indices.size(); ++i) {
    const int idx = topo_result.connector_indices[i];
    if (idx < 0 || static_cast<size_t>(idx) >= connectors_.size()) {
      return coarse_result.path;
    }

    const auto & connector = connectors_[idx];

    const size_t layer_index = i + 1;
    if (layer_index >= coarse_layers.size() || layer_index >= coarse_result.selected_indices.size()) {
      return coarse_result.path;
    }

    const int selected_index = coarse_result.selected_indices[layer_index];
    if (selected_index < 0 || static_cast<size_t>(selected_index) >= coarse_layers[layer_index].size()) {
      return coarse_result.path;
    }

    const double center_t = projectionTOnConnector(
      connector, coarse_layers[layer_index][selected_index]);
    auto refined_samples = sampleConnectorPosesAround(
      connector, plan_stamp, center_t, coarse_half_width, portal_refine_sample_count_);

    if (refined_samples.empty()) {
      return coarse_result.path;
    }
    refined_layers.push_back(refined_samples);
  }

  refined_layers.push_back({goal});

  auto refined_result = solve_layers(refined_layers, expected_regions);
  if (refined_result.success && refined_result.cost <= coarse_result.cost) {
    // RCLCPP_INFO(
    //   logger_,
    //   "Portal adaptive topology path success: %zu connector(s), coarse=%d, refine=%d, cost %.3f -> %.3f, planner_calls=%d+%d",
    //   topo_result.connector_indices.size(), portal_sample_count_, portal_refine_sample_count_,
    //   coarse_result.cost, refined_result.cost, coarse_result.planner_calls,
    //   refined_result.planner_calls);
    return refined_result.path;
  }

  RCLCPP_INFO(
    logger_,
    "Portal adaptive refinement kept coarse result: %zu connector(s), coarse=%d, refine=%d, cost=%.3f, planner_calls=%d+%d",
    topo_result.connector_indices.size(), portal_sample_count_, portal_refine_sample_count_,
    coarse_result.cost, coarse_result.planner_calls, refined_result.planner_calls);
  return coarse_result.path;
}


}  // namespace topology_global_planner
