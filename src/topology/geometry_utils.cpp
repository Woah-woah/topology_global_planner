#include "topology_global_planner/topology_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace topology_global_planner
{

std::string TopologyGlobalPlanner::findRegion(double x, double y) const
{
  std::vector<std::string> inside_candidates;
  std::vector<std::string> boundary_candidates;

  for (const auto & region : regions_) {
    if (pointInPolygon(x, y, region.polygon)) {
      inside_candidates.push_back(region.id);
    } else if (pointOnPolygonBoundary(x, y, region.polygon, region_boundary_tolerance_)) {
      boundary_candidates.push_back(region.id);
    }
  }

  if (!inside_candidates.empty()) {
    if (inside_candidates.size() > 1) {
      RCLCPP_DEBUG(
        logger_, "Point (%.3f, %.3f) is inside multiple regions; choosing nearest centroid.", x, y);
    }
    return chooseBestRegionByCentroid(x, y, inside_candidates);
  }

  if (!boundary_candidates.empty()) {
    if (boundary_candidates.size() > 1) {
      RCLCPP_DEBUG(logger_,
        "Point (%.3f, %.3f) is on multiple region boundaries; choosing nearest centroid.",
        x,
        y);
    }
    return chooseBestRegionByCentroid(x, y, boundary_candidates);
  }

  if (!allow_nearest_region_fallback_ || regions_.empty()) {
    return "";
  }

  double best_dist = std::numeric_limits<double>::infinity();
  std::string best_id;
  for (const auto & region : regions_) {
    const double d = std::hypot(x - region.centroid.x, y - region.centroid.y);
    if (d < best_dist) {
      best_dist = d;
      best_id = region.id;
    }
  }

  if (best_dist <= std::max(0.0, max_nearest_region_distance_)) {
    RCLCPP_DEBUG(logger_,
      "Point (%.3f, %.3f) is outside regions; using nearest region '%s' at %.3fm.",
      x,
      y,
      best_id.c_str(),
      best_dist);
    return best_id;
  }

  return "";
}

std::string TopologyGlobalPlanner::chooseBestRegionByCentroid(
  double x, double y, const std::vector<std::string> & candidates) const
{
  double best_dist = std::numeric_limits<double>::infinity();
  std::string best_id;

  for (const auto & id : candidates) {
    const auto * region = getRegionById(id);
    if (!region) {
      continue;
    }

    const double d = std::hypot(x - region->centroid.x, y - region->centroid.y);
    if (d < best_dist) {
      best_dist = d;
      best_id = id;
    }
  }

  return best_id;
}

bool TopologyGlobalPlanner::pointInPolygon(
  double x, double y, const std::vector<Point2D> & polygon) const
{
  // 射线法判断点是否在多边形内
  // 水平边由跨越条件排除
  // 边界点另行判断
  bool inside = false;
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    if ((pi.y > y) != (pj.y > y)) {
      const double x_intersect = (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x;
      if (x < x_intersect) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool TopologyGlobalPlanner::pointOnPolygonBoundary(
  double x, double y, const std::vector<Point2D> & polygon, double tolerance) const
{
  const Point2D p{x, y};
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    if (distancePointToSegment(p, a, b) <= std::max(0.0, tolerance)) {
      return true;
    }
  }
  return false;
}

const Region * TopologyGlobalPlanner::getRegionById(const std::string & id) const
{
  const auto it = std::find_if(regions_.begin(), regions_.end(), [&](const Region & r) {
    return r.id == id;
  });
  if (it == regions_.end()) {
    return nullptr;
  }
  return &(*it);
}

bool TopologyGlobalPlanner::pathInsideRegionWithTolerance(
  const nav_msgs::msg::Path & path, const std::string & region_id, double tolerance) const
{
  if (region_id.empty() || path.poses.empty()) {
    return false;
  }

  // 采样过程中复用同一区域
  const auto * region = getRegionById(region_id);
  if (!region) {
    return false;
  }

  auto previous = path.poses.front().pose.position;
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    const int samples = std::max(
      1, static_cast<int>(std::ceil(std::hypot(p.x - previous.x, p.y - previous.y) / 0.02)));
    for (int i = 0; i <= samples; ++i) {
      const double f = static_cast<double>(i) / samples;
      const double x = previous.x + f * (p.x - previous.x);
      const double y = previous.y + f * (p.y - previous.y);
      if (!pointInPolygon(x, y, region->polygon) &&
          !pointOnPolygonBoundary(x, y, region->polygon, tolerance))
      {
        return false;
      }
      // 区域重叠时也不允许普通路径穿过通道内部
      // 入口和出口附近留给分段衔接
      for (const auto & connector : connectors_) {
        const auto & a = connector.portal_start;
        const auto & b = connector.portal_end;
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-6) {
          return false;
        }
        const double along = ((x - a.x) * dx + (y - a.y) * dy) / length;
        const double lateral = std::abs((x - a.x) * dy - (y - a.y) * dx) / length;
        const double endpoint_margin = std::min(0.05, length * 0.1);
        if (along > endpoint_margin && along < length - endpoint_margin &&
            lateral < connector_path_half_width_)
        {
          return false;
        }
      }
    }
    previous = p;
  }

  return true;
}

bool TopologyGlobalPlanner::pathInsideConnectorStrip(const nav_msgs::msg::Path & path,
  const Point2D & wait,
  const Point2D & exit,
  double half_width) const
{
  const double vx = exit.x - wait.x;
  const double vy = exit.y - wait.y;
  const double length_sq = vx * vx + vy * vy;

  if (length_sq <= 1e-9) {
    return false;
  }

  double previous_t = -1.0;
  for (const auto & pose : path.poses) {
    const double px = pose.pose.position.x - wait.x;
    const double py = pose.pose.position.y - wait.y;

    const double t = (px * vx + py * vy) / length_sq;

    // 限制沿通道方向的位置和回退量
    const double cell_slack = std::max(0.05, costmap_->geometry().resolution) / std::sqrt(length_sq);
    if (t < -cell_slack || t > 1.0 + cell_slack || t + cell_slack < previous_t) {
      return false;
    }

    previous_t = std::max(previous_t, t);
    const double clamped_t = std::clamp(t, 0.0, 1.0);
    const double proj_x = wait.x + clamped_t * vx;
    const double proj_y = wait.y + clamped_t * vy;

    const double lateral_distance =
      std::hypot(pose.pose.position.x - proj_x, pose.pose.position.y - proj_y);

    if (lateral_distance > half_width) {
      return false;
    }
  }
  return true;
}

double TopologyGlobalPlanner::distancePointToSegment(
  const Point2D & p, const Point2D & a, const Point2D & b) const
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = p.x - a.x;
  const double wy = p.y - a.y;
  const double c1 = vx * wx + vy * wy;
  if (c1 <= 0.0) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  const double c2 = vx * vx + vy * vy;
  if (c2 <= 1e-12) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  if (c2 <= c1) {
    return std::hypot(p.x - b.x, p.y - b.y);
  }
  const double t = c1 / c2;
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  return std::hypot(p.x - proj_x, p.y - proj_y);
}

Point2D TopologyGlobalPlanner::computeCentroid(const std::vector<Point2D> & polygon) const
{
  Point2D c;
  if (polygon.empty()) {
    return c;
  }
  for (const auto & p : polygon) {
    c.x += p.x;
    c.y += p.y;
  }
  c.x /= static_cast<double>(polygon.size());
  c.y /= static_cast<double>(polygon.size());
  return c;
}

bool TopologyGlobalPlanner::validateSmoothedPlan(
  const TopologyPlan & reference, const TopologyPlan & candidate) const
{
  if (candidate.segments.size() != reference.segments.size() || candidate.segments.empty()) {
    return false;
  }
  for (size_t i = 0; i < candidate.segments.size(); ++i) {
    const auto & segment = candidate.segments[i];
    const auto & original = reference.segments[i];
    if (segment.path.poses.empty() || segment.type != original.type ||
        segment.connector_id != original.connector_id || segment.action != original.action ||
        segment.from_region != original.from_region || segment.to_region != original.to_region ||
        segment.wait_pose != original.wait_pose || segment.exit_pose != original.exit_pose ||
        segment.corridor_half_width != original.corridor_half_width)
    {
      return false;
    }
    if (!use_topology_) {
      continue;
    }
    if (segment.type == SegmentType::CONNECTOR) {
      const auto & wait = segment.wait_pose.pose.position;
      const auto & exit = segment.exit_pose.pose.position;
      const double length = std::hypot(exit.x - wait.x, exit.y - wait.y);
      const Point2D begin{wait.x - (exit.x - wait.x) / length * connector_entry_clearance_,
        wait.y - (exit.y - wait.y) / length * connector_entry_clearance_};
      if (!pathInsideConnectorStrip(
            segment.path, begin, {exit.x, exit.y}, segment.corridor_half_width))
      {
        return false;
      }
    } else {
      // 按相邻通道确定区域
      // 避免重叠区域的归属发生变化
      std::string region;
      if (i > 0 && reference.segments[i - 1].type == SegmentType::CONNECTOR) {
        region = reference.segments[i - 1].to_region;
      } else if (i + 1 < reference.segments.size() &&
                 reference.segments[i + 1].type == SegmentType::CONNECTOR)
      {
        region = reference.segments[i + 1].from_region;
      } else {
        const auto & p = original.path.poses[original.path.poses.size() / 2].pose.position;
        region = findRegion(p.x, p.y);
      }
      if (!pathInsideRegionWithTolerance(segment.path, region, region_constraint_tolerance_)) {
        return false;
      }
    }
  }
  return true;
}

}
