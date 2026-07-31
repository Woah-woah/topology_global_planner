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

bool TopologyGlobalPlanner::hasRegion(const std::string & id) const
{
  return std::any_of(regions_.begin(), regions_.end(), [&](const Region & r) {return r.id == id;});
}

std::string TopologyGlobalPlanner::findRegion(double x, double y) const
{
  std::vector<std::string> inside_candidates;
  std::vector<std::string> boundary_candidates;

  for (const auto & region : regions_) {
    if (pointInPolygon(x, y, region.polygon)) {
      inside_candidates.push_back(region.id);
    } else if (pointOnPolygonBoundary(x, y, region.polygon)) {
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
      RCLCPP_DEBUG(
        logger_, "Point (%.3f, %.3f) is on multiple region boundaries; choosing nearest centroid.", x, y);
    }
    return chooseBestRegionByCentroid(x, y, boundary_candidates);
  }

  if (!allow_nearest_region_fallback_ || regions_.empty()) {
    return "";
  }

  double best_dist = std::numeric_limits<double>::infinity();
  std::string best_id;
  for (const auto & region : regions_) {
    const double d = euclidean(x, y, region.centroid.x, region.centroid.y);
    if (d < best_dist) {
      best_dist = d;
      best_id = region.id;
    }
  }

  if (best_dist <= std::max(0.0, max_nearest_region_distance_)) {
    RCLCPP_DEBUG(
      logger_, "Point (%.3f, %.3f) is outside regions; using nearest region '%s' at %.3fm.",
      x, y, best_id.c_str(), best_dist);
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
    const auto it = std::find_if(regions_.begin(), regions_.end(),
      [&](const Region & r) {return r.id == id;});
    if (it == regions_.end()) {
      continue;
    }

    const double d = euclidean(x, y, it->centroid.x, it->centroid.y);
    if (d < best_dist) {
      best_dist = d;
      best_id = id;
    }
  }

  return best_id;
}

bool TopologyGlobalPlanner::pointInPolygon(double x, double y, const std::vector<Point2D> & polygon) const
{
  // Standard ray casting. Horizontal edges are naturally ignored by the first condition,
  // so no artificial epsilon is added to the denominator. Boundary handling is done
  // separately in pointOnPolygonBoundary().
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

bool TopologyGlobalPlanner::pointOnPolygonBoundary(double x, double y, const std::vector<Point2D> & polygon) const
{
  return pointOnPolygonBoundary(x, y, polygon, region_boundary_tolerance_);
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
  const auto it = std::find_if(
    regions_.begin(), regions_.end(),
    [&](const Region & r) {return r.id == id;});
  if (it == regions_.end()) {
    return nullptr;
  }
  return &(*it);
}

bool TopologyGlobalPlanner::pointInRegionWithTolerance(
  double x, double y, const std::string & region_id, double tolerance) const
{
  const auto * region = getRegionById(region_id);
  if (region == nullptr) {
    return false;
  }

  return pointInPolygon(x, y, region->polygon) ||
         pointOnPolygonBoundary(x, y, region->polygon, tolerance);
}

bool TopologyGlobalPlanner::pathInsideRegionWithTolerance(
  const nav_msgs::msg::Path & path, const std::string & region_id, double tolerance) const
{
  if (region_id.empty()) {
    return true;
  }

  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    if (!pointInRegionWithTolerance(p.x, p.y, region_id, tolerance)) {
      return false;
    }
  }

  return true;
}

bool TopologyGlobalPlanner::pathInsideConnectorStrip(
  const nav_msgs::msg::Path & path,
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

  for (const auto & pose : path.poses) {
    const double px = pose.pose.position.x - wait.x;
    const double py = pose.pose.position.y - wait.y;

    const double t = (px * vx + py * vy) / length_sq;

    // 不允许跑到 wait 后面或 exit 后面太远
    if (t < -0.50 || t > 1.50) {
      return false;
    }

    const double clamped_t = std::clamp(t, 0.0, 1.0);
    const double proj_x = wait.x + clamped_t * vx;
    const double proj_y = wait.y + clamped_t * vy;

    const double lateral_distance = euclidean(
      pose.pose.position.x,
      pose.pose.position.y,
      proj_x,
      proj_y);

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
    return euclidean(p.x, p.y, a.x, a.y);
  }
  const double c2 = vx * vx + vy * vy;
  if (c2 <= 1e-12) {
    return euclidean(p.x, p.y, a.x, a.y);
  }
  if (c2 <= c1) {
    return euclidean(p.x, p.y, b.x, b.y);
  }
  const double t = c1 / c2;
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  return euclidean(p.x, p.y, proj_x, proj_y);
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

double TopologyGlobalPlanner::euclidean(double x1, double y1, double x2, double y2) const
{
  return std::hypot(x1 - x2, y1 - y2);
}


}  // namespace topology_global_planner
