// Copyright 2026 nav2_experimental_planner contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_diffusion_benchmarks/collision_metrics.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace nav2_diffusion_benchmarks
{

namespace
{

/// Distance from (wx, wy) to the nearest LETHAL cell within max_clearance,
/// saturating at max_clearance when none is found.
double nearestLethalDistance(
  nav2_costmap_2d::Costmap2D * costmap, double wx, double wy, double max_clearance)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    return max_clearance;
  }

  const double resolution = costmap->getResolution();
  const int radius_cells = static_cast<int>(std::ceil(max_clearance / resolution));
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());

  double best = max_clearance;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    const int cy = static_cast<int>(my) + dy;
    if (cy < 0 || cy >= size_y) {
      continue;
    }
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int cx = static_cast<int>(mx) + dx;
      if (cx < 0 || cx >= size_x) {
        continue;
      }
      if (costmap->getCost(cx, cy) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        best = std::min(best, std::hypot(dx, dy) * resolution);
      }
    }
  }
  return best;
}

}  // namespace

CollisionMetrics evaluateCollisions(
  const nav2_diffusion_core::Trajectory & executed,
  nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double lethal_threshold, double max_clearance)
{
  CollisionMetrics metrics;
  metrics.min_clearance = max_clearance;
  if (executed.points.empty() || costmap == nullptr) {
    return metrics;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  for (const auto & point : executed.points) {
    const double cost = checker.footprintCostAtPose(point.x, point.y, point.yaw, footprint);
    if (cost >= lethal_threshold &&
      cost != static_cast<double>(nav2_costmap_2d::NO_INFORMATION))
    {
      ++metrics.collision_count;
    }
    metrics.min_clearance = std::min(
      metrics.min_clearance, nearestLethalDistance(costmap, point.x, point.y, max_clearance));
  }
  metrics.collided = metrics.collision_count > 0;
  return metrics;
}

}  // namespace nav2_diffusion_benchmarks
