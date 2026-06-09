// Copyright 2026 Nav2PlannerBattle contributors
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

#ifndef NAV2_DIFFUSION_BENCHMARKS__COLLISION_METRICS_HPP_
#define NAV2_DIFFUSION_BENCHMARKS__COLLISION_METRICS_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_diffusion_core/trajectory.hpp"

namespace nav2_diffusion_benchmarks
{

/// Costmap-based safety metrics for an executed run (docs/benchmarking.md
/// section 9.4, Safety category). The executed trajectory must be expressed in
/// the costmap's frame.
struct CollisionMetrics
{
  int collision_count{0};      ///< path poses whose footprint hits an obstacle
  bool collided{false};        ///< collision_count > 0
  double min_clearance{0.0};   ///< min distance from robot centre to a lethal cell [m]
};

/// Evaluate footprint collisions and clearance along an executed path.
///
/// @param executed Path the robot drove, in the costmap frame.
/// @param costmap Costmap holding the obstacles (not owned).
/// @param footprint Robot footprint in the robot frame.
/// @param lethal_threshold Footprint costs at/above this count as a collision.
/// @param max_clearance Clearance search radius and cap [m]; cells beyond it are
///        ignored and the reported clearance saturates here.
/// @returns all-default metrics (clearance = max_clearance) for an empty path or
///          null costmap.
CollisionMetrics evaluateCollisions(
  const nav2_diffusion_core::Trajectory & executed,
  nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double lethal_threshold = 253.0,
  double max_clearance = 2.0);

}  // namespace nav2_diffusion_benchmarks

#endif  // NAV2_DIFFUSION_BENCHMARKS__COLLISION_METRICS_HPP_
