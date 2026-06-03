// Copyright 2026 nav2_diffusion_planner contributors
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

#ifndef NAV2_DIFFUSION_SAFETY__FOOTPRINT_COLLISION_FILTER_HPP_
#define NAV2_DIFFUSION_SAFETY__FOOTPRINT_COLLISION_FILTER_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_diffusion_safety/safety_filter.hpp"

namespace nav2_diffusion_safety
{

/// Rejects trajectories whose oriented robot footprint collides with the
/// costmap. This is the Footprint Collision Layer of docs/safety.md section 8.2
/// and treats the costmap as a runtime truth source (docs/architecture.md
/// section 3.4).
///
/// The trajectory points passed to check() must be expressed in the costmap's
/// global frame. The caller is responsible for holding the costmap mutex
/// (Costmap2D::getMutex()) for the duration of the check(), because the costmap
/// is updated concurrently by Nav2.
class FootprintCollisionFilter : public SafetyFilter
{
public:
  using Footprint = std::vector<geometry_msgs::msg::Point>;

  /// @param costmap Live costmap used as the truth source (not owned).
  /// @param footprint Robot footprint in the robot frame.
  /// @param lethal_threshold Costs at or above this value are collisions.
  /// @param consider_unknown_lethal Treat NO_INFORMATION cells as collisions.
  FootprintCollisionFilter(
    nav2_costmap_2d::Costmap2D * costmap,
    Footprint footprint,
    double lethal_threshold = 253.0,
    bool consider_unknown_lethal = false);

  void setFootprint(const Footprint & footprint);
  void setCostmap(nav2_costmap_2d::Costmap2D * costmap);

  std::string name() const override;
  SafetyResult check(const nav2_diffusion_core::Trajectory & trajectory) const override;

private:
  nav2_costmap_2d::Costmap2D * costmap_;
  Footprint footprint_;
  double lethal_threshold_;
  bool consider_unknown_lethal_;
};

}  // namespace nav2_diffusion_safety

#endif  // NAV2_DIFFUSION_SAFETY__FOOTPRINT_COLLISION_FILTER_HPP_
