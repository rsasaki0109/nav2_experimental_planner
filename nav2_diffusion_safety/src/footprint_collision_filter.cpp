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

#include "nav2_diffusion_safety/footprint_collision_filter.hpp"

#include <string>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace nav2_diffusion_safety
{

FootprintCollisionFilter::FootprintCollisionFilter(
  nav2_costmap_2d::Costmap2D * costmap, Footprint footprint,
  double lethal_threshold, bool consider_unknown_lethal)
: costmap_(costmap),
  footprint_(std::move(footprint)),
  lethal_threshold_(lethal_threshold),
  consider_unknown_lethal_(consider_unknown_lethal)
{
}

void FootprintCollisionFilter::setFootprint(const Footprint & footprint)
{
  footprint_ = footprint;
}

void FootprintCollisionFilter::setCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
}

std::string FootprintCollisionFilter::name() const
{
  return "footprint_collision";
}

SafetyResult FootprintCollisionFilter::check(
  const nav2_diffusion_core::Trajectory & trajectory) const
{
  if (costmap_ == nullptr) {
    return SafetyResult::rejected("no costmap");
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_);
  for (const auto & point : trajectory.points) {
    const double cost = checker.footprintCostAtPose(point.x, point.y, point.yaw, footprint_);
    if (cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
      if (consider_unknown_lethal_) {
        return SafetyResult::rejected("footprint over unknown space");
      }
      continue;
    }
    if (cost >= lethal_threshold_) {
      return SafetyResult::rejected("footprint collision");
    }
  }
  return SafetyResult::accepted();
}

}  // namespace nav2_diffusion_safety
