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

#ifndef NAV2_DIFFUSION_SAFETY__KINEMATIC_LIMITS_FILTER_HPP_
#define NAV2_DIFFUSION_SAFETY__KINEMATIC_LIMITS_FILTER_HPP_

#include <string>

#include "nav2_diffusion_safety/safety_filter.hpp"

namespace nav2_diffusion_safety
{

/// Rejects trajectories whose finite-difference translational or rotational
/// speed exceeds the configured limits. This is the Kinematic Safety Layer of
/// docs/safety.md section 8.2.
class KinematicLimitsFilter : public SafetyFilter
{
public:
  KinematicLimitsFilter(double max_linear_speed, double max_angular_speed);

  std::string name() const override;
  SafetyResult check(const nav2_diffusion_core::Trajectory & trajectory) const override;

private:
  double max_linear_speed_;   ///< m/s
  double max_angular_speed_;  ///< rad/s
};

}  // namespace nav2_diffusion_safety

#endif  // NAV2_DIFFUSION_SAFETY__KINEMATIC_LIMITS_FILTER_HPP_
