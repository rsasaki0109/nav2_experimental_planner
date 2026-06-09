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

#include "nav2_diffusion_safety/kinematic_limits_filter.hpp"

#include <cmath>
#include <cstddef>
#include <string>

namespace nav2_diffusion_safety
{

namespace
{

/// Shortest signed angular difference from `from` to `to`, in (-pi, pi].
double angularDistance(double from, double to)
{
  double diff = std::fmod(to - from + M_PI, 2.0 * M_PI);
  if (diff < 0.0) {
    diff += 2.0 * M_PI;
  }
  return diff - M_PI;
}

}  // namespace

KinematicLimitsFilter::KinematicLimitsFilter(
  double max_linear_speed, double max_angular_speed)
: max_linear_speed_(max_linear_speed),
  max_angular_speed_(max_angular_speed)
{
}

std::string KinematicLimitsFilter::name() const
{
  return "kinematic_limits";
}

SafetyResult KinematicLimitsFilter::check(
  const nav2_diffusion_core::Trajectory & trajectory) const
{
  const auto & points = trajectory.points;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dt = points[i].time - points[i - 1].time;
    if (dt <= 0.0) {
      return SafetyResult::rejected("non-increasing time samples");
    }

    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double linear_speed = std::hypot(dx, dy) / dt;
    if (linear_speed > max_linear_speed_) {
      return SafetyResult::rejected("linear speed limit exceeded");
    }

    const double dyaw = std::abs(angularDistance(points[i - 1].yaw, points[i].yaw));
    const double angular_speed = dyaw / dt;
    if (angular_speed > max_angular_speed_) {
      return SafetyResult::rejected("angular speed limit exceeded");
    }
  }
  return SafetyResult::accepted();
}

}  // namespace nav2_diffusion_safety
