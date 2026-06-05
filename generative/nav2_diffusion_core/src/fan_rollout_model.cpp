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

#include "nav2_diffusion_core/fan_rollout_model.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_diffusion_core
{

namespace
{

Trajectory rollout(double linear, double angular, double horizon, double time_step)
{
  Trajectory trajectory;
  const double dt = time_step > 1e-6 ? time_step : 0.1;
  const int steps = std::max(1, static_cast<int>(horizon / dt));

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  trajectory.points.push_back({x, y, yaw, 0.0});
  for (int i = 1; i <= steps; ++i) {
    yaw += angular * dt;
    x += linear * std::cos(yaw) * dt;
    y += linear * std::sin(yaw) * dt;
    trajectory.points.push_back({x, y, yaw, i * dt});
  }
  return trajectory;
}

}  // namespace

std::string FanRolloutModel::name() const
{
  return "fan_rollout";
}

std::vector<Trajectory> FanRolloutModel::generate(const ModelContext & context) const
{
  const double dist_sq = context.goal_x * context.goal_x + context.goal_y * context.goal_y;
  const double nominal_curvature = dist_sq > 1e-6 ? 2.0 * context.goal_y / dist_sq : 0.0;
  const double nominal_angular = std::clamp(
    context.linear_speed * nominal_curvature,
    -context.max_angular_speed, context.max_angular_speed);

  const int count = std::max(1, context.num_candidates);
  std::vector<Trajectory> candidates;
  candidates.reserve(count);
  for (int i = 0; i < count; ++i) {
    double angular = nominal_angular;
    if (count > 1) {
      const double frac = static_cast<double>(i) / static_cast<double>(count - 1);
      angular = -context.max_angular_speed + frac * (2.0 * context.max_angular_speed);
    }
    candidates.push_back(
      rollout(context.linear_speed, angular, context.horizon, context.time_step));
  }
  return candidates;
}

}  // namespace nav2_diffusion_core
