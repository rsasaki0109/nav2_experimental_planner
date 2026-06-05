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

#include "nav2_diffusion_core/scoring.hpp"

#include <cmath>
#include <cstddef>

namespace nav2_diffusion_core
{

double endpointDistance(const Trajectory & trajectory, double goal_x, double goal_y)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  const auto & last = trajectory.points.back();
  return std::hypot(goal_x - last.x, goal_y - last.y);
}

double totalTurning(const Trajectory & trajectory)
{
  double turning = 0.0;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    turning += std::abs(trajectory.points[i].yaw - trajectory.points[i - 1].yaw);
  }
  return turning;
}

double scoreTrajectory(
  const Trajectory & trajectory, double goal_x, double goal_y,
  const ScoringWeights & weights)
{
  const double progress_cost = weights.progress * endpointDistance(trajectory, goal_x, goal_y);
  const double smoothness_cost = weights.smoothness * totalTurning(trajectory);
  return -(progress_cost + smoothness_cost);
}

}  // namespace nav2_diffusion_core
