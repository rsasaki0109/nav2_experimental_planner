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

#include "nav2_diffusion_benchmarks/metrics.hpp"

#include <cmath>

#include "nav2_diffusion_core/scoring.hpp"

namespace nav2_diffusion_benchmarks
{

RunMetrics evaluateRun(
  const nav2_diffusion_core::Trajectory & executed,
  double goal_x, double goal_y, double goal_tolerance)
{
  RunMetrics metrics;
  if (executed.points.empty()) {
    return metrics;
  }

  metrics.goal_distance = nav2_diffusion_core::endpointDistance(executed, goal_x, goal_y);
  metrics.reached_goal = metrics.goal_distance <= goal_tolerance;
  metrics.time_to_goal = nav2_diffusion_core::duration(executed);
  metrics.path_length = nav2_diffusion_core::pathLength(executed);
  metrics.total_turning = nav2_diffusion_core::totalTurning(executed);

  const auto & start = executed.points.front();
  const double straight_line = std::hypot(goal_x - start.x, goal_y - start.y);
  constexpr double kMinStraightLine = 1e-3;
  metrics.detour_ratio =
    straight_line > kMinStraightLine ? metrics.path_length / straight_line : 1.0;

  return metrics;
}

}  // namespace nav2_diffusion_benchmarks
