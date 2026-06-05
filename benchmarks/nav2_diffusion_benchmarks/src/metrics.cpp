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

#include "nav2_diffusion_benchmarks/metrics.hpp"

#include <cmath>
#include <cstddef>

#include "nav2_diffusion_core/scoring.hpp"

namespace nav2_diffusion_benchmarks
{

namespace
{

/// Wrap an angle difference to (-pi, pi].
double normalizeAngle(double angle)
{
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

int signOf(double value, double epsilon)
{
  if (value > epsilon) {
    return 1;
  }
  if (value < -epsilon) {
    return -1;
  }
  return 0;
}

}  // namespace

RunMetrics evaluateRun(
  const nav2_diffusion_core::Trajectory & executed,
  double goal_x, double goal_y, double goal_tolerance,
  double stop_speed_threshold)
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

  constexpr double kEpsilon = 1e-6;
  const auto & points = executed.points;
  int last_turn_sign = 0;
  int last_dir_sign = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dt = points[i].time - points[i - 1].time;
    if (dt <= 0.0) {
      continue;
    }

    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double speed = std::hypot(dx, dy) / dt;
    if (speed < stop_speed_threshold) {
      metrics.stop_duration += dt;
    }

    // Forward vs backward motion relative to the previous heading.
    const double forward = dx * std::cos(points[i - 1].yaw) + dy * std::sin(points[i - 1].yaw);
    const int dir_sign = signOf(forward, kEpsilon);
    if (dir_sign != 0) {
      if (last_dir_sign != 0 && dir_sign != last_dir_sign) {
        ++metrics.direction_changes;
      }
      last_dir_sign = dir_sign;
    }

    const double dyaw = normalizeAngle(points[i].yaw - points[i - 1].yaw);
    const int turn_sign = signOf(dyaw, kEpsilon);
    if (turn_sign != 0) {
      if (last_turn_sign != 0 && turn_sign != last_turn_sign) {
        ++metrics.oscillation_count;
      }
      last_turn_sign = turn_sign;
    }
  }

  return metrics;
}

}  // namespace nav2_diffusion_benchmarks
