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

#ifndef NAV2_DIFFUSION_BENCHMARKS__METRICS_HPP_
#define NAV2_DIFFUSION_BENCHMARKS__METRICS_HPP_

#include "nav2_diffusion_core/trajectory.hpp"

namespace nav2_diffusion_benchmarks
{

/// Geometry-only metrics for one executed run, computed from the recorded
/// time-indexed SE(2) path. A subset of docs/benchmarking.md section 9.4 that
/// needs no costmap; clearance/collision/social metrics are added separately
/// when a costmap or obstacle log is available.
struct RunMetrics
{
  bool reached_goal{false};   ///< final pose within goal_tolerance of the goal
  double goal_distance{0.0};  ///< final pose distance to the goal [m]
  double time_to_goal{0.0};   ///< executed duration [s]
  double path_length{0.0};    ///< total path length [m]
  double detour_ratio{1.0};   ///< path_length / straight-line(start, goal)
  double total_turning{0.0};  ///< sum of absolute heading changes [rad]
  int oscillation_count{0};   ///< sign changes of angular velocity (wiggling)
  int direction_changes{0};   ///< forward/backward reversals (cusps)
  double stop_duration{0.0};  ///< time spent below stop_speed_threshold [s]
};

/// Evaluate an executed run against a goal. The trajectory is the path the robot
/// actually drove (time-indexed SE(2) samples). Returns all-default metrics for
/// an empty trajectory.
///
/// @param stop_speed_threshold Segments slower than this count toward
///        stop_duration [m/s].
RunMetrics evaluateRun(
  const nav2_diffusion_core::Trajectory & executed,
  double goal_x, double goal_y, double goal_tolerance,
  double stop_speed_threshold = 0.01);

}  // namespace nav2_diffusion_benchmarks

#endif  // NAV2_DIFFUSION_BENCHMARKS__METRICS_HPP_
