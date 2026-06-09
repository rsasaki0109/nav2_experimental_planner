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

#ifndef NAV2_DIFFUSION_CORE__TRAJECTORY_HPP_
#define NAV2_DIFFUSION_CORE__TRAJECTORY_HPP_

#include <cstddef>
#include <vector>

namespace nav2_diffusion_core
{

/// A single time-indexed SE(2) trajectory sample.
struct TrajectoryPoint
{
  double x{0.0};     ///< metres, in the trajectory frame
  double y{0.0};     ///< metres, in the trajectory frame
  double yaw{0.0};   ///< radians
  double time{0.0};  ///< seconds, offset from the trajectory start
};

/// A time-indexed SE(2) trajectory candidate.
///
/// This is the primary internal representation described in
/// docs/architecture.md section 4.4: a future pose sequence rather than a raw
/// velocity command. The cmd_vel is derived downstream by the Command
/// Extractor. ROS message conversions live outside this (ROS-light) package.
struct Trajectory
{
  std::vector<TrajectoryPoint> points;

  /// Generative model score / unnormalized likelihood. This is a soft prior
  /// signal only; it does NOT imply safety. Safety is decided by the
  /// deterministic safety layer (see nav2_diffusion_safety).
  double model_score{0.0};

  bool empty() const {return points.empty();}
  std::size_t size() const {return points.size();}
};

/// Total arc length of the trajectory, in metres.
double pathLength(const Trajectory & trajectory);

/// Time span between the first and last sample, in seconds.
/// Returns 0.0 when the trajectory has fewer than two samples.
double duration(const Trajectory & trajectory);

}  // namespace nav2_diffusion_core

#endif  // NAV2_DIFFUSION_CORE__TRAJECTORY_HPP_
