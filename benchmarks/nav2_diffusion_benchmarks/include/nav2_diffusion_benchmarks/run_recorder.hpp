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

#ifndef NAV2_DIFFUSION_BENCHMARKS__RUN_RECORDER_HPP_
#define NAV2_DIFFUSION_BENCHMARKS__RUN_RECORDER_HPP_

#include <cstddef>
#include <string>

#include "nav2_diffusion_benchmarks/run_result.hpp"
#include "nav2_diffusion_core/trajectory.hpp"

namespace nav2_diffusion_benchmarks
{

/// Accumulates a stream of robot poses (e.g. from /odom or a rosbag) into an
/// executed trajectory and turns it into a RunResult. ROS-independent so it can
/// be unit-tested; the runner node feeds it live data.
class RunRecorder
{
public:
  /// Discard all recorded samples.
  void reset();

  /// Append one executed pose. Samples are expected in non-decreasing time.
  void addSample(double time, double x, double y, double yaw);

  std::size_t size() const;
  bool empty() const;
  const nav2_diffusion_core::Trajectory & path() const;

  /// Evaluate the recorded path against a goal and return a labelled result.
  /// Collision metrics are left at defaults (no costmap here); compute them
  /// separately with evaluateCollisions when a costmap is available.
  RunResult finish(
    const std::string & scenario, const std::string & controller,
    double goal_x, double goal_y, double goal_tolerance,
    double stop_speed_threshold = 0.01) const;

private:
  nav2_diffusion_core::Trajectory path_;
};

}  // namespace nav2_diffusion_benchmarks

#endif  // NAV2_DIFFUSION_BENCHMARKS__RUN_RECORDER_HPP_
