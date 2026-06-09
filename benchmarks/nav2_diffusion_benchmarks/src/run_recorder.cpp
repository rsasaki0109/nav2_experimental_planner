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

#include "nav2_diffusion_benchmarks/run_recorder.hpp"

#include <string>

#include "nav2_diffusion_benchmarks/metrics.hpp"

namespace nav2_diffusion_benchmarks
{

void RunRecorder::reset()
{
  path_.points.clear();
}

void RunRecorder::addSample(double time, double x, double y, double yaw)
{
  path_.points.push_back({x, y, yaw, time});
}

std::size_t RunRecorder::size() const
{
  return path_.points.size();
}

bool RunRecorder::empty() const
{
  return path_.points.empty();
}

const nav2_diffusion_core::Trajectory & RunRecorder::path() const
{
  return path_;
}

RunResult RunRecorder::finish(
  const std::string & scenario, const std::string & controller,
  double goal_x, double goal_y, double goal_tolerance,
  double stop_speed_threshold) const
{
  RunResult result;
  result.scenario = scenario;
  result.controller = controller;
  result.metrics = evaluateRun(path_, goal_x, goal_y, goal_tolerance, stop_speed_threshold);
  return result;
}

}  // namespace nav2_diffusion_benchmarks
