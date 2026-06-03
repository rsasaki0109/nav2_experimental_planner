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

#ifndef NAV2_DIFFUSION_CORE__FAN_ROLLOUT_MODEL_HPP_
#define NAV2_DIFFUSION_CORE__FAN_ROLLOUT_MODEL_HPP_

#include <string>
#include <vector>

#include "nav2_diffusion_core/trajectory_model.hpp"

namespace nav2_diffusion_core
{

/// Built-in analytic trajectory model: samples a fan of constant angular
/// velocities toward the goal and forward-simulates each into an SE(2)
/// trajectory. This is the placeholder generative model that lets the full
/// pipeline (safety, scoring, fallback, benchmarking) run before a learned
/// model is plugged in behind the same TrajectoryModel interface.
class FanRolloutModel : public TrajectoryModel
{
public:
  std::string name() const override;
  std::vector<Trajectory> generate(const ModelContext & context) const override;
};

}  // namespace nav2_diffusion_core

#endif  // NAV2_DIFFUSION_CORE__FAN_ROLLOUT_MODEL_HPP_
