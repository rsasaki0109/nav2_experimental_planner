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

#ifndef NAV2_DIFFUSION_CORE__TRAJECTORY_MODEL_HPP_
#define NAV2_DIFFUSION_CORE__TRAJECTORY_MODEL_HPP_

#include <string>
#include <vector>

#include "nav2_diffusion_core/trajectory.hpp"

namespace nav2_diffusion_core
{

/// Conditioning passed to a generative trajectory model. Kept minimal and
/// ROS-independent so the same interface works for the built-in analytic model
/// and for learned backends (PyTorch / ONNX / TensorRT) added later
/// (docs/architecture.md sections 5.2 and 5.3).
struct ModelContext
{
  double goal_x{0.0};            ///< local goal in the robot base frame [m]
  double goal_y{0.0};            ///< local goal in the robot base frame [m]
  double linear_speed{0.0};      ///< desired forward speed [m/s]
  double max_angular_speed{0.0};  ///< angular speed limit [rad/s]
  double horizon{0.0};           ///< prediction horizon [s]
  double time_step{0.1};         ///< rollout discretization [s]
  int num_candidates{1};         ///< number of trajectories to generate
};

/// Abstract generative trajectory model: the "propose" stage of the pipeline.
/// Produces K candidate trajectories in the base frame; the safety layer and
/// scorer decide which (if any) is executed.
class TrajectoryModel
{
public:
  virtual ~TrajectoryModel() = default;

  /// One-time setup for pluginlib-loaded models. The argument is a model
  /// resource locator (e.g. an ONNX file path); built-in analytic models ignore
  /// it. Plugins are default-constructed, so any heavy initialization that needs
  /// the model path belongs here, not in the constructor.
  virtual void configure(const std::string & model_path) {(void)model_path;}

  /// Short identifier for diagnostics / model registry.
  virtual std::string name() const = 0;

  /// Generate candidate trajectories (base frame) for the given context.
  virtual std::vector<Trajectory> generate(const ModelContext & context) const = 0;
};

}  // namespace nav2_diffusion_core

#endif  // NAV2_DIFFUSION_CORE__TRAJECTORY_MODEL_HPP_
