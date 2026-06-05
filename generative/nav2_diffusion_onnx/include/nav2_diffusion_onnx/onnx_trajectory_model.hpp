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

#ifndef NAV2_DIFFUSION_ONNX__ONNX_TRAJECTORY_MODEL_HPP_
#define NAV2_DIFFUSION_ONNX__ONNX_TRAJECTORY_MODEL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_diffusion_core/trajectory_model.hpp"
#include "onnxruntime_cxx_api.h"  // NOLINT(build/include_subdir) external header

namespace nav2_diffusion_onnx
{

/// ONNX Runtime implementation of nav2_diffusion_core::TrajectoryModel
/// (docs/architecture.md sections 5.2/7.2). Loads a model that maps a context
/// vector [goal_x, goal_y, linear_speed, max_angular_speed] to a [1, K, H, 3]
/// trajectory tensor (K candidates, H steps, x/y/yaw in the base frame) and
/// turns it into Trajectory candidates. The per-step time uses
/// ModelContext::time_step.
///
/// Default-constructible so it can be loaded via pluginlib; call configure()
/// with the ONNX model path before generate().
class OnnxTrajectoryModel : public nav2_diffusion_core::TrajectoryModel
{
public:
  OnnxTrajectoryModel() = default;
  explicit OnnxTrajectoryModel(const std::string & model_path);

  void configure(const std::string & model_path) override;
  std::string name() const override;
  std::vector<nav2_diffusion_core::Trajectory> generate(
    const nav2_diffusion_core::ModelContext & context) const override;

private:
  std::shared_ptr<Ort::Env> env_;
  mutable std::shared_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  // Optional second input for costmap-conditioned models (input named "costmap").
  bool has_costmap_input_{false};
  std::string costmap_name_;
  int costmap_dim_{0};  // expected square patch side length
};

}  // namespace nav2_diffusion_onnx

#endif  // NAV2_DIFFUSION_ONNX__ONNX_TRAJECTORY_MODEL_HPP_
