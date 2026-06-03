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

#ifndef NAV2_DIFFUSION_ONNX__ONNX_PATH_MODEL_HPP_
#define NAV2_DIFFUSION_ONNX__ONNX_PATH_MODEL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_diffusion_core/path_model.hpp"
#include "onnxruntime_cxx_api.h"  // NOLINT(build/include_subdir) external header

namespace nav2_diffusion_onnx
{

/// ONNX Runtime implementation of nav2_diffusion_core::PathModel (Nav2 Mode B).
/// Loads a generative global-path model that maps a goal-aligned context
/// [goal_distance, 0] to a [1, K, H, 2] tensor of K candidate paths (H waypoints,
/// x/y in the goal-aligned frame, goal at (d, 0)). generate() rotates and
/// translates each candidate back into the map frame using the start pose and
/// goal bearing, and snaps the endpoints exactly onto start/goal.
///
/// Default-constructible so it can be loaded via pluginlib by the
/// DiffusionGlobalPlanner (model_plugin); call configure() with the ONNX path.
class OnnxPathModel : public nav2_diffusion_core::PathModel
{
public:
  OnnxPathModel() = default;
  explicit OnnxPathModel(const std::string & model_path);

  void configure(const std::string & model_path) override;
  std::string name() const override;
  std::vector<nav2_diffusion_core::PathCandidate> generate(
    const nav2_diffusion_core::PathContext & context) const override;

private:
  std::shared_ptr<Ort::Env> env_;
  mutable std::shared_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
};

}  // namespace nav2_diffusion_onnx

#endif  // NAV2_DIFFUSION_ONNX__ONNX_PATH_MODEL_HPP_
