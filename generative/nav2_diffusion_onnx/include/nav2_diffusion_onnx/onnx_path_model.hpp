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
/// Costmap-conditioned models that export a second input named "costmap" are
/// auto-detected: the global costmap carried in PathContext is resampled into a
/// goal-aligned S x S patch (aligned x in [0, PATCH_FWD], y in
/// [-PATCH_HALF, PATCH_HALF]) and fed alongside the context. Context-only models
/// (no "costmap" input) are unaffected.
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

  // Goal-aligned patch window, shared with the training side
  // (nav2_diffusion_training.path_planners). The patch covers aligned x in
  // [0, kPatchFwd] and y in [-kPatchHalf, kPatchHalf]; row -> x, col -> y.
  static constexpr double kPatchFwd = 6.0;
  static constexpr double kPatchHalf = 3.0;

private:
  std::shared_ptr<Ort::Env> env_;
  mutable std::shared_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  bool has_costmap_input_{false};
  std::string costmap_name_;
  int costmap_dim_{0};  // expected square patch side length
};

}  // namespace nav2_diffusion_onnx

#endif  // NAV2_DIFFUSION_ONNX__ONNX_PATH_MODEL_HPP_
