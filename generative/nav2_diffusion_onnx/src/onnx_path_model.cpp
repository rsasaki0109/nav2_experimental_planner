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

#include "nav2_diffusion_onnx/onnx_path_model.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_diffusion_onnx
{

OnnxPathModel::OnnxPathModel(const std::string & model_path)
{
  configure(model_path);
}

void OnnxPathModel::configure(const std::string & model_path)
{
  env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "nav2_diffusion_onnx_path");
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  session_ = std::make_shared<Ort::Session>(*env_, model_path.c_str(), options);

  Ort::AllocatorWithDefaultOptions allocator;
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

  // Identify the context input and an optional "costmap" input by name so that
  // both context-only and costmap-conditioned models load through this backend.
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  const std::size_t input_count = session_->GetInputCount();
  for (std::size_t i = 0; i < input_count; ++i) {
    const std::string name = session_->GetInputNameAllocated(i, allocator).get();
    if (name == "context") {
      input_name_ = name;
    } else if (name == "costmap") {
      has_costmap_input_ = true;
      costmap_name_ = name;
      const std::vector<int64_t> shape =
        session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      costmap_dim_ = shape.empty() ? 0 : static_cast<int>(shape.back());
    }
  }
}

namespace
{

/// Resample the global costmap (carried in the PathContext, normalized [0, 1])
/// into a goal-aligned S x S patch: row -> forward x in [0, fwd], col -> lateral
/// y in [-half, half]. Cells outside the costmap read as 0 (free).
std::vector<float> alignedPatch(
  const nav2_diffusion_core::PathContext & ctx, int size, double fwd, double half,
  double cos_b, double sin_b)
{
  std::vector<float> patch(static_cast<std::size_t>(size) * size, 0.0f);
  if (ctx.costmap.empty() || ctx.costmap_resolution <= 0.0 ||
    ctx.costmap_size_x == 0 || ctx.costmap_size_y == 0)
  {
    return patch;
  }
  for (int row = 0; row < size; ++row) {
    const double ax = fwd * (row + 0.5) / size;
    for (int col = 0; col < size; ++col) {
      const double ay = -half + 2.0 * half * (col + 0.5) / size;
      const double wx = ctx.start_x + ax * cos_b - ay * sin_b;
      const double wy = ctx.start_y + ax * sin_b + ay * cos_b;
      const int mx = static_cast<int>((wx - ctx.costmap_origin_x) / ctx.costmap_resolution);
      const int my = static_cast<int>((wy - ctx.costmap_origin_y) / ctx.costmap_resolution);
      if (mx >= 0 && my >= 0 &&
        mx < static_cast<int>(ctx.costmap_size_x) && my < static_cast<int>(ctx.costmap_size_y))
      {
        patch[static_cast<std::size_t>(row) * size + col] =
          ctx.costmap[static_cast<std::size_t>(my) * ctx.costmap_size_x + mx];
      }
    }
  }
  return patch;
}

}  // namespace

std::string OnnxPathModel::name() const
{
  return "onnx_path";
}

std::vector<nav2_diffusion_core::PathCandidate> OnnxPathModel::generate(
  const nav2_diffusion_core::PathContext & context) const
{
  std::vector<nav2_diffusion_core::PathCandidate> candidates;
  if (session_ == nullptr) {
    return candidates;
  }

  // Work in a goal-aligned frame: the model sees only the goal distance and
  // outputs paths from (0, 0) to (d, 0); we rotate/translate back to the map.
  const double dx = context.goal_x - context.start_x;
  const double dy = context.goal_y - context.start_y;
  const double dist = std::hypot(dx, dy);
  const double bearing = std::atan2(dy, dx);
  const double cos_b = std::cos(bearing);
  const double sin_b = std::sin(bearing);

  // context = [goal distance, auxiliary]. The aux slot is 0 for the plain models and
  // the commanded min turn radius R for kinematics-conditioned ones (PathContext::context_aux).
  std::array<float, 2> input_values = {
    static_cast<float>(dist), static_cast<float>(context.context_aux)};
  const std::array<int64_t, 2> input_shape = {1, 2};
  const Ort::MemoryInfo memory =
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  std::vector<const char *> input_names = {input_name_.c_str()};
  std::vector<Ort::Value> inputs;
  inputs.push_back(
    Ort::Value::CreateTensor<float>(
      memory, input_values.data(), input_values.size(),
      input_shape.data(), input_shape.size()));

  // Feed the goal-aligned costmap patch when the model expects one.
  std::vector<float> patch_values;
  std::array<int64_t, 4> patch_shape{1, 1, 0, 0};
  if (has_costmap_input_ && costmap_dim_ > 0) {
    patch_values = alignedPatch(context, costmap_dim_, kPatchFwd, kPatchHalf, cos_b, sin_b);
    patch_shape[2] = costmap_dim_;
    patch_shape[3] = costmap_dim_;
    inputs.push_back(
      Ort::Value::CreateTensor<float>(
        memory, patch_values.data(), patch_values.size(),
        patch_shape.data(), patch_shape.size()));
    input_names.push_back(costmap_name_.c_str());
  }

  const char * output_names[] = {output_name_.c_str()};
  std::vector<Ort::Value> outputs = session_->Run(
    Ort::RunOptions{nullptr}, input_names.data(), inputs.data(), inputs.size(),
    output_names, 1);

  const Ort::Value & output = outputs.front();
  const std::vector<int64_t> shape = output.GetTensorTypeAndShapeInfo().GetShape();
  const float * data = output.GetTensorData<float>();
  if (shape.size() < 3) {
    return candidates;
  }
  const std::size_t num = static_cast<std::size_t>(shape[shape.size() - 3]);
  const std::size_t steps = static_cast<std::size_t>(shape[shape.size() - 2]);
  const std::size_t dim = static_cast<std::size_t>(shape[shape.size() - 1]);
  if (steps < 2 || dim < 2) {
    return candidates;
  }

  candidates.reserve(num);
  for (std::size_t k = 0; k < num; ++k) {
    nav2_diffusion_core::PathCandidate candidate;
    candidate.points.reserve(steps);
    for (std::size_t h = 0; h < steps; ++h) {
      const std::size_t base = ((k * steps) + h) * dim;
      const double ax = data[base + 0];
      const double ay = data[base + 1];
      // Aligned -> map frame: rotate by the goal bearing, translate by start.
      const double wx = context.start_x + ax * cos_b - ay * sin_b;
      const double wy = context.start_y + ax * sin_b + ay * cos_b;
      candidate.points.push_back({wx, wy});
    }
    // Snap endpoints exactly onto start/goal so the returned plan is anchored
    // regardless of small model regression error.
    candidate.points.front() = {context.start_x, context.start_y};
    candidate.points.back() = {context.goal_x, context.goal_y};
    candidates.push_back(candidate);
  }
  return candidates;
}

}  // namespace nav2_diffusion_onnx

PLUGINLIB_EXPORT_CLASS(
  nav2_diffusion_onnx::OnnxPathModel, nav2_diffusion_core::PathModel)
