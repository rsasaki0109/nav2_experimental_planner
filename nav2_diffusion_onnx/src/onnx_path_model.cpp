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
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
}

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

  std::array<float, 2> input_values = {static_cast<float>(dist), 0.0f};
  const std::array<int64_t, 2> input_shape = {1, 2};
  const Ort::MemoryInfo memory =
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input = Ort::Value::CreateTensor<float>(
    memory, input_values.data(), input_values.size(),
    input_shape.data(), input_shape.size());

  const char * input_names[] = {input_name_.c_str()};
  const char * output_names[] = {output_name_.c_str()};
  std::vector<Ort::Value> outputs = session_->Run(
    Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);

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
