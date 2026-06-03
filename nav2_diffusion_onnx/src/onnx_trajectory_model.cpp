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

#include "nav2_diffusion_onnx/onnx_trajectory_model.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_diffusion_onnx
{

OnnxTrajectoryModel::OnnxTrajectoryModel(const std::string & model_path)
{
  configure(model_path);
}

void OnnxTrajectoryModel::configure(const std::string & model_path)
{
  env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "nav2_diffusion_onnx");
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  session_ = std::make_shared<Ort::Session>(*env_, model_path.c_str(), options);

  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
}

std::string OnnxTrajectoryModel::name() const
{
  return "onnx";
}

std::vector<nav2_diffusion_core::Trajectory> OnnxTrajectoryModel::generate(
  const nav2_diffusion_core::ModelContext & context) const
{
  if (session_ == nullptr) {
    return {};
  }
  std::array<float, 4> input_values = {
    static_cast<float>(context.goal_x),
    static_cast<float>(context.goal_y),
    static_cast<float>(context.linear_speed),
    static_cast<float>(context.max_angular_speed),
  };
  const std::array<int64_t, 2> input_shape = {1, 4};

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

  // Expect a trailing [..., K, H, D] layout with D >= 2 (x, y[, yaw]).
  std::vector<nav2_diffusion_core::Trajectory> candidates;
  if (shape.size() < 3) {
    return candidates;
  }
  const double time_step = context.time_step > 1e-6 ? context.time_step : 0.1;
  const std::size_t num = static_cast<std::size_t>(shape[shape.size() - 3]);
  const std::size_t steps = static_cast<std::size_t>(shape[shape.size() - 2]);
  const std::size_t dim = static_cast<std::size_t>(shape[shape.size() - 1]);

  candidates.reserve(num);
  for (std::size_t k = 0; k < num; ++k) {
    nav2_diffusion_core::Trajectory trajectory;
    trajectory.points.reserve(steps);
    for (std::size_t h = 0; h < steps; ++h) {
      const std::size_t base = ((k * steps) + h) * dim;
      nav2_diffusion_core::TrajectoryPoint point;
      point.x = data[base + 0];
      point.y = data[base + 1];
      point.yaw = dim > 2 ? data[base + 2] : 0.0;
      point.time = static_cast<double>(h) * time_step;
      trajectory.points.push_back(point);
    }
    candidates.push_back(trajectory);
  }
  return candidates;
}

}  // namespace nav2_diffusion_onnx

PLUGINLIB_EXPORT_CLASS(
  nav2_diffusion_onnx::OnnxTrajectoryModel, nav2_diffusion_core::TrajectoryModel)
