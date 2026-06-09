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

#ifndef NAV2_DIFFUSION_CORE__PATH_MODEL_HPP_
#define NAV2_DIFFUSION_CORE__PATH_MODEL_HPP_

#include <string>
#include <vector>

#include "nav2_diffusion_core/path.hpp"

namespace nav2_diffusion_core
{

/// Conditioning passed to a generative global-path model. Kept ROS-independent
/// so the same interface serves the built-in analytic model and learned
/// backends (PyTorch / ONNX) added later. Coordinates are in the planner's
/// global (map) frame (docs/architecture.md section 3.2, Mode B).
struct PathContext
{
  double start_x{0.0};   ///< start pose [m], global frame
  double start_y{0.0};
  double goal_x{0.0};    ///< goal pose [m], global frame
  double goal_y{0.0};

  int num_candidates{1};  ///< number of candidate paths to propose
  int num_points{20};    ///< waypoints per candidate (including endpoints)

  /// Auxiliary scalar conditioning fed to the model as the second context input
  /// (the first is the goal distance). 0 = unused (the default for models trained
  /// with a zero there). Kinematics-conditioned models read it as the vehicle's
  /// **min turn radius R**, so one model can serve several steering geometries.
  double context_aux{0.0};

  /// Optional global costmap for costmap-conditioned models, row-major
  /// size_x * size_y, normalized to [0, 1] (1 = lethal). Analytic models ignore
  /// it; the planner does the authoritative collision check itself regardless.
  unsigned int costmap_size_x{0};
  unsigned int costmap_size_y{0};
  double costmap_resolution{0.0};
  double costmap_origin_x{0.0};
  double costmap_origin_y{0.0};
  std::vector<float> costmap;
};

/// Abstract generative global-path model: the "propose" stage of Mode B.
/// Produces K candidate start->goal paths; the deterministic validity layer in
/// the planner decides which (if any) is returned to Nav2.
class PathModel
{
public:
  virtual ~PathModel() = default;

  /// One-time setup for pluginlib-loaded models (e.g. an ONNX file path).
  /// Built-in analytic models ignore it. Plugins are default-constructed, so
  /// heavy initialization needing the model path belongs here.
  virtual void configure(const std::string & model_path) {(void)model_path;}

  /// Short identifier for diagnostics / model registry.
  virtual std::string name() const = 0;

  /// Propose candidate global paths for the given context.
  virtual std::vector<PathCandidate> generate(const PathContext & context) const = 0;
};

}  // namespace nav2_diffusion_core

#endif  // NAV2_DIFFUSION_CORE__PATH_MODEL_HPP_
