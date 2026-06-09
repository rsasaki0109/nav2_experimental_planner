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

#ifndef NAV2_DIFFUSION_CORE__FAN_PATH_MODEL_HPP_
#define NAV2_DIFFUSION_CORE__FAN_PATH_MODEL_HPP_

#include <string>
#include <vector>

#include "nav2_diffusion_core/path_model.hpp"

namespace nav2_diffusion_core
{

/// Built-in analytic placeholder PathModel for Mode B.
///
/// Proposes a fan of start->goal candidates: a straight line plus laterally
/// bowed variants (a half-sine bump perpendicular to the straight line, swept
/// symmetrically left and right). It is deterministic and conditioning-free; it
/// stands in for a learned generative model behind the same PathModel seam so
/// the planner's propose -> validate -> select pipeline can be developed and
/// tested without a trained model. The bowed candidates give the validity layer
/// detours to choose from when the straight line is blocked.
class FanPathModel : public PathModel
{
public:
  /// @param max_bow_fraction peak lateral bow as a fraction of start-goal range
  explicit FanPathModel(double max_bow_fraction = 0.5)
  : max_bow_fraction_(max_bow_fraction) {}

  std::string name() const override;

  std::vector<PathCandidate> generate(const PathContext & context) const override;

private:
  double max_bow_fraction_;
};

}  // namespace nav2_diffusion_core

#endif  // NAV2_DIFFUSION_CORE__FAN_PATH_MODEL_HPP_
