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

#ifndef NAV2_DIFFUSION_CORE__SCORING_HPP_
#define NAV2_DIFFUSION_CORE__SCORING_HPP_

#include "nav2_diffusion_core/trajectory.hpp"

namespace nav2_diffusion_core
{

/// Soft-scoring weights for selecting the best candidate among safe ones.
/// These are soft preferences only; hard safety is enforced separately by the
/// safety layer (see docs/architecture.md section 3.3, Trajectory Scorer).
struct ScoringWeights
{
  double progress{1.0};     ///< weight on getting the endpoint close to the goal
  double smoothness{0.1};   ///< weight penalizing total heading change (turning)
};

/// Euclidean distance from the trajectory endpoint to (goal_x, goal_y).
/// Returns 0.0 for an empty trajectory.
double endpointDistance(const Trajectory & trajectory, double goal_x, double goal_y);

/// Sum of absolute heading changes between consecutive samples, in radians.
double totalTurning(const Trajectory & trajectory);

/// Soft score for a candidate trajectory; higher is better.
///
/// score = -(w.progress * endpointDistance + w.smoothness * totalTurning),
/// so a trajectory that ends nearer the goal with less turning scores highest.
double scoreTrajectory(
  const Trajectory & trajectory, double goal_x, double goal_y,
  const ScoringWeights & weights = ScoringWeights());

}  // namespace nav2_diffusion_core

#endif  // NAV2_DIFFUSION_CORE__SCORING_HPP_
