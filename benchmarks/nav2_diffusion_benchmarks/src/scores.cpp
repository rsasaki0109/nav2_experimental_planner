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

#include "nav2_diffusion_benchmarks/scores.hpp"

#include <algorithm>

namespace nav2_diffusion_benchmarks
{

namespace
{
double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}
}  // namespace

Scores computeScores(const RunResult & result, const ScoreWeights & weights)
{
  Scores scores;

  const double clearance_ref = std::max(weights.clearance_reference, 1e-6);
  scores.safety = result.collision.collided ?
    0.0 :
    clamp01(result.collision.min_clearance / clearance_ref);

  if (result.metrics.reached_goal) {
    const double detour = std::max(result.metrics.detour_ratio, 1e-6);
    scores.progress = clamp01(1.0 / detour);
  }

  scores.comfort = 1.0 / (1.0 + std::max(0.0, result.metrics.total_turning));

  const double weight_sum = weights.safety + weights.progress + weights.comfort;
  const double norm = weight_sum > 1e-6 ? weight_sum : 1.0;
  scores.overall =
    (weights.safety * scores.safety +
    weights.progress * scores.progress +
    weights.comfort * scores.comfort) / norm;

  return scores;
}

}  // namespace nav2_diffusion_benchmarks
