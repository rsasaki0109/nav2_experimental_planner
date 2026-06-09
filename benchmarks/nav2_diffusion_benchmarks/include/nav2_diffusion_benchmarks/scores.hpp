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

#ifndef NAV2_DIFFUSION_BENCHMARKS__SCORES_HPP_
#define NAV2_DIFFUSION_BENCHMARKS__SCORES_HPP_

#include "nav2_diffusion_benchmarks/run_result.hpp"

namespace nav2_diffusion_benchmarks
{

/// Weights for the composite score (docs/benchmarking.md section 9.6). Defaults
/// are safety-first: a controller cannot rank well by being fast-but-unsafe.
struct ScoreWeights
{
  double safety{0.5};
  double progress{0.3};
  double comfort{0.2};
  double clearance_reference{0.5};  ///< clearance [m] that earns full safety credit
};

/// Component and overall scores, each in [0, 1] (higher is better).
struct Scores
{
  double safety{0.0};
  double progress{0.0};
  double comfort{0.0};
  double overall{0.0};
};

/// Compute the composite score for one run.
///
/// - safety:   0 if any collision, else min_clearance / clearance_reference (capped at 1)
/// - progress: 0 if the goal was not reached, else 1 / detour_ratio (capped at 1)
/// - comfort:  1 / (1 + total_turning)
/// - overall:  weight-normalized sum of the three components
Scores computeScores(const RunResult & result, const ScoreWeights & weights = ScoreWeights());

}  // namespace nav2_diffusion_benchmarks

#endif  // NAV2_DIFFUSION_BENCHMARKS__SCORES_HPP_
