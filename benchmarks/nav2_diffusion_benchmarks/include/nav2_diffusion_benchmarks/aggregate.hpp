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

#ifndef NAV2_DIFFUSION_BENCHMARKS__AGGREGATE_HPP_
#define NAV2_DIFFUSION_BENCHMARKS__AGGREGATE_HPP_

#include <string>
#include <vector>

#include "nav2_diffusion_benchmarks/run_result.hpp"
#include "nav2_diffusion_benchmarks/scores.hpp"

namespace nav2_diffusion_benchmarks
{

/// Per-controller aggregate over a set of runs (a scenario x controller matrix).
struct ControllerSummary
{
  std::string controller;
  int runs{0};
  double success_rate{0.0};   ///< fraction of runs that reached the goal
  double mean_overall{0.0};   ///< mean safety-first composite score
  double mean_safety{0.0};    ///< mean safety component
};

/// Aggregate runs by controller, sorted by mean overall score (descending).
std::vector<ControllerSummary> summarizeByController(
  const std::vector<RunResult> & results,
  const ScoreWeights & weights = ScoreWeights());

/// Render the per-controller aggregate as a Markdown ranking table
/// (docs/benchmarking.md sections 9.5/9.6).
std::string toMarkdownSummary(
  const std::vector<RunResult> & results,
  const ScoreWeights & weights = ScoreWeights());

}  // namespace nav2_diffusion_benchmarks

#endif  // NAV2_DIFFUSION_BENCHMARKS__AGGREGATE_HPP_
