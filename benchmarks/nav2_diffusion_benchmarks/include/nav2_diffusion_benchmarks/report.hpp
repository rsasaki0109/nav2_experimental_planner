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

#ifndef NAV2_DIFFUSION_BENCHMARKS__REPORT_HPP_
#define NAV2_DIFFUSION_BENCHMARKS__REPORT_HPP_

#include <string>
#include <vector>

#include "nav2_diffusion_benchmarks/run_result.hpp"
#include "nav2_diffusion_benchmarks/scores.hpp"

namespace nav2_diffusion_benchmarks
{

/// Render a Markdown comparison table over runs (docs/benchmarking.md section
/// 9.5). Each row is one scenario/controller pair; columns cover the task and
/// safety metrics so different controllers can be compared on the same scenario.
std::string toMarkdownTable(const std::vector<RunResult> & results);

/// Render a Markdown leaderboard sorted by the safety-first composite score
/// (docs/benchmarking.md section 9.6). Highest overall score ranks first.
std::string toMarkdownLeaderboard(
  const std::vector<RunResult> & results,
  const ScoreWeights & weights = ScoreWeights());

}  // namespace nav2_diffusion_benchmarks

#endif  // NAV2_DIFFUSION_BENCHMARKS__REPORT_HPP_
