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

#include "nav2_diffusion_benchmarks/aggregate.hpp"

#include <algorithm>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace nav2_diffusion_benchmarks
{

namespace
{
struct Accumulator
{
  int runs = 0;
  int reached = 0;
  double overall = 0.0;
  double safety = 0.0;
};

std::string fixed(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}
}  // namespace

std::vector<ControllerSummary> summarizeByController(
  const std::vector<RunResult> & results, const ScoreWeights & weights)
{
  std::map<std::string, Accumulator> accumulators;
  for (const auto & result : results) {
    const Scores scores = computeScores(result, weights);
    Accumulator & acc = accumulators[result.controller];
    acc.runs += 1;
    acc.reached += result.metrics.reached_goal ? 1 : 0;
    acc.overall += scores.overall;
    acc.safety += scores.safety;
  }

  std::vector<ControllerSummary> summaries;
  for (const auto & entry : accumulators) {
    const Accumulator & acc = entry.second;
    ControllerSummary summary;
    summary.controller = entry.first;
    summary.runs = acc.runs;
    if (acc.runs > 0) {
      summary.success_rate = static_cast<double>(acc.reached) / acc.runs;
      summary.mean_overall = acc.overall / acc.runs;
      summary.mean_safety = acc.safety / acc.runs;
    }
    summaries.push_back(summary);
  }

  std::stable_sort(
    summaries.begin(), summaries.end(),
    [](const ControllerSummary & a, const ControllerSummary & b) {
      return a.mean_overall > b.mean_overall;
    });
  return summaries;
}

std::string toMarkdownSummary(
  const std::vector<RunResult> & results, const ScoreWeights & weights)
{
  const std::vector<ControllerSummary> summaries = summarizeByController(results, weights);

  std::ostringstream out;
  out << "| Rank | Controller | Runs | Success rate | Mean overall | Mean safety |\n";
  out << "|---|---|---|---|---|---|\n";
  int rank = 1;
  for (const auto & summary : summaries) {
    out << "| " << rank++
        << " | " << summary.controller
        << " | " << summary.runs
        << " | " << fixed(summary.success_rate, 3)
        << " | " << fixed(summary.mean_overall, 3)
        << " | " << fixed(summary.mean_safety, 3)
        << " |\n";
  }
  return out.str();
}

}  // namespace nav2_diffusion_benchmarks
