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

#include "nav2_diffusion_benchmarks/report.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "nav2_diffusion_benchmarks/scores.hpp"

namespace nav2_diffusion_benchmarks
{

namespace
{

std::string fixed(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

std::string toMarkdownTable(const std::vector<RunResult> & results)
{
  std::ostringstream out;
  out << "| Scenario | Controller | Reached | Time [s] | Path [m] | Detour | "
    "Collisions | Min clear [m] | Turning [rad] | Osc | Dir chg | Stop [s] |\n";
  out << "|---|---|---|---|---|---|---|---|---|---|---|---|\n";
  for (const auto & result : results) {
    out << "| " << result.scenario
        << " | " << result.controller
        << " | " << (result.metrics.reached_goal ? "yes" : "no")
        << " | " << fixed(result.metrics.time_to_goal, 2)
        << " | " << fixed(result.metrics.path_length, 2)
        << " | " << fixed(result.metrics.detour_ratio, 2)
        << " | " << result.collision.collision_count
        << " | " << fixed(result.collision.min_clearance, 2)
        << " | " << fixed(result.metrics.total_turning, 2)
        << " | " << result.metrics.oscillation_count
        << " | " << result.metrics.direction_changes
        << " | " << fixed(result.metrics.stop_duration, 2)
        << " |\n";
  }
  return out.str();
}

std::string toMarkdownLeaderboard(
  const std::vector<RunResult> & results, const ScoreWeights & weights)
{
  struct Ranked
  {
    const RunResult * run;
    Scores scores;
  };

  std::vector<Ranked> ranked;
  ranked.reserve(results.size());
  for (const auto & result : results) {
    ranked.push_back({&result, computeScores(result, weights)});
  }
  std::stable_sort(
    ranked.begin(), ranked.end(),
    [](const Ranked & a, const Ranked & b) {return a.scores.overall > b.scores.overall;});

  std::ostringstream out;
  out << "| Rank | Controller | Scenario | Overall | Safety | Progress | Comfort |\n";
  out << "|---|---|---|---|---|---|---|\n";
  int rank = 1;
  for (const auto & entry : ranked) {
    out << "| " << rank++
        << " | " << entry.run->controller
        << " | " << entry.run->scenario
        << " | " << fixed(entry.scores.overall, 3)
        << " | " << fixed(entry.scores.safety, 3)
        << " | " << fixed(entry.scores.progress, 3)
        << " | " << fixed(entry.scores.comfort, 3)
        << " |\n";
  }
  return out.str();
}

}  // namespace nav2_diffusion_benchmarks
