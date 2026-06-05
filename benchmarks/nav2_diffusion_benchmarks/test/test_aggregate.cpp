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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "nav2_diffusion_benchmarks/aggregate.hpp"
#include "nav2_diffusion_benchmarks/run_result.hpp"

using nav2_diffusion_benchmarks::RunResult;
using nav2_diffusion_benchmarks::summarizeByController;
using nav2_diffusion_benchmarks::toMarkdownSummary;

namespace
{

RunResult makeRun(
  const std::string & scenario, const std::string & controller,
  bool reached, bool collided)
{
  RunResult run;
  run.scenario = scenario;
  run.controller = controller;
  run.metrics.reached_goal = reached;
  run.metrics.detour_ratio = 1.0;
  run.collision.collided = collided;
  run.collision.min_clearance = collided ? 0.0 : 1.0;
  return run;
}

std::vector<RunResult> matrix()
{
  // Cautious: reaches both scenarios safely. Reckless: collides on one.
  return {
    makeRun("corridor", "Cautious", true, false),
    makeRun("doorway", "Cautious", true, false),
    makeRun("corridor", "Reckless", true, true),
    makeRun("doorway", "Reckless", false, false),
  };
}

}  // namespace

TEST(AggregateTest, SummarizesPerControllerAndSortsByOverall)
{
  const auto summaries = summarizeByController(matrix());

  ASSERT_EQ(summaries.size(), 2u);
  // Cautious should rank first (safe + reached everywhere).
  EXPECT_EQ(summaries.front().controller, "Cautious");
  EXPECT_EQ(summaries.front().runs, 2);
  EXPECT_DOUBLE_EQ(summaries.front().success_rate, 1.0);
  EXPECT_GT(summaries.front().mean_overall, summaries.back().mean_overall);

  // Reckless reached 1 of 2 scenarios.
  EXPECT_EQ(summaries.back().controller, "Reckless");
  EXPECT_DOUBLE_EQ(summaries.back().success_rate, 0.5);
}

TEST(AggregateTest, MarkdownSummaryHasHeaderAndRows)
{
  const std::string table = toMarkdownSummary(matrix());
  EXPECT_NE(table.find("| Rank | Controller |"), std::string::npos);
  EXPECT_NE(table.find("Cautious"), std::string::npos);
  EXPECT_NE(table.find("Reckless"), std::string::npos);
}
