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

#include <algorithm>
#include <string>
#include <vector>

#include "nav2_diffusion_benchmarks/report.hpp"

using nav2_diffusion_benchmarks::RunResult;
using nav2_diffusion_benchmarks::toMarkdownLeaderboard;
using nav2_diffusion_benchmarks::toMarkdownTable;

namespace
{

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

}  // namespace

TEST(ReportTest, EmptyResultsStillRenderHeader)
{
  const std::string table = toMarkdownTable({});
  EXPECT_TRUE(contains(table, "| Scenario | Controller |"));
  EXPECT_TRUE(contains(table, "|---|---|"));
}

TEST(ReportTest, RendersOneRowPerRun)
{
  RunResult diffusion;
  diffusion.scenario = "narrow_doorway";
  diffusion.controller = "DiffusionController";
  diffusion.metrics.reached_goal = true;
  diffusion.metrics.time_to_goal = 12.5;
  diffusion.metrics.path_length = 8.0;
  diffusion.metrics.detour_ratio = 1.1;
  diffusion.metrics.total_turning = 0.4;
  diffusion.collision.collision_count = 0;
  diffusion.collision.min_clearance = 0.35;

  RunResult mppi;
  mppi.scenario = "narrow_doorway";
  mppi.controller = "MPPI";
  mppi.metrics.reached_goal = false;
  mppi.collision.collision_count = 2;

  const std::string table = toMarkdownTable({diffusion, mppi});

  EXPECT_TRUE(contains(table, "DiffusionController"));
  EXPECT_TRUE(contains(table, "MPPI"));
  EXPECT_TRUE(contains(table, "narrow_doorway"));
  EXPECT_TRUE(contains(table, "yes"));
  EXPECT_TRUE(contains(table, "no"));
  EXPECT_TRUE(contains(table, "12.50"));
  EXPECT_TRUE(contains(table, "0.35"));

  // One header row, one separator row, and two data rows.
  const auto newlines = std::count(table.begin(), table.end(), '\n');
  EXPECT_EQ(newlines, 4);
}

TEST(ReportTest, LeaderboardRanksSaferRunFirst)
{
  // A reckless run that reached the goal but collided.
  RunResult reckless;
  reckless.scenario = "crowded_hallway";
  reckless.controller = "Reckless";
  reckless.metrics.reached_goal = true;
  reckless.metrics.detour_ratio = 1.0;
  reckless.collision.collided = true;
  reckless.collision.collision_count = 2;

  // A cautious run that also reached the goal without collision -> ranks first.
  RunResult cautious;
  cautious.scenario = "crowded_hallway";
  cautious.controller = "Cautious";
  cautious.metrics.reached_goal = true;
  cautious.metrics.detour_ratio = 1.0;
  cautious.collision.collided = false;
  cautious.collision.min_clearance = 1.0;

  const std::string board = toMarkdownLeaderboard({reckless, cautious});

  EXPECT_TRUE(contains(board, "| Rank | Controller |"));
  // The cautious controller should appear before the reckless one in the ranking.
  EXPECT_LT(board.find("Cautious"), board.find("Reckless"));
}
