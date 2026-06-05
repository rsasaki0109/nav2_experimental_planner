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

#include "nav2_diffusion_benchmarks/run_result.hpp"
#include "nav2_diffusion_benchmarks/scores.hpp"

using nav2_diffusion_benchmarks::computeScores;
using nav2_diffusion_benchmarks::RunResult;

namespace
{

RunResult idealRun()
{
  RunResult run;
  run.metrics.reached_goal = true;
  run.metrics.detour_ratio = 1.0;
  run.metrics.total_turning = 0.0;
  run.collision.collided = false;
  run.collision.collision_count = 0;
  run.collision.min_clearance = 1.0;  // >= clearance_reference
  return run;
}

}  // namespace

TEST(ScoresTest, IdealRunScoresPerfect)
{
  const auto s = computeScores(idealRun());
  EXPECT_DOUBLE_EQ(s.safety, 1.0);
  EXPECT_DOUBLE_EQ(s.progress, 1.0);
  EXPECT_DOUBLE_EQ(s.comfort, 1.0);
  EXPECT_DOUBLE_EQ(s.overall, 1.0);
}

TEST(ScoresTest, CollisionZeroesSafety)
{
  RunResult run = idealRun();
  run.collision.collided = true;
  run.collision.collision_count = 1;
  const auto s = computeScores(run);
  EXPECT_DOUBLE_EQ(s.safety, 0.0);
  EXPECT_LT(s.overall, 1.0);
}

TEST(ScoresTest, UnreachedGoalZeroesProgress)
{
  RunResult run = idealRun();
  run.metrics.reached_goal = false;
  const auto s = computeScores(run);
  EXPECT_DOUBLE_EQ(s.progress, 0.0);
}

TEST(ScoresTest, SafetyFirstRanksSafeMissAboveUnsafeReach)
{
  // A safe run that did not reach the goal...
  RunResult safe_miss = idealRun();
  safe_miss.metrics.reached_goal = false;  // progress 0, safety 1

  // ...should outrank a run that reached the goal but collided.
  RunResult unsafe_reach = idealRun();
  unsafe_reach.collision.collided = true;  // safety 0, progress 1
  unsafe_reach.collision.collision_count = 3;

  EXPECT_GT(
    computeScores(safe_miss).overall,
    computeScores(unsafe_reach).overall);
}

TEST(ScoresTest, MoreTurningLowersComfort)
{
  RunResult smooth = idealRun();
  RunResult jerky = idealRun();
  jerky.metrics.total_turning = 5.0;
  EXPECT_GT(computeScores(smooth).comfort, computeScores(jerky).comfort);
}
