// Copyright 2026 nav2_diffusion_planner contributors
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

#include "nav2_diffusion_benchmarks/metrics.hpp"
#include "nav2_diffusion_core/trajectory.hpp"

using nav2_diffusion_benchmarks::evaluateRun;
using nav2_diffusion_core::Trajectory;
using nav2_diffusion_core::TrajectoryPoint;

TEST(MetricsTest, EmptyRunIsNotReached)
{
  Trajectory executed;
  const auto m = evaluateRun(executed, 5.0, 0.0, 0.25);
  EXPECT_FALSE(m.reached_goal);
  EXPECT_DOUBLE_EQ(m.path_length, 0.0);
}

TEST(MetricsTest, StraightRunToGoalIsOptimal)
{
  Trajectory executed;
  executed.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{2.5, 0.0, 0.0, 5.0},
    TrajectoryPoint{5.0, 0.0, 0.0, 10.0},
  };
  const auto m = evaluateRun(executed, 5.0, 0.0, 0.25);
  EXPECT_TRUE(m.reached_goal);
  EXPECT_DOUBLE_EQ(m.goal_distance, 0.0);
  EXPECT_DOUBLE_EQ(m.path_length, 5.0);
  EXPECT_DOUBLE_EQ(m.time_to_goal, 10.0);
  EXPECT_DOUBLE_EQ(m.detour_ratio, 1.0);
  EXPECT_DOUBLE_EQ(m.total_turning, 0.0);
}

TEST(MetricsTest, DetourRatioExceedsOneForIndirectPath)
{
  // Goes up and over instead of straight to (2,0): length 1+1+1 = 3 vs 2.
  Trajectory executed;
  executed.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{0.0, 1.0, 0.0, 1.0},
    TrajectoryPoint{2.0, 1.0, 0.0, 2.0},
    TrajectoryPoint{2.0, 0.0, 0.0, 3.0},
  };
  const auto m = evaluateRun(executed, 2.0, 0.0, 0.25);
  EXPECT_TRUE(m.reached_goal);
  EXPECT_DOUBLE_EQ(m.path_length, 4.0);
  EXPECT_GT(m.detour_ratio, 1.0);
}

TEST(MetricsTest, RunStoppingShortDoesNotReachGoal)
{
  Trajectory executed;
  executed.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{4.0, 0.0, 0.0, 8.0},
  };
  const auto m = evaluateRun(executed, 5.0, 0.0, 0.25);
  EXPECT_FALSE(m.reached_goal);
  EXPECT_DOUBLE_EQ(m.goal_distance, 1.0);
}
