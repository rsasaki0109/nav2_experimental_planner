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

TEST(MetricsTest, SmoothForwardRunHasNoOscillationOrStops)
{
  Trajectory executed;
  executed.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{1.0, 0.0, 0.0, 1.0},
    TrajectoryPoint{2.0, 0.0, 0.0, 2.0},
  };
  const auto m = evaluateRun(executed, 2.0, 0.0, 0.25);
  EXPECT_EQ(m.oscillation_count, 0);
  EXPECT_EQ(m.direction_changes, 0);
  EXPECT_DOUBLE_EQ(m.stop_duration, 0.0);
}

TEST(MetricsTest, StopDurationCountsStationarySegments)
{
  Trajectory executed;
  executed.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{1.0, 0.0, 0.0, 1.0},  // moving
    TrajectoryPoint{1.0, 0.0, 0.0, 2.0},  // stationary for 1 s
    TrajectoryPoint{2.0, 0.0, 0.0, 3.0},  // moving
  };
  const auto m = evaluateRun(executed, 2.0, 0.0, 0.25);
  EXPECT_DOUBLE_EQ(m.stop_duration, 1.0);
}

TEST(MetricsTest, DirectionChangeCountsForwardBackwardReversal)
{
  Trajectory executed;
  executed.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{1.0, 0.0, 0.0, 1.0},  // forward
    TrajectoryPoint{0.0, 0.0, 0.0, 2.0},  // backward (heading still 0)
  };
  const auto m = evaluateRun(executed, 0.0, 0.0, 0.25);
  EXPECT_EQ(m.direction_changes, 1);
}

TEST(MetricsTest, OscillationCountsTurnSignReversal)
{
  Trajectory executed;
  executed.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{1.0, 0.0, 0.3, 1.0},   // turn left
    TrajectoryPoint{2.0, 0.0, 0.0, 2.0},   // turn right
  };
  const auto m = evaluateRun(executed, 2.0, 0.0, 0.25);
  EXPECT_EQ(m.oscillation_count, 1);
}
