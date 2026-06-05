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

#include <cmath>

#include "nav2_diffusion_core/scoring.hpp"
#include "nav2_diffusion_core/trajectory.hpp"

using nav2_diffusion_core::ScoringWeights;
using nav2_diffusion_core::Trajectory;
using nav2_diffusion_core::TrajectoryPoint;

TEST(ScoringTest, EndpointDistanceMeasuresDistanceToGoal)
{
  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{3.0, 4.0, 0.0, 1.0},
  };
  EXPECT_DOUBLE_EQ(nav2_diffusion_core::endpointDistance(traj, 3.0, 0.0), 4.0);
}

TEST(ScoringTest, TotalTurningSumsAbsoluteHeadingChanges)
{
  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{0.0, 0.0, 0.5, 1.0},
    TrajectoryPoint{0.0, 0.0, 0.2, 2.0},
  };
  EXPECT_DOUBLE_EQ(nav2_diffusion_core::totalTurning(traj), 0.8);
}

TEST(ScoringTest, StraightToGoalScoresHigherThanCurvyAway)
{
  // Candidate A: ends right at the goal, no turning.
  Trajectory straight;
  straight.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{1.0, 0.0, 0.0, 1.0},
  };
  // Candidate B: turns and ends away from the goal.
  Trajectory curvy;
  curvy.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{0.5, 0.5, 0.8, 1.0},
  };

  const double goal_x = 1.0;
  const double goal_y = 0.0;
  EXPECT_GT(
    nav2_diffusion_core::scoreTrajectory(straight, goal_x, goal_y),
    nav2_diffusion_core::scoreTrajectory(curvy, goal_x, goal_y));
}
