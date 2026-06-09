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

#include <cmath>

#include "nav2_diffusion_core/fan_path_model.hpp"
#include "nav2_diffusion_core/path_model.hpp"

using nav2_diffusion_core::FanPathModel;
using nav2_diffusion_core::PathContext;

namespace
{

PathContext straightContext(int num_candidates, int num_points = 20)
{
  PathContext ctx;
  ctx.start_x = 0.0;
  ctx.start_y = 0.0;
  ctx.goal_x = 4.0;
  ctx.goal_y = 0.0;
  ctx.num_candidates = num_candidates;
  ctx.num_points = num_points;
  return ctx;
}

}  // namespace

TEST(FanPathModelTest, GeneratesRequestedCountWithEndpointsAnchored)
{
  FanPathModel model;
  const auto candidates = model.generate(straightContext(5));
  ASSERT_EQ(candidates.size(), 5u);
  for (const auto & c : candidates) {
    ASSERT_EQ(c.points.size(), 20u);
    // Every candidate starts at start and ends at goal (bow vanishes there).
    EXPECT_NEAR(c.points.front().x, 0.0, 1e-9);
    EXPECT_NEAR(c.points.front().y, 0.0, 1e-9);
    EXPECT_NEAR(c.points.back().x, 4.0, 1e-9);
    EXPECT_NEAR(c.points.back().y, 0.0, 1e-9);
  }
}

TEST(FanPathModelTest, FanBowsLeftToRight)
{
  FanPathModel model;
  const auto candidates = model.generate(straightContext(5));
  // Midpoint lateral offset increases monotonically across the fan.
  const auto mid = [](const auto & c) {return c.points[c.points.size() / 2].y;};
  EXPECT_LT(mid(candidates.front()), mid(candidates.back()));
}

TEST(FanPathModelTest, OddCountIncludesStraightLineInMiddle)
{
  FanPathModel model;
  const auto candidates = model.generate(straightContext(5));
  const auto & mid = candidates[2];
  for (const auto & p : mid.points) {
    EXPECT_NEAR(p.y, 0.0, 1e-9);  // straight line has no lateral offset
  }
}

TEST(FanPathModelTest, SingleCandidateIsStraightLine)
{
  FanPathModel model;
  const auto candidates = model.generate(straightContext(1));
  ASSERT_EQ(candidates.size(), 1u);
  for (const auto & p : candidates.front().points) {
    EXPECT_NEAR(p.y, 0.0, 1e-9);
  }
}

TEST(FanPathModelTest, BowAmplitudeScalesWithDistance)
{
  // A larger start-goal range produces a larger maximum bow.
  FanPathModel model(0.5);
  PathContext near_ctx = straightContext(3);
  near_ctx.goal_x = 2.0;
  PathContext far_ctx = straightContext(3);
  far_ctx.goal_x = 8.0;

  const auto near_c = model.generate(near_ctx);
  const auto far_c = model.generate(far_ctx);
  const auto peak = [](const auto & c) {
      double m = 0.0;
      for (const auto & p : c.points) {
        m = std::max(m, std::abs(p.y));
      }
      return m;
    };
  EXPECT_GT(peak(far_c.back()), peak(near_c.back()));
}
