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

#include <cmath>

#include "nav2_diffusion_core/path_model.hpp"
#include "nav2_diffusion_onnx/onnx_path_model.hpp"

#ifndef ONNX_PATH_MODEL
#define ONNX_PATH_MODEL ""
#endif

using nav2_diffusion_onnx::OnnxPathModel;

namespace
{
nav2_diffusion_core::PathContext context(double sx, double sy, double gx, double gy)
{
  nav2_diffusion_core::PathContext ctx;
  ctx.start_x = sx;
  ctx.start_y = sy;
  ctx.goal_x = gx;
  ctx.goal_y = gy;
  return ctx;
}
}  // namespace

TEST(OnnxPathModelTest, LoadsModelAndProducesAnchoredCandidates)
{
  OnnxPathModel model;
  model.configure(ONNX_PATH_MODEL);
  EXPECT_EQ(model.name(), "onnx_path");

  const auto candidates = model.generate(context(1.0, 1.0, 4.0, 1.0));

  // The fixture emits K=5 candidates of H=12 waypoints each.
  ASSERT_EQ(candidates.size(), 5u);
  for (const auto & c : candidates) {
    ASSERT_EQ(c.points.size(), 12u);
    // Endpoints are snapped exactly onto start and goal.
    EXPECT_DOUBLE_EQ(c.points.front().x, 1.0);
    EXPECT_DOUBLE_EQ(c.points.front().y, 1.0);
    EXPECT_DOUBLE_EQ(c.points.back().x, 4.0);
    EXPECT_DOUBLE_EQ(c.points.back().y, 1.0);
  }
}

TEST(OnnxPathModelTest, CandidatesFanLeftToRight)
{
  OnnxPathModel model(ONNX_PATH_MODEL);
  // Goal straight ahead along +x; midpoint lateral offset spans the fan.
  const auto candidates = model.generate(context(0.0, 0.0, 4.0, 0.0));
  const auto mid_y = [](const auto & c) {return c.points[c.points.size() / 2].y;};
  EXPECT_LT(mid_y(candidates.front()), mid_y(candidates.back()));
}

TEST(OnnxPathModelTest, RotatesIntoGoalBearing)
{
  OnnxPathModel model(ONNX_PATH_MODEL);
  // Goal straight up (+y): the aligned-frame x axis maps to +y, so the path
  // should advance mostly in y between start and goal.
  const auto candidates = model.generate(context(0.0, 0.0, 0.0, 4.0));
  ASSERT_FALSE(candidates.empty());
  const auto & mid = candidates[candidates.size() / 2].points[6];
  EXPECT_GT(mid.y, 1.0);                 // climbed toward the goal in +y
  EXPECT_LT(std::abs(mid.x), 1.0);       // little lateral drift in x
}
