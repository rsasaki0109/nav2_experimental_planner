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
#include <vector>

#include "nav2_diffusion_core/path_model.hpp"
#include "nav2_diffusion_onnx/onnx_path_model.hpp"

#ifndef ONNX_PATH_MODEL
#define ONNX_PATH_MODEL ""
#endif
#ifndef ONNX_COSTMAP_PATH_MODEL
#define ONNX_COSTMAP_PATH_MODEL ""
#endif
// The curated, committed Mode B artifact shipped in model_zoo/ (not a build-time
// fixture). Exercising it guards the shipped binary against corruption / drift.
#ifndef ONNX_ZOO_COSTMAP_PATH_MODEL
#define ONNX_ZOO_COSTMAP_PATH_MODEL ""
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

namespace
{
// A PathContext carrying a normalized global costmap with a rectangular obstacle
// over world x in [1.5, 4.0] on one lateral side (sign>0 => +y / left half).
nav2_diffusion_core::PathContext costmapContext(double obstacle_side)
{
  nav2_diffusion_core::PathContext ctx;
  ctx.start_x = 0.0;
  ctx.start_y = 0.0;
  ctx.goal_x = 4.0;   // bearing 0 -> aligned frame == world frame
  ctx.goal_y = 0.0;
  const unsigned int sx = 60;   // x: 0..6 m
  const unsigned int sy = 80;   // y: -4..4 m
  ctx.costmap_size_x = sx;
  ctx.costmap_size_y = sy;
  ctx.costmap_resolution = 0.1;
  ctx.costmap_origin_x = 0.0;
  ctx.costmap_origin_y = -4.0;
  ctx.costmap.assign(static_cast<std::size_t>(sx) * sy, 0.0f);
  for (unsigned int my = 0; my < sy; ++my) {
    const double wy = ctx.costmap_origin_y + (my + 0.5) * ctx.costmap_resolution;
    for (unsigned int mx = 0; mx < sx; ++mx) {
      const double wx = ctx.costmap_origin_x + (mx + 0.5) * ctx.costmap_resolution;
      const bool ahead = wx > 1.5 && wx < 4.0;
      // Signed lateral coordinate on the obstacle side (avoids "< -" literals).
      const double side_y = obstacle_side * wy;
      const bool on_side = side_y > 0.5 && side_y < 3.0;
      if (ahead && on_side) {
        ctx.costmap[static_cast<std::size_t>(my) * sx + mx] = 1.0f;
      }
    }
  }
  return ctx;
}
}  // namespace

TEST(OnnxPathModelTest, CostmapConditionedVeersAwayFromObstacle)
{
  OnnxPathModel model(ONNX_COSTMAP_PATH_MODEL);

  // Obstacle on the +y (left) side ahead -> candidates should bow toward -y.
  const auto left = model.generate(costmapContext(+1.0));
  ASSERT_FALSE(left.empty());
  EXPECT_LT(left[left.size() / 2].points[6].y, -0.1);

  // Obstacle on the -y (right) side ahead -> candidates should bow toward +y.
  const auto right = model.generate(costmapContext(-1.0));
  ASSERT_FALSE(right.empty());
  EXPECT_GT(right[right.size() / 2].points[6].y, 0.1);

  // Endpoints stay anchored regardless of the costmap.
  EXPECT_DOUBLE_EQ(left.front().points.front().x, 0.0);
  EXPECT_DOUBLE_EQ(left.front().points.back().x, 4.0);
}

// The shipped model_zoo artifact (diffusion_global_costmap_flow_v0): load the
// actual committed .onnx and assert its headline behaviour — every proposed
// candidate veers away from a one-sided obstacle. This is the repo's first
// learned model in the loop, so the curated binary itself is a regression guard.
TEST(OnnxPathModelTest, CuratedZooModelVeersAwayFromObstacle)
{
  const std::string zoo = ONNX_ZOO_COSTMAP_PATH_MODEL;
  if (zoo.empty()) {
    GTEST_SKIP() << "model_zoo costmap path model path not provided";
  }
  OnnxPathModel model(zoo);
  EXPECT_EQ(model.name(), "onnx_path");

  // Obstacle on the +y (left) side ahead -> ALL candidates should lean -y.
  const auto left = model.generate(costmapContext(+1.0));
  ASSERT_FALSE(left.empty());
  for (const auto & c : left) {
    EXPECT_LT(c.points[c.points.size() / 2].y, 0.0);
  }
  // Obstacle on the -y (right) side ahead -> ALL candidates should lean +y.
  const auto right = model.generate(costmapContext(-1.0));
  ASSERT_FALSE(right.empty());
  for (const auto & c : right) {
    EXPECT_GT(c.points[c.points.size() / 2].y, 0.0);
  }
}
