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

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_diffusion_controller/egocentric_patch.hpp"

using nav2_diffusion_controller::cropEgocentricPatch;

namespace
{
constexpr int kSize = 32;

// Row of the maximum-valued cell in a row-major size x size patch.
int argmaxRow(const std::vector<float> & patch, int size)
{
  int best = 0;
  float best_v = -1.0f;
  for (int i = 0; i < size * size; ++i) {
    if (patch[i] > best_v) {
      best_v = patch[i];
      best = i / size;
    }
  }
  return best;
}

nav2_costmap_2d::Costmap2D sceneWithObstacleAhead()
{
  // 5x5 m, obstacle block at world (3.0, 2.5); robot will sit at (2.5, 2.5).
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  unsigned int mx = 0;
  unsigned int my = 0;
  costmap.worldToMap(3.0, 2.5, mx, my);
  for (int dy = -2; dy <= 2; ++dy) {
    for (int dx = -2; dx <= 2; ++dx) {
      costmap.setCost(mx + dx, my + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  return costmap;
}

}  // namespace

TEST(EgocentricPatchTest, ObstacleAheadAppearsInFrontWhenFacingIt)
{
  const auto costmap = sceneWithObstacleAhead();
  // Facing +x: the obstacle (at +x) is ahead -> upper rows (row < center).
  const auto patch = cropEgocentricPatch(costmap, 2.5, 2.5, 0.0, kSize, 0.05);
  EXPECT_LT(argmaxRow(patch, kSize), (kSize - 1) / 2);
}

TEST(EgocentricPatchTest, ObstacleMovesBehindWhenFacingAway)
{
  const auto costmap = sceneWithObstacleAhead();
  // Facing -x (yaw = pi): the same obstacle is now behind -> lower rows.
  const auto patch = cropEgocentricPatch(costmap, 2.5, 2.5, M_PI, kSize, 0.05);
  EXPECT_GT(argmaxRow(patch, kSize), (kSize - 1) / 2);
}

TEST(EgocentricPatchTest, ClearSceneIsAllZero)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  const auto patch = cropEgocentricPatch(costmap, 2.5, 2.5, 0.0, kSize, 0.05);
  float total = 0.0f;
  for (float v : patch) {
    total += v;
  }
  EXPECT_FLOAT_EQ(total, 0.0f);
}
