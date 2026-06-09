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

#include "nav2_diffusion_core/path.hpp"

using nav2_diffusion_core::PathCandidate;
using nav2_diffusion_core::pathLength;

TEST(PathTest, EmptyAndSinglePointHaveZeroLength)
{
  PathCandidate empty;
  EXPECT_TRUE(empty.empty());
  EXPECT_EQ(pathLength(empty), 0.0);

  PathCandidate single;
  single.points.push_back({1.0, 2.0});
  EXPECT_EQ(pathLength(single), 0.0);
}

TEST(PathTest, LengthSumsSegmentDistances)
{
  PathCandidate path;
  path.points = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};  // 3 + 4
  EXPECT_DOUBLE_EQ(pathLength(path), 7.0);
  EXPECT_EQ(path.size(), 3u);
}
