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

#include "nav2_diffusion_msgs/msg/safety_state.hpp"
#include "nav2_diffusion_rviz_plugins/safety_state_marker.hpp"
#include "visualization_msgs/msg/marker.hpp"

using nav2_diffusion_msgs::msg::SafetyState;
using nav2_diffusion_rviz_plugins::safetyStateLabel;
using nav2_diffusion_rviz_plugins::toSafetyMarker;

TEST(SafetyStateMarkerTest, LabelsKnownStates)
{
  EXPECT_EQ(safetyStateLabel(SafetyState::NOMINAL), "NOMINAL");
  EXPECT_EQ(safetyStateLabel(SafetyState::FALLBACK), "FALLBACK");
  EXPECT_EQ(safetyStateLabel(SafetyState::EMERGENCY_STOP), "EMERGENCY_STOP");
  EXPECT_EQ(safetyStateLabel(200), "UNKNOWN");
}

TEST(SafetyStateMarkerTest, NominalIsGreenTextMarker)
{
  SafetyState state;
  state.header.frame_id = "base_link";
  state.state = SafetyState::NOMINAL;
  const auto marker = toSafetyMarker(state);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(marker.header.frame_id, "base_link");
  EXPECT_EQ(marker.text, "NOMINAL");
  EXPECT_GT(marker.color.g, 0.5f);
  EXPECT_LT(marker.color.r, 0.5f);
}

TEST(SafetyStateMarkerTest, DetailIsAppendedAndBrakeIsRed)
{
  SafetyState state;
  state.state = SafetyState::BRAKE;
  state.detail = "footprint collision";
  const auto marker = toSafetyMarker(state);
  EXPECT_EQ(marker.text, "BRAKE: footprint collision");
  EXPECT_GT(marker.color.r, 0.5f);
  EXPECT_LT(marker.color.g, 0.5f);
}
