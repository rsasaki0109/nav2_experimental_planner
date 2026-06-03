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

#include "nav2_diffusion_msgs/msg/trajectory_candidates.hpp"
#include "nav2_diffusion_rviz_plugins/candidate_markers.hpp"
#include "visualization_msgs/msg/marker.hpp"

using nav2_diffusion_rviz_plugins::toMarkerArray;

namespace
{

nav2_diffusion_msgs::msg::TrajectoryCandidate makeCandidate(double end_x)
{
  nav2_diffusion_msgs::msg::TrajectoryCandidate candidate;
  geometry_msgs::msg::Pose a;
  a.orientation.w = 1.0;
  candidate.poses.push_back(a);
  geometry_msgs::msg::Pose b;
  b.position.x = end_x;
  b.orientation.w = 1.0;
  candidate.poses.push_back(b);
  return candidate;
}

}  // namespace

TEST(CandidateMarkersTest, EmitsDeleteAllThenOnePerCandidate)
{
  nav2_diffusion_msgs::msg::TrajectoryCandidates candidates;
  candidates.header.frame_id = "base_link";
  candidates.candidates.push_back(makeCandidate(1.0));
  candidates.candidates.push_back(makeCandidate(2.0));
  candidates.safe_flags = {true, false};
  candidates.best_index = 0;

  const auto markers = toMarkerArray(candidates);

  // DELETEALL + 2 candidate markers.
  ASSERT_EQ(markers.markers.size(), 3u);
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(markers.markers[1].header.frame_id, "base_link");
  EXPECT_EQ(markers.markers[1].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers[1].points.size(), 2u);
}

TEST(CandidateMarkersTest, ColorsBestSafeAndRejected)
{
  nav2_diffusion_msgs::msg::TrajectoryCandidates candidates;
  candidates.candidates.push_back(makeCandidate(1.0));  // best
  candidates.candidates.push_back(makeCandidate(1.0));  // safe
  candidates.candidates.push_back(makeCandidate(1.0));  // rejected
  candidates.safe_flags = {true, true, false};
  candidates.best_index = 0;

  const auto markers = toMarkerArray(candidates);
  ASSERT_EQ(markers.markers.size(), 4u);  // DELETEALL + 3

  const auto & best = markers.markers[1];
  const auto & safe = markers.markers[2];
  const auto & rejected = markers.markers[3];

  EXPECT_GT(best.color.g, 0.5f);     // green
  EXPECT_LT(best.color.r, 0.5f);
  EXPECT_GT(safe.color.b, 0.5f);     // blue
  EXPECT_GT(rejected.color.r, 0.5f);  // red
  EXPECT_LT(rejected.color.g, 0.5f);

  // The best trajectory is drawn thicker than the others.
  EXPECT_GT(best.scale.x, safe.scale.x);
}

TEST(CandidateMarkersTest, AddsRejectionReasonText)
{
  nav2_diffusion_msgs::msg::TrajectoryCandidates candidates;
  candidates.candidates.push_back(makeCandidate(1.0));  // best
  candidates.candidates.push_back(makeCandidate(1.0));  // rejected with reason
  candidates.safe_flags = {true, false};
  candidates.rejection_reasons = {"", "footprint collision"};
  candidates.best_index = 0;

  const auto markers = toMarkerArray(candidates);

  bool found_text = false;
  for (const auto & m : markers.markers) {
    if (m.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING) {
      found_text = true;
      EXPECT_EQ(m.text, "footprint collision");
      EXPECT_EQ(m.ns, "rejection_reasons");
    }
  }
  EXPECT_TRUE(found_text);
}
