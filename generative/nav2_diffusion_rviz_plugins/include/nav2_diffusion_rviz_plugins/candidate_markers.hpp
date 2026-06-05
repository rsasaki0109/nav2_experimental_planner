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

#ifndef NAV2_DIFFUSION_RVIZ_PLUGINS__CANDIDATE_MARKERS_HPP_
#define NAV2_DIFFUSION_RVIZ_PLUGINS__CANDIDATE_MARKERS_HPP_

#include "nav2_diffusion_msgs/msg/trajectory_candidates.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_diffusion_rviz_plugins
{

/// Convert trajectory candidates into RViz markers so the behavior is
/// explainable (docs/architecture.md section 3.4: all candidates must be
/// visualizable). Each candidate becomes a LINE_STRIP, colored by verdict:
/// green = selected best (drawn thicker), blue = safe, red = rejected by the
/// safety gate. When show_rejection_text is set, each rejected candidate with a
/// reason gets a small red TEXT marker at its end. A leading DELETEALL marker
/// clears the previous cycle's markers.
visualization_msgs::msg::MarkerArray toMarkerArray(
  const nav2_diffusion_msgs::msg::TrajectoryCandidates & candidates,
  double line_width = 0.02,
  bool show_rejection_text = true);

}  // namespace nav2_diffusion_rviz_plugins

#endif  // NAV2_DIFFUSION_RVIZ_PLUGINS__CANDIDATE_MARKERS_HPP_
