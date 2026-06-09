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

#ifndef NAV2_DIFFUSION_RVIZ_PLUGINS__SAFETY_STATE_MARKER_HPP_
#define NAV2_DIFFUSION_RVIZ_PLUGINS__SAFETY_STATE_MARKER_HPP_

#include <string>

#include "nav2_diffusion_msgs/msg/safety_state.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_diffusion_rviz_plugins
{

/// Human-readable label for a SafetyState value (docs/safety.md section 8.3).
std::string safetyStateLabel(uint8_t state);

/// Convert a SafetyState into a floating TEXT_VIEW_FACING marker, colored by
/// severity (nominal green -> emergency red), so the controller's runtime state
/// is visible in RViz. Placed z_offset above the marker frame origin.
visualization_msgs::msg::Marker toSafetyMarker(
  const nav2_diffusion_msgs::msg::SafetyState & state, double z_offset = 1.0);

}  // namespace nav2_diffusion_rviz_plugins

#endif  // NAV2_DIFFUSION_RVIZ_PLUGINS__SAFETY_STATE_MARKER_HPP_
