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

#include "nav2_diffusion_rviz_plugins/safety_state_marker.hpp"

#include <string>

namespace nav2_diffusion_rviz_plugins
{

namespace
{
using SafetyState = nav2_diffusion_msgs::msg::SafetyState;

void setColor(visualization_msgs::msg::Marker & marker, float r, float g, float b)
{
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
}
}  // namespace

std::string safetyStateLabel(uint8_t state)
{
  switch (state) {
    case SafetyState::NOMINAL:
      return "NOMINAL";
    case SafetyState::CAUTIOUS:
      return "CAUTIOUS";
    case SafetyState::DEGRADED:
      return "DEGRADED";
    case SafetyState::FALLBACK:
      return "FALLBACK";
    case SafetyState::BRAKE:
      return "BRAKE";
    case SafetyState::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case SafetyState::RECOVERY:
      return "RECOVERY";
    default:
      return "UNKNOWN";
  }
}

visualization_msgs::msg::Marker toSafetyMarker(
  const nav2_diffusion_msgs::msg::SafetyState & state, double z_offset)
{
  visualization_msgs::msg::Marker marker;
  marker.header = state.header;
  marker.ns = "safety_state";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.z = 0.3;  // text height [m]
  marker.pose.position.z = z_offset;
  marker.pose.orientation.w = 1.0;

  std::string text = safetyStateLabel(state.state);
  if (!state.detail.empty()) {
    text += ": " + state.detail;
  }
  marker.text = text;

  switch (state.state) {
    case SafetyState::NOMINAL:
      setColor(marker, 0.0f, 1.0f, 0.0f);    // green
      break;
    case SafetyState::CAUTIOUS:
      setColor(marker, 1.0f, 1.0f, 0.0f);    // yellow
      break;
    case SafetyState::DEGRADED:
    case SafetyState::FALLBACK:
      setColor(marker, 1.0f, 0.5f, 0.0f);    // orange
      break;
    case SafetyState::BRAKE:
    case SafetyState::EMERGENCY_STOP:
      setColor(marker, 1.0f, 0.0f, 0.0f);    // red
      break;
    case SafetyState::RECOVERY:
      setColor(marker, 0.0f, 0.4f, 1.0f);    // blue
      break;
    default:
      setColor(marker, 0.5f, 0.5f, 0.5f);    // gray
      break;
  }
  return marker;
}

}  // namespace nav2_diffusion_rviz_plugins
