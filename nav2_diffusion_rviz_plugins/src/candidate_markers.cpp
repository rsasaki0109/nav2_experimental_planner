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

#include "nav2_diffusion_rviz_plugins/candidate_markers.hpp"

#include <cstddef>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_diffusion_rviz_plugins
{

namespace
{
void setColor(visualization_msgs::msg::Marker & marker, float r, float g, float b)
{
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
}
}  // namespace

visualization_msgs::msg::MarkerArray toMarkerArray(
  const nav2_diffusion_msgs::msg::TrajectoryCandidates & candidates,
  double line_width,
  bool show_rejection_text)
{
  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker clear;
  clear.ns = "candidates";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  for (std::size_t i = 0; i < candidates.candidates.size(); ++i) {
    const auto & candidate = candidates.candidates[i];
    visualization_msgs::msg::Marker marker;
    marker.header = candidates.header;
    marker.ns = "candidates";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    const bool is_best = static_cast<int>(i) == candidates.best_index;
    const bool safe = i < candidates.safe_flags.size() && candidates.safe_flags[i];
    if (is_best) {
      setColor(marker, 0.0f, 1.0f, 0.0f);    // best: green
      marker.scale.x = line_width * 2.5;     // highlight the selected trajectory
    } else if (safe) {
      setColor(marker, 0.0f, 0.4f, 1.0f);    // safe: blue
      marker.scale.x = line_width;
    } else {
      setColor(marker, 1.0f, 0.0f, 0.0f);    // rejected: red
      marker.scale.x = line_width;
    }

    for (const auto & pose : candidate.poses) {
      geometry_msgs::msg::Point point;
      point.x = pose.position.x;
      point.y = pose.position.y;
      point.z = pose.position.z;
      marker.points.push_back(point);
    }
    array.markers.push_back(marker);

    const std::string reason =
      i < candidates.rejection_reasons.size() ? candidates.rejection_reasons[i] : std::string();
    if (show_rejection_text && !safe && !is_best && !reason.empty() && !candidate.poses.empty()) {
      visualization_msgs::msg::Marker text;
      text.header = candidates.header;
      text.ns = "rejection_reasons";
      text.id = static_cast<int>(i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.scale.z = line_width * 5.0;
      setColor(text, 1.0f, 0.0f, 0.0f);
      text.pose.orientation.w = 1.0;
      text.pose.position = candidate.poses.back().position;
      text.text = reason;
      array.markers.push_back(text);
    }
  }
  return array;
}

}  // namespace nav2_diffusion_rviz_plugins
