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

// Republishes nav2_diffusion_msgs/TrajectoryCandidates as a
// visualization_msgs/MarkerArray so the standard RViz MarkerArray display can
// show the candidates (best / safe / rejected). Thin ROS glue around the
// unit-tested toMarkerArray conversion.

#include <memory>
#include <string>

#include "nav2_diffusion_msgs/msg/safety_state.hpp"
#include "nav2_diffusion_msgs/msg/trajectory_candidates.hpp"
#include "nav2_diffusion_rviz_plugins/candidate_markers.hpp"
#include "nav2_diffusion_rviz_plugins/safety_state_marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_diffusion_rviz_plugins
{

class CandidateMarkersNode : public rclcpp::Node
{
public:
  CandidateMarkersNode()
  : rclcpp::Node("candidate_markers")
  {
    const std::string input_topic = declare_parameter<std::string>(
      "input_topic", "trajectory_candidates");
    const std::string output_topic = declare_parameter<std::string>(
      "output_topic", "candidate_markers");
    const std::string safety_topic = declare_parameter<std::string>(
      "safety_topic", "safety_state");
    const std::string safety_marker_topic = declare_parameter<std::string>(
      "safety_marker_topic", "safety_state_marker");
    line_width_ = declare_parameter<double>("line_width", 0.02);
    safety_text_height_ = declare_parameter<double>("safety_text_height", 1.0);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 1);
    candidates_sub_ = create_subscription<nav2_diffusion_msgs::msg::TrajectoryCandidates>(
      input_topic, rclcpp::SystemDefaultsQoS(),
      [this](const nav2_diffusion_msgs::msg::TrajectoryCandidates::SharedPtr msg) {
        marker_pub_->publish(toMarkerArray(*msg, line_width_));
      });

    safety_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(safety_marker_topic, 1);
    safety_sub_ = create_subscription<nav2_diffusion_msgs::msg::SafetyState>(
      safety_topic, rclcpp::SystemDefaultsQoS(),
      [this](const nav2_diffusion_msgs::msg::SafetyState::SharedPtr msg) {
        safety_marker_pub_->publish(toSafetyMarker(*msg, safety_text_height_));
      });

    RCLCPP_INFO(
      get_logger(), "candidate_markers: '%s' -> '%s', '%s' -> '%s'",
      input_topic.c_str(), output_topic.c_str(),
      safety_topic.c_str(), safety_marker_topic.c_str());
  }

private:
  double line_width_{0.02};
  double safety_text_height_{1.0};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr safety_marker_pub_;
  rclcpp::Subscription<nav2_diffusion_msgs::msg::TrajectoryCandidates>::SharedPtr candidates_sub_;
  rclcpp::Subscription<nav2_diffusion_msgs::msg::SafetyState>::SharedPtr safety_sub_;
};

}  // namespace nav2_diffusion_rviz_plugins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_diffusion_rviz_plugins::CandidateMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
