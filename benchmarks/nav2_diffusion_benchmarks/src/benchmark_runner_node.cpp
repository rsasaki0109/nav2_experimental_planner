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

// Benchmark runner node: records the robot's executed path from an odometry
// topic and, on a Trigger service call, computes the run metrics and emits a
// Markdown report (docs/benchmarking.md section 9.5). The metric computation is
// the unit-tested RunRecorder; this node is the thin ROS glue around it.

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "nav2_diffusion_benchmarks/report.hpp"
#include "nav2_diffusion_benchmarks/run_recorder.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_diffusion_benchmarks
{

class BenchmarkRunnerNode : public rclcpp::Node
{
public:
  BenchmarkRunnerNode()
  : rclcpp::Node("benchmark_runner")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    scenario_ = declare_parameter<std::string>("scenario", "unnamed");
    controller_ = declare_parameter<std::string>("controller", "DiffusionController");
    goal_x_ = declare_parameter<double>("goal_x", 0.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.25);
    output_file_ = declare_parameter<std::string>("output_file", "");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(*msg);});

    finish_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/finish",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        onFinish(request, response);
      });

    RCLCPP_INFO(
      get_logger(), "benchmark_runner recording '%s' on topic '%s'",
      scenario_.c_str(), odom_topic_.c_str());
  }

private:
  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const double yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double time = rclcpp::Time(msg.header.stamp).seconds();
    recorder_.addSample(time, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);
  }

  void onFinish(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    if (recorder_.empty()) {
      response->success = false;
      response->message = "no odometry recorded";
      return;
    }

    const RunResult result = recorder_.finish(
      scenario_, controller_, goal_x_, goal_y_, goal_tolerance_);
    const std::vector<RunResult> results = {result};
    const std::string report = toMarkdownTable(results);

    RCLCPP_INFO(get_logger(), "Benchmark report:\n%s", report.c_str());
    if (!output_file_.empty()) {
      std::ofstream file(output_file_);
      if (file) {
        file << report;
        RCLCPP_INFO(get_logger(), "Wrote report to %s", output_file_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Could not write report to %s", output_file_.c_str());
      }
    }

    response->success = result.metrics.reached_goal;
    response->message = report;
    recorder_.reset();
  }

  std::string odom_topic_;
  std::string scenario_;
  std::string controller_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_tolerance_{0.25};
  std::string output_file_;

  RunRecorder recorder_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_srv_;
};

}  // namespace nav2_diffusion_benchmarks

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_diffusion_benchmarks::BenchmarkRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
