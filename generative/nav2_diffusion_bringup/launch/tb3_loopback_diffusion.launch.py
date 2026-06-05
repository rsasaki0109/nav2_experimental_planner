# Copyright 2026 nav2_experimental_planner contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Closed-loop demo: Nav2 TB3 loopback simulation driven by DiffusionController."""

# This reuses nav2_bringup's loopback simulation (a lightweight mock simulator
# that integrates cmd_vel into odom and synthesizes sensor data, see
# docs/simulation.md section 10.1) and only swaps in our controller via the
# params file. No Gazebo or GPU is required, so it runs headless.
# Requires the nav2_loopback_sim package to be installed.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('nav2_diffusion_bringup')

    default_params = os.path.join(pkg_dir, 'params', 'nav2_diffusion_tb3.yaml')

    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    candidates_topic = LaunchConfiguration('candidates_topic')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full Nav2 params file with the DiffusionController FollowPath plugin',
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RViz'
    )
    declare_candidates_topic_cmd = DeclareLaunchArgument(
        'candidates_topic',
        default_value='/FollowPath/trajectory_candidates',
        description='Topic the controller publishes trajectory candidates on',
    )

    loopback_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'tb3_loopback_simulation.launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_rviz': use_rviz,
        }.items(),
    )

    # Republish candidate trajectories as RViz markers (add a MarkerArray display
    # on /candidate_markers to see best=green / safe=blue / rejected=red).
    candidate_markers = Node(
        package='nav2_diffusion_rviz_plugins',
        executable='candidate_markers',
        name='candidate_markers',
        output='screen',
        parameters=[{
            'input_topic': candidates_topic,
            'output_topic': '/candidate_markers',
        }],
    )

    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_rviz_cmd,
        declare_candidates_topic_cmd,
        loopback_sim,
        candidate_markers,
    ])
