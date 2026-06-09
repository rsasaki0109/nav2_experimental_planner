# Copyright 2026 Nav2PlannerBattle contributors
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

"""
Foxglove visualization bringup for Nav2PlannerBattle.

Starts foxglove_bridge (so Foxglove Studio can connect over ws://<host>:8765)
and the candidate_markers node that converts the controller's TrajectoryCandidates
/ SafetyState into visualization_msgs Markers. Import
``foxglove/nav2_diffusion_layout.json`` in Foxglove Studio for the matching
panel layout. See docs/visualization.md.

Requires the foxglove_bridge package (``ros-${ROS_DISTRO}-foxglove-bridge``); set
``use_bridge:=false`` to launch only the marker converter (e.g. when the bridge
runs elsewhere).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_bridge = LaunchConfiguration('use_bridge')
    port = LaunchConfiguration('port')
    candidates_topic = LaunchConfiguration('candidates_topic')
    safety_topic = LaunchConfiguration('safety_topic')

    declare_use_bridge = DeclareLaunchArgument(
        'use_bridge', default_value='true',
        description='Start foxglove_bridge (set false if it runs elsewhere)')
    declare_port = DeclareLaunchArgument(
        'port', default_value='8765',
        description='foxglove_bridge WebSocket port')
    declare_candidates_topic = DeclareLaunchArgument(
        'candidates_topic',
        default_value='/controller_server/FollowPath/trajectory_candidates',
        description='TrajectoryCandidates topic published by the controller')
    declare_safety_topic = DeclareLaunchArgument(
        'safety_topic',
        default_value='/controller_server/FollowPath/safety_state',
        description='SafetyState topic published by the controller')

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(use_bridge),
        parameters=[{
            'port': ParameterValue(port, value_type=int),
            'send_buffer_limit': 100000000,
        }],
        output='screen',
    )

    candidate_markers = Node(
        package='nav2_diffusion_rviz_plugins',
        executable='candidate_markers',
        name='candidate_markers',
        parameters=[{
            'input_topic': candidates_topic,
            'safety_topic': safety_topic,
            'output_topic': '/candidate_markers',
            'safety_marker_topic': '/safety_state_marker',
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_use_bridge,
        declare_port,
        declare_candidates_topic,
        declare_safety_topic,
        foxglove_bridge,
        candidate_markers,
    ])
