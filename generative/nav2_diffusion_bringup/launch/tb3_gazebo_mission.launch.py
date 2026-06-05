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

r"""
Automated headless Gazebo closed-loop benchmark for the DiffusionController.

Brings up the TB3 Gazebo sim (reusing tb3_gazebo_diffusion.launch.py, headless, no
RViz) and an in-launch ``sim_mission`` node that drives a **course of NavigateToPose
legs**, records the executed odometry per leg, and writes a single Markdown
leaderboard. The mission node runs *inside* this launch so it shares the DDS graph
(external-process discovery is unreliable in restricted sandboxes —
docs/simulation.md section 10.5). When the mission node exits, the launch shuts down.

Single goal (backward compatible)::

    ros2 launch nav2_diffusion_bringup tb3_gazebo_mission.launch.py \\
        goal_x:=0.0 goal_y:=-0.5 results_file:=/tmp/sim_mission_result.md

Named course preset (default / there_and_back / patrol)::

    ros2 launch nav2_diffusion_bringup tb3_gazebo_mission.launch.py \\
        course:=there_and_back results_file:=/tmp/sim_course_result.md

Explicit multi-leg course (';'-separated 'label|x|y|yaw|timeout' legs; overrides
the preset)::

    ros2 launch nav2_diffusion_bringup tb3_gazebo_mission.launch.py \\
        missions:="out|0.0|-0.5|0.0|120;back|-2.0|-0.5|0.0|120" \\
        results_file:=/tmp/sim_course_result.md
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('nav2_diffusion_bringup')

    # Split the ';'-separated course string into a list parameter the node parses
    # per leg ('label|x|y|yaw|timeout'). Empty -> node falls back to the goal_* args.
    missions_str = LaunchConfiguration('missions').perform(context)
    missions = [leg.strip() for leg in missions_str.split(';') if leg.strip()]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'tb3_gazebo_diffusion.launch.py')
        ),
        launch_arguments={'use_rviz': 'False', 'headless': 'True'}.items(),
    )

    mission = Node(
        package='nav2_diffusion_bringup',
        executable='sim_mission.py',
        name='sim_mission',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
            'missions': missions if missions else [''],
            'course': LaunchConfiguration('course'),
            'timeout_sec': LaunchConfiguration('timeout_sec'),
            'stop_on_failure': LaunchConfiguration('stop_on_failure'),
            'label': LaunchConfiguration('label'),
            'results_file': LaunchConfiguration('results_file'),
        }],
    )

    # Tear the whole launch down once the mission node finishes.
    shutdown_on_mission_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=mission,
            on_exit=[EmitEvent(event=Shutdown(reason='mission complete'))],
        )
    )

    return [sim, mission, shutdown_on_mission_exit]


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('goal_x', default_value='0.0'),
        DeclareLaunchArgument('goal_y', default_value='-0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'missions', default_value='',
            description="';'-separated course legs: 'label|x|y|yaw|timeout' each; "
                        'overrides `course`; empty falls back to `course` or the '
                        'single goal_x/goal_y/goal_yaw goal'),
        DeclareLaunchArgument(
            'course', default_value='',
            description='Named course preset (default / there_and_back / patrol); '
                        'used when `missions` is empty'),
        DeclareLaunchArgument('timeout_sec', default_value='120.0'),
        DeclareLaunchArgument(
            'stop_on_failure', default_value='False',
            description='Abort the remaining legs after the first one that fails'),
        DeclareLaunchArgument(
            'label', default_value='Diffusion (Mode A) in Gazebo'),
        DeclareLaunchArgument(
            'results_file', default_value='/tmp/sim_mission_result.md'),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=_launch_setup)])
