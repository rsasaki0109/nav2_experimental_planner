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

r"""
Closed-loop Gazebo benchmark on an obstacle course (centred / gap / slalom /
micro_mouse_easy / micro_mouse_hard).

Loads the course's generated gz-sim world and matching occupancy map (built from
one spec by nav2_diffusion_sim.gen_courses, so world+map+goals stay consistent),
spawns TB3 at the course start, brings up Nav2 with the DiffusionController, and
runs the in-launch ``sim_mission`` node through the course goals — writing a
Markdown leaderboard. The mission node shares the launch DDS graph; on its exit
the whole launch shuts down.

The course shapes mirror the off-line ``planner_benchmark`` obstacle scenarios so
the full closed-loop stack can be measured on the same geometry. The closed-loop
run needs a real ROS host (the sandbox blocks inter-process DDS — see
docs/simulation.md section 10.5); the course assets are generated and geometry-
checked in-tree.

Example::

    ros2 launch nav2_diffusion_sim tb3_gazebo_course.launch.py course:=gap \\
        results_file:=/tmp/course_gap_result.md
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

from nav2_diffusion_sim import gen_courses


def _setup(context, *args, **kwargs):
    course = LaunchConfiguration('course').perform(context)
    if course not in gen_courses.COURSE_SPECS:
        raise RuntimeError(
            "unknown course '{}'; valid: {}".format(
                course, sorted(gen_courses.COURSE_SPECS)))

    sim_share = get_package_share_directory('nav2_diffusion_sim')
    bringup_share = get_package_share_directory('nav2_diffusion_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    world = os.path.join(sim_share, 'worlds', course + '.sdf.xacro')
    map_yaml = os.path.join(sim_share, 'maps', course + '.yaml')
    params = os.path.join(bringup_share, 'params', 'nav2_diffusion_tb3.yaml')
    sx, sy, _ = gen_courses.COURSE_SPECS[course]['start']
    missions = gen_courses.course_missions(course)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'world': world,
            'map': map_yaml,
            'params_file': params,
            'x_pose': str(sx),
            'y_pose': str(sy),
            'use_rviz': 'False',
            'headless': 'True',
        }.items(),
    )

    mission = Node(
        package='nav2_diffusion_bringup',
        executable='sim_mission.py',
        name='sim_mission',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'missions': missions,
            'label': 'Diffusion on course "{}"'.format(course),
            'results_file': LaunchConfiguration('results_file'),
        }],
    )

    shutdown_on_mission_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=mission,
            on_exit=[EmitEvent(event=Shutdown(reason='mission complete'))],
        )
    )

    return [sim, mission, shutdown_on_mission_exit]


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument(
            'course', default_value='gap',
            description='Obstacle course: centred / gap / slalom / micro_mouse_easy / micro_mouse_hard'),
        DeclareLaunchArgument(
            'results_file', default_value='/tmp/course_result.md'),
    ]
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
