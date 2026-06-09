#!/usr/bin/env python3
# Copyright 2026 Nav2PlannerBattle contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Record README GIFs from real Gazebo Sim (3D perspective + diff-drive physics).

Each course is loaded in gz-sim with:
- TB3 waffle (meshes + LiDAR + diff-drive on ``/cmd_vel``)
- a corner **perspective** camera (not a flat top-down matplot look)
- shadows enabled

The robot **drives** along a footprint-valid A* route (``cmd_vel`` + ``/odom``),
not teleport ``set_pose``. Frames come from the simulated RGB camera via
``ros_gz_image``.

Usage::

    PYTHONPATH=generative/nav2_diffusion_sim python3 tools/gazebo_gif_demo.py
"""

from __future__ import annotations

import argparse
import atexit
import math
import os
import re
import subprocess
import sys
import time

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, '..', 'docs')
PKG = os.path.join(HERE, '..', 'generative', 'nav2_diffusion_sim')
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
sys.path.insert(0, PKG)

TB3_SDF = '/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'
CAM_W, CAM_H = 960, 540
MAX_DRIVE_SEC = 55.0
GOAL_TOL = 0.14
SIM_WARMUP_SEC = 10.0
CAPTURE_EVERY = 3  # control cycles between frames

README_JOBS = [
    ('battle_race.gif', 'gap', 'Gazebo Sim · gap course'),
    ('battle_maze.gif', 'micro_mouse_easy', 'Gazebo Sim · micro-mouse easy'),
    ('battle_duel.gif', 'slalom', 'Gazebo Sim · slalom course'),
]


def _ensure_ros_env():
    if not os.path.isfile(ROS_SETUP):
        return
    env = subprocess.check_output(
        ['bash', '-c', 'source {} && env'.format(ROS_SETUP)], text=True)
    for line in env.splitlines():
        if '=' not in line:
            continue
        key, _, val = line.partition('=')
        if key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'PATH', 'AMENT_PREFIX_PATH',
                   'ROS_DISTRO', 'ROS_VERSION'):
            os.environ[key] = val
    ros_py = '/opt/ros/jazzy/lib/python3.12/site-packages'
    if ros_py not in sys.path:
        sys.path.insert(0, ros_py)


_ensure_ros_env()
from nav2_diffusion_sim import gen_courses  # noqa: E402
import gazebo_courses_demo as grid_demo  # noqa: E402

SIM_COURSES_ORDER = list(grid_demo.ORDER)


def _tb3_resource_path():
    base = '/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    return base if os.path.isdir(base) else ''


def _ros_env():
    env = os.environ.copy()
    if os.path.isfile(ROS_SETUP):
        bash_env = subprocess.check_output(
            ['bash', '-c', 'source {} && env'.format(ROS_SETUP)], text=True)
        for line in bash_env.splitlines():
            if '=' in line:
                key, _, val = line.partition('=')
                env[key] = val
    res = _tb3_resource_path()
    if res:
        env['GZ_SIM_RESOURCE_PATH'] = res
    env.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    ros_py = '/opt/ros/jazzy/lib/python3.12/site-packages'
    if os.path.isdir(ros_py):
        env['PYTHONPATH'] = ros_py + os.pathsep + env.get('PYTHONPATH', '')
    return env


def _camera_block(xmin, ymin, span):
    """3/4 perspective camera from the SW corner — unmistakably 3D."""
    cam_x = xmin - span * 0.42
    cam_y = ymin - span * 0.42
    cam_z = span * 1.05
    return (
        '    <model name="scene_camera">\n'
        '      <static>true</static>\n'
        '      <pose>{cx} {cy} {cz} 0 0.68 0.785398</pose>\n'
        '      <link name="link">\n'
        '        <sensor name="camera" type="camera">\n'
        '          <camera>\n'
        '            <horizontal_fov>0.95</horizontal_fov>\n'
        '            <image><width>{w}</width><height>{h}</height>'
        '<format>R8G8B8</format></image>\n'
        '            <clip><near>0.1</near><far>100</far></clip>\n'
        '          </camera>\n'
        '          <always_on>1</always_on>\n'
        '          <update_rate>20</update_rate>\n'
        '          <visualize>false</visualize>\n'
        '          <topic>scene/image</topic>\n'
        '        </sensor>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(cx=cam_x, cy=cam_y, cz=cam_z, w=CAM_W, h=CAM_H)


def _goal_marker(gx, gy):
    return (
        '    <model name="goal_marker">\n'
        '      <static>true</static>\n'
        '      <pose>{gx} {gy} 0.05 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <visual name="visual">\n'
        '          <geometry><cylinder><radius>0.2</radius><length>0.08</length>'
        '</cylinder></geometry>\n'
        '          <material><ambient>0.95 0.75 0.1 1</ambient>'
        '<diffuse>1.0 0.85 0.15 1</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(gx=gx, gy=gy)


def _arena_floor(xmin, xmax, ymin, ymax):
    """Bounded floor slab so the course reads as an arena, not an infinite grey plane."""
    cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
    sx, sy = (xmax - xmin) + 0.6, (ymax - ymin) + 0.6
    return (
        '    <model name="arena_floor">\n'
        '      <static>true</static>\n'
        '      <pose>{cx} {cy} -0.01 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        '          <geometry><box><size>{sx} {sy} 0.02</size></box></geometry>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry><box><size>{sx} {sy} 0.02</size></box></geometry>\n'
        '          <material><ambient>0.55 0.58 0.62 1</ambient>'
        '<diffuse>0.62 0.65 0.7 1</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(cx=cx, cy=cy, sx=sx, sy=sy)


def _recording_sdf(course):
    sdf = gen_courses.world_sdf(course)
    sdf = sdf.replace('<cast_shadows>0</cast_shadows>', '<cast_shadows>1</cast_shadows>')
    spec = gen_courses.COURSE_SPECS[course]
    xmin, xmax, ymin, ymax = spec['extent']
    span = max(xmax - xmin, ymax - ymin)
    goal = spec['goals'][0]
    extras = _arena_floor(xmin, xmax, ymin, ymax)
    extras += _camera_block(xmin, ymin, span)
    extras += _goal_marker(goal[1], goal[2])
    return sdf.replace('  </world>', extras + '  </world>')


def _waypoints(course, n=18):
    route = grid_demo._route(course)
    if len(route) < 2:
        raise RuntimeError("no route for course '{}'".format(course))
    idx = np.linspace(0, len(route) - 1, n).astype(int)
    return route[idx]


def _overlay_title(frame, title):
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    draw.rectangle((0, 0, img.width, 38), fill=(12, 20, 48))
    try:
        font = ImageFont.truetype('DejaVuSans-Bold.ttf', 18)
    except OSError:
        font = ImageFont.load_default()
    draw.text((12, 9), title, fill=(232, 236, 255), font=font)
    draw.text((12, img.height - 26), 'gz-sim · TB3 waffle · diff-drive', fill=(180, 190, 220),
              font=font)
    return np.asarray(img)


def _gz_cmd_vel(env, linear, angular):
    subprocess.run(
        ['gz', 'topic', '-t', '/cmd_vel', '-m', 'gz.msgs.Twist',
         '-p', 'linear: {{x: {}}}, angular: {{z: {}}}'.format(linear, angular)],
        capture_output=True, env=env)


def _read_odom(env):
    proc = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/odom', '-n', '1'],
        capture_output=True, text=True, timeout=4, env=env)
    text = proc.stdout
    x = float(re.search(r'position \{\s*x: ([-\d.e]+)', text).group(1))
    y = float(re.search(
        r'position \{\s*x: [^\n]+\n\s*y: ([-\d.e]+)', text).group(1))
    qz = float(re.search(
        r'orientation \{[^}]*?z: ([-\d.e]+)', text, re.S).group(1))
    qw = float(re.search(
        r'orientation \{[^}]*?w: ([-\d.e]+)', text, re.S).group(1))
    yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    return x, y, yaw


class GazeboRecorder:
    def __init__(self):
        self._env = _ros_env()
        self._gz = None
        self._bridge = None
        self._node = None
        self._latest = None
        self._rclpy = None

    def _cleanup(self):
        _gz_cmd_vel(self._env, 0.0, 0.0)
        for proc in (self._bridge, self._gz):
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    proc.kill()
        if self._node is not None:
            self._node.destroy_node()
            self._rclpy.shutdown()
            self._node = None

    def start(self, sdf_text):
        atexit.register(self._cleanup)
        tmp = '/tmp/nav2_gazebo_gif_world.sdf'
        with open(tmp, 'w') as fh:
            fh.write(sdf_text)
        self._gz = subprocess.Popen(
            ['gz', 'sim', '-r', '-s', tmp],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=self._env)
        time.sleep(SIM_WARMUP_SEC)

        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image as RosImage

        self._rclpy = rclpy
        rclpy.init()
        recorder = self

        class Cap(Node):
            def __init__(self):
                super().__init__('gazebo_gif_capture')
                self.create_subscription(
                    RosImage, '/scene/image', recorder._on_image, 10)

        self._node = Cap()
        self._bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_image', 'image_bridge', '/scene/image'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=self._env)
        time.sleep(3.0)
        subprocess.run(
            ['gz', 'topic', '-t', '/scene/image/enable_streaming',
             '-m', 'gz.msgs.Boolean', '-p', 'data: 1'],
            capture_output=True, env=self._env)

    def _on_image(self, msg):
        self._latest = msg

    def spawn_tb3(self, x, y, yaw):
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        req = (
            'sdf_filename: "{}" name: "tb3" pose: {{ position: {{ x: {} y: {} z: 0.01 }} '
            'orientation: {{ z: {} w: {} }} }}'
        ).format(TB3_SDF, x, y, qz, qw)
        subprocess.run(
            ['gz', 'service', '-s', '/world/default/create',
             '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '8000', '--req', req],
            capture_output=True, text=True, env=self._env)

    def capture(self):
        self._latest = None
        for _ in range(50):
            self._rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._latest is not None:
                msg = self._latest
                return np.frombuffer(
                    msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 3).copy()
        raise RuntimeError('timed out waiting for /scene/image')

    def reload_world(self, sdf_text):
        self._cleanup()
        atexit.unregister(self._cleanup)
        self.__init__()
        self.start(sdf_text)

    def _drive_route(self, waypoints):
        """Follow waypoints with diff-drive physics; yield RGB frames."""
        idx = 0
        t0 = time.monotonic()
        step = 0
        while idx < len(waypoints) and time.monotonic() - t0 < MAX_DRIVE_SEC:
            try:
                x, y, yaw = _read_odom(self._env)
            except (subprocess.TimeoutExpired, AttributeError, ValueError):
                time.sleep(0.05)
                continue
            tx, ty = waypoints[idx]
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist < GOAL_TOL:
                idx += 1
                continue
            target_yaw = math.atan2(dy, dx)
            dyaw = math.atan2(
                math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
            lin = min(0.22, max(0.04, dist * 0.8))
            ang = max(-1.4, min(1.4, 2.8 * dyaw))
            if abs(dyaw) > 0.45:
                lin *= 0.35
            _gz_cmd_vel(self._env, lin, ang)
            self._rclpy.spin_once(self._node, timeout_sec=0.02)
            if step % CAPTURE_EVERY == 0:
                try:
                    yield self.capture()
                except RuntimeError:
                    pass
            step += 1
            time.sleep(0.07)
        _gz_cmd_vel(self._env, 0.0, 0.0)

    def record_course(self, course, title):
        sdf = _recording_sdf(course)
        spec = gen_courses.COURSE_SPECS[course]
        waypoints = _waypoints(course)
        sx, sy, _ = spec['start']
        # Face the first waypoint so diff-drive motion matches the route direction.
        if len(waypoints) > 1:
            syaw = math.atan2(waypoints[1][1] - sy, waypoints[1][0] - sx)
        else:
            syaw = spec['start'][2]
        if self._gz is None:
            self.start(sdf)
        else:
            self.reload_world(sdf)
        self.spawn_tb3(sx, sy, syaw)
        time.sleep(1.5)
        frames = []
        for raw in self._drive_route(waypoints):
            frames.append(_overlay_title(raw, title))
        if not frames:
            frames.append(_overlay_title(self.capture(), title))
        for _ in range(6):
            frames.append(frames[-1])
        return frames


def _write_gif(path, frames, duration=0.1):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    imageio.mimsave(path, frames, duration=duration, loop=0)
    print('wrote {} ({} frames)'.format(os.path.normpath(path), len(frames)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--only', choices=['all', 'battle', 'sim'], default='all')
    args = parser.parse_args()

    if not os.path.isfile(TB3_SDF):
        raise SystemExit('TB3 model not found at {}'.format(TB3_SDF))

    rec = GazeboRecorder()
    try:
        if args.only in ('all', 'battle'):
            for fname, course, title in README_JOBS:
                frames = rec.record_course(course, title)
                _write_gif(os.path.join(DOCS, fname), frames)

        if args.only in ('all', 'sim'):
            montage = []
            for course in SIM_COURSES_ORDER:
                title = 'Gazebo Sim · {}'.format(course)
                montage.extend(rec.record_course(course, title))
            _write_gif(os.path.join(DOCS, 'sim_courses.gif'), montage, duration=0.09)
    finally:
        rec._cleanup()


if __name__ == '__main__':
    main()
