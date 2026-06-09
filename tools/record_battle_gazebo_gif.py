#!/usr/bin/env python3
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
Record Planner Battle GIFs in real Gazebo Sim with TurtleBot3 waffle models.

Replays ``battle_data.json`` paths in a 3D gz-sim arena (same obstacle layout as
``battle_trace``). Multiple TB3 models race / draw paths simultaneously; frames
come from a perspective scene camera (not the flat browser canvas).

Usage::

    python3 tools/record_battle_gazebo_gif.py
    # writes docs/battle_gazebo_race.gif, battle_gazebo_maze.gif, battle_gazebo_duel.gif

Requires: ROS 2 Jazzy, gz-sim, turtlebot3_gazebo, ros_gz_image, built workspace optional.
"""

from __future__ import annotations

import argparse
import atexit
import json
import math
import os
import subprocess
import sys
import time

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, '..', 'docs')
BATTLE_JSON = os.path.join(HERE, 'nav2_planner_battle', 'battle_data.json')
TB3_SDF = '/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf'
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
CAM_W, CAM_H = 960, 540
SIM_WARMUP_SEC = 8.0
WALL_H = 0.45

COLORS = [
    (0.35, 0.82, 1.0), (1.0, 0.36, 0.42), (0.22, 0.88, 0.60), (1.0, 0.83, 0.30),
    (0.75, 0.55, 1.0), (1.0, 0.62, 0.26), (0.32, 0.88, 0.88), (1.0, 0.48, 0.84),
]

JOBS = [
    ('A', 1, 'battle_gazebo_race.gif', 6, 'Planner Battle · Gazebo · Mode A frontal race'),
    ('A', 4, 'battle_gazebo_maze.gif', 6, 'Planner Battle · Gazebo · micro-mouse easy'),
    ('B', 3, 'battle_gazebo_duel.gif', 6, 'Planner Battle · Gazebo · Mode B off-centre gap'),
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


def _ros_env():
    env = os.environ.copy()
    if os.path.isfile(ROS_SETUP):
        bash_env = subprocess.check_output(
            ['bash', '-c', 'source {} && env'.format(ROS_SETUP)], text=True)
        for line in bash_env.splitlines():
            if '=' in line:
                key, _, v = line.partition('=')
                env[key] = v
    base = '/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    if os.path.isdir(base):
        env['GZ_SIM_RESOURCE_PATH'] = base
    env.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')
    ros_py = '/opt/ros/jazzy/lib/python3.12/site-packages'
    if os.path.isdir(ros_py):
        env['PYTHONPATH'] = ros_py + os.pathsep + env.get('PYTHONPATH', '')
    return env


def _kill_stale_gz():
    subprocess.run(
        ['pkill', '-f', 'gz sim -r -s /tmp/nav2_battle'],
        capture_output=True)
    subprocess.run(
        ['pkill', '-f', 'gz sim -r -s /tmp/nav2_gazebo'],
        capture_output=True)
    time.sleep(1.5)


def _load_battle(path=BATTLE_JSON):
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


def _rgb01_to_ambient(rgb):
    r, g, b = rgb
    return '{} {} {} 1'.format(r, g, b)


def _rect_obstacle_sdf(idx, rect):
    cx = rect['x'] + rect['w'] / 2.0
    cy = rect['y'] + rect['h'] / 2.0
    return (
        '    <model name="obs_{idx}">\n'
        '      <static>true</static>\n'
        '      <pose>{cx} {cy} {hz} 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        '          <geometry><box><size>{w} {h} {wall}</size></box></geometry>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry><box><size>{w} {h} {wall}</size></box></geometry>\n'
        '          <material><ambient>0.65 0.18 0.18 1</ambient>'
        '<diffuse>0.75 0.2 0.2 1</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(
        idx=idx, cx=cx, cy=cy, hz=WALL_H / 2.0,
        w=rect['w'], h=rect['h'], wall=WALL_H)


def _arena_floor(arena):
    w, h = arena['w'], arena['h']
    return (
        '    <model name="arena_floor">\n'
        '      <static>true</static>\n'
        '      <pose>{cx} {cy} -0.01 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <visual name="visual">\n'
        '          <geometry><box><size>{w} {h} 0.02</size></box></geometry>\n'
        '          <material><ambient>0.55 0.58 0.62 1</ambient>'
        '<diffuse>0.62 0.65 0.7 1</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(cx=w / 2.0, cy=h / 2.0, w=w + 0.4, h=h + 0.4)


def _camera_block(arena):
    w, h = arena['w'], arena['h']
    span = max(w, h)
    cam_x = -span * 0.35
    cam_y = -span * 0.35
    cam_z = span * 1.15
    return (
        '    <model name="scene_camera">\n'
        '      <static>true</static>\n'
        '      <pose>{cx} {cy} {cz} 0 0.62 0.785398</pose>\n'
        '      <link name="link">\n'
        '        <sensor name="camera" type="camera">\n'
        '          <camera>\n'
        '            <horizontal_fov>1.05</horizontal_fov>\n'
        '            <image><width>{w}</width><height>{himg}</height>'
        '<format>R8G8B8</format></image>\n'
        '            <clip><near>0.1</near><far>120</far></clip>\n'
        '          </camera>\n'
        '          <always_on>1</always_on>\n'
        '          <update_rate>20</update_rate>\n'
        '          <visualize>false</visualize>\n'
        '          <topic>scene/image</topic>\n'
        '        </sensor>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(cx=cam_x, cy=cam_y, cz=cam_z, w=CAM_W, himg=CAM_H)


def _goal_marker(gx, gy):
    return (
        '    <model name="goal_marker">\n'
        '      <static>true</static>\n'
        '      <pose>{gx} {gy} 0.05 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <visual name="visual">\n'
        '          <geometry><cylinder><radius>0.18</radius><length>0.07</length>'
        '</cylinder></geometry>\n'
        '          <material><ambient>0.95 0.75 0.1 1</ambient>'
        '<diffuse>1.0 0.85 0.15 1</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(gx=gx, gy=gy)


def _marker_sdf(idx, rgb):
    amb = _rgb01_to_ambient(rgb)
    return (
        '    <model name="marker_{idx}">\n'
        '      <static>true</static>\n'
        '      <pose>0 0 0.28 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <visual name="visual">\n'
        '          <geometry><sphere><radius>0.11</radius></sphere></geometry>\n'
        '          <material><ambient>{amb}</ambient><diffuse>{amb}</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(idx=idx, amb=amb)


def battle_world_sdf(scenario, arena, n_markers=0):
    head = (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.6">\n'
        '  <world name="default">\n'
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>\n'
        '    <plugin filename="gz-sim-user-commands-system" '
        'name="gz::sim::systems::UserCommands"/>\n'
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n'
        '      <render_engine>ogre2</render_engine>\n'
        '    </plugin>\n'
        '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>\n'
        '    <light name="sun" type="directional">\n'
        '      <cast_shadows>1</cast_shadows>\n'
        '      <pose>0 0 10 0 0 0</pose>\n'
        '      <direction>-0.5 0.15 -0.92</direction>\n'
        '    </light>\n'
        '    <model name="ground_plane">\n'
        '      <static>1</static>\n'
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        '          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>\n'
        '        </collision>\n'
        '      </link>\n'
        '    </model>\n'
    )
    obs = ''.join(
        _rect_obstacle_sdf(i, r) for i, r in enumerate(scenario.get('obstacles') or []))
    gx, gy = scenario['goal']
    markers = ''.join(_marker_sdf(i, COLORS[i % len(COLORS)]) for i in range(n_markers))
    body = _arena_floor(arena) + obs + _camera_block(arena) + _goal_marker(gx, gy) + markers
    return head + body + '  </world>\n</sdf>\n'


def _overlay_title(frame, title, subtitle):
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    draw.rectangle((0, 0, img.width, 42), fill=(12, 20, 48))
    try:
        font = ImageFont.truetype('DejaVuSans-Bold.ttf', 17)
        small = ImageFont.truetype('DejaVuSans.ttf', 13)
    except OSError:
        font = small = ImageFont.load_default()
    draw.text((12, 8), title, fill=(232, 236, 255), font=font)
    draw.text((12, 28), subtitle, fill=(139, 151, 196), font=small)
    draw.text((12, img.height - 24), 'gz-sim · TB3 waffle × N · battle_trace replay',
              fill=(180, 190, 220), font=small)
    return np.asarray(img)


def _path_yaw(path, idx):
    if len(path[idx]) > 2 and abs(path[idx][2]) > 1e-6:
        return path[idx][2]
    if idx + 1 < len(path):
        dx = path[idx + 1][0] - path[idx][0]
        dy = path[idx + 1][1] - path[idx][1]
        return math.atan2(dy, dx)
    if idx > 0:
        dx = path[idx][0] - path[idx - 1][0]
        dy = path[idx][1] - path[idx - 1][1]
        return math.atan2(dy, dx)
    return 0.0


def _pick_fighters_mode_a(scenario, max_n):
    fs = scenario['fighters']
    ranked = sorted(
        fs,
        key=lambda f: (
            0 if f.get('outcome') == 'reached' else 1,
            f.get('steps', 9999),
            f.get('label', ''),
        ))
    return ranked[:max_n]


def _pick_fighters_mode_b(scenario, max_n):
    ok = [f for f in scenario['fighters'] if f.get('success') and f.get('path')]
    ok.sort(key=lambda f: (f.get('length', 1e9), f.get('label', '')))
    if len(ok) >= max_n:
        return ok[:max_n]
    rest = [f for f in scenario['fighters'] if f not in ok]
    return ok + rest[: max(0, max_n - len(ok))]


class BattleGazeboRecorder:
    def __init__(self):
        self._env = _ros_env()
        self._gz = None
        self._bridge = None
        self._node = None
        self._latest = None
        self._rclpy = None

    def _cleanup(self):
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
        tmp = '/tmp/nav2_battle_gazebo_world.sdf'
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
                super().__init__('battle_gazebo_capture')
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

    def reload(self, sdf_text):
        self._cleanup()
        atexit.unregister(self._cleanup)
        self.__init__()
        self.start(sdf_text)

    def _spawn_tb3(self, name, x, y, yaw):
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        req = (
            'sdf_filename: "{sdf}" name: "{name}" pose: {{ position: {{ x: {x} y: {y} '
            'z: 0.01 }} orientation: {{ z: {qz} w: {qw} }} }}'
        ).format(sdf=TB3_SDF, name=name, x=x, y=y, qz=qz, qw=qw)
        subprocess.run(
            ['gz', 'service', '-s', '/world/default/create',
             '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '8000', '--req', req],
            capture_output=True, env=self._env)

    def _set_pose(self, name, x, y, yaw, z=0.01):
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        req = (
            'name: "{name}" position: {{ x: {x} y: {y} z: {z} }} '
            'orientation: {{ z: {qz} w: {qw} }}'
        ).format(name=name, x=x, y=y, z=z, qz=qz, qw=qw)
        subprocess.run(
            ['gz', 'service', '-s', '/world/default/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', req],
            capture_output=True, env=self._env)

    def capture(self):
        self._latest = None
        for _ in range(60):
            self._rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._latest is not None:
                msg = self._latest
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3).copy()
        raise RuntimeError('timed out waiting for /scene/image')

    def record_scenario(self, mode, scenario, arena, title, max_fighters, stride):
        fighters = (_pick_fighters_mode_a if mode == 'A' else _pick_fighters_mode_b)(
            scenario, max_fighters)
        if not fighters:
            raise RuntimeError('no fighters in scenario')

        sdf = battle_world_sdf(scenario, arena, len(fighters))
        if self._gz is None:
            self.start(sdf)
        else:
            self.reload(sdf)

        sx, sy = scenario['start']
        for i, f in enumerate(fighters):
            path = f.get('path') or [[sx, sy, 0.0]]
            yaw0 = _path_yaw(path, 0)
            self._spawn_tb3('tb3_{}'.format(i), sx, sy, yaw0)
        time.sleep(1.5)

        max_len = min(max(len(f.get('path') or [1]) for f in fighters), 120)
        frames = []
        subtitle = '{} fighters · {}'.format(
            len(fighters), scenario.get('description') or scenario.get('name'))
        n_steps = max(1, len(range(0, max_len, stride)))
        for step_i, t in enumerate(range(0, max_len, stride)):
            print('  frame {}/{}'.format(step_i + 1, n_steps), file=sys.stderr, flush=True)
            for i, f in enumerate(fighters):
                path = f.get('path') or [[sx, sy, 0.0]]
                idx = min(t, len(path) - 1)
                p = path[idx]
                yaw = _path_yaw(path, idx)
                self._set_pose('tb3_{}'.format(i), p[0], p[1], yaw)
                self._set_pose('marker_{}'.format(i), p[0], p[1], 0.0, z=0.30)
            try:
                raw = self.capture()
                frames.append(_overlay_title(raw, title, subtitle))
            except RuntimeError:
                pass
            time.sleep(0.03)

        if not frames:
            frames.append(_overlay_title(self.capture(), title, subtitle))
        for _ in range(8):
            frames.append(frames[-1])
        return frames


def _write_gif(path, frames, duration=0.11):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    imageio.mimsave(path, frames, duration=duration, loop=0)
    print('wrote {} ({} frames)'.format(os.path.normpath(path), len(frames)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--battle-json', default=BATTLE_JSON)
    parser.add_argument('--max-fighters', type=int, default=6)
    parser.add_argument('--stride', type=int, default=2)
    parser.add_argument('--job', choices=['all', 'race', 'maze', 'duel'], default='all')
    args = parser.parse_args()

    if not os.path.isfile(TB3_SDF):
        raise SystemExit('TB3 model not found at {}'.format(TB3_SDF))

    _kill_stale_gz()
    data = _load_battle(args.battle_json)
    arena = data['arena']
    job_map = {'race': 0, 'maze': 1, 'duel': 2}
    jobs = JOBS
    if args.job != 'all':
        jobs = [JOBS[job_map[args.job]]]

    rec = BattleGazeboRecorder()
    try:
        for mode, sc_idx, out_name, _max_f, title in jobs:
            key = 'modeA' if mode == 'A' else 'modeB'
            scenario = data[key]['scenarios'][sc_idx]
            frames = rec.record_scenario(
                mode, scenario, arena, title,
                min(args.max_fighters, _max_f), args.stride)
            _write_gif(os.path.join(DOCS, out_name), frames)
    finally:
        rec._cleanup()


if __name__ == '__main__':
    main()
