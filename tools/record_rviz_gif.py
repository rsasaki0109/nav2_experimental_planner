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
Record GIFs from real RViz2 playing battle MCAP data.

Replays ``docs/battle_*.mcap`` (from ``battle_mcap_demo.py``) into RViz2 and
captures the actual RViz window each frame — not a matplotlib stand-in.

Usage::

    source /opt/ros/jazzy/setup.bash
    python3 tools/battle_mcap_demo.py          # export MCAPs first
    python3 tools/record_rviz_gif.py           # -> docs/battle_*.gif
"""

from __future__ import annotations

import argparse
import atexit
import os
import subprocess
import sys
import time

import imageio.v2 as imageio
import mss
import numpy as np
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, '..', 'docs')
RVIZ_CFG = os.path.join(HERE, 'battle_view.rviz')
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
ROS_PY = '/opt/ros/jazzy/lib/python3.12/site-packages'

_JOBS = [
    ('battle_race.mcap', 'battle_race.gif'),
    ('battle_maze.mcap', 'battle_maze.gif'),
    ('battle_duel.mcap', 'battle_duel.gif'),
    ('sim_courses.mcap', 'sim_courses.gif'),
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
    if ROS_PY not in sys.path:
        sys.path.insert(0, ROS_PY)


def _topic_types():
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path

    types = {
        '/battle/costmap': OccupancyGrid,
        '/goal_pose': PoseStamped,
    }
    for i in range(8):
        types['/battle/path_{}'.format(i)] = Path
    return types


def _load_frames(mcap_path):
    """Group deserialized MCAP messages by log timestamp."""
    types = _topic_types()
    frames = {}
    with open(mcap_path, 'rb') as fh:
        for _schema, channel, message in make_reader(fh).iter_messages():
            if channel.topic not in types:
                continue
            msg = deserialize_message(message.data, types[channel.topic])
            frames.setdefault(message.log_time, {})[channel.topic] = msg
    return [frames[t] for t in sorted(frames)]


def _find_rviz_window():
    for pattern in ('battle_view.rviz', 'RViz'):
        out = subprocess.run(
            ['xdotool', 'search', '--name', pattern],
            capture_output=True, text=True)
        wids = [w for w in out.stdout.strip().split('\n') if w.strip()]
        if wids:
            break
    else:
        wids = []
    best = None
    best_area = 0
    for wid in wids:
        try:
            geom = subprocess.check_output(
                ['xdotool', 'getwindowgeometry', wid], text=True)
            for line in geom.splitlines():
                if 'Geometry:' in line:
                    wh = line.split('Geometry:')[1].strip().split('x')
                    area = int(wh[0]) * int(wh[1])
                    if area > best_area:
                        best_area = area
                        best = wid
        except (subprocess.CalledProcessError, ValueError, IndexError):
            continue
    return best


def _capture_window(wid):
    geom = subprocess.check_output(
        ['xdotool', 'getwindowgeometry', '--shell', wid], text=True)
    vals = {}
    for line in geom.splitlines():
        if '=' in line:
            k, v = line.split('=', 1)
            vals[k] = int(v)
    full = {
        'left': vals['X'], 'top': vals['Y'],
        'width': vals['WIDTH'], 'height': vals['HEIGHT'],
    }
    # Crop to the render pane when side panels are visible.
    pad_top, pad_left, pad_right, pad_bottom = 72, 318, 8, 24
    crop = {
        'left': vals['X'] + pad_left,
        'top': vals['Y'] + pad_top,
        'width': vals['WIDTH'] - pad_left - pad_right,
        'height': vals['HEIGHT'] - pad_top - pad_bottom,
    }
    with mss.mss() as sct:
        for mon in (crop, full):
            if mon['width'] < 120 or mon['height'] < 120:
                continue
            try:
                return np.array(sct.grab(mon))[:, :, :3]
            except Exception:
                continue
    raise RuntimeError('failed to capture RViz window {}'.format(wid))


class RvizRecorder:
    def __init__(self):
        self._rviz = None
        self._node = None
        self._rclpy = None
        self._wid = None
        self._env = os.environ.copy()
        if os.path.isfile(ROS_SETUP):
            bash_env = subprocess.check_output(
                ['bash', '-c', 'source {} && env'.format(ROS_SETUP)], text=True)
            for line in bash_env.splitlines():
                if '=' in line:
                    k, _, v = line.partition('=')
                    self._env[k] = v

    def _cleanup(self):
        if self._node is not None:
            self._node.destroy_node()
            self._rclpy.shutdown()
            self._node = None
        if self._rviz and self._rviz.poll() is None:
            self._rviz.terminate()
            try:
                self._rviz.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self._rviz.kill()

    def start(self):
        atexit.register(self._cleanup)
        self._rviz = subprocess.Popen(
            ['rviz2', '-d', RVIZ_CFG],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=self._env)
        time.sleep(7.0)
        for _ in range(30):
            wid = _find_rviz_window()
            if wid:
                subprocess.run(['xdotool', 'windowactivate', wid],
                               capture_output=True, env=self._env)
                self._wid = wid
                break
            time.sleep(0.5)
        if not self._wid:
            raise RuntimeError('RViz window not found (need DISPLAY + xdotool)')

        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import OccupancyGrid, Path
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile

        self._rclpy = rclpy
        rclpy.init()

        class Pub(Node):
            def __init__(self):
                super().__init__('battle_mcap_replay')
                latched = QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.pubs = {
                    '/battle/costmap': self.create_publisher(
                        OccupancyGrid, '/battle/costmap', latched),
                    '/goal_pose': self.create_publisher(
                        PoseStamped, '/goal_pose', 10),
                }
                for i in range(8):
                    self.pubs['/battle/path_{}'.format(i)] = self.create_publisher(
                        Path, '/battle/path_{}'.format(i), 10)

        self._node = Pub()
        time.sleep(1.0)

    def publish_frame(self, frame):
        for topic, msg in frame.items():
            if topic in self._node.pubs:
                self._node.pubs[topic].publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0.05)

    def capture(self):
        time.sleep(0.3)
        return _capture_window(self._wid)

    def record_mcap(self, mcap_path, gif_path, duration=0.09):
        subprocess.run(
            ['xdotool', 'windowactivate', self._wid],
            capture_output=True, env=self._env)
        time.sleep(0.5)
        frames_data = _load_frames(mcap_path)
        images = []
        for f in frames_data:
            self.publish_frame(f)
            images.append(self.capture())
        for _ in range(5):
            images.append(images[-1])
        os.makedirs(os.path.dirname(gif_path), exist_ok=True)
        imageio.mimsave(gif_path, images, duration=duration, loop=0)
        print('wrote {} ({} frames)'.format(gif_path, len(images)))


def main():
    _ensure_ros_env()
    if not os.environ.get('DISPLAY'):
        raise SystemExit('DISPLAY is not set — RViz recording needs a GUI display')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mcap', help='single MCAP input')
    parser.add_argument('--out', help='single GIF output')
    args = parser.parse_args()

    rec = RvizRecorder()
    try:
        rec.start()
        if args.mcap and args.out:
            rec.record_mcap(args.mcap, args.out)
        else:
            for mcap_name, gif_name in _JOBS:
                rec.record_mcap(
                    os.path.join(DOCS, mcap_name),
                    os.path.join(DOCS, gif_name))
    finally:
        rec._cleanup()


if __name__ == '__main__':
    main()
