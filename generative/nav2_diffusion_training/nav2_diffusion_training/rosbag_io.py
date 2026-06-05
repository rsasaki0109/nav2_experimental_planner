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

"""
Adapter that reads a recorded SE(2) track from a rosbag2 file.

This is the thin ingestion layer (docs/training.md section 6.2): it turns an
odometry topic in a bag into the TrackState list consumed by dataset.build_samples.
Imported separately from dataset so the pure dataset logic stays usable without
rosbag2_py installed.
"""

import math
from typing import List

from nav2_diffusion_training.dataset import TrackState
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def _state_from_odometry(msg, bag_stamp_ns: int) -> TrackState:
    """Extract a TrackState from an Odometry message."""
    q = msg.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    stamp = msg.header.stamp
    time = stamp.sec + stamp.nanosec * 1e-9
    if time == 0.0:
        time = bag_stamp_ns * 1e-9
    position = msg.pose.pose.position
    return TrackState(time=time, x=position.x, y=position.y, yaw=yaw)


def track_from_bag(
    bag_uri: str, topic: str = '/odom', storage_id: str = 'sqlite3',
) -> List[TrackState]:
    """
    Read an odometry topic from a rosbag2 file into a time-sorted track.

    Messages on other topics are ignored. Falls back to the bag timestamp when a
    message has no header stamp.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''))

    type_map = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    states: List[TrackState] = []
    while reader.has_next():
        topic_name, data, bag_stamp_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, get_message(type_map[topic_name]))
        states.append(_state_from_odometry(msg, bag_stamp_ns))

    states.sort(key=lambda state: state.time)
    return states
