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

"""Round-trip test: write a tiny bag and read it back into a track."""

import os

from nav2_diffusion_training.rosbag_io import track_from_bag
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
import rosbag2_py


def _write_bag(bag_uri):
    """Write three Odometry messages advancing in +x to a sqlite3 bag."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0, name='/odom', type='nav_msgs/msg/Odometry',
            serialization_format='cdr'))
    for i in range(3):
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.header.stamp.sec = i
        msg.pose.pose.position.x = float(i)
        msg.pose.pose.orientation.w = 1.0
        writer.write('/odom', serialize_message(msg), i * 1_000_000_000)
    del writer  # flush/close


def test_track_from_bag_roundtrip(tmp_path):
    """The track recovers the recorded poses in time order."""
    bag_uri = os.path.join(str(tmp_path), 'odom_bag')
    _write_bag(bag_uri)

    track = track_from_bag(bag_uri, topic='/odom')

    assert len(track) == 3
    assert [round(s.x, 3) for s in track] == [0.0, 1.0, 2.0]
    assert track[0].time <= track[1].time <= track[2].time


def test_other_topics_ignored(tmp_path):
    """Requesting a missing topic yields an empty track."""
    bag_uri = os.path.join(str(tmp_path), 'odom_bag2')
    _write_bag(bag_uri)
    assert track_from_bag(bag_uri, topic='/not_there') == []
