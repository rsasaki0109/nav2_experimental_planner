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
Build training samples from a recorded SE(2) track.

This turns a time-ordered sequence of robot poses (e.g. extracted from /odom in a
rosbag) into supervised samples whose action label is the future trajectory
expressed in the robot base frame at the sample time, matching the dataset schema
in docs/architecture.md section 6.3 and the SE(2) representation in section 4.4.

Pure Python (stdlib only) so it is dependency-light and unit-testable; rosbag
ingestion is a thin adapter that produces the TrackState list consumed here.
"""

from dataclasses import dataclass
import json
import math
from typing import Dict, List


@dataclass
class TrackState:
    """One recorded robot pose at a time (map/odom frame)."""

    time: float
    x: float
    y: float
    yaw: float


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def _to_base_frame(origin: TrackState, state: TrackState) -> Dict[str, float]:
    """Express ``state`` relative to ``origin`` (origin at the SE(2) origin)."""
    dx = state.x - origin.x
    dy = state.y - origin.y
    cos_o = math.cos(-origin.yaw)
    sin_o = math.sin(-origin.yaw)
    return {
        'x': dx * cos_o - dy * sin_o,
        'y': dx * sin_o + dy * cos_o,
        'yaw': _normalize_angle(state.yaw - origin.yaw),
        'dt': state.time - origin.time,
    }


def build_samples(
    track: List[TrackState],
    history: int = 4,
    horizon: int = 20,
    stride: int = 1,
    source: str = 'rosbag',
) -> List[Dict]:
    """
    Slice a track into samples with a base-frame future-trajectory label.

    For each anchor index with ``history`` past frames and ``horizon`` future
    frames available, emit one sample. The action label is the future path in the
    base frame at the anchor; the observation window is the recent absolute poses.

    Returns an empty list when the track is too short.
    """
    if history < 0 or horizon < 1 or stride < 1:
        raise ValueError('history>=0, horizon>=1, stride>=1 required')

    samples: List[Dict] = []
    last_anchor = len(track) - horizon - 1
    for anchor in range(history, last_anchor + 1, stride):
        origin = track[anchor]
        observation_window = [
            {'x': s.x, 'y': s.y, 'yaw': s.yaw, 'time': s.time}
            for s in track[anchor - history:anchor + 1]
        ]
        action_label = [
            _to_base_frame(origin, track[j])
            for j in range(anchor, anchor + horizon + 1)
        ]
        samples.append({
            'observation_window': observation_window,
            'robot_state': {'x': origin.x, 'y': origin.y, 'yaw': origin.yaw,
                            'time': origin.time},
            'action_label': action_label,
            'timing_metadata': {'horizon_steps': horizon, 'history_steps': history},
            'source_metadata': {'source': source},
        })
    return samples


def save_jsonl(samples: List[Dict], path: str) -> None:
    """Write samples as JSON Lines (one sample per line)."""
    with open(path, 'w', encoding='utf-8') as handle:
        for sample in samples:
            handle.write(json.dumps(sample))
            handle.write('\n')
