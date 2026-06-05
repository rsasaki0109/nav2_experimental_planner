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

"""Tests for the rule-based expert track generators."""

import math

from nav2_diffusion_training import build_samples, TrackState
from nav2_diffusion_training.experts import unicycle_to_goal


def test_reaches_goal_ahead():
    """The expert drives to a goal in front of it and ends within tolerance."""
    start = TrackState(time=0.0, x=0.0, y=0.0, yaw=0.0)
    track = unicycle_to_goal(start, 3.0, 0.0, tolerance=0.1)
    assert track[0].x == 0.0 and track[0].y == 0.0
    assert math.hypot(3.0 - track[-1].x, 0.0 - track[-1].y) <= 0.15


def test_turns_toward_offset_goal():
    """A goal to the side is still reached, requiring some turning."""
    start = TrackState(time=0.0, x=0.0, y=0.0, yaw=0.0)
    track = unicycle_to_goal(start, 2.0, 2.0, tolerance=0.1)
    assert math.hypot(2.0 - track[-1].x, 2.0 - track[-1].y) <= 0.15
    assert len(track) > 1


def test_already_at_goal_returns_start_only():
    """No motion is generated when already within tolerance of the goal."""
    start = TrackState(time=0.0, x=1.0, y=1.0, yaw=0.0)
    track = unicycle_to_goal(start, 1.0, 1.0, tolerance=0.5)
    assert len(track) == 1


def test_expert_track_feeds_build_samples():
    """An expert track can be turned straight into training samples."""
    start = TrackState(time=0.0, x=0.0, y=0.0, yaw=0.0)
    track = unicycle_to_goal(start, 3.0, 0.0)
    samples = build_samples(track, history=2, horizon=5)
    assert len(samples) > 0
    assert 'action_label' in samples[0]
