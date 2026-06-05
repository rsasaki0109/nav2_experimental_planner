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

"""Tests for nav2_diffusion_training.dataset."""

import json

from nav2_diffusion_training import build_samples, save_jsonl, TrackState


def _straight_track(n):
    """Build a straight +x track at 1 m/s, 1 Hz."""
    return [TrackState(time=float(i), x=float(i), y=0.0, yaw=0.0) for i in range(n)]


def test_sample_count():
    """One sample per anchor that has both history and horizon available."""
    samples = build_samples(_straight_track(30), history=4, horizon=10)
    assert len(samples) == 16  # anchors 4..19 inclusive


def test_too_short_track_yields_nothing():
    """A track shorter than history+horizon produces no samples."""
    assert build_samples(_straight_track(5), history=4, horizon=10) == []


def test_straight_future_label_is_forward_in_base_frame():
    """A straight run yields a forward, lateral-free, rotation-free label."""
    samples = build_samples(_straight_track(30), history=2, horizon=5)
    label = samples[0]['action_label']
    assert abs(label[0]['x']) < 1e-9
    assert abs(label[0]['y']) < 1e-9
    assert label[-1]['x'] > 0.0
    assert abs(label[-1]['y']) < 1e-9
    assert abs(label[-1]['yaw']) < 1e-9


def test_turning_track_has_nonzero_relative_yaw():
    """An in-place rotation produces a nonzero relative heading in the label."""
    track = [TrackState(time=float(i), x=0.0, y=0.0, yaw=0.2 * i) for i in range(20)]
    samples = build_samples(track, history=1, horizon=5)
    assert abs(samples[0]['action_label'][-1]['yaw']) > 1e-6


def test_save_jsonl_roundtrip(tmp_path):
    """save_jsonl writes one JSON object per sample."""
    samples = build_samples(_straight_track(20), history=2, horizon=5)
    path = tmp_path / 'out.jsonl'
    save_jsonl(samples, str(path))
    lines = path.read_text().strip().split('\n')
    assert len(lines) == len(samples)
    assert 'action_label' in json.loads(lines[0])
