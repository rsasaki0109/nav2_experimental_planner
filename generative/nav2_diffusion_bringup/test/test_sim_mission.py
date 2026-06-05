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
Unit tests for the pure (no-ROS) helpers of ``sim_mission.py``.

The driving loop needs a live Nav2 + Gazebo, but leg-spec parsing, metric
aggregation, and leaderboard formatting are pure functions — exercised here so
the harness logic is verified without joining a DDS graph (the closed-loop run
itself still needs a real ROS host; see docs/simulation.md section 10.5).
"""

import importlib.util
import math
import os

import pytest

# Load sim_mission.py by path: it installs as a program, not an importable module.
# Its ROS imports are lazy (inside _run_node), so importing the file is ROS-free.
_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.join(_HERE, '..', 'scripts', 'sim_mission.py')
_spec = importlib.util.spec_from_file_location('sim_mission', _SCRIPT)
sim_mission = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim_mission)


def test_parse_missions_full_and_optional_fields():
    missions = sim_mission.parse_missions(
        ['gap|2.0|-1.0|1.57|90', 'back|-2.0|-0.5'], default_timeout=120.0)
    assert len(missions) == 2
    assert missions[0] == sim_mission.Mission('gap', 2.0, -1.0, 1.57, 90.0)
    # Omitted yaw -> 0.0, omitted timeout -> the default.
    assert missions[1] == sim_mission.Mission('back', -2.0, -0.5, 0.0, 120.0)


def test_parse_missions_label_may_contain_spaces():
    missions = sim_mission.parse_missions(
        ['off-centre gap|1.0|2.0'], default_timeout=60.0)
    assert missions[0].label == 'off-centre gap'


def test_parse_missions_skips_blank_entries():
    # The launch declares a default [''] so blanks must be ignored, not errors.
    assert sim_mission.parse_missions(['', '  '], default_timeout=60.0) == []


def test_parse_missions_rejects_too_few_fields():
    with pytest.raises(ValueError):
        sim_mission.parse_missions(['only|1.0'], default_timeout=60.0)


def test_parse_missions_rejects_non_numeric():
    with pytest.raises(ValueError):
        sim_mission.parse_missions(['bad|x|2.0'], default_timeout=60.0)


def test_course_preset_expands_to_missions():
    missions = sim_mission.course_to_missions('there_and_back', default_timeout=60.0)
    assert [m.label for m in missions] == ['out', 'back']
    assert all(isinstance(m, sim_mission.Mission) for m in missions)


def test_course_empty_name_returns_empty():
    assert sim_mission.course_to_missions('', default_timeout=60.0) == []


def test_course_unknown_name_raises():
    with pytest.raises(ValueError):
        sim_mission.course_to_missions('no_such_course', default_timeout=60.0)


def test_course_presets_all_parse():
    # Every shipped preset must be well-formed (label|x|y|...).
    for name in sim_mission.COURSES:
        missions = sim_mission.course_to_missions(name, default_timeout=120.0)
        assert missions, "course '{}' expanded to nothing".format(name)


def test_yaw_to_quaternion_zero_and_quarter_turn():
    z, w = sim_mission.yaw_to_quaternion(0.0)
    assert z == pytest.approx(0.0)
    assert w == pytest.approx(1.0)
    z, w = sim_mission.yaw_to_quaternion(math.pi / 2)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


def test_format_leaderboard_rows_and_summary():
    outcomes = [
        sim_mission.MissionOutcome('gap', True, 12.3, 4.56, 120, 0.08, 'succeeded'),
        sim_mission.MissionOutcome('slalom', False, 30.0, 2.10, 90, 1.40, 'timeout'),
    ]
    md = sim_mission.format_leaderboard(outcomes, 'map', 'note')
    # One row per leg.
    assert '| gap | yes |' in md
    assert '| slalom | no |' in md
    # Summary aggregates reached count and totals.
    assert '1/2 legs reached' in md
    assert 'total path 6.66 m' in md
    # Markdown table header is present.
    assert '| Leg | Reached goal |' in md


def test_format_leaderboard_all_reached():
    outcomes = [
        sim_mission.MissionOutcome('a', True, 1.0, 1.0, 10, 0.0, 'succeeded'),
        sim_mission.MissionOutcome('b', True, 1.0, 1.0, 10, 0.0, 'succeeded'),
    ]
    md = sim_mission.format_leaderboard(outcomes, 'map', 'note')
    assert '2/2 legs reached' in md
