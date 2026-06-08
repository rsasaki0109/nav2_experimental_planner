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
Regression gate for battle_trace JSON.

Compares a fresh ``battle_trace`` export against the committed golden file.
Fails on outcome/success regressions or large metric drift (path length, etc.).

Usage::

    ros2 run nav2_planner_benchmarks battle_trace > /tmp/fresh.json
    python3 tools/check_battle_trace.py \\
        tools/nav2_planner_battle/battle_data.json /tmp/fresh.json
"""

from __future__ import annotations

import json
import math
import sys


def _load(path):
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


def _close(a, b, rel=0.03, abs_tol=0.08):
    if a is None or b is None:
        return a == b
    if abs(a - b) <= abs_tol:
        return True
    denom = max(abs(a), abs(b), 1e-9)
    return abs(a - b) / denom <= rel


def _fighter_map(scenario):
    return {f['label']: f for f in scenario['fighters']}


def _check_mode(golden_scenarios, fresh_scenarios, mode_key):
    errors = []
    g_by_name = {s['name']: s for s in golden_scenarios}
    f_by_name = {s['name']: s for s in fresh_scenarios}

    if set(g_by_name) != set(f_by_name):
        missing = set(g_by_name) - set(f_by_name)
        extra = set(f_by_name) - set(g_by_name)
        if missing:
            errors.append('{} missing scenarios: {}'.format(mode_key, sorted(missing)))
        if extra:
            errors.append('{} unexpected scenarios: {}'.format(mode_key, sorted(extra)))
        return errors

    for name, gsc in g_by_name.items():
        fsc = f_by_name[name]
        gf = _fighter_map(gsc)
        ff = _fighter_map(fsc)
        if set(gf) != set(ff):
            errors.append(
                '{} / {} fighter set changed: golden {} fresh {}'.format(
                    mode_key, name, sorted(gf), sorted(ff)))
            continue
        for label, g in gf.items():
            f = ff[label]
            prefix = '{} / {} / {}'.format(mode_key, name, label)
            if mode_key == 'modeA':
                if g.get('outcome') != f.get('outcome'):
                    errors.append('{} outcome {} -> {}'.format(
                        prefix, g.get('outcome'), f.get('outcome')))
                if not _close(g.get('length'), f.get('length')):
                    errors.append('{} length {} vs {}'.format(
                        prefix, g.get('length'), f.get('length')))
                if abs(g.get('steps', 0) - f.get('steps', 0)) > 3:
                    errors.append('{} steps {} vs {}'.format(
                        prefix, g.get('steps'), f.get('steps')))
            else:
                if g.get('success') != f.get('success'):
                    errors.append('{} success {} -> {}'.format(
                        prefix, g.get('success'), f.get('success')))
                if g.get('success') and f.get('success'):
                    if not _close(g.get('length'), f.get('length'), rel=0.02, abs_tol=0.15):
                        errors.append('{} length {} vs {}'.format(
                            prefix, g.get('length'), f.get('length')))
                    if not _close(g.get('time_ms'), f.get('time_ms'), rel=0.35, abs_tol=50):
                        errors.append('{} time_ms {} vs {}'.format(
                            prefix, g.get('time_ms'), f.get('time_ms')))
    return errors


def main():
    if len(sys.argv) != 3:
        print('usage: check_battle_trace.py GOLDEN.json FRESH.json', file=sys.stderr)
        sys.exit(2)
    golden = _load(sys.argv[1])
    fresh = _load(sys.argv[2])
    errors = []
    errors.extend(_check_mode(
        golden['modeA']['scenarios'], fresh['modeA']['scenarios'], 'modeA'))
    errors.extend(_check_mode(
        golden['modeB']['scenarios'], fresh['modeB']['scenarios'], 'modeB'))
    if errors:
        print('battle_trace regression FAILED ({} issue(s)):'.format(len(errors)))
        for e in errors[:40]:
            print('  -', e)
        if len(errors) > 40:
            print('  ... and {} more'.format(len(errors) - 40))
        sys.exit(1)
    print('battle_trace regression OK')


if __name__ == '__main__':
    main()
