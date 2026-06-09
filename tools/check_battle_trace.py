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
Fails on outcome/success regressions or large metric drift (path length,
clearance, centering, planning time, …). On failure prints a grouped summary
so CI logs show *what* regressed before the full issue list.

Usage::

    ros2 run nav2_planner_benchmarks battle_trace > /tmp/fresh.json
    python3 tools/check_battle_trace.py \\
        tools/nav2_planner_battle/battle_data.json /tmp/fresh.json
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from dataclasses import dataclass


@dataclass(frozen=True)
class Issue:
    category: str
    mode: str
    scenario: str
    label: str
    message: str

    @property
    def key(self):
        return (self.mode, self.scenario, self.label)


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


def _clearance_ok(golden, fresh, abs_tol=0.03, rel=0.05):
    if golden is None or fresh is None:
        return True
    drop = golden - fresh
    if drop <= abs_tol:
        return True
    if golden <= 1e-9:
        return drop <= abs_tol
    return drop / golden <= rel


def _fighter_map(scenario):
    return {f['label']: f for f in scenario['fighters']}


def _outcome_rank(outcome):
    order = {'reached': 3, 'timeout': 2, 'collision': 1, 'stall': 1}
    return order.get(outcome or '', 0)


def _check_mode(golden_scenarios, fresh_scenarios, mode_key):
    issues = []
    g_by_name = {s['name']: s for s in golden_scenarios}
    f_by_name = {s['name']: s for s in fresh_scenarios}

    if set(g_by_name) != set(f_by_name):
        missing = set(g_by_name) - set(f_by_name)
        extra = set(f_by_name) - set(g_by_name)
        if missing:
            issues.append(Issue(
                'structure', mode_key, '*', '*',
                'missing scenarios: {}'.format(sorted(missing))))
        if extra:
            issues.append(Issue(
                'structure', mode_key, '*', '*',
                'unexpected scenarios: {}'.format(sorted(extra))))
        return issues

    for name, gsc in g_by_name.items():
        fsc = f_by_name[name]
        gf = _fighter_map(gsc)
        ff = _fighter_map(fsc)
        if set(gf) != set(ff):
            issues.append(Issue(
                'structure', mode_key, name, '*',
                'fighter set changed: golden {} fresh {}'.format(
                    sorted(gf), sorted(ff))))
            continue
        for label, g in gf.items():
            f = ff[label]
            if mode_key == 'modeA':
                g_out = g.get('outcome')
                f_out = f.get('outcome')
                if g_out != f_out:
                    issues.append(Issue(
                        'outcome', mode_key, name, label,
                        'outcome {} -> {}'.format(g_out, f_out)))
                elif _outcome_rank(f_out) < _outcome_rank(g_out):
                    issues.append(Issue(
                        'outcome', mode_key, name, label,
                        'outcome rank regressed {} -> {}'.format(g_out, f_out)))

                if not _close(g.get('length'), f.get('length')):
                    issues.append(Issue(
                        'reach', mode_key, name, label,
                        'length {:.3f} -> {:.3f}'.format(
                            g.get('length'), f.get('length'))))
                if abs(g.get('steps', 0) - f.get('steps', 0)) > 3:
                    issues.append(Issue(
                        'reach', mode_key, name, label,
                        'steps {} -> {}'.format(g.get('steps'), f.get('steps'))))
                if not _clearance_ok(g.get('clearance'), f.get('clearance')):
                    issues.append(Issue(
                        'clearance', mode_key, name, label,
                        'clearance {:.3f} -> {:.3f} (drop {:.3f})'.format(
                            g.get('clearance'), f.get('clearance'),
                            (g.get('clearance') or 0) - (f.get('clearance') or 0))))
                if 'mean_dw' in g and not _close(
                        g.get('mean_dw'), f.get('mean_dw'), rel=0.05, abs_tol=0.02):
                    issues.append(Issue(
                        'metrics', mode_key, name, label,
                        'mean_dw {:.3f} -> {:.3f}'.format(
                            g.get('mean_dw'), f.get('mean_dw'))))
                if 'centering' in g and not _close(
                        g.get('centering'), f.get('centering'), rel=0.05, abs_tol=0.03):
                    issues.append(Issue(
                        'metrics', mode_key, name, label,
                        'centering {:.3f} -> {:.3f}'.format(
                            g.get('centering'), f.get('centering'))))
            else:
                g_ok = g.get('success')
                f_ok = f.get('success')
                if g_ok != f_ok:
                    issues.append(Issue(
                        'outcome', mode_key, name, label,
                        'success {} -> {}'.format(g_ok, f_ok)))
                if g_ok and f_ok:
                    if not _close(g.get('length'), f.get('length'), rel=0.02, abs_tol=0.15):
                        issues.append(Issue(
                            'reach', mode_key, name, label,
                            'length {:.3f} -> {:.3f}'.format(
                                g.get('length'), f.get('length'))))
                    if not _close(g.get('time_ms'), f.get('time_ms'), rel=0.35, abs_tol=50):
                        issues.append(Issue(
                            'metrics', mode_key, name, label,
                            'time_ms {:.1f} -> {:.1f}'.format(
                                g.get('time_ms'), f.get('time_ms'))))
    return issues


_CATEGORY_LABELS = {
    'structure': 'Structure',
    'outcome': 'Outcome / success',
    'reach': 'Reach / length',
    'clearance': 'Clearance',
    'metrics': 'Metrics',
}


def _print_summary(issues):
    counts = Counter(i.category for i in issues)
    print('Summary by category:')
    for cat in ('structure', 'outcome', 'reach', 'clearance', 'metrics'):
        if counts.get(cat):
            print('  {:<18} {}'.format(_CATEGORY_LABELS[cat], counts[cat]))
    print()

    headline = []
    by_key = {}
    for issue in issues:
        if issue.category not in ('outcome', 'clearance', 'reach'):
            continue
        by_key.setdefault(issue.key, []).append(issue)
    for key, group in by_key.items():
        headline.append((group[0], group))
    if headline:
        print('Key regressions:')
        for lead, group in headline[:24]:
            msg = '; '.join(i.message for i in group)
            print('  {} / {} / {}  {}'.format(
                lead.mode, lead.scenario, lead.label, msg))
        if len(headline) > 24:
            print('  ... and {} more fighters with regressions'.format(len(headline) - 24))
        print()


def compare(golden, fresh):
    issues = []
    issues.extend(_check_mode(
        golden['modeA']['scenarios'], fresh['modeA']['scenarios'], 'modeA'))
    issues.extend(_check_mode(
        golden['modeB']['scenarios'], fresh['modeB']['scenarios'], 'modeB'))
    return issues


def main():
    parser = argparse.ArgumentParser(
        description='Compare battle_trace JSON against a golden file.')
    parser.add_argument('golden', help='committed battle_data.json')
    parser.add_argument('fresh', help='fresh battle_trace export')
    parser.add_argument(
        '--quiet', action='store_true',
        help='print only the final OK/FAILED line (no summary)')
    args = parser.parse_args()

    golden = _load(args.golden)
    fresh = _load(args.fresh)
    issues = compare(golden, fresh)

    if issues:
        print('battle_trace regression FAILED ({} issue(s)):'.format(len(issues)))
        if not args.quiet:
            print()
            _print_summary(issues)
            print('Details:')
        for issue in issues[:40]:
            prefix = '{} / {} / {}'.format(issue.mode, issue.scenario, issue.label)
            print('  - {}: {}'.format(prefix, issue.message))
        if len(issues) > 40:
            print('  ... and {} more'.format(len(issues) - 40))
        sys.exit(1)
    print('battle_trace regression OK')


if __name__ == '__main__':
    main()
