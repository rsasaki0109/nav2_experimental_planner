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
Merge a partial ``battle_trace`` export (typically one custom fighter) into the
committed Nav2 Planner Battle JSON.

Usage::

    python3 tools/merge_battle_fighter.py \\
        tools/nav2_planner_battle/battle_data.json /tmp/custom.json \\
        tools/nav2_planner_battle/battle_data.json
"""

from __future__ import annotations

import json
import sys


def _load(path: str) -> dict:
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


def _write_js(json_path: str) -> None:
    js_path = json_path.rsplit('.', 1)[0] + '.js'
    with open(json_path, encoding='utf-8') as src, open(js_path, 'w', encoding='utf-8') as dst:
        dst.write('window.BATTLE_DATA = ')
        dst.write(src.read())
        dst.write(';\n')


def merge(base: dict, custom: dict) -> list[str]:
    notes: list[str] = []
    for mode in ('modeA', 'modeB'):
        if mode not in custom:
            continue
        if mode not in base:
            base[mode] = custom[mode]
            notes.append('added {}'.format(mode))
            continue
        by_name = {sc['name']: sc for sc in custom[mode]['scenarios']}
        for sc in base[mode]['scenarios']:
            if sc['name'] not in by_name:
                continue
            incoming = by_name[sc['name']].get('fighters') or []
            if not incoming:
                continue
            labels = {f['label'] for f in incoming}
            kept = [f for f in sc.get('fighters', []) if f['label'] not in labels]
            sc['fighters'] = kept + incoming
            notes.append(
                '{} / {}: merged {}'.format(mode, sc['name'], ', '.join(sorted(labels))))
    return notes


def main() -> int:
    if len(sys.argv) != 4:
        print(
            'usage: merge_battle_fighter.py BASE.json CUSTOM.json OUT.json',
            file=sys.stderr)
        return 2
    base_path, custom_path, out_path = sys.argv[1:4]
    base = _load(base_path)
    custom = _load(custom_path)
    notes = merge(base, custom)
    with open(out_path, 'w', encoding='utf-8') as fh:
        json.dump(base, fh, indent=2)
        fh.write('\n')
    _write_js(out_path)
    if notes:
        for line in notes:
            print(line)
    print('wrote {} and battle_data.js'.format(out_path))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
