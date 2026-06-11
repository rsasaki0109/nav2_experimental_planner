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
Record README battle GIFs from the real browser Nav2 Planner Battle UI.

Renders in GIF mode (action-cropped viewport, HUD strip above the arena,
agent label tags, finish banner) at native target resolution, then encodes
with ffmpeg two-pass palette optimization.

Usage::

    python3 tools/record_battle_gif.py            # all jobs
    python3 tools/record_battle_gif.py --job race # one job
"""

from __future__ import annotations

import argparse
import io
import os
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image
from playwright.sync_api import sync_playwright

from gif_writer import write_gif

HERE = os.path.dirname(os.path.abspath(__file__))
BATTLE = os.path.join(HERE, 'nav2_planner_battle')
DOCS = os.path.join(HERE, '..', 'docs')

_JOBS = {
    'race': dict(mode='A', sc=1, out='battle_race.gif', target_width=800,
                 stride=2, fps=14, first_hold=6, end_hold=22, budget_mb=4.0),
    'maze': dict(mode='A', sc=4, out='battle_maze.gif', target_width=640,
                 stride=3, fps=12, first_hold=5, end_hold=18, budget_mb=2.5),
    'duel': dict(mode='B', sc=3, out='battle_duel.gif', target_width=560,
                 stride=2, fps=11, first_hold=5, end_hold=18, budget_mb=2.5),
}
_CHAMP = dict(out='battle_championship.gif', target_width=640,
              stride=1, fps=8, end_hold=12, budget_mb=1.5)
# Canvas CSS background (#0a1330) — used to letterbox-pad championship frames.
_BG = (10, 19, 48)


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass


def _start_http(port):
    os.chdir(BATTLE)
    server = ThreadingHTTPServer(('127.0.0.1', port), _QuietHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.3)
    return server


def _capture_canvas(page):
    raw = page.locator('#cv').screenshot()
    img = Image.open(io.BytesIO(raw)).convert('RGB')
    return np.array(img)


def _capture_frames(page, stride):
    max_f = page.evaluate('() => window.__battleGif.maxFrames()')
    images = []
    last = -1
    for i in range(0, max_f + 1, stride):
        page.evaluate('(n) => window.__battleGif.setFrame(n)', i)
        images.append(_capture_canvas(page))
        last = i
    if last != max_f:  # always land on the final frame (finish banner)
        page.evaluate('(n) => window.__battleGif.setFrame(n)', max_f)
        images.append(_capture_canvas(page))
    return images


def record_gif(job, port, headless):
    out_path = os.path.join(DOCS, job['out'])
    url = 'http://127.0.0.1:{}/'.format(port)
    opts = {'crop': True, 'targetWidth': job['target_width'],
            'hudStrip': True, 'banner': True}
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=headless)
        page = browser.new_page(viewport={'width': 1280, 'height': 1024})
        page.goto(url, wait_until='networkidle', timeout=60000)
        page.wait_for_function('window.__battleGif')
        page.evaluate('([m,s,o]) => window.__battleGif.setup(m, s, o)',
                      [job['mode'], job['sc'], opts])
        time.sleep(0.3)
        size = page.evaluate('() => window.__battleGif.canvasSize()')
        print('{}: canvas {}x{}'.format(job['out'], size[0], size[1]))
        images = _capture_frames(page, job['stride'])
        browser.close()
    images = [images[0]] * job['first_hold'] + images
    images += [images[-1]] * job['end_hold']
    os.makedirs(DOCS, exist_ok=True)
    write_gif(out_path, images, fps=job['fps'], budget_mb=job['budget_mb'])


def _pad_to(img, w, h):
    if img.shape[1] == w and img.shape[0] == h:
        return img
    out = np.full((h, w, 3), _BG, dtype=np.uint8)
    out[:img.shape[0], :img.shape[1]] = img
    return out


def record_championship_gif(port, headless):
    job = _CHAMP
    out_path = os.path.join(DOCS, job['out'])
    url = 'http://127.0.0.1:{}/'.format(port)
    images = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=headless)
        page = browser.new_page(viewport={'width': 1280, 'height': 1024})
        page.goto(url, wait_until='networkidle', timeout=120000)
        page.wait_for_function('window.__battleGif', timeout=120000)
        for sub in ('A', 'B'):
            page.evaluate(
                '([s,o]) => window.__battleGif.setupChampionship(s, o)',
                [sub, {'targetWidth': job['target_width']}])
            time.sleep(0.3)
            images.extend(_capture_frames(page, job['stride']))
            images += [images[-1]] * job['end_hold']
        browser.close()
    # Mode A (7 rows) and Mode B (8 rows) canvases differ in height — pad.
    w = max(i.shape[1] for i in images)
    h = max(i.shape[0] for i in images)
    images = [_pad_to(i, w, h) for i in images]
    os.makedirs(DOCS, exist_ok=True)
    write_gif(out_path, images, fps=job['fps'], dither='none',
              budget_mb=job['budget_mb'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--headed', action='store_true')
    parser.add_argument('--job', default='all',
                        choices=['all', 'race', 'maze', 'duel', 'champ'])
    args = parser.parse_args()

    server = _start_http(args.port)
    try:
        for name, job in _JOBS.items():
            if args.job in ('all', name):
                record_gif(job, args.port, headless=not args.headed)
        if args.job in ('all', 'champ'):
            record_championship_gif(args.port, headless=not args.headed)
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
