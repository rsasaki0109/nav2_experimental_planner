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

Every fighter (all controllers / planners in the scenario) moves on the arena
canvas with coloured trails, heading arrows, LIVE HUD, collisions and goal flags.

Usage::

    python3 tools/record_battle_gif.py
    # writes docs/battle_race.gif, battle_maze.gif, battle_duel.gif,
    # battle_championship.gif
"""

from __future__ import annotations

import argparse
import io
import os
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import imageio.v2 as imageio
import numpy as np
from PIL import Image
from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
BATTLE = os.path.join(HERE, 'nav2_planner_battle')
DOCS = os.path.join(HERE, '..', 'docs')

_JOBS = [
    # mode, scenario_idx, outfile, stride, duration, upscale
    ('A', 1, 'battle_race.gif', 2, 0.07, 1.15),
    ('A', 4, 'battle_maze.gif', 2, 0.08, 1.0),
    ('B', 3, 'battle_duel.gif', 2, 0.09, 1.0),
]


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


def _capture_canvas(page, upscale):
    raw = page.locator('#cv').screenshot()
    img = Image.open(io.BytesIO(raw)).convert('RGB')
    if upscale != 1.0:
        w = int(img.width * upscale)
        h = int(img.height * upscale)
        img = img.resize((w, h), Image.Resampling.LANCZOS)
    return np.array(img)


def record_gif(mode, sc_idx, out_name, port, stride, duration, upscale, headless):
    out_path = os.path.join(DOCS, out_name)
    url = 'http://127.0.0.1:{}/'.format(port)
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=headless)
        page = browser.new_page(viewport={'width': 900, 'height': 900})
        page.goto(url, wait_until='networkidle', timeout=60000)
        page.wait_for_function('window.__battleGif')
        page.evaluate(
            '([m,s]) => window.__battleGif.setup(m, s)', [mode, sc_idx])
        time.sleep(0.3)
        max_f = page.evaluate('() => window.__battleGif.maxFrames()')
        images = []
        for i in range(0, max_f + 1, stride):
            page.evaluate('(n) => window.__battleGif.setFrame(n)', i)
            images.append(_capture_canvas(page, upscale))
        for _ in range(8):
            images.append(images[-1])
        browser.close()
    os.makedirs(DOCS, exist_ok=True)
    imageio.mimsave(out_path, images, duration=duration, loop=0)
    print('wrote {} ({} frames, {} fighters)'.format(
        out_path, len(images), 'all'))


def record_championship_gif(out_name, port, stride, duration, upscale, headless):
    out_path = os.path.join(DOCS, out_name)
    url = 'http://127.0.0.1:{}/'.format(port)
    images = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=headless)
        page = browser.new_page(viewport={'width': 900, 'height': 900})
        page.goto(url, wait_until='networkidle', timeout=120000)
        page.wait_for_function('window.__battleGif', timeout=120000)
        for sub in ('A', 'B'):
            page.evaluate(
                '(s) => window.__battleGif.setupChampionship(s)', sub)
            time.sleep(0.3)
            max_f = page.evaluate('() => window.__battleGif.maxFrames()')
            for i in range(0, max_f + 1, stride):
                page.evaluate('(n) => window.__battleGif.setFrame(n)', i)
                images.append(_capture_canvas(page, upscale))
            for _ in range(10):
                images.append(images[-1])
        browser.close()
    os.makedirs(DOCS, exist_ok=True)
    imageio.mimsave(out_path, images, duration=duration, loop=0)
    print('wrote {} ({} frames, race + duel championship)'.format(
        out_path, len(images)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--headed', action='store_true')
    args = parser.parse_args()

    server = _start_http(args.port)
    try:
        for mode, sc_idx, out_name, stride, duration, upscale in _JOBS:
            record_gif(
                mode, sc_idx, out_name, args.port, stride, duration, upscale,
                headless=not args.headed)
        record_championship_gif(
            'battle_championship.gif', args.port, 1, 0.12, 1.0,
            headless=not args.headed)
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
