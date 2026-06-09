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
Record GIFs from real Lichtblick (Foxglove-compatible) playing battle MCAPs.

Starts a local HTTP server for MCAP + layout JSON, opens Lichtblick in Chromium
via Playwright, imports the battle layout, scrubs the timeline, and captures
actual viewer screenshots.

Usage::

    docker run -d --rm --name lichtblick_rec -p 8080:8080 \\
      ghcr.io/lichtblick-suite/lichtblick:latest
    python3 tools/battle_mcap_demo.py
    python3 tools/record_lichtblick_gif.py
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import threading
import time
import urllib.parse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import imageio.v2 as imageio
import numpy as np
from mcap.reader import make_reader
from PIL import Image
from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, '..', 'docs')
LAYOUT = os.path.join(HERE, 'battle_layout.json')
VIEWER = os.environ.get('LICHTBLICK_URL', 'http://127.0.0.1:8080')

_JOBS = [
    ('battle_race.mcap', 'battle_race.gif'),
    ('battle_maze.mcap', 'battle_maze.gif'),
    ('battle_duel.mcap', 'battle_duel.gif'),
]


def _frame_times(mcap_path):
    times = set()
    with open(mcap_path, 'rb') as fh:
        for _s, _c, m in make_reader(fh).iter_messages():
            times.add(m.log_time)
    return sorted(times)


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass


def _start_http(port):
    os.chdir(DOCS)
    server = ThreadingHTTPServer(('127.0.0.1', port), _QuietHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def _import_layout(page, layout_path):
    """Import layout JSON through Lichtblick's Layout menu."""
    with page.expect_file_chooser(timeout=15000) as fc_info:
        page.get_by_role('button', name='Layout').click()
        page.get_by_text('Import from file').click()
    fc = fc_info.value
    fc.set_files(layout_path)
    time.sleep(2.0)


def _seek_timeline(page, frac):
    """Drag the timeline scrubber to a normalized position [0, 1]."""
    bar = page.locator('[data-testid="PlaybackBar"], .MuiSlider-root').first
    box = bar.bounding_box()
    if not box:
        return
    x = box['x'] + box['width'] * frac
    y = box['y'] + box['height'] / 2
    page.mouse.click(x, y)
    time.sleep(0.35)


def _capture_3d(page):
    """Screenshot the main 3D canvas region."""
    panel = page.locator('canvas').first
    panel.wait_for(state='visible', timeout=30000)
    box = panel.bounding_box()
    if not box:
        shot = page.screenshot()
    else:
        shot = page.screenshot(clip={
            'x': box['x'], 'y': box['y'],
            'width': box['width'], 'height': box['height'],
        })
    return np.array(Image.open(__import__('io').BytesIO(shot)).convert('RGB'))


def record_mcap_gif(mcap_name, gif_name, http_port, viewer_url, headless=True):
    mcap_url = 'http://127.0.0.1:{}/{}'.format(
        http_port, urllib.parse.quote(mcap_name))
    open_url = '{}?ds=remote-file&ds.url={}'.format(
        viewer_url, urllib.parse.quote(mcap_url, safe=''))
    times = _frame_times(os.path.join(DOCS, mcap_name))
    if not times:
        raise RuntimeError('no frames in {}'.format(mcap_name))

    images = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=headless)
        page = browser.new_page(viewport={'width': 1280, 'height': 800})
        page.goto(open_url, wait_until='networkidle', timeout=120000)
        time.sleep(4.0)
        try:
            _import_layout(page, LAYOUT)
        except Exception:
            # layout menu labels vary; continue with default if import fails
            pass
        time.sleep(3.0)
        n = len(times)
        for i in range(n):
            frac = i / max(1, n - 1)
            _seek_timeline(page, frac)
            images.append(_capture_3d(page))
        for _ in range(4):
            images.append(images[-1])
        browser.close()

    out = os.path.join(DOCS, gif_name)
    imageio.mimsave(out, images, duration=0.09, loop=0)
    print('wrote {} ({} frames)'.format(out, len(images)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=9877)
    parser.add_argument('--viewer', default=VIEWER)
    parser.add_argument('--headed', action='store_true')
    parser.add_argument('--mcap')
    parser.add_argument('--out')
    args = parser.parse_args()

    server = _start_http(args.port)
    try:
        if args.mcap and args.out:
            record_mcap_gif(
                args.mcap, args.out, args.port, args.viewer,
                headless=not args.headed)
        else:
            for mcap_name, gif_name in _JOBS:
                record_mcap_gif(
                    mcap_name, gif_name, args.port, args.viewer,
                    headless=not args.headed)
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
