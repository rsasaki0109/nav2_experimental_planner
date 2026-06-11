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
Shared GIF writer for the battle recorders.

Uses ffmpeg two-pass palette encoding (palettegen/paletteuse) when ffmpeg is
on PATH — typically 3-5x smaller than unoptimized imageio output on the
mostly-static dark battle frames. Falls back to imageio otherwise.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile

import imageio.v2 as imageio


def write_gif(path, frames, fps, max_colors=128, dither='bayer', budget_mb=None):
    """Write frames (uniform-size RGB arrays) to path; return size in bytes."""
    if shutil.which('ffmpeg'):
        _write_ffmpeg(path, frames, fps, max_colors, dither)
        size = os.path.getsize(path)
        if budget_mb is not None and size > budget_mb * 1e6 and max_colors > 96:
            _write_ffmpeg(path, frames, fps, 96, dither)
            size = os.path.getsize(path)
    else:
        imageio.mimsave(path, frames, duration=1.0 / fps, loop=0)
        size = os.path.getsize(path)
    note = ''
    if budget_mb is not None and size > budget_mb * 1e6:
        note = ' — WARNING: over {:.1f} MB budget'.format(budget_mb)
    print('wrote {} ({} frames, {:.2f} MB{})'.format(
        path, len(frames), size / 1e6, note))
    return size


def _write_ffmpeg(path, frames, fps, max_colors, dither):
    if dither == 'bayer':
        use = 'dither=bayer:bayer_scale=5:diff_mode=rectangle'
    else:
        use = 'dither={}:diff_mode=rectangle'.format(dither)
    vf = ('split[a][b];[a]palettegen=max_colors={}:stats_mode=diff[p];'
          '[b][p]paletteuse={}'.format(max_colors, use))
    with tempfile.TemporaryDirectory() as td:
        for i, frame in enumerate(frames):
            imageio.imwrite(os.path.join(td, 'f_{:04d}.png'.format(i)), frame)
        subprocess.run(
            ['ffmpeg', '-y', '-loglevel', 'error',
             '-framerate', str(fps), '-i', os.path.join(td, 'f_%04d.png'),
             '-vf', vf, '-loop', '0', path],
            check=True)
