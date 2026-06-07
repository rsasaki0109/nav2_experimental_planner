#!/usr/bin/env python3
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
Reproduce the curated *obstacle-threading* Mode A controller model here.

This is the first learned Mode A controller in the zoo that **threads** obstacles in
closed loop instead of stalling in front of them. It is the costmap-token transformer
(`CostmapTransformerPlanner`) DAgger-trained on a **corrected reactive dodge oracle**
(`nav2_diffusion_training.dagger`), and is meant to run with the controller's
**windowed footprint gate** (`safety_check_points`, the C++ `DiffusionController`
parameter / `SAFETY_WINDOW` in the sim).

Why it works where the earlier Mode A models stalled (docs/generative_limits.md):
  1. the shipped DAgger oracle itself collided (a 0.20 m transient bow + an on-line
     carrot drives pure-pursuit straight into the block); the corrected oracle commits
     a *sustained* lateral offset to the free side and clears it (4/4 expert-only); and
  2. the deployed full-horizon footprint gate hard-rejects a tight reactive skirt whose
     1 m lookahead clips the block though step-wise execution skirts it safely — so the
     controller validates only the leading `safety_check_points` (receding-horizon:
     it replans every cycle against the live costmap and executes just the first segment).
With both, the DAgger-trained transformer reaches the goal **4/4 closed-loop** in the
costmap sim (`dagger.eval_closed_loop`), vs the 1/4 documented ceiling. The small
CNN-embedding flow model cannot fit the sharp dodge and stays at 1/4 — capacity matters.

Trains on the GPU when available; always exports on CPU for a portable artifact:

    PYTHONPATH=../../generative/nav2_diffusion_training CUDA_VISIBLE_DEVICES= python3 export.py

Mode A TrajectoryModel ONNX contract (consumed by nav2_diffusion_onnx::OnnxTrajectoryModel):

    context [1, 4] = [goal_x, goal_y, linear_speed, max_angular_speed]
    costmap [1, 1, 32, 32]            (egocentric patch; col 0 = +y / left)
    ->  trajectories [1, 3, 10, 3]    (x, y, yaw in the robot base frame)
"""

import os

from nav2_diffusion_training import dagger
from nav2_diffusion_training.dagger import dagger_train_costmap_transformer

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'costmap_threading.onnx')

ITERS = 8
BASE_SAMPLES = 320
EPOCHS = 900
LR = 0.003


def main():
    model = dagger_train_costmap_transformer(
        OUT, iters=ITERS, base_samples=BASE_SAMPLES, epochs=EPOCHS, lr=LR, verbose=True)
    sidecar = OUT + '.data'
    if os.path.exists(sidecar):
        os.remove(sidecar)
    reached, total = dagger.eval_closed_loop(model)
    print('exported %s (closed-loop %d/%d, window=%d, %d bytes)' % (
        OUT, reached, total, dagger.SAFETY_WINDOW, os.path.getsize(OUT)))


if __name__ == '__main__':
    main()
