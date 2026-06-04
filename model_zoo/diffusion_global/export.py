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
Reproduce the curated costmap-conditioned Mode B path model in this directory.

This trains nav2_diffusion_training.path_planners.CostmapPathFlowPlanner on the
package's synthetic one-sided-obstacle dataset and exports the ONNX file
`costmap_flow.onnx` that the model card / manifest describe. Training is
deterministic (torch.manual_seed(0) inside the training function); run on CPU for
portability:

    PYTHONPATH=../../nav2_diffusion_training CUDA_VISIBLE_DEVICES= \\
        python3 export.py

The exported file matches the Mode B PathModel ONNX contract consumed by
nav2_diffusion_onnx::OnnxPathModel:

    context [1, 2] = [goal_distance, 0]
    costmap [1, 1, 24, 24]            (goal-aligned, row->fwd x, col->lateral y)
    ->  paths [1, 5, 12, 2]           (x, y in the goal-aligned frame)
"""

import os

from nav2_diffusion_training.path_planners import train_and_export_costmap_path

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'costmap_flow.onnx')

# Curated hyperparameters. Larger than the path_planners defaults: the extra
# samples/epochs give a symmetric, unbiased response (clear -> centred; a
# one-sided obstacle -> every candidate veers to the open side). Documented in
# model_card.md so the artifact is reproducible.
NUM_SAMPLES = 240
EPOCHS = 700
LR = 0.01
STEPS = 4

if __name__ == '__main__':
    loss = train_and_export_costmap_path(
        OUT, num_samples=NUM_SAMPLES, epochs=EPOCHS, lr=LR, steps=STEPS)
    # The exported .onnx inlines all weights and is self-contained; the torch
    # exporter may leave an orphan external-data sidecar, so drop it.
    sidecar = OUT + '.data'
    if os.path.exists(sidecar):
        os.remove(sidecar)
    print('exported %s (final loss %.4f, %d bytes)' % (OUT, loss, os.path.getsize(OUT)))
