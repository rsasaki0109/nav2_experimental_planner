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
Reproduce the curated costmap-conditioned *recurrent* Mode B path model here.

This trains nav2_diffusion_training.path_planners.CostmapPathRecurrentPlanner
(a GRU that emits each path one waypoint at a time, feeding the previous point
back in) on the one-sided-obstacle dataset, and exports `costmap_recurrent.onnx`.
It is the Mode B (global path) analogue of the recurrent local trajectory model
(../diffusion_local_recurrent): the sequential inductive bias matches a path's
waypoint-by-waypoint structure, vs the transformer's one-shot set decode and the
flow model's iterative denoising.

Like the flow Mode B model (../diffusion_global), this model conditions on a
16-d CNN embedding of the costmap patch, so it learns to **pick the free side**
of a one-sided obstacle — it does NOT route through an arbitrary off-centre gap.
Localizing a gap requires attention over explicit costmap tokens, which only the
transformer Mode B model has (../diffusion_global_transformer); the gap-routing
ceiling that the flow model could not cross (docs/generative_limits.md) is
unchanged here.

Trains on the GPU when available; always exports on CPU for a portable artifact.
Deterministic CPU rebuild:

    PYTHONPATH=../../generative/nav2_diffusion_training CUDA_VISIBLE_DEVICES= python3 export.py

Mode B PathModel ONNX contract (consumed by nav2_diffusion_onnx::OnnxPathModel):

    context [1, 2] = [goal_distance, 0]
    costmap [1, 1, 24, 24]            (goal-aligned patch; row -> fwd x, col -> +y)
    ->  paths [1, 5, 12, 2]           (x, y in the goal-aligned frame)
"""

import os

from nav2_diffusion_training.path_planners import train_and_export_costmap_path

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'costmap_recurrent.onnx')

# Curated hyperparameters. recon_loss is a direct MSE onto a lateral fan around
# the smooth routing expert, plus a jerk penalty, so the K candidates stay smooth
# and spread without flow-step tuning. The 'side' dataset is the one-sided
# obstacle distribution (pick the free side) the CNN-embedding conditioning can
# represent. Documented in model_card.md.
NUM_SAMPLES = 240
EPOCHS = 2000
LR = 0.01
DEVICE = 'cuda' if torch.cuda.is_available() else None

if __name__ == '__main__':
    loss = train_and_export_costmap_path(
        OUT, num_samples=NUM_SAMPLES, epochs=EPOCHS, lr=LR,
        kind='recurrent', dataset='side', device=DEVICE)
    sidecar = OUT + '.data'
    if os.path.exists(sidecar):
        os.remove(sidecar)
    print('exported %s on %s (final loss %.4f, %d bytes)' % (
        OUT, DEVICE or 'cpu', loss, os.path.getsize(OUT)))
