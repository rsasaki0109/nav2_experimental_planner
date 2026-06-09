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
Reproduce the curated costmap-conditioned *transformer* Mode B path model here.

This trains nav2_diffusion_training.path_planners.CostmapPathTransformerPlanner
(a DETR-style set-prediction decoder that cross-attends over tokenized costmap
patch cells) on the combined one-sided-obstacle + off-centre-gap dataset with a
**differentiable footprint-clearance loss**, and exports `costmap_transformer.onnx`.
Attention over explicit costmap tokens lets the model *aim* its proposals at an
off-centre slot (which the flow model's 16-d CNN embedding cannot); the footprint
loss then optimizes the proposals to be *what the validity layer accepts*, pulling
each candidate's wall crossing into the free slot with margin. Together they let
this pure-generative model **thread the footprint-validated off-centre gap** in
`planner_comparison.md` — the first Mode B model to do so without a classical
fallback (flow / recurrent still report no path there; see
docs/generative_limits.md). It keeps the clear / side-obstacle competence; the
S-shaped *slalom* (two staggered walls) remains a no-path for pure generative.

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
OUT = os.path.join(HERE, 'costmap_transformer.onnx')

# Curated hyperparameters. The loss is the fan-recon MSE onto the smooth routing
# expert (a lateral fan so the K candidates spread for the validator), a jerk
# penalty for smoothness, and the footprint-clearance term. FOOTPRINT weights that
# term; BLUR_SIGMA (cells) blurs the patch occupancy into a smooth proximity field
# so a candidate stranded in the wall interior still feels a pull toward the slot
# (a raw occupancy penalty has zero gradient there). The 'both' dataset mixes
# one-sided obstacles (pick the free side) and off-centre gaps (route through the
# slot). Documented in model_card.md.
NUM_SAMPLES = 240
EPOCHS = 2500
LR = 0.01
FOOTPRINT = 3.0
BLUR_SIGMA = 2.5
DEVICE = 'cuda' if torch.cuda.is_available() else None

if __name__ == '__main__':
    loss = train_and_export_costmap_path(
        OUT, num_samples=NUM_SAMPLES, epochs=EPOCHS, lr=LR,
        kind='transformer', dataset='both', device=DEVICE,
        footprint=FOOTPRINT, blur_sigma=BLUR_SIGMA)
    sidecar = OUT + '.data'
    if os.path.exists(sidecar):
        os.remove(sidecar)
    print('exported %s on %s (final loss %.4f, %d bytes)' % (
        OUT, DEVICE or 'cpu', loss, os.path.getsize(OUT)))
