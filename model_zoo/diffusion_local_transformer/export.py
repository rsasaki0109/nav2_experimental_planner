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
Reproduce the curated costmap-conditioned *transformer* Mode A model here.

This trains nav2_diffusion_training.generative_planners.CostmapTransformerPlanner
(a DETR-style set-prediction decoder — K learned query tokens cross-attend to a
context token plus tokenized costmap patch, each decoding a full SE(2) trajectory
in one deterministic forward pass) on the package's synthetic one-sided-obstacle
dataset and exports `costmap_transformer.onnx`. It is the transformer member of
the model zoo, alongside the flow Mode A model in ../diffusion_local.

Training uses the GPU when one is available (the heavy loop benefits from it);
the model is always exported on CPU so the artifact is portable. For a
deterministic CPU rebuild, force the device off:

    PYTHONPATH=../../generative/nav2_diffusion_training CUDA_VISIBLE_DEVICES= python3 export.py

The exported file matches the Mode A TrajectoryModel ONNX contract consumed by
nav2_diffusion_onnx::OnnxTrajectoryModel:

    context [1, 4] = [goal_x, goal_y, linear_speed, max_angular_speed]
    costmap [1, 1, 32, 32]            (egocentric patch; col 0 = +y / left)
    ->  trajectories [1, 3, 10, 3]    (x, y, yaw in the robot base frame)
"""

import os

from nav2_diffusion_training.generative_planners import train_and_export_costmap

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'costmap_transformer.onnx')

# Curated hyperparameters. The transformer's loss is already a direct MSE onto the
# smooth pure-pursuit-arc expert (recon_loss), so the sampled trajectory is smooth
# and ordered by construction — no sample_weight / integration-steps tricks are
# needed for the controller's kinematic gate to accept it. A wide dataset (varied
# goals / obstacle bands, mirrored +y/-y pairs, clear samples) keeps the response
# symmetric: clear -> straight at the carrot; a one-sided obstacle -> every
# candidate veers to the open side. Documented in model_card.md.
KIND = 'transformer'
NUM_SAMPLES = 240
EPOCHS = 2000
LR = 0.01
DEVICE = 'cuda' if torch.cuda.is_available() else None

if __name__ == '__main__':
    loss = train_and_export_costmap(
        OUT, kind=KIND, num_samples=NUM_SAMPLES, epochs=EPOCHS, lr=LR,
        device=DEVICE)
    # The exported .onnx inlines all weights and is self-contained; the torch
    # exporter may leave an orphan external-data sidecar, so drop it.
    sidecar = OUT + '.data'
    if os.path.exists(sidecar):
        os.remove(sidecar)
    print('exported %s on %s (final loss %.4f, %d bytes)' % (
        OUT, DEVICE or 'cpu', loss, os.path.getsize(OUT)))
