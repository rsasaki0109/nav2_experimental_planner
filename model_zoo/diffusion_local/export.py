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
Reproduce the curated costmap-conditioned Mode A trajectory model here.

This trains nav2_diffusion_training.generative_planners.CostmapFlowPlanner on the
package's synthetic one-sided-obstacle dataset and exports `costmap_flow.onnx`,
the model the card / manifest describe. Training is deterministic
(torch.manual_seed(0) inside the training function); run on CPU for portability:

    PYTHONPATH=../../generative/nav2_diffusion_training CUDA_VISIBLE_DEVICES= python3 export.py

The exported file matches the Mode A TrajectoryModel ONNX contract consumed by
nav2_diffusion_onnx::OnnxTrajectoryModel:

    context [1, 4] = [goal_x, goal_y, linear_speed, max_angular_speed]
    costmap [1, 1, 32, 32]            (egocentric patch; col 0 = +y / left)
    ->  trajectories [1, 3, 10, 3]    (x, y, yaw in the robot base frame)
"""

import os

from nav2_diffusion_training.generative_planners import train_and_export_costmap

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'costmap_flow.onnx')

# Curated hyperparameters. Larger than the generative_planners defaults so the
# response is symmetric and unbiased (clear -> straight; a one-sided obstacle ->
# every candidate veers to the open side). STEPS=4 and SAMPLE_WEIGHT>0 (a direct
# MSE to the smooth expert target) keep the sampled trajectory smooth and ordered
# so its per-step speeds stay within the controller's kinematic limits and the
# safety gate accepts it. Documented in model_card.md.
KIND = 'flow'
NUM_SAMPLES = 240
EPOCHS = 1500
LR = 0.01
STEPS = 4
SAMPLE_WEIGHT = 30.0

if __name__ == '__main__':
    loss = train_and_export_costmap(
        OUT, kind=KIND, num_samples=NUM_SAMPLES, epochs=EPOCHS, lr=LR,
        steps=STEPS, sample_weight=SAMPLE_WEIGHT)
    # The exported .onnx inlines all weights and is self-contained; the torch
    # exporter may leave an orphan external-data sidecar, so drop it.
    sidecar = OUT + '.data'
    if os.path.exists(sidecar):
        os.remove(sidecar)
    print('exported %s (final loss %.4f, %d bytes)' % (OUT, loss, os.path.getsize(OUT)))
