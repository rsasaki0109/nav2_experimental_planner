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
Reproduce the curated *kinematics-conditioned* Mode B path model here.

One model serves several steering geometries: the second context input carries the
vehicle's **min turn radius R** (0.0 = omni-directional), and the model shapes each
maneuver's detour so its peak curvature ~ 1/R — a sharp, tight detour for an omni / a
differential-drive robot (small R, can pivot) and a gentle, wide one for an Ackermann
car (large R) through the *same* gap. The planner's curvature validator (`min_turn_radius`
parameter) then *disposes* of any proposal that turns tighter than the vehicle can — the
propose/dispose split extended from footprint to vehicle dynamics.

Architecture is the same no-fan attnseq family as diffusion_global_attnseq; only the
training data differs (make_costmap_path_kinematics_dataset: R-shaped detours across
*all eight* benchmark courses — clear / centred / double gate (straight, R-invariant),
off-centre & far off-centre gap and side obstacle (R-shaped), and an omni-only slalom
block — with deployment-matched patches). The benchmark slalom's +/-2 m crossings in a
~1.6 m gap need curvature ~1/0.02 m, past any wheeled turning circle, so slalom threads
for omni (R=0) only and the curvature validator disposes it for diff / Ackermann.

Trains on the GPU when available; always exports on CPU for a portable artifact.
Deterministic CPU rebuild:

    PYTHONPATH=../../generative/nav2_diffusion_training CUDA_VISIBLE_DEVICES= python3 export.py

Mode B PathModel ONNX contract (consumed by nav2_diffusion_onnx::OnnxPathModel):

    context [1, 2] = [goal_distance, min_turn_radius_R]
    costmap [1, 1, 24, 24]            (goal-aligned patch; row -> fwd x, col -> +y)
    ->  paths [1, 5, 12, 2]           (x, y in the goal-aligned frame)
"""

import os

from nav2_diffusion_training.path_planners import (
    CostmapPathAttnSeqPlanner, CTX_DIM, PATH_COSTMAP_SIZE, _path_dataset)

import onnx

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'costmap_kinematics.onnx')

NUM_SAMPLES = 500
EPOCHS = 3500
LR = 0.004
DEVICE = 'cuda' if torch.cuda.is_available() else None


def main():
    dev = torch.device(DEVICE) if DEVICE else torch.device('cpu')
    torch.manual_seed(0)
    ctx, cm, tgt = _path_dataset('kinematics', NUM_SAMPLES)
    ctx, cm, tgt = ctx.to(dev), cm.to(dev), tgt.to(dev)
    model = CostmapPathAttnSeqPlanner().to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=LR)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=EPOCHS)
    model.train()
    best = float('inf')
    best_state = None
    for _ in range(EPOCHS):
        opt.zero_grad()
        loss = model.recon_loss(ctx, cm, tgt)
        out = model(ctx[:24], cm[:24])
        jerk = out[:, :, 2:, :] - 2 * out[:, :, 1:-1, :] + out[:, :, :-2, :]
        loss = loss + 2.0 * (jerk ** 2).mean()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        opt.step()
        sched.step()
        lv = float(loss.item())
        if lv < best:
            best = lv
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
    model.load_state_dict(best_state)
    model.eval().to('cpu')
    torch.onnx.export(
        model, (torch.zeros(1, CTX_DIM), torch.zeros(1, 1, PATH_COSTMAP_SIZE, PATH_COSTMAP_SIZE)),
        OUT, input_names=['context', 'costmap'], output_names=['paths'], opset_version=18)
    onnx.save_model(onnx.load(OUT), OUT, save_as_external_data=False)
    sidecar = OUT + '.data'
    if os.path.exists(sidecar):
        os.remove(sidecar)
    print('exported %s on %s (best loss %.4f, %d bytes)' % (
        OUT, DEVICE or 'cpu', best, os.path.getsize(OUT)))


if __name__ == '__main__':
    main()
