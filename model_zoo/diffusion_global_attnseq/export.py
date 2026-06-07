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
Reproduce the curated costmap-conditioned *attnseq* Mode B path model here.

This trains nav2_diffusion_training.path_planners.CostmapPathAttnSeqPlanner — a
no-fan family: a cross-attention perception tokenizer feeds a per-step
cross-attention GRU decoder that reads the costmap memory as it rolls each path
out one waypoint at a time, with K learned seeds for candidate diversity (no
lateral fan). It is the first Mode B model here to thread **every** benchmark
course as a pure-generative proposer (8/8), including the two that were long
documented as ceilings — the S-shaped *slalom* and the *far off-centre gap*.

Those were not an architecture ceiling but two DATA bugs (see
docs/generative_limits.md):

1. the slalom expert was a two-Gaussian bump that reached each slot offset only at
   the band centre, so it grazed the walls at the band edges — no fit could thread
   it. Fixed with a collision-clean **plateau** expert that holds the slot offset
   across the whole (thin) wall band; and
2. the training patch (hand-filled ``_gap_patch``) could land a row off the patch
   the deployed planner samples from the live costmap. Fixed by building training
   patches through the same fine-grid resample (``_resampled_aligned_patch``).

With those fixes a no-fan family trained on the five-way 'all' mix (one-sided +
off-centre/far gap + centred/narrow gap + double gate + slalom) threads all eight.

Trains on the GPU when available; always exports on CPU for a portable artifact.
GPU training is not bit-exact run-to-run (the committed artifact is 8/8 in the C++
planner_benchmark; a re-export reproduces the capability but may vary slightly).
Deterministic CPU rebuild:

    PYTHONPATH=../../generative/nav2_diffusion_training CUDA_VISIBLE_DEVICES= python3 export.py

Mode B PathModel ONNX contract (consumed by nav2_diffusion_onnx::OnnxPathModel):

    context [1, 2] = [goal_distance, 0]
    costmap [1, 1, 24, 24]            (goal-aligned patch; row -> fwd x, col -> +y)
    ->  paths [1, 5, 12, 2]           (x, y in the goal-aligned frame)
"""

import os

from nav2_diffusion_training.path_planners import (
    CostmapPathAttnSeqPlanner, CTX_DIM, PATH_COSTMAP_SIZE, _path_dataset)

import onnx

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'costmap_attnseq.onnx')

# Curated hyperparameters. The loss is the no-fan recon MSE (every candidate targets
# the plateau expert directly; the K learned seeds supply diversity) plus a jerk
# smoothness penalty. Cosine LR decay + gradient clipping + best-checkpoint keep the
# autoregressive rollout from diverging late in training. Documented in model_card.md.
NUM_SAMPLES = 400
EPOCHS = 3000
LR = 0.004
DEVICE = 'cuda' if torch.cuda.is_available() else None


def main():
    dev = torch.device(DEVICE) if DEVICE else torch.device('cpu')
    torch.manual_seed(0)
    ctx, cm, tgt = _path_dataset('all', NUM_SAMPLES)
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
