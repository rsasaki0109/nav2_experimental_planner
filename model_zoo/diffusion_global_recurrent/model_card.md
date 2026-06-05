# Model Card: diffusion_global_costmap_recurrent_v0

> The curated generative **global path** (Mode B) model in the **recurrent**
> family — a GRU that emits each path **one waypoint at a time**, the Mode B
> analogue of the recurrent local trajectory model. Manifest:
> [manifest.yaml](manifest.yaml). Reproduce: [export.py](export.py). Siblings:
> [../diffusion_global/model_card.md](../diffusion_global/model_card.md) (flow),
> [../diffusion_global_transformer/model_card.md](../diffusion_global_transformer/model_card.md) (transformer),
> [../diffusion_local_recurrent/model_card.md](../diffusion_local_recurrent/model_card.md) (Mode A recurrent).

## Summary

- **Model family:** recurrent — GRU autoregressive rollout (H=12 sequential steps)
- **Task:** global **path** proposal for the Nav2 **Mode B** planner
  (`nav2_diffusion_global_planner::DiffusionGlobalPlanner` →
  `nav2_diffusion_onnx::OnnxPathModel`)
- **Inputs:** goal-aligned costmap patch (24×24, 6×6 m window) + goal distance
- **Output:** 5 candidate start→goal paths × 12 waypoints in the goal-aligned frame,
  validated against the live costmap by the planner
- **Runtime / precision:** ONNX / fp32
- **Artifact:** `costmap_recurrent.onnx` (small, checked in, reproducible from
  `export.py`).

## What it shows — a third Mode B family, distinct inductive bias

Mode B (the global PathModel seam) already had two generative families: the **flow**
model (iterative denoising of K fixed latents) and the **transformer** (one-shot
set-prediction decode with cross-attention over costmap tokens). This adds a third:
a **recurrent** GRU rollout. A 16-d CNN embedding of the goal-aligned patch plus the
goal-distance context forms a conditioning vector; K=5 learned seed vectors give each
candidate a distinct initial state, and a single GRUCell + linear head then emits the
path waypoint by waypoint, feeding the previous point back in. The sequential
inductive bias matches a path's waypoint-by-waypoint structure — a different way to
arrive at the **same PathModel contract**, demonstrating the seam carries the same
family on both the local (Mode A) and global (Mode B) sides.

**Honest scope — a benchmark *peer* of the flow model; not a gap solver.** This
model conditions on the **same 16-d CNN embedding** as the flow Mode B model, *not*
the transformer's attention over explicit costmap tokens. So it learns to **pick the
free side** of a one-sided obstacle and bows every candidate away from it, but it
does **not** localize an off-centre slot — the same representational ceiling the flow
model has ([docs/generative_limits.md](../../docs/generative_limits.md)). Only the
transformer aims at the slot, and even that aim does not thread the narrow
footprint-validated gap; the **hybrid** planner (generative propose → classical
search dispose) remains the completeness guarantee. The K candidates are trained as a
small lateral fan around the routing expert so the footprint validator gets a spread
of options around the chosen side. Cost: the autoregressive rollout takes H=12
sequential steps — the **highest inference latency** of the three Mode B families
(flow 4 integration steps, transformer 1 forward pass).

## Intended use

Research demonstration that the PathModel seam carries a sequential (world-model
style) family alongside denoising and set-prediction families, on the same contract.
Not a deployment model and not a drop-in upgrade over the flow Mode B model for the
benchmark; the deterministic costmap-validity layer gates every proposal and the
hybrid planner remains the completeness guarantee.

## Out-of-scope / limitations

- **Synthetic data only.** Never validated on a real robot or rosbag.
- **Benchmark peer, not a gap solver.** Picks the free side of a one-sided obstacle
  (peer of the flow model); does **not** aim at or thread an off-centre gap (CNN
  embedding ceiling — the transformer aims, the hybrid threads).
- **Highest latency of the Mode B families.** H=12 sequential rollout steps vs flow's
  4 integration steps and the transformer's single forward pass.
- **Window-bounded.** Side selection demonstrated inside the 24×24 / 6×6 m goal-aligned
  patch; classical search / the hybrid planner own the general routing problem.
- **Research model.** Do not deploy on hardware; the validity layer is the authority.

## Training data

`make_costmap_path_dataset` (`'side'` dataset): a one-sided obstacle ahead with the
expert path bowing to the open side, emitted as mirrored +y/−y pairs plus clear
(straight-line) anchors so the response is symmetric and robust to partial obstacle
signals (240 samples). Fully procedural; no real data.

## Benchmark results

Research placeholder → checked **behaviourally** at the proposal level:

- **One-sided obstacle (exported ONNX):** obstacle on +y → every candidate's mid
  waypoint leans −y (and vice versa); the rollout advances forward in x. Guarded by
  `OnnxPathModelTest.CuratedZooPathRecurrentVeersAwayFromObstacle`.
- **Off-centre slot:** not aimed (CNN-embedding ceiling, same as the flow model; the
  transformer aims and the hybrid threads).
- **Collisions:** the planner's deterministic costmap-validity layer gates every
  proposal regardless of model quality.

## Safety

- The model has **no** validity authority; the planner validates every proposal
  against the live costmap and falls back / reports no-path if none is valid.
- Not for hardware use; no ODD validation performed.

## Reproducibility

- **Command:** `PYTHONPATH=../../generative/nav2_diffusion_training python3 export.py`
  (CUDA when available; `CUDA_VISIBLE_DEVICES= ` forces a deterministic CPU build)
- **Seed:** 0 · **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0; trained on CUDA,
  exported on CPU
- **Hyperparameters:** recurrent, `'side'` dataset, 240 samples, 2000 epochs, lr 0.01
- Bit-for-bit reproduction may vary across torch versions / hardware; behaviour
  (side selection, forward rollout) is stable.

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated here) · Code: Apache-2.0
