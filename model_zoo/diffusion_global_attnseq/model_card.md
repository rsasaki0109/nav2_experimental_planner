# Model Card: diffusion_global_costmap_attnseq_v0

> The curated generative **global path** (Mode B) model in the **attnseq** family —
> a no-fan, autoregressive cross-attention planner that **threads every benchmark
> course as a pure-generative proposer (8/8)**, including the two long documented as
> ceilings: the S-shaped **slalom** and the **far off-centre gap**. Manifest:
> [manifest.yaml](manifest.yaml). Reproduce: [export.py](export.py). Siblings:
> [../diffusion_global_transformer/model_card.md](../diffusion_global_transformer/model_card.md),
> [../diffusion_global/model_card.md](../diffusion_global/model_card.md).

## Summary

- **Model family:** attnseq — cross-attention perception + per-step cross-attention
  GRU rollout (autoregressive), K learned seeds, **no lateral fan**
- **Task:** global **path** proposal for the Nav2 **Mode B** planner
  (`nav2_diffusion_global_planner::DiffusionGlobalPlanner` →
  `nav2_diffusion_onnx::OnnxPathModel`)
- **Inputs:** goal-aligned costmap patch (24×24, 6×6 m window) + goal distance
- **Output:** 5 candidate start→goal paths × 12 waypoints in the goal-aligned frame,
  validated against the live costmap by the planner
- **Runtime / precision:** ONNX / fp32
- **Artifact:** `costmap_attnseq.onnx` (checked in, reproducible from `export.py`).

## What it shows — pure-generative threading of ALL eight courses

[docs/generative_limits.md](../../docs/generative_limits.md) long documented two
pure-generative ceilings: the **slalom** (two staggered walls, an S-shaped *two*-lateral-
crossing detour) and the **far off-centre gap**. Three architecture families
(capacity transformer, MLP head, an earlier attnseq) all plateaued at the same training
loss on slalom, which read as an *architecture* ceiling.

It was not. It was **two data bugs**, found by reproducing the C++ validator faithfully:

1. **The expert grazed its own walls.** The slalom expert was a narrow two-Gaussian
   bump that reached each slot offset only at the *band centre*; across the rest of the
   (forward-thick) wall band the path was still inside the wall. The training target
   itself was a colliding path — no fit of it could thread (max occupancy 0.5–1.0 under
   the footprint validator). **Fix:** a collision-clean **plateau** expert that *holds*
   the slot offset across the whole thin wall band and only weaves between offsets in the
   free gap (max occupancy ~0).
2. **The training patch didn't match the deployed one.** The hand-filled `_gap_patch`
   floors `x_lo·S/fwd`, while the deployed `OnnxPathModel::alignedPatch` point-samples the
   live costmap at each cell *centre* — so a thin wall could land a whole row off. The
   model threaded a hand-built slalom patch but went straight on the resampled one.
   **Fix:** build every training patch through the same fine-grid resample
   (`_resampled_aligned_patch`), so the training distribution matches inference.

With those fixes a **no-fan** family does the rest. The transformer / recurrent
families train K candidates as a uniform **lateral fan** (all candidates shifted by a
constant), which cannot represent an S — it shifts *both* crossings off their slots.
attnseq drops the fan: K learned seeds let each candidate take any shape, and a per-step
cross-attention GRU reads the costmap as it rolls the path out, so a candidate can bend
into slot A then back through slot B.

**Result (verified in the real C++ `planner_benchmark`, 8 courses, pure generative, no
fallback):** `Diffusion (Mode B, attnseq)` threads **all eight** — *clear*, *centred
gap*, *narrow gap*, *off-centre gap*, **far off-centre gap**, *double gate*, **slalom**,
*side obstacle* — every path the model's own 12-pose output (~3–4 ms). This is the first
Mode B model here to thread the slalom and the far off-centre gap as a pure-generative
proposer, and it **strictly exceeds** the transformer (6/8: no slalom, no far off-centre).

## Honest scope — what it does NOT do

- **Not a completeness guarantee.** 8/8 holds on the benchmark's curated wall courses
  (geometry the training mix matches). On arbitrary / far-out-of-distribution maps the
  **hybrid** planner's classical fallback remains the completeness authority.
- **Synthetic data only.** Never validated on a real robot or rosbag.
- **Off-axis bounded.** Slot aiming demonstrated for offsets up to ~2 m within the
  24×24 / 6×6 m patch window.
- **Highest latency of the Mode B families** (autoregressive rollout; still ~4 ms here).
- **GPU training is not bit-exact** run-to-run, so which candidates thread the borderline
  courses can vary — but the **committed artifact is 8/8 in the C++ benchmark** (checksum
  fixed in the manifest).

## Intended use

Research demonstration that the *slalom* and *far off-centre* "ceilings" were a data
artifact (a colliding expert + a train/inference patch mismatch), not a fundamental
limit of pure-generative proposal — and a drop-in costmap-conditioned Mode B model that
threads every benchmark course. Not a deployment model; the deterministic costmap-validity
layer gates every proposal and the hybrid planner remains the completeness guarantee for
general maps.

## Training data

Five-way `'all'` mix (400 samples): `make_costmap_path_dataset` (one-sided obstacle),
`make_costmap_path_gap_dataset` (off-centre / far off-centre slot routing),
`make_costmap_path_centred_gap_dataset` (dead-ahead / narrow slot),
`make_costmap_path_double_gate_dataset` (two on-line gates) and
`make_costmap_path_slalom_dataset` (S-shaped two-crossing detour). **All** wall courses
use collision-clean **plateau** experts and **deployment-matched** patches
(`_resampled_aligned_patch`). Mirrored ±y pairs keep the response symmetric. Fully
procedural; no real data.

## Benchmark results

- **Footprint-validated planner benchmark (real C++, 8 courses):** **8/8** as a pure
  generative planner (12-pose model paths, no fallback) — including *slalom* (~9.9 m) and
  *far off-centre gap* (~6.7 m). See
  [docs/planner_comparison.md](../../docs/planner_comparison.md) and
  [docs/generative_limits.md](../../docs/generative_limits.md).
- **Collisions:** the planner's deterministic costmap-validity layer gates every
  proposal regardless of model quality.

## Safety

- The model has **no** validity authority; the planner validates every proposal against
  the live costmap and falls back / reports no-path if none is valid.
- Not for hardware use; no ODD validation performed.

## Reproducibility

- **Command:** `PYTHONPATH=../../generative/nav2_diffusion_training python3 export.py`
  (CUDA when available; `CUDA_VISIBLE_DEVICES= ` forces a CPU build)
- **Seed:** 0 · **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0; trained on CUDA,
  exported on CPU
- **Hyperparameters:** attnseq (dim 64 / 8 heads), `'all'` five-way dataset, 400 samples,
  3000 epochs, lr 0.004 cosine, grad-clip 1.0, best-checkpoint
- GPU training is not bit-exact across runs/hardware; a re-export may vary slightly in
  which candidates thread the borderline courses but reproduces the 8/8 capability
  (verified for the committed artifact).

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated here) · Code: Apache-2.0
