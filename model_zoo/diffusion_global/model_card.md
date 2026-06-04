# Model Card: diffusion_global_costmap_flow_v0

> The first curated generative model in this repo's zoo, and the first learned
> model wired end-to-end through the C++ inference path (not a unit-test fixture).
> Manifest: [manifest.yaml](manifest.yaml). Reproduce: [export.py](export.py).

## Summary

- **Model family:** flow matching (conditional, 4 Euler integration steps)
- **Task:** global **path** proposal for the Nav2 **Mode B** planner
  (`nav2_diffusion_global_planner::DiffusionGlobalPlanner` →
  `nav2_diffusion_onnx::OnnxPathModel`)
- **Robot kinematics:** generic (geometric paths; no kinodynamic assumptions)
- **Inputs:** goal-aligned local costmap patch (24×24) + goal distance
- **Output:** 5 candidate start→goal paths × 12 waypoints, validated and selected
  by the planner's deterministic costmap layer
- **Runtime / precision:** ONNX / fp32
- **Artifact:** `costmap_flow.onnx` (≈349 KB), checked in directly because it is
  small and fully reproducible from `export.py`.

## Intended use

Demonstration / research only. It exists to prove the *propose → validate*
architecture with a real learned proposer: the model reads the costmap and biases
**all** of its proposals toward the obstacle-free side, then the planner's
deterministic validity layer keeps the shortest collision-free one. This is
something the built-in analytic `FanPathModel` cannot do — the fan proposes
symmetric bows blind to the map and relies entirely on the validity layer to
filter them.

## Out-of-scope / limitations

Honest and load-bearing (these are why the catalog benchmark shows it failing the
hard scenarios):

- **Synthetic data only.** Never validated on a real robot or rosbag.
- **Lateral veer ≲ 1.5 m.** It cannot propose a large detour, so it does **not**
  solve the benchmark's off-centre-gap scenario (a 2 m detour) — the analytic fan
  (whose `max_bow_fraction` reaches 2 m) does. The ceiling here is the *training
  distribution*, not the architecture: richer expert data lifts it.
- **Single-mode avoidance.** Handles a one-sided obstacle; cannot produce an
  S-shaped / slalom route.
- **Weak / far / small obstacle signals** may not flip every candidate. It is
  robust on a clear one-sided band ahead; a thin patch far off the centre line can
  leave some candidates un-deflected. The validity layer still gates the output.
- **Not a search planner.** No completeness guarantee; on cluttered maps use
  upstream NavFn / Smac, or a classical planner in this repo.
- **Do not deploy on hardware.**

## Training data

`nav2_diffusion_training.path_planners.make_costmap_path_dataset` (240 samples).
Each configuration places a single obstacle band on the +y or −y half of the
goal-aligned patch, with **varied lateral width and forward extent** so the model
learns from strong and partial signals, and is emitted as a **mirrored +y/−y
pair** so the response is symmetric (no built-in left/right bias). **Clear
(no-obstacle) samples** with a straight target anchor the unconditioned behaviour
to the centre line; the K latents still fan the proposals out around it. Expert
paths are a half-sine bow toward the open side (amplitude 0.9 m). Goal distances
sweep 3–5 m. Fully procedural; no real data.

## Benchmark results

This is a research placeholder, so it is checked **behaviourally**, not against the
full Nav2 deployment benchmark:

- **Costmap side-selection (end-to-end C++):** obstacle on the left → all 5
  proposed candidates veer right (mean lateral ≈ −0.6 m); obstacle on the right →
  all veer left (≈ +0.6 m); no obstacle → centred (≈ 0). Asserted in
  `nav2_diffusion_onnx`'s `test_onnx_path_model`
  (`CuratedZooModelVeersAwayFromObstacle`, runs where onnxruntime is available).
- **Catalog comparison:** appears as *Diffusion (Mode B, learned)* in
  [docs/planner_comparison.md](../../docs/planner_comparison.md). It succeeds on
  *clear* and on a single-sided *side obstacle*, and reports *no path* on
  *off-centre gap* and *slalom* — the limitations above, made measurable.
- **Collisions:** 0 by construction — the planner's deterministic validity layer
  gates every proposal regardless of model quality.

## Safety

- The model has **no** safety authority. Every proposal passes through the
  planner's collision check; if none is free, the planner raises
  `NoValidPathCouldBeFound` (fail-closed) rather than returning an unsafe path.
- Failure cases are the limitations above (large detours, S-shapes) — they
  surface as *no path*, never as a collision.
- Not for hardware use; no ODD validation performed.

## Reproducibility

- **Command:** `PYTHONPATH=../../nav2_diffusion_training CUDA_VISIBLE_DEVICES= python3 export.py`
- **Seed:** 0 (`torch.manual_seed(0)` inside the training function)
- **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0, exported on CPU
- **Hyperparameters:** 240 samples, 700 epochs, lr 0.01, 4 flow steps (set in
  `export.py`; larger than the `path_planners` defaults for a symmetric, unbiased
  response)
- Bit-for-bit reproduction may vary across torch versions / hardware; behaviour
  (side-selection, veer magnitude) is stable.

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated here) · Code: Apache-2.0
