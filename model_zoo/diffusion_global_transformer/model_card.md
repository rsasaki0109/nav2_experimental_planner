# Model Card: diffusion_global_costmap_transformer_v0

> The curated generative **global path** (Mode B) model in the **transformer**
> family — a DETR-style set-prediction planner that **threads the footprint-validated
> off-centre gap** as a pure-generative planner (the first Mode B model here to do so
> without a classical fallback). Manifest: [manifest.yaml](manifest.yaml). Reproduce:
> [export.py](export.py). Sibling:
> [../diffusion_global/model_card.md](../diffusion_global/model_card.md).

## Summary

- **Model family:** transformer set prediction (1 deterministic forward pass)
- **Task:** global **path** proposal for the Nav2 **Mode B** planner
  (`nav2_diffusion_global_planner::DiffusionGlobalPlanner` →
  `nav2_diffusion_onnx::OnnxPathModel`)
- **Inputs:** goal-aligned costmap patch (24×24, 6×6 m window) + goal distance
- **Output:** 5 candidate start→goal paths × 12 waypoints in the goal-aligned frame,
  validated against the live costmap by the planner
- **Runtime / precision:** ONNX / fp32
- **Artifact:** `costmap_transformer.onnx` (small, checked in, reproducible from
  `export.py`).

## What it shows — pure-generative threading of the validated off-centre gap

[docs/generative_limits.md](../../docs/generative_limits.md) documented a ceiling:
the off-centre gap (a wall blocking the straight line with a 1 m slot ~2 m off-axis)
could **not** be threaded by a pure-generative Mode B model. The flow / recurrent
models (a 16-d CNN embedding) cannot even *aim* at an off-centre slot; an earlier
gap-trained transformer could aim but its proposals still grazed the slot edge, so
the footprint-validity layer found no survivor (`no path`). Three pure-imitation
levers — aim, candidate fan, plateau expert — did not cross it.

This model crosses it by combining two ingredients:

1. **Attention over costmap tokens (aim).** A strided conv tokenizes the patch into
   spatial cells with learned positions; K query tokens **cross-attend** over them,
   so the model localizes the off-centre slot and bends every candidate toward it —
   which the flow / recurrent 16-d CNN embedding cannot.
2. **Differentiable footprint-clearance loss (validator-aware).**
   `path_planners._footprint_penalty` samples a Gaussian-blurred obstacle-proximity
   field along the densely-interpolated path and penalizes overlap, so training
   optimizes the proposals to be *what the validity layer accepts* — pulling each
   candidate's wall crossing into the free slot with margin (the blur gives a gradient
   even where a raw occupancy penalty is flat in the wall interior; the dense
   interpolation samples the crossing like the C++ `isPathValid` does).

**Result (verified in the real C++ `planner_benchmark`):** this model threads the
footprint-validated *off-centre gap* — `Diffusion (Mode B, transformer)`: *off-centre
gap* = **yes, ~5.5 m, 12-pose generative path, ~0.2 ms, no fallback** — and also the
*far off-centre gap* (wall pushed ~3 m forward), while the flow and recurrent Mode B
models report *no path* on both. It keeps *clear*, *side obstacle*, and the on-line
*double gate*.

**Honest trade-off (an 8-course sweep surfaced this).** The off-centre aim is a
**specialization, not a strict upgrade**: this transformer **over-aims** and now
**fails the *centred gap* and *narrow gap*** (a slot dead ahead on the straight line)
that the flow and recurrent siblings thread trivially. The two CNN-embedding families
and the attention transformer are therefore **complementary** — dead-ahead gaps vs
off-axis slots — not strictly ordered. A centred-sample rebalance of the `'both'`
dataset is the likely path to having both (future work).

## Honest scope — what it does NOT do

- **Centred / narrow on-line gaps are no-path (the specialization cost).** Having been
  trained to aim off-axis, this model misses a slot sitting **dead ahead** on the
  straight line — the flow and recurrent siblings (and the hybrid) cover those.
- **Slalom is still no-path.** The S-shaped *slalom* (two staggered walls, two
  crossings) is beyond a single forward-crossing generative proposal; the **hybrid**
  planner remains the completeness guarantee there.
- **Not complete.** Pure generative gives no any-map guarantee. Off-centre gap
  threading generalizes well across wall forward-distance (it threads both the ~2 m and
  the ~3 m-forward *far off-centre gap*) — an earlier claim that threading was *bounded*
  to the ~2 m training span is **retracted** — but completeness on arbitrary maps is
  still the hybrid / search planners' job.
- **GPU training is not bit-exact** run-to-run, so which candidates thread can vary,
  but the **committed artifact threads the gap in the C++ benchmark** (checksum fixed
  in the manifest).

## Intended use

Research demonstration that the proposal-stage gap ceiling is a matter of
representation (attention) and loss (validator-aware footprint term), not a
fundamental limit — and a drop-in costmap-conditioned Mode B model that, unlike the
flow / recurrent siblings, threads the validated off-centre gap. Not a deployment
model; the deterministic costmap-validity layer gates every proposal and the hybrid
planner remains the completeness guarantee for slalom and general maps.

## Out-of-scope / limitations

- **Synthetic data only.** Never validated on a real robot or rosbag.
- **Centred / narrow on-line gaps unsolved** (over-aim trade-off); flow / recurrent /
  hybrid solve them.
- **Slalom unsolved by pure generative** (two-crossing S); the hybrid solves it.
- **Off-axis bounded, not forward-distance bounded.** Gap threading is demonstrated for
  slots up to ~2 m off-axis (and across wall forward-distance, ~2–3 m), inside the
  24×24 / 6×6 m patch; classical search / the hybrid own the general problem.
- **Research model.** Do not deploy on hardware; the validity layer is the authority.

## Training data

`make_costmap_path_dataset` (one-sided obstacle → bow to the free side) +
`make_costmap_path_gap_dataset` (wall with one off-centre slot → expert routes through
the slot via a Gaussian detour peaking at the slot offset where the path crosses the
wall), combined as the `'both'` dataset (240 samples). Mirrored +y/−y pairs and clear
samples keep the response symmetric. Fully procedural; no real data.

## Benchmark results

- **Footprint-validated planner benchmark (real C++, 8 courses):** **threads
  *off-centre gap*** (yes, ~5.5 m, 12 poses) **and *far off-centre gap*** (~3 m-forward
  wall), plus *clear*, *side obstacle*, and the on-line *double gate*. **Trade-off:**
  it reports *no path* on *centred gap* / *narrow gap* (dead-ahead slots that flow /
  recurrent thread). *slalom* remains *no path* (hybrid solves it). See
  [docs/planner_comparison.md](../../docs/planner_comparison.md) and
  [docs/generative_limits.md](../../docs/generative_limits.md).
- **Off-centre-slot aiming (exported ONNX):** wall + slot at ±2 m → every candidate's
  lateral offset at the wall ≈ the slot offset (flow / recurrent fail this probe).
  Guarded by `OnnxPathModelTest.CuratedZooTransformerAimsAtOffCentreSlot`.
- **One-sided obstacle:** candidates veer to the free side.
- **Collisions:** the planner's deterministic costmap-validity layer gates every
  proposal regardless of model quality.

## Safety

- The model has **no** validity authority; the planner validates every proposal
  against the live costmap and falls back / reports no-path if none is valid.
- Not for hardware use; no ODD validation performed.

## Reproducibility

- **Command:** `PYTHONPATH=../../generative/nav2_diffusion_training python3 export.py`
  (CUDA when available; `CUDA_VISIBLE_DEVICES= ` forces a CPU build)
- **Seed:** 0 · **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0; trained on CUDA,
  exported on CPU
- **Hyperparameters:** transformer, `'both'` dataset, 240 samples, 2500 epochs,
  lr 0.01, footprint 3.0, blur_sigma 2.5
- GPU training is not bit-exact across runs/hardware; a re-export may vary slightly in
  which candidates thread but reproduces the capability (gap threading verified for the
  committed artifact).

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated here) · Code: Apache-2.0
