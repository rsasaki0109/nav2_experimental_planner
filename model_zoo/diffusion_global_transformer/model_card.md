# Model Card: diffusion_global_costmap_transformer_v0

> The curated generative **global path** (Mode B) model in the **transformer**
> family — a DETR-style set-prediction planner whose raw proposals **aim at an
> off-centre slot** where the flow Mode B model's proposals cannot. Manifest:
> [manifest.yaml](manifest.yaml). Reproduce: [export.py](export.py). Sibling:
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

## What it shows — a representational advance, not a benchmark win

[docs/generative_limits.md](../../docs/generative_limits.md) documented that the
flow Mode B model could **not** aim a proposal at an off-centre gap (a wall blocking
the straight line with a slot ~2 m off-axis): trained on gap data it stayed
near-straight or veered to the wrong side, because a 16-dimensional CNN embedding
cannot localize a thin slot.

This transformer, trained on the **same** gap data, **aims its proposals at the
slot on both sides**. The difference is architectural: a strided conv tokenizes the
patch into spatial cells with learned positions, and K query tokens **cross-attend**
over those tokens, so the model localizes the slot and bends every candidate toward
it. Measured (held-out gap patches, exported ONNX): slot at +2.0 m → all candidates'
lateral offset at the wall ≈ +2.0 m; slot at −2.0 m → ≈ −2.0 m. The flow model on the
identical data does not (loss 0.12 vs 0.002; stays near-straight / wrong side).

**Honest scope — this is NOT a benchmark win.** Aiming a proposal at the slot is a
*representational* result, verified by the direct A/B probe and the C++ direction
test (`OnnxPathModelTest.CuratedZooTransformerAimsAtOffCentreSlot`). It does **not**
mean the full planner solves the gap: in the footprint-validated
`planner_benchmark`, the proposed path does **not** thread the narrow (1 m) slot
without clipping, so `DiffusionGlobalPlanner` reports *no path* on *off-centre gap*
— same as the flow model — and this `'both'`-trained model is currently **weaker than
the flow model on the side-obstacle scenario** too. The complete solution for the gap
remains the **hybrid** planner (generative propose → classical search dispose). What
this model adds is evidence that the *proposal* limitation is architectural, not
fundamental — a step toward a generative planner that could one day pass the
validated benchmark with a wider slot / larger capacity / footprint-aware training.

## Intended use

Research demonstration of architecture-dependent spatial routing in the *propose*
stage. Not a deployment model and not a drop-in upgrade over the flow Mode B model
for the benchmark; the deterministic costmap-validity layer gates every proposal and
the hybrid planner remains the completeness guarantee.

## Out-of-scope / limitations

- **Synthetic data only.** Never validated on a real robot or rosbag.
- **Proposal-level result only.** Aims at the slot in raw output; does **not** pass
  the footprint-validated `planner_benchmark` off-centre gap (no collision-free path
  through the 1 m slot), and underperforms the flow model on *side obstacle*.
- **Window-bounded.** Slot aiming demonstrated for offsets up to ~2 m inside the
  24×24 / 6×6 m goal-aligned patch; classical search / the hybrid planner own the
  general routing problem.
- **Research model.** Do not deploy on hardware; the validity layer is the authority.

## Training data

`make_costmap_path_dataset` (one-sided obstacle → bow to the free side) +
`make_costmap_path_gap_dataset` (wall with one off-centre slot → expert routes through
the slot via a Gaussian detour peaking at the slot offset where the path crosses the
wall), combined as the `'both'` dataset (192 samples). Mirrored +y/−y pairs and clear
samples keep the response symmetric. Fully procedural; no real data.

## Benchmark results

Research placeholder → checked **behaviourally** at the proposal level:

- **Off-centre-slot aiming (exported ONNX):** wall + slot at ±2 m → every candidate's
  lateral offset at the wall ≈ the slot offset. The flow Mode B model fails the same
  probe. Guarded by `OnnxPathModelTest.CuratedZooTransformerAimsAtOffCentreSlot`.
- **One-sided obstacle:** candidates veer to the free side (raw output).
- **Footprint-validated planner benchmark:** does **not** yield a valid path through
  the off-centre gap (the hybrid planner does); see
  [docs/planner_comparison.md](../../docs/planner_comparison.md) and
  [docs/generative_limits.md](../../docs/generative_limits.md).
- **Collisions:** the planner's deterministic costmap-validity layer gates every
  proposal regardless of model quality.

## Safety

- The model has **no** validity authority; the planner validates every proposal
  against the live costmap and falls back / reports no-path if none is valid.
- Not for hardware use; no ODD validation performed.

## Reproducibility

- **Command:** `PYTHONPATH=../../nav2_diffusion_training python3 export.py`
  (CUDA when available; `CUDA_VISIBLE_DEVICES= ` forces a deterministic CPU build)
- **Seed:** 0 · **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0; trained on CUDA,
  exported on CPU
- **Hyperparameters:** transformer, `'both'` dataset, 192 samples, 3000 epochs, lr 0.01
- Bit-for-bit reproduction may vary across torch versions / hardware; behaviour
  (slot aiming, side selection) is stable.

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated here) · Code: Apache-2.0
