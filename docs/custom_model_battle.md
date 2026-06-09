# Battle your ONNX model

Run your trained Nav2 generative model through the **same** `battle_trace` harness as the
default roster, then replay it in the browser Nav2 Planner Battle UI — no scripted winners.

## Quick start

Build the workspace, export your ONNX, then:

```bash
# Mode A — local controller arena race (unicycle closed loop)
tools/battle_custom_model.sh \
  --mode A --label my-flow --onnx /path/to/local.onnx

# Mode B — global planner path duel
tools/battle_custom_model.sh \
  --mode B --label my-planner --onnx /path/to/global.onnx --min-turn-radius 0
```

Open `tools/nav2_planner_battle/index.html` (or serve the folder locally) and play any
scenario. Your fighter appears alongside VFH+, RRT*, Diffusion, etc.

The default script path is **fast**: it runs only your model (`--custom-only`), then merges
the traces into the committed `battle_data.json` / `battle_data.js`. Re-run with the same
`--label` to refresh your traces.

For a full re-export of every default fighter **plus** yours (slow, ~minutes):

```bash
tools/battle_custom_model.sh --full --mode A --label my-flow --onnx /path/to/local.onnx
```

## ONNX contracts

Your artifact must match the repo’s existing I/O seams:

| Mode | Plugin | ONNX backend | Model cards |
|---|---|---|---|
| **A** | `nav2_diffusion_controller::DiffusionController` | `OnnxTrajectoryModel` | [diffusion_local](../model_zoo/diffusion_local/model_card.md) · [transformer](../model_zoo/diffusion_local_transformer/model_card.md) |
| **B** | `nav2_diffusion_global_planner::DiffusionGlobalPlanner` | `OnnxPathModel` | [diffusion_global](../model_zoo/diffusion_global/model_card.md) · [kinematics](../model_zoo/diffusion_global_kinematics/model_card.md) |

**Mode A inputs:** egocentric 32×32 costmap patch + carrot context + speed limits → multimodal
trajectory candidates (see `export.py` in the model zoo entry you trained from).

**Mode B inputs:** costmap patch + start/goal context (+ optional `min_turn_radius` in the
second context slot for kinematics-conditioned models).

If loading fails, check `ros2 run nav2_planner_benchmarks battle_trace --help` and run the
matching `nav2_diffusion_onnx` tests locally.

## Manual / CI-safe workflow

CI compares the **committed** golden `battle_data.json`. Custom fighters are for local
experiments unless you intentionally refresh and commit the merged JSON.

```bash
# 1) Export only your fighter (Mode A example)
ros2 run nav2_planner_benchmarks battle_trace \
  --mode A --custom-only \
  --custom-controller "my-flow" /path/to/local.onnx "custom flow" \
  > /tmp/custom.json

# 2) Merge into the golden battle file + regenerate battle_data.js
python3 tools/merge_battle_fighter.py \
  tools/nav2_planner_battle/battle_data.json /tmp/custom.json \
  tools/nav2_planner_battle/battle_data.json
```

`battle_trace` flags:

| Flag | Purpose |
|---|---|
| `--custom-controller LABEL ONNX [FAMILY]` | Append a Mode A DiffusionController |
| `--custom-planner LABEL ONNX [MIN_TURN_RADIUS]` | Append a Mode B DiffusionGlobalPlanner |
| `--custom-only` | Skip the default roster (fast iteration) |
| `--mode A\|B\|both` | Limit JSON export to one battle mode |

## What you get

- Same scenarios, metrics, and HUD as the public game
- Honest outcomes: `reached` / `collision` / `timeout` (Mode A) or valid path length (Mode B)
- Championship points if you merged into the full roster and re-open **🏆 Championship**

## Related

- [Nav2 Planner Battle README](../tools/nav2_planner_battle/README.md)
- [Controller comparison](controller_comparison.md) · [Planner comparison](planner_comparison.md)
- [Training](training.md) · [Model zoo](model_zoo.md)
