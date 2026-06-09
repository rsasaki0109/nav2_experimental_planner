#!/usr/bin/env bash
# Run battle_trace with a custom ONNX and merge the fighter into battle_data.json/js.
# Requires a sourced ROS 2 workspace with nav2_planner_benchmarks built.
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/tools/nav2_planner_battle"
MODE=""
LABEL=""
ONNX=""
FAMILY=""
MIN_TURN=""
FULL=0

usage() {
  cat <<'EOF'
Battle your ONNX model against the default Nav2 Planner Battle roster.

Usage:
  tools/battle_custom_model.sh --mode A|B --label NAME --onnx PATH [options]

Options:
  --mode A|B           Mode A controller race or Mode B global planner duel
  --label NAME         Fighter name shown in the browser UI
  --onnx PATH          Absolute path to your ONNX (must match repo I/O contract)
  --family TEXT        Optional family tag (default: custom (local|global))
  --min-turn-radius R  Mode B only — kinematics context slot (default: 0 omni)
  --full               Append custom fighter to a full battle_trace export (slow)
  --help               Show this help

Default (fast): runs only your model (--custom-only), then merges into the
committed battle_data.json so you can open index.html locally and compare.

Examples:
  tools/battle_custom_model.sh --mode A --label my-flow --onnx ~/exports/local.onnx
  tools/battle_custom_model.sh --mode B --label my-planner --onnx ~/exports/global.onnx --min-turn-radius 0.3

See docs/custom_model_battle.md for ONNX contracts and troubleshooting.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="${2^^}"; shift 2 ;;
    --label) LABEL="$2"; shift 2 ;;
    --onnx) ONNX="$2"; shift 2 ;;
    --family) FAMILY="$2"; shift 2 ;;
    --min-turn-radius) MIN_TURN="$2"; shift 2 ;;
    --full) FULL=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$MODE" || -z "$LABEL" || -z "$ONNX" ]]; then
  echo "error: --mode, --label, and --onnx are required" >&2
  usage
  exit 2
fi
if [[ "$MODE" != "A" && "$MODE" != "B" ]]; then
  echo "error: --mode must be A or B" >&2
  exit 2
fi
if [[ ! -f "$ONNX" ]]; then
  echo "error: ONNX not found: $ONNX" >&2
  exit 2
fi

set +u
if [[ -f "$ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/install/setup.bash"
else
  # shellcheck disable=SC1091
  source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
fi
set -u

ARGS=(--mode "$MODE")
if [[ "$FULL" -eq 0 ]]; then
  ARGS+=(--custom-only)
fi
if [[ "$MODE" == "A" ]]; then
  ARGS+=(--custom-controller "$LABEL" "$ONNX")
  [[ -n "$FAMILY" ]] && ARGS+=("$FAMILY")
else
  ARGS+=(--custom-planner "$LABEL" "$ONNX")
  [[ -n "$MIN_TURN" ]] && ARGS+=("$MIN_TURN")
  [[ -n "$FAMILY" ]] && echo "note: --family is ignored for Mode B (uses custom (global))" >&2
fi

TMP="$(mktemp /tmp/battle_custom.XXXXXX.json)"
trap 'rm -f "$TMP"' EXIT

echo "running battle_trace ${ARGS[*]}" >&2
ros2 run nav2_planner_benchmarks battle_trace "${ARGS[@]}" > "$TMP"

if [[ "$FULL" -eq 1 ]]; then
  cp "$TMP" "$OUT/battle_data.json"
  printf 'window.BATTLE_DATA = ' > "$OUT/battle_data.js"
  cat "$OUT/battle_data.json" >> "$OUT/battle_data.js"
  printf ';\n' >> "$OUT/battle_data.js"
  echo "wrote full export to $OUT/battle_data.json (custom fighter included)" >&2
else
  python3 "$ROOT/tools/merge_battle_fighter.py" \
    "$OUT/battle_data.json" "$TMP" "$OUT/battle_data.json"
fi

echo "open tools/nav2_planner_battle/index.html and pick a scenario to watch \"$LABEL\" fight." >&2
