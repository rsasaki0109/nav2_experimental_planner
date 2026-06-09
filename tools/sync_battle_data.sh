#!/usr/bin/env bash
# Regenerate battle_data.json/js from battle_trace (requires sourced ROS workspace).
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/tools/nav2_planner_battle"
set +u
if [[ -f "$ROOT/install/setup.bash" ]]; then
  source "$ROOT/install/setup.bash"
else
  source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
fi
ros2 run nav2_planner_benchmarks battle_trace > "$OUT/battle_data.json"
printf 'window.BATTLE_DATA = ' > "$OUT/battle_data.js"
cat "$OUT/battle_data.json" >> "$OUT/battle_data.js"
printf ';\n' >> "$OUT/battle_data.js"
echo "wrote $OUT/battle_data.json and battle_data.js"
