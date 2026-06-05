# nav2_diffusion_benchmarks

scenarios, metrics, reports。

**Status: metrics ライブラリあり（ビルド & テスト通過）。scenario/harness は未実装。**

MPPI / RPP / Smac / DWB と公正に比較できる再現可能な benchmark suite（[../docs/benchmarking.md](../docs/benchmarking.md)）。「評価可能な navigation framework」であるための中核（[../docs/architecture.md](../docs/architecture.md) §15.5）。

## 現状の実装

- `nav2_diffusion_benchmarks/metrics.hpp`: 実行済み軌道（time-indexed SE(2) path）+ goal から §9.4 の geometry 系 metrics を算出する `evaluateRun()` / `RunMetrics`
  - Task系: `reached_goal` / `goal_distance` / `time_to_goal` / `path_length` / `detour_ratio`
  - Smoothness/Efficiency系: `total_turning` / `oscillation_count`（旋回方向の反転）/ `direction_changes`（前後反転=cusp）/ `stop_duration`
  - costmap 非依存（GPU/シム不要でユニットテスト可能）
- `nav2_diffusion_benchmarks/collision_metrics.hpp`: costmap ベースの safety 系 metrics（§9.4 Safety）`evaluateCollisions()` / `CollisionMetrics`
  - `collision_count` / `collided`（footprint が障害物に当たった path pose 数）
  - `min_clearance`（robot 中心から最近傍 lethal セルまでの距離 [m]、探索半径で saturate）
- `nav2_diffusion_benchmarks/run_result.hpp`: `RunResult`（scenario/controller + metrics）
- `nav2_diffusion_benchmarks/scores.hpp`: **safety 最重視の複合スコア**（§9.6）`computeScores()` / `Scores` / `ScoreWeights`
  - `safety`（衝突で 0、それ以外は clearance/参照値）/ `progress`（未到達で 0、それ以外は 1/detour）/ `comfort`（1/(1+turning)）/ `overall`（重み正規化和、既定 safety 0.5 / progress 0.3 / comfort 0.2）
- `nav2_diffusion_benchmarks/report.hpp`: `toMarkdownTable()`（生 metrics 比較表、§9.5）と `toMarkdownLeaderboard()`（overall 降順の leaderboard、§9.6）
- gtest（`test/test_metrics.cpp`, `test/test_collision_metrics.cpp`, `test/test_scores.cpp`, `test/test_report.cpp`）

### レポート出力例

```
| Scenario | Controller | Reached | Time [s] | Path [m] | Detour | Collisions | Min clear [m] | Turning [rad] | Osc | Dir chg | Stop [s] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| narrow_doorway | DiffusionController | yes | 12.50 | 8.00 | 1.10 | 0 | 0.35 | 0.40 | 0 | 0 | 0.00 |
| narrow_doorway | MPPI | no | 0.00 | 0.00 | 1.00 | 2 | 2.00 | 0.00 | 0 | 0 | 0.00 |
```

- `nav2_diffusion_benchmarks/run_recorder.hpp`: `RunRecorder`。pose ストリーム（`/odom` や rosbag）を executed path に蓄積し、goal を与えて `RunResult` を生成（ROS 非依存・テスト可能）。
- `benchmark_runner` ノード（`src/benchmark_runner_node.cpp`）: `odom` を購読して走行を記録し、`~/finish`（`std_srvs/srv/Trigger`）呼び出しで metrics を算出して markdown レポートを log / file 出力する薄い ROS グルー。

```bash
ros2 run nav2_diffusion_benchmarks benchmark_runner \
  --ros-args -p scenario:=narrow_doorway -p controller:=DiffusionController \
             -p goal_x:=2.0 -p goal_y:=0.0 -p output_file:=/tmp/report.md
# 走行後に:
ros2 service call /benchmark_runner/finish std_srvs/srv/Trigger
```

- gtest（`test/test_metrics.cpp`, `test/test_collision_metrics.cpp`, `test/test_scores.cpp`, `test/test_report.cpp`, `test/test_run_recorder.cpp`）

- `nav2_diffusion_benchmarks/scenario.hpp`: 再現可能な scenario 定義（§9.3 / §10.3）。`Scenario`（name/map/robot/start/goal/goal_tolerance/seed）と `parseScenario()`（YAML 文字列）/ `loadScenarioFile()`。`scenarios/` に golden scenario の例（simple_corridor, narrow_doorway）。
- gtest（`test/test_metrics.cpp`, `test/test_collision_metrics.cpp`, `test/test_scores.cpp`, `test/test_report.cpp`, `test/test_run_recorder.cpp`, `test/test_scenario.cpp`）

- `nav2_diffusion_benchmarks/aggregate.hpp`: scenario × controller の複数 run を **controller ごとに集約**（`summarizeByController`）し、success rate / mean overall / mean safety のランキングを markdown 出力（`toMarkdownSummary`、§9.5/§9.6）。

social 系 metrics（personal-space 等）は人トラッキングのログがある場合に別途追加予定。

この metrics は、同一 scenario で controller を差し替えて実行した結果（executed path）を入力に、MPPI/RPP/Smac と横並び比較するための土台。

## 想定する内容

- Baselines（benchmarking §9.2、default / tuned / compute-matched を分離）
- Scenario Taxonomy（benchmarking §9.3）
- Metrics（benchmarking §9.4）
- Benchmark Harness（同一 map / robot / global planner で controller だけ差し替え）
- Report 生成（markdown / html）、CI regression threshold
