# nav2_diffusion_benchmarks

scenarios, metrics, reports。

**Status: metrics ライブラリあり（ビルド & テスト通過）。scenario/harness は未実装。**

MPPI / RPP / Smac / DWB と公正に比較できる再現可能な benchmark suite（[../docs/benchmarking.md](../docs/benchmarking.md)）。「評価可能な navigation framework」であるための中核（[../docs/architecture.md](../docs/architecture.md) §15.5）。

## 現状の実装

- `nav2_diffusion_benchmarks/metrics.hpp`: 実行済み軌道（time-indexed SE(2) path）+ goal から §9.4 の geometry 系 metrics を算出する `evaluateRun()` / `RunMetrics`
  - `reached_goal` / `goal_distance` / `time_to_goal` / `path_length` / `detour_ratio` / `total_turning`
  - costmap 非依存（GPU/シム不要でユニットテスト可能）。clearance/collision/social は costmap・障害物ログがある場合に別途追加予定。
- gtest（`test/test_metrics.cpp`）

この metrics は、同一 scenario で controller を差し替えて実行した結果（executed path）を入力に、MPPI/RPP/Smac と横並び比較するための土台。

## 想定する内容

- Baselines（benchmarking §9.2、default / tuned / compute-matched を分離）
- Scenario Taxonomy（benchmarking §9.3）
- Metrics（benchmarking §9.4）
- Benchmark Harness（同一 map / robot / global planner で controller だけ差し替え）
- Report 生成（markdown / html）、CI regression threshold
