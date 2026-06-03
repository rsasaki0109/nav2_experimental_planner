# nav2_diffusion_benchmarks

scenarios, metrics, reports。

**Status: 未実装（スケルトン）。**

MPPI / RPP / Smac / DWB と公正に比較できる再現可能な benchmark suite（[../docs/benchmarking.md](../docs/benchmarking.md)）。「評価可能な navigation framework」であるための中核（[../docs/architecture.md](../docs/architecture.md) §15.5）。

## 想定する内容

- Baselines（benchmarking §9.2、default / tuned / compute-matched を分離）
- Scenario Taxonomy（benchmarking §9.3）
- Metrics（benchmarking §9.4）
- Benchmark Harness（同一 map / robot / global planner で controller だけ差し替え）
- Report 生成（markdown / html）、CI regression threshold
