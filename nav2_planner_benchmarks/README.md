# nav2_planner_benchmarks

本リポジトリの **classical GlobalPlanner 群を同一シナリオで比較**するオフラインベンチマーク。

稼働中の `nav2_costmap_2d::Costmap2DROS`（6×6 m, 0.05 m, GPU/シム不要）上で、各 planner を **pluginlib でクラス名ロード**し、3 シナリオ（clear / off-centre gap / slalom）で **成功可否・経路長・pose 数・計画時間（中央値）** を計測して Markdown 表を出力する。

## 実行

```bash
ros2 run nav2_planner_benchmarks planner_benchmark > docs/planner_comparison.md
```

結果は [../docs/planner_comparison.md](../docs/planner_comparison.md)（コミット済み・再現可能）。

reactive Controller（VFH+ / ND）の閉ループ比較も同梱:

```bash
ros2 run nav2_planner_benchmarks controller_benchmark > docs/controller_comparison.md
```

unicycle モデルで costmap 上をロールアウトし、到達可否・経路長・最小クリアランス・操舵の滑らかさ・回廊中央寄せを計測する。結果は [../docs/controller_comparison.md](../docs/controller_comparison.md)。

## 比較対象（すべて Nav2 公式に無い `nav2_core::GlobalPlanner`）

| planner | パラダイム | パッケージ |
|---|---|---|
| RRT\* | sampling（漸近最適） | [nav2_rrt_planner](../nav2_rrt_planner) |
| RRT-Connect | sampling（双方向） | [nav2_rrt_planner](../nav2_rrt_planner) |
| PRM | sampling（ロードマップ） | [nav2_prm_planner](../nav2_prm_planner) |
| D\* Lite | incremental search | [nav2_dstar_lite_planner](../nav2_dstar_lite_planner) |
| JPS | grid A\* 高速化 | [nav2_jps_planner](../nav2_jps_planner) |
| Lazy Theta\* | any-angle | [nav2_lazy_theta_star_planner](../nav2_lazy_theta_star_planner) |
| ARA\* | anytime | [nav2_ara_star_planner](../nav2_ara_star_planner) |
| visibility graph | geometric | [nav2_visibility_graph_planner](../nav2_visibility_graph_planner) |

## 注意

- 時間は dev マシンでの中央値（21 回）で**目安**。絶対値は負荷で変わるため、相対的な大小と経路長・形状の列で比較する。
- **D\* Lite** は探索状態を呼び出し間でキャッシュするため、中央値は warm な incremental 再計画を反映する（初回 cold はより遅い）。他は毎回ゼロから再計画する。
- 経路長が短い＝常に良い、ではない（sampling は最適性を速度と、ARA\* は anytime 境界と引き換える）。pose 数は密化粒度を反映。
