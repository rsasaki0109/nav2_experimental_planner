# nav2_planner_benchmarks

本リポジトリの **classical GlobalPlanner 群を同一シナリオで比較**するオフラインベンチマーク。

稼働中の `nav2_costmap_2d::Costmap2DROS`（6×6 m, 0.05 m, GPU/シム不要）上で、各 planner を **pluginlib でクラス名ロード**し、3 シナリオ（clear / off-centre gap / slalom）で **成功可否・経路長・pose 数・計画時間（中央値）** を計測して Markdown 表を出力する。

## 実行

```bash
ros2 run nav2_planner_benchmarks planner_benchmark > docs/planner_comparison.md
```

結果は [../docs/planner_comparison.md](../docs/planner_comparison.md)（コミット済み・再現可能）。

局所 Controller（reactive **VFH+ / ND** + 生成型 **Mode A learned / hybrid**）の閉ループ比較も同梱:

```bash
ros2 run nav2_planner_benchmarks controller_benchmark > docs/controller_comparison.md
```

unicycle モデルで costmap 上をロールアウトし、到達可否・経路長・最小クリアランス・操舵の滑らかさ・回廊中央寄せを計測する。結果は [../docs/controller_comparison.md](../docs/controller_comparison.md)。**learned** は open で goal 到達するが障害物では安全停止、**hybrid**（learned + VFH+ fallback）は全シナリオ到達 — Mode B planner の hybrid と対称（[../docs/generative_limits.md](../docs/generative_limits.md)）。

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

加えて **生成型 Mode B**（[nav2_diffusion_global_planner](../nav2_diffusion_global_planner)）を **2 変種**並べる:
- **analytic** — 解析的 `FanPathModel`（ONNX 不要・決定論的・対称 bow fan）
- **learned** — [model_zoo](../model_zoo/diffusion_global) の costmap 条件付き flow モデルを `OnnxPathModel` 経由で実際に推論（リポジトリ初の「ループに入った学習済みモデル」）
- **hybrid** — learned 提案 + classical（JPS）fallback。無効候補時に完全な探索へ委譲し、**全シナリオを解く**（off-centre gap / slalom は fallback、clear / side obstacle は generative）
- **guided** — 密結合 hybrid（`hybrid_mode: guided`）。常に組み込みの完全な A* を走らせ、有効 proposal 近傍のコストを割引いて learned が毎回の経路を誘導。全シナリオ解決（cell 解像度の pose 数）

「モデルが候補を *propose* → 決定論的安全層が *dispose*」を同一土俵で示す。learned が活きる **side obstacle** シナリオも追加した。hybrid は学習単体の天井（gap-routing）を実際に超える（[../docs/generative_limits.md](../docs/generative_limits.md)）。

## 注意

- 時間は dev マシンでの中央値（21 回）で**目安**。絶対値は負荷で変わるため、相対的な大小と経路長・形状の列で比較する。
- **D\* Lite** は探索状態を呼び出し間でキャッシュするため、中央値は warm な incremental 再計画を反映する（初回 cold はより遅い）。他は毎回ゼロから再計画する。
- 経路長が短い＝常に良い、ではない（sampling は最適性を速度と、ARA\* は anytime 境界と引き換える）。pose 数は密化粒度を反映。
- **生成型 Mode B** は探索系と違い**完全ではない**。analytic fan は clear / off-centre gap / side obstacle を通すが slalom（S 字）は表現できず **no path**。learned は costmap を読んで空き側へ寄せるので clear / side obstacle を通すが、合成学習分布が横方向の迂回量を制限するため off-centre gap（2 m 迂回）と slalom は **no path**。いずれも安全層が全候補を棄却して安全側に fail-closed する。**天井はアーキテクチャでなく学習データ**で、より豊かなデータが上げる（[../model_zoo/diffusion_global/model_card.md](../model_zoo/diffusion_global/model_card.md)）。これが「propose / dispose」分業の意図した挙動。
