# nav2_visibility_graph_planner

**Visibility-graph global planner**（`nav2_core::GlobalPlanner`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

Nav2 公式のグリッド planner（NavFn / Smac / Theta\*）や本リポジトリのサンプリング planner と違い、visibility graph は**連続空間で障害物の幾何**を扱う。障害物を避ける最短経路が曲がれる場所は**障害物の凸コーナーだけ**なので、グラフの頂点をそれらのコーナー（＋ start / goal）、辺を「互いに見える頂点対」とし、最短路探索（A\*）で**直線をつないだコーナー沿いの経路**を得る。本実装ではコーナーを costmap グリッドから抽出（障害物の凸コーナーに接する free セル）し、可視性は line-of-sight 判定なので、幾何 visibility graph のグリッド近似となる。

Nav2 公式に visibility-graph planner は無い（`Nav2PlannerBattle` = Nav2 に無い planner 群の一部）。他の classical 兄弟: サンプリング [nav2_rrt_planner](../nav2_rrt_planner/README.md)・[nav2_prm_planner](../nav2_prm_planner/README.md)、インクリメンタル [nav2_dstar_lite_planner](../nav2_dstar_lite_planner/README.md)、グリッド高速化 [nav2_jps_planner](../nav2_jps_planner/README.md)、any-angle [nav2_lazy_theta_star_planner](../nav2_lazy_theta_star_planner/README.md)、anytime [nav2_ara_star_planner](../nav2_ara_star_planner/README.md)。

## アルゴリズム

`nav2_visibility_graph_planner::VisibilityGraphPlanner`:

1. **入力検証**: frame 不一致 → `PlannerTFError`、start/goal が範囲外/障害物 → `StartOccupied` / `GoalOccupied`。
2. **コーナー抽出**: 各 free セルについて、斜め隣接が障害物で、その斜めを挟む 2 つの直交セルが free なら「凸コーナー」頂点として採用（`max_corners` で上限。超えたら警告して打ち切り）。
3. **可視グラフ構築**: 全頂点対のうち LOS が通る対を無向辺で結ぶ（重み = ワールド Euclidean 距離）。
4. **A\***: Euclidean ヒューリスティック（admissible）で start→goal を探索。
5. **再構築 + 密化**: 頂点列の各（LOS 済み）直線区間を `interpolation_resolution` 間隔で密化（端点は実 start/goal にスナップ）。連結しなければ `NoValidPathCouldBeFound`、`cancel_checker` true で `PlannerCancelled`。

LOS は密化と同一のワールド標本化で判定するため、グラフが受理した区間は密化後も衝突しない。**均一コスト前提**のため costmap を二値化（`lethal_threshold` 以上を障害物）。完全に決定論的。

> 注: 全頂点対の LOS 判定は O(V²) なので、障害物コーナーが非常に多い地図では `max_corners` で頂点数を制限する（打ち切り時は警告を出し経路が最適でない可能性がある）。狭所・少数障害物の局所/大域 costmap に向く。

## closed-loop 統合テスト

`test/test_visibility_graph_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → 軸に乗らない goal へ**単一の直線**（start→goal の可視辺。直線からの逸脱 ~0）
- 壁＋**中心から外れた隙間** → コーナー経由で隙間を通過（衝突なし）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `lethal_threshold` | 253 | このコスト以上のセルを障害物とみなす |
| `allow_unknown` | true | costmap の unknown を通行可能とみなすか |
| `interpolation_resolution` | 0.05 | 経路密化 & LOS 標本化の間隔 [m] |
| `max_corners` | 1500 | グラフ頂点（コーナー）数の上限（O(V²) 抑制） |

## 使い方（例）

planner_server の `GridBased` plugin を差し替える:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_visibility_graph_planner::VisibilityGraphPlanner"
      lethal_threshold: 253
      allow_unknown: true
      max_corners: 1500
```

最小例: [../nav2_diffusion_bringup/params/visibility_graph_planner_example.yaml](../nav2_diffusion_bringup/params/visibility_graph_planner_example.yaml)。

## 関連

- サンプリング系: [../nav2_rrt_planner/README.md](../nav2_rrt_planner/README.md), [../nav2_prm_planner/README.md](../nav2_prm_planner/README.md)
- インクリメンタル系: [../nav2_dstar_lite_planner/README.md](../nav2_dstar_lite_planner/README.md)
- グリッド高速化: [../nav2_jps_planner/README.md](../nav2_jps_planner/README.md)
- any-angle: [../nav2_lazy_theta_star_planner/README.md](../nav2_lazy_theta_star_planner/README.md)
- anytime: [../nav2_ara_star_planner/README.md](../nav2_ara_star_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
