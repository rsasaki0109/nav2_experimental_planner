# nav2_rrt_planner

サンプリングベースの **RRT\* global planner**（`nav2_core::GlobalPlanner`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

Nav2 公式の global planner は NavFn / Smac(A*, Hybrid-A*, State Lattice) / Theta\* と**すべてグリッド探索ベース**で、**サンプリングベースの planner が存在しない**。本パッケージはその空白を埋める classical な RRT\*（asymptotically optimal）を nav2_core プラグインとして提供する（`nav2_experimental_planner` = Nav2 に無い planner 群の一部）。

## アルゴリズム

`nav2_rrt_planner::RRTStarPlanner::createPlan()`:

1. **入力検証**: start/goal の frame が global frame と一致するか（不一致は `PlannerTFError`）、lethal/範囲外なら `StartOccupied` / `GoalOccupied`。
2. **木の成長（RRT\*）**: costmap 範囲内で点をサンプル（確率 `goal_bias` で goal を直接サンプル）→ 最近傍ノードから `step_size` だけ steer → 点とエッジを costmap で衝突判定 → `rewire_radius` 内で**最小コストの親**を選んで追加 → 近傍ノードを新ノード経由に**rewire**（コスト低減時）。
3. **終了/抽出**: goal 半径 `goal_tolerance` に到達したノードのうち最小コストのものを記録し、`max_iterations` まで改善。親を辿って start→goal の `nav_msgs::Path` を構築（各 pose は次点へ向けて orientation 設定）。
4. **fallback**: 経路が見つからなければ `NoValidPathCouldBeFound`。`cancel_checker` が true なら `PlannerCancelled`。

衝突判定はエッジを `interpolation_resolution` 間隔でサンプルし、各セルが LETHAL / INSCRIBED なら不可（unknown は `allow_unknown` 次第）。

グリッド近傍に縛られないため、**壁の隙間や非格子方向の通路**を通せるのが探索系との差別化点（統合テストで off-center の隙間通過を検証）。

## closed-loop 統合テスト

`test/test_rrt_star_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → start→goal の衝突なし経路
- 壁＋**中心から外れた隙間** → RRT が隙間へ迂回して通過（衝突なし）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

固定 `random_seed` で決定論的に動作するためテスト再現性がある。

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `max_iterations` | 4000 | RRT\* の反復回数（上限） |
| `step_size` | 0.5 | steer の最大エッジ長 [m] |
| `goal_bias` | 0.10 | goal を直接サンプルする確率 |
| `goal_tolerance` | 0.25 | goal 到達半径 [m] |
| `rewire_radius` | 1.0 | RRT\* の近傍 rewire 半径 [m] |
| `interpolation_resolution` | 0.05 | エッジ衝突判定のサンプル間隔 [m] |
| `allow_unknown` | true | costmap の unknown を通行可能とみなすか |
| `random_seed` | 1 | サンプラの seed（決定論性のため固定） |

## 使い方（例）

planner_server の `GridBased` plugin を差し替える:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_rrt_planner::RRTStarPlanner"
      max_iterations: 4000
      step_size: 0.5
      goal_bias: 0.1
      goal_tolerance: 0.25
      rewire_radius: 1.0
```

最小例: [../nav2_diffusion_bringup/params/rrt_planner_example.yaml](../nav2_diffusion_bringup/params/rrt_planner_example.yaml)。

## 関連

- 生成型 GlobalPlanner（Mode B）: [../nav2_diffusion_global_planner/README.md](../nav2_diffusion_global_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
