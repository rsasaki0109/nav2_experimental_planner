# nav2_rrt_planner

サンプリングベースの **classical global planner** 群（`nav2_core::GlobalPlanner`）。現在 2 種を提供:

- **`RRTStarPlanner`** — RRT\*（asymptotically optimal、最短経路寄り）
- **`RRTConnectPlanner`** — RRT-Connect（双方向・狭路で高速、feasible）

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

Nav2 公式の global planner は NavFn / Smac(A*, Hybrid-A*, State Lattice) / Theta\* と**すべてグリッド探索ベース**で、**サンプリングベースの planner が存在しない**。本パッケージはその空白を埋める classical なサンプリング planner を nav2_core プラグインとして提供する（`nav2_experimental_planner` = Nav2 に無い planner 群の一部）。

## RRT\* (`RRTStarPlanner`)

### アルゴリズム

`nav2_rrt_planner::RRTStarPlanner::createPlan()`:

1. **入力検証**: start/goal の frame が global frame と一致するか（不一致は `PlannerTFError`）、lethal/範囲外なら `StartOccupied` / `GoalOccupied`。
2. **木の成長（RRT\*）**: costmap 範囲内で点をサンプル（確率 `goal_bias` で goal を直接サンプル）→ 最近傍ノードから `step_size` だけ steer → 点とエッジを costmap で衝突判定 → `rewire_radius` 内で**最小コストの親**を選んで追加 → 近傍ノードを新ノード経由に**rewire**（コスト低減時）。
3. **終了/抽出**: goal 半径 `goal_tolerance` に到達したノードのうち最小コストのものを記録し、`max_iterations` まで改善。親を辿って start→goal の `nav_msgs::Path` を構築（各 pose は次点へ向けて orientation 設定）。
4. **fallback**: 経路が見つからなければ `NoValidPathCouldBeFound`。`cancel_checker` が true なら `PlannerCancelled`。

衝突判定はエッジを `interpolation_resolution` 間隔でサンプルし、各セルが LETHAL / INSCRIBED なら不可（unknown は `allow_unknown` 次第）。

グリッド近傍に縛られないため、**壁の隙間や非格子方向の通路**を通せるのが探索系との差別化点（統合テストで off-center の隙間通過を検証）。

### closed-loop 統合テスト

`test/test_rrt_star_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → start→goal の衝突なし経路
- 壁＋**中心から外れた隙間** → RRT が隙間へ迂回して通過（衝突なし）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

固定 `random_seed` で決定論的に動作するためテスト再現性がある。

### パラメータ

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

## RRT-Connect (`RRTConnectPlanner`)

### アルゴリズム

`nav2_rrt_planner::RRTConnectPlanner::createPlan()` は**双方向**サンプリング:

1. **入力検証**: RRT\* と同じ（frame 不一致 → `PlannerTFError`、start/goal が lethal/範囲外 → `StartOccupied` / `GoalOccupied`）。
2. **2 本の木**: start 根の木と goal 根の木を同時に育てる。各反復でランダム点をサンプル → 片方の木を `step_size` だけ **EXTEND**（最近傍から steer + 衝突判定）→ もう片方の木をその新ノードへ **CONNECT**（到達 or 詰まりまで貪欲に EXTEND を反復）→ 反復ごとに 2 本の役割を **swap**。
3. **接続/抽出**: CONNECT が新ノードに到達したら 2 本の木が繋がった瞬間。両木の枝を根まで辿り、start→goal 順に連結して `nav_msgs::Path` を構築。
4. **fallback**: `max_iterations` 以内に繋がらなければ `NoValidPathCouldBeFound`、`cancel_checker` true で `PlannerCancelled`。

貪欲な CONNECT ステップにより、**狭路（narrow passage）を RRT/RRT\* より遥かに少ない反復で貫通**できるのが利点。ただし最短性は保証しない（feasible であって optimal ではない）。最短経路が要るなら `RRTStarPlanner` を使う。

### closed-loop 統合テスト

`test/test_rrt_connect_planner.cpp`（RRT\* と同構成、GPU/シム不要）: クリア路の経路 / off-center の隙間貫通 / 隙間なし壁 → `NoValidPathCouldBeFound` / goal lethal → `GoalOccupied` / cancel → `PlannerCancelled`。固定 `random_seed` で決定論的。

### パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `max_iterations` | 4000 | 反復回数（上限） |
| `step_size` | 0.5 | steer の最大エッジ長 [m] |
| `interpolation_resolution` | 0.05 | エッジ衝突判定のサンプル間隔 [m] |
| `allow_unknown` | true | costmap の unknown を通行可能とみなすか |
| `random_seed` | 1 | サンプラの seed（決定論性のため固定） |

RRT-Connect は両側から goal/start に向かって育つため `goal_bias` / `goal_tolerance` / `rewire_radius` は持たない。

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

RRT-Connect に切り替えるには `plugin` を `nav2_rrt_planner::RRTConnectPlanner` にし、RRT\* 固有パラメータ（`goal_bias` / `goal_tolerance` / `rewire_radius`）を外すだけ。

最小例: [../nav2_diffusion_bringup/params/rrt_planner_example.yaml](../nav2_diffusion_bringup/params/rrt_planner_example.yaml)（RRT\* と RRT-Connect の両方を記載）。

## 関連

- 生成型 GlobalPlanner（Mode B）: [../nav2_diffusion_global_planner/README.md](../nav2_diffusion_global_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
