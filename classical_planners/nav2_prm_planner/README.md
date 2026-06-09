# nav2_prm_planner

確率ロードマップ **PRM global planner**（`nav2_core::GlobalPlanner`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

Nav2 公式の global planner は NavFn / Smac(A*, Hybrid-A*, State Lattice) / Theta\* と**すべてグリッド探索ベース**で、**サンプリングベースのロードマップ planner が存在しない**。本パッケージはその空白を埋める classical な PRM（Probabilistic Roadmap）を nav2_core プラグインとして提供する（`Nav2PlannerBattle` = Nav2 に無い planner 群の一部）。サンプリング系の兄弟として [nav2_rrt_planner](../nav2_rrt_planner/README.md)（RRT\* / RRT-Connect）がある。

## アルゴリズム

`nav2_prm_planner::PRMPlanner::createPlan()`:

1. **入力検証**: start/goal の frame が global frame と一致するか（不一致は `PlannerTFError`）、lethal/範囲外なら `StartOccupied` / `GoalOccupied`。
2. **マイルストーン生成**: costmap 範囲内で点をサンプルし、衝突しない点だけを採用（`num_samples` 個まで）。index 0 を start、1 を goal とし、その後にサンプルを並べる。
3. **ロードマップ配線**: 各マイルストーンを `connection_radius` 内の近傍（最大 `max_neighbours` 個）へ、エッジが衝突しなければ無向辺で接続してグラフを作る。
4. **最短路探索**: start(0)→goal(1) を **Dijkstra** で探索し、`nav_msgs::Path` を構築（各 pose は次点へ向けて orientation 設定）。
5. **fallback**: roadmap 上で start と goal が連結しなければ `NoValidPathCouldBeFound`。`cancel_checker` が true なら `PlannerCancelled`。

衝突判定はエッジを `interpolation_resolution` 間隔でサンプルし、各セルが LETHAL / INSCRIBED なら不可（unknown は `allow_unknown` 次第）。

RRT/RRT-Connect が単一クエリへ木を伸ばすのに対し、PRM は自由空間の**再利用可能なグラフ**を作るのが本来の特徴（multi-query）。ここでは plan ごとに costmap が変わるため roadmap は毎回構築する。

## closed-loop 統合テスト

`test/test_prm_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → start→goal の衝突なし経路
- 壁＋**中心から外れた隙間** → サンプルされた roadmap が隙間を通って連結（衝突なし）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

固定 `random_seed` で決定論的に動作するためテスト再現性がある。

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `num_samples` | 500 | 衝突しないマイルストーンのサンプル数 |
| `connection_radius` | 1.5 | グラフ配線時の最大エッジ長 [m] |
| `max_neighbours` | 10 | 1 マイルストーンあたりの最大辺数（k 近傍上限） |
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
      plugin: "nav2_prm_planner::PRMPlanner"
      num_samples: 500
      connection_radius: 1.5
      max_neighbours: 10
```

最小例: [../nav2_diffusion_bringup/params/prm_planner_example.yaml](../nav2_diffusion_bringup/params/prm_planner_example.yaml)。

## 関連

- サンプリング系の兄弟（RRT\* / RRT-Connect）: [../nav2_rrt_planner/README.md](../nav2_rrt_planner/README.md)
- 生成型 GlobalPlanner（Mode B）: [../nav2_diffusion_global_planner/README.md](../nav2_diffusion_global_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
