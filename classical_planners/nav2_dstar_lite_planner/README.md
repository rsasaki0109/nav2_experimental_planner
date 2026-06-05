# nav2_dstar_lite_planner

インクリメンタル探索 **D\* Lite global planner**（`nav2_core::GlobalPlanner`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

Nav2 公式の global planner（NavFn / Smac / Theta\*）は**毎サイクル一から再計画する one-shot 探索**で、**インクリメンタル planner が存在しない**。本パッケージはその空白を埋める classical な **D\* Lite**（Koenig & Likhachev, 2002）を nav2_core プラグインとして提供する（`nav2_experimental_planner` = Nav2 に無い planner 群の一部）。サンプリング系の兄弟に [nav2_rrt_planner](../nav2_rrt_planner/README.md)（RRT\* / RRT-Connect）、[nav2_prm_planner](../nav2_prm_planner/README.md)（PRM）がある。

## アルゴリズム

`nav2_dstar_lite_planner::DStarLitePlanner` は 8 近傍の costmap グリッドを**goal から逆向き**に探索し、各セルの `g`（確定コスト）/ `rhs`（一手先読みコスト）と優先度キューを **createPlan の呼び出しを跨いで保持**する。

- **初回 / goal 変更時（cold start）**: `g`/`rhs` を初期化し goal を seed して `ComputeShortestPath`。
- **再計画時（incremental repair）**: グリッド形状と goal が同じなら状態を再利用。ロボット移動分だけ優先度キーをずらす `km += h(s_last, s_start)` を加え、**前回からコストが変わったセル（とその 8 近傍）だけ** `UpdateVertex` で修復してから `ComputeShortestPath`。変化が小さいほど更新が少なく、一から再計画するより遥かに安い。

変化検出は前回 plan 時の costmap スナップショットと現在の差分。新たに見えた障害物・動的エージェントが少数セルだけ動く現実的な状況で効く。

エッジコストは隣接 2 セルの正規化コスト平均に `cost_weight` を掛けて高コストセルを避ける（対角は √2 倍）。ヒューリスティックは Euclidean（`heuristic_weight` ≤ 1 で admissible 維持）。LETHAL / INSCRIBED セルは通行不可（unknown は `allow_unknown` 次第）。経路抽出は start から最小コストの後続セルを辿る。

完全に決定論的（乱数を使わない）。

## closed-loop 統合テスト

`test/test_dstar_lite_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → start→goal の衝突なし経路
- 壁＋**中心から外れた隙間** → 隙間へ迂回して通過（衝突なし）
- **incremental replan**: クリアな地図で1回 plan → 壁を出現させ同一 goal で再計画 → 状態を再利用・修復して壁を避ける（D\* Lite の核心機能を検証）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `cost_weight` | 3.0 | 高コストセルをどれだけ避けるか（エッジコストの重み） |
| `heuristic_weight` | 1.0 | ヒューリスティックの重み（≤1 で admissible） |
| `allow_unknown` | true | costmap の unknown を通行可能とみなすか |

## 使い方（例）

planner_server の `GridBased` plugin を差し替える:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_dstar_lite_planner::DStarLitePlanner"
      cost_weight: 3.0
      heuristic_weight: 1.0
      allow_unknown: true
```

最小例: [../nav2_diffusion_bringup/params/dstar_lite_planner_example.yaml](../nav2_diffusion_bringup/params/dstar_lite_planner_example.yaml)。

## 関連

- サンプリング系の兄弟: [../nav2_rrt_planner/README.md](../nav2_rrt_planner/README.md)（RRT\* / RRT-Connect）, [../nav2_prm_planner/README.md](../nav2_prm_planner/README.md)（PRM）
- 生成型 GlobalPlanner（Mode B）: [../nav2_diffusion_global_planner/README.md](../nav2_diffusion_global_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
