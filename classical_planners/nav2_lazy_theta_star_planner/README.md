# nav2_lazy_theta_star_planner

**Lazy Theta\* any-angle global planner**（`nav2_core::GlobalPlanner`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

Theta\*（Nash et al.）はノードの親を「視線（line-of-sight）が通る任意の先行ノード」にできるため、経路が 8 グリッド方向に縛られず**障害物の角でだけ曲がり、それ以外は直進する any-angle 経路**を生む。**Lazy Theta\***（Nash, Koenig and Tovey, 2010）はその LOS 判定を**遅延**する変種で、後続ノードが祖父ノードから見えると楽観的に仮定し、ノード展開時にだけ検証・修復する。これにより LOS 判定回数が「エッジ毎」から「展開ノード毎」へ激減する。

Nav2 公式は **eager な Theta\*** を `nav2_theta_star_planner` で提供するが、本 **lazy 変種**は別アルゴリズムで未収録（`Nav2PlannerBattle` = Nav2 に無い planner 群の一部）。サンプリング系に [nav2_rrt_planner](../nav2_rrt_planner/README.md)（RRT\* / RRT-Connect）・[nav2_prm_planner](../nav2_prm_planner/README.md)（PRM）、インクリメンタル系に [nav2_dstar_lite_planner](../nav2_dstar_lite_planner/README.md)（D\* Lite）、グリッド高速化に [nav2_jps_planner](../nav2_jps_planner/README.md)（JPS）。

## アルゴリズム

`nav2_lazy_theta_star_planner::LazyThetaStarPlanner`:

1. **入力検証**: frame 不一致 → `PlannerTFError`、start/goal が範囲外/障害物 → `StartOccupied` / `GoalOccupied`。
2. **lazy 後続生成（path 2）**: ノード `u` 展開時、各 open 隣接 `n` を `u` の親経由に楽観的に繋ぐ（`g(parent(u)) + dist(parent(u), n)`）。LOS は確認しない。
3. **SetVertex（遅延検証）**: ノードを pop した時、`LOS(parent, node)` が実は通らなければ、視線の通る展開済み隣接の中で最小コストのものに親を張り替える（path 1）。
4. **再構築 + 密化**: goal から親を辿って any-angle の頂点列を得て、各（LOS 済みの）直線区間を `interpolation_resolution` 間隔で密化して `nav_msgs::Path` を構築（端点は実 start/goal にスナップ）。

LOS は密化と同一のワールド標本化で判定するため、探索が受理した区間は密化後も衝突しない（整合性を保証）。**均一コスト前提**のため costmap を二値化（`lethal_threshold` 以上を障害物）。ヒューリスティックは Euclidean（`heuristic_weight` ≤ 1 で admissible）。完全に決定論的。

## closed-loop 統合テスト

`test/test_lazy_theta_star_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → start→goal の衝突なし経路
- **any-angle 直線**: クリア地図で軸に乗らない goal へ → 階段状でなく単一の直線（直線からの逸脱が ~0）
- 壁＋**中心から外れた隙間** → 隙間へ迂回して通過（衝突なし）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `lethal_threshold` | 253 | このコスト以上のセルを障害物とみなす（既定は INSCRIBED） |
| `allow_unknown` | true | costmap の unknown を通行可能とみなすか |
| `heuristic_weight` | 1.0 | ヒューリスティックの重み（≤1 で admissible） |
| `interpolation_resolution` | 0.05 | 経路密化 & LOS 標本化の間隔 [m] |

## 使い方（例）

planner_server の `GridBased` plugin を差し替える:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_lazy_theta_star_planner::LazyThetaStarPlanner"
      lethal_threshold: 253
      allow_unknown: true
      interpolation_resolution: 0.05
```

最小例: [../nav2_diffusion_bringup/params/lazy_theta_star_planner_example.yaml](../nav2_diffusion_bringup/params/lazy_theta_star_planner_example.yaml)。

## 関連

- サンプリング系: [../nav2_rrt_planner/README.md](../nav2_rrt_planner/README.md)（RRT\* / RRT-Connect）, [../nav2_prm_planner/README.md](../nav2_prm_planner/README.md)（PRM）
- インクリメンタル系: [../nav2_dstar_lite_planner/README.md](../nav2_dstar_lite_planner/README.md)（D\* Lite）
- グリッド高速化: [../nav2_jps_planner/README.md](../nav2_jps_planner/README.md)（JPS）
- 生成型 GlobalPlanner（Mode B）: [../nav2_diffusion_global_planner/README.md](../nav2_diffusion_global_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
