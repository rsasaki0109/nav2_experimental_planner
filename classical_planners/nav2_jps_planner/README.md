# nav2_jps_planner

**Jump Point Search (JPS) global planner**（`nav2_core::GlobalPlanner`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

JPS（Harabor & Grastien, 2011）は均一コストグリッド上の 8 連結 A\* を**最適性を保ったまま高速化**する手法。グリッド経路の対称性を利用し、A\* が 1 セルずつ展開する直線/斜めのセル列を**ジャンプで飛ばし**、ターニングポイントと forced neighbor（強制隣接）だけを open list に積む。展開ノード数が大幅に減り、A\* と同じ最適経路を返す。Nav2 公式のグリッド planner（NavFn / Smac）に JPS は含まれない（`Nav2PlannerBattle` = Nav2 に無い planner 群の一部）。サンプリング系の兄弟に [nav2_rrt_planner](../nav2_rrt_planner/README.md)（RRT\* / RRT-Connect）・[nav2_prm_planner](../nav2_prm_planner/README.md)（PRM）、インクリメンタル系に [nav2_dstar_lite_planner](../nav2_dstar_lite_planner/README.md)（D\* Lite）。

## アルゴリズム

`nav2_jps_planner::JPSPlanner` は jump point を節点とした A\*:

1. **入力検証**: frame 不一致 → `PlannerTFError`、start/goal が範囲外/障害物 → `StartOccupied` / `GoalOccupied`。
2. **jump**: ある方向へ進み、forced neighbor が現れる・goal・障害物に当たるまで一気にジャンプ。斜め方向では各ステップで水平・垂直成分のジャンプを先に調べる。
3. **prunedDirections**: 親からの到達方向に基づき、対称な経路を生む冗長な方向を枝刈りして探索方向（natural + forced）だけを残す。
4. **A\***: octile 距離ヒューリスティック（admissible）で jump point 間を探索。経路再構築後、連続する jump point 間の直線/斜め区間をセル単位で**密化**して `nav_msgs::Path` を構築（端点は実 start/goal にスナップ）。
5. **fallback**: 到達不能なら `NoValidPathCouldBeFound`、`cancel_checker` true で `PlannerCancelled`。

**均一コスト前提**のため、costmap を二値（free / blocked）として扱う：コストが `lethal_threshold` 以上のセルを障害物とみなし、inflation の段階コストは無視する。ソフトなコスト整形が要る場合は NavFn/Smac や D\* Lite を使う。完全に決定論的。

## closed-loop 統合テスト

`test/test_jps_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → start→goal の衝突なし最適経路
- 壁＋**中心から外れた隙間** → 隙間へ迂回して通過（衝突なし）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `lethal_threshold` | 253 | このコスト以上のセルを障害物とみなす（既定は INSCRIBED） |
| `allow_unknown` | true | costmap の unknown を通行可能とみなすか |

## 使い方（例）

planner_server の `GridBased` plugin を差し替える:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_jps_planner::JPSPlanner"
      lethal_threshold: 253
      allow_unknown: true
```

最小例: [../nav2_diffusion_bringup/params/jps_planner_example.yaml](../nav2_diffusion_bringup/params/jps_planner_example.yaml)。

## 関連

- サンプリング系の兄弟: [../nav2_rrt_planner/README.md](../nav2_rrt_planner/README.md)（RRT\* / RRT-Connect）, [../nav2_prm_planner/README.md](../nav2_prm_planner/README.md)（PRM）
- インクリメンタル系: [../nav2_dstar_lite_planner/README.md](../nav2_dstar_lite_planner/README.md)（D\* Lite）
- 生成型 GlobalPlanner（Mode B）: [../nav2_diffusion_global_planner/README.md](../nav2_diffusion_global_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
