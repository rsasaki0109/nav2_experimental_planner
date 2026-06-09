# nav2_ara_star_planner

**ARA\* (Anytime Repairing A\*) global planner**（`nav2_core::GlobalPlanner`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）planner。**

ARA\*（Likhachev, Gordon and Thrun, 2003）は **inflation factor ε を下げながら重み付き A\* を反復**する anytime planner。最初の探索（大きい ε）は安価で、**最適の ε 倍以内**を保証する解を素早く返す。以降は ε を下げ、前回の探索結果（OPEN と INCONS リスト）を**再利用**して解を改善し、suboptimality bound を 1（最適）へ締めていく。よって時間予算が限られても**まず妥当な解を返し、予算が許す限り改善**する。

Nav2 公式には **anytime / bounded-suboptimal な planner が存在しない**ため、この *capability* 自体が空白（`Nav2PlannerBattle` = Nav2 に無い planner 群の一部）。他の classical 兄弟: サンプリング系 [nav2_rrt_planner](../nav2_rrt_planner/README.md)・[nav2_prm_planner](../nav2_prm_planner/README.md)、インクリメンタル [nav2_dstar_lite_planner](../nav2_dstar_lite_planner/README.md)、グリッド高速化 [nav2_jps_planner](../nav2_jps_planner/README.md)、any-angle [nav2_lazy_theta_star_planner](../nav2_lazy_theta_star_planner/README.md)。

## アルゴリズム

`nav2_ara_star_planner::ARAStarPlanner`:

1. **入力検証**: frame 不一致 → `PlannerTFError`、start/goal が範囲外/障害物 → `StartOccupied` / `GoalOccupied`。
2. **ImprovePath(ε)**: key = `g(s) + ε·h(s)` の優先度で、`key(goal)` より小さい key がある限り展開する重み付き A\*。展開済みノードが再度改善されたら CLOSED ではなく **INCONS** に退避。
3. **anytime ループ**: 初回 ImprovePath の後、`ε` を `epsilon_decrement` ずつ下げ、INCONS を OPEN へ戻し CLOSED をクリアして再探索。`ε` が 1 に達するか、総展開数が `max_iterations`（予算）に達するまで改善を続ける。
4. **抽出**: goal から親を辿って 8 連結セル経路を構築（端点は実 start/goal にスナップ）。予算切れでも最後に得た最良解を返す。到達不能なら `NoValidPathCouldBeFound`、`cancel_checker` true で `PlannerCancelled`。

8 連結グリッド上で、エッジコストは `base(1 or √2)·(1 + cost_weight·正規化コスト)`（`cost_weight=0` で純粋距離）。ヒューリスティックは octile（admissible）。完全に決定論的。

## closed-loop 統合テスト

`test/test_ara_star_planner.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要）:

- クリアな costmap → start→goal の衝突なし経路
- **inflated ε（=5.0）** → anytime ループを通っても妥当な経路を返す
- 壁＋**中心から外れた隙間** → 隙間へ迂回して通過（衝突なし）
- 隙間なしの壁 → `NoValidPathCouldBeFound`
- goal が lethal → `GoalOccupied`
- `cancel_checker` が true → `PlannerCancelled`

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `initial_epsilon` | 3.0 | 開始時の inflation factor（≥1） |
| `epsilon_decrement` | 0.5 | anytime 反復ごとの ε 減少幅 |
| `max_iterations` | 200000 | 全パス合計の展開数予算 |
| `cost_weight` | 0.0 | 高コストセル回避の重み（0 で純粋距離） |
| `lethal_threshold` | 253 | このコスト以上のセルを障害物とみなす |
| `allow_unknown` | true | costmap の unknown を通行可能とみなすか |

## 使い方（例）

planner_server の `GridBased` plugin を差し替える:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_ara_star_planner::ARAStarPlanner"
      initial_epsilon: 3.0
      epsilon_decrement: 0.5
      max_iterations: 200000
```

最小例: [../nav2_diffusion_bringup/params/ara_star_planner_example.yaml](../nav2_diffusion_bringup/params/ara_star_planner_example.yaml)。

## 関連

- サンプリング系: [../nav2_rrt_planner/README.md](../nav2_rrt_planner/README.md), [../nav2_prm_planner/README.md](../nav2_prm_planner/README.md)
- インクリメンタル系: [../nav2_dstar_lite_planner/README.md](../nav2_dstar_lite_planner/README.md)
- グリッド高速化: [../nav2_jps_planner/README.md](../nav2_jps_planner/README.md)
- any-angle: [../nav2_lazy_theta_star_planner/README.md](../nav2_lazy_theta_star_planner/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
