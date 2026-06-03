# nav2_diffusion_global_planner

Nav2 GlobalPlanner Plugin integration（Mode B）。

**Status: スケルトン実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。**

`nav2_core::GlobalPlanner` を実装する Nav2 Planner Server プラグイン。start / goal と Global Costmap を受け取り、**生成モデルで大域パス候補を複数提案**し、決定論的な検証層を通した最短の衝突なしパスを返す（Mode B、[../docs/architecture.md](../docs/architecture.md) §3.2）。

> 調査の結果、**生成モデルを `nav2_core::GlobalPlanner` として統合した OSS は存在しない**（既存の NavFn / Smac / Theta\* は全て探索ベース。生成パスプランニングの研究実装は config-space〔マニピュレータ〕や nuPlan〔自動運転〕、RGB local policy に限られ、Nav2 GlobalPlanner ではない）。本パッケージはその統合 seam を埋める。新規性は「Nav2 ネイティブ統合 + costmap 条件付け + 検証/fallback 安全ラッパ」であり、diffusion パスプランニングの着想そのものではない。

## パイプライン（propose → dispose → select）

`nav2_diffusion_global_planner::DiffusionGlobalPlanner::createPlan()`:

1. **提案（multimodal）**: `nav2_diffusion_core::PathModel`（生成パスモデルの plugin seam）に start→goal の **K 個の候補パス**を生成させる。既定は組み込み `FanPathModel`（直線 + 左右に膨らむ detour 群のプレースホルダ）。`model_plugin` を設定すると **pluginlib で学習モデルを実行時ロード**（Controller の `TrajectoryModel` seam と対称）。
2. **入力検証**: start / goal が lethal セルや範囲外なら `StartOccupied` / `GoalOccupied` を送出。frame 不一致は `PlannerTFError`。
3. **検証（候補ごと）**: 各候補を `interpolation_resolution` で密にサンプルし、Global Costmap の各セルコストを判定（LETHAL / INSCRIBED → 不可、unknown は `allow_unknown` 次第）。
4. **選択**: 衝突なし候補のうち**最短**のものを `nav_msgs::Path` に変換して返す（各 pose は次点へ向けて orientation を設定、終点は goal の向き）。
5. **fallback**: 衝突なし候補が無ければ `NoValidPathCouldBeFound` を送出 → planner_server が recovery / replan。`cancel_checker` が true なら `PlannerCancelled`。

ニューラルモデルは直接パスを採用しない。必ず決定論的検証層を経る（「Learned models propose. Classical safety disposes.」）。

## closed-loop 統合テスト

`test/test_diffusion_global_planner.cpp` は、稼働中の `nav2_costmap_2d::Costmap2DROS` に対して実プラグインを configure/activate し、GPU/シム無しで検証する:

- クリアな costmap → start から goal への**直線パス**（最短候補が勝つ）
- 直線を塞ぐ部分障害物 → **detour 候補**を選び衝突なしで回避（横方向に膨らむ）
- goal が lethal セル → `GoalOccupied`
- 全幅を塞ぐ壁 → `NoValidPathCouldBeFound`
- `cancel_checker` が true → `PlannerCancelled`

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `num_candidates` | 11 | 生成する候補パス数（detour ファンのサンプル数） |
| `num_points` | 40 | 候補パス1本あたりの waypoint 数 |
| `interpolation_resolution` | 0.05 | 衝突判定のためのサンプル間隔 [m] |
| `allow_unknown` | true | costmap の unknown セルを通行可能とみなすか |
| `max_bow_fraction` | 0.5 | `FanPathModel` の最大膨らみ量（start-goal 距離に対する比） |
| `model_plugin` | "" | 生成パスモデルの `PathModel` plugin 名。空で組み込み `FanPathModel` |
| `model_path` | "" | `model_plugin` の `configure()` に渡すモデルパス（例: ONNX ファイル） |

## 使い方（例）

planner_server の `GridBased` plugin を差し替える:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_diffusion_global_planner::DiffusionGlobalPlanner"
      num_candidates: 11
      num_points: 40
      interpolation_resolution: 0.05
      allow_unknown: true
```

## 関連

- Mode A（Controller）: [../nav2_diffusion_controller/README.md](../nav2_diffusion_controller/README.md)
- パス提案モデル seam: `nav2_diffusion_core::PathModel`（[../nav2_diffusion_core](../nav2_diffusion_core)）
- 安全思想: [../docs/safety.md](../docs/safety.md)
