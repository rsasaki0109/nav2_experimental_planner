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
| `provide_costmap` | true | 正規化した大域 costmap を `PathContext` に詰めて costmap 条件付きモデルへ渡す（analytic モデルは無視。検証層は常に独立して衝突判定する） |
| `model_plugin` | "" | 生成パスモデルの `PathModel` plugin 名。空で組み込み `FanPathModel` |
| `model_path` | "" | `model_plugin` の `configure()` に渡すモデルパス（例: ONNX ファイル） |
| `fallback_planner_plugin` | "" | 有効候補が無いとき委譲する classical `nav2_core::GlobalPlanner` 名（例: `nav2_jps_planner::JPSPlanner`）。空なら `NoValidPathCouldBeFound` を投げる。設定すると **疎結合 hybrid**（learned 提案 → 失敗時のみ完全探索が dispose）になり、難所でも探索系を下回らない |
| `hybrid_mode` | "fallback" | `"guided"` にすると **密結合 hybrid**: 常に組み込みの完全な A* を走らせ、有効 proposal 近傍セルのコストを割引いて learned が毎回の経路形状を誘導する（完全性は探索が保証） |
| `guidance_strength` | 0.5 | guided 時、proposal 近傍セルのコスト割引率 [0,1) |
| `guidance_radius` | 0.3 | guided 時、proposal 周りの誘導コリドー半幅 [m] |

## 使い方（例）

コピペ用の最小例: [../nav2_diffusion_bringup/params/diffusion_global_planner_example.yaml](../nav2_diffusion_bringup/params/diffusion_global_planner_example.yaml)。planner_server の `GridBased` plugin を差し替える（既定: 組み込み `FanPathModel`）:

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

### 学習済み生成パスモデル（ONNX）を使う

`model_plugin` に `nav2_diffusion_onnx::OnnxPathModel` を指定すると、flow matching で学習した生成パスモデルを pluginlib で実行時ロードする（planner は onnxruntime に直接依存しない）。学習・export は `python3 -c "from nav2_diffusion_training.path_planners import train_and_export_path as t; t('path.onnx')"`。

```yaml
    GridBased:
      plugin: "nav2_diffusion_global_planner::DiffusionGlobalPlanner"
      model_plugin: "nav2_diffusion_onnx::OnnxPathModel"
      model_path: "/abs/path/to/path.onnx"
```

モデルは goal-aligned frame で K 個の多峰な候補パスを生成し、backend が map frame へ戻す。**候補の検証・選択は変わらず決定論的安全層が担う**（モデルは提案するだけ）。契約は [../nav2_diffusion_onnx/README.md](../nav2_diffusion_onnx/README.md) を参照。

**costmap 条件付きモデル**（`train_and_export_costmap_path` で学習）を使う場合、planner が `provide_costmap:=true`（既定）で大域 costmap を渡し、backend が goal-aligned パッチにリサンプルして供給する。モデルは障害物の無い側へ候補を寄せる＝costmap を読んだ賢い提案になる（local controller の costmap 条件付けと対称）。`model_plugin`/`model_path` の指定方法は context-only モデルと同じ。

**学習済みモデル（すぐ使える）**: [../model_zoo/diffusion_global/costmap_flow.onnx](../model_zoo/diffusion_global) が curated 済み。`model_path` にこれを指すだけで、costmap を読んで提案する学習済み Mode B が動く。挙動・限界は [model_card](../model_zoo/diffusion_global/model_card.md)、同一土俵の比較は [../docs/planner_comparison.md](../docs/planner_comparison.md) の *Mode B, learned* 行。

## 関連

- Mode A（Controller）: [../nav2_diffusion_controller/README.md](../nav2_diffusion_controller/README.md)
- パス提案モデル seam: `nav2_diffusion_core::PathModel`（[../nav2_diffusion_core](../nav2_diffusion_core)）
- 安全思想: [../docs/safety.md](../docs/safety.md)
