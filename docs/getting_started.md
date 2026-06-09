# Getting Started

> 対象: 既存の Nav2 ユーザー。関連: [architecture.md](architecture.md) §3.2 Mode A、[simulation.md](simulation.md)、[../nav2_diffusion_bringup/README.md](../generative/nav2_diffusion_bringup/README.md)

`Nav2PlannerBattle` は、Nav2 の plugin を差し替えるだけで、**Nav2 公式に無い planner / controller** を試せます。生成型ローカルプランナ（「Learned models propose. Classical safety disposes. Nav2 executes.」）に加え、**classical な GlobalPlanner 8 種**（RRT\* / RRT-Connect / PRM / D\* Lite / JPS / Lazy Theta\* / ARA\* / visibility graph）と **reactive Controller 2 種**（VFH+ / ND）を収録しています。

どれを選ぶかは [choosing_a_planner.md](choosing_a_planner.md)（状況別の推奨・決定フロー）、実測比較は [planner_comparison.md](planner_comparison.md) / [controller_comparison.md](controller_comparison.md) を参照。

## 必要環境

- ROS 2 Jazzy + Nav2
- closed-loop demo を回す場合: `nav2_loopback_sim`（軽量）または Gazebo（要 GPU レンダリング、シム LiDAR のため）

## ビルド

```bash
# ワークスペースの src/ に本リポジトリを置く
colcon build
source install/setup.bash
```

ランタイムに必要なパッケージは C++ 中心で軽量です（学習依存は `nav2_diffusion_training` 側に分離、[architecture.md](architecture.md) §12.2）。

## プラグインが見えるか確認

```bash
ros2 plugin list | grep DiffusionController
# nav2_diffusion_controller::DiffusionController (base: nav2_core::Controller)
```

MPPI / RPP と同じ `nav2_core::Controller` として登録されます。

## 既存 Nav2 config に差し替える

controller_server の `FollowPath` プラグインを差し替えるだけです。

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_diffusion_controller::DiffusionController"
      # パラメータは nav2_diffusion_controller/README.md を参照
```

完全な例: [../nav2_diffusion_bringup/params/nav2_diffusion_tb3.yaml](../generative/nav2_diffusion_bringup/params/nav2_diffusion_tb3.yaml)（Nav2 デフォルト params の FollowPath だけを差し替えたもの）。

## classical な planner / controller を試す

生成モデル無しで、Nav2 公式に無い **classical** な GlobalPlanner / Controller も同じく plugin 差し替えだけで使えます（学習依存・GPU 不要）。

### GlobalPlanner を差し替える（planner_server）

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_rrt_planner::RRTStarPlanner"   # 下表の class に置き換え
```

| 使いたいもの | plugin class | 例 yaml |
|---|---|---|
| RRT\* / RRT-Connect | `nav2_rrt_planner::RRTStarPlanner` / `::RRTConnectPlanner` | `rrt_planner_example.yaml` |
| PRM | `nav2_prm_planner::PRMPlanner` | `prm_planner_example.yaml` |
| D\* Lite | `nav2_dstar_lite_planner::DStarLitePlanner` | `dstar_lite_planner_example.yaml` |
| JPS | `nav2_jps_planner::JPSPlanner` | `jps_planner_example.yaml` |
| Lazy Theta\* | `nav2_lazy_theta_star_planner::LazyThetaStarPlanner` | `lazy_theta_star_planner_example.yaml` |
| ARA\* | `nav2_ara_star_planner::ARAStarPlanner` | `ara_star_planner_example.yaml` |
| visibility graph | `nav2_visibility_graph_planner::VisibilityGraphPlanner` | `visibility_graph_planner_example.yaml` |
| 生成型 Mode B | `nav2_diffusion_global_planner::DiffusionGlobalPlanner` | `diffusion_global_planner_example.yaml` |

例 yaml はすべて [../nav2_diffusion_bringup/params/](../generative/nav2_diffusion_bringup/params) にあり、各 plugin の全パラメータは対応パッケージの README を参照。

生成型 Mode B は既定では解析的 `FanPathModel` で動くが、**学習済みモデル**に差し替えられる。[model_zoo](../model_zoo/diffusion_global) の costmap 条件付き flow モデル（`costmap_flow.onnx`）を使うには:

```yaml
    GridBased:
      plugin: "nav2_diffusion_global_planner::DiffusionGlobalPlanner"
      model_plugin: "nav2_diffusion_onnx::OnnxPathModel"   # 学習済みモデルをロード
      model_path: "/path/to/model_zoo/diffusion_global/costmap_flow.onnx"
      provide_costmap: true
```

このモデルは costmap を読んで提案を空き側へ寄せる（挙動と限界は [model_card](../model_zoo/diffusion_global/model_card.md)、横断比較は [planner_comparison.md](planner_comparison.md) の *Mode B, learned* 行）。`OnnxPathModel` は `nav2_diffusion_onnx` + onnxruntime が必要。

さらに `fallback_planner_plugin: "nav2_jps_planner::JPSPlanner"` を足すと **hybrid**（learned 提案 → 有効候補が無ければ classical search が完全性を保証）になり、off-centre gap のような難所も解ける（比較表の *Mode B, hybrid* 行は全シナリオ通過、[generative_limits.md](generative_limits.md)）。

### reactive Controller を差し替える（controller_server）

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_vfh_controller::VFHController"   # または nav2_nd_controller::NDController
```

例: `vfh_controller_example.yaml` / `nd_controller_example.yaml`。

生成型 Mode A(`nav2_diffusion_controller::DiffusionController`)も**学習済み軌道モデル**に差し替えられる。[model_zoo](../model_zoo/diffusion_local) の costmap 条件付き flow モデルを使うには:

```yaml
    FollowPath:
      plugin: "nav2_diffusion_controller::DiffusionController"
      model_plugin: "nav2_diffusion_onnx::OnnxTrajectoryModel"
      model_path: "/path/to/model_zoo/diffusion_local/costmap_flow.onnx"
      costmap_patch_size: 32          # egocentric patch をモデルへ渡す
```

モデルが costmap を読んで障害物の反対側へ軌道を提案し、決定論的安全層(kinematic + footprint)が検証する。閉ループでは open シナリオで goal に到達し、障害物シナリオでは安全層が手前で安全停止する(小型研究モデルの限界)。挙動・限界は [model_card](../model_zoo/diffusion_local/model_card.md) を参照。`OnnxTrajectoryModel` は `nav2_diffusion_onnx` + onnxruntime が必要。

さらに `fallback_controller_plugin: "nav2_vfh_controller::VFHController"` を足すと **hybrid**(安全候補が無ければ classical reactive controller へ委譲)になり、障害物シナリオも回避して全シナリオ goal 到達(比較表の *Mode A, hybrid* 行、[generative_limits.md](generative_limits.md))。Mode B planner の hybrid と対称。

### 登録確認

```bash
ros2 plugin list | grep -E "Planner|Controller"
```

8 つの GlobalPlanner と VFH+ / ND Controller が `nav2_core::GlobalPlanner` / `nav2_core::Controller` として並びます。

## closed-loop demo

```bash
# loopback（軽量・GPU 不要、nav2_loopback_sim が必要）
ros2 launch nav2_diffusion_bringup tb3_loopback_diffusion.launch.py

# Gazebo（標準 demo、シム LiDAR/カメラの描画に GPU が必要）
ros2 launch nav2_diffusion_bringup tb3_gazebo_diffusion.launch.py headless:=True
```

RViz の "2D Pose Estimate" で初期姿勢、"Nav2 Goal" でゴールを与えると走行します。

demo launch は `candidate_markers` ノードも起動します。RViz に **MarkerArray display** を追加して `/candidate_markers` を指定すると、候補軌道が **best=緑 / safe=青 / rejected=赤** で表示されます（[../nav2_diffusion_rviz_plugins/README.md](../generative/nav2_diffusion_rviz_plugins/README.md)）。controller の候補トピックが異なる場合は `candidates_topic:=...` で指定してください。

## 安全性の前提（必読）

- これは安全認証済み製品ではありません。実機では hardware EStop・速度制限・ODD 定義・現場 risk assessment が必須です（[safety.md](safety.md)）。
- GPU が死んでもロボットは停止 or fallback します。`fallback_controller_plugin` に MPPI/RPP を設定すると、安全候補が無い場合に委譲します（§8.4）。

## 次のステップ

- どれを使うか迷ったら: [choosing_a_planner.md](choosing_a_planner.md)（状況別の推奨・決定フロー）
- 横断比較: [planner_comparison.md](planner_comparison.md)（classical GlobalPlanner 8 種 + 生成型 Mode B）/ [controller_comparison.md](controller_comparison.md)（VFH+ vs ND）
- 挙動の理解: 候補軌道（`TrajectoryCandidates`）と `SafetyState` を RViz / `ros2 topic echo` で観察
- 自分の robot へ: [../nav2_diffusion_controller/README.md](../generative/nav2_diffusion_controller/README.md) のパラメータ
- 自社データで学習・評価: [training.md](training.md) / [benchmarking.md](benchmarking.md)
- モデルを追加: [contributing.md](contributing.md)
