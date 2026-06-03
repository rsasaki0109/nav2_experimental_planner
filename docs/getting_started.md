# Getting Started

> 対象: 既存の Nav2 ユーザー。関連: [architecture.md](architecture.md) §3.2 Mode A、[simulation.md](simulation.md)、[../nav2_diffusion_bringup/README.md](../nav2_diffusion_bringup/README.md)

`nav2_diffusion_planner` は、Nav2 の Controller を差し替えるだけで生成型ローカルプランナを試せます。「Learned models propose. Classical safety disposes. Nav2 executes.」

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

完全な例: [../nav2_diffusion_bringup/params/nav2_diffusion_tb3.yaml](../nav2_diffusion_bringup/params/nav2_diffusion_tb3.yaml)（Nav2 デフォルト params の FollowPath だけを差し替えたもの）。

## closed-loop demo

```bash
# loopback（軽量・GPU 不要、nav2_loopback_sim が必要）
ros2 launch nav2_diffusion_bringup tb3_loopback_diffusion.launch.py

# Gazebo（標準 demo、シム LiDAR/カメラの描画に GPU が必要）
ros2 launch nav2_diffusion_bringup tb3_gazebo_diffusion.launch.py headless:=True
```

RViz の "2D Pose Estimate" で初期姿勢、"Nav2 Goal" でゴールを与えると走行します。

demo launch は `candidate_markers` ノードも起動します。RViz に **MarkerArray display** を追加して `/candidate_markers` を指定すると、候補軌道が **best=緑 / safe=青 / rejected=赤** で表示されます（[../nav2_diffusion_rviz_plugins/README.md](../nav2_diffusion_rviz_plugins/README.md)）。controller の候補トピックが異なる場合は `candidates_topic:=...` で指定してください。

## 安全性の前提（必読）

- これは安全認証済み製品ではありません。実機では hardware EStop・速度制限・ODD 定義・現場 risk assessment が必須です（[safety.md](safety.md)）。
- GPU が死んでもロボットは停止 or fallback します。`fallback_controller_plugin` に MPPI/RPP を設定すると、安全候補が無い場合に委譲します（§8.4）。

## 次のステップ

- 挙動の理解: 候補軌道（`TrajectoryCandidates`）と `SafetyState` を RViz / `ros2 topic echo` で観察
- 自分の robot へ: [../nav2_diffusion_controller/README.md](../nav2_diffusion_controller/README.md) のパラメータ
- 自社データで学習・評価: [training.md](training.md) / [benchmarking.md](benchmarking.md)
- モデルを追加: [contributing.md](contributing.md)
