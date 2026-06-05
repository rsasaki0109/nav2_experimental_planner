# nav2_diffusion_bringup

example launch/config for Nav2。

**Status: closed-loop demo の launch / params あり。**

既存 Nav2 ユーザーが **Controller を差し替えるだけ** で試せる launch / param 例を提供する。試用障壁を下げることが DX 上の勝ち筋（[../docs/architecture.md](../docs/architecture.md) §15.6）。

## 内容

| ファイル | 役割 |
|---|---|
| `params/nav2_diffusion_tb3.yaml` | nav2_bringup の `nav2_params.yaml` から **controller_server → FollowPath だけ** を `DiffusionController` に差し替えた完全な Nav2 params（他は Nav2 デフォルト） |
| `params/diffusion_controller_example.yaml` | controller_server ブロックのみの最小スニペット（Mode A 生成型、ドキュメント用） |
| `params/vfh_controller_example.yaml` | controller_server ブロックのみの最小スニペット（classical reactive、`VFHController`） |
| `params/nd_controller_example.yaml` | controller_server ブロックのみの最小スニペット（classical reactive、`NDController`） |
| `params/diffusion_global_planner_example.yaml` | planner_server ブロックのみの最小スニペット（Mode B、`DiffusionGlobalPlanner`） |
| `params/rrt_planner_example.yaml` | planner_server ブロックのみの最小スニペット（classical、`RRTStarPlanner` / `RRTConnectPlanner`） |
| `params/prm_planner_example.yaml` | planner_server ブロックのみの最小スニペット（classical、`PRMPlanner`） |
| `params/dstar_lite_planner_example.yaml` | planner_server ブロックのみの最小スニペット（classical incremental、`DStarLitePlanner`） |
| `params/jps_planner_example.yaml` | planner_server ブロックのみの最小スニペット（classical、`JPSPlanner`） |
| `params/lazy_theta_star_planner_example.yaml` | planner_server ブロックのみの最小スニペット（classical any-angle、`LazyThetaStarPlanner`） |
| `params/ara_star_planner_example.yaml` | planner_server ブロックのみの最小スニペット（classical anytime、`ARAStarPlanner`） |
| `params/visibility_graph_planner_example.yaml` | planner_server ブロックのみの最小スニペット（classical geometric、`VisibilityGraphPlanner`） |
| `launch/tb3_loopback_diffusion.launch.py` | loopback シムでの closed-loop demo |
| `launch/tb3_gazebo_diffusion.launch.py` | Gazebo での closed-loop demo |
| `launch/foxglove.launch.py` | Foxglove 可視化（`foxglove_bridge` + 候補/安全状態 marker 変換ノード） |
| `foxglove/nav2_diffusion_layout.json` | Foxglove Studio レイアウト（3D 候補軌道 / 安全状態 / cmd_vel） |

## 実行

```bash
# loopback（軽量・GPU 不要。nav2_loopback_sim が必要）
ros2 launch nav2_diffusion_bringup tb3_loopback_diffusion.launch.py

# Gazebo（標準 demo。シム LiDAR/カメラの描画に GPU が必要）
ros2 launch nav2_diffusion_bringup tb3_gazebo_diffusion.launch.py headless:=True
```

RViz の "2D Pose Estimate" で初期姿勢を、"Nav2 Goal" でゴールを与えると、`DiffusionController` が走行する。

### Foxglove 可視化

```bash
ros2 launch nav2_diffusion_bringup foxglove.launch.py
```

Foxglove Studio で `ws://<host>:8765` に接続し、`foxglove/nav2_diffusion_layout.json` を Import すると、候補軌道・安全状態・cmd_vel のパネルが揃う。詳細は [../docs/visualization.md](../docs/visualization.md)。

## 前提条件と検証状況（重要）

- **loopback demo** は `nav2_loopback_sim`（apt: `ros-<distro>-nav2-loopback-sim`）が必要。
- **Gazebo demo** はシム LiDAR/カメラのレンダリングに **動作する GPU** が必要。GPU レンダリング不可の環境では `/scan` が出ず closed-loop が成立しない。

### NVIDIA GPU でヘッドレス実行する場合

NVIDIA + headless で `libEGL: failed to create dri2 screen`（Mesa が選ばれる）になる場合、NVIDIA EGL ベンダを明示すると解消する（検証済み: dri2 エラーが解消し gz サーバーがシーンをレンダリングして起動する）:

```bash
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export TURTLEBOT3_MODEL=waffle   # DISPLAY も有効にしておく
ros2 launch nav2_diffusion_bringup tb3_gazebo_diffusion.launch.py use_rviz:=false headless:=True
```

gz サーバーを手動起動する場合は、ワールドのモデルが見つかるよう `GZ_SIM_RESOURCE_PATH` に
`$(ros2 pkg prefix nav2_minimal_tb3_sim)/share/nav2_minimal_tb3_sim/models` を含めること（未設定だと
"Unable to find or download file" で gz サーバーが終了する）。Fuel からのモデル取得が必要なワールドではネットワークも要る。
- これらの実走行 demo に依存しない **closed-loop 検証**として、`nav2_diffusion_controller` に統合テスト（`test/test_diffusion_controller_integration.cpp`）を用意している。稼働中の `nav2_costmap_2d::Costmap2DROS` に対して実プラグインを configure/activate し、**クリア路では前進・footprint が障害物に当たる場合は stop** することを GPU/シム無しで検証する。これが現状の「動く」ことの一次保証。
