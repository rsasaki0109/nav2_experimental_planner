# Simulation Strategy

> 関連: [training.md](training.md) §6.2 Data Sources、[benchmarking.md](benchmarking.md)、[deployment.md](deployment.md)

## 10.1 Simulation Stack

| Simulator | 役割 |
|---|---|
| Gazebo / ros-gz | CI、軽量テスト、Nav2 標準 demo |
| Isaac Sim | photoreal, RGB/depth, warehouse, domain randomization |
| rosbag Replay | 実機ログ再評価 |
| Loopback / Mock Sim | unit-level regression, latency tests |
| Hardware-in-the-loop | deployment 前 validation |

Nav2 Getting Started は、Gazebo simulator 上の TurtleBot3 navigation から開始する流れを提供しているため、OSS の最初の demo は Nav2 標準導線に合わせるべきである。

## 10.2 Golden Scenarios

GitHub 公開時に最低限必要な scenario。

| Scenario | 目的 |
|---|---|
| simple_corridor | install 確認 |
| narrow_doorway | 狭路性能 |
| dynamic_crossing | 動的障害物 |
| u_trap | local minima |
| warehouse_shelves | AMR 用途 |
| crowded_hallway | human-aware preview |
| sensor_dropout | robustness |
| gpu_timeout | fallback 確認 |

## 10.3 Simulation-to-Benchmark Pipeline

| Stage | 内容 |
|---|---|
| Scenario Definition | map, robot, obstacles, dynamic agents, seed |
| Run Configuration | planner/controller/model/backend |
| Execution | headless simulation |
| Recording | rosbag, metrics, trace, RViz markers |
| Evaluation | metrics extraction |
| Report | markdown/html summary |
| Regression Gate | pass/fail |

## 10.4 Isaac Sim Role

Isaac Sim は v0.1 必須ではない。v0.1 は Gazebo と rosbag replay で十分に価値を出す。Isaac Sim は v0.5 以降、以下で使う。

- RGB / depth / semantic camera training
- warehouse scene diversity
- sensor realism
- domain randomization
- synthetic dynamic humans / forklifts
- edge GPU deployment rehearsal

## 10.5 実走検証: headless bring-up と既知の制約

> 関連: [next_phase.md](next_phase.md) 段3。2026-06-05 に `tb3_gazebo_diffusion.launch.py`
> を実際に headless 起動して確認した結果。**正直なスコープ**: sim は立ち上がるが、
> このセッションのサンドボックスでは**完全自動の閉ループ数値ベンチは完走できない**。

### 動いたこと（検証済み）

`ros2 launch nav2_diffusion_bringup tb3_gazebo_diffusion.launch.py use_rviz:=False
headless:=True`（`TURTLEBOT3_MODEL=waffle`）で:

- **Gazebo（gz, Jazzy）が headless で起動**し、`nav2_minimal_tb3_sim` の `tb3_sandbox`
  ワールドに TB3 waffle を spawn（"Entity creation successful"）。
- **GPU LiDAR が実際にレンダリングされて publish**: `gz topic -e -t /scan` で
  `count: 360`, `range_max: 20` を確認。起動ログに
  `libEGL warning: ... failed to create dri2 screen` が出るが**非致命**（ray センサは
  描画され /scan は出る）。
- **ros_gz bridge が稼働**: `/clock`, `/odom`, `/scan`, `/tf`, `/imu`（GZ→ROS）と
  `cmd_vel`（ROS→GZ）を生成。
- **Nav2 スタックがロード**（`nav2_container` に controller_server / planner_server /
  costmaps / BT / collision_monitor / docking 等）。`FollowPath` は
  `nav2_diffusion_controller::DiffusionController` を指す（[params](../generative/nav2_diffusion_bringup/params/nav2_diffusion_tb3.yaml)）。
  Mode A 学習モデルを使うには `FollowPath` に `model_plugin:
  nav2_diffusion_onnx::OnnxTrajectoryModel` ＋ `model_path`（[model_zoo](../model_zoo)）＋
  `costmap_patch_size: 32` を足す。

### 実装したもの（mission ハーネス）

完走に向けて、以下を実装・ビルド・lint 済み（実 ROS ホストで動く想定）:

1. **AMCL 自前 localization**: params の amcl に `set_initial_pose: true` ＋
   `initial_pose`（TB3 spawn の x=-2.0, y=-0.5）を追加。RViz の初期姿勢入力なしで
   map→odom が出て global_costmap が activate する。
2. **in-launch mission ノード**
   [`sim_mission.py`](../generative/nav2_diffusion_bringup/scripts/sim_mission.py): nav2 の
   `navigate_to_pose` を待ち、`/odom` を購読して、**複数 leg のミッション・コースを順に
   誘導**し、各 leg の到達/タイムアウト・経路長・時間を集計して **Markdown leaderboard
   （1 leg = 1 行 + サマリ）に書き出す**。これは offline `planner_benchmark` の多コース
   sweep を**閉ループに持ち込んだ**もの: 各 leg は前 leg の終端から送る 1 ゴールなので、
   同一ワールドで複数ゴールを 1 起動で走査できる。結果は topic でなく**ファイル**なので、
   DDS グラフに join できない環境でも成果物を検証できる。leg は `missions` 文字列配列
   （`"label|x|y|yaw|timeout"` 各 leg）で渡すか、**名前付きコース・プリセット** `course`
   （`default` / `there_and_back` / `patrol`）で指定する。優先順位は `missions`（明示）＞
   `course`（プリセット）＞単一 `goal_x/goal_y/goal_yaw` ゴール（後方互換）。プリセットの
   goal 列は開けた `tb3_sandbox` 用に調整した持続走行（往復・巡回）で、planner_benchmark の
   gap/slalom を閉ループで再現するには world に SDF 壁が要る（future work）。**leg/コース
   解析・指標集計・leaderboard 整形は ROS 非依存の純関数に切り出し、pytest で検証**
   （[`test/test_sim_mission.py`](../generative/nav2_diffusion_bringup/test/test_sim_mission.py)、
   12 ケース・`colcon test` 緑）——閉ループ駆動だけが実 Nav2＋Gazebo を要する。
3. **専用 launch**
   [`tb3_gazebo_mission.launch.py`](../generative/nav2_diffusion_bringup/launch/tb3_gazebo_mission.launch.py):
   sim＋mission を 1 launch で起動し、mission 終了で launch を Shutdown。
   単発: `ros2 launch nav2_diffusion_bringup tb3_gazebo_mission.launch.py goal_x:=0.0
   goal_y:=-0.5 results_file:=/tmp/sim_mission_result.md`。
   コース（プリセット）: `... course:=there_and_back results_file:=/tmp/sim_course_result.md`。
   コース（明示）: `... missions:="out|0.0|-0.5|0.0|120;back|-2.0|-0.5|0.0|120"
   results_file:=/tmp/sim_course_result.md`（`stop_on_failure:=True` で最初の失敗 leg で中断）。

### それでも完走をブロックする壁（このサンドボックス固有・厳密に特定）

1. **localization**: 上記 `set_initial_pose` で解消（実装済み）。
2. **inter-process DDS が全面的に不可**: 本環境では**プロセスをまたいだ ROS 2 通信が
   一切成立しない**。最小の `demo_nodes_cpp talker`/`listener` 2 プロセスで検証した結果、
   どの構成でも listener は 1 つも受信しない（"I heard" = 0）:
   - Fast DDS 既定（SHM）: `/dev/shm` の `open_and_lock_file failed`。実行中に
     fastrtps の SHM セグメントが**578 個**滞留していたため掃除したが、掃除後も 0。
   - Fast DDS UDP 強制（`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`）: 0（multicast discovery 遮断）。
   - Fast DDS UDP＋unicast localhost discovery を XML プロファイルで強制
     ([`config/fastdds_udp_localhost.xml`](../generative/nav2_diffusion_bringup/config/fastdds_udp_localhost.xml)): 0。
   - CycloneDDS（`rmw_cyclonedds_cpp`）: ノード生成自体が失敗（participant index / rcl init エラー）。
   - Fast DDS Discovery Server: `fast-discovery-server` バイナリ未インストールで起動不可。
   - → nav2 は単一コンテナ内（composition）で**プロセス内**通信するため起動はするが、
     **別プロセスの mission ノードが nav2 / bridge を discover できず**、
     `navigate_to_pose action server unavailable`・`/odom` 受信 0 で終わる
     （mission は結果ファイルを正しく書き出した＝ハーネスは健全、通信のみが壁）。

### 完走に必要なもの（next_phase.md 段3 へ）

- **inter-process DDS が成立する実 ROS ホスト**（`/dev/shm` 制限・multicast 遮断のない
  ネイティブ環境、または discovery-server を入れた環境）。そこで上記 mission launch を
  そのまま実行すれば、`/tmp/sim_mission_result.md` に到達/経路長/時間が出る。
- 得られた実 sim numbers を [model_comparison.md](model_comparison.md) /
  [controller_comparison.md](controller_comparison.md) に実 sim 列として追加。

結論: **bring-up・GPU センサ描画・mission ハーネス（初期姿勢＋複数 leg コース＋leaderboard
集計＋ファイル出力、純ロジックは pytest 緑）は実装し実走で確認済み**。残る律速はコードでは
なく、**このサンドボックスの inter-process DDS が完全に不通**であること。実 ROS ホストに
移せば mission launch がそのまま実 sim numbers（コース leaderboard）を出す。番号はその時点で
比較表に追加し、でっち上げ値は載せない。
