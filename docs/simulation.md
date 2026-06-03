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
