# Benchmark Suite

> 関連: [architecture.md](architecture.md) §3.2 Mode C / §7.5 Profiling、[simulation.md](simulation.md)

## 9.1 Benchmark Goal

Benchmark の目的は「Diffusion がすごい」ことを示すことではない。目的は次の 3 つ。

1. Nav2 標準構成と公正に比較する
2. 実運用で重要な失敗モードを定量化する
3. 新しいモデル追加時に regression を防ぐ

ROS 2 の benchmark では、既知 workload で性能を測り、アルゴリズムや version 間の trade-off、regression を把握することが重要であるという考え方が示されている。

## 9.2 Baselines

| Baseline | 比較理由 |
|---|---|
| Smac + MPPI | Nav2 modern strong baseline |
| Smac + RPP | 産業・サービス用途の実用 baseline |
| NavFn + RPP | 軽量・伝統的構成 |
| Smac + DWB | 歴史的 Controller 比較 |
| MPPI tuned | default ではなく、scenario ごとに合理的に tuned |
| Human Teleop | social / narrow passage reference |
| Oracle Sim Expert | 上限参考 |

MPPI との比較では、default config だけで勝ったと主張してはいけない。MPPI は critic 設計と tuning で挙動が大きく変わるため、**tuned baseline、default baseline、compute-matched baseline** を分ける。

## 9.3 Scenario Taxonomy

| Scenario Class | 例 |
|---|---|
| Static Easy | open space, simple corridor |
| Static Hard | narrow passage, doorway, U-shaped trap, maze |
| Dynamic Obstacles | crossing agents, opposing traffic, sudden obstacle |
| Human-aware | person standing, person crossing, crowd flow, queue |
| Warehouse | pallets, shelves, forklifts, blind corners |
| Delivery / Service | hallway, elevator front, lobby, people clusters |
| Sensor Noise | scan dropout, delayed odom, costmap flicker |
| Map Mismatch | moved obstacle, unknown area, partial occlusion |
| Recovery | blocked path, forced reverse, local minima |
| Stress | high obstacle density, low compute, GPU contention |

## 9.4 Metrics

| Category | Metrics |
|---|---|
| Task | success rate, timeout rate, goal error |
| Safety | collision count, near-collision count, minimum clearance, TTC violation |
| Efficiency | time-to-goal, path length, detour ratio, stop duration |
| Smoothness | linear/angular jerk, oscillation count, command sign flips |
| Social | personal-space intrusion time, human crossing delay, pass-behind/pass-front ratio |
| Robustness | stuck count, recovery count, fallback count |
| Compute | CPU, GPU, memory, p50/p95/p99 latency |
| Model | candidate diversity, rejection rate, confidence calibration |
| Nav2 Integration | lifecycle stability, TF errors, costmap stale events |
| Reproducibility | seed variance, bag replay consistency |

## 9.5 Benchmark Methodology

1. 同じ map、同じ robot model、同じ global planner で比較する
2. Controller だけ差し替える条件を作る
3. random seed を固定し、dynamic obstacle の初期条件を保存する
4. default config と tuned config を分けて報告する
5. closed-loop simulation を主評価にする
6. open-loop ADE/FDE だけで主張しない
7. latency p99 を必ず出す
8. safety override 率を出す
9. benchmark 結果は model card に記録する
10. regression threshold を CI に入れる

## 9.6 Public Leaderboard

GitHub Star を獲得するうえで、benchmark leaderboard は強い。だが、leaderboard は危険でもある。単純な success rate だけにすると、遅すぎる安全運転や過剰停止が勝つ可能性がある。

推奨 score は複合指標にする。

| Score | 構成 |
|---|---|
| Safety Score | collision, near-collision, TTC, emergency stop |
| Progress Score | success, time, path efficiency |
| Comfort Score | jerk, oscillation, stop-start |
| Social Score | human distance, crossing behavior |
| Runtime Score | latency p95/p99, GPU memory |
| Deployment Score | Jetson/x86 compatibility |
| Overall Score | safety を最重視した weighted score |
