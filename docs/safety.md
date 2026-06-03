# Safety Architecture

> 関連: [architecture.md](architecture.md) §3.4 Non-Negotiable Rules / §7.4 Runtime Gating、[risks.md](risks.md) §14.3 Safety・Liability Risks

## 8.1 Safety Philosophy

この OSS の安全原則は次の通り。

> **The learned planner is never the final authority.**

生成モデルは「候補生成器」であり、「安全判定器」ではない。安全判定は Nav2 costmap、footprint、速度制約、sensor freshness、Collision Monitor、fallback behavior に分離する。

## 8.2 Safety Layers

| Layer | 役割 |
|---|---|
| Input Validity Layer | TF、odom、costmap、sensor timestamp を確認 |
| Observation Sanity Layer | costmap size、unknown ratio、sensor dropout を確認 |
| Candidate Validity Layer | NaN、範囲外、frame 不一致、horizon 不一致を除外 |
| Kinematic Safety Layer | max velocity, acceleration, angular velocity, curvature を確認 |
| Footprint Collision Layer | footprint-based collision check |
| Dynamic Risk Layer | moving obstacle との time-to-collision / predicted occupancy |
| Social Safety Layer | human personal space, crossing behavior, blind corner slow-down |
| Command Safety Layer | `cmd_vel` limit, smoothing, rate limiting |
| Nav2 Collision Monitor Layer | planner をバイパスした非常停止監視 |
| Hardware EStop Layer | ソフトウェア外の最終停止系 |

Nav2 Collision Monitor は、sensor 入力から collision avoidance 関連処理を行い、costmap や trajectory planner をバイパスして emergency-stop level で衝突を防ぐ追加安全層として設計されている。これは学習 planner の外側に置くべき層である。

## 8.3 Safety State Machine

| State | 状態 | `cmd_vel` |
|---|---|---|
| Nominal | safe candidates あり、latency 正常 | generative best trajectory 由来 |
| Cautious | confidence 低下、dynamic risk 高め | speed limited |
| Degraded | model 遅延、sensor 一部欠損 | previous safe / fallback |
| Fallback | model 無効、安全候補なし | MPPI / RPP / configured controller |
| Brake | imminent risk | controlled stop |
| Emergency Stop | collision monitor / hardware stop | zero or hardware stop |
| Recovery | Nav2 recovery behavior へ委譲 | BT 管理 |

## 8.4 Fallback Hierarchy

推奨 fallback 順序は robot 用途で変える。

### Warehouse AMR
1. speed limit
2. MPPI fallback
3. Regulated Pure Pursuit fallback
4. stop
5. recovery behavior
6. operator intervention

### Delivery / Service Robot
1. slow down
2. wait
3. socially conservative detour
4. fallback controller
5. stop
6. remote assistance

### Narrow Passage Robot
1. hold previous safe trajectory briefly
2. reverse-capable fallback if available
3. stop
4. recovery
5. operator intervention

## 8.5 Safety Deliverables

GitHub 公開前に最低限必要な安全 deliverables。

| Deliverable | 内容 |
|---|---|
| Safety Design Doc | 本ドキュメント |
| Failure Mode Table | GPU, TF, sensor, model, costmap, controller failure |
| Default Safe Config | model が壊れても止まる config |
| Collision Regression Tests | static / dynamic obstacle で候補棄却を確認 |
| RViz Safety Visualization | 棄却候補、理由、best trajectory |
| Fallback Demo | model timeout 時に MPPI/RPP/stop へ移行 |
| Hardware EStop Integration Guide | 実機向け注意事項 |

---

> ⚠️ この OSS は安全認証済み製品ではない。実機導入者は hardware EStop、速度制限、ODD 定義、現場 risk assessment を行う必要がある。OSS 側が提供すべきは認証の代替ではなく、安全設計の透明性・failure mode の明記・fallback の標準化・benchmark による比較・実機導入手順・危険な使い方の明示である（[risks.md](risks.md) §14.3 参照）。
