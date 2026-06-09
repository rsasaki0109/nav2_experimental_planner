# Nav2PlannerBattle — 開発計画 (PLAN)

> 本ドキュメントは「いま何ができていて、次に何を、どの順で、何を満たせば完了とするか」を
> 一枚に束ねた **生きた計画書**。バージョン軸の高レベル計画は
> [docs/roadmap.md](docs/roadmap.md)、データ・環境依存の着手手順は
> [docs/next_phase.md](docs/next_phase.md)、純生成の限界の実証は
> [docs/generative_limits.md](docs/generative_limits.md)、変更履歴は
> [CHANGELOG.md](CHANGELOG.md) を正本とする。本書はそれらを横断して
> **現在地 → 次の一手** の流れで読めるようにしたもの。
>
> 最終更新: 2026-06-09（Gazebo mission harness 強化 · Nav2PlannerBattle 名で維持）

---

## 0. このドキュメントの読み方

- **第 1〜3 章**: いまの立ち位置と、何が本当に動いているか（誇張なし・実 C++ ベンチの数値が正本）。
- **第 4 章**: 純生成で越えられていない天井と、それに対する設計判断。
- **第 5 章**: 次フェーズの実行計画（GPU / シム / 実データ / 実機のどれか 1 つで着手できる段）。
- **第 6 章**: v1.0 / v2.0 への長期ロードマップ。
- **第 7 章**: Nav2 Planner Battle（ブラウザゲーム）の発展計画 — リブランドで前面に出た新しい軸。
- **第 8〜9 章**: スコープ外と着手チェックリスト。

---

## 1. プロジェクトの現在地

### 1.1 アイデンティティ

**Nav2PlannerBattle** は Nav2 を置き換えるプロジェクトではない。Nav2 の既存アーキテクチャ
（Behavior Tree / Lifecycle Node / Planner & Controller プラグイン / Costmap / Collision
Monitor）を最大限活かし、その上に **生成型ナビゲーションモデルを安全に接続する OSS 基盤**である。

合言葉は **「Learned models propose. Classical safety disposes. Nav2 executes.」**
（学習モデルが提案し、古典的な安全層が棄却し、Nav2 が実行する）。

リブランド（2026-06-08）により、リポジトリ名は **Nav2PlannerBattle** に統一。
これは単なる改名ではなく、本リポジトリのもう一つの軸 ——
**「Nav2 公式に無い planner / controller を実装し、同一シナリオで戦わせて比較する」**
—— を前面に出すもの。比較は表（docs/\*\_comparison.md）だけでなく、本物のプラグインを
そのままブラウザで競わせる [Nav2 Planner Battle](tools/nav2_planner_battle) ゲームにも結実している。

### 1.2 二つのモードと安全層

| モード | 役割 | seam | 安全層（dispose） |
|---|---|---|---|
| **Mode A**（local / Controller） | `nav2_core::Controller`。costmap+goal 条件で K 本の軌道候補を提案 → スコアリングで最良を選び `cmd_vel` 化 | `TrajectoryModel`（pluginlib / ONNX） | footprint 衝突ゲート（full / windowed）・kinematic 制限・停止 fallback |
| **Mode B**（global / GlobalPlanner） | `nav2_core::GlobalPlanner`。パス候補を提案 → costmap 検証 → 最短安全パスを選択 | `PathModel`（pluginlib / ONNX） | footprint 検証・**曲率検証（min turn radius）**・classical fallback |

提案は学習モデル、棄却は決定論的な安全層 —— この **propose / dispose 分離**が本リポジトリの背骨。
v0.x の間に footprint だけでなく**車両動力学（曲率）**にも dispose を拡張済み。

### 1.3 収録物（一区切り済みの土台）

- **生成ファミリ 5 系統**（Mode A）/ **3 系統**（Mode B）を同一 ONNX 契約で実装
  （flow / diffusion / consistency / transformer(DETR 風 set-prediction) / recurrent(GRU 自己回帰)）。
- **classical GlobalPlanner 8 種**: RRT\* / RRT-Connect / PRM / D\* Lite / JPS / Lazy Theta\* / ARA\* / visibility graph。
- **reactive Controller 2 種**: VFH+ / ND。
- **DAgger 閉ループ学習基盤**（`dagger.py`）と **rosbag/sim ingestion**（`rosbag_io.py` / `dataset.py`）。
- **model_zoo**（manifest + model card + 再現可能な `export.py`、`.onnx` は gitignore・sha256 が正本）。
- **横断比較ベンチ**（`nav2_planner_benchmarks` / `nav2_diffusion_benchmarks`）と自動生成 leaderboard。
- **Nav2 Planner Battle**（`tools/nav2_planner_battle`）+ トレース出力器 `battle_trace`。
- 可搬可視化（MCAP + Foxglove/Lichtblick、Gazebo コース GIF）。

---

## 2. これまでの到達点（バージョン別ハイライト）

| ver | テーマ | 主な到達点 |
|---|---|---|
| v0.1–0.3 | Nav2 ネイティブ MVP + 生成スタック | Controller プラグイン・決定論的安全層・multimodal 候補 + scorer・ONNX backend・flow/diffusion/consistency・costmap 条件付け・学習パイプライン・RViz/Foxglove |
| v0.4 | charter 拡大 | classical GlobalPlanner 8 種 + VFH+。「生成型」から「Nav2 に無い実験的 planner 全般」へ |
| v0.5 | 再現性 + 選択ガイド | ND・横断比較ベンチ・`choosing_a_planner.md` |
| v0.6 | **初の学習モデルがループに入る** | model_zoo に Mode A/B の costmap 条件付き flow を収録、実 C++ ONNX 推論で一周（learned Mode A が open で閉ループ goal 到達） |
| v0.7 | ハイブリッド | 各モードに classical fallback（Mode B=JPS、Mode A=VFH+）。両モードとも全シナリオ解決 |
| v0.8 | 密結合 guided + DAgger | 常時 A\* + 提案近傍コスト割引、DAgger 閉ループ学習基盤 |
| v0.9 | 生成ファミリ拡充 | transformer + recurrent を両 seam に実装・出荷 |
| v0.10 | **off-centre-gap 天井突破** | 微分可能 footprint クリアランス損失で Mode B transformer が off-centre gap を純生成で貫通。8 コース評価・パッケージ再編・CI 復活・Gazebo 単一生成コース |
| v0.11 | **gap トレードオフを容量で解消** | transformer 増強(dim64/8heads/3blocks) + dead-ahead tri-mix で off-centre と dead-ahead を同時貫通。MCAP/Foxglove 可搬可視化 |
| Unreleased | **Mode A 全障害物貫通 + kinematics 条件 Mode B + Battle ゲーム** | 下記 2.1〜2.3 |

### 2.1 Mode A obstacle-threading: 全障害物コースを純生成で貫通（Unreleased）

学習 Mode A コントローラとして**初めて、fallback 無しで全障害物コースを貫通**。3 つの梃子で実現:

1. **windowed footprint gate**（`safety_check_points`）: 直近 N 点だけ検証する receding-horizon ゲート（default 0 = 従来の全 horizon 棄却 → 後方互換）。far lookahead で障害物を掠めても、タイトな回避スカートが棄却されず生き残る。
2. **持続コミット oracle**（`dagger.py`）: 衝突する DAgger oracle を、自由側へ**ブロック通過まで保持**するコミット回避に修正。`corridor` シナリオ追加。単一コミット dodge を回帰（multimodal セットは progress-greedy セレクタが左右で打ち消すため）。
3. **視野拡大**（`costmap_patch_resolution`）: パッチの stride を costmap 解像度から分離（default 0.0 = native）。同じ 32 セルが ±0.775 m → **±1.24 m** を張り、dead-ahead ブロックを **~1.6 倍早く**検出。

**実 C++ `controller_benchmark`（fallback なし）の結果（正本）**:
- *frontal*（正面）: 到達 4.24 m / クリアランス 0.48 m（従来の最後の holdout）
- *side*（側方）: 4.05 m 走破（learned/transformer/recurrent は ~1.0 m で停滞）
- *corridor*（回廊）: 到達かつ **古典より良いセンタリング**（mean |y-centre| 0.20 m vs VFH+ 0.28 / ND 0.31）
- **DAgger 閉ループ sim: 1/4 → 5/6**（残る失敗は sim 専用の "two" slalom のみ）

> 教訓: 検出が間に合わないなら、容量より**視野**を広げる。新規シーンでは hybrid(VFH+) が完全性を担保。

### 2.2 Kinematics 条件 Mode B planner（Unreleased）

1 モデルで複数の steering geometry を全 8 コースで提案。`PathModel` の 2 つ目の context スロットに
**最小旋回半径 R**（R=0 = omni）を載せ、`isPathValid` に **Menger 外接円半径による曲率検証**を追加
（1/R より急な提案を棄却）。propose/dispose を footprint から**車両動力学**へ拡張。

**実 C++ benchmark（同一 weights）**: **omni 8/8**（slalom 含む全通過、gate off）/ **diff 8/8** /
**Ackermann 3/8**（near-straight な clear / centred gap / double gate のみ。曲率検証が
narrow/off-centre/far/side/slalom を正しく棄却 —— 1.5 m 旋回円を越える横移動が必要なため）。

### 2.3 Nav2 Planner Battle（ブラウザゲーム）（Unreleased）

本物のプラグインのトレースを `battle_trace`（`controller_benchmark` / `planner_benchmark` を厳密にミラー）で
JSON 出力し、自己完結 HTML/canvas で再生。**Mode A · Race**（local controller が共有アリーナを競走、
threading が dead-ahead を貫通する横で plain learned/transformer/recurrent が停滞）と
**Mode B · Duel**（global planner が `createPlan` を同時描画、最短安全パス勝ち）。
Gazebo では同一 JSON パスを複数 TB3 で 3D リプレイ（`tools/record_battle_gazebo_gif.py` →
`docs/battle_gazebo_*.gif`）。

---

## 3. 現状の能力マトリクス

| 領域 | 状況 | 備考 |
|---|---|---|
| Nav2 Controller Plugin（Mode A） | ✅ | in-process 閉ループ統合テスト |
| Nav2 GlobalPlanner Plugin（Mode B） | ✅（先取り） | 生成型 GlobalPlanner（OSS 不在を確認の上） |
| 安全層（入力検証 / kinematic / footprint / 曲率 / 状態機械 / fallback） | ✅ | propose/dispose の dispose 側 |
| multimodal 候補 + scorer | ✅ | progress-greedy セレクタ |
| TrajectoryModel / PathModel seam（pluginlib） | ✅ | 実行時ロード |
| ONNX Runtime backend | ✅ | optional パッケージ |
| 生成ファミリ（flow/diffusion/consistency/transformer/recurrent） | ✅ | 同一 ONNX 契約 |
| costmap+goal 条件付き生成 | ✅ | heading-aligned egocentric パッチ・2 入力 ONNX |
| 学習パイプライン（rosbag/sim→dataset→PyTorch→ONNX） | ✅ | DAgger 基盤含む |
| 可視化（RViz / Foxglove / MCAP / GIF） | ✅ | 候補・棄却理由・安全状態 |
| 横断比較ベンチ + leaderboard | ✅ | 自動生成 |
| **Mode A 全障害物コース純生成貫通** | ✅ | frontal + side + corridor（§2.1） |
| **Mode B off-centre + dead-ahead gap 純生成貫通** | ✅ | v0.11（far off-centre / slalom は hybrid 領域） |
| **kinematics 条件 Mode B** | ✅ | omni 8/8・diff 8/8・Ackermann 3/8（§2.2） |
| Battle ゲーム | ✅ | §2.3 |
| 実機 shadow mode / 実走行 numbers | ⬜ | 実機・実 sim 環境が必要（§5 段 3） |
| 実 sim 閉ループ numbers（Gazebo） | ⬜ | mission ハーネス + launch 起動順硬化済み（§5 段 1）。数値は専用前景実行で取得 |
| TensorRT backend / Jetson 検証 | ⬜ | §5 段 4 |
| social / human-aware / world model | ⬜ | v2.0 研究枠 |

> API は **1.0.0 まで安定化しない**。本 OSS は**安全認証済み製品ではない**（実機展開には
> ハードウェア E-stop・速度制限・ODD 定義・現地リスクアセスメントが必須。[docs/safety.md](docs/safety.md)）。

---

## 4. 未解決の天井と判断

[docs/generative_limits.md](docs/generative_limits.md) に実証済み。要点:

- **純生成で越えた**: Mode A の frontal/side/corridor、Mode B の off-centre / dead-ahead gap。
- **まだ hybrid 領域**: Mode B の *far off-centre gap* と *slalom*、Mode A sim 専用 "two" slalom。
- **kinematics 制約**: Ackermann は曲率検証が物理的に不可能な横移動を正しく棄却（5/8 は到達不能が正解）。

**判断**: これ以上の純ソフト（合成データの作り込み）での天井突破は、転移しないことが実証済み。
本筋は **容量増 + 実データ + 忠実な閉ループ**（§5）。新規・想定外シーンでは hybrid が完全性を担保する
設計を維持する（learned が全部を解く必要はない —— propose が外れたら dispose と fallback が拾う）。

---

## 5. 次フェーズ実行計画

> 詳細は [docs/next_phase.md](docs/next_phase.md)。本章はその要約 + リブランド後の現況反映。
> **共通の鍵**: 次の前進は「もっとコードを書く」ことではなく、
> **GPU・シム・実データ・実機のどれか 1 つを用意して対応する段に着手する**こと。
> 足場（5 生成ファミリ・2 入力 ONNX 契約・DAgger 基盤・安全層・hybrid・比較ベンチ）は揃っている。

### 前提（どれか 1 つ整えば対応段に着手可）

| 区分 | 必要なもの | 現状 |
|---|---|---|
| 計算 | 学習用 GPU（CUDA が安定初期化できる占有環境） | ⬜（現状 CUDA 初期化が停滞 → 学習は CPU で凌ぐ） |
| シム | Gazebo / Isaac 閉ループ | 部分（launch/params あり、実走 numbers 未収集） |
| データ | rosbag / sim ログ（多様な gap・expert/介入ラベル） | ⬜（現状は合成のみ、ingestion 経路は実装済み） |
| 実機 | 差し替え可能な Nav2 ロボット | ⬜（安全層・fallback・診断は実装済み） |
| エッジ | Jetson | ⬜ |

### 段 1 — 忠実な閉ループでの DAgger（GPU + シム）

軽量 numpy シムの marginal を脱し、分布シフトを本当に潰す。
1. ロールアウトを numpy シムから**実 C++ コントローラ / Gazebo** に差し替え（crop/first-segment/lookahead/dt は C++ と既に整合）。
2. モデル容量を上げる（transformer エンコーダ増強）。
3. β スケジュール減衰で集約・再学習を反復。

**DoD**: `controller_benchmark` の障害物シナリオで learned 単体の閉ループ成功を拡張し衝突ゼロを保つ。

> **Gazebo bring-up + mission ハーネスは実装・実走検証済み**（headless 起動・GPU LiDAR /scan 360・
> ros_gz bridge・Nav2 ロード・mission の結果ファイル書き出しまで健全）。**唯一の壁は本サンドボックスの
> inter-process DDS が完全不通**（talker/listener でも受信 0）。**実 ROS ホストに移せば
> `tb3_gazebo_mission.launch.py` がそのまま実 sim numbers を出す**（[docs/simulation.md](docs/simulation.md) §10.5）。

### 段 2 — 大容量モデル + 多様な実/シムデータ（GPU + データ）

小型容量と「合成→実」転移ギャップを埋める。
1. rosbag/sim から goal・障害物・**off-centre gap** 配置を広く収集（ingestion 実装済み）。
2. costmap エンコーダ増強（トークン化 + 位置埋め込みは既存 → 段数/幅/解像度を上げる）。
3. 段 1 の DAgger と併用。

**DoD**: Mode B の *far off-centre gap* を hybrid 委譲でなく **learned 単体**で通せる事例を出す
（少なくとも 1 配置、再サンプル patch で転移が反転しないことを確認）。達成できなければ
「探索が勝つ領域」という現結論をより強い証拠付きで再確認して記録。

### 段 3 — 実機 shadow mode（実機 + データ）

安全層を効かせたまま実走行分布で learned 提案の質を測る。
1. learned コントローラを shadow（cmd は既存プランナ、提案はログのみ）で実機に。
2. 介入・棄却理由・安全状態を収集。
3. ログを段 2 のデータへ還流（fleet learning の最小形）。

**DoD**: 実走行 numbers（成功率・介入率・棄却理由分布）を [docs/model_comparison.md](docs/model_comparison.md) に実機列として追加。

### 段 4 — TensorRT / Jetson（エッジ実機）

edge-GPU での実時間性を確定。
1. ONNX → TensorRT backend を `nav2_diffusion_onnx` の seam の隣に追加（optional パッケージ作法を踏襲）。
2. Jetson でレイテンシ / 消費電力を計測。

**DoD**: roadmap v1.0 の「x86 GPU and Jetson validated」を満たす数値を [docs/deployment.md](docs/deployment.md) に記録。

---

## 6. バージョンロードマップ（長期）

> 正本は [docs/roadmap.md](docs/roadmap.md)。本章は要約。

- **v1.0 — Production-Ready OSS**: API/メッセージスキーマ安定化、TensorRT backend、完全な安全アーキ、
  公開再現ベンチ、model zoo に検証済み 2 モデル以上、x86 GPU + Jetson 検証、Controller 安定 / Planner optional、
  CPU CI + GPU nightly + sim regression、deployment/tuning/troubleshooting docs、binary release。
  **DoD**: Nav2 ユーザーが既存ロボットに導入し、ベンチと安全ガイドで採用可否を判断できる。
- **v2.0 — Generative Navigation Platform**: world model rollout planner、human-aware / social cost、
  semantic（camera/BEV）統合、fleet learning、hidden-seed leaderboard + 実機レポート、
  サードパーティ model プラグイン、maintainer council による RFC ガバナンス。
  **DoD**: 研究者が新モデルを枠組みに載せて Nav2 ベンチで比較でき、企業が安全層 + deployment 込みで実機評価できる。

---

## 7. Nav2 Planner Battle（ゲーム）発展計画

リブランドで前面に出た軸。現状（§2.3）から先の候補（優先度順、いずれも任意）:

1. **トレースの拡充**: battle に出すシナリオ/ファイターを増やす（kinematics 条件 Mode B の omni/diff/Ackermann を
   並走させる、Mode A に hybrid 行を出す等）。データ生成は `battle_trace` を回すだけ。
2. **指標オーバーレイ**: クリアランス・センタリング・曲率・planning time をレース中にライブ表示し、
   docs/\*\_comparison.md の数値とゲームを 1:1 で対応づける。
3. **GitHub Pages 公開**: 自己完結 HTML なので静的ホスティングでそのまま遊べる。共有可能な deep-link は実装済み。
4. **「あなたのモデル」対戦**: ユーザーが学習した ONNX を `battle_trace` に通して既存陣営と戦わせる導線。
   ✅ [`docs/custom_model_battle.md`](docs/custom_model_battle.md) · `tools/battle_custom_model.sh` ·
   `battle_trace --custom-controller|--custom-planner`.
   段 2/3 の fleet learning と結びつく。
5. **回帰の可視化**: CI で `battle_trace` を回し、トレースの差分（到達/クリアランスの劣化）を検出する軽い regression gate。
   ✅ `tools/check_battle_trace.py` — outcome/success・length・**clearance** の劣化検出 + カテゴリ別サマリ（CI ログ向け）。

> 原則: ゲームは**本物のプラグインの再生**であり続ける（スクリプト勝者を作らない）。
> 数値の正本はあくまで Markdown ベンチ（`battle_trace` はそれを JSON でミラーするだけ）。

---

## 8. スコープ外

- 合成データのさらなる作り込みでの天井突破（転移しないと実証済み。容量 + 実データが本筋）。
- classical planner/controller の網羅追加（一区切り済み）。
- 安全認証（本リポジトリは研究/評価用、[docs/safety.md](docs/safety.md)）。
- Manipulation / MoveIt / Humanoid / Full VLA / Multi-Agent Planning（主目的外）。

---

## 9. 着手チェックリスト

次に着手する人（含む将来の自分）への最短経路:

- [ ] **GPU が空いた** → 段 1（DAgger を Gazebo 閉ループへ）。`CUDA_VISIBLE_DEVICES` を確認、学習は CPU でも回る。
- [ ] **実 ROS ホストが使える** → `tb3_gazebo_mission.launch.py` を実行し、実 sim numbers を取得（サンドボックスの DDS 壁を回避）。
- [ ] **rosbag/sim ログが手に入った** → 段 2（ingestion → 容量増モデル再学習 → 再サンプル patch で検証）。
- [ ] **実機が用意できた** → 段 3（shadow mode、提案はログのみ・cmd は既存プランナ）。
- [ ] **Jetson が用意できた** → 段 4（TensorRT backend 追加 → レイテンシ計測）。
- [ ] **どれも無い** → battle ゲームの発展（§7）か docs/比較の磨き込み。コードはこれ以上書かなくても前進は止まらない。
