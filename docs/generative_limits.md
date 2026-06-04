# 生成モデルの効きどころと天井（実証ノート）

> 関連: [choosing_a_planner.md](choosing_a_planner.md) / [planner_comparison.md](planner_comparison.md) / [controller_comparison.md](controller_comparison.md) / [model_zoo/](../model_zoo)
>
> 本リポジトリの中核仮説は **「Learned models propose. Classical safety disposes. Nav2 executes.」**。本ノートは、v0.6.0 で収録した小型・合成学習モデル（costmap 条件付き flow の Mode A trajectory / Mode B path）を実際にループへ通して**どこで効き、どこで天井に当たるか**を実証的に整理し、天井を超えるための現実的な道筋を示す。安全認証品ではない点は [safety.md](safety.md) を参照。

## TL;DR

| 能力 | 小型・合成学習モデル | 状態 |
|---|---|---|
| costmap を読んで**回避側を選ぶ**（片側障害物 → 反対へ寄せる） | ✅ できる（両モード、全候補が空き側へ） | v0.6.0 出荷・end-to-end C++ テストで検証 |
| 学習分布内の patch での**側選択の頑健性** | ✅ できる | 同上 |
| Mode A: **open での閉ループ goal 到達** | ✅ できる（pure-pursuit 弧 expert で carrot 追従） | v0.6.0 出荷 |
| Mode B: **off-centre gap の方向検出（スロットを狙う）** | ⚠️ flow（CNN）は不可 → ✅ **transformer（attention）は raw 提案でスロット方向を向く** | 下記「追記」参照（A/B + C++ 方向テスト） |
| Mode B: **off-centre gap を実際に通る（footprint 検証 benchmark）** | ❌ 学習単体では天井（transformer でも狭スロット未貫通）→ ✅ **ハイブリッドで解決** | 下記参照 |
| Mode A: **障害物のスレッディング（回り込み通過）** | ❌ 学習単体では天井 → ✅ **ハイブリッドで解決** | 下記参照 |

要点: **side-selection と open goal 到達は小型モデルでも実機構で動く**。一方 **gap-routing と obstacle-threading は探索/分布シフトの問題**で、小型・合成学習モデル単体の天井。これは偶然ではなく、**classical search / reactive 法が本来勝つ領域**であり、本リポジトリが 8 種の classical planner と 2 種の reactive controller を併載する理由そのもの。そして **ハイブリッド**（generative 提案 + classical fallback）はこの天井を実際に超える: Mode B の off-centre gap / slalom は learned+JPS の hybrid が解く（後述）。

## 何が効くか（v0.6.0 で出荷済み）

- **costmap 条件付き側選択**: 片側に障害物がある egocentric / goal-aligned patch を入力すると、K 個の提案すべてが空き側へ寄る。解析的 placeholder（`FanPathModel` / `FanRolloutModel`、地図を見ず対称 bow）には出せない、本物の学習挙動。`nav2_diffusion_onnx` の end-to-end gtest（`CuratedZooModelVeersAwayFromObstacle`）で出荷バイナリを直接検証。
- **clear のバイアス除去**: 障害物なしでは中央（直進）。学習データに mirror ペア + clear サンプルを入れて左右バイアスを消した。
- **Mode A の open 完走**: expert を **pure-pursuit 弧（carrot へ向けて曲がる）**に再設計し、carrot 方位を多様化したことで、閉ループの横ズレを能動補正でき、open シナリオで goal 到達（`controller_comparison.md` の *Mode A, learned* 行）。

## 何が天井か（実証）

### Mode B: off-centre gap（壁のスロットを通る大迂回）

`planner_comparison.md` の *off-centre gap* は、直線を塞ぐ薄い壁に**直線から大きく外れた位置（約 2 m）にスロット**がある。learned Mode B はこれを通せない。試した順と所見:

1. gap シナリオ（壁+スロット patch、スロットを通る target）を学習データに追加 → 出力は**直進**（薄壁を obstacle と認識せず clear 扱い）。
2. 学習の壁を薄く・厚みを多様化（benchmark の数セル厚に合わせる）→ なお**直進**。
3. **直接 MSE（sample_weight）**で大 bow を強制 → *magnitude* は学習でき大きく曲がるが、**benchmark の patch では逆側へ曲がる**（スロットは −y にあるのに +y へ）。

**所見**: 迂回の大きさは学習できるが、薄壁スロットの**方向検出**が training patch → benchmark の再サンプル patch へ**転移しない**（小型 CNN encoder の脆さ）。off-centre gap は本質的に **routing/search 問題**で、わずかな patch の差で答えが反転する。完全性保証のある探索系（NavFn/Smac、本リポジトリの D\* Lite/JPS/visibility graph 等）の領域。

> 出荷している learned Mode B（flow）は v0.6.0 のまま（flow の gap 実験は出荷せず revert）。off-centre gap は flow では引き続き *no path*（安全側に fail-closed）として正直に表に出している。

#### 追記: transformer は「方向検出」だけは越える（が benchmark は未突破）

上の天井のうち **(3) 方向が転移しない** 部分は、**アーキテクチャで部分的に越えられる**ことが分かった（2026-06、`diffusion_global_costmap_transformer_v0`）。flow の 16 次元 CNN embedding を、**costmap パッチをトークン化して cross-attention する transformer**（DETR 風 set-prediction）に替えると、**同じ gap データで raw 提案がスロット方向を正しく向く**:

- 直接 A/B（同一 gap データ・同一 patch）: flow は loss 0.12 で **routing せず**（直進/逆側）、transformer は loss 0.002 で **両側ともスロット方向**（wall での横ズレ ≈ ±2 m）。C++ 方向テスト `OnnxPathModelTest.CuratedZooTransformerAimsAtOffCentreSlot` で出荷バイナリを検証。
- **つまり「方向検出が小型 encoder の脆さで転移しない」は容量/アーキの問題**であり、本質的限界ではない。

**ただし、これは benchmark の突破ではない（正直なスコープ）**:

- footprint 検証付きの `planner_comparison.md` *off-centre gap*（幅 1 m の狭スロット）では、transformer の提案も**有効 path を通せず** `DiffusionGlobalPlanner` は *no path*（flow と同じ）。提案はスロットを「狙う」が、狭スロットを footprint 余裕込みで**貫通**するには至らない。
- さらに gap と side を混ぜて学習した `'both'` モデルは、現状 **side obstacle で flow に劣る**（benchmark で *no path*）。
- よって **gap の完全な解は引き続き hybrid**（generative 提案 → classical 探索が完全性を担保）。transformer が示したのは「**提案ステージの方向限界は表現の問題で、容量・footprint 対応学習・広いスロットで将来 benchmark 突破に届きうる**」という布石であって、現時点の benchmark 勝利ではない。

> 出荷: `diffusion_global_costmap_transformer_v0` は**研究デモ**として model_zoo に収録（raw 方向の A/B と C++ 方向テスト付き）。benchmark には載せていない（pure-generative では未突破で、載せると flow より全面的に劣って誤解を招くため）。

### Mode A: 障害物スレッディング（回り込み通過）

learned Mode A は open では goal 到達するが、`controller_benchmark` の *frontal / side / corridor* では障害物が egocentric patch に入った時点で **drift → 安全候補なし → 安全停止**（障害物の手前、衝突なし）。これは:

- **閉ループの分布シフト**: 提案 → 実行 → 再観測の系で小さな誤差が累積し、モデルが学習分布の外の状態（障害物が近い + carrot が斜め）に入ると出力が崩れる。
- **context 感度**: モデルは context（goal_x/y, max_angular など）の値に敏感で、benchmark のパラメータを学習分布に合わせる必要があった（lookahead/限界の調整で改善するが脆い）。

単発の Mode B gap ですら転移しないことから、閉ループで誤差が累積する obstacle-threading は同じ（むしろ強い）天井。安全層（kinematic + footprint）が**衝突は常に防ぐ**点は意図どおり。

**ただしハイブリッドで解決済み（✅）**: `DiffusionController` の `fallback_controller_plugin`（既存）に classical の reactive controller（VFH+ 等）を設定すると、安全候補が無いとき停止する代わりに委譲する。`controller_comparison.md` の **Diffusion (Mode A, hybrid)** 行は **全シナリオで goal 到達**（open は learned、障害物は VFH+ fallback が回避。corridor 行は VFH+ と完全一致 = fallback 稼働の証拠）。Mode B planner の hybrid と完全に対称。

## なぜ天井になるか（要因）

1. **小型 CNN/MLP の容量**: encoder 16 次元・MLP 128 隠れ。薄い・部分的な障害物信号や gap 位置の汎化に弱い。
2. **合成 → 実 costmap の転移**: 学習 patch と C++ 再サンプル patch の微差（壁厚・gap 幅・行位置）で判断が反転。
3. **単発 imitation の限界**: 探索（gap-routing）は完全性が要る問題で、提案分布の被覆だけでは安定して解けない。
4. **閉ループの分布シフト**: open は安定だが、障害物近傍は訪問状態が学習分布から外れる。

## 天井を超える道筋（future work）

1. **ハイブリッド（✅ 実装済み・2 形態）**: generative 提案 + classical の完全性。本リポジトリは 2 つの結合度を実装している。
   - **疎結合（fallback）**: `DiffusionGlobalPlanner` の `fallback_planner_plugin`（Mode A controller の `fallback_controller_plugin` と同型）。learned 提案が無効なときだけ classical search/reactive に委譲。簡単な地図は高速な generative 経路、難所は探索系。`planner_comparison.md` の **Mode B, hybrid**（learned + JPS）は全シナリオ解決（clear/side は generative 12 pose、gap/slalom は JPS 81/107 pose）。`controller_comparison.md` の **Mode A, hybrid**（learned + VFH+）も全シナリオ到達。
   - **密結合（guided）**: `hybrid_mode: guided`。**常に**完全な A* を走らせ、有効な proposal の近傍セルのコストを割引くので、learned が**毎回**の経路形状を誘導しつつ探索が完全性を保証する。**Mode B, guided** も全シナリオ解決（cell 解像度 81/111 pose — fallback と違い簡単な地図でも探索を走らせる）。
2. **閉ループ学習（DAgger）（✅ 基盤実装済み・効果は限定的）**: `nav2_diffusion_training/dagger.py`。現方策を numpy costmap シム（C++ の egocentric crop / first-segment 抽出 / lookahead / dt を踏襲）でロールアウトし、訪れた状態で expert（carrot への pure-pursuit + costmap 回避）にラベルを問い、集約して再学習する。**分布シフトに対する正しい道具**で、ループは end-to-end で動く（pytest 付き）。ただし現状の **小型モデル + 軽量シム**では閉ループ成功は marginal（0/4 → 1/4、集約が増えると loss も増えて fit しきれない）。**忠実な改善には大容量モデルと、より忠実な閉ループ（実 C++/Gazebo ロールアウト）が要る**——下の項目と組み合わせるのが筋。
3. **大容量モデル + 多様な実データ**: rosbag / sim から goal・障害物・gap 配置を広く集めて学習（[training.md](training.md)）。CNN/Transformer encoder の増強。DAgger と組み合わせると効く見込み。
4. **TensorRT / Jetson 実機検証**: [deployment.md](deployment.md) §11 / [roadmap.md](roadmap.md)。

## 結論

小型・合成学習モデルでも **side-selection と open goal 到達は実機構で動く**（v0.6.0 出荷）。**gap-routing と obstacle-threading は classical が勝つ探索/分布シフトの領域**で、学習単体の天井。だが **ハイブリッド**は両モードでその天井を実際に超える: Mode B planner は `fallback_planner_plugin`（learned 提案 → classical search が完全性を保証）で、Mode A controller は `fallback_controller_plugin`（learned → classical reactive が回避）で、いずれも**全シナリオを解く**。この境界を正直に測り、両者を組み合わせて最良を取れること自体が「generative propose / classical dispose」設計の価値であり、両者を同一リポジトリ・同一土俵に載せている理由である。
