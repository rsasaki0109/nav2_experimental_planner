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
| Mode B: **off-centre gap（大迂回・スロット通過）** | ❌ 天井 | 下記参照 |
| Mode A: **障害物のスレッディング（回り込み通過）** | ❌ 天井（安全層が手前で安全停止） | 下記参照 |

要点: **side-selection と open goal 到達は小型モデルでも実機構で動く**。一方 **gap-routing と obstacle-threading は探索/分布シフトの問題**で、小型・合成学習モデルの天井。これは偶然ではなく、**classical search / reactive 法が本来勝つ領域**であり、本リポジトリが 8 種の classical planner と 2 種の reactive controller を併載する理由そのもの。

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

> 出荷している learned Mode B は v0.6.0 のまま（gap 実験は出荷せず revert）。off-centre gap は引き続き *no path*（安全側に fail-closed）として正直に表に出している。

### Mode A: 障害物スレッディング（回り込み通過）

learned Mode A は open では goal 到達するが、`controller_benchmark` の *frontal / side / corridor* では障害物が egocentric patch に入った時点で **drift → 安全候補なし → 安全停止**（障害物の手前、衝突なし）。これは:

- **閉ループの分布シフト**: 提案 → 実行 → 再観測の系で小さな誤差が累積し、モデルが学習分布の外の状態（障害物が近い + carrot が斜め）に入ると出力が崩れる。
- **context 感度**: モデルは context（goal_x/y, max_angular など）の値に敏感で、benchmark のパラメータを学習分布に合わせる必要があった（lookahead/限界の調整で改善するが脆い）。

単発の Mode B gap ですら転移しないことから、閉ループで誤差が累積する obstacle-threading は同じ（むしろ強い）天井。安全層（kinematic + footprint）が**衝突は常に防ぐ**点は意図どおり。

## なぜ天井になるか（要因）

1. **小型 CNN/MLP の容量**: encoder 16 次元・MLP 128 隠れ。薄い・部分的な障害物信号や gap 位置の汎化に弱い。
2. **合成 → 実 costmap の転移**: 学習 patch と C++ 再サンプル patch の微差（壁厚・gap 幅・行位置）で判断が反転。
3. **単発 imitation の限界**: 探索（gap-routing）は完全性が要る問題で、提案分布の被覆だけでは安定して解けない。
4. **閉ループの分布シフト**: open は安定だが、障害物近傍は訪問状態が学習分布から外れる。

## 天井を超える道筋（future work）

1. **大容量モデル + 多様な実データ**: rosbag / sim から goal・障害物・gap 配置を広く集めて学習（[training.md](training.md)）。CNN/Transformer encoder の増強。
2. **閉ループ学習（DAgger 等）**: 実行時に訪れる状態でラベルを足し、分布シフトに頑健化。obstacle-threading に直接効く。
3. **ハイブリッド**: generative が**粗い waypoint / 多峰の概形**を提案し、classical（探索 or 最適化）が**精緻化・検証**する。完全性は classical、提案の多様性は generative が担う。本リポジトリの「propose / dispose」をさらに一段深める現実解。
4. **TensorRT / Jetson 実機検証**: [deployment.md](deployment.md) §11 / [roadmap.md](roadmap.md)。

## 結論

小型・合成学習モデルでも **side-selection と open goal 到達は実機構で動く**（v0.6.0 出荷）。**gap-routing と obstacle-threading は classical が勝つ探索/分布シフトの領域**で、現状モデルの天井。この境界を正直に測れること自体が「generative propose / classical dispose」設計の価値であり、両者を同一リポジトリ・同一土俵に載せている理由である。
