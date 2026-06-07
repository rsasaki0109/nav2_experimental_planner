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
| Mode B: **off-centre gap の方向検出（スロットを狙う）** | ⚠️ flow/recurrent（CNN）は不可 → ✅ **transformer（attention）は raw 提案でスロット方向を向く** | 下記「追記」参照（A/B + C++ 方向テスト） |
| Mode B: **off-centre gap を実際に通る（footprint 検証 benchmark）** | ✅ **transformer + footprint-aware 損失で純生成貫通**（flow・recurrent は不可）。当初は off-centre 特化で直進の隙間を取りこぼすトレードオフがあったが、**容量増（dim64/h8/l3）+ centred tri-mix で解消**＝off-centre と dead-ahead（centred/narrow/double）を同時に通す／ far off-centre と slalom は残る bound | 下記「天井突破」「多コース評価 / 容量増で解消」参照（実 C++ benchmark で検証） |
| Mode A: **障害物のスレッディング（回り込み通過）** | ❌ 学習単体では天井 → ✅ **ハイブリッドで解決** | 下記参照 |

要点: **side-selection と open goal 到達は小型モデルでも実機構で動く**。**off-centre gap（壁のスロット貫通）は、当初は天井だったが、token attention で aim できる transformer に footprint-aware 損失を足すと pure-generative で貫通できるようになった**（後述、実 C++ benchmark で検証）。さらに **「天井」とされた *slalom*（S 字二段壁）と *far off-centre gap* は、実はアーキの限界ではなく 2 つのデータバグ（壁を擦る教師＋train/inference の patch 不一致）だった** — collision-clean な台形教師と deployment 一致の patch で直すと、no-fan の **attnseq ファミリが本ベンチ全 8 コースを純生成で貫通（8/8）**する（後述、実 C++ benchmark で検証、`diffusion_global_costmap_attnseq_v0` 出荷）。残る本質的天井は **Mode A の obstacle-threading（閉ループ分布シフト）**で、ここは **classical search / reactive 法が本来勝つ領域**。そして **ハイブリッド**（generative 提案 + classical fallback）は**任意地図での完全性保証**を与える（本ベンチのカタログ・コースは純生成で足りるが、out-of-distribution な地図は hybrid が担保）。

## 何が効くか（v0.6.0 で出荷済み）

- **costmap 条件付き側選択**: 片側に障害物がある egocentric / goal-aligned patch を入力すると、K 個の提案すべてが空き側へ寄る。解析的 placeholder（`FanPathModel` / `FanRolloutModel`、地図を見ず対称 bow）には出せない、本物の学習挙動。`nav2_diffusion_onnx` の end-to-end gtest（`CuratedZooModelVeersAwayFromObstacle`）で出荷バイナリを直接検証。
- **clear のバイアス除去**: 障害物なしでは中央（直進）。学習データに mirror ペア + clear サンプルを入れて左右バイアスを消した。
- **Mode A の open 完走**: expert を **pure-pursuit 弧（carrot へ向けて曲がる）**に再設計し、carrot 方位を多様化したことで、閉ループの横ズレを能動補正でき、open シナリオで goal 到達（`controller_comparison.md` の *Mode A, learned* 行）。
- **生成ファミリの拡充（同一封筒の peer）**: Mode A の seam 上に flow / diffusion / consistency に加え **transformer**（DETR 風 set-prediction）と **recurrent**（GRU 自己回帰ロールアウト）の計5系統を実装・出荷した（`diffusion_local_costmap_transformer_v0` / `diffusion_local_costmap_recurrent_v0`、C++ curated-zoo テスト + `controller_benchmark` 行付き）。いずれも costmap 側選択・clear バイアス除去・前進を満たす本物の学習挙動だが、**競合範囲（competence envelope）は flow と同じ**で、下記の天井（Mode A 障害物スレッディング・Mode B 検証付き gap）を変えるものではない。意義は「同じ契約で帰納バイアスの異なる proposer を比較できる」点であり、天井突破は依然データ/容量/hybrid の問題。
- **Mode B も同様にファミリ拡充**: Mode B（`PathModel`）の seam 上に flow / transformer に加え **recurrent**（GRU 自己回帰ロールアウト）を出荷し計3系統にした（`diffusion_global_costmap_recurrent_v0`、C++ `CuratedZooPathRecurrentVeersAwayFromObstacle` + `planner_benchmark` 行付き）。recurrent は flow と**同じ 16 次元 CNN embedding** に条件付けするため、競合範囲は **flow と同等の peer**（*clear* / *side obstacle* を解き、*off-centre gap* / *slalom* は *no path*）。transformer のような off-centre slot へのエイムは持たない（token attention が必要）。同一ファミリ（逐次バイアス）を Mode A・Mode B 双方の契約で運べることの実証であり、天井は不変。コストは H=12 逐次ロールアウトで Mode B 3 系統中レイテンシ最大。

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

aim は出るが、これだけでは benchmark の gap は通らなかった（履歴・負の結果）:

- raw 提案はスロットを「狙う」が、**候補の壁横断位置が再サンプル patch でドリフト**し、幅 1 m スロットの外（壁）を擦って `DiffusionGlobalPlanner` は *no path*。
- **候補多様性ファン**（K 候補を expert 周りの横 ±0.4 m fan に）で side obstacle は解ける peer に回復したが、gap は未貫通。
- **直進 plateau expert**（slot_y で平坦に壁帯通過）も raw aim は正しいが benchmark gap は *no path*。
- → 当時の結論は「aim・ファン・plateau の純 imitation 3手では幅 1 m 検証スロットは pure-generative では貫通できない、hybrid が解く」。残された future-work レバーとして **「footprint を陽に学習に入れる（提案を validator が通す形に最適化する）」** を明記していた。

#### 天井突破: footprint-aware 損失で純生成が検証付き gap を貫通（2026-06、実 C++ benchmark 検証）

その future-work レバーを実装した。`path_planners._footprint_penalty` は、**costmap パッチをガウシアンでぼかした近接場**を経路に沿って密補間サンプルする**微分可能 footprint クリアランス損失**で、fan-recon に加えて学習する（`train_and_export_costmap_path(..., footprint=, blur_sigma=)`）。狙いは「expert 形状の模倣」から「**validator が通す提案への最適化**」への転換:

- **ぼかし場**: 二値占有のままだと壁内部で勾配ゼロ（中央に取り残された waypoint がスロット方向へ動けない）。ガウシアンぼかしでスロットに谷を持つ滑らかな場にすると、内部からでもフリー回廊への引力が効く。
- **密補間**: H=12 の waypoint がどこに落ちても、`isPathValid` 同様に**壁横断点**をサンプルするため、横断の擦りを直接罰せる。

この transformer（attention で aim + footprint-aware で精緻化）を `dataset='both'` で学習すると、**実 C++ `planner_benchmark` の *off-centre gap* を純生成で貫通**する（`Diffusion (Mode B, transformer)`: *off-centre gap* = **yes / ~5.5 m / 12 pose / ~0.2 ms**、fallback なし）。**Mode B で classical fallback なしに検証付き gap を通した初のモデル**。flow / recurrent（16 次元 CNN embedding）は依然 *no path* で、この差が「aim できる表現 × validator-aware 学習」の効果を示す。

**正直なスコープ（不変の天井もある）**:

- **slalom は依然 *no path***: 二段の食い違い壁を S 字で抜けるには 2 回横断が要り、単一前進横断の純生成提案では届かない。**slalom の完全解は引き続き hybrid**。
- GPU 学習は run 間で bit-exact でなく、どの候補が通るかは多少ぶれるが、**出荷アーティファクトが gap を通すことを実 benchmark で検証済み**（manifest checksum 固定）。
- 完全性保証（任意地図で必ず解を返す）は依然 hybrid のみ。pure-generative は fail-closed のまま。

> 出荷: `diffusion_global_costmap_transformer_v0`（footprint-aware 再学習）を model_zoo に更新し、benchmark の **Diffusion (Mode B, transformer)** 行も *off-centre gap* = yes に更新。raw 方向の C++ テスト（`CuratedZooTransformerAimsAtOffCentreSlot`）は継続 pass。slalom と完全性は hybrid が担保。

#### 多コース評価で判明したトレードオフ（2026-06、8 コースに拡張）

`planner_comparison.md` を 4 → 8 コースに拡張し（*centred gap* / *narrow gap* / *far off-centre gap* / *double gate* を追加）、3 生成ファミリの競合範囲を**正直に切り分けた**。そこで小容量 transformer（dim 32 / 4 heads / 2 blocks）に**トレードオフ**が見つかった: off-centre 特化で **off-centre / far** は通すが、**直線上の隙間（centred / narrow）を取りこぼす**（flow/recurrent は逆に直進に強く off-axis に弱い）。

**容量増でトレードオフを解消した（2026-06、検証済み）。** future work とした「centred サンプル追加で両立」を段階的に検証した:

| 段階 | centred/narrow | off-centre | far | double |
|---|:-:|:-:|:-:|:-:|
| 小容量・2-way（side+off-centre） | ❌ | ✅ | ✅ | ✅ |
| 小容量・centred 三分割 | ✅獲得 | ❌喪失 | ✅ | ❌喪失 |
| 小容量・off-centre 厚め | ❌ | ❌ | ✅ | — |
| **大容量（dim64/h8/l3）・centred 三分割（出荷）** | **✅** | **✅** | ❌ | **✅** |

小容量では centred を足すと**目玉の off-centre を失う**＝データ配合では解けない**容量限界**だった。しかし **transformer を dim 64 / 8 heads / 3 blocks に増強**し centred を tri-mix すると、**off-centre と dead-ahead（centred/narrow/double）を同時に通す**——トレードオフが**解消**した（実 C++ `planner_benchmark` で検証、出荷モデル）。**残る bound は *far off-centre gap***（同じ off-axis スロットを前方約 3 m に押したもの）で、大容量モデルはこれを通さない＝容量は「種類の幅」を買うが far-forward 変種までは届かない。GPU 学習は run 間非決定で off-centre / far は borderline だが、**出荷アーティファクトが off-centre + dead-ahead を通すことは実 benchmark で検証済み**（checksum 固定）。

更新後の 4 ファミリ競合範囲（実 C++ `planner_benchmark`、純生成・fallback なし）:

| コース | flow | transformer | recurrent | **attnseq** | 解釈 |
|---|:-:|:-:|:-:|:-:|---|
| *clear* / *side obstacle* | ✅ | ✅ | ✅ | ✅ | 全員 |
| *centred gap* / *narrow gap* / *double gate*（直進の隙間） | ✅ | ✅ | ✅ | ✅ | 全員 |
| *off-centre gap* | ❌ | ✅ | ❌ | ✅ | transformer / attnseq（attention で aim） |
| *far off-centre gap* | ❌ | ❌ | ❌ | **✅** | **attnseq のみ** |
| *slalom*（S 字二段壁） | ❌ | ❌ | ❌ | **✅** | **attnseq のみ** |

transformer は off-axis も dead-ahead も通す proposer（6/8）。そして **attnseq ファミリは全 8 コースを純生成で貫通（8/8）** — 長く「天井」とされた *far off-centre* と *slalom* を含む。これは下記の通り**アーキの天井ではなくデータバグ**だった。

#### slalom と far off-centre は「アーキの天井」ではなく **データバグ**だった（2026-06 解決）

長い間 *slalom*（二段互い違い壁の S 字・2 回横断）と *far off-centre gap* は純生成の天井とされ、
3 系統のアーキ（容量増 transformer・MLP head・初期 attnseq）が**同じ学習 loss プラトー（fp=0 で ~0.14）に
収束**したため「アーキの問題」と結論していた。**これは誤診だった。** C++ validator を Python で忠実に
再現して掘ったところ、原因は**2 つのデータバグ**で、直すと no-fan の attnseq が **8/8** で貫通した。

**バグ① 教師経路が壁を擦っていた。** 旧 slalom expert は二山**ガウシアン**で、slot offset に達するのが
各壁バンドの**中心の一瞬だけ**。壁は forward 方向に厚みを持つので、バンド前縁・後縁では path がまだ壁の中
を通っていた＝**教師自体が衝突経路**（footprint validator 上で max occupancy 0.5〜1.0）。何で学習しても
壁を擦る経路に収束し、loss も下がりきらない（0.14 の床）。
→ **修正**: 各壁バンド全域で slot offset を**保持する台形（plateau）S**（`_plateau_track`、max occupancy ~0）。

**バグ② 学習 patch と推論 patch が食い違っていた。** 学習は手書きの `_gap_patch`（`int(x_lo·S/fwd)` で
行を切る）、推論は C++ `OnnxPathModel::alignedPatch`（各セル中心 `fwd*(row+0.5)/S` で実コストマップを
サンプル）。薄い壁では**まる 1 行ズレる**ことがあり、モデルは手製 patch では S 字を出すのに、実再サンプル
patch では直進に退化していた。
→ **修正**: 学習 patch も**同じ細グリッド markWall + resample** で生成（`_resampled_aligned_patch`）。
これで train と inference の patch 分布が一致。

**残りは no-fan アーキが担う。** transformer / recurrent は K 候補を expert ± **横一律オフセットの fan**
で出すため S 字を表現できない（横にずらすと両クロッシングが同時に隙間を外す）。`CostmapPathAttnSeqPlanner`
は fan を廃し、K 個の学習 seed ＋ **per-step クロスアテンションで costmap memory を参照する逐次 GRU
decoder** で、候補が**自由に S 字を取れる**。

**結果（実 C++ `planner_benchmark`、純生成・fallback なし）**: 上記 2 修正済みデータの **5-way mix
（side + off-centre/far gap + centred/narrow gap + double gate + slalom、`dataset='all'`）**で attnseq を
学習すると、**全 8 コースを貫通（8/8、すべて 12-pose のモデル経路）** — slalom（~9.9m）も far off-centre
gap（~6.7m）も含む。v0.11.0 transformer（6/8、slalom・far off-centre は no-path）を**厳密に上回る**。
出荷: `diffusion_global_costmap_attnseq_v0`（model_zoo、4 つ目の learned Mode B family）。

**教訓**: 「3 アーキが同じプラトー」は強力に**アーキ天井**を示唆したが、真因は**到達不能（衝突する）教師を
fit させていた**ことと**train/inference の入力分布ズレ**だった。collision-clean な教師と deployment 一致の
入力を用意すれば、純生成提案は本ベンチの全コースを通す。hybrid は引き続き**任意地図の完全性**を担保するが、
本ベンチのカタログ・コースに関しては純生成だけで足りる。

#### 運動学条件付き(kinematics-conditioned)プランナ：1 モデルで多車種（全8コース＋omni、2026-06）

seam の**空き context スロット**（`context = [goal_distance, 0]` の 2 つ目）に車両の**最小回転半径 R**
（`R=0` は omni）を入れると、**1 モデルで複数の操舵型**に対応できる。`make_costmap_path_kinematics_dataset`
は各回り込みの横ずれ幅を `w=√(2·off·R)` にして**ピーク曲率 ~ 1/R** に整形する → omni / 小さい R（差動二輪、
その場旋回可）は鋭い回り込み、大きい R（Ackermann/車）は緩い回り込みを、**同じ gap**に対して提案する。学習は
**全8コース**を含む（clear/centred/double-gate は直進・R 不変、off-centre/far gap と side は R 整形、slalom は
**omni 専用ブロック**）。ベンチ slalom は ±2m の交差を ~1.6m の隙間で抜くため必要曲率 ~1/0.02m と車輪型の旋回円を
遥かに超える＝omni でしか成立しない。

そして `DiffusionGlobalPlanner` に `min_turn_radius` パラメータと **曲率 validator**（`isPathValid` 内、
連続 3 点の Menger 外接円半径で曲率を測り 1/R 超を棄却）を追加した。**propose/dispose を footprint から
車両ダイナミクスへ拡張**したもの: モデルが車種別に提案し、決定論層が運動学的に不可能な経路を捨てる。

実 C++ benchmark（純生成、同一重み・R 入力と曲率ゲートのみ差し替え）:

| コース | omni (R=0) | diff (R=0.3) | Ackermann (R=1.5) |
|---|:-:|:-:|:-:|
| clear / centred gap / double gate | ✅ | ✅ | ✅ |
| narrow gap | ✅ | ✅ | ❌ |
| off-centre / far off-centre gap | ✅ | ✅ | ❌ |
| side obstacle | ✅ | ✅ | ❌ |
| slalom | ✅ | ✅ | ❌ |
| **合計** | **8/8** | **8/8** | **3/8** |

omni（gate off）は slalom 含む全コース貫通。diff（gate 1/R=3.33）も全コース貫通——鋭い旋回円で横移動を全てこなし、
ベンチ slalom の粗い 12 点 S も gate を下回る。Ackermann（gate 1/R=0.67）は**ほぼ直進のコースのみ**通り、横移動が要る
コース——narrow gap（狭スロットへの照準補正 κ~1.5）、off-centre/far gap（前方 ~1m で 2m 横ジョグ）、side（κ~2.2 の回避弓）、
slalom——を曲率 validator が全て棄却（1.5m の旋回円を超える）。出力ピーク曲率は指令 R に**単調**（exported ONNX で実測）。
出荷: `diffusion_global_costmap_kinematics_v0`（全8コース＋omni に再学習）。gtest `CuratedZooKinematicsConditionsOnTurnRadius`
（R 条件付け）と `CuratedZooKinematicsOmniProposesSlalomSCurve`（omni の S 提案）が保証。曲率ゲート無しで全8コースを 1 モデル
で通すのは [[attnseq-8of8-slalom-solved]] の attnseq。

### Mode A: 障害物スレッディング（回り込み通過）

learned Mode A は open では goal 到達するが、`controller_benchmark` の *frontal / side / corridor* では障害物が egocentric patch に入った時点で **drift → 安全候補なし → 安全停止**（障害物の手前、衝突なし）。これは:

- **閉ループの分布シフト**: 提案 → 実行 → 再観測の系で小さな誤差が累積し、モデルが学習分布の外の状態（障害物が近い + carrot が斜め）に入ると出力が崩れる。
- **context 感度**: モデルは context（goal_x/y, max_angular など）の値に敏感で、benchmark のパラメータを学習分布に合わせる必要があった（lookahead/限界の調整で改善するが脆い）。

単発の Mode B gap は footprint-aware 損失で貫通できたが、それは**1 回の前進横断で済む単発計画**だから。閉ループで誤差が累積する obstacle-threading（再観測のたびに分布外へ）はより強い天井で、同じ手は効かない。安全層（kinematic + footprint）が**衝突は常に防ぐ**点は意図どおり。

#### 天井の精密診断：原因は「モデル容量」ではなく「教師」と「安全ゲートの粒度」（2026-06）

Mode B の attnseq で効いた手（高容量 + token attention + DAgger）を Mode A に当てて切り分けた結果、天井の正体は**2 つの具体的・再現可能なメカニズム**であり、いずれも**学習モデルの容量・模倣精度ではない**ことが判明した（`dagger.py` の numpy 閉ループシムで再現、実 C++ controller の検証ロジックと一致）。

1. **DAgger の oracle 自身が衝突している**。現行の手書き expert（`_expert_trajectory`）は回避バウが**ピーク 0.20m・各リプランでリセット**、かつ carrot が常に直線参照上にあるため、閉ループでは pure-pursuit が直線追従を支配して**障害物に直進衝突**する（expert 単独の閉ループ＝**1/4**、open のみ到達。side/two はブロック手前で衝突）。DAgger は訪問状態で oracle にラベルを問うので、**衝突する教師を集約**してしまう。これは [[attnseq-8of8-slalom-solved]] で見つけた「教師が壁を擦る」と同型。修正 oracle（自由側へずらした carrot へ pure-pursuit、曲率ベースで回頭・オフセットを持続）に差し替えると **expert 単独で 4/4 到達・無衝突**になる。

2. **だが修正 oracle でも gated 閉ループは 1/4 のまま**。高容量 transformer（costmap token への cross-attention）が修正 oracle を **loss~0 で完全 fit** しても閉ループ成功は変わらない。原因は**デプロイの安全ゲートの粒度**: `_select`／実 C++ `FootprintCollisionFilter::check` は候補軌道の **H ステップ全点**を footprint チェックし、**1 点でも衝突すれば hard reject** する。ところがタイトな skirt 回避は、**ステップ実行では安全に脇を抜ける**（robot は y≈2.85 を通過、ブロックは y≥3.0）のに、**1m 先までの lookahead 弧の先端がブロックに被る**ため毎ステップ「安全候補なし」と判定され停止する。つまり「再観測のたびに分布外へ」だけでなく、**全ホライズン hard-reject の安全検証がタイトな反応的回避と非両立**——これが「障害物が patch に入った瞬間に安全停止」の機構的な正体である。

**含意**: obstacle-threading を**純生成**で通すには、データ・モデルを増やすより、**安全ゲート/選択を「実際に実行する近傍ウィンドウ」基準に変える**（毎サイクル costmap を再評価する前提で、全ホライズンの hard-reject ではなく実行区間の検証にする）必要がある。

#### 両修正を実装して部分的に突破（side obstacle を純生成で貫通、2026-06）

上の 2 機構を実際に直した:

- **窓化 footprint ゲート**: `DiffusionController` に `safety_check_points` パラメータを追加（既定 0＝従来の全ホライズン hard-reject で後方互換）。`>0` で候補軌道の先頭 N 点（実際に実行する区間）だけ footprint 検証する——receding-horizon。kinematic ゲートは常に全軌道を見る。
- **修正 reactive oracle**: `dagger.py` の衝突する手書き expert を、自由側へ持続オフセットする曲率ベースの dodge（`_dodge_offset` + 近い offset carrot への pure-pursuit）に置換。
- **高容量モデル**: `dagger_train_costmap_transformer` が costmap-token transformer を DAgger 学習（小容量 flow は鋭い dodge を fit できず 1/4 のまま＝容量が効く）。

結果:
- **DAgger 閉ループ sim**: 窓化ゲート + 修正 oracle + transformer で **1/4 → 4/4（後に frontal 追加で 5 シナリオ中 4/5）**。
- **実 C++ `controller_benchmark`（`safety_check_points=3`, fallback 無し）**: `diffusion_local_costmap_threading_v0` が **side obstacle を純生成で貫通（4.27m 完走）**——**本リポジトリで学習 Mode A が初めて障害物を生成的に通過**（learned/transformer/recurrent は全て ~1.0m で timeout）。dead-ahead 中央の **frontal** は最も前進（1.69m / clearance 0.20m）するが未完、**corridor** も timeout（dodge oracle は通路センタリングをしない／実コストマップの inflation で dead-ahead が想定より太い）。sim の 4/5 は実機ベンチに**完全には転移しない**。

**正直な現状**: 「閉ループ分布シフト＋安全ゲート粒度」という天井は機構的に同定し、**side obstacle については純生成で突破**した（実 C++ ベンチで検証）。frontal/corridor は未解決で、`safety/kinematic` が**衝突を常に防ぐ**現設計の下、**ハイブリッド（VFH+ fallback）が全シナリオ到達**を保証する点は変わらない（[[kinematics-conditioned-planner]] の propose/dispose と対称、[[mode-a-threading-ceiling-diagnosed]]）。

**ただしハイブリッドで解決済み（✅）**: `DiffusionController` の `fallback_controller_plugin`（既存）に classical の reactive controller（VFH+ 等）を設定すると、安全候補が無いとき停止する代わりに委譲する。`controller_comparison.md` の **Diffusion (Mode A, hybrid)** 行は **全シナリオで goal 到達**（open は learned、障害物は VFH+ fallback が回避。corridor 行は VFH+ と完全一致 = fallback 稼働の証拠）。Mode B planner の hybrid と完全に対称。

## なぜ天井になるか（要因）

1. **小型 CNN/MLP の容量**: encoder 16 次元・MLP 128 隠れ。薄い・部分的な障害物信号や gap 位置の汎化に弱い。
2. **合成 → 実 costmap の転移**: 学習 patch と C++ 再サンプル patch の微差（壁厚・gap 幅・行位置）で判断が反転。
3. **純 imitation の限界（→ validator-aware で部分的に克服）**: expert 形状の模倣だけでは検証付き gap を安定して通せない（提案が再サンプル patch で擦る）。**footprint-aware 損失で「validator が通す提案」へ直接最適化**すると単発 gap は貫通する。ただし完全性保証ではない（学習 span 外の壁距離・slalom は依然 no-path）。
4. **閉ループの分布シフト**: open は安定だが、障害物近傍は訪問状態が学習分布から外れる。単発計画と違い、validator-aware 学習だけでは閉ループの累積誤差は塞げない。

## 天井を超える道筋（future work）

1. **ハイブリッド（✅ 実装済み・2 形態）**: generative 提案 + classical の完全性。本リポジトリは 2 つの結合度を実装している。
   - **疎結合（fallback）**: `DiffusionGlobalPlanner` の `fallback_planner_plugin`（Mode A controller の `fallback_controller_plugin` と同型）。learned 提案が無効なときだけ classical search/reactive に委譲。簡単な地図は高速な generative 経路、難所は探索系。`planner_comparison.md` の **Mode B, hybrid**（learned + JPS）は全シナリオ解決（clear/side は generative 12 pose、gap/slalom は JPS 81/107 pose）。`controller_comparison.md` の **Mode A, hybrid**（learned + VFH+）も全シナリオ到達。
   - **密結合（guided）**: `hybrid_mode: guided`。**常に**完全な A* を走らせ、有効な proposal の近傍セルのコストを割引くので、learned が**毎回**の経路形状を誘導しつつ探索が完全性を保証する。**Mode B, guided** も全シナリオ解決（cell 解像度 81/111 pose — fallback と違い簡単な地図でも探索を走らせる）。
2. **閉ループ学習（DAgger）（✅ 基盤実装済み・効果は限定的）**: `nav2_diffusion_training/dagger.py`。現方策を numpy costmap シム（C++ の egocentric crop / first-segment 抽出 / lookahead / dt を踏襲）でロールアウトし、訪れた状態で expert（carrot への pure-pursuit + costmap 回避）にラベルを問い、集約して再学習する。**分布シフトに対する正しい道具**で、ループは end-to-end で動く（pytest 付き）。ただし現状の **小型モデル + 軽量シム**では閉ループ成功は marginal（0/4 → 1/4、集約が増えると loss も増えて fit しきれない）。**忠実な改善には大容量モデルと、より忠実な閉ループ（実 C++/Gazebo ロールアウト）が要る**——下の項目と組み合わせるのが筋。
3. **大容量モデル + 多様な実データ**: rosbag / sim から goal・障害物・gap 配置を広く集めて学習（[training.md](training.md)）。CNN/Transformer encoder の増強。DAgger と組み合わせると効く見込み。
4. **TensorRT / Jetson 実機検証**: [deployment.md](deployment.md) §11 / [roadmap.md](roadmap.md)。

## 結論

小型・合成学習モデルでも **side-selection と open goal 到達は実機構で動く**（v0.6.0 出荷）。さらに **off-centre gap（壁のスロット貫通）は、token attention で aim できる transformer に footprint-aware（validator-aware）損失を足すと pure-generative で貫通できる**ことを実 C++ benchmark で示した（flow / recurrent は不可）。そして **「天井」とされた *slalom* と *far off-centre gap* は、3 アーキが同じ loss プラトーに収束したことから当初アーキ限界と誤診したが、実は 2 つのデータバグ（壁を擦る台形未満の教師＋train/inference の patch サンプリング不一致）だった** — collision-clean な台形教師と deployment 一致の patch で直すと、no-fan の **attnseq ファミリが本ベンチ全 8 コースを純生成で貫通（8/8）**する（実 C++ benchmark で検証）。これは「提案ステージの限界は**表現・ロス・そしてデータ分布の整合**の問題で、必ずしも本質的天井ではない」ことの最も強い実証である。残る学習単体の天井は **Mode A の obstacle-threading（閉ループ分布シフト）**で、classical が勝つ領域。そして **ハイブリッド**は**任意地図での完全性を保証**する: Mode B planner は `fallback_planner_plugin`（learned 提案 → classical search）、Mode A controller は `fallback_controller_plugin`（learned → classical reactive）で、いずれも**全シナリオを解く**。境界を正直に測り、学習で押し上げられる所は押し上げ、残りは classical と組んで完全性を取る——これが「generative propose / classical dispose」設計の価値であり、両者を同一リポジトリ・同一土俵に載せている理由である。
