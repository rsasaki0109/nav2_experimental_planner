# 生成モデル比較（オフライン leaderboard）

> 自動生成: `python3 tools/benchmark_models.py`。関連: [benchmarking.md](benchmarking.md)、[model_zoo.md](model_zoo.md)。

10の生成プランナ構成（flow / diffusion / consistency / transformer / recurrent × context / costmap+goal）を**同一の合成回避シナリオ**で評価し、各モデルが提案する K 候補軌道の品質を比較する。シナリオは前方を塞ぐ障害物と片側 gap（gap-right / gap-left）＋ clear。直進すると衝突し、gap 側へ回避した候補のみクリアできる。これは CPU・決定論的な**提案品質**の比較であり、シミュレータ走行ベンチ（`benchmark_runner`）とは別物。

スコアは安全優先の重み付け（safety 0.5 / progress 0.3 / smoothness 0.2）。safety は **success**（安全な候補が1つ以上存在する割合、0.7）＋ clearance（選択候補の余裕、0.3）。progress / smoothness はモデル間で min-max 正規化（高いほど良い）。**success が安全層にとって最重要**: 候補を1つでも安全に出せれば controller がそれを選べる。

| # | Model | Family | Conditioning | Steps | Success | Clearance[m] | Progress | Turning[rad] | CollRate | Score |
|---|---|---|---|--:|--:|--:|--:|--:|--:|--:|
| 1 | `costmap-recurrent` | recurrent (GRU rollout) | costmap+goal | 10 | 1.00 | 0.544 | 0.593 | 0.106 | 0.00 | **0.896** |
| 2 | `costmap-consistency` | consistency (1-step) | costmap+goal | 1 | 1.00 | 0.543 | 0.564 | 0.094 | 0.00 | **0.883** |
| 3 | `costmap-transformer` | transformer (set-pred) | costmap+goal | 1 | 1.00 | 0.544 | 0.529 | 0.126 | 0.00 | **0.872** |
| 4 | `costmap-flow` | flow matching | costmap+goal | 2 | 0.67 | 0.537 | 0.588 | 0.593 | 0.56 | **0.710** |
| 5 | `recurrent` | recurrent (GRU rollout) | context | 10 | 0.33 | 0.514 | 0.856 | 0.002 | 0.67 | **0.636** |
| 6 | `transformer` | transformer (set-pred) | context | 1 | 0.33 | 0.513 | 0.859 | 0.010 | 0.67 | **0.632** |
| 7 | `consistency` | consistency (1-step) | context | 1 | 0.33 | 0.512 | 0.838 | 0.059 | 0.67 | **0.619** |
| 8 | `costmap-diffusion` | diffusion (DDIM) | costmap+goal | 4 | 0.33 | 0.509 | 0.612 | 0.935 | 0.67 | **0.458** |
| 9 | `diffusion` | diffusion (DDIM) | context | 4 | 0.33 | 0.525 | 0.294 | 0.688 | 0.67 | **0.428** |
| 10 | `flow` | flow matching | context | 1 | 0.67 | 0.525 | 0.027 | 2.660 | 0.56 | **0.302** |

## 読み方

- **Conditioning**: `context` モデルは costmap を見ないため、左右対称な学習分布では平均（直進）しか学べず、前方障害物に突っ込んで success が落ちる。`costmap+goal` モデルは egocentric パッチを読み gap 側へ回避できる＝ costmap 条件付けの価値が success に直接出る。
- **Success**: 安全な候補が1つ以上存在した割合。propose/dispose/select 構成では、衝突候補が混じっても安全層が落とすため、「最低1つ安全な高 progress 候補を出せるか」が実運用上の安全成否。
- **Steps**: 推論ステップ数（レイテンシの代理指標）。consistency=1, transformer=1, flow=1〜2, diffusion=4, recurrent=10（自己回帰ロールアウトは horizon 分の逐次ステップ）。1ステップでsuccess を取れる consistency / transformer が edge-GPU 向きに有利。
- **CollRate**: 全 K 候補のうち衝突した割合（参考値）。多峰性のため高くても success が満たされれば許容される。
- 合成データ・CPU 評価であり、実機/実 sim の数値ではない（[risks.md](risks.md)）。学習・評価データは harness 内 `make_bench_dataset`（前方障害物＋片側 gap を回避する expert）。

## まとめ

- 上位は **costmap 条件付き**モデルが占める。costmap を読めるモデルだけがgap 側を選んで安全候補を出せる＝条件付けの価値が success に直結する。
- **costmap-recurrent**（GRU 自己回帰ロールアウト）が衝突0・success 1.00 で総合首位だが、逐次生成で **10 ステップ**とレイテンシは最大。**costmap-consistency**（1ステップ蒸留）と **costmap-transformer**（DETR 風 set-prediction）は衝突0・success 1.00 を**1フォワード**で達成し、edge-GPU では依然有利。新規の recurrent / transformer ファミリが反復サンプリング系と同等以上の安全候補品質を出せることを示す（context-only でも recurrent / transformer が context 勢の最上位）。
- これは**固定 seed の単一試行・toy モデル**による例示であり、絶対順位はseed / epoch / 推論ステップ数に依存する。手法選定の確定指標ではなく、再現可能な比較 harness と相対傾向のデモとして読むこと。
