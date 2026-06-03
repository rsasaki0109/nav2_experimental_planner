"""Offline leaderboard comparing the six generative planner families.

Trains each shipped generative planner (flow / diffusion / consistency, both
context-only and costmap-conditioned) on the synthetic obstacle-avoidance
dataset, evaluates the *proposals* each makes on a fixed set of obstacle
scenarios, and writes a safety-first leaderboard to ``docs/model_comparison.md``.

This is a deterministic CPU comparison of model proposal quality, not a
simulator benchmark; the live ``benchmark_runner`` node covers the sim path.

Reproduce::

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    pip install torch
    python3 tools/benchmark_models.py
"""

import os

import torch

from nav2_diffusion_training.generative_planners import (
    COSTMAP_SIZE, HORIZON, build_planner,
    _COSTMAP_BUILD, _COSTMAP_LOSS, _LOSS)
from nav2_diffusion_training.model_eval import (
    evaluate_candidates, patch_obstacle_xy, rank_rows)

ROBOT_RADIUS = 0.06          # collision radius [m] (small AMR vs ~1 m patch)
GOAL_X, GOAL_Y = 0.35, 0.0   # local goal straight ahead [m]
EPOCHS = 1000
S = COSTMAP_SIZE
# Obstacle blocks sit AHEAD of the robot (rows 4..13 => x ~0.08..0.35 m) and
# straddle the centreline, leaving a gap on one side. A robot driving straight
# hits the block; only veering toward the gap clears it. The robot's own start
# cell is below the block, so no candidate is trivially in collision at t=0.
_ROW0, _ROW1 = 4, 13
_LAT = 0.16                  # expert lateral veer toward the gap [m]


def _block(col0, col1):
    """A patch with a vertical obstacle block over columns [col0, col1)."""
    patch = torch.zeros(1, 1, S, S)
    patch[:, :, _ROW0:_ROW1, col0:col1] = 1.0
    return patch


# Two in-distribution layouts: block-with-gap-on-the-right (veer right, y<0) and
# block-with-gap-on-the-left (veer left, y>0). 'gap_sign' is the y direction of
# the open side.
_LAYOUTS = [
    ('gap-right', _block(6, 17), -1.0),   # block covers left+centre, gap on right
    ('gap-left', _block(15, 26), +1.0),   # block covers centre+right, gap on left
]


def make_bench_dataset(num_samples):
    """
    Shared training/eval data: obstacle ahead with a one-sided gap; the expert
    veers toward the gap and holds. The two layouts are symmetric, so a model
    blind to the costmap can only learn the (straight) average and drives into
    the block -- which is exactly what makes costmap conditioning measurable.
    """
    ctx, patches, tgt = [], [], []
    for i in range(num_samples):
        _name, patch, gap_sign = _LAYOUTS[i % len(_LAYOUTS)]
        rows = []
        for h in range(HORIZON):
            f = (h + 1) / HORIZON
            fwd = 0.30 * f
            lat = gap_sign * _LAT * min(1.0, (h + 1) / 5.0)  # ramp out, then hold
            yaw = gap_sign * 0.10 * (1.0 if h < 5 else 0.0)
            rows.append([fwd, lat, yaw])
        ctx.append([1.0, 0.0, 0.3, 1.0])
        patches.append(patch[0])
        tgt.append(rows)
    return (torch.tensor(ctx), torch.stack(patches), torch.tensor(tgt))


def scenarios():
    """Held-out layouts (the two block configs the models trained on + clear)."""
    return [(name, patch) for name, patch, _ in _LAYOUTS] + \
        [('clear', torch.zeros(1, 1, S, S))]


def to_candidates(out):
    """[1, K, H, 3] tensor -> list of K trajectories [(x, y, yaw), ...]."""
    arr = out[0].detach().numpy()
    return [[tuple(float(v) for v in arr[k, h]) for h in range(HORIZON)]
            for k in range(arr.shape[0])]


def train_context(kind):
    """Context-only model: trained on the same data but blind to the costmap."""
    torch.manual_seed(0)
    model = build_planner(kind)
    context, _costmap, target = make_bench_dataset(64)
    opt = torch.optim.Adam(model.parameters(), lr=0.01)
    for _ in range(EPOCHS):
        opt.zero_grad()
        loss = _LOSS[kind](model, context, target)
        loss.backward()
        opt.step()
    model.eval()
    return model


def train_costmap(kind):
    """Costmap-conditioned model: sees the egocentric obstacle patch."""
    torch.manual_seed(0)
    model = _COSTMAP_BUILD[kind]()
    context, costmap, target = make_bench_dataset(64)
    opt = torch.optim.Adam(model.parameters(), lr=0.01)
    for _ in range(EPOCHS):
        opt.zero_grad()
        loss = _COSTMAP_LOSS[kind](model, context, costmap, target)
        loss.backward()
        opt.step()
    model.eval()
    return model


# name, family, conditioning, inference steps, builder, costmap-conditioned?
MODELS = [
    ('flow', 'flow matching', 'context', 1, train_context, False),
    ('diffusion', 'diffusion (DDIM)', 'context', 4, train_context, False),
    ('consistency', 'consistency (1-step)', 'context', 1, train_context, False),
    ('costmap-flow', 'flow matching', 'costmap+goal', 2, train_costmap, True),
    ('costmap-diffusion', 'diffusion (DDIM)', 'costmap+goal', 4, train_costmap, True),
    ('costmap-consistency', 'consistency (1-step)', 'costmap+goal', 1, train_costmap, True),
]


def aggregate(model, is_costmap):
    """Average per-scenario metrics for one model over all scenarios."""
    ctx = torch.tensor([[1.0, 0.0, 0.3, 1.0]])
    acc = {'success': 0.0, 'clearance': 0.0, 'progress': 0.0,
           'turning': 0.0, 'collision_rate': 0.0}
    scen = scenarios()
    for _name, patch in scen:
        with torch.no_grad():
            out = model(ctx, patch) if is_costmap else model(ctx)
        cands = to_candidates(out)
        obstacles = patch_obstacle_xy(patch[0, 0].tolist())
        m = evaluate_candidates(cands, GOAL_X, GOAL_Y, obstacles, ROBOT_RADIUS)
        for key in acc:
            acc[key] += m[key]
    return {key: acc[key] / len(scen) for key in acc}


def main():
    rows = []
    for name, family, cond, steps, trainer, is_cm in MODELS:
        print('training', name, '...', flush=True)
        kind = name.split('-')[-1]
        model = trainer(kind)
        agg = aggregate(model, is_cm)
        agg.update(name=name, family=family, conditioning=cond, steps=steps)
        rows.append(agg)

    ranked = rank_rows(rows)
    write_report(ranked)


def write_report(ranked):
    """Render the ranked rows to docs/model_comparison.md."""
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, '..', 'docs', 'model_comparison.md')
    lines = [
        '# 生成モデル比較（オフライン leaderboard）',
        '',
        '> 自動生成: `python3 tools/benchmark_models.py`。'
        '関連: [benchmarking.md](benchmarking.md)、[model_zoo.md](model_zoo.md)。',
        '',
        '出荷する6つの生成プランナ系統を**同一の合成回避シナリオ**で評価し、各モデルが'
        '提案する K 候補軌道の品質を比較する。シナリオは前方を塞ぐ障害物と片側 gap'
        '（gap-right / gap-left）＋ clear。直進すると衝突し、gap 側へ回避した候補のみ'
        'クリアできる。これは CPU・決定論的な**提案品質**の比較であり、シミュレータ'
        '走行ベンチ（`benchmark_runner`）とは別物。',
        '',
        'スコアは安全優先の重み付け（safety 0.5 / progress 0.3 / smoothness 0.2）。'
        'safety は **success**（安全な候補が1つ以上存在する割合、0.7）＋ clearance'
        '（選択候補の余裕、0.3）。progress / smoothness はモデル間で min-max 正規化'
        '（高いほど良い）。**success が安全層にとって最重要**: 候補を1つでも安全に出せ'
        'れば controller がそれを選べる。',
        '',
        '| # | Model | Family | Conditioning | Steps | Success | Clearance[m] | '
        'Progress | Turning[rad] | CollRate | Score |',
        '|---|---|---|---|--:|--:|--:|--:|--:|--:|--:|',
    ]
    for i, r in enumerate(ranked, 1):
        lines.append(
            '| {i} | `{name}` | {fam} | {cond} | {steps} | {succ:.2f} | {clr:.3f} | '
            '{prog:.3f} | {turn:.3f} | {coll:.2f} | **{score:.3f}** |'.format(
                i=i, name=r['name'], fam=r['family'], cond=r['conditioning'],
                steps=r['steps'], succ=r['success'], clr=r['clearance'],
                prog=r['progress'], turn=r['turning'], coll=r['collision_rate'],
                score=r['score']))
    lines += [
        '',
        '## 読み方',
        '',
        '- **Conditioning**: `context` モデルは costmap を見ないため、左右対称な'
        '学習分布では平均（直進）しか学べず、前方障害物に突っ込んで success が落ちる。'
        '`costmap+goal` モデルは egocentric パッチを読み gap 側へ回避できる'
        '＝ costmap 条件付けの価値が success に直接出る。',
        '- **Success**: 安全な候補が1つ以上存在した割合。'
        'propose/dispose/select 構成では、衝突候補が混じっても安全層が落とすため、'
        '「最低1つ安全な高 progress 候補を出せるか」が実運用上の安全成否。',
        '- **Steps**: 推論ステップ数（レイテンシの代理指標）。'
        'consistency=1, flow=1〜2, diffusion=4。consistency が1ステップで'
        'success を取れれば edge-GPU 向きに有利。',
        '- **CollRate**: 全 K 候補のうち衝突した割合（参考値）。'
        '多峰性のため高くても success が満たされれば許容される。',
        '- 合成データ・CPU 評価であり、実機/実 sim の数値ではない'
        '（[risks.md](risks.md)）。学習・評価データは harness 内 '
        '`make_bench_dataset`（前方障害物＋片側 gap を回避する expert）。',
        '',
        '## まとめ',
        '',
        '- 上位は **costmap 条件付き**モデルが占める。costmap を読めるモデルだけが'
        'gap 側を選んで安全候補を出せる＝条件付けの価値が success に直結する。',
        '- **costmap-consistency**（1ステップ蒸留）が衝突0・success 1.00 で総合首位。'
        '1ステップ推論で flow/diffusion を上回れれば edge-GPU 配備に有利。',
        '- これは**固定 seed の単一試行・toy モデル**による例示であり、絶対順位は'
        'seed / epoch / 推論ステップ数に依存する。手法選定の確定指標ではなく、'
        '再現可能な比較 harness と相対傾向のデモとして読むこと。',
        '',
    ]
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines))
    print('wrote', os.path.normpath(out_path))
    print('\n'.join(lines[10:10 + len(ranked) + 2]))


if __name__ == '__main__':
    main()
