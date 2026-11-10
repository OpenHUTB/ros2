# -*- coding: utf-8 -*-
"""test_pipeline.py —— 用极小规模跑通「采集 → 训练 → DAgger → 评估」四步。

为什么值得跑：这四步是文档里「launch 一键跑」的内容，但每一步都靠 CWD 里的
约定文件名（demo.npz → mlp.npz/Xstat.npy → eval_summary.csv）串起来，
任何一处改名或路径写死，都只有真跑一遍才会暴露。这里把规模压到秒级，
CI 才有条件每 push 都验一次。
"""
import csv
import math
import os

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco', reason='需要 mujoco 才能跑流水线冒烟')

from reacher_track import collect_demo, dagger_iter, deploy_eval, train_mlp  # noqa: E402

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_package_layout_for_launch():
    """launch 串起来的脚本与数据文件必须都在（改名即红）。"""
    for rel in ('reacher_track/collect_demo.py',
                'reacher_track/train_mlp.py',
                'reacher_track/dagger_iter.py',
                'reacher_track/deploy_eval.py',
                'launch/main.launch.py',
                'models/reacher.xml'):
        assert os.path.exists(os.path.join(PKG_ROOT, rel)), '缺少 %s' % rel


def test_mini_pipeline_end_to_end(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    np.random.seed(0)

    # 1) 采集：3 回合 × 最多 40 步
    collect_demo.main(n_eps=3, max_steps=40, seed=0)
    demo = np.load('demo.npz')
    X, Y = demo['X'], demo['Y']
    assert X.ndim == 2 and X.shape[1] == 6
    assert Y.shape == (len(X), 2)
    assert len(X) > 0

    # 2) 训练：3 个 epoch 足够验证「损失在降 + 存档齐全」
    _, losses = train_mlp.main(epochs=3, batch=16, lr=3e-3, seed=0)
    assert losses[-1] < losses[0], '训练损失没有下降：%r' % (losses,)
    for f in ('mlp.npz', 'Xstat.npy', 'Ystat.npy', 'losses.npy'):
        assert os.path.exists(f), '缺少 %s' % f

    # 3) DAgger：1 轮、2 回合、2 epoch
    dagger_iter.main(iter_n=1, n_eps=2, max_steps=20, epochs=2, lr=3e-3)
    assert os.path.exists('mlp.npz')

    # 4) 评估：2 回合，要落 eval_summary.csv 且两行都是有限数
    deploy_eval.main(n_eps=2, max_steps=60, seed=0)
    assert os.path.exists('eval_summary.csv')
    with open('eval_summary.csv', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    assert [r['method'] for r in rows] == ['MLP', 'IKPD']
    for r in rows:
        for k in ('mean_err', 'median_err', 'success_rate', 'mean_steps'):
            v = float(r[k])
            assert math.isfinite(v), '%s.%s 不是有限数' % (r['method'], k)
    # 专家是演示数据的来源，小规模下也应当能到点；MLP 只训练了 3 epoch，不设阈值
    assert float(rows[1]['success_rate']) > 0.0, 'IK+PD 基线一回合都没成功'
