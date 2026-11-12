# -*- coding: utf-8 -*-
"""评测：留一条件交叉验证、抗噪曲线、拒识（FAR/FRR）、混淆矩阵。"""

import numpy as np

import humanoid_kws_command.config as config
import humanoid_kws_command.corpus as corpus
from humanoid_kws_command.kws import MLP


def leave_one_condition_out(X, y, cond, epochs=config.EPOCHS):
    """按「变调/变速条件」留出做交叉验证，返回每折准确率。"""
    accs = []
    for held in sorted(set(cond)):
        tr = cond != held
        te = ~tr
        m = MLP(X.shape[1])
        m.fit(X[tr], y[tr], epochs=epochs)
        accs.append(float((m.predict(X[te]) == y[te]).mean()))
    return accs


def snr_sweep_from_features(model, X, y, snrs_col, snrs=config.EVAL_SNRS):
    """在固化的特征上按 SNR 分组评测，返回 [(snr, acc, n)]。

    不需要 WAV：特征里已经包含各 SNR 的副本，因此 CI 也能复现抗噪曲线。
    """
    out = []
    for snr in snrs:
        tag = "clean" if snr is None else str(int(snr))
        mask = snrs_col == tag
        if not mask.any():
            continue
        pred = model.predict(X[mask])
        out.append((snr, float((pred == y[mask]).mean()), int(mask.sum())))
    return out


def rejection_metrics(model, X, y):
    """把未知词当 ``unknown`` 类时的 FAR / FRR。

    FAR = 未知词被判成某个已知命令的比例（该拒未拒）
    FRR = 已知命令被拒（判成 unknown 或判错）的比例
    """
    pred = model.predict(X)
    unk_idx = corpus.LABELS.index(config.UNKNOWN_LABEL)
    is_unk = y == unk_idx
    far = float((pred[is_unk] != unk_idx).mean()) if is_unk.any() else float("nan")
    frr = float((pred[~is_unk] != y[~is_unk]).mean()) if (~is_unk).any() else float("nan")
    return far, frr


def confusion(model, X, y):
    """混淆矩阵 (11, 11)。"""
    n = len(corpus.LABELS)
    cm = np.zeros((n, n), dtype=int)
    for t, p in zip(y, model.predict(X)):
        cm[int(t), int(p)] += 1
    return cm