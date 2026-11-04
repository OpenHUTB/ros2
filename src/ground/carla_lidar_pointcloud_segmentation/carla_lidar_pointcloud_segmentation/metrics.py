"""分割评价指标（纯计算，不依赖 CARLA）。

逐点分类的常用指标都能从混淆矩阵推出来：第 ``i`` 行是真值类别，第 ``j`` 列是预测类别。

* 精确率 ``P_i = cm[i][i] / sum(cm[:, i])``（预测成 i 的点里有多少真的是 i）
* 召回率 ``R_i = cm[i][i] / sum(cm[i, :])``
* IoU ``= cm[i][i] / (sum(cm[i, :]) + sum(cm[:, i]) - cm[i][i])``
* F1 ``= 2PR / (P + R)``

某一类在真值和预测里都没出现时，该类的 IoU/F1 记 ``nan``，统计宏平均时跳过，
否则"没有行人的帧"会把宏平均拉低到没有意义。
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Sequence

import numpy as np

from .labels import CLASS_NAMES, N_CLASSES, UNKNOWN


def confusion_matrix(y_true: Sequence[int], y_pred: Sequence[int],
                     n_classes: int = N_CLASSES) -> np.ndarray:
    """统计混淆矩阵，形状 ``(n_classes, n_classes)``；越界类别直接忽略。"""
    true = np.asarray(y_true, dtype=int)
    pred = np.asarray(y_pred, dtype=int)
    if true.shape != pred.shape:
        raise ValueError('真值与预测长度不一致：%d vs %d' % (true.size, pred.size))
    cm = np.zeros((n_classes, n_classes), dtype=np.int64)
    for t, p in zip(true, pred):
        t, p = int(t), int(p)
        if 0 <= t < n_classes and 0 <= p < n_classes:
            cm[t, p] += 1
    return cm


def iou_per_class(cm: np.ndarray) -> List[float]:
    """逐类 IoU，无样本的类返回 nan。"""
    cm = np.asarray(cm, dtype=float)
    out: List[float] = []
    for i in range(cm.shape[0]):
        tp = cm[i, i]
        denom = cm[i, :].sum() + cm[:, i].sum() - tp
        out.append(float(tp / denom) if denom > 0 else float('nan'))
    return out


def precision_recall_f1(cm: np.ndarray):
    """返回 ``(precision, recall, f1)`` 三个列表，无样本处为 nan。"""
    cm = np.asarray(cm, dtype=float)
    precision, recall, f1 = [], [], []
    for i in range(cm.shape[0]):
        tp = cm[i, i]
        pred_pos = cm[:, i].sum()
        true_pos = cm[i, :].sum()
        p = float(tp / pred_pos) if pred_pos > 0 else float('nan')
        r = float(tp / true_pos) if true_pos > 0 else float('nan')
        precision.append(p)
        recall.append(r)
        if p == p and r == r and (p + r) > 0:      # nan 判断：p == p
            f1.append(2 * p * r / (p + r))
        else:
            f1.append(float('nan'))
    return precision, recall, f1


def nanmean(values: Iterable[float]) -> float:
    """忽略 nan 的均值；全为 nan 时返回 nan。"""
    vals = [v for v in values if v == v]
    return float(sum(vals) / len(vals)) if vals else float('nan')


def overall_accuracy(cm: np.ndarray) -> float:
    """总体逐点准确率。"""
    cm = np.asarray(cm, dtype=float)
    total = cm.sum()
    if total <= 0:
        return float('nan')
    return float(np.trace(cm) / total)


def macro_iou(cm: np.ndarray) -> float:
    """各类 IoU 的非空平均。"""
    return nanmean(iou_per_class(cm))


def evaluate(y_true: Sequence[int], y_pred: Sequence[int],
             n_classes: int = N_CLASSES) -> Dict[str, object]:
    """一次性给出完整报告：混淆矩阵、各类指标、宏平均、总体准确率。"""
    cm = confusion_matrix(y_true, y_pred, n_classes)
    iou = iou_per_class(cm)
    precision, recall, f1 = precision_recall_f1(cm)
    names = list(CLASS_NAMES[:n_classes])
    return {
        'confusion_matrix': cm,
        'iou': dict(zip(names, iou)),
        'precision': dict(zip(names, precision)),
        'recall': dict(zip(names, recall)),
        'f1': dict(zip(names, f1)),
        'macro_iou': macro_iou(cm),
        'macro_f1': nanmean(f1),
        'accuracy': overall_accuracy(cm),
    }


def range_edges(max_range: float, step: float = 20.0) -> List[float]:
    """按距离分档的边界，例如 (60, 20) -> [0, 20, 40, 60]。"""
    if max_range <= 0 or step <= 0:
        raise ValueError('距离上限与分档宽度必须为正')
    edges = [0.0]
    while edges[-1] < max_range - 1e-9:
        edges.append(min(edges[-1] + step, max_range))
    return edges


def evaluate_by_range(ranges: Sequence[float], y_true: Sequence[int],
                      y_pred: Sequence[int], edges: Sequence[float],
                      n_classes: int = N_CLASSES) -> Dict[str, Dict[str, float]]:
    """分距离档评价。远处点稀疏，精度通常会掉，这条曲线比单一准确率有信息量。"""
    r = np.asarray(ranges, dtype=float)
    t = np.asarray(y_true, dtype=int)
    p = np.asarray(y_pred, dtype=int)
    if not (r.size == t.size == p.size):
        raise ValueError('距离、真值、预测长度不一致')
    out: Dict[str, Dict[str, float]] = {}
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = (r >= lo) & (r < hi)
        key = '%.0f-%.0f m' % (lo, hi)
        if not mask.any():
            out[key] = {'n': 0, 'accuracy': float('nan'), 'macro_iou': float('nan')}
            continue
        cm = confusion_matrix(t[mask], p[mask], n_classes)
        out[key] = {'n': int(mask.sum()),
                    'accuracy': overall_accuracy(cm),
                    'macro_iou': macro_iou(cm)}
    return out


def drop_unknown(y_true: Sequence[int], y_pred: Sequence[int], *extra):
    """把真值为 -1（未知标签）的点剔除，可选同时同步过滤额外数组。"""
    keep = [i for i, v in enumerate(y_true) if int(v) != UNKNOWN]
    outs = [[int(y_true[i]) for i in keep], [int(y_pred[i]) for i in keep]]
    for arr in extra:
        seq = list(arr)
        outs.append([seq[i] for i in keep])
    return outs