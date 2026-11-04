"""从"逐帧点云 + 逐点标签"到可训练特征矩阵的流水线（纯计算，不依赖 CARLA）。

一条帧数据的组织方式与 :mod:`app` 采集出来的 ``dataset.npz`` 一致：

* ``points``  ``(N, 3)``  普通激光雷达点，车体坐标系，米；
* ``tags``    ``(N,)``    CARLA 语义激光雷达给的 ``object_tag``（未映射的原始值）；
* ``frames``  ``(N,)``    每个点属于第几帧，用于按帧估路面高度。

:func:`frame_features` 按帧计算特征（路面高度必须逐帧估，不能拿全局的），
:func:`train_eval` 负责划分、训练、分距离评价。
"""

from __future__ import annotations

from typing import Dict, Sequence, Tuple

import numpy as np

from . import features as F
from . import labels as L
from . import metrics as M
from . import model as MD


def frame_features(points: np.ndarray, tags: np.ndarray,
                   frames: np.ndarray, voxel: float = 0.5
                   ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """逐帧提特征并映射标签。

    返回 ``(X, y, r)``：特征矩阵、类别标签（未知标签的点已被丢弃）、水平距离。
    三个数组按同一顺序对齐。
    """
    pts = np.asarray(points, dtype=float)
    tag_arr = np.asarray(tags, dtype=int)
    frame_arr = np.asarray(frames, dtype=int)
    if not (pts.shape[0] == tag_arr.size == frame_arr.size):
        raise ValueError('points/tags/frames 数量不一致：%d/%d/%d'
                         % (pts.shape[0], tag_arr.size, frame_arr.size))
    if pts.shape[0] == 0:
        return np.zeros((0, len(F.FEATURE_NAMES))), np.zeros(0, dtype=int), np.zeros(0)

    xs, ys, rs = [], [], []
    for frame_id in np.unique(frame_arr):
        mask = frame_arr == frame_id
        sub_pts = pts[mask]
        sub_tags = tag_arr[mask]
        feats = F.point_features(sub_pts, ground_z=F.estimate_ground_z(sub_pts), voxel=voxel)
        cls = np.array(L.group_labels(sub_tags), dtype=int)
        keep = cls != L.UNKNOWN
        if not keep.any():
            continue
        xs.append(feats[keep])
        ys.append(cls[keep])
        rs.append(sub_pts[keep, 0] ** 2 + sub_pts[keep, 1] ** 2)

    if not xs:
        return np.zeros((0, len(F.FEATURE_NAMES))), np.zeros(0, dtype=int), np.zeros(0)
    return (np.vstack(xs), np.concatenate(ys),
            np.sqrt(np.concatenate(rs)))


def train_eval(X: np.ndarray, y: np.ndarray, r: np.ndarray,
               hidden: Sequence[int] = (32, 32), epochs: int = 30,
               batch_size: int = 256, lr: float = 0.01,
               val_ratio: float = 0.2, weighted: bool = True,
               seed: int = 0, range_step: float = 20.0
               ) -> Dict[str, object]:
    """标准化 + 划分 + 训练 + 评价，返回一份可直接打印的报告。

    标签类别不平衡时默认按 :func:`labels.class_weights` 加权，
    否则"路面"一类会吃掉大部分梯度。
    """
    X = np.asarray(X, dtype=float)
    y = np.asarray(y, dtype=int)
    r = np.asarray(r, dtype=float)
    if X.shape[0] == 0:
        raise ValueError('没有可用样本')
    if not (X.shape[0] == y.size == r.size):
        raise ValueError('特征/标签/距离长度不一致')

    x_tr, y_tr, x_val, y_val = MD.train_val_split(X, y, val_ratio=val_ratio, seed=seed)
    mean, std = F.standardize_fit(x_tr)
    x_tr_n = F.standardize_apply(x_tr, mean, std)
    x_val_n = F.standardize_apply(x_val, mean, std)

    net = MD.MLP(n_in=X.shape[1], hidden=hidden, n_out=L.N_CLASSES, seed=seed)
    class_w = L.class_weights(y_tr) if weighted else None
    history = net.fit(x_tr_n, y_tr, epochs=epochs, batch_size=batch_size,
                      lr=lr, sample_weights=class_w, seed=seed + 1)

    pred_val = net.predict(x_val_n)
    report = M.evaluate(y_val, pred_val)

    # 验证集的距离：用与 train_val_split 相同的排列取回（同一 seed、同一划分规则）
    n = X.shape[0]
    n_val = int(round(n * val_ratio))
    perm = np.random.default_rng(seed).permutation(n)
    r_val = r[perm[:n_val]]

    edges = M.range_edges(float(np.max(r_val)) if r_val.size else range_step, range_step)
    by_range = M.evaluate_by_range(r_val, y_val, pred_val, edges)

    return {
        'model': net,
        'mean': mean,
        'std': std,
        'history': history,
        'report': report,
        'by_range': by_range,
        'n_train': int(x_tr.shape[0]),
        'n_val': int(x_val.shape[0]),
        'class_weights': class_w,
        'n_parameters': net.n_parameters(),
    }


def format_report(result: Dict[str, object]) -> str:
    """把 :func:`train_eval` 的结果排版成表格文本（终端输出用）。"""
    report = result['report']
    lines = []
    lines.append('样本：训练 %d / 验证 %d，参数量 %d'
                 % (result['n_train'], result['n_val'], result['n_parameters']))
    lines.append('')
    lines.append('%-12s %8s %8s %8s %8s' % ('类别', 'IoU', '精确率', '召回率', 'F1'))
    lines.append('-' * 48)
    for name in L.CLASS_NAMES:
        lines.append('%-12s %8.3f %8.3f %8.3f %8.3f' % (
            name, report['iou'][name], report['precision'][name],
            report['recall'][name], report['f1'][name]))
    lines.append('-' * 48)
    lines.append('宏平均 IoU %.3f，宏平均 F1 %.3f，总体准确率 %.3f'
                 % (report['macro_iou'], report['macro_f1'], report['accuracy']))
    lines.append('')
    lines.append('混淆矩阵（行=真实，列=预测）')
    names = list(L.CLASS_NAMES)
    matrix = report['confusion_matrix']
    width = max(12, max(len(n) for n in names) + 2)
    lines.append('%*s' % (width, '') + ''.join('%*s' % (width, n) for n in names))
    for index, name in enumerate(names):
        lines.append('%-*s' % (width, name)
                     + ''.join('%*d' % (width, int(v)) for v in matrix[index]))
    lines.append('')
    lines.append('%-14s %6s %8s %8s' % ('距离档', '点数', '准确率', '宏平均IoU'))
    lines.append('-' * 40)
    for key, value in result['by_range'].items():
        lines.append('%-14s %6d %8.3f %8.3f'
                     % (key, value['n'], value['accuracy'], value['macro_iou']))
    return '\n'.join(lines)