#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``metrics`` 与 ``model`` 的离线单元测试（不需要 CARLA）。

    python3 test/test_segmentation.py
"""

import os
import shutil
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_lidar_pointcloud_segmentation import labels as L   # noqa: E402

from carla_lidar_pointcloud_segmentation import metrics as M   # noqa: E402
from carla_lidar_pointcloud_segmentation import model as MD   # noqa: E402

_passed = 0
_failed = []


def check(desc, ok, detail=''):
    global _passed
    if ok:
        _passed += 1
        print('[通过] %s' % desc)
    else:
        _failed.append(desc)
        print('[失败] %s%s' % (desc, ('  ' + detail) if detail else ''))


def expect_error(desc, func, exc=ValueError):
    try:
        func()
    except exc:
        check(desc, True)
    except Exception as other:      # noqa: BLE001
        check(desc, False, '抛了 %s' % type(other).__name__)
    else:
        check(desc, False, '没有抛异常')


def main():
    print('=' * 72)
    print('metrics.py / model.py 离线单元测试')
    print('=' * 72)

    # ---------------- 1. 混淆矩阵 ----------------
    print('\n[1] 混淆矩阵')
    cm = M.confusion_matrix([0, 0, 1, 1], [0, 1, 1, 1], n_classes=2)
    check('矩阵形状正确', cm.shape == (2, 2), str(cm.shape))
    check('真 0 预 0 计数', cm[0, 0] == 1, str(cm[0, 0]))
    check('真 0 预 1 计数', cm[0, 1] == 1, str(cm[0, 1]))
    check('真 1 预 1 计数', cm[1, 1] == 2, str(cm[1, 1]))
    check('总数等于样本数', int(cm.sum()) == 4)
    expect_error('长度不一致时报错', lambda: M.confusion_matrix([0, 1], [0]))
    ignore = M.confusion_matrix([0, 9], [0, 0], n_classes=2)
    check('越界类别被忽略', int(ignore.sum()) == 1, str(int(ignore.sum())))

    # ---------------- 2. IoU / F1 ----------------
    print('\n[2] IoU 与 F1')
    perfect = M.confusion_matrix([0, 1, 2], [0, 1, 2], n_classes=3)
    check('完全正确时 IoU 全为 1', all(abs(v - 1.0) < 1e-9 for v in M.iou_per_class(perfect)))
    cm2 = M.confusion_matrix([0, 0, 1, 1], [0, 1, 1, 1], n_classes=2)
    check('类 1 的 IoU = 2/3', abs(M.iou_per_class(cm2)[1] - 2.0 / 3.0) < 1e-9,
          '%.4f' % M.iou_per_class(cm2)[1])
    check('类 0 的 IoU = 1/2', abs(M.iou_per_class(cm2)[0] - 0.5) < 1e-9,
          '%.4f' % M.iou_per_class(cm2)[0])
    empty_cm = np.array([[1, 0], [0, 0]])
    check('没有样本的类 IoU 为 nan', M.iou_per_class(empty_cm)[1] != M.iou_per_class(empty_cm)[1])

    precision, recall, f1 = M.precision_recall_f1(cm2)
    check('类 1 精确率 = 2/3', abs(precision[1] - 2.0 / 3.0) < 1e-9, '%.4f' % precision[1])
    check('类 1 召回率 = 1.0', abs(recall[1] - 1.0) < 1e-9, '%.4f' % recall[1])
    check('类 0 召回率 = 0.5', abs(recall[0] - 0.5) < 1e-9, '%.4f' % recall[0])
    check('F1 在 [0,1] 内', all(0.0 <= v <= 1.0 for v in f1 if v == v))
    check('宏平均 IoU 忽略 nan', abs(M.macro_iou(empty_cm) - 1.0) < 1e-9,
          '%.4f' % M.macro_iou(empty_cm))
    check('总体准确率 = 3/4', abs(M.overall_accuracy(cm2) - 0.75) < 1e-9)
    check('nanmean 忽略 nan', abs(M.nanmean([1.0, float('nan'), 3.0]) - 2.0) < 1e-9)
    check('全 nan 时 nanmean 返回 nan', M.nanmean([float('nan')]) != M.nanmean([float('nan')]))

    # ---------------- 3. 分距离评价 ----------------
    print('\n[3] 分距离评价')
    edges = M.range_edges(60.0, 20.0)
    check('分档边界为 [0,20,40,60]', edges == [0.0, 20.0, 40.0, 60.0], str(edges))
    expect_error('分档参数非正时报错', lambda: M.range_edges(0.0, 20.0))
    ranges = [5.0, 15.0, 25.0, 55.0]
    by_range = M.evaluate_by_range(ranges, [0, 0, 1, 1], [0, 0, 1, 0], edges, n_classes=2)
    check('分档键名正确', list(by_range) == ['0-20 m', '20-40 m', '40-60 m'], str(list(by_range)))
    check('第一档 2 个点', by_range['0-20 m']['n'] == 2, str(by_range['0-20 m']['n']))
    check('第一档全对', abs(by_range['0-20 m']['accuracy'] - 1.0) < 1e-9)
    check('第三档 1 个点且判错', by_range['40-60 m']['accuracy'] == 0.0,
          str(by_range['40-60 m']['accuracy']))
    expect_error('长度不一致时报错',
                 lambda: M.evaluate_by_range([1.0, 2.0], [0], [0], edges))

    report = M.evaluate([0, 1, 1], [0, 1, 1], n_classes=2)
    check('报告含混淆矩阵与宏平均',
          {'confusion_matrix', 'macro_iou', 'macro_f1', 'accuracy'} <= set(report))
    y_t, y_p, extra = M.drop_unknown([0, -1, 1], [0, 0, 1], [10.0, 20.0, 30.0])
    check('drop_unknown 剔除未知标签', y_t == [0, 1], str(y_t))
    check('drop_unknown 同步过滤附加数组', extra == [10.0, 30.0], str(extra))

    # ---------------- 4. 网络结构与训练 ----------------
    print('\n[4] 多层感知机')
    mlp = MD.MLP(n_in=9, hidden=(16,), n_out=3, seed=0)
    logits, acts = mlp.forward(np.zeros((5, 9)))
    check('前向输出形状为 (N, 3)', logits.shape == (5, 3), str(logits.shape))
    check('激活值层数为 2', len(acts) == 2, str(len(acts)))
    probs = mlp.predict_proba(np.zeros((5, 9)))
    check('概率每行和为 1', np.allclose(probs.sum(axis=1), 1.0))
    check('参数量大于 0', mlp.n_parameters() > 0)
    expect_error('输入维度不符时报错', lambda: mlp.forward(np.zeros((2, 4))))

    rng = np.random.default_rng(3)
    centers = np.array([[0.0] * 9, [4.0] * 9, [-4.0] * 9])
    x = np.vstack([rng.normal(c, 1.0, size=(150, 9)) for c in centers])
    y = np.repeat([0, 1, 2], 150)
    x_tr, y_tr, x_val, y_val = MD.train_val_split(x, y, val_ratio=0.2, seed=1)
    check('训练集与验证集不重叠',
          len(x_tr) + len(x_val) == len(x) and len(x_val) == 90,
          '%d + %d' % (len(x_tr), len(x_val)))

    mlp = MD.MLP(n_in=9, hidden=(16,), n_out=3, seed=2)
    history = mlp.fit(x_tr, y_tr, epochs=40, batch_size=64, lr=0.05, seed=3)
    check('损失随训练下降', history['loss'][-1] < history['loss'][0],
          '%.4f -> %.4f' % (history['loss'][0], history['loss'][-1]))
    val_acc = float(np.mean(mlp.predict(x_val) == y_val))
    check('三簇数据验证准确率 > 0.9', val_acc > 0.9, '%.3f' % val_acc)
    check('训练历史长度等于轮数', len(history['acc']) == 40, str(len(history['acc'])))

    weights = [1.0, 1.0, 5.0]
    mlp2 = MD.MLP(n_in=9, hidden=(8,), n_out=3, seed=4)
    hist2 = mlp2.fit(x_tr, y_tr, epochs=5, batch_size=64, lr=0.02,
                     sample_weights=weights, seed=5)
    check('带类别权重也能训练', hist2['loss'][-1] == hist2['loss'][-1])
    expect_error('类别权重数量不对时报错',
                 lambda: MD.MLP(n_in=9, hidden=(8,), n_out=3).fit(
                     x_tr, y_tr, epochs=1, sample_weights=[1.0, 1.0]))
    expect_error('标签越界时报错',
                 lambda: MD.MLP(n_in=9, hidden=(8,), n_out=2).fit(
                     np.zeros((4, 9)), [0, 1, 2, 0], epochs=1))

    # ---------------- 5. 存取 ----------------
    print('\n[5] 权重存取')
    tmp = os.path.join(os.path.dirname(os.path.abspath(__file__)), '_tmp_weights')
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp, exist_ok=True)
    try:
        path = os.path.join(tmp, 'weights.npz')
        mlp.save(path, meta={'feature_names': ['a'], 'classes': ['x', 'y', 'z']})
        check('权重文件已生成', os.path.isfile(path))
        check('元信息文件已生成', os.path.isfile(os.path.join(tmp, 'weights.json')))
        loaded = MD.MLP.load(path)
        before = mlp.predict_proba(x_val[:20])
        after = loaded.predict_proba(x_val[:20])
        check('读回的模型预测一致', np.allclose(before, after))
        check('读回的结构一致', loaded.params['W0'].shape == mlp.params['W0'].shape)
        check('权重文件小于 1 MB', os.path.getsize(path) < 1024 * 1024,
              '%.1f KB' % (os.path.getsize(path) / 1024.0))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # ---------------- 6. 报告排版 ----------------
    print('\n[6] 报告排版')
    from carla_lidar_pointcloud_segmentation import pipeline as PP
    fake = {
        'report': {
            'confusion_matrix': [[90, 5, 3, 2], [4, 80, 1, 5], [2, 1, 70, 7], [3, 6, 4, 87]],
            'iou': {'drivable': 0.9, 'vehicle': 0.8, 'pedestrian': 0.7, 'obstacle': 0.85},
            'precision': {'drivable': 0.9, 'vehicle': 0.8, 'pedestrian': 0.7, 'obstacle': 0.85},
            'recall': {'drivable': 0.9, 'vehicle': 0.8, 'pedestrian': 0.7, 'obstacle': 0.85},
            'f1': {'drivable': 0.9, 'vehicle': 0.8, 'pedestrian': 0.7, 'obstacle': 0.85},
            'macro_iou': 0.812, 'macro_f1': 0.825, 'accuracy': 0.905,
        },
        'by_range': {'0-20 m': {'n': 100, 'accuracy': 0.95, 'macro_iou': 0.9}},
        'n_train': 1000, 'n_val': 200, 'n_parameters': 1508,
    }
    text = PP.format_report(fake)
    check('报告含混淆矩阵标题', '混淆矩阵' in text)
    check('报告含四类名称', all(name in text for name in L.CLASS_NAMES))
    check('报告含宏平均一行', '宏平均 IoU 0.812' in text)

    print('\n' + '=' * 72)
    if _failed:
        print('[结果] %d 项通过，%d 项失败' % (_passed, len(_failed)))
        for name in _failed:
            print('  失败：%s' % name)
        return 1
    print('[结果] %d 项全部通过' % _passed)
    print('=' * 72)
    return 0


if __name__ == '__main__':
    sys.exit(main())