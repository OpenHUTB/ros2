#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``gesture_train`` 离线回归（不需要 ROS / 摄像头 / torch）。

    python3 tests/test_gesture_train_local.py
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_REPO, "src", "air", "gesture_control_drone",
                                "scripts"))

import gesture_dataset as gd      # noqa: E402
import gesture_train as gt        # noqa: E402

_R = []


def check(name, ok, detail=""):
    _R.append((bool(ok), name, str(detail)))
    if not ok:
        print("[失败] %s  %s" % (name, detail))


def make_dataset(per_class=30, seed=0):
    """构造 7 类线性可分数据（每类一个不同的均值方向）。"""
    rng = np.random.RandomState(seed)
    X, y = [], []
    for ci in range(gd.NUM_CLASSES):
        center = np.zeros(gd.FEATURE_DIM, dtype=np.float32)
        center[ci * 6:(ci + 1) * 6] = 3.0
        X.append(center + rng.normal(0, 0.3, (per_class, gd.FEATURE_DIM)))
        y.extend([ci] * per_class)
    X = np.concatenate(X).astype(np.float32)
    y = np.array(y, dtype=np.int64)
    # 必须打乱：按类顺序排列的话，按时间划分会把最后几类整段留在验证集里
    # （这正是真实采集时踩过的坑，测试数据别自己再踩一次）
    order = rng.permutation(len(y))
    return X[order], y[order]


# --------------------------------------------------------------------------- #
def test_mlp_forward():
    X, y = make_dataset(per_class=5)
    m = gt.MLP(seed=0)
    h, probs = m.forward(X)
    check("隐层形状 (N, hidden)", h.shape == (len(y), m.hidden), h.shape)
    check("概率形状 (N, 7)", probs.shape == (len(y), gd.NUM_CLASSES), probs.shape)
    check("每行概率和为 1", np.allclose(probs.sum(axis=1), 1.0, atol=1e-5),
          probs.sum(axis=1)[:3])
    check("预测形状正确", m.predict(X).shape == (len(y),))
    check("置信度在 (0,1]", bool(np.all((m.confidence(X) > 0) & (m.confidence(X) <= 1))))

    try:
        m.forward(np.zeros((3, 10)))
        ok = False
    except ValueError:
        ok = True
    check("输入维度错误抛 ValueError", ok)


def test_train_converges():
    X, y = make_dataset(per_class=30)
    tr, va = gt.split_indices(y, 0.3, mode="time", seed=0)
    m = gt.MLP(seed=0)
    hist = m.train(X[tr], y[tr], epochs=120, lr=0.05, batch=32, seed=0)
    check("训练历史长度正确", len(hist) == 120, len(hist))
    check("loss 下降", hist[-1]["loss"] < hist[0]["loss"], (hist[0]["loss"], hist[-1]["loss"]))
    res = gt.evaluate(m, X[va], y[va])
    check("可分数据上验证准确率 > 0.9 (%.3f)" % res["accuracy"], res["accuracy"] > 0.9,
          res["accuracy"])
    check("evaluate 含关键字段",
          set(res) >= {"accuracy", "confusion", "per_class_recall",
                       "mean_confidence", "n"}, list(res))
    check("混淆矩阵形状", res["confusion"].shape == (gd.NUM_CLASSES, gd.NUM_CLASSES),
          res["confusion"].shape)
    text = gt.format_report(res, "测试")
    check("报告里有准确率行", "准确率" in text and "每类召回" in text, text[:80])


def test_train_errors():
    X, y = make_dataset(per_class=3)
    m = gt.MLP(seed=0)
    for args, what in ((dict(X=np.zeros((0, 42), np.float32),
                             y=np.zeros((0,), np.int64)), "空训练集"),
                       (dict(X=X[:5], y=np.zeros(5, np.int64)), "只有一类")):
        try:
            m.train(**args)
            ok = False
        except ValueError:
            ok = True
        check("%s 抛 ValueError" % what, ok)


def test_split():
    y = np.repeat(np.arange(gd.NUM_CLASSES), 10)
    tr, va = gt.split_indices(y, 0.3, mode="time")
    check("time: 索引连续（训练在前）", tr.tolist() == list(range(len(tr))), tr[:5])
    check("time: 验证在后", va.tolist() == list(range(len(tr), len(y))), va[:5])
    check("time: 无交集", len(set(tr) & set(va)) == 0)
    check("time: 并集全覆盖", sorted(tr.tolist() + va.tolist()) == list(range(len(y))))

    for mode in ("random", "stratified"):
        tr2, va2 = gt.split_indices(y, 0.3, mode=mode, seed=1)
        check("%s: 无交集" % mode, len(set(tr2) & set(va2)) == 0)
        check("%s: 并集全覆盖" % mode,
              sorted(tr2.tolist() + va2.tolist()) == list(range(len(y))))

    try:
        gt.split_indices(y, mode="nope")
        ok = False
    except ValueError:
        ok = True
    check("非法 mode 抛 ValueError", ok)

    for bad in (0.0, 1.0, -0.5):
        try:
            gt.split_indices(y, val_ratio=bad)
            ok = False
        except ValueError:
            ok = True
        check("非法 val_ratio %.1f 抛 ValueError" % bad, ok)

    try:
        gt.split_indices(np.array([0]))
        ok = False
    except ValueError:
        ok = True
    check("样本数 < 2 抛 ValueError", ok)


def test_confusion_and_save(tmp):
    cm = gt.confusion_matrix([0, 0, 1, 1, 1], [0, 1, 1, 1, 0], gd.NUM_CLASSES)
    check("混淆矩阵对角正确", cm[0, 0] == 1 and cm[1, 1] == 2, cm[:2, :2].tolist())
    check("混淆矩阵误分类正确", cm[0, 1] == 1 and cm[1, 0] == 1, cm[:2, :2].tolist())
    check("未出现的类别全 0", cm[2:].sum() == 0)

    X, y = make_dataset(per_class=10)
    m = gt.MLP(seed=0)
    m.train(X, y, epochs=30, lr=0.05, batch=16, seed=0)
    pred_before = m.predict(X)

    path = m.save(os.path.join(tmp, "m.npz"))
    check("保存生成文件", os.path.isfile(path), path)
    loaded = gt.MLP.load(path)
    check("读回隐层一致", loaded.hidden == m.hidden, loaded.hidden)
    check("读回预测一致", np.array_equal(loaded.predict(X), pred_before))

    try:
        gt.MLP.load(os.path.join(tmp, "nope.npz"))
        ok = False
    except FileNotFoundError:
        ok = True
    check("读不存在的模型抛 FileNotFoundError", ok)

    try:
        gt.MLP(weights={"W1": np.zeros((42, 8), np.float32)})
        ok = False
    except ValueError:
        ok = True
    check("权重缺字段抛 ValueError", ok)


def test_cli(tmp):
    X, y = make_dataset(per_class=20)
    data = os.path.join(tmp, "d.npz")
    gd.save_session(data, X, y)
    out = os.path.join(tmp, "cli_model.npz")
    rc = gt.main(["--data", data, "--out", out, "--epochs", "40", "--compare"])
    check("CLI 正常退出", rc == 0, rc)
    check("CLI 生成了模型", os.path.isfile(out), out)

    # 单类别数据：应返回 2（不训练）
    one = os.path.join(tmp, "one.npz")
    gd.save_session(one, X[:5], np.zeros(5, dtype=np.int64))
    rc2 = gt.main(["--data", one, "--out", out])
    check("单类别数据返回退出码 2", rc2 == 2, rc2)


def test_data_path_resolution(tmp):
    """默认数据定位：源码树能找到；找不到时给出可读的错误。"""
    found = gt.default_data_path()
    check("能找到仓库自带的样本数据", os.path.isfile(found), found)
    check("返回绝对路径", os.path.isabs(found), found)

    cands = gt.data_candidates()
    check("候选路径覆盖源码树布局", any(os.pardir in c for c in cands), cands[:2])
    check("候选路径里含 share 兜底",
          any("share" in c.replace(os.sep, "/") for c in cands), cands)

    try:
        gt.default_data_path("definitely_missing.npz")
        ok = False
    except FileNotFoundError as exc:
        ok = "--data" in str(exc)   # 错误信息里要告诉用户怎么办
    check("找不到数据时抛 FileNotFoundError 且给出建议", ok)

    # --data 指向不存在的文件：返回 2，不抛 traceback
    rc = gt.main(["--data", os.path.join(tmp, "nope.npz"), "--epochs", "5"])
    check("--data 不存在时返回退出码 2", rc == 2, rc)




def test_mlp_classes_validation(tmp):
    """模型里的类别顺序必须和当前词表一致，否则静默错标（评审第 3 条）。"""
    X, y = make_dataset(per_class=5)
    m = gt.MLP(seed=0)
    m.train(X, y, epochs=10, lr=0.05, batch=8, seed=0)
    path = m.save(os.path.join(tmp, "cls.npz"))

    check("正常模型能读回", gt.MLP.load(path).hidden == m.hidden)

    # 篡改 classes 顺序
    with np.load(path, allow_pickle=False) as data:
        payload = {k: data[k] for k in data.files}
    payload["classes"] = np.array(list(reversed(gd.trainable_gestures())))
    bad = os.path.join(tmp, "cls_bad.npz")
    np.savez_compressed(bad, **payload)
    try:
        gt.MLP.load(bad)
        ok = False
    except ValueError as exc:
        ok = "词表" in str(exc) or "类别" in str(exc)
    check("类别顺序不一致时抛 ValueError", ok)


def test_majority_baseline(capsys=None):
    """--compare 必须给出多数类基线（评审第 5 条）。"""
    # 直接测函数：真实验证集就是 26 PALM + 9 FIST + 1 POINT
    va_labels = np.array([0] * 26 + [1] * 9 + [2] * 1, dtype=np.int64)
    baseline = gt.majority_baseline(va_labels)
    check("基线按验证集类别比例算（26/36=0.722）", abs(baseline - 0.722) < 0.001, baseline)
    try:
        gt.majority_baseline(np.array([], dtype=np.int64))
        ok = False
    except ValueError:
        ok = True
    check("空标签算基线抛 ValueError", ok)

    lines = []
    import builtins
    _print = builtins.print
    builtins.print = lambda *a, **k: lines.append(" ".join(str(x) for x in a))
    try:
        gt._print_comparison([("time", 1.0, 0.972, 36)], baseline)
    finally:
        builtins.print = _print
    text = "\n".join(lines)
    check("对比表里有多数类基线行", "多数类基线" in text, text[:120])
    check("基线数值正确", "0.722" in text, text[:120])


def main():
    tmp = tempfile.mkdtemp(prefix="gesture_train_")
    try:
        test_mlp_forward()
        test_train_converges()
        test_train_errors()
        test_split()
        test_confusion_and_save(tmp)
        test_cli(tmp)
        test_data_path_resolution(tmp)
        test_mlp_classes_validation(tmp)
        test_majority_baseline()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    total = len(_R)
    passed = sum(1 for ok, _, _ in _R if ok)
    print("=" * 60)
    print("gesture_train 离线回归：%d/%d 通过" % (passed, total))
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
