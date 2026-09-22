#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 无人车 · 神经网络回归测试。

同时支持两种运行方式：
  1) 纯脚本（无需 pytest）：  python tests/test_nn_models.py
  2) pytest：                 pytest tests/

覆盖：
  - nn_models.MLPClassifier 分类收敛（精度）
  - nn_models.MLPPolicy 回归收敛（MSE）
  - 模型保存 / 加载往返一致性
  - 作业二/三 的 synth_dataset 形状
"""
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import nn_models  # noqa: E402


def _class_data(n=300, seed=0):
    rng = np.random.RandomState(seed)
    X = rng.randn(n, 4)
    Y = (X[:, 0] + X[:, 1] > 0).astype(int)
    return X, Y


def _reg_data(n=300, seed=1):
    rng = np.random.RandomState(seed)
    X = rng.randn(n, 2)
    Y = np.sin(X[:, 0]) * 0.5 + X[:, 1] * 0.3
    return X, Y.reshape(-1, 1)


def test_mlp_classifier_converges():
    X, Y = _class_data()
    net = nn_models.MLPClassifier([4, 12, 2], seed=2)
    net.train(X, Y, epochs=300, lr=0.2, verbose=0)
    acc = float(np.mean(net.predict(X) == Y))
    assert acc > 0.9, f"classification acc too low: {acc}"


def test_mlp_policy_regression_converges():
    X, Y = _reg_data()
    net = nn_models.MLPPolicy([2, 16, 1], seed=3)
    net.train(X, Y, epochs=300, lr=0.1, verbose=0)
    mse = float(np.mean((net.predict(X) - Y) ** 2))
    assert mse < 0.02, f"regression MSE too high: {mse}"


def test_model_save_load_roundtrip():
    X, Y = _class_data()
    net = nn_models.MLPClassifier([4, 12, 2], seed=4)
    net.train(X, Y, epochs=200, lr=0.2, verbose=0)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "m.json")
        net.save(p)
        net2 = nn_models.MLPClassifier.load(p)
    assert np.array_equal(net.predict(X), net2.predict(X)), "save/load changed predictions"


def test_synth_dataset_shapes():
    import importlib.util as iu
    for mod_name, path in [("percep", "02_perception"), ("nav", "03_navigation")]:
        spec = iu.spec_from_file_location(mod_name, os.path.join(ROOT, path, "main.py"))
        mod = iu.module_from_spec(spec)
        spec.loader.exec_module(mod)
        d = mod.synth_dataset(80)
        assert len(d) in (4, 2), f"{path} synth_dataset unexpected len {len(d)}"


def test_simple_cnn_end_to_end():
    """纯 numpy CNN 图像->转向 收敛；save/load 往返一致。"""
    rng = np.random.RandomState(0)
    n = 48
    X = np.zeros((n, 20, 24, 3), dtype=np.uint8)
    Y = np.zeros(n)
    for i in range(n):
        side = 1 if rng.rand() > 0.5 else -1
        col = (0 if side < 0 else 19) + rng.randint(0, 3)
        X[i, :, col, :] = 220 + rng.randint(0, 20)
        X[i, :, col + 1, :] = 200
        Y[i] = 0.5 * side
    net = nn_models.SimpleCNN(in_channels=3, hidden=6, out=1, img=(20, 24), seed=0)
    net.train(X, Y, epochs=120, lr=0.05, batch=16, verbose=0)
    pred = np.tanh(net.predict(X).ravel())
    mse = float(np.mean((pred - Y) ** 2))
    assert mse < 0.01, f"SimpleCNN MSE too high: {mse}"
    # save/load
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "cnn.json")
        net.save(p)
        net2 = nn_models.SimpleCNN.load(p)
    assert np.allclose(np.sign(net.predict(X)), np.sign(net2.predict(X))), "CNN save/load mismatch"


def _run_all():
    """standalone __main__：逐项运行带 test_ 前缀的函数并报告 PASS/FAIL。"""
    fns = sorted(k for k in globals() if k.startswith("test_"))
    passed, failed = 0, []
    for fn in fns:
        try:
            globals()[fn]()
            passed += 1
            print(f"[PASS] {fn}")
        except Exception as e:  # noqa: BLE001
            failed.append(fn)
            print(f"[FAIL] {fn}: {e}")
    print(f"\n==== 测试结果：PASS={passed}  FAIL={len(failed)} ====")
    if failed:
        print("失败的测试:", failed)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
