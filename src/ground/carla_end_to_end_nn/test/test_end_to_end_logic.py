#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 端到端神经网络（图像 → 控制）—— 单元测试。

不依赖 CARLA、不依赖 ROS、不依赖 TensorFlow：
  1. 合成图像确实**含有**转向信息（车道线位置与标签强相关）
  2. maxpool 反向传播：梯度只回传给窗口内最大值位置（回归测试：早期版本均摊到全部位置）
  3. maxpool 梯度与数值梯度一致（有限差分校验）
  4. 推理输出与训练目标同值域 [-1,1]（回归测试：早期 predict 返回未过 tanh 的无界值）
  5. CNN 能真正学到"图像 → 转向"（相关系数与方向一致率）
  6. 模型保存/加载往返一致
  7. 预测在图像被加噪时稳定（不依赖单一像素）
  8. 专家控制器几何律正确
  9. 离线取证模式跑通并导出图

运行：  python3 test/test_end_to_end_logic.py
"""

import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location("e2e_main", os.path.join(_PKG, "main.py"))
main_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(main_mod)

from carla_end_to_end_nn.nn_models import SimpleCNN  # noqa: E402


# ---------------------------------------------------------------- 数据
def test_synth_images_contain_steering_info():
    """合成图像必须真的含有"路往哪弯"的信息。

    这是端到端学习的前提：图像与标签要有因果关系。用"车道线亮像素的横向重心"
    作为图像中转向信息的代理量，它应与 steer 标签强相关。
    """
    X, Y = main_mod.synth_dataset(n=60, seed=0)
    cents = []
    for i in range(len(X)):
        gray = X[i].astype(np.float32).mean(axis=2)
        bright = gray > 200
        if bright.sum() > 0:
            _ys, xs = np.nonzero(bright)
            cents.append(xs.mean())
        else:
            cents.append(np.nan)
    cents = np.asarray(cents)
    r = float(np.corrcoef(cents, Y.ravel())[0, 1])
    assert abs(r) > 0.9, (
        f"车道线位置与转向标签相关性过弱 (r={r:.3f})，"
        f"说明图像里看不出转向，网络不可能学到映射")


def test_synth_dataset_shapes():
    X, Y = main_mod.synth_dataset(n=10, seed=0)
    assert X.shape == (10, main_mod.IMG_H, main_mod.IMG_W, 3), f"图像形状错误: {X.shape}"
    assert Y.shape == (10, 1), f"标签形状错误: {Y.shape}"
    assert X.dtype == np.uint8, f"图像应为 uint8，实际 {X.dtype}"


# ---------------------------------------------------------------- 梯度
def test_maxpool_backward_only_max_position():
    """maxpool 反向：梯度只应回传给窗口内最大值所在位置。

    回归测试：早期实现把 dout 赋给窗口内全部 p×p 个位置，等价于把梯度放大 p² 倍
    并把梯度错分给未被选中的元素，导致网络无法学习（输出塌缩为常数）。
    """
    net = SimpleCNN(in_channels=3, hidden=4, out=1, img=(10, 12), seed=0)
    # 构造 4×4 单通道输入，2×2 maxpool → 2×2 输出
    x = np.zeros((1, 4, 4, 1), dtype=np.float32)
    x[0, 0, 0, 0] = 5.0      # 窗口(0,0)的最大值
    x[0, 2, 3, 0] = 7.0      # 窗口(1,1)的最大值
    x[0, 0, 1, 0] = 1.0
    x[0, 1, 0, 0] = 2.0
    dout = np.ones((1, 2, 2, 1), dtype=np.float32)
    dx = net._pool_backward(dout, x)

    assert dx[0, 0, 0, 0] == 1.0, f"最大值位置应收到梯度 1，实际 {dx[0, 0, 0, 0]}"
    assert dx[0, 2, 3, 0] == 1.0, f"最大值位置应收到梯度 1，实际 {dx[0, 2, 3, 0]}"
    # 非最大值位置不应收到梯度
    assert dx[0, 0, 1, 0] == 0.0, f"非最大值位置不应收到梯度，实际 {dx[0, 0, 1, 0]}"
    assert dx[0, 1, 0, 0] == 0.0, f"非最大值位置不应收到梯度，实际 {dx[0, 1, 0, 0]}"
    # 总梯度不应被放大
    assert abs(dx.sum() - dout.sum()) < 1e-6, \
        f"回传梯度总和应守恒（{dout.sum()}），实际 {dx.sum()}"


def test_pool_gradient_matches_finite_difference():
    """用有限差分校验 maxpool 反向传播正确性。"""
    net = SimpleCNN(in_channels=2, hidden=3, out=1, img=(8, 8), seed=0)
    rng = np.random.RandomState(0)
    x = rng.rand(2, 8, 8, 2).astype(np.float32) * 255
    p1, _ = net._pool(net._conv_forward(x, net.c1[0], net.c1[1]))  # 池化前的激活

    def f(a):
        pr, _ = net._pool(a)
        return float(np.sum(pr ** 2))

    dout = 2.0 * net._pool(p1)[0]              # d(sum(pool^2))/d(pool)
    ana = net._pool_backward(dout, p1)

    eps = 1e-3
    num = np.zeros_like(p1)
    idx = rng.choice(p1.size, size=12, replace=False)
    flat = p1.reshape(-1)
    for k in idx:
        orig = flat[k]
        flat[k] = orig + eps
        fp = f(p1)
        flat[k] = orig - eps
        fm = f(p1)
        flat[k] = orig
        num.reshape(-1)[k] = (fp - fm) / (2 * eps)
    rel = np.abs(ana.reshape(-1)[idx] - num.reshape(-1)[idx]) / (
        np.abs(ana.reshape(-1)[idx]) + np.abs(num.reshape(-1)[idx]) + 1e-8)
    assert rel.max() < 1e-2, f"maxpool 梯度与数值梯度不符，最大相对误差 {rel.max():.3e}"


# ---------------------------------------------------------------- 推理一致性
def test_predict_output_in_tanh_range():
    """推理输出必须与训练目标同值域（[-1,1]）。

    回归测试：早期 `predict()` 返回未过 tanh 的线性输出，导致预测值可达 ±13，
    而真实转向标签在 [-1,1]，训练/推理不一致。
    """
    net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    X, _Y = main_mod.synth_dataset(n=8, seed=0)
    p = net.predict(X.astype(np.float32))
    assert np.all(np.abs(p) <= 1.0), f"推理输出超出 [-1,1]: {p.ravel()}"


def test_cnn_learns_image_to_steer():
    """端到端 CNN 应真正学到"图像 → 转向"：预测与标签强相关、方向判对率高。

    这是端到端模块的核心验收项。为控制纯 numpy 训练耗时，使用较小的样本量。
    """
    X, Y = main_mod.synth_dataset(n=100, seed=0)
    net = SimpleCNN(in_channels=3, hidden=8, out=1,
                    img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    net.train(X, Y.ravel(), epochs=45, lr=0.15, batch=16, verbose=0)

    pred = net.predict(X.astype(np.float32)).ravel()
    y = Y.ravel()
    mae = float(np.mean(np.abs(pred - y)))
    base = float(np.mean(np.abs(y)))                # 恒输出 0 的基线
    corr = float(np.corrcoef(pred, y)[0, 1])
    sign_acc = float(np.mean(np.sign(pred) == np.sign(y)))

    assert mae < base * 0.6, f"MAE({mae:.4f}) 未明显优于零输出基线({base:.4f})，网络没学到映射"
    assert corr > 0.8, f"预测与标签相关性过低 (r={corr:.3f})"
    assert sign_acc > 0.8, f"转向方向判对率过低 ({sign_acc * 100:.1f}%)"


def test_prediction_stable_under_noise():
    """加入轻微噪声后预测不应剧烈跳变（说明网络学到的是结构而非单像素）。"""
    X, Y = main_mod.synth_dataset(n=60, seed=0)
    net = SimpleCNN(in_channels=3, hidden=8, out=1,
                    img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    net.train(X, Y.ravel(), epochs=45, lr=0.15, batch=16, verbose=0)

    rng = np.random.RandomState(1)
    noisy = np.clip(X.astype(np.float32) + rng.normal(0, 6.0, X.shape), 0, 255).astype(np.uint8)
    p0 = net.predict(X.astype(np.float32)).ravel()
    p1 = net.predict(noisy.astype(np.float32)).ravel()
    max_jump = float(np.max(np.abs(p1 - p0)))
    assert max_jump < 0.25, f"轻微噪声导致预测跳变过大: {max_jump:.3f}"


def test_model_save_load_roundtrip():
    """端到端 CNN 存取后预测应一致。"""
    X, Y = main_mod.synth_dataset(n=20, seed=2)
    net = SimpleCNN(in_channels=3, hidden=8, out=1,
                    img=(main_mod.IMG_H, main_mod.IMG_W), seed=0)
    net.train(X, Y.ravel(), epochs=5, lr=0.15, batch=16, verbose=0)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "cnn.json")
        net.save(p)
        loaded = main_mod.load_cnn(p, backend="numpy")
    assert np.allclose(loaded.predict(X.astype(np.float32)),
                       net.predict(X.astype(np.float32)), atol=1e-6), \
        "端到端 CNN 存取后预测不一致"


# ---------------------------------------------------------------- 专家控制器
def test_expert_steer_geometry():
    """专家控制器：正对目标转向为 0；目标在左则左转（正）。"""
    st0, diff0 = main_mod.expert_steer((0.0, 0.0), 0.0, [(10.0, 0.0)])
    assert abs(st0) < 1e-6 and abs(diff0) < 1e-6, f"正对目标应零转向，实际 {st0}"

    st_left, _ = main_mod.expert_steer((0.0, 0.0), 0.0, [(5.0, 5.0)])
    assert st_left > 0, f"目标在左应左转（正），实际 {st_left}"

    st_right, _ = main_mod.expert_steer((0.0, 0.0), 0.0, [(5.0, -5.0)])
    assert st_right < 0, f"目标在右应右转（负），实际 {st_right}"


def test_expert_steer_clipped():
    """专家转向必须落在 [-1,1]。"""
    for wp in ([(0.0, 50.0)], [(0.5, -1.0)], [(100.0, 0.2)]):
        s, _ = main_mod.expert_steer((0.0, 0.0), 0.0, wp)
        assert -1.0 <= s <= 1.0, f"专家转向越界: {s}"


# ---------------------------------------------------------------- 离线取证
def test_offline_demo_runs():
    """离线取证模式应能在无 CARLA / 无 GPU / 无 TensorFlow 环境下跑通并导出图。"""
    with tempfile.TemporaryDirectory() as d:
        rc = main_mod.run_offline_demo(d, epochs=6, n_samples=30)
        assert rc == 0
        produced = sorted(os.listdir(d))
        for need in ("cnn_loss.png", "steer_pred_vs_true.png", "road_samples.png", "cnn_steer.json"):
            assert need in produced, f"缺少产物 {need}: {produced}"


def _run_all():
    fns = sorted(k for k in list(globals()) if k.startswith("test_"))
    passed, failed = 0, []
    for fn in fns:
        try:
            globals()[fn]()
            passed += 1
            print(f"[PASS] {fn}")
        except Exception as exc:  # noqa: BLE001
            failed.append(fn)
            print(f"[FAIL] {fn}: {exc}")
    print(f"\n==== 测试结果：PASS={passed}  FAIL={len(failed)} ====")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
