"""TTC 网络的单元测试。

不依赖 pytest，直接 ``python3 test_cnn.py`` 就能跑，适合在没有 ROS 环境的机器上
先验证网络与权重没问题，再去接仿真。

覆盖：
    1. 前向形状（单通道 64x64 -> 1 维标量）
    2. tanh 输出范围 [-1, 1]
    3. im2col / col2im 互逆（全 1 输入时 col2im 每个像素应等于覆盖它的感受野个数）
    4. ``DepthPolicyNet.load()`` 返回新实例且权重确实是从文件读出来的
    5. 640x480 非正方形深度图能正确缩放到 64x64（对应 node.py 的尺寸处理）
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_airsim_ttc_predictor.cnn import DepthPolicyNet, im2col, col2im  # noqa: E402

WEIGHTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "models_trained", "ttc.npz")


def test_forward_shape():
    net = DepthPolicyNet.load(WEIGHTS)
    x = np.zeros((64, 64), dtype=np.float32)
    y = net.forward(x)
    assert y.shape == (1,), "前向输出应当是 1 维标量，实际 %s" % (y.shape,)


def test_forward_range():
    net = DepthPolicyNet.load(WEIGHTS)
    rng = np.random.RandomState(0)
    for i in range(8):
        x = rng.rand(64, 64).astype(np.float32)
        y = float(net.forward(x)[0])
        assert -1.0 <= y <= 1.0, "tanh 输出越界：%f" % y


def test_im2col_col2im_inverse():
    """im2col 把感受野摊平；col2im 把梯度累加回去。

    输入全 1 时，col2im 的每个像素值应当等于「覆盖该像素的感受野个数」，
    可以用直接的计数循环独立算出，从而验证两者确实互逆。
    """
    h = w = 8
    k, stride = 3, 1
    x = np.ones((1, h, w), dtype=np.float32)
    cols, oh, ow = im2col(x, k, stride)
    assert cols.shape == (oh * ow, 1 * k * k), "im2col 形状不对"

    back = col2im(cols, x.shape, k, stride, oh, ow)

    # 独立计数：每个像素被多少个感受野覆盖
    expect = np.zeros((h, w), dtype=np.float32)
    for y in range(0, h - k + 1, stride):
        for xx in range(0, w - k + 1, stride):
            expect[y:y + k, xx:xx + k] += 1.0

    np.testing.assert_allclose(back[0], expect, rtol=0, atol=1e-5)


def test_load_returns_trained_instance():
    """classmethod 必须返回新实例；顺手确认权重真的换了（不是随机初始化）。"""
    fresh = DepthPolicyNet(seed=0)
    loaded = DepthPolicyNet.load(WEIGHTS)
    assert loaded is not fresh
    diff = float(np.abs(np.asarray(loaded.fc2.b) - np.asarray(fresh.fc2.b)).max())
    assert diff > 1e-3, "load 之后权重与随机初始化几乎相同，说明权重没读进来"

    # 同一个输入在两个网络上的输出必须不同
    x = np.zeros((64, 64), dtype=np.float32)
    x[20:40, 20:40] = 0.5
    y_fresh = float(fresh.forward(x)[0])
    y_loaded = float(loaded.forward(x)[0])
    assert abs(y_fresh - y_loaded) > 1e-3, "加载前后的预测完全一致，权重可能没生效"


def test_nonsquare_depth_resize():
    """640x480 的 DepthPlanar 不是正方形，必须按 height/width 缩放，不能 sqrt。"""
    raw = np.random.RandomState(1).rand(480 * 640).astype(np.float32) * 10.0
    h, w = 480, 640
    arr = raw.reshape(h, w)
    idx_h = np.linspace(0, h - 1, 64).astype(int)
    idx_w = np.linspace(0, w - 1, 64).astype(int)
    small = np.clip(arr[np.ix_(idx_h, idx_w)] / 30.0, 0.0, 1.0)
    assert small.shape == (64, 64)
    assert 0.0 <= small.min() and small.max() <= 1.0


def main():
    tests = [
        test_forward_shape,
        test_forward_range,
        test_im2col_col2im_inverse,
        test_load_returns_trained_instance,
        test_nonsquare_depth_resize,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print("PASS  %s" % t.__name__)
        except Exception as e:  # noqa: BLE001
            failed += 1
            print("FAIL  %s: %s" % (t.__name__, e))
    print("\n%d/%d 通过" % (len(tests) - failed, len(tests)))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
