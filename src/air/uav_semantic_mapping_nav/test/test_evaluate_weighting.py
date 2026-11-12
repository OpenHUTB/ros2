# -*- coding: utf-8 -*-
"""评估损失口径的回归测试。

``evaluate()`` / ``evaluate_pairs()`` 报出的 loss 必须与训练时
``loss_and_grad()`` 的逐像素加权交叉熵一致，否则验证损失不能用来判断收敛。
"""
import numpy as np

from uav_semantic_mapping_nav.core.labels import N_CLASS
from uav_semantic_mapping_nav.core.nn import softmax
from uav_semantic_mapping_nav.core.segnet import SegConfig, SegNet, class_weights
from uav_semantic_mapping_nav.core.training import (
    SegTrainConfig,
    evaluate,
    evaluate_pairs,
)


class _Episode:
    def __init__(self, seed=0):
        rng = np.random.default_rng(seed)
        self.rgb = rng.integers(0, 255, (2, 32, 32, 3), dtype=np.uint8)
        seg = np.ones((2, 32, 32), np.int32)  # 地面为主
        seg[:, :4, :4] = 2                    # 少量建筑
        seg[1, :2, :2] = 3                    # 更少的植被
        self.seg = seg

    def __len__(self):
        return len(self.rgb)


class _Dataset:
    def __init__(self):
        self.episodes = [_Episode()]


def _make():
    ds = _Dataset()
    cfg = SegConfig(in_hw=(32, 32), n_class=N_CLASS, ch=(4, 8, 16), seed=0)
    return ds, SegNet(cfg)


def _train_loss(net, ep, wgt):
    rgb = ep.rgb.astype(np.float32) / 255.0
    return float(net.loss_and_grad(rgb, ep.seg, weight=wgt)[0])


def test_evaluate_pairs_matches_training_loss():
    ds, net = _make()
    ep = ds.episodes[0]
    counts = np.bincount(ep.seg.ravel(), minlength=N_CLASS)
    wgt = class_weights(counts, 1.0)

    got = evaluate_pairs(net, ds, [(0, 0), (0, 1)], (32, 32), wgt)["loss"]
    want = _train_loss(net, ep, wgt)
    assert abs(got - want) < 1e-9, (got, want)


def test_evaluate_pairs_without_weights_matches_unweighted_loss():
    ds, net = _make()
    ep = ds.episodes[0]

    got = evaluate_pairs(net, ds, [(0, 0), (0, 1)], (32, 32), None)["loss"]
    want = _train_loss(net, ep, None)
    assert abs(got - want) < 1e-9, (got, want)


def test_evaluate_matches_training_loss_with_weights():
    ds, net = _make()
    ep = ds.episodes[0]
    counts = np.bincount(ep.seg.ravel(), minlength=N_CLASS)
    wgt = class_weights(counts, 1.0)

    cfg = SegTrainConfig(in_hw=(32, 32), weight_power=1.0)
    got = evaluate(net, ds, [0], cfg)["loss"]
    want = _train_loss(net, ep, wgt)
    assert abs(got - want) < 1e-9, (got, want)


def test_evaluate_respects_weight_power():
    """weight_power 必须影响评估损失，否则该配置项是死的。"""
    ds, net = _make()
    cfg0 = SegTrainConfig(in_hw=(32, 32), weight_power=0.0)
    cfg1 = SegTrainConfig(in_hw=(32, 32), weight_power=1.0)

    l0 = evaluate(net, ds, [0], cfg0)["loss"]
    l1 = evaluate(net, ds, [0], cfg1)["loss"]
    assert abs(l0 - l1) > 1e-6, (l0, l1)


def test_weighted_loss_value_and_gradient_agree():
    """加权损失的值与梯度必须对应同一个目标函数。"""
    ds, net = _make()
    ep = ds.episodes[0]
    rgb = ep.rgb.astype(np.float32) / 255.0
    seg = ep.seg
    counts = np.bincount(seg.ravel(), minlength=N_CLASS)
    wgt = class_weights(counts, 1.0)

    net.zero_grad()
    loss, d = net.loss_and_grad(rgb, seg, weight=wgt)

    p = softmax(net.forward(rgb), axis=1)
    n, c, h, w = p.shape
    onehot = np.zeros_like(p)
    onehot[np.arange(n)[:, None, None], seg,
           np.arange(h)[None, :, None], np.arange(w)[None, None, :]] = 1.0
    w_pix = (onehot * wgt.reshape(1, c, 1, 1)).sum(axis=1, keepdims=True)
    n_pix = float(n * h * w)

    want = float(-np.sum(np.log(np.maximum(
        (p * onehot).sum(axis=1, keepdims=True), 1e-12)) * w_pix) / n_pix)
    assert abs(loss - want) < 1e-9, (loss, want)
    assert np.allclose(d, (p - onehot) * w_pix / n_pix, atol=1e-12)
