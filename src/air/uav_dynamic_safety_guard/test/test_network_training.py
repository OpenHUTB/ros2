# -*- coding: utf-8 -*-
"""网络、数据增广与训练回路的单元测试。"""
import os

import numpy as np
import pytest

from uav_dynamic_safety_guard.core.network import (SINGLE_IN, SEQ_IN, GuardNet,
                                                   NetConfig)
from uav_dynamic_safety_guard.core.samples import (build_xy, empty,
                                                   mirror_dataset,
                                                   slice_features)
from uav_dynamic_safety_guard.core.training import (TrainConfig, fit_norm,
                                                    offline_metrics,
                                                    split_episodes, train_policy)


def _toy(n=40, eps=4, seed=0):
    rng = np.random.default_rng(seed)
    from uav_dynamic_safety_guard.core import ranges as R
    d = empty(n)
    d["hist"] = rng.uniform(0.5, 8.0, (n, R.HISTORY, R.N_EL, R.N_AZ)).astype(np.float32)
    d["vel"] = rng.normal(scale=0.5, size=(n, 3)).astype(np.float32)
    d["goal_dir"] = np.tile(np.array([1.0, 0.0, 0.0], np.float32), (n, 1))
    d["dist"] = rng.uniform(1.0, 12.0, n).astype(np.float32)
    d["z"] = np.full(n, 1.6, np.float32)
    d["u_nom"] = np.tile(np.array([1.8, 0.0, 0.0], np.float32), (n, 1))
    d["u_tgt"] = (d["u_nom"] * 0.5).astype(np.float32)
    d["h"] = rng.uniform(0.8, 3.0, n).astype(np.float32)
    d["episode"] = np.repeat(np.arange(eps), n // eps).astype(np.int32)
    d["seed"] = np.zeros(n, np.int32)
    return d


def test_network_input_dim_includes_command():
    net = GuardNet(NetConfig(in_dim=SEQ_IN, hidden=16, seed=0))
    assert net.mlp.dims[0] == SEQ_IN + 3
    assert net.forward(np.zeros((2, SEQ_IN + 3))).shape == (2, 3)


def test_network_rejects_wrong_norm_length():
    net = GuardNet(NetConfig(in_dim=SEQ_IN, hidden=8, seed=0))
    with pytest.raises(ValueError):
        net.set_norm(np.zeros(SEQ_IN), np.ones(SEQ_IN))


def test_network_param_count():
    net = GuardNet(NetConfig(in_dim=SINGLE_IN, hidden=32, depth=2, seed=0))
    assert net.n_params() == (SINGLE_IN + 3) * 32 + 32 + 32 * 32 + 32 + 32 * 3 + 3


def test_save_load_roundtrip(tmp_path=None):
    net = GuardNet(NetConfig(in_dim=SINGLE_IN, hidden=12, seed=3))
    net.set_norm(np.arange(SINGLE_IN + 3) * 0.1, np.ones(SINGLE_IN + 3))
    x = np.random.default_rng(1).normal(size=(4, SINGLE_IN + 3))
    y = net.forward(x)
    path = os.path.join(os.getcwd(), "_tmp_guard_test.npz")
    try:
        net.save(path, meta={"stage": "unit"})
        net2 = GuardNet.load(path)
        assert np.allclose(net2.forward(x), y)
        assert np.allclose(net2.sd, net.sd)
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_mirror_doubles_and_flips_y():
    d = _toy(20, 2)
    m = mirror_dataset(d)
    n = len(d["vel"])
    assert len(m["vel"]) == 2 * n
    assert np.allclose(m["vel"][n:, 1], -d["vel"][:, 1])
    assert np.allclose(m["u_tgt"][n:, 1], -d["u_tgt"][:, 1])
    assert np.allclose(m["hist"][n:], d["hist"][..., _mirror_idx()])


def _mirror_idx():
    from uav_dynamic_safety_guard.core.ranges import MIRROR_IDX
    return MIRROR_IDX


def test_feature_slices_have_expected_width():
    d = _toy(12, 3)
    assert slice_features(d, temporal=True).shape == (12, SEQ_IN)
    assert slice_features(d, temporal=False).shape == (12, SINGLE_IN)
    X, Y = build_xy(d, temporal=False)
    assert X.shape == (12, SINGLE_IN + 3)
    assert Y.shape == (12, 3)


def test_split_by_episode_is_disjoint():
    ep = np.repeat(np.arange(10), 5)
    tr, va = split_episodes(ep, 0.2, np.random.default_rng(0))
    assert len(set(tr.tolist()) & set(va.tolist())) == 0
    assert len(np.unique(ep[va])) == 2


def test_split_handles_single_episode():
    ep = np.zeros(6, np.int32)
    tr, va = split_episodes(ep, 0.2, np.random.default_rng(0))
    assert len(tr) == len(va) == 6


def test_fit_norm_standardizes_columns():
    net = GuardNet(NetConfig(in_dim=8, hidden=8, seed=0))
    X = np.random.default_rng(2).normal(size=(64, 11)) * 2.0 + 0.5
    fit_norm(net, X)
    assert np.allclose(X.mean(axis=0), net.mu)
    assert np.allclose((X - net.mu).std(axis=0) / net.sd, 1.0, atol=1e-6)


def test_training_reduces_loss_on_toy_data():
    d = _toy(48, 6)
    X, Y = build_xy(d, temporal=False)
    net = GuardNet(NetConfig(in_dim=SINGLE_IN, hidden=24, seed=0))
    hist = train_policy(net, X, Y, d["episode"],
                        TrainConfig(epochs=25, lr=3e-3, batch=16, verbose=False))
    assert hist["train"][-1] < hist["train"][0] * 0.8


def test_offline_metrics_close_to_zero_for_constant_target():
    net = GuardNet(NetConfig(in_dim=8, hidden=16, seed=0))
    X = np.zeros((5, 11))
    Y = np.zeros((5, 3))
    m = offline_metrics(net, X, Y)
    assert m["mse"] >= 0.0 and np.isfinite(m["mae"])
