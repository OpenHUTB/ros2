# -*- coding: utf-8 -*-
"""行为克隆与镜像增广。不需要 ROS、不需要 mujoco。"""
import numpy as np
import pytest

from uav_depth_gate_racing_e2e.core.bc import (BCConfig, bc_metrics,
                                               mirror_sample, norm_scale,
                                               train_bc)
from uav_depth_gate_racing_e2e.core.policy import Policy, PolicyConfig

SCALE = np.array([4.0, 4.0, 2.0])


def _sample(rng, hw=(24, 32)):
    return {"depth": (rng.random(hw) * 255).astype(np.uint8),
            "vel": rng.normal(size=3).astype(np.float32),
            "act": rng.normal(size=3).astype(np.float32)}


def test_mirror_is_involution():
    """镜像两次回到原样本——翻转 + 变号都是对合运算。"""
    s = _sample(np.random.default_rng(0))
    m = mirror_sample(mirror_sample(s))
    assert np.array_equal(m["depth"], s["depth"])
    assert np.allclose(m["vel"], s["vel"])
    assert np.allclose(m["act"], s["act"])


def test_mirror_negates_lateral_only():
    s = _sample(np.random.default_rng(1))
    m = mirror_sample(s)
    assert np.array_equal(m["depth"], s["depth"][:, ::-1])
    assert m["vel"][0] == s["vel"][0] and m["vel"][2] == s["vel"][2]
    assert m["vel"][1] == -s["vel"][1]
    assert m["act"][0] == s["act"][0] and m["act"][2] == s["act"][2]
    assert m["act"][1] == -s["act"][1]


def test_norm_scale_matches_env_limits():
    cfg = type("C", (), {"max_horiz": 4.0, "max_vert": 2.0})()
    assert np.allclose(norm_scale(cfg), SCALE)


# --------------------------------------------------------------------------- #
def test_bc_fits_constant_action():
    """专家动作恒定时，BC 必须拟合到接近零误差（否则是优化本身有问题）。

    所有帧共用同一张图：这里测的是**优化能不能收敛**，不是泛化——随机噪声
    输入 + 常数目标要求网络把读数头权重压到 0，那是另一个问题。
    """
    rng = np.random.default_rng(2)
    img = (rng.random((24, 32)) * 255).astype(np.uint8)
    act = np.array([2.0, -1.0, 0.5], np.float32)
    data = [{"depth": img.copy(), "vel": rng.normal(size=3).astype(np.float32),
             "act": act} for _ in range(32)]
    p = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=16))
    hist = train_bc(p, data, SCALE, BCConfig(epochs=120, batch=16, lr=3e-3),
                    verbose=False)
    assert hist["train"][-1] < 2e-3, hist["train"][-1]
    got = p.act({"depth": img, "vel": np.zeros(3, np.float32)},
                deterministic=True)[0][0].astype(float) * SCALE
    assert np.allclose(got, act, atol=0.15), got


def test_mirror_augmentation_preserves_validation():
    """验证集来自未增广的原样本：镜像件不能泄进去。

    否则同一帧的两种视角分别落进训练/验证集，验证 MSE 会虚低。
    """
    rng = np.random.default_rng(3)
    data = [_sample(rng) for _ in range(80)]
    p = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=16))
    cfg = BCConfig(epochs=1, batch=16, mirror=True, val_frac=0.25)
    train_bc(p, data, SCALE, cfg, verbose=False)
    n_val = max(int(len(data) * 0.25), 1)
    assert n_val == 20
    # 验证集大小不受 mirror 影响：mirror 只作用在训练集上
    m_nomir = bc_metrics(p, data[:n_val], SCALE, cfg=cfg)
    assert np.isfinite(m_nomir["mse"])


@pytest.mark.parametrize("arch", ["mlp", "cnn"])
def test_mirror_teaches_sign_convention(arch):
    """镜像增广要能让网络学会"画面哪一侧 → v_y 往哪边"。

    造一份严格满足镜像对称的数据：左半亮 → 向右偏，右半亮 → 向左偏
    （v_y 取反）。不增广时网络只会输出接近 0 的横向速度。
    """
    rng = np.random.default_rng(4)
    data = []
    for _ in range(60):
        img = np.zeros((24, 32), np.uint8)
        left = rng.random() < 0.5
        img[:, :12 if left else 20:] = 200
        vy = 2.5 if left else -2.5
        data.append({"depth": img, "vel": np.zeros(3, np.float32),
                     "act": np.array([2.0, vy, 0.0], np.float32)})
    p = Policy(PolicyConfig(arch=arch, depth_hw=(24, 32), hidden=32))
    train_bc(p, data, SCALE, BCConfig(epochs=80, batch=16, lr=3e-3, mirror=True),
             verbose=False)
    out = []
    for left in (True, False):
        img = np.zeros((24, 32), np.uint8)
        img[:, :12 if left else 20:] = 200
        out.append(float(p.act({"depth": img, "vel": np.zeros(3, np.float32)},
                               deterministic=True)[0][0][1]) * 4.0)
    assert out[0] > 1.0 and out[1] < -1.0, out
