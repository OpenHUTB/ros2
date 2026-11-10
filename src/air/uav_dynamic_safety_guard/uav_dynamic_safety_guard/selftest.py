#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自检：不需要 MuJoCo、不需要 ROS、不需要显示器，只要 numpy。

覆盖几何间距、距离栅格、动态障碍场、候选集过滤、网络前向/存取、训练回路。
"""
from __future__ import annotations

import math
import tempfile
import os
from typing import Callable, List, Tuple

import numpy as np

from .core import geometry as G
from .core import ranges as R
from .core.field import Mover, safety_field, sample_endpoints
from .core.filter import (BrakeFilter, FilterConfig, NoFilter, candidates,
                          select)
from .core.network import GuardNet, NetConfig, SEQ_IN, SINGLE_IN
from .core.nn import Adam, Linear, MLP
from .core.samples import (build_xy, empty, mirror_dataset, slice_features)
from .core.training import TrainConfig, fit_norm, split_episodes, train_policy

__all__ = ["CHECKS", "run"]

CHECKS: List[Tuple[str, Callable[[], None]]] = []
_ok = True


def check(fn: Callable[[], None]) -> Callable[[], None]:
    CHECKS.append((fn.__name__, fn))
    return fn


def _close(a: float, b: float, tol: float = 1e-6) -> None:
    if abs(float(a) - float(b)) > tol:
        raise AssertionError("期望 %.6f，实际 %.6f" % (b, a))


# --------------------------------------------------------------------------- #
# 距离栅格
# --------------------------------------------------------------------------- #
@check
def ranges_grid_shape() -> None:
    g = R.points_to_grid(np.zeros((0, 3)))
    assert g.shape == (R.N_EL, R.N_AZ), g.shape
    assert np.allclose(g, R.MAX_RANGE)


@check
def ranges_point_lands_in_right_cell() -> None:
    pts = np.array([[2.0, 0.0, 0.0]])          # 正前方、0 度俯仰
    g = R.points_to_grid(pts)
    ai = int(round((0.0 + math.pi) / (2 * math.pi) * R.N_AZ)) % R.N_AZ
    assert abs(float(g[1, ai]) - 2.0) < 1e-6, g[1, ai]
    assert np.sum(g < R.MAX_RANGE) == 1


@check
def ranges_elevation_layers() -> None:
    up = R.points_to_grid(np.array([[2.0, 0.0, 1.2]]))     # 约 +31 度 → 顶层
    down = R.points_to_grid(np.array([[2.0, 0.0, -1.2]]))  # → 底层
    assert np.min(up[2]) < R.MAX_RANGE
    assert np.min(down[0]) < R.MAX_RANGE


@check
def ranges_feature_dims() -> None:
    hist = np.full((R.HISTORY, R.N_EL, R.N_AZ), 3.0, np.float32)
    f_single = R.build_features(hist[0], np.zeros(3), np.array([1.0, 0, 0]), 5, 1.6)
    f_seq = R.build_sequence_features(hist, np.zeros(3), np.array([1.0, 0, 0]), 5, 1.6)
    assert f_single.shape == (R.FEAT_DIM,) == (116,)
    assert f_seq.shape == (R.FEAT_DIM_SEQ,) == (332,)


@check
def ranges_mirror_is_involution() -> None:
    g = np.arange(R.N_EL * R.N_AZ, dtype=np.float32).reshape(R.N_EL, R.N_AZ)
    assert np.allclose(R.mirror_grid(R.mirror_grid(g)), g)
    assert not np.allclose(R.mirror_grid(g), g)


@check
def ranges_corrupt_respects_range() -> None:
    g = np.full((R.N_EL, R.N_AZ), 2.0, np.float32)
    rng = np.random.default_rng(0)
    c = R.corrupt_grid(g, rng, sigma=0.5, drop=0.5)
    assert c.min() >= 0.0 and c.max() <= R.MAX_RANGE


@check
def ranges_direction_range_zero_command() -> None:
    g = np.full((R.N_EL, R.N_AZ), 2.0, np.float32)
    assert R.direction_range(g, np.array([0.0, 0.0, 1.0])) == (1.0, 1.0)


@check
def ranges_lower_bound_shrinks_with_speed() -> None:
    g = np.full((R.N_EL, R.N_AZ), 4.0, np.float32)
    slow = R.lidar_lower_bound(g, np.array([1.0, 0.0, 0.0]), 0.6)
    fast = R.lidar_lower_bound(g, np.array([3.0, 0.0, 0.0]), 0.6)
    assert fast < slow


# --------------------------------------------------------------------------- #
# 几何间距
# --------------------------------------------------------------------------- #
@check
def geometry_box_sdf() -> None:
    p = np.array([[0.0, 0.0, 0.5]])
    d = float(G.box_sdf(p, np.array([2.0, 0.0, 0.5]), np.array([0.5, 0.5, 0.5]))[0])
    _close(d, 1.5)


@check
def geometry_cylinder_sdf() -> None:
    p = np.array([[0.0, 0.0, 1.0]])
    d = float(G.cylinder_sdf(p, np.array([1.0, 0.0, 1.0]), 0.25, 1.0)[0])
    _close(d, 0.75)


@check
def geometry_sphere_sdf() -> None:
    p = np.array([[0.0, 0.0, 0.0]])
    d = float(G.sphere_sdf(p, np.array([0.0, 0.0, 2.0]), 0.5)[0])
    _close(d, 1.5)


@check
def geometry_floor_dominates() -> None:
    prims = [("box", np.array([10.0, 0.0, 0.5]), np.array([0.5, 0.5, 0.5]))]
    _close(G.clearance_at(prims, [0.0, 0.0, 1.0]), 1.0 - G.DRONE_RADIUS)


@check
def geometry_batch_matches_scalar() -> None:
    rng = np.random.default_rng(3)
    prims = [("box", np.array([1.0, 0.5, 0.6]), np.array([0.3, 0.4, 0.6])),
             ("cylinder", np.array([-1.2, 0.2, 0.5]), np.array([0.25, 0.5])),
             ("sphere", np.array([0.3, -1.0, 1.2]), np.array([0.35]))]
    pts = rng.uniform(-2.0, 2.0, (25, 3))
    a = G.clearance_points(prims, pts)
    b = G.clearance_points_batch(G.stack_batch(prims), pts)
    assert np.allclose(a, b, atol=1e-9)


@check
def geometry_flight_through_mover_is_unsafe() -> None:
    mover = Mover("m", (0.0, 0.0), 2.0, 0.5, 0.0, (0.2, 0.9))
    batch = G.stack_batch([mover.prim_at(0.0)])
    p = np.array([0.0, 0.0, 1.6])
    still = G.rollout_clearance(lambda t: batch, p, np.zeros((1, 3)), 0.0,
                                np.zeros((1, 3)), horizon=0.6, acc_max=0.0)
    # z=1.6 时地面比侧面更近：min(1.6, 1.8) - 机体半径
    _close(float(still[0]), 1.6 - G.DRONE_RADIUS)


@check
def geometry_rollout_decreases_toward_wall() -> None:
    wall = G.stack_batch([("box", np.array([1.5, 0.0, 1.0]),
                          np.array([0.2, 6.0, 1.0]))])
    p = np.array([0.0, 0.0, 1.6])
    vs = np.zeros((2, 3))
    us = np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    q = G.rollout_clearance(lambda t: wall, p, vs, 0.0, us, horizon=0.6,
                            acc_max=4.0)
    assert q[0] > q[1], q


# --------------------------------------------------------------------------- #
# 动态障碍场
# --------------------------------------------------------------------------- #
@check
def field_is_deterministic() -> None:
    a = safety_field(seed=5, n_moving=4)
    b = safety_field(seed=5, n_moving=4)
    assert len(a.static) == len(b.static) == 18
    assert np.allclose(a.mover_positions(1.7), b.mover_positions(1.7))


@check
def field_movers_keep_circle_radius() -> None:
    fld = safety_field(seed=2, n_moving=4)
    for m in fld.movers:
        for t in (0.0, 1.0, 4.5):
            p = m.pos_at(t)
            _close(math.hypot(p[0] - m.center[0], p[1] - m.center[1]),
                   m.radius, 1e-9)


@check
def field_batch_changes_with_time() -> None:
    fld = safety_field(seed=1, n_moving=5)
    assert not np.allclose(fld.batch_at(0.0)["cyl_pos"],
                           fld.batch_at(2.0)["cyl_pos"])


@check
def field_clearance_drops_near_mover() -> None:
    fld = safety_field(seed=7, n_moving=4)
    m = fld.movers[0]
    p = m.pos_at(0.3)
    near = fld.clearance_at(p + np.array([0.0, 0.0, 0.0]), 0.3)
    far = fld.clearance_at(p + np.array([0.0, 6.0, 0.0]), 0.3)
    assert near < far


@check
def field_mover_speeds_in_range() -> None:
    fld = safety_field(seed=9, n_moving=6)
    s = fld.speeds()
    assert s.min() >= 0.4 - 1e-9 and s.max() <= 1.0 + 1e-9, s


@check
def field_endpoints_are_clear() -> None:
    fld = safety_field(seed=11, n_moving=5)
    start, goal = sample_endpoints(fld, np.random.default_rng(0))
    assert fld.clearance_at(start, 0.0) > 0.5
    assert fld.clearance_at(goal, 0.0) > 0.4
    assert np.linalg.norm(goal - start) > 8.0


# --------------------------------------------------------------------------- #
# 候选集与选择
# --------------------------------------------------------------------------- #
@check
def filter_candidates_dedup_and_clip() -> None:
    cfg = FilterConfig()
    c = candidates(np.array([2.0, 0.0, 0.0]), cfg)
    assert len(c) > 60
    assert np.max(np.abs(c[:, 0])) <= cfg.max_horiz + 1e-9
    assert np.max(np.abs(c[:, 1])) <= 3.0 + 1e-9
    rounded = np.unique(np.round(c, 3), axis=0)
    assert len(rounded) == len(c)


@check
def filter_select_prefers_feasible_near_nominal() -> None:
    cfg = FilterConfig(margin=0.8)
    u_nom = np.array([2.0, 0.0, 0.0])
    c = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0]])
    q = np.array([0.5, 0.9, 1.5])
    u, diag = select(u_nom, c, 0.3, q, cfg)
    assert np.allclose(u, [2.0, 0.0, 0.0]), u
    assert diag["n_feasible"] == 2.0


@check
def filter_select_fallback_maximizes_clearance() -> None:
    cfg = FilterConfig(margin=0.8)
    u_nom = np.array([2.0, 0.0, 0.0])
    c = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0]])
    q = np.array([0.3, 0.2, 0.7])
    u, diag = select(u_nom, c, 0.1, q, cfg)
    assert np.allclose(u, [1.0, 1.0, 0.0]), u
    assert diag["n_feasible"] == 0.0


@check
def filter_none_and_brake() -> None:
    obs = {"ranges": np.full((R.N_EL, R.N_AZ), 3.0, np.float32), "h": 1.0}
    u_nom = np.array([1.5, 0.0, 0.0])
    assert np.allclose(NoFilter().act(obs, u_nom)[0], u_nom)
    assert np.allclose(BrakeFilter(1.2).act(obs, u_nom)[0], u_nom)
    obs["ranges"][:] = 0.5
    u = BrakeFilter(1.2).act(obs, u_nom)[0]
    assert np.linalg.norm(u) < np.linalg.norm(u_nom)


# --------------------------------------------------------------------------- #
# 网络
# --------------------------------------------------------------------------- #
@check
def nn_linear_shapes() -> None:
    lin = Linear(4, 3, rng=np.random.default_rng(0))
    y = lin.forward(np.ones((5, 4)))
    assert y.shape == (5, 3)
    lin.backward(np.ones((5, 3)))
    assert lin.grads[0].shape == (4, 3)


@check
def nn_mlp_forward_backward() -> None:
    mlp = MLP([8, 16, 3], rng=np.random.default_rng(1))
    y, masks = mlp.forward(np.ones((4, 8)), cache=True)
    assert y.shape == (4, 3) and len(masks) == 1
    mlp.backward(np.ones((4, 3)), masks)
    assert np.any(mlp.layers[0].grads[0] != 0.0)


@check
def network_forward_shape() -> None:
    net = GuardNet(NetConfig(in_dim=SEQ_IN, hidden=32, depth=2, seed=0))
    y = net.forward(np.zeros((7, SEQ_IN + 3)))
    assert y.shape == (7, 3)
    assert net.mlp.dims[0] == SEQ_IN + 3
    assert net.n_params() > SEQ_IN * 32


@check
def network_save_load_roundtrip() -> None:
    net = GuardNet(NetConfig(in_dim=SINGLE_IN, hidden=24, depth=2, seed=4))
    net.set_norm(np.zeros(SINGLE_IN + 3), np.ones(SINGLE_IN + 3) * 2.0)
    x = np.random.default_rng(0).normal(size=(3, SINGLE_IN + 3))
    y = net.forward(x)
    p = os.path.join(os.getcwd(), "_selftest_net.npz")
    try:
        net.save(p, meta={"stage": "test"})
        net2 = GuardNet.load(p)
    finally:
        if os.path.exists(p):
            os.remove(p)
    assert np.allclose(net2.forward(x), y)
    assert np.allclose(net2.sd, net.sd)


@check
def network_one_step_reduces_loss() -> None:
    net = GuardNet(NetConfig(in_dim=8, hidden=16, depth=2, seed=0))
    rng = np.random.default_rng(0)
    X = rng.normal(size=(32, 11))
    Y = np.tile(np.array([1.0, 0.0, 0.0]), (32, 1))
    opt = net.optimizer(lr=1e-2)
    def loss() -> float:
        e = net.forward(X) - Y
        return float(np.mean(np.sum(e * e, axis=1)))
    before = loss()
    for _ in range(20):
        pred, masks = net.forward(X, cache=True)
        diff = pred - Y
        net.backward(2.0 * diff / diff.size, masks)
        opt.step()
    assert loss() < before * 0.5, (before, loss())


@check
def network_adam_state_shapes() -> None:
    lin = Linear(3, 2, rng=np.random.default_rng(0))
    opt = Adam([lin], lr=1e-3)
    lin.forward(np.ones((2, 3)))
    lin.backward(np.ones((2, 2)))
    g = opt.step()
    assert g > 0.0 and opt.t == 1


# --------------------------------------------------------------------------- #
# 数据与训练
# --------------------------------------------------------------------------- #
def _toy_dataset(n_frames: int = 40, n_eps: int = 4) -> dict:
    rng = np.random.default_rng(0)
    d = empty(n_frames)
    d["hist"] = rng.uniform(0.5, 8.0, (n_frames, R.HISTORY, R.N_EL, R.N_AZ)
                            ).astype(np.float32)
    d["vel"] = rng.normal(scale=0.5, size=(n_frames, 3)).astype(np.float32)
    d["goal_dir"] = np.tile(np.array([1.0, 0.0, 0.0], np.float32),
                            (n_frames, 1))
    d["dist"] = rng.uniform(1.0, 12.0, n_frames).astype(np.float32)
    d["z"] = np.full(n_frames, 1.6, np.float32)
    d["u_nom"] = np.tile(np.array([1.8, 0.0, 0.0], np.float32), (n_frames, 1))
    d["u_tgt"] = np.clip(d["u_nom"] * 0.5, -4, 4).astype(np.float32)
    d["h"] = rng.uniform(0.8, 3.0, n_frames).astype(np.float32)
    d["episode"] = np.repeat(np.arange(n_eps), n_frames // n_eps).astype(np.int32)
    d["seed"] = np.zeros(n_frames, np.int32)
    return d


@check
def samples_mirror_doubles_and_flips() -> None:
    d = _toy_dataset(20, 2)
    m = mirror_dataset(d)
    assert len(m["vel"]) == 2 * len(d["vel"])
    assert np.allclose(m["vel"][len(d["vel"]):, 1], -d["vel"][:, 1])
    assert len(np.unique(m["episode"])) == 2 * len(np.unique(d["episode"]))
    assert not np.allclose(m["hist"][len(d["hist"]):], d["hist"])


@check
def samples_feature_shapes() -> None:
    d = _toy_dataset(12, 3)
    assert slice_features(d, temporal=True).shape == (12, 332)
    assert slice_features(d, temporal=False).shape == (12, 116)
    X, Y = build_xy(d, temporal=True)
    assert X.shape == (12, 335) and Y.shape == (12, 3)


@check
def training_split_is_disjoint() -> None:
    ep = np.repeat(np.arange(10), 5)
    tr, va = split_episodes(ep, 0.2, np.random.default_rng(0))
    assert len(set(tr.tolist()) & set(va.tolist())) == 0
    assert len(tr) + len(va) == len(ep)
    assert len(np.unique(ep[va])) == 2


@check
def training_norm_is_standardized() -> None:
    net = GuardNet(NetConfig(in_dim=8, hidden=8, seed=0))
    X = np.random.default_rng(0).normal(size=(64, 11)) * 3.0 + 1.5
    fit_norm(net, X)
    assert np.allclose(np.mean(X, axis=0), net.mu)
    Z = (X - net.mu) / net.sd
    assert np.allclose(np.std(Z, axis=0), 1.0, atol=1e-6)


@check
def training_loop_lowers_loss() -> None:
    d = _toy_dataset(48, 6)
    X, Y = build_xy(d, temporal=False)
    net = GuardNet(NetConfig(in_dim=SINGLE_IN, hidden=24, depth=2, seed=0))
    hist = train_policy(net, X, Y, d["episode"],
                        TrainConfig(epochs=25, lr=3e-3, batch=16, val_frac=0.2,
                                    verbose=False))
    assert hist["train"][-1] < hist["train"][0] * 0.8, hist["train"][:3]


@check
def training_val_is_finite() -> None:
    d = _toy_dataset(32, 4)
    X, Y = build_xy(d, temporal=False)
    net = GuardNet(NetConfig(in_dim=SINGLE_IN, hidden=16, seed=0))
    hist = train_policy(net, X, Y, d["episode"],
                        TrainConfig(epochs=6, batch=8, verbose=False,
                                    log_every=1))
    assert np.isfinite(hist["best_val"][0])


def run(verbose: bool = False) -> Tuple[bool, str]:
    """跑全部检查，返回 (是否全过, 报告文本)。"""
    lines: List[str] = []
    n_fail = 0
    for name, fn in CHECKS:
        try:
            fn()
            lines.append("  [ok]   %s" % name)
        except Exception as exc:                          # noqa: BLE001
            n_fail += 1
            lines.append("  [FAIL] %s: %s: %s"
                         % (name, type(exc).__name__, exc))
    head = "自检 %d 项：%d 通过，%d 失败" % (len(CHECKS), len(CHECKS) - n_fail,
                                            n_fail)
    return n_fail == 0, head + "\n" + "\n".join(lines)
