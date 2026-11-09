#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pytest 冒烟测试：核心模块的可运行性与数值性质。

    cd src/uav_neural_residual_control
    pytest -q            # 需要 numpy + mujoco（仿真侧解释器）

需要 ROS 的部分不在这里，见 README 的"验证"一节。
"""
import os
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.dirname(_HERE),
           os.path.join(os.path.dirname(os.path.dirname(_HERE)), "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_neural_residual_control.core import (          # noqa: E402
    Dataset, EpisodeCollector, Gains, MLP, ResidualNet, ResidualPlant,
    RecursiveLeastSquares, make_plant, make_tracker, make_trajectory,
    run_once)
from uav_neural_residual_control.core.disturbances import (  # noqa: E402
    DragModel, PayloadModel, PlantConfig, WindModel, make_plant as mk_plant)
from uav_neural_residual_control.core.residual import (      # noqa: E402
    FEATURE_DIM, build_features)


# --------------------------------------------------------------------------- #
# 扰动模型
# --------------------------------------------------------------------------- #
def test_drag_is_dissipative():
    d = DragModel()
    for v in ([0.5, 0, 0], [0, -2, 0], [0, 0, 3], [1, 1, -1]):
        f = d.body_force(np.asarray(v, float))
        assert float(np.dot(f, v)) < 0, "阻力必须与速度反向"


def test_wind_ou_statistics():
    w = WindModel(mean=(2.0, 0.0, 0.0), gust=1.2, seed=7)
    w.reset()
    xs = np.empty(120000)
    for i in range(len(xs)):
        w.update(0.01)
        xs[i] = w.vel[0]
    assert abs(xs.mean() - 2.0) < 0.2
    assert abs(xs.std() - 1.2) < 0.15


def test_payload_ratio():
    assert PayloadModel(mass=0.475).ratio == pytest.approx(0.5)
    assert PlantConfig().ground_scale(0.0) > PlantConfig().ground_scale(1.0)
    assert PlantConfig().ground_scale(5.0) < 1e-8


# --------------------------------------------------------------------------- #
# 被控对象
# --------------------------------------------------------------------------- #
def test_payload_injected_into_model():
    p0 = ResidualPlant(plant=make_plant("none", "calm"), scene="open_ground")
    p1 = ResidualPlant(plant=make_plant("heavy", "calm"), scene="open_ground")
    assert p1.true_mass == pytest.approx(p0.true_mass + 0.5, abs=1e-6)
    assert p1.nominal_mass == pytest.approx(p0.true_mass, abs=1e-9)
    assert p1.nominal_params.mass == pytest.approx(p0.true_mass, abs=1e-9)


def test_hover_residual_matches_analytic():
    """悬停时垂直残差必须等于 Δm·g/m_nom —— 这是整条链路的定义基准。"""
    pl = ResidualPlant(plant=make_plant("heavy", "calm", ground_effect=0.0),
                       scene="open_ground")
    tj = make_trajectory("hover", pos=(0, 0, 2.0), duration=5.0)
    tk = make_tracker("geo", pl, tj)
    pl.reset(pos=(0, 0, 2.0))
    for k in range(int(5.0 / pl.dt)):
        pl.step_with_control(tk.act(k * pl.dt))
        tk.observe()
    assert tk.last_delta[2] == pytest.approx(-0.5 * 9.81 / 0.95, abs=0.06)


# --------------------------------------------------------------------------- #
# 网络
# --------------------------------------------------------------------------- #
def test_features_layout():
    f = build_features([1, 2, -3], [0.1, 0.2, 0.3], [0, 0, 0], 4.0, 5.0)
    assert f.shape == (FEATURE_DIM,)
    assert f[0:3].tolist() == [1, 2, -3]
    assert f[3] == pytest.approx(1 * np.hypot(1, 2))
    assert f[12] == 4.0 and f[13] == 5.0


def test_zero_init_output_is_identity():
    net = ResidualNet(seed=0)
    X = np.random.default_rng(0).normal(size=(32, FEATURE_DIM))
    assert np.allclose(net.predict_batch(X), 0.0)


def test_gradient_matches_numeric():
    mlp = MLP((5, 7, 3), seed=1, out_gain=1.0)
    rng = np.random.default_rng(2)
    X, Y = rng.normal(size=(11, 5)), rng.normal(size=(11, 3))

    def loss():
        Yh, hs = mlp.forward(X, cache=True)
        d = Yh - Y
        return float(np.mean(d ** 2)), mlp.backward(X, hs, 2.0 * d / d.size)

    _, (gW, _) = loss()
    eps = 1e-6
    mlp.W[1][2, 1] += eps
    l1 = loss()[0]
    mlp.W[1][2, 1] -= eps
    num = (l1 - loss()[0]) / eps
    assert num == pytest.approx(gW[1][2, 1], rel=1e-5)


def test_net_learns_offset_and_linear_part():
    rng = np.random.default_rng(3)
    X = rng.normal(size=(600, FEATURE_DIM))
    Y = np.zeros((600, 3))
    Y[:, 2] = -4.0
    Y[:, 0] = 0.4 * X[:, 0]
    net = ResidualNet(sizes=(FEATURE_DIM, 24, 24, 3), seed=0)
    net.fit_scaler(X)
    for _ in range(150):
        net.train_epoch(X, Y, batch=64, lr=5e-3, seed=0)
    err = np.sqrt(np.mean((net.predict_batch(X) - Y) ** 2))
    assert err < 0.05


def test_clip_bounds_output():
    net = ResidualNet(seed=0)
    big = net.predict_batch(np.full((4, FEATURE_DIM), 1e4))
    assert np.all(np.abs(big) <= ResidualNet.CLIP + 1e-9)


def test_net_roundtrip(tmp_path):
    net = ResidualNet(sizes=(FEATURE_DIM, 8, 3), seed=4)
    net.fit_scaler(np.random.default_rng(0).normal(size=(50, FEATURE_DIM)))
    net.mlp.W[0] += 0.1
    p = str(tmp_path / "n.npz")
    net.save(p, meta={"hello": "世界"})
    back = ResidualNet.load(p)
    X = np.random.default_rng(1).normal(size=(5, FEATURE_DIM))
    assert np.allclose(net.predict_batch(X), back.predict_batch(X))
    assert back.meta["hello"] == "世界"


# --------------------------------------------------------------------------- #
# 在线辨识
# --------------------------------------------------------------------------- #
def test_rls_multi_output_converges():
    rng = np.random.default_rng(0)
    theta = rng.normal(size=(6, 3))
    rls = RecursiveLeastSquares(6, n_out=3, lam=1.0)
    for _ in range(800):
        phi = rng.normal(size=6)
        rls.update(phi, theta.T @ phi)
    assert np.linalg.norm(rls.theta - theta) / np.linalg.norm(theta) < 0.02


def test_rls_skips_unexcited_and_bounds_covariance():
    rls = RecursiveLeastSquares(4, n_out=1, min_reg=0.1)
    for _ in range(50):
        assert not rls.update(np.zeros(4), np.ones(1))
    assert rls.updates == 0 and rls.skipped == 50
    # 长期只喂同一个方向（定点悬停的极端情形）：协方差必须有界，θ 不能跑飞
    rng = np.random.default_rng(1)
    phi = np.array([1.0, 0.0, 0.0, 0.0])
    for _ in range(20000):
        rls.update(phi, np.array([1.0]))
    assert np.trace(rls.P) <= rls.trace_cap * 1.05
    assert np.all(np.abs(rls.theta) < 1e3)


# --------------------------------------------------------------------------- #
# 控制器
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("ctl", ["pid", "geo"])
def test_baselines_track_hover(ctl):
    pl = ResidualPlant(plant=make_plant("none", "calm"), scene="open_ground")
    tj = make_trajectory("hover", pos=(0, 0, 1.5), duration=6.0)
    tk = make_tracker(ctl, pl, tj)
    pl.reset(pos=(0, 0, 1.5))
    e = 1.0
    for k in range(int(6.0 / pl.dt)):
        pl.step_with_control(tk.act(k * pl.dt))
        tk.observe()
        e = float(np.linalg.norm(pl.state().pos - tj.eval(k * pl.dt)[0]))
    assert e < 0.05, "%s 悬停误差 %.4f m" % (ctl, e)


@pytest.mark.parametrize("ctl", ["pid", "geo"])
def test_thrust_within_actuator_limits(ctl):
    pl = ResidualPlant(plant=make_plant("extra", "windy", seed=2), scene="open_ground")
    tj = make_trajectory("figure8")
    tk = make_tracker(ctl, pl, tj)
    p0, _, _, y0, _ = tj.eval(0.0)
    pl.reset(pos=tuple(p0), yaw=float(y0))
    for k in range(int(tj.duration / pl.dt)):
        u = tk.act(k * pl.dt)
        assert np.all(u >= -1e-9) and np.all(u <= pl.params.max_thrust + 1e-9)
        pl.step_with_control(u)
        tk.observe()


def test_payload_causes_static_droop():
    """几何控制器不用积分，静差必须随载荷线性增长 —— 这是残差要补的东西。"""
    errs = []
    for p, dm in (("none", 0.0), ("heavy", 0.5)):
        r = run_once("geo", p, "calm", "hover", 0)
        errs.append(r.metrics["rmse"])
        assert r.metrics["rmse"] == pytest.approx(dm * 9.81 / 0.95 / 20.0, abs=0.03)
    assert errs[1] > errs[0]


def test_residual_tracker_beats_geometric_on_payload():
    rng = np.random.default_rng(0)
    pl = ResidualPlant(plant=make_plant("heavy", "calm", ground_effect=0.0),
                       scene="open_ground")
    X = np.empty((3000, FEATURE_DIM))
    Y = np.empty((3000, 3))
    X[:] = rng.normal(size=X.shape) * 0.3
    X[:, 12] = 2.0                       # 高度
    X[:, 13] = 15.0                      # 归一化推力
    Y[:] = 0.0
    Y[:, 2] = -0.5 * 9.81 / 0.95
    net = ResidualNet(sizes=(FEATURE_DIM, 24, 24, 3), seed=0)
    net.fit_scaler(X)
    for _ in range(120):
        net.train_epoch(X, Y, batch=128, lr=5e-3, seed=0)

    e_geo = run_once("geo", "heavy", "calm", "hover", 0).metrics["rmse"]
    e_res = run_once("geo_res", "heavy", "calm", "hover", 0, net=net).metrics["rmse"]
    assert e_res < 0.35 * e_geo, "残差网络应把静差压到 35% 以下（实测 %.4f vs %.4f）" \
        % (e_res, e_geo)


def test_rls_tracker_does_not_require_offline_accuracy():
    """最后一层零初始化也能靠 RLS 在线把常值残差学出来。"""
    net = ResidualNet(sizes=(FEATURE_DIM, 24, 24, 3), seed=0)
    net.fit_scaler(np.random.default_rng(0).normal(size=(200, FEATURE_DIM)))
    r = run_once("geo_rls", "heavy", "calm", "figure8", 1, net=net)
    assert r.metrics["rmse"] < 0.25, "RLS 应把重载误差压到 0.25 m 内"


def test_unknown_names_raise():
    pl = ResidualPlant(plant=make_plant("none", "calm"), scene="open_ground")
    tj = make_trajectory("hover")
    with pytest.raises(KeyError):
        make_tracker("nope", pl, tj)
    with pytest.raises(KeyError):
        make_trajectory("nope")
    with pytest.raises(KeyError):
        make_plant("nope", "calm")


# --------------------------------------------------------------------------- #
# 采集与实验
# --------------------------------------------------------------------------- #
def test_episode_collector_outputs_finite_shapes():
    pl = ResidualPlant(plant=make_plant("light", "breeze", seed=1),
                       scene="open_ground")
    ec = EpisodeCollector(pl, excite_std=0.8, seed=1)
    X, Y = ec.run(make_trajectory("helix", turns=0.5))
    assert len(X) == len(Y) > 100
    assert X.shape[1] == FEATURE_DIM and Y.shape[1] == 3
    assert np.all(np.isfinite(X)) and np.all(np.isfinite(Y))
    assert np.median(Y[:, 2]) == pytest.approx(-0.25 * 9.81 / 0.95, abs=0.6)


def test_dataset_split_and_roundtrip(tmp_path):
    rng = np.random.default_rng(0)
    ds = Dataset(rng.normal(size=(100, FEATURE_DIM)), rng.normal(size=(100, 3)),
                 [{"seed": i} for i in range(100)])
    tr, va = ds.split(0.3, seed=0)
    assert len(tr) + len(va) == len(ds) == 100
    p = str(tmp_path / "d.npz")
    ds.save(p)
    back = Dataset.load(p)
    assert np.allclose(back.X, ds.X) and len(back.meta) == 100


def test_trajectory_is_smooth_and_starts_in_air():
    for name in ("hover", "circle", "helix", "figure8"):
        tj = make_trajectory(name)
        p, v, a, yaw, _ = tj.eval(0.0)
        assert p.shape == v.shape == a.shape == (3,)
        assert p[2] > 0.5, "%s 起始高度过低" % name
        p2, v2, a2, _, _ = tj.eval(tj.duration)
        assert np.all(np.isfinite(p2)) and np.all(np.isfinite(a2))


def test_gains_are_negative_by_convention():
    g = Gains()
    assert all(x < 0 for x in g.kp + g.kd + g.ki + g.kr + g.kw)


def test_run_once_is_reproducible():
    a = run_once("geo", "heavy", "windy", "helix", 3)
    b = run_once("geo", "heavy", "windy", "helix", 3)
    assert a.metrics["rmse"] == pytest.approx(b.metrics["rmse"], rel=1e-12)
