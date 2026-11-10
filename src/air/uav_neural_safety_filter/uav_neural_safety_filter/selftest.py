#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自检：几何、距离栅格、网络梯度、过滤器、环境与端到端。"""
from __future__ import annotations

import os
import time

import numpy as np

from .core.geometry import (DRONE_RADIUS, clearance_at, clearance_points,
                            rollout_clearance_many, swept_clearance)
from .core.ranges import (MAX_RANGE, N_AZ, N_EL, direction_range,
                          lidar_lower_bound, points_to_grid)
from .core.network import NetConfig, PolicyNet, SafetyNet


class _Report:
    def __init__(self):
        self.n = 0
        self.fail = 0
        self.t0 = time.time()

    def check(self, name, ok, detail=""):
        self.n += 1
        if not ok:
            self.fail += 1
        print("  [%s] %-24s %s" % ("PASS" if ok else "FAIL", name, detail))


def _scene():
    from uav_mujoco_sim.core.scenes import Scene, SceneObject
    objs = [SceneObject("obstacle_box0", "box", (2.0, 0.0, 1.0), (0.5, 0.5, 1.0)),
            SceneObject("obstacle_cyl1", "cylinder", (0.0, 2.0, 1.0), (0.3, 1.0)),
            SceneObject("obstacle_sph2", "sphere", (-2.0, 0.0, 1.0), (0.4,))]
    return Scene(name="selftest", objects=objs)


def _sec_geometry(rep):
    sc = _scene()
    rep.check("盒体表面间距", abs(clearance_at(sc, [1.0, 0.0, 1.0])
                                  - (0.5 - DRONE_RADIUS)) < 1e-9,
              "%.3f" % clearance_at(sc, [1.0, 0.0, 1.0]))
    rep.check("柱体表面间距", abs(clearance_at(sc, [0.0, 1.3, 1.0])
                                  - (0.4 - DRONE_RADIUS)) < 1e-9,
              "%.3f" % clearance_at(sc, [0.0, 1.3, 1.0]))
    rep.check("球体表面间距", abs(clearance_at(sc, [-1.2, 0.0, 1.0])
                                  - (0.4 - DRONE_RADIUS)) < 1e-9,
              "%.3f" % clearance_at(sc, [-1.2, 0.0, 1.0]))
    rep.check("地面间距", abs(clearance_at(sc, [5.0, 5.0, 0.5]) - 0.34) < 1e-9,
              "%.3f" % clearance_at(sc, [5.0, 5.0, 0.5]))
    a = swept_clearance(sc, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0])
    b = swept_clearance(sc, [0.0, 0.0, 1.0], [2.0, 0.0, 0.0])
    rep.check("扫掠间距随速度下降", b < a, "%.2f → %.2f" % (a, b))
    q = rollout_clearance_many(sc, [0.0, 0.0, 1.0], np.zeros((1, 3)), 0.0,
                               np.array([[2.0, 0.0, 0.0]]))[0]
    rep.check("rollout 间距有限", np.isfinite(q), "%.2f" % q)
    pts = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])
    rep.check("批量点间距", clearance_points(sc, pts).shape == (2,), "")


def _sec_ranges(rep):
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    g = points_to_grid(pts, max_range=8.0)
    rep.check("栅格形状", g.shape == (N_EL, N_AZ), "%s" % (g.shape,))
    rep.check("正前方命中", abs(g[1, N_AZ // 2] - 1.0) < 1e-6,
              "%.2f" % g[1, N_AZ // 2])
    c, nb = direction_range(g, [1.0, 0.0, 0.0])
    rep.check("方向距离", 0.0 < c <= 1.0 and 0.0 < nb <= 1.0,
              "%.3f %.3f" % (c, nb))
    lb = lidar_lower_bound(g, [1.0, 0.0, 0.0], horizon=0.6)
    rep.check("激光下界", lb <= c * MAX_RANGE + 1e-9, "%.2f" % lb)
    z = direction_range(g, [0.0, 0.0, 1.0])
    rep.check("垂直指令无方向约束", z == (1.0, 1.0), "%s" % (z,))


def _grad_check(net, X, Y, eps=1e-6):
    y, cache = net.forward(X, cache=True)
    loss = float(np.mean((y - Y) ** 2))
    net.optimizer(1e-3).zero_grad()
    net.backward(2.0 * (y - Y) / y.size, cache)
    g = net.layers[0].grads[0]
    k = (0, 0)
    old = net.layers[0].W[k]
    net.layers[0].W[k] = old + eps
    lp = float(np.mean((net.forward(X) - Y) ** 2))
    net.layers[0].W[k] = old - eps
    lm = float(np.mean((net.forward(X) - Y) ** 2))
    net.layers[0].W[k] = old
    num = (lp - lm) / (2 * eps)
    return abs(num - g[k]) / max(abs(num) + abs(g[k]), 1e-9), loss


def _sec_network(rep, rng):
    p = PolicyNet(hidden=16, seed=0)
    X = rng.normal(size=(8, 119))
    Y = rng.normal(size=(8, 3))
    e, loss = _grad_check(p, X, Y)
    rep.check("策略梯度校验", e < 1e-5, "相对误差 %.1e" % e)
    p.save(os.path.join(os.path.dirname(__file__), "_tmp_policy.npz"))
    p2 = PolicyNet.load(os.path.join(os.path.dirname(__file__), "_tmp_policy.npz"))
    rep.check("策略存取一致", np.allclose(p.forward(X), p2.forward(X)), "")
    os.remove(os.path.join(os.path.dirname(__file__), "_tmp_policy.npz"))

    net = SafetyNet(NetConfig(hidden=16))
    Xs = rng.normal(size=(8, 116))
    A = rng.normal(size=(8, 5))
    h, q, cache = net.forward(Xs, A)
    loss = float(np.mean(h ** 2) + np.mean(q ** 2))
    net.optimizer(1e-3).zero_grad()
    net.backward(2 * h / len(h), 2 * q / len(q), cache)
    g = net.l1.grads[0][0, 0]
    old = net.l1.W[0, 0]
    net.l1.W[0, 0] = old + 1e-6
    h2, q2, _ = net.forward(Xs, A)
    lp = float(np.mean(h2 ** 2) + np.mean(q2 ** 2))
    net.l1.W[0, 0] = old - 1e-6
    h3, q3, _ = net.forward(Xs, A)
    lm = float(np.mean(h3 ** 2) + np.mean(q3 ** 2))
    net.l1.W[0, 0] = old
    num = (lp - lm) / 2e-6
    rep.check("安全网梯度校验", abs(num - g) / max(abs(num) + abs(g), 1e-9) < 1e-4,
              "相对误差 %.1e" % (abs(num - g) / max(abs(num) + abs(g), 1e-9)))


def _sec_filter(rep, rng):
    from .core.filter import (FilterConfig, GeometricFilter, LearnedFilter,
                              candidates, NoFilter)
    u = np.array([2.5, 0.0, 0.1])
    c = candidates(u)
    rep.check("候选集含零指令", np.any(np.all(np.abs(c) < 1e-9, axis=1)),
              "%d 个候选" % len(c))
    rep.check("候选集限幅", np.all(c[:, 0] <= 4.0) and np.all(c[:, 2] <= 2.0), "")
    sc = _scene()
    env = type("E", (), {"scene": sc})()
    f = GeometricFilter(env, FilterConfig())
    obs = {"pos": np.array([-1.0, -1.0, 1.0]), "vel": np.zeros(3),
           "yaw": 0.0, "ranges": np.full((N_EL, N_AZ), MAX_RANGE, np.float32),
           "h": np.float32(2.0), "feat": np.zeros(116, np.float32),
           "goal_dir": np.array([1.0, 0.0, 0.0], np.float32)}
    uu, diag = f.act(obs, u)
    rep.check("几何过滤输出有限", np.all(np.isfinite(uu)), "%s" % np.round(uu, 2))
    nf = NoFilter()
    rep.check("无过滤原样返回", np.allclose(nf.act(obs, u)[0], u), "")
    net = PolicyNet(hidden=16)
    lf = LearnedFilter(net)
    rep.check("学习过滤输出有限", np.all(np.isfinite(lf.act(obs, u)[0])), "")


def _sec_env(rep):
    from .core.env import EnvConfig, SafetyEnv, make_episodes
    env = SafetyEnv(EnvConfig(max_time=5.0, n_obstacles=12))
    specs = make_episodes([0], per_seed=1, n_obstacles=12)
    obs = env.reset(specs[0])
    rep.check("观测形状", obs["ranges"].shape == (N_EL, N_AZ)
              and obs["feat"].shape == (116,), "%s" % (obs["ranges"].shape,))
    rep.check("初始间距为正", float(obs["h"]) > 0.0, "%.2f" % obs["h"])
    u = env.nominal_action()
    obs2, info, done = env.step(u)
    rep.check("步进返回结构", "collision" in info and "progress" not in info, "")
    rep.check("动作有限", np.all(np.isfinite(u)), "%s" % np.round(u, 2))
    rep.check("场景对象数量", len(env.scene.objects) == 12,
              "%d" % len(env.scene.objects))


def _sec_data(rep):
    from .core.dataset import CollectConfig, collect
    d = collect(CollectConfig(seeds=[0], per_seed=1, max_frames=15,
                              max_time=2.0, verbose=False))
    rep.check("采集非空", len(d["h"]) > 0, "%d 帧" % len(d["h"]))
    rep.check("标签有限", np.isfinite(d["h"]).all() and np.isfinite(d["q"]).all(),
              "")
    rep.check("动作形状", d["action"].shape[1] == 3, "")


def _sec_end2end(rep):
    from .core.env import EnvConfig, SafetyEnv, make_episodes
    from .core.experiment import run_episode
    from .core.filter import FilterConfig, GeometricFilter, LearnedFilter
    from .core.network import PolicyNet
    env = SafetyEnv(EnvConfig(max_time=6.0, n_obstacles=12))
    spec = make_episodes([0], per_seed=1, n_obstacles=12)[0]
    r = run_episode(env, GeometricFilter(env, FilterConfig(progress=0.0)), spec)
    rep.check("几何层跑完一集", r["steps"] > 0 and np.isfinite(r["h_min"]),
              "步数 %d" % r["steps"])
    model = os.path.join(os.path.dirname(__file__), "..", "data", "safety_policy.npz")
    if os.path.exists(model):
        net = PolicyNet.load(model)
        r2 = run_episode(env, LearnedFilter(net, FilterConfig()), spec)
        rep.check("神经层跑完一集", r2["steps"] > 0, "步数 %d" % r2["steps"])
        rep.check("神经层延迟 < 2 ms", r2["latency_ms"] < 2.0,
                  "%.2f ms" % r2["latency_ms"])
    else:
        rep.check("预训练策略存在", False, model)


def run() -> bool:
    rep = _Report()
    rng = np.random.default_rng(0)
    print("=" * 74)
    print("  uav_neural_safety_filter 自检")
    print("=" * 74)
    for title, fn in (("几何与扫掠间距", _sec_geometry),
                      ("距离栅格", _sec_ranges),
                      ("网络梯度", lambda r: _sec_network(r, rng)),
                      ("过滤器", lambda r: _sec_filter(r, rng)),
                      ("环境", _sec_env),
                      ("数据采集", _sec_data),
                      ("端到端", _sec_end2end)):
        print("\n[%s]" % title)
        fn(rep)
    print("\n" + "=" * 74)
    print("  通过 %d/%d   总耗时 %.1f s" % (rep.n - rep.fail, rep.n,
                                           time.time() - rep.t0))
    print("=" * 74)
    return rep.fail == 0


if __name__ == "__main__":
    raise SystemExit(0 if run() else 1)
