"""离线单元测试（不需要 ROS / MuJoCo）：观测编码、梯度、控制律、距离图。"""
import math

import numpy as np

from policy_learning.encoder import N_OBS, PATCH, encode
from policy_learning.expert import W_MAX, steer, target_point
from policy_learning.grid import build_grid, planning_grid, to_cell
from policy_learning.mlp import PolicyMLP
from policy_learning.sim import step
from policy_learning.value import descend_path, distance_map


def test_obs_shape_and_range():
    g = planning_grid()
    obs = encode(g, (2.0, 2.0, 0.0), (9.0, 9.0), distance_map(g, (9.0, 9.0)))
    assert obs.shape == (N_OBS,)
    assert np.all(np.isfinite(obs))
    occ = obs[:PATCH * PATCH]
    assert set(np.unique(occ)).issubset({0.0, 1.0})


def test_obstacle_ahead_is_marked():
    """正前方 0.6 m 有障碍时，对应格应为占据。"""
    g = np.zeros((120, 120), np.uint8)
    i, j = to_cell(g, 2.6, 2.0)
    g[j, i] = 1
    obs = encode(g, (2.0, 2.0, 0.0), (9.0, 2.0))
    patch = obs[:PATCH * PATCH].reshape(PATCH, PATCH)
    assert patch[PATCH // 2 - 6, PATCH // 2] == 1.0


def test_steer_limits():
    v, w = steer(0.8, 1.5)
    assert v > 0.0
    assert abs(w) <= W_MAX + 1e-6
    v0, w0 = steer(0.8, 0.0)
    assert abs(w0) < 1e-9
    assert v0 > 0.0


def test_target_point_body_frame():
    path = [(3.0, 2.0)]
    fwd, lat = target_point(path, (2.0, 2.0, 0.0))
    assert fwd > 0.9 and abs(lat) < 0.2

    fwd, lat = target_point(path, (2.0, 2.0, np.pi / 2))
    assert fwd < 0.1 and lat < -0.9


def test_distance_map_and_descent():
    g = planning_grid()
    goal = (10.2, 10.2)
    d = distance_map(g, goal)
    gi, gj = to_cell(g, *goal)
    assert abs(float(d[gj, gi])) < 1e-6

    path = descend_path(d, g, (0.85, 3.05))
    assert path is not None and len(path) > 5
    vals = [float(d[to_cell(g, x, y)[1], to_cell(g, x, y)[0]]) for x, y in path]
    assert all(b <= a + 1e-6 for a, b in zip(vals, vals[1:]))
    assert vals[-1] < vals[0]


def test_astar_matches_value_descent():
    """A* 与距离图下降应给出等长的路径，两条实现互为校验。"""
    from policy_learning.planner import astar

    g = planning_grid()
    goal, start = (10.2, 10.2), (0.85, 3.05)
    pa = astar(g, start, goal)
    assert pa is not None

    d = distance_map(g, goal)
    pv = descend_path(d, g, start)
    assert pv is not None

    la = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pa, pa[1:]))
    lv = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pv, pv[1:]))
    assert abs(la - lv) < 0.5, (la, lv)


def test_mlp_gradcheck():
    rng = np.random.default_rng(0)
    net = PolicyMLP(n_in=6, n_h1=5, n_h2=4, n_out=2, seed=0)
    x = rng.normal(size=(4, 6))
    y = rng.normal(size=(4, 2))
    net.set_stats(x, y)

    xn = net._nx(x)
    yn = (y - net.y_mean) / net.y_std
    yh, cache = net.forward(xn, cache=True)
    err = yh - yn
    grads = net.backward(2.0 * err / err.size, cache)

    def loss():
        h = net.forward(xn)
        e = h - yn
        return float(np.mean(e * e))

    for name, grad in zip(("W1", "b1", "W2", "b2", "W3", "b3"), grads):
        arr = getattr(net, name)
        for _ in range(3):
            idx = tuple(rng.integers(0, s) for s in arr.shape)
            old = arr[idx]
            arr[idx] = old + 1e-6
            lp = loss()
            arr[idx] = old - 1e-6
            lm = loss()
            arr[idx] = old
            num = (lp - lm) / 2e-6
            assert abs(num - grad[idx]) < 1e-4, (name, num, grad[idx])


def test_mlp_save_load(tmp_path):
    net = PolicyMLP(n_in=N_OBS, n_h1=8, n_h2=4, n_out=2, seed=1)
    x = np.random.default_rng(0).normal(size=(16, N_OBS)).astype(np.float32)
    net.set_stats(x, np.random.default_rng(1).normal(size=(16, 2)))
    p = tmp_path / "m.npz"
    net.save(str(p))
    net2 = PolicyMLP.load(str(p))
    assert np.allclose(net.predict(x), net2.predict(x), atol=1e-6)


def test_policy_action_conversion():
    """DAgger 采样必须和闭环评测走同一条转换，不能把归一化前视点当 (v, ω)。"""
    from policy_learning.dataset import policy_action
    from policy_learning.expert import LOOKAHEAD, steer

    class _FakeNet:
        def __init__(self, out):
            self.out = np.asarray(out, np.float32)

        def predict(self, obs):
            return self.out

    v, w = policy_action(_FakeNet([1.0, 0.0]), None)
    ref = steer(LOOKAHEAD, 0.0)
    assert abs(v - ref[0]) < 1e-6 and abs(w - ref[1]) < 1e-6

    _, w2 = policy_action(_FakeNet([0.0, 0.5]), None)
    assert abs(w2) > 0.5          # 侧向偏移应产生角速度


def test_sim_step_forward():
    x, y, th = step((0.0, 0.0, 0.0), 0.5, 0.0, dt=0.1)
    assert abs(x - 0.05) < 1e-9 and abs(y) < 1e-9 and abs(th) < 1e-9
