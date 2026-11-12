"""mujoco_mppi_neural_tracker 单元测试（不需要 ROS）。"""

from __future__ import annotations

import numpy as np
import pytest
from pathlib import Path

from mppi_nn import track
from mppi_nn.env import (CONE_RADIUS, DT, LIDAR_BEAMS, LIDAR_RANGE, WHEELBASE,
                         AckermannEnv, wrap)
from mppi_nn.mppi import MPPIConfig, MPPIController, pure_pursuit
from mppi_nn.nav import PoseEstimator, collect_expert, run_episode
from mppi_nn.net import IN_DIM, V_MAX, V_MIN, TrackerNet, build_features


@pytest.fixture(scope="module")
def env():
    return AckermannEnv(0)


@pytest.fixture(scope="module")
def path(env):
    return env.path


# ---------------------------------------------------------------- 轨迹几何
def test_reference_point_and_tangent():
    p, d = track.ref_point(0.0)
    assert np.allclose(p, [track.A, 0.0], atol=1e-9)
    assert abs(np.linalg.norm(d) - 1.0) < 1e-9
    p2, _ = track.ref_point(np.pi)
    assert np.allclose(p2, [-track.A, 0.0], atol=1e-6)


def test_polyline_is_closed_and_smooth(path):
    assert len(path) == 600
    step = np.linalg.norm(path[1] - path[0])
    assert 0.03 < step < 0.10
    assert np.linalg.norm(path[-1] - path[0]) < 0.2      # 首尾相接


def test_cone_positions_pair_around_path():
    cones = track.cone_positions()
    assert cones.shape[0] % 2 == 0
    # 相邻两个锥桶到参考线的距离都应该 ≈ 车道半宽
    p, d = track.ref_point(0.0)
    left, right = cones[0], cones[1]
    assert abs(np.linalg.norm(left - p) - track.LANE_HALF) < 1e-6
    assert abs(np.linalg.norm(right - p) - track.LANE_HALF) < 1e-6


def test_track_errors_sign(path):
    idx = 0
    p = path[0]
    nxt = path[1]
    t = (nxt - p) / np.linalg.norm(nxt - p)
    left_of_path = p + np.array([-t[1], t[0]]) * 0.3
    e, _ = track.track_errors(path, left_of_path, np.arctan2(t[1], t[0]), idx)
    assert e > 0.2, "车在参考线左侧时 e 应为正"


def test_nearest_index_window_does_not_jump(path):
    i0 = track.nearest_index(path, path[100])
    i1 = track.nearest_index(path, path[100] + 0.01, hint=i0)
    assert i1 == i0


# ---------------------------------------------------------------- 环境
def test_reset_pose_and_yaw(env):
    s = env.reset()
    assert np.allclose(s.pos, AckermannEnv.START, atol=0.05)
    assert abs(wrap(s.yaw - AckermannEnv.START_YAW)) < 0.05


def test_straight_line_speed(env):
    env.reset()
    for _ in range(40):                       # 2 s
        env.step(1.0, 0.0)
    s = env.state()
    assert 1.6 < np.linalg.norm(s.pos - np.array(AckermannEnv.START)) < 2.1
    assert 0.85 < s.v < 1.15


def test_yaw_rate_matches_bicycle_model(env):
    env.reset()
    v, delta = 0.8, 0.3
    for _ in range(int(1.0 / DT)):
        s = env.step(v, delta)
    expect = v * np.tan(delta) / WHEELBASE * 1.0
    got = wrap(s.yaw - AckermannEnv.START_YAW)
    assert abs(got - expect) < 0.12


def test_lidar_shape_and_range(env):
    env.reset()
    scan = env.scan()
    assert scan.shape == (LIDAR_BEAMS,)
    assert scan.min() > 0.0 and scan.max() <= LIDAR_RANGE + 1e-6


def test_lidar_sees_cones_at_lane_width(env):
    """车在车道中心时，最近的锥桶约在车道半宽处。"""
    env.reset()
    scan = env.scan()
    assert 0.4 < float(scan.min()) < 1.2


def test_lidar_min_matches_bruteforce(env):
    """解析雷达与暴力逐锥桶求距一致（同一射线方向）。"""
    env.reset()
    s = env.state()
    scan = env.scan()
    ang = s.yaw + np.linspace(-np.pi, np.pi, LIDAR_BEAMS, endpoint=False)
    d = np.array([np.cos(ang[3]), np.sin(ang[3])])
    rel = env.cones - s.pos
    proj = rel @ d
    perp2 = (rel ** 2).sum(axis=1) - proj ** 2
    ok = (proj > 0) & (perp2 < CONE_RADIUS ** 2)
    want = (proj[ok] - np.sqrt(CONE_RADIUS ** 2 - perp2[ok])).min()
    assert abs(float(scan[3]) - float(want)) < 1e-5


def test_lidar_estimates_lateral_error(env):
    env.reset()
    e_true, _ = env.errors()
    e_meas, _ = env.lane_errors_from_lidar()
    assert abs(abs(e_meas) - abs(e_true)) < 0.35


def test_odom_from_pose_increment(env):
    env.reset()
    a = env.state()
    for _ in range(20):                       # 先加速到稳态
        b = env.step(1.0, 0.0)
    a = b
    b = env.step(1.0, 0.0)                    # odom 按相邻两帧求速度
    v, w = env.odom(a, b)
    assert abs(v - 1.0) < 0.25


# ---------------------------------------------------------------- MPPI
def test_mppi_action_within_bounds(env, path):
    m = MPPIController(path, MPPIConfig())
    v, d = m.act((6.0, 0.0, AckermannEnv.START_YAW), v_now=0.9)
    assert 0.0 < v <= MPPIConfig().v_max + 1e-6
    assert abs(d) <= MPPIConfig().delta_max + 1e-6


def test_mppi_prefers_lower_cost(path):
    """在偏左 0.5 m 的位置，MPPI 应该往右打（负 δ）。"""
    m = MPPIController(path, MPPIConfig(), seed=3)
    p, t = track.ref_point(0.0)
    left = p + np.array([-t[1], t[0]]) * 0.5
    yaw = float(np.arctan2(t[1], t[0]))
    _, delta = m.act((left[0], left[1], yaw), v_now=0.9)
    assert delta < 0.05


def test_mppi_is_seed_deterministic(path):
    a = MPPIController(path, MPPIConfig(), seed=7)
    b = MPPIController(path, MPPIConfig(), seed=7)
    u1 = a.act((6.0, 0.0, AckermannEnv.START_YAW))
    u2 = b.act((6.0, 0.0, AckermannEnv.START_YAW))
    assert np.allclose(u1, u2)


def test_pure_pursuit_steers_towards_path(path):
    v, d = pure_pursuit((6.0, -1.0, AckermannEnv.START_YAW), path, 0)
    assert v > 0 and np.isfinite(d) and abs(d) <= 0.6


# ---------------------------------------------------------------- 网络
def test_features_dimension(path):
    f = build_features(0.1, 0.05, 0.9, 0.0, 0.9, path, 10, np.array([6.0, 0.0, 1.57]))
    assert f.shape == (IN_DIM,)
    assert np.all(np.isfinite(f))


def test_net_output_ranges():
    net = TrackerNet(seed=0)
    v, d = net.act(np.zeros(IN_DIM, dtype=np.float32))
    assert V_MIN - 1e-6 <= v <= V_MAX + 1e-6
    assert abs(d) <= 0.6 + 1e-6


def test_net_gradient_matches_numeric():
    """损失口径必须与 backward 一致：(1/N)·ΣΣ，不是 np.mean（那是 1/(2N)）。

    上一版测试用 np.mean、容差 2e-2·max(1,|num|)，而该元素梯度量级只有 1e-3，
    解析梯度正好是数值的 2 倍也能通过——这条测试当时没有约束力。
    """
    net = TrackerNet(seed=1)
    rng = np.random.default_rng(0)
    x = rng.normal(0, 0.4, (5, IN_DIM)).astype(np.float64)
    y = rng.uniform(0, 1, (5, 2)).astype(np.float64)
    n = x.shape[0]

    def loss():
        pred, _ = net.forward(x.astype(np.float32))
        return float(((pred - y) ** 2).sum() / n)

    _, cache = net.forward(x.astype(np.float32))
    g = net.backward(cache, y.astype(np.float32))

    eps = 1e-3          # float32 参数：步长太小会被舍入误差吃掉
    for key, idx in (("w2", (4, 3)), ("w3", (2, 1)), ("w1", (0, 5))):
        old = float(net.p[key][idx])
        net.p[key][idx] = old + eps
        lp = loss()
        net.p[key][idx] = old - eps
        lm = loss()
        net.p[key][idx] = old
        num = (lp - lm) / (2 * eps)
        rel = abs(num - float(g[key][idx])) / max(1e-8, abs(num) + abs(float(g[key][idx])))
        assert rel < 2e-3, f"{key}{idx}: 相对误差 {rel:.2e}"


def test_net_save_load(tmp_path):
    net = TrackerNet(seed=2)
    x = np.random.default_rng(1).normal(0, 0.3, (3, IN_DIM)).astype(np.float32)
    before, _ = net.forward(x)
    p = net.save(tmp_path / "w.npz")
    after, _ = TrackerNet(seed=9).load(p).forward(x)
    assert np.allclose(before, after, atol=1e-6)


# ---------------------------------------------------------------- 闭环
def test_pure_pursuit_completes_track(env):
    r = run_episode(env, controller="pp", noise=0.0, seed=0, steps=1200)
    assert r.completed and r.rms < 0.20


def test_mppi_beats_pure_pursuit(env):
    """MPPI 用简化模型滚动优化，横向误差应明显小于几何基线。"""
    pp = run_episode(env, controller="pp", noise=0.0, seed=0, steps=1200)
    mppi = run_episode(env, controller="mppi", noise=0.0, seed=0, steps=1200)
    assert mppi.rms < pp.rms * 0.8
    # 不在这里断言耗时比：ms 是机器相关的量，写死阈值只会在别人的机器上误报；
    # 算力对比放在文档 §6.1 的实测表格里，且注明是"本机实测"。


def test_episode_is_deterministic(env):
    a = run_episode(env, controller="pp", noise=0.0, seed=5, steps=200)
    b = run_episode(env, controller="pp", noise=0.0, seed=5, steps=200)
    assert a.steps == b.steps and abs(a.rms - b.rms) < 1e-9


def test_expert_collection_restores_env(env):
    env.reset()
    pose0 = env.state().pos.copy()
    collect_expert(env, episodes=1, steps=60, log=lambda *_: None)
    env.reset()
    assert np.allclose(env.state().pos, np.array(AckermannEnv.START), atol=1e-6)
    assert np.allclose(pose0, np.array(AckermannEnv.START), atol=1e-6)


def test_pose_estimator_corrects_lateral_error(env, path):
    env.reset()
    est = PoseEstimator(env)
    est.pos = est.pos + np.array([0.0, 0.5])      # 人为推偏
    before = abs(track.track_errors(path, est.pos, est.yaw, est.idx)[0])
    for _ in range(4):
        est.correct(path, 0.0)
    after = abs(track.track_errors(path, est.pos, est.yaw, est.idx)[0])
    assert after < before


def test_committed_weights_reproduce_doc_numbers():
    """仓库里的 models/policy.npz 必须能复现文档 §6.1 的 0.028 m。

    上一版 PR 里提交的权重是"两回合专家"那一版的旧文件，而文档写的是三回合训练后的
    0.028 m，评审照着仓库跑只能得到 0.051 m。这条测试把"权重与文档数字一致"钉死。
    """
    path = Path(__file__).resolve().parents[1] / "models" / "policy.npz"
    net = TrackerNet().load(path)
    env = AckermannEnv(0)
    r = run_episode(env, controller="nn", noise=0.0, seed=0, net=net)
    assert r.completed
    # 阈值比文档的 0.028 m 宽一档：不同 mujoco 主版本之间动力学有 1e-3 量级漂移，
    # 卡太死会在别人机器上误报；但 0.05 m（权重错版时会出现）仍然挡得住。
    assert r.rms < 0.045, f"提交的权重复现出的 RMS={r.rms:.3f} m，与文档 0.028 m 不符"
