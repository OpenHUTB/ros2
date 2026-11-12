"""单测：赛道几何、环境、延迟线、补偿器、网络梯度、闭环回归。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from lat_nav import track
from lat_nav.env import DT, LIDAR_BEAMS, LIDAR_RANGE, LatencyEnv
from lat_nav.nav import (DelayLine, features, integrate, predict_pose, pure_pursuit,
                         query_pose, run_episode, warp_scan)
from lat_nav.net import IN_DIM, V_MAX_MODEL, W_MAX_MODEL, LatencyNet

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def env():
    return LatencyEnv("circle", seed=0)


# ---------------------------------------------------------------- 赛道几何
@pytest.mark.parametrize("shape", track.SHAPES)
def test_centerline_is_closed_and_smooth(shape):
    p = track.centerline(shape)
    gap = np.linalg.norm(p[0] - p[-1])
    seg = np.linalg.norm(np.diff(np.vstack([p, p[:1]]), axis=0), axis=1)
    assert gap < 3.0 * seg.mean(), "首尾应当闭合（相邻点距量级）"
    assert seg.max() < 2.5 * seg.mean(), "采样应当均匀"


@pytest.mark.parametrize("shape", track.SHAPES)
def test_curvature_allows_lane_width(shape):
    """最小曲率半径必须大于车道半宽，否则内侧墙会自己压到自己身上。"""
    p = track.centerline(shape)
    d = np.gradient(p, axis=0)
    dd = np.gradient(d, axis=0)
    cross = np.abs(d[:, 0] * dd[:, 1] - d[:, 1] * dd[:, 0])
    speed = np.linalg.norm(d, axis=1)
    radius = speed**3 / np.maximum(cross, 1e-12)
    assert radius.min() > track.LANE_HALF_WIDTH + 0.5, f"{shape} 最小曲率半径 {radius.min():.2f} m"


def test_boundary_offset_matches_lane_width():
    p = track.centerline("kidney")
    border = p + track.LANE_HALF_WIDTH * track.normals(p)
    d = np.linalg.norm(border - p, axis=1)
    assert np.allclose(d, track.LANE_HALF_WIDTH, atol=1e-9)


def test_wall_points_are_equally_spaced():
    p = track.centerline("circle")
    b = track.boundary_points(p, track.LANE_HALF_WIDTH)
    seg = np.linalg.norm(np.diff(np.vstack([b, b[:1]]), axis=0), axis=1)
    assert seg.max() - seg.min() < 0.02
    assert abs(seg.mean() - track.WALL_SEG) < 0.03


# ---------------------------------------------------------------- 环境
def test_xml_builds_and_has_expected_joints():
    import mujoco

    m = mujoco.MjModel.from_xml_string(track.build_xml("circle"))
    assert m.njnt == 3
    assert m.nu == 3
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot") > 0


def test_reset_is_deterministic(env):
    a = env.reset(seed=3)
    b = env.reset(seed=3)
    assert np.allclose(a, b)
    c = env.reset(seed=4)
    assert not np.allclose(a, c), "换种子应当换起点"


def test_step_moves_forward(env):
    env.reset(seed=1)
    p0 = env.pose()
    for _ in range(20):
        env.step(1.0, 0.0)
    p1 = env.pose()
    assert np.linalg.norm(p1[:2] - p0[:2]) > 0.5
    assert abs(env.velocity()[0] - 1.0) < 0.15


def test_lidar_hits_walls(env):
    env.reset(seed=0)
    s = env.scan()
    assert s.shape == (LIDAR_BEAMS,)
    assert s.min() >= 0.02 and s.max() <= LIDAR_RANGE
    assert s.min() < 1.2, "车道半宽 0.75 m，必然有一束打到墙"


def test_lidar_ignores_own_chassis(env):
    """射线起点在车身内部，必须排除 robot body，否则每束都读回自身。"""
    env.reset(seed=0)
    assert env.scan().min() > 0.3


def test_errors_are_zero_on_centerline(env):
    p = env.path[10]
    tan = track.tangents(env.path)[10]
    pose = np.array([p[0], p[1], np.arctan2(tan[1], tan[0])])
    e, psi = env.errors(pose)
    assert abs(e) < 1e-6 and abs(psi) < 1e-6


def test_nearest_index_tracks_continuously(env):
    env.reset(seed=2)
    idx = env.nearest_index(env.pose()[:2])
    for _ in range(30):
        env.step(1.0, 0.0)
        nxt = env.nearest_index(env.pose()[:2], idx)
        assert (nxt - idx) % len(env.path) < 40, "相邻帧索引不应跳变"
        idx = nxt


# ---------------------------------------------------------------- 延迟与补偿
def test_delay_line_passes_through_when_zero():
    d = DelayLine(0)
    assert d.push((1.0, 2.0)) == (1.0, 2.0)
    assert d.pending() == []


def test_delay_line_shifts_by_ticks():
    d = DelayLine(3)
    out = [d.push((float(i), 0.0))[0] for i in range(5)]
    assert out == [0.0, 0.0, 0.0, 0.0, 1.0], "前 3 帧输出初始指令，第 4 帧才轮到第 0 帧"
    assert len(d.pending()) == 3


def test_delay_line_pending_is_fifo():
    d = DelayLine(2)
    d.push((1.0, 0.0))
    d.push((2.0, 0.0))
    assert [c[0] for c in d.pending()] == [1.0, 2.0]


def test_integrate_straight_line():
    pose = np.array([0.0, 0.0, 0.0])
    out = integrate(pose, 1.0, 0.0, 0.5)
    assert np.allclose(out, [0.5, 0.0, 0.0], atol=1e-9)


def test_integrate_turn_is_on_circle():
    """ω 恒定时积分出来的轨迹应当落在半径 v/ω 的圆上。"""
    pose = np.array([0.0, 0.0, 0.0])
    r = 1.0 / 0.5                     # 左转时圆心在 (0, r)
    for _ in range(200):
        pose = integrate(pose, 1.0, 0.5, 0.01)
    assert abs(np.hypot(pose[0], pose[1] - r) - r) < 0.02


def test_predict_pose_without_pending_is_identity():
    pose = np.array([1.0, 2.0, 0.3])
    assert np.allclose(predict_pose(pose, [], DT), pose)


def test_predict_pose_equals_manual_integration():
    pose = np.array([0.0, 0.0, 0.0])
    pending = [(1.0, 0.1), (0.8, -0.2)]
    out = predict_pose(pose, pending, DT)
    manual = integrate(integrate(pose, 1.0, 0.1, DT), 0.8, -0.2, DT)
    assert np.allclose(out, manual)


def test_query_pose_modes(env):
    env.reset(seed=0)
    pose = env.pose()
    pending = [(1.0, 0.2)] * 4
    assert np.allclose(query_pose(env, pose, 1.0, 0.2, pending, DT, "none"), pose)
    ext = query_pose(env, pose, 1.0, 0.2, pending, DT, "extrap")
    mod = query_pose(env, pose, 1.0, 0.2, pending, DT, "model")
    assert np.linalg.norm(ext[:2] - pose[:2]) > 0.1
    assert np.linalg.norm(mod[:2] - pose[:2]) > 0.1
    with pytest.raises(ValueError):
        query_pose(env, pose, 1.0, 0.2, pending, DT, "magic")


def test_features_shape_and_range(env):
    env.reset(seed=0)
    f = features(env.scan(), 1.0, 0.3)
    assert f.shape == (IN_DIM,)
    assert np.all(f[:LIDAR_BEAMS] >= 0) and np.all(f[:LIDAR_BEAMS] <= 1.0)


def test_pure_pursuit_turns_toward_target(env):
    env.reset(seed=0)
    idx = env.nearest_index(env.pose()[:2])
    v, w = pure_pursuit(env.pose(), env.path, idx)
    assert abs(v - 1.0) < 1e-6
    assert abs(w) <= 1.6


# ---------------------------------------------------------------- 网络
def test_net_output_ranges():
    net = LatencyNet(seed=0)
    out, _ = net.forward(np.zeros(IN_DIM, dtype=np.float32))
    assert 0.0 <= out[0, 0] <= V_MAX_MODEL
    assert abs(out[0, 1]) <= W_MAX_MODEL


def test_net_gradient_matches_numeric():
    """损失口径与 backward 一致：(1/N)·ΣΣ。"""
    net = LatencyNet(seed=2)
    rng = np.random.default_rng(0)
    x = rng.uniform(0, 1, (4, IN_DIM)).astype(np.float32)
    y = np.stack([rng.uniform(0.3, 1.2, 4), rng.uniform(-1, 1, 4)], axis=1).astype(np.float32)
    n = x.shape[0]

    def loss():
        pred, _ = net.forward(x)
        return float(((pred - y) ** 2).sum() / n)

    _, cache = net.forward(x)
    g = net.backward(cache, y)
    eps = 1e-3
    for key, idx in (("w3", (3, 1)), ("w2", (5, 2)), ("w1", (0, 7))):
        old = float(net.p[key][idx])
        net.p[key][idx] = old + eps
        lp = loss()
        net.p[key][idx] = old - eps
        lm = loss()
        net.p[key][idx] = old
        num = (lp - lm) / (2 * eps)
        rel = abs(num - float(g[key][idx])) / max(1e-9, abs(num) + abs(float(g[key][idx])))
        assert rel < 5e-2, f"{key}{idx} 相对误差 {rel:.2e}"


def test_net_save_load_roundtrip(tmp_path):
    net = LatencyNet(seed=1)
    out0, _ = net.forward(np.full(IN_DIM, 0.5, dtype=np.float32))
    p = tmp_path / "policy.npz"
    net.save(p)
    other = LatencyNet(seed=9).load(p)
    out1, _ = other.forward(np.full(IN_DIM, 0.5, dtype=np.float32))
    assert np.allclose(out0, out1)
    assert other.n_params() == net.n_params()


def test_net_learns_a_linear_map():
    """能学一个平凡映射就说明反向传播和优化器接得对。"""
    net = LatencyNet(seed=3)
    x = np.random.default_rng(0).uniform(0, 1, (256, IN_DIM)).astype(np.float32)
    y = np.stack([x[:, 0] * 1.0, (x[:, 1] - 0.5) * 2.0], axis=1).astype(np.float32)
    first = net.fit(x, y, epochs=1)[0]
    last = net.fit(x, y, epochs=40)[-1]
    assert last < 0.5 * first


# ---------------------------------------------------------------- 闭环
def test_episode_is_deterministic(env):
    a = run_episode(env, policy=None, seed=5, max_time=20)
    b = run_episode(env, policy=None, seed=5, max_time=20)
    assert a["steps"] == b["steps"] and abs(a["rms"] - b["rms"]) < 1e-9


def test_expert_completes_all_tracks():
    for sh in track.SHAPES:
        r = run_episode(LatencyEnv(sh, seed=0), policy=None, seed=0, max_time=60)
        assert r["completed"], f"{sh} 上专家没能跑完"
        assert r["collide_frames"] == 0
        assert r["rms"] < 0.09


def test_committed_weights_reproduce_doc_numbers():
    """仓库权重在 0 延迟下必须复现文档的 0.03～0.06 m，三条赛道都要过。

    阈值取 0.06：文档实测 0.030（peanut）～0.051（kidney），留一点跨 mujoco 版本的漂移余量，
    但权重错版那种 0.2 m 级别的退化一定挡得住。
    """
    path = ROOT / "models" / "policy.npz"
    assert path.is_file()
    net = LatencyNet().load(path)
    for shape in track.SHAPES:
        r = run_episode(LatencyEnv(shape, seed=0), policy=net, seed=0, max_time=60)
        assert r["completed"], f"{shape} 上没跑完"
        assert r["rms"] < 0.06, f"{shape} 权重复现 RMS={r['rms']:.3f} m，与文档不符"


def test_smith_prediction_beats_no_compensation_at_600ms():
    """核心结论的回归保护：600 ms 延迟下模型前向补偿必须明显更好。"""
    net = LatencyNet().load(ROOT / "models" / "policy.npz")
    env = LatencyEnv("circle", seed=0)
    raw = run_episode(env, policy=net, comp="none", delay_ms=600.0, seed=0, max_time=60)
    smith = run_episode(env, policy=net, comp="model", delay_ms=600.0, seed=0, max_time=60)
    assert smith["rms"] < 0.5 * raw["rms"], "补偿应当把误差压掉一半以上"


def test_node_module_imports_without_ros():
    from lat_nav import node

    assert node._HAS_ROS in (True, False)
    assert callable(node.main)


def test_asset_lookup_finds_source_model():
    from lat_nav.node import _asset

    p = _asset(ROOT / "models" / "policy.npz")
    assert p.is_file()


def test_warp_scan_identity(env):
    """位姿不变时，扫描变换应当原样返回。"""
    env.reset(seed=0)
    s = env.scan()
    w = warp_scan(s, env.pose(), env.pose(), env.beam_angles)
    assert np.allclose(w, s, atol=0.03)


def test_warp_scan_forward_shortens_front_range(env):
    """向前平移后，正前方那几束应当变短（墙更近了）。"""
    env.reset(seed=0)
    p = env.pose()
    s = env.scan()
    q = integrate(p, 1.0, 0.0, 0.5)
    w = warp_scan(s, p, q, env.beam_angles)
    front = np.abs(env.beam_angles) < 0.3
    assert w[front].mean() < s[front].mean() - 0.2


def test_warp_scan_stays_in_range(env):
    env.reset(seed=0)
    s = env.scan()
    q = integrate(env.pose(), 1.0, 0.5, 0.8)
    w = warp_scan(s, env.pose(), q, env.beam_angles)
    assert np.all(w >= 0.02) and np.all(w <= LIDAR_RANGE)
    assert w.shape == s.shape
