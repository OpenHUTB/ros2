# -*- coding: utf-8 -*-
"""冒烟测试：检测器、建图、数据集、端到端。不需要 ROS。

涉及 mujoco 的用例在缺依赖时自动跳过，这样没有仿真环境的机器也能跑
``--evaluate``/``--figures`` 这条路。
"""
import math
import os
import tempfile

import numpy as np
import pytest

from uav_neural_vio_slam.core.dataset import (DEPTH_MAX, DEPTH_MIN, VioDataset,
                                              VioEpisode, depth_to_u8,
                                              u8_to_depth)
from uav_neural_vio_slam.core.landmarks import LandmarkDetector
from uav_neural_vio_slam.core.mapping import GridConfig, OccupancyGrid
from uav_neural_vio_slam.core.world import LoopTrajectory, build_world

_HAS_MUJOCO = True
try:
    import mujoco  # noqa: F401
except Exception:                                             # noqa: BLE001
    _HAS_MUJOCO = False


# --------------------------------------------------------------------------- #
def test_depth_codec_roundtrip():
    d = np.array([[0.0, 0.5, 3.7, 9.99, 20.0]])
    back = u8_to_depth(depth_to_u8(d))
    assert back[0, 0] == 0.0                    # 无效像素
    assert back[0, 4] == 0.0                    # 超量程置 0
    assert abs(back[0, 2] - 3.7) < 0.06         # 量化误差 < 半步长


def test_world_is_deterministic():
    a = build_world(seed=0)
    b = build_world(seed=0)
    assert np.abs(a.positions - b.positions).max() == 0.0
    assert len(a.landmarks) > 10


def test_world_landmarks_avoid_centre():
    """立柱不能堵住航线中心。"""
    w = build_world(seed=0)
    d = np.linalg.norm(w.positions, axis=1)
    assert d.min() > 1.0


def test_trajectory_is_closed_loop():
    tj = LoopTrajectory()
    p0 = tj.eval(0.0)[0]
    p1 = tj.eval(tj.period)[0]
    assert np.abs(p0 - p1).max() < 1e-9
    # 八字有自交点：轨迹两次经过原点附近
    pts = tj.sample(2000)
    near = np.linalg.norm(pts[:, :2] - tj.eval(0.0)[0][:2], axis=1) < 0.35
    assert near.sum() >= 2


# --------------------------------------------------------------------------- #
def test_detector_finds_single_pole():
    det = LandmarkDetector()
    r3 = np.zeros((det.n_el, det.n_az))
    for a in range(det.n_az):
        for e in range(det.n_el):
            r3[e, a] = 4.0 / math.cos(det.elev[e])
    # 只保留正前方两个方位波束：一根柱子
    keep = np.zeros(det.n_az, bool)
    for a in range(det.n_az):
        keep[a] = abs(math.atan2(math.sin(det.az[a]), math.cos(det.az[a]))) < 0.03
    r3[:, ~keep] = 0.0
    obs = det.detect(r3)
    assert len(obs) == 1
    assert abs(obs[0].rng - (4.0 + det.cfg.r_nominal)) < 0.05


def test_detector_ignores_walls():
    det = LandmarkDetector()
    r3 = np.full((det.n_el, det.n_az), 5.0) / np.cos(det.elev)[:, None]
    assert len(det.detect(r3)) == 0


def test_detector_points_ranges_roundtrip():
    det = LandmarkDetector()
    pts = np.array([[3.0, 0.0, 0.0], [0.0, 2.0, 0.0], [1.0, 1.0, 0.5]])
    r = det.ranges_from_points(pts)
    assert r.shape == (det.rays(),)
    assert r.max() > 0.0


# --------------------------------------------------------------------------- #
def test_occupancy_grid_marks_wall():
    det = LandmarkDetector()
    g = OccupancyGrid(GridConfig())
    u2 = det.dirs[:, :2]
    n = np.linalg.norm(u2, axis=1)
    u2 = u2 / np.maximum(n, 1e-9)[:, None]
    ranges = np.where(np.arange(det.rays()) < det.n_az, 6.0 * n, 0.0)
    g.integrate(np.array([0.0, 0.0, 0.0]), ranges, u2, 16.0)
    assert (g.probability() > 0.5).sum() > 0


def test_occupancy_grid_is_bounded():
    det = LandmarkDetector()
    g = OccupancyGrid(GridConfig())
    u2 = det.dirs[:, :2]
    n = np.linalg.norm(u2, axis=1)
    u2 = u2 / np.maximum(n, 1e-9)[:, None]
    for _ in range(200):                      # 反复扫同一面墙
        g.integrate(np.array([0.0, 0.0, 0.0]), 6.0 * n, u2, 16.0)
    p = g.probability()
    assert p.min() >= 0.0 and p.max() <= 1.0
    assert g.logodds.max() <= g.cfg.l_clamp + 1e-6


# --------------------------------------------------------------------------- #
def test_episode_subset_and_roundtrip():
    ep = VioEpisode(
        depth=np.zeros((6, 48, 64), np.uint8),
        imu=np.zeros((6, 10, 6), np.float32),
        dp=np.zeros((6, 3), np.float32), dr=np.zeros((6, 3), np.float32),
        pos=np.zeros((6, 3), np.float32), yaw=np.zeros(6, np.float32),
        obs_r=np.zeros((6, 12), np.float32), obs_b=np.zeros((6, 12), np.float32),
        obs_n=np.zeros(6, np.int32), lidar=np.zeros((6, 540), np.uint8))
    assert len(ep.subset(2, 5)) == 3
    ds = VioDataset([ep, ep], np.zeros((0, 2)))
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "ds.npz")
        ds.save(p)
        back = VioDataset.load(p)
    assert len(back.episodes) == 2
    assert back.episodes[0].depth.shape == (6, 48, 64)


# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not _HAS_MUJOCO, reason="需要 mujoco")
def test_collect_and_run_end_to_end():
    from uav_neural_vio_slam.core.dataset import EpisodeSpec, collect_episode
    from uav_neural_vio_slam.core.experiment import (BACKENDS, BackendConfig,
                                                     run_backend)
    from uav_neural_vio_slam.core.network import VioNet
    from uav_neural_vio_slam.core.training import predict_episode

    world = build_world(seed=0, n_poles=10, area=7.0, n_pillars=3)
    spec = EpisodeSpec(0, loops=0.4, ax=5.0, ay=3.5, height=1.6, period=18.0)
    ep = collect_episode(world, spec)
    assert len(ep) > 30
    ep._index = 0
    ep._gt_landmarks = world.positions
    pred = predict_episode(VioNet(), ep)
    assert pred["dp"].shape == (len(ep), 3)
    for backend in BACKENDS:
        r = run_backend(ep, pred, backend, BackendConfig(seed=0), make_map=False)
        assert np.isfinite(r.metrics["ate_raw"])
