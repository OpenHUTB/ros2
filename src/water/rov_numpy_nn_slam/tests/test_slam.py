#!/usr/bin/env python3
"""SLAM 模块测试：占有栅格单测 + pool 场景建图与定位精度。

运行：``/home/lxy/roscx_runtime/venv/bin/python tests/test_slam.py``
指标：ATE RMSE（有/无扫描匹配）与地图覆盖率（%）。
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.geometry import wrap_angle
from rov_numpy_nn_slam.scenes import get_scene
from rov_numpy_nn_slam.sim import ROVSim
from rov_numpy_nn_slam.slam import OccupancyGridMap, SlamFrontend


# ------------------------------------------------------------------ unit tests
def test_occupancy_grid_map() -> None:
    grid = OccupancyGridMap(6.0, 4.0)
    assert grid.width == 120 and grid.height == 80
    pose = [1.0, 2.0, 0.0]
    beams = np.array([0.0, 0.05, -0.05])
    ranges = np.array([2.0, 2.002, 2.002])
    before = grid.log_odds.copy()
    grid.update(pose, np.full(3, C.SONAR_RANGE_MAX), beams)
    assert np.allclose(grid.log_odds, before), "no-return beams must be ignored"
    for _ in range(3):
        grid.update(pose, ranges, beams)

    c, r = grid.from_world(3.0, 2.0)
    window = grid.occupied()[max(0, r - 2): r + 3, max(0, c - 2): c + 3]
    assert bool(window.any()), "ray endpoints should mark occupied cells"
    fc, fr = grid.from_world(2.0, 2.0)
    assert bool(grid.free_mask()[fr, fc]), "traversed cells should become free"
    prob = grid.probability
    assert prob.shape == (grid.height, grid.width)
    assert prob.dtype == np.float32
    assert 0.0 <= float(prob.min()) and float(prob.max()) <= 1.0

    x, y = grid.to_world(c, r)
    c2, r2 = grid.from_world(x, y)
    assert (c2, r2) == (c, r)
    assert abs(x - 3.0) <= grid.resolution and abs(y - 2.0) <= grid.resolution

    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "map.npz")
    grid.save(path)
    other = OccupancyGridMap(1.0, 1.0)
    other.load(path)
    assert other.log_odds.shape == grid.log_odds.shape
    assert np.allclose(other.log_odds, grid.log_odds)
    assert abs(other.resolution - C.MAP_RESOLUTION) < 1e-9
    print("[unit] occupancy grid: occupied/free/save/load OK")


# ---------------------------------------------------------------- SLAM tour
_WAYPOINTS = [
    (2.4, 2.2), (4.6, 1.8), (7.0, 1.8), (9.5, 2.0), (11.6, 2.4),
    (12.6, 4.0), (12.5, 6.4), (9.6, 6.4), (7.5, 6.5), (5.0, 6.8),
    (2.0, 6.2), (1.9, 4.4), (1.8, 2.8),
]


def _drive_tour(use_scan_matching: bool, seed: int = 7,
                duration: float = 40.0):
    sim = ROVSim("pool", seed=seed, noise=True)
    slam = SlamFrontend(sim.scene, use_scan_matching=use_scan_matching)
    dt = 0.02
    n_steps = int(round(duration / dt))
    wp = 0
    truth_poses = []
    for k in range(n_steps):
        pose = sim.pose
        while wp < len(_WAYPOINTS):
            tx, ty = _WAYPOINTS[wp]
            if math.hypot(tx - pose[0], ty - pose[1]) < 0.8:
                wp += 1
            else:
                break
        if wp >= len(_WAYPOINTS):
            vx, wz = 0.0, 0.0
        else:
            tx, ty = _WAYPOINTS[wp]
            alpha = wrap_angle(math.atan2(ty - pose[1], tx - pose[0]) - pose[3])
            wz = float(np.clip(1.8 * alpha, -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
            vx = 0.90 * max(0.12, math.cos(alpha))
        heave = float(np.clip(0.8 * (sim.depth() - 1.9), -0.4, 0.4))
        sim.step([vx, 0.0, heave, wz], dt)
        if (k + 1) % 5 == 0:
            odom = sim.integrate_odometry(0.1)
            slam.update(odom, sim.sonar())
            truth_poses.append(sim.pose.copy())
    return sim, slam, truth_poses


def _coverage_percent(slam: SlamFrontend, scene, truth_poses, radius: float = 2.5):
    res = slam.map.resolution
    ground = scene.occupancy(res)
    known = np.abs(slam.map.log_odds) >= 0.25
    near = np.zeros_like(ground)
    h, w = ground.shape
    k = int(round(radius / res))
    for pose in truth_poses[::5]:
        c = int(pose[0] / res)
        r = int(pose[1] / res)
        c0, c1 = max(0, c - k), min(w, c + k + 1)
        r0, r1 = max(0, r - k), min(h, r + k + 1)
        near[r0:r1, c0:c1] = True
    region = near & (~ground)
    if not np.any(region):
        return 0.0
    return 100.0 * float(known[region].mean())


def main() -> int:
    print("== test_slam ==")
    test_occupancy_grid_map()

    sim_m, slam_m, truth_m = _drive_tour(use_scan_matching=True)
    ate_m = slam_m.ate_rmse(truth_m)
    cov_m = _coverage_percent(slam_m, sim_m.scene, truth_m)
    final = slam_m.trajectory[-1]
    gt = truth_m[-1]
    print("[slam] scan-matching ON : ATE RMSE %.3f m, coverage %.1f %%, "
          "final xy err %.3f m, collisions %d, map %dx%d"
          % (ate_m, cov_m, math.hypot(final[0] - gt[0], final[1] - gt[1]),
             sim_m.collisions, slam_m.map.width, slam_m.map.height))

    sim_o, slam_o, truth_o = _drive_tour(use_scan_matching=False)
    ate_o = slam_o.ate_rmse(truth_o)
    cov_o = _coverage_percent(slam_o, sim_o.scene, truth_o)
    print("[slam] odometry only  : ATE RMSE %.3f m, coverage %.1f %%"
          % (ate_o, cov_o))

    assert sim_m.collisions == 0, "SLAM tour must not collide"
    assert ate_m < 0.35, "scan-matching ATE %.3f m exceeds 0.35 m" % ate_m
    assert cov_m > 50.0, "map coverage %.1f %% too low" % cov_m
    # the sim's DVL/IMU odometry is already very accurate (ATE ~0.06 m), so the
    # trust-region scan matching must not blow the error budget up
    assert ate_m < ate_o + 0.20, \
        "scan matching degraded ATE from %.3f to %.3f m" % (ate_o, ate_m)
    print("[slam] PASS (ATE < 0.35 m, coverage > 50 %%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
