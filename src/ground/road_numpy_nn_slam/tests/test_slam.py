#!/usr/bin/env python3
"""SLAM tests: occupancy-grid unit tests + a 60 s downtown mapping/localisation
patrol (out and back along the main street).

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_slam.py``
Metrics: ATE RMSE, map coverage (%), final xy error, collisions.

Downtown is 140 x 140 m, i.e. 1400 x 1400 = 1.96 M occupancy cells at
``config.MAP_RESOLUTION`` (0.10 m); the front end must stay vectorised.
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.control import MissionController
from road_numpy_nn_slam.scenes import get_scene
from road_numpy_nn_slam.sim import RoadSim
from road_numpy_nn_slam.slam import OccupancyGridMap, SlamFrontend, lidar_beam_angles


# ------------------------------------------------------------------ unit tests
def test_occupancy_grid_map() -> None:
    grid = OccupancyGridMap(6.0, 4.0, C.MAP_RESOLUTION)
    assert grid.width == int(round(6.0 / C.MAP_RESOLUTION)), grid.width
    assert grid.height == int(round(4.0 / C.MAP_RESOLUTION)), grid.height
    assert (grid.width, grid.height) == (60, 40)     # 0.10 m cells
    beams = np.array([0.0, 0.05, -0.05])
    # a no-return beam (range == LIDAR_RANGE_MAX) must be ignored completely
    before = grid.log_odds.copy()
    grid.update([1.0, 2.0, 0.0], np.full(3, C.LIDAR_RANGE_MAX), beams)
    assert np.allclose(grid.log_odds, before), "no-return beams must be ignored"
    for _ in range(3):
        grid.update([1.0, 2.0, 0.0], np.array([2.0, 2.0, 2.0]), beams)

    c, r = grid.from_world(3.0, 2.0)
    window = grid.occupied()[max(0, r - 4): r + 5, max(0, c - 4): c + 5]
    assert bool(window.any()), "ray endpoints should mark occupied cells"
    fc, fr = grid.from_world(2.0, 2.0)
    assert bool(grid.free_mask()[fr, fc]), "traversed cells should become free"
    prob = grid.probability
    assert prob.shape == (grid.height, grid.width)
    assert prob.dtype == np.float32
    assert 0.0 <= float(prob.min()) and float(prob.max()) <= 1.0
    known = grid.known_mask()
    assert known.shape == prob.shape and bool(known[fr, fc])

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

    # front end: map must be scene.width x scene.height at MAP_RESOLUTION and
    # the z slot of the pose is passed through untouched (planar vehicle)
    scene = get_scene("downtown")
    slam = SlamFrontend(scene)
    assert slam.map.width == 1400 and slam.map.height == 1400
    assert lidar_beam_angles().size == C.LIDAR_BEAMS
    assert abs(lidar_beam_angles()[0] + math.pi) < 1e-9
    no_return = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    pose = slam.update([10.0, 12.0, 0.0, 0.3], no_return)
    assert abs(pose[2]) < 1e-12 and abs(pose[3] - 0.3) < 1e-9
    pose = slam.update([10.0, 12.0, 0.0, 0.3], no_return)
    assert abs(pose[2]) < 1e-12, "z slot must stay a plain passthrough"
    print("[unit] occupancy grid: occupied/free/save/load, 1400x1400 map, "
          "z pass-through OK")


# ---------------------------------------------------------------- SLAM tour
def _drive_tour(use_scan_matching: bool, duration: float = 60.0,
                seed: int = 7):
    """Patrol downtown (goal and back); feed odometry + lidar to SLAM at 5 Hz."""
    scene = get_scene("downtown")
    sim = RoadSim("downtown", seed=seed, noise=True)
    slam = SlamFrontend(scene, use_scan_matching=use_scan_matching)
    mission = MissionController(scene)
    goals = [(130.0, 70.0), (10.0, 70.0)]
    goal_index = 0
    dt = 0.05
    n_steps = int(round(duration / dt))
    scan = sim.lidar()
    angles = sim.lidar_beam_angles()
    truth_poses = []
    for k in range(n_steps):
        if k % 4 == 0:
            scan = sim.lidar()
            angles = sim.lidar_beam_angles()
        goal = np.array([goals[goal_index][0], goals[goal_index][1], 0.0])
        cmd = mission.update(sim.pose, goal, scan, angles, dt,
                             light=sim.traffic_light(),
                             pedestrians=scene.pedestrian_positions(sim.time))
        sim.step(cmd, dt)
        if math.hypot(sim.pose[0] - goals[goal_index][0],
                      sim.pose[1] - goals[goal_index][1]) < 6.0:
            goal_index = 1 - goal_index
        if (k + 1) % 4 == 0:
            odom = sim.integrate_odometry(dt * 4)
            slam.update(odom, scan)
            truth_poses.append(sim.pose.copy())
    return sim, slam, truth_poses


def _coverage_percent(slam: SlamFrontend, scene, truth_poses,
                      radius: float = 2.5) -> float:
    """Fraction of known free cells within ``radius`` m of the true trajectory."""
    res = slam.map.resolution
    ground = scene.occupancy(res)
    known = slam.map.known_mask(0.25)
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
    final_m = slam_m.final_xy_error(truth_m)
    print("[slam] scan-matching ON : ATE RMSE %.3f m, coverage %.1f %%, "
          "final xy err %.3f m, collisions %d, red-light %d, map %dx%d, "
          "updates %d"
          % (ate_m, cov_m, final_m, sim_m.collisions,
             sim_m.red_light_violations, slam_m.map.width, slam_m.map.height,
             slam_m.updates))

    sim_o, slam_o, truth_o = _drive_tour(use_scan_matching=False, duration=30.0)
    ate_o = slam_o.ate_rmse(truth_o)
    cov_o = _coverage_percent(slam_o, sim_o.scene, truth_o)
    print("[slam] odometry only  : ATE RMSE %.3f m, coverage %.1f %% "
          "(30 s run), collisions %d" % (ate_o, cov_o, sim_o.collisions))

    assert sim_m.collisions == 0, "SLAM patrol must not collide"
    assert ate_m < 1.5, "scan-matching ATE %.3f m exceeds 1.5 m" % ate_m
    assert cov_m > 50.0, "map coverage %.1f %% too low" % cov_m
    # scan matching must not blow the error budget up compared to raw odometry
    assert ate_m < ate_o + 1.0, \
        "scan matching degraded ATE from %.3f to %.3f m" % (ate_o, ate_m)
    print("[slam] PASS (ATE < 1.5 m, coverage > 50 %)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
