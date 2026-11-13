#!/usr/bin/env python3
"""SLAM tests: occupancy-grid unit tests + a 60 s city mapping/localisation tour.

Run: ``python3 tests/test_slam.py``
Metrics: ATE RMSE (scan matching on/off) and map coverage (%).

The city airspace is 130 x 130 m, i.e. 650 x 650 occupancy cells at
``config.MAP_RESOLUTION`` (0.20 m); the front end must stay vectorised to cope.
Pose convention is ``[x, y, altitude, yaw]`` with the altitude positive up and
passed through untouched.
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.planning import GridPlanner
from uav_numpy_nn_slam.scenes import get_scene
from uav_numpy_nn_slam.sim import UAVSim
from uav_numpy_nn_slam.slam import OccupancyGridMap, SlamFrontend, lidar_beam_angles


# ------------------------------------------------------------------ unit tests
def test_occupancy_grid_map() -> None:
    grid = OccupancyGridMap(6.0, 4.0, C.MAP_RESOLUTION)
    assert grid.width == int(round(6.0 / C.MAP_RESOLUTION)), grid.width
    assert grid.height == int(round(4.0 / C.MAP_RESOLUTION)), grid.height
    beams = np.array([0.0, 0.05, -0.05])
    # a no-return beam (range == LIDAR_RANGE_MAX) must be ignored completely
    before = grid.log_odds.copy()
    grid.update([1.0, 2.0, 0.0], np.full(3, C.LIDAR_RANGE_MAX), beams)
    assert np.allclose(grid.log_odds, before), "no-return beams must be ignored"
    for _ in range(3):
        grid.update([1.0, 2.0, 0.0], np.array([2.0, 2.0, 2.0]), beams)

    c, r = grid.from_world(3.0, 2.0)
    window = grid.occupied()[max(0, r - 2): r + 3, max(0, c - 2): c + 3]
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

    # slam front end: grid must be scene.width x scene.height at MAP_RESOLUTION,
    # and the altitude slot must be passed through untouched
    scene = get_scene("city")
    slam = SlamFrontend(scene)
    assert slam.map.width == int(round(scene.width / C.MAP_RESOLUTION))
    assert slam.map.height == int(round(scene.height / C.MAP_RESOLUTION))
    assert slam.map.width == 650 and slam.map.height == 650
    no_return = np.full(C.LIDAR_BEAMS, C.LIDAR_RANGE_MAX)
    pose = slam.update([10.0, 12.0, 33.5, 0.3], no_return)
    assert abs(pose[2] - 33.5) < 1e-9, "altitude slot must not be sign-flipped"
    assert abs(pose[3] - 0.3) < 1e-9
    assert len(slam.trajectory) == 1
    assert lidar_beam_angles().size == C.LIDAR_BEAMS
    assert abs(lidar_beam_angles()[0] + math.pi) < 1e-9, "full-turn beam convention"
    print("[unit] occupancy grid: occupied/free/save/load, altitude pass-through OK")


# ---------------------------------------------------------------- SLAM tour
def _tour_path(scene):
    """A* path through the city streets (start -> goal), used as the tour."""
    planner = GridPlanner(scene.planning_map(C.PLAN_RESOLUTION),
                          C.PLAN_RESOLUTION)
    start = (float(scene.start[0]), float(scene.start[1]))
    goal = (float(scene.goal[0]), float(scene.goal[1]))
    path = planner.plan(start, goal)
    if path is None:
        path = [start, goal]
    return [(float(p[0]), float(p[1])) for p in path]


def _lookahead_target(pts, x, y, lookahead):
    """Closest-point projection + arc-length lookahead on a polyline."""
    best = (math.inf, 0, 0.0)
    for j in range(len(pts) - 1):
        ax, ay = pts[j]
        bx, by = pts[j + 1]
        abx, aby = bx - ax, by - ay
        seg2 = abx * abx + aby * aby
        t = 0.0 if seg2 <= 1e-12 else \
            max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / seg2))
        px, py = ax + t * abx, ay + t * aby
        d = math.hypot(px - x, py - y)
        if d < best[0]:
            best = (d, j, t)
    _, j0, t0 = best
    ax, ay = pts[j0]
    bx, by = pts[j0 + 1]
    seg_len = math.hypot(bx - ax, by - ay)
    remain = (1.0 - t0) * seg_len
    if remain >= lookahead:
        ratio = lookahead / max(seg_len, 1e-9)
        return ax + (t0 + ratio) * (bx - ax), ay + (t0 + ratio) * (by - ay)
    for j in range(j0 + 1, len(pts) - 1):
        sx, sy = pts[j]
        nx, ny = pts[j + 1]
        seg_len = math.hypot(nx - sx, ny - sy)
        if remain + seg_len >= lookahead:
            ratio = (lookahead - remain) / max(seg_len, 1e-9)
            return sx + ratio * (nx - sx), sy + ratio * (ny - sy)
        remain += seg_len
    return pts[-1]


def _drive_tour(use_scan_matching: bool, seed: int = 7,
                duration: float = 60.0, speed: float = 3.5):
    """Fly the city tour; feed odometry + lidar to SLAM at 5 Hz."""
    scene = get_scene("city")
    sim = UAVSim("city", seed=seed, noise=True)
    slam = SlamFrontend(scene, use_scan_matching=use_scan_matching)
    pts = _tour_path(scene)
    loop = pts + list(reversed(pts[:-1]))       # out-and-back tour
    dt = 0.05
    n_steps = int(round(duration / dt))
    scan_every = 4                              # 5 Hz slam front end
    truth_poses = []
    for k in range(n_steps):
        pose = sim.pose
        tx, ty = _lookahead_target(loop, pose[0], pose[1], 2.0)
        dx, dy = tx - pose[0], ty - pose[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            vx, vy = 0.0, 0.0
        else:
            vx, vy = speed * dx / d, speed * dy / d
        sim.step([vx, vy, 0.0, 0.0], dt)
        if (k + 1) % scan_every == 0:
            odom = sim.integrate_odometry(dt * scan_every)
            slam.update(odom, sim.lidar())
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
          "final xy err %.3f m, collisions %d, map %dx%d, updates %d"
          % (ate_m, cov_m, final_m, sim_m.collisions,
             slam_m.map.width, slam_m.map.height, slam_m.updates))

    sim_o, slam_o, truth_o = _drive_tour(use_scan_matching=False, duration=30.0)
    ate_o = slam_o.ate_rmse(truth_o)
    cov_o = _coverage_percent(slam_o, sim_o.scene, truth_o)
    print("[slam] odometry only  : ATE RMSE %.3f m, coverage %.1f %% (30 s run), "
          "collisions %d" % (ate_o, cov_o, sim_o.collisions))

    assert sim_m.collisions == 0, "SLAM tour must not collide"
    assert ate_m < 3.0, "scan-matching ATE %.3f m exceeds 3.0 m" % ate_m
    assert cov_m > 50.0, "map coverage %.1f %% too low" % cov_m
    # scan matching must not blow the error budget up compared to raw odometry
    assert ate_m < ate_o + 2.0, \
        "scan matching degraded ATE from %.3f to %.3f m" % (ate_o, ate_m)
    print("[slam] PASS (ATE < 3.0 m, coverage > 50 %)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
