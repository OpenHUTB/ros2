#!/usr/bin/env python3
"""Extra quantitative evaluations used by the documentation.

Produces
--------
results/slam_metrics.json      SLAM ATE / coverage / map quality vs odometry-only
results/planning_metrics.json  A* path quality per scene + DWA closed-loop check

These numbers are measured on the real MuJoCo simulation, not estimated.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.control import MissionController
from rov_numpy_nn_slam.planning import DWAPlanner, GridPlanner
from rov_numpy_nn_slam.scenes import get_scene, scene_names
from rov_numpy_nn_slam.sim import ROVSim
from rov_numpy_nn_slam.slam import SlamFrontend


def _clearance(scene, path) -> float:
    """Minimum distance from path vertices to the analytic obstacle shapes."""
    best = float("inf")
    for x, y in path:
        for cx, cy, hw, hh in scene.rects:
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            best = min(best, math.hypot(dx, dy))
        for cx, cy, r in scene.circles:
            best = min(best, max(0.0, math.hypot(x - cx, y - cy) - r))
        best = min(best, x - 0.12, scene.width - 0.12 - x,
                   y - 0.12, scene.height - 0.12 - y)
    return float(best)


def eval_slam(scene_name: str = "pool", seconds: float = 40.0) -> dict:
    sim = ROVSim(scene_name, seed=C.DEFAULT_SEED, noise=True)
    slam = SlamFrontend(sim.scene)
    mission = MissionController(sim.scene)
    goal = sim.goal()
    truth, est, map_cov = [], [], []
    t0 = time.time()
    steps = int(seconds * C.CONTROL_HZ)
    for _ in range(steps):
        scan = sim.sonar()
        odom = sim.integrate_odometry(1.0 / C.CONTROL_HZ)
        p = slam.update(odom, scan)
        cmd = mission.update(p, goal, scan, sim.sonar_beam_angles(),
                             1.0 / C.CONTROL_HZ)
        sim.step(cmd)
        truth.append(sim.pose.copy())
        est.append(np.asarray(p, dtype=np.float64))
        map_cov.append(float((slam.map.occupied() | slam.map.free_mask()).mean()))
        if (math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1]) < 0.7
                and abs(sim.pose[2] - goal[2]) < 0.6):
            break
    truth = np.asarray(truth)
    est = np.asarray(est)
    err = np.asarray([math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(truth, est)])
    out = {
        "scene": scene_name,
        "seconds": float(sim.time),
        "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))),
        "ate_max_m": float(err.max()) if len(err) else 0.0,
        "final_position_error_m": float(err[-1]) if len(err) else 0.0,
        "coverage_mean": float(np.mean(map_cov)),
        "coverage_final": float(map_cov[-1]) if map_cov else 0.0,
        "map_cells": list(slam.map.occupied().shape),
        "resolution_m": C.MAP_RESOLUTION,
        "collisions": int(sim.collisions),
        "wall_seconds": round(time.time() - t0, 2),
    }
    return out


def eval_slam_odometry(scene_name: str = "pool", seconds: float = 40.0) -> dict:
    """Odometry-only reference trajectory error for the same tour."""
    sim = ROVSim(scene_name, seed=C.DEFAULT_SEED, noise=True)
    mission = MissionController(sim.scene)
    goal = sim.goal()
    truth, odom_est = [], []
    for _ in range(int(seconds * C.CONTROL_HZ)):
        scan = sim.sonar()
        odom = sim.integrate_odometry(1.0 / C.CONTROL_HZ)
        cmd = mission.update(odom, goal, scan, sim.sonar_beam_angles(),
                             1.0 / C.CONTROL_HZ)
        sim.step(cmd)
        truth.append(sim.pose.copy())
        odom_est.append(np.asarray(odom, dtype=np.float64))
        if (math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1]) < 0.7
                and abs(sim.pose[2] - goal[2]) < 0.6):
            break
    truth = np.asarray(truth)
    odom_est = np.asarray(odom_est)
    err = np.asarray([math.hypot(a[0] - b[0], a[1] - b[1])
                      for a, b in zip(truth, odom_est)])
    return {"scene": scene_name, "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))),
            "final_position_error_m": float(err[-1]), "seconds": float(sim.time)}


def eval_planning() -> dict:
    scenes = {}
    for name in scene_names():
        scene = get_scene(name)
        grid = scene.coarse_map(C.PLAN_RESOLUTION)
        planner = GridPlanner(grid, C.PLAN_RESOLUTION)
        t0 = time.time()
        path = planner.plan(scene.start[:2], scene.goal[:2])
        elapsed = time.time() - t0
        if not path:
            scenes[name] = {"path_found": False}
            continue
        length = planner.path_length(path)
        straight = math.hypot(scene.goal[0] - scene.start[0],
                              scene.goal[1] - scene.start[1])
        scenes[name] = {
            "path_found": True,
            "waypoints": len(path),
            "length_m": round(length, 3),
            "straight_line_m": round(straight, 3),
            "detour_ratio": round(length / straight, 3),
            "min_clearance_m": round(_clearance(scene, path), 3),
            "plan_ms": round(elapsed * 1000, 2),
        }
    # DWA closed-loop sanity run in the pool
    sim = ROVSim("pool", seed=C.DEFAULT_SEED, noise=True)
    dwa = DWAPlanner()
    start = sim.pose.copy()
    goal = np.array([start[0] + 6.0, start[1] + 0.6])
    steps = 0
    for _ in range(int(25 * C.CONTROL_HZ)):
        scan = sim.sonar()
        vx, wz = dwa.compute(sim.pose, goal, scan, sim.sonar_beam_angles())
        depth = mission_depth = 0.0
        sim.step([vx, 0.0, depth, wz])
        steps += 1
        if math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1]) < 0.4:
            break
    dwa_result = {
        "done": bool(math.hypot(sim.pose[0] - goal[0],
                                sim.pose[1] - goal[1]) < 0.5),
        "final_distance_m": round(float(math.hypot(sim.pose[0] - goal[0],
                                                   sim.pose[1] - goal[1])), 3),
        "collisions": int(sim.collisions),
        "seconds": round(float(sim.time), 2),
    }
    return {"scenes": scenes, "dwa_pool": dwa_result}


def main() -> int:
    os.makedirs(C.RESULTS_DIR, exist_ok=True)
    t0 = time.time()
    slam = eval_slam()
    slam["odometry_only"] = eval_slam_odometry()
    slam["generated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(os.path.join(C.RESULTS_DIR, "slam_metrics.json"), "w") as fh:
        json.dump(slam, fh, indent=2, ensure_ascii=False)
    print("slam_metrics:", json.dumps(slam, ensure_ascii=False))

    planning = eval_planning()
    planning["generated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(os.path.join(C.RESULTS_DIR, "planning_metrics.json"), "w") as fh:
        json.dump(planning, fh, indent=2, ensure_ascii=False)
    print("planning_metrics:", json.dumps(planning, ensure_ascii=False))
    print("wall seconds", round(time.time() - t0, 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
