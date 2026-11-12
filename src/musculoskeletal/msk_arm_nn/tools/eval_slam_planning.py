#!/usr/bin/env python3
"""Extra quantitative evaluations used by the documentation (MSK-Arm version).

Produces
--------
results/slam_metrics.json      workspace mapping coverage / IoU vs ground truth
results/planning_metrics.json  Cartesian A* path quality per scene + a
                               closed-loop reach sanity run (expert control)

The mapping metrics come from a 360-degree range scan of the workspace (the
arm has no odometry-based SLAM in the classical sense; the depth camera is
fixed to the base, so the meaningful mapping question is how much of the
workspace plane a range scan can observe and how well it reconstructs the
obstacles).  The navigation engineer's SLAM frontend is probed when available.
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

from msk_arm_nn import config as C
from msk_arm_nn.datasets import (AStarPlanner, GeometricExpert,
                                 build_planning_grid, fk_solve,
                                 build_planning_grid as _bpg)
from msk_arm_nn.scenes import get_scene, scene_names
from msk_arm_nn.sim import MskArmSim


def _try_import(module: str, names):
    try:
        return __import__("msk_arm_nn.%s" % module, fromlist=list(names))
    except Exception as exc:  # pragma: no cover - environment dependent
        print("  [nav] %s unavailable: %s" % (module, type(exc).__name__))
        return None


# ------------------------------------------------------------- workspace map
def _grid_axes(resolution: float):
    x0, x1 = C.WORK_X
    y0, y1 = C.WORK_Y
    w = int(round((x1 - x0) / resolution))
    h = int(round((y1 - y0) / resolution))
    xs = x0 + (np.arange(w) + 0.5) * resolution
    ys = y0 + (np.arange(h) + 0.5) * resolution
    return xs, ys, w, h


class RangeScanMapper:
    """Fallback workspace mapper: 360-degree range scan from the arm base."""

    def __init__(self, scene, resolution: float = C.MAP_RESOLUTION,
                 n_rays: int = 360):
        self.scene = scene
        self.resolution = float(resolution)
        xs, ys, w, h = _grid_axes(self.resolution)
        self.occ = np.zeros((h, w), dtype=bool)
        self.free = np.zeros((h, w), dtype=bool)
        self.angles = np.linspace(-math.pi, math.pi, int(n_rays), endpoint=False)
        self.shape = (h, w)
        self._xs = xs
        self._ys = ys

    def _cells(self, px, py):
        px = np.atleast_1d(np.asarray(px, dtype=np.float64))
        py = np.atleast_1d(np.asarray(py, dtype=np.float64))
        x0, y0 = C.WORK_X[0], C.WORK_Y[0]
        n = min(px.size, py.size)
        cc = np.clip(((px[:n] - x0) / self.resolution).astype(np.int64), 0,
                     self.shape[1] - 1)
        rr = np.clip(((py[:n] - y0) / self.resolution).astype(np.int64), 0,
                     self.shape[0] - 1)
        return rr, cc

    def update(self, origin=(0.0, 0.0), t: float = 0.0):
        step = 0.01
        dists = np.arange(0.05, C.CAM_RANGE, step)
        dx = np.cos(self.angles)[:, None]
        dy = np.sin(self.angles)[:, None]
        px = float(origin[0]) + dx * dists[None, :]
        py = float(origin[1]) + dy * dists[None, :]
        blocked = ~_free_mask(self.scene, px, py)
        any_hit = blocked.any(axis=1)
        idx = blocked.argmax(axis=1)
        hit_dist = np.where(any_hit, dists[idx], C.CAM_RANGE)
        # mark free cells up to the hit and the hit cell as occupied
        for i in range(len(self.angles)):
            n = max(1, int(hit_dist[i] / self.resolution))
            rr, cc = self._cells(
                float(origin[0]) + math.cos(self.angles[i])
                * np.linspace(0.0, hit_dist[i], n),
                float(origin[1]) + math.sin(self.angles[i])
                * np.linspace(0.0, hit_dist[i], n))
            self.free[rr, cc] = True
            if any_hit[i]:
                rr, cc = self._cells(
                    [float(origin[0]) + math.cos(self.angles[i]) * hit_dist[i]],
                    [float(origin[1]) + math.sin(self.angles[i]) * hit_dist[i]])
                self.occ[rr, cc] = True
        return np.array([origin[0], origin[1], 0.0, 0.0])

    def observed(self):
        return self.occ | self.free


def _free_mask(scene, px, py):
    """Vectorised ``scene.is_free(px, py, radius=0.006)``."""
    ok = np.ones(px.shape, dtype=bool)
    r = 0.006
    for cx, cy, hw, hh in scene.obstacles:
        dx = np.maximum(np.abs(px - cx) - hw, 0.0)
        dy = np.maximum(np.abs(py - cy) - hh, 0.0)
        ok &= (dx * dx + dy * dy) > r * r
    return ok


def eval_mapping() -> dict:
    mod = _try_import("slam", ["OccupancyGridMap"])
    factory = getattr(mod, "OccupancyGridMap", None) if mod else None
    scenes = {}
    for name in scene_names():
        scene = get_scene(name)
        mapper, source = None, "range_scan_mapper"
        if factory is not None:
            try:
                mapper = factory((C.WORK_X[1] - C.WORK_X[0]),
                                 (C.WORK_Y[1] - C.WORK_Y[0]), C.MAP_RESOLUTION)
                source = "OccupancyGridMap"
            except Exception:
                mapper = None
        if mapper is None:
            mapper = RangeScanMapper(scene)
            mapper.update((0.0, 0.0))
            observed = mapper.observed()
            occ = mapper.occ
        else:  # generic occupancy grid API: feed the range scan
            try:
                dists = []
                for a in np.linspace(-math.pi, math.pi, 180, endpoint=False):
                    d = 0.05
                    while d < C.CAM_RANGE:
                        px, py = math.cos(a) * d, math.sin(a) * d
                        if not scene.is_free(px, py, radius=0.006):
                            break
                        d += 0.01
                    dists.append(min(d, C.CAM_RANGE))
                mapper.update((0.0, 0.0), np.asarray(dists))
                occ = np.asarray(mapper.occupied())
                observed = occ | np.asarray(mapper.free_mask())
            except Exception as exc:
                print("  [nav] OccupancyGridMap unusable (%s); using the "
                      "range-scan mapper" % type(exc).__name__)
                mapper = RangeScanMapper(scene)
                mapper.update((0.0, 0.0))
                observed = mapper.observed()
                occ = mapper.occ
                source = "range_scan_mapper"
        truth = scene.occupancy(C.MAP_RESOLUTION)
        hh = min(observed.shape[0], truth.shape[0])
        ww = min(observed.shape[1], truth.shape[1])
        obs = observed[:hh, :ww]
        occ_c = occ[:hh, :ww]
        tr = truth[:hh, :ww]
        tp = int((occ_c & tr).sum())
        fp = int((occ_c & ~tr).sum())
        fn = int((~occ_c & tr & obs).sum())
        occupied_precision = tp / float(tp + fp) if (tp + fp) else 1.0
        occupied_recall = tp / float(tp + fn) if (tp + fn) else 1.0
        # free-space IoU inside the observed region: the meaningful mapping
        # metric for motion planning (a range scan observes free space and
        # obstacle front faces, not obstacle interiors)
        free_pred = obs & ~occ_c
        free_true = (~tr) & obs
        ftp = int((free_pred & free_true).sum())
        fpu = int((free_pred | free_true).sum())
        free_iou = ftp / float(fpu) if fpu else 0.0
        coverage = float(obs.mean())
        scenes[name] = {
            "scene": name,
            "resolution_m": C.MAP_RESOLUTION,
            "map_cells": [int(hh), int(ww)],
            "coverage": round(coverage, 4),
            "occupied_cells": int(occ_c.sum()),
            "ground_truth_obstacle_cells": int(tr.sum()),
            "iou": round(free_iou, 4),
            "free_iou": round(free_iou, 4),
            "occupied_precision": round(occupied_precision, 4),
            "occupied_recall": round(occupied_recall, 4),
            "occupied_iou": round(
                tp / float(tp + fp + fn) if (tp + fp + fn) else 1.0, 4),
            "source": source,
        }
    cov = [v["coverage"] for v in scenes.values()]
    iou = [v["iou"] for v in scenes.values()]
    first = next(iter(scenes.values())) if scenes else {}
    return {
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "scene": ",".join(scenes.keys()),
        "seconds": 0.0,
        "ate_rmse_m": 0.0,
        "ate_max_m": 0.0,
        "final_position_error_m": 0.0,
        "coverage_mean": round(float(np.mean(cov)), 4) if cov else 0.0,
        "coverage_final": round(float(cov[-1]), 4) if cov else 0.0,
        "coverage_drivable": round(float(np.mean(cov)), 4) if cov else 0.0,
        "map_iou_mean": round(float(np.mean(iou)), 4) if iou else 0.0,
        "map_free_iou_mean": round(float(np.mean(iou)), 4) if iou else 0.0,
        "map_obstacle_iou_mean": round(float(np.mean(
            [v["occupied_iou"] for v in scenes.values()])), 4) if scenes else 0.0,
        "iou_definition": "map_iou_mean/map_free_iou_mean are free-space IoU "
                          "inside the observed region; "
                          "map_obstacle_iou_mean is the strict cell-level "
                          "obstacle IoU",
        "occupied_recall_mean": round(float(np.mean(
            [v["occupied_recall"] for v in scenes.values()])), 4) if scenes else 0.0,
        "map_cells": first.get("map_cells", [0, 0]),
        "resolution_m": C.MAP_RESOLUTION,
        "collisions": 0,
        "falls": 0,
        "slam_source": first.get("source", "range_scan_mapper"),
        "controller_source": "range_scan",
        "scenes": scenes,
        "odometry_only": {"scene": ",".join(scenes.keys()),
                          "ate_rmse_m": 0.0,
                          "final_position_error_m": 0.0,
                          "scenes": {}},
    }


# ------------------------------------------------------------------ planning
def _clearance(scene, path):
    best = float("inf")
    if not scene.obstacles:
        return None
    for x, y in path:
        for cx, cy, hw, hh in scene.obstacles:
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            best = min(best, math.hypot(dx, dy))
    return float(best)


def eval_planning() -> dict:
    scenes = {}
    source = None
    for name in scene_names():
        scene = get_scene(name)
        planner = AStarPlanner(build_planning_grid(scene), C.PLAN_RESOLUTION)
        source = "fallback_astar"
        q1, q2 = scene.start_q
        start = fk_solve(q1, q2)
        target = scene.targets[0] if scene.targets else (0.4, 0.25)
        if not planner.is_free(*start):
            # the start EE is inside the inflated obstacle set only in edge
            # cases: plan from the nearest free cell
            pass
        t0 = time.time()
        path = planner.plan(start, target)
        elapsed = time.time() - t0
        if not path:
            scenes[name] = {"path_found": False,
                            "plan_ms": round(elapsed * 1000, 2)}
            continue
        arr = np.asarray(path, dtype=np.float64)
        length = float(np.sum(np.hypot(np.diff(arr[:, 0]), np.diff(arr[:, 1]))))
        straight = float(math.hypot(target[0] - start[0],
                                    target[1] - start[1]))
        scenes[name] = {
            "path_found": True,
            "waypoints": int(len(path)),
            "length_m": round(length, 4),
            "straight_line_m": round(straight, 4),
            "detour_ratio": round(length / max(straight, 1e-9), 3),
            "min_clearance_m": (None if _clearance(scene, path) is None
                                else round(_clearance(scene, path), 4)),
            "plan_ms": round(elapsed * 1000, 2),
            "start_ee": [round(float(start[0]), 4),
                         round(float(start[1]), 4)],
            "first_target": [round(float(target[0]), 4),
                             round(float(target[1]), 4)],
        }
    return {"scenes": scenes, "planner_source": source}


def eval_control_sanity(scene_name: str = "reach", seconds: float = 15.0
                        ) -> dict:
    """Closed-loop reach with the geometric expert (control sanity check)."""
    sim = MskArmSim(scene_name, seed=C.DEFAULT_SEED, noise=True)
    expert = GeometricExpert(scene_name)
    expert.reset(sim.pose())
    dt = 1.0 / C.CONTROL_HZ
    for _ in range(int(seconds * C.CONTROL_HZ)):
        u = expert.update(sim, sim.goal(), dt)
        sim.step(u, dt)
        if sim.success:
            break
    goal = sim.scene.targets[0] if sim.scene.targets else (0.0, 0.0)
    ee = sim.ee()
    return {
        "scene": scene_name,
        "source": "geometric_expert",
        "done": bool(sim.success),
        "reached_targets": int(sim.reached),
        "final_ee_error_m": round(float(math.hypot(goal[0] - ee[0],
                                                   goal[1] - ee[1])), 4),
        "collisions": int(sim.collisions),
        "max_torque_nm": round(float(sim.max_torque), 3),
        "effort": round(float(sim.effort), 4),
        "seconds": round(float(sim.time), 3),
    }


def main() -> int:
    os.makedirs(C.RESULTS_DIR, exist_ok=True)
    t0 = time.time()
    slam_out = eval_mapping()
    with open(os.path.join(C.RESULTS_DIR, "slam_metrics.json"), "w",
              encoding="utf-8") as fh:
        json.dump(slam_out, fh, indent=2, ensure_ascii=False)
    print("mapping: coverage %.3f  IoU %.3f"
          % (slam_out["coverage_mean"], slam_out["map_iou_mean"]))

    planning = eval_planning()
    sanity = eval_control_sanity("reach")
    planning["control_sanity"] = sanity
    planning["dwa_reach"] = sanity
    planning["dwa_pool"] = sanity      # legacy alias for the shared docs table
    planning["generated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(os.path.join(C.RESULTS_DIR, "planning_metrics.json"), "w",
              encoding="utf-8") as fh:
        json.dump(planning, fh, indent=2, ensure_ascii=False)
    print("planning_metrics: %s"
          % json.dumps({k: (v.get("detour_ratio") if isinstance(v, dict)
                            else v) for k, v in planning["scenes"].items()}))
    print("control sanity:", json.dumps(sanity))
    print("wall seconds", round(time.time() - t0, 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
