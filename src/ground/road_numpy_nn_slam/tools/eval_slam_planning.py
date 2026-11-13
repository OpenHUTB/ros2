#!/usr/bin/env python3
"""Extra quantitative evaluations used by the documentation (road version).

Produces
--------
results/slam_metrics.json      SLAM ATE / coverage / map quality vs odometry-only
results/planning_metrics.json  A* path quality per scene + local-planner check

Everything is measured on the real bicycle simulation.  The SLAM / planning /
control modules owned by the navigation engineer are imported lazily and
probed; if they are not importable (or fail at run time) the tool falls back to
the self-contained implementations in :mod:`road_numpy_nn_slam.datasets` and records
the actual ``source`` that was used.
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

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.datasets import (AStarPlanner, GeometricExpert,
                                  build_cost_map, build_planning_grid,
                                  reactive_action)
from road_numpy_nn_slam.geometry import polyline_length
from road_numpy_nn_slam.scenes import get_scene, scene_names
from road_numpy_nn_slam.sim import RoadSim


# --------------------------------------------------------------- imports
def _try_import(module: str, names):
    try:
        mod = __import__("road_numpy_nn_slam.%s" % module, fromlist=list(names))
    except Exception as exc:  # pragma: no cover - environment dependent
        print("  [nav] %s unavailable: %s" % (module, type(exc).__name__))
        return None
    return mod


def _make_mission(scene):
    mod = _try_import("control", ["MissionController"])
    factory = getattr(mod, "MissionController", None) if mod else None
    if factory is not None:
        for args in ((scene,), (scene, False)):
            try:
                return factory(*args), "MissionController"
            except Exception:
                continue
        print("  [nav] MissionController could not be constructed; using the "
              "geometric expert")
    return GeometricExpert(scene.name), "fallback_geometric_expert"


# ------------------------------------------------------------- fallback slam
class FallbackMapper:
    """Odometry-only occupancy mapper used when ``slam.py`` is unavailable.

    It deliberately performs *no* scan matching: the pose estimate is the
    filtered odometry, which makes the ATE a real measurement of the
    dead-reckoning + GNSS fuse in :meth:`RoadSim.integrate_odometry`.
    """

    def __init__(self, scene, resolution: float = C.MAP_RESOLUTION):
        self.scene = scene
        self.resolution = float(resolution)
        self.rows = int(round(scene.height / self.resolution))
        self.cols = int(round(scene.width / self.resolution))
        self.occ = np.zeros((self.rows, self.cols), dtype=bool)
        self.free = np.zeros((self.rows, self.cols), dtype=bool)
        self.angles = np.linspace(-math.pi, math.pi, C.LIDAR_BEAMS,
                                  endpoint=False)

    def _cells(self, xs, ys):
        cc = np.clip((xs / self.resolution).astype(np.int64), 0, self.cols - 1)
        rr = np.clip((ys / self.resolution).astype(np.int64), 0, self.rows - 1)
        return rr, cc

    def update(self, odom_pose, scan):
        pose = np.asarray(odom_pose, dtype=np.float64)
        ranges = np.asarray(scan, dtype=np.float64)
        world = self.angles + float(pose[3])
        px = pose[0] + ranges * np.cos(world)
        py = pose[1] + ranges * np.sin(world)
        rr, cc = self._cells(px, py)
        self.occ[rr, cc] = True
        frac = np.linspace(0.0, 1.0, 10)[None, :]
        sx = pose[0] + frac * (px - pose[0])[:, None]
        sy = pose[1] + frac * (py - pose[1])[:, None]
        rr, cc = self._cells(sx.reshape(-1), sy.reshape(-1))
        self.free[rr, cc] = True
        return pose.copy()

    def coverage(self) -> float:
        return float((self.occ | self.free).mean())

    def shape(self):
        return [int(self.rows), int(self.cols)]


def _make_slam(scene):
    mod = _try_import("slam", ["SlamFrontend"])
    factory = getattr(mod, "SlamFrontend", None) if mod else None
    if factory is not None:
        try:
            obj = factory(scene)
            return obj, "SlamFrontend"
        except Exception as exc:
            print("  [nav] SlamFrontend init failed (%s); using fallback mapper"
                  % type(exc).__name__)
    return FallbackMapper(scene), "fallback_odometry_mapper"


def _slam_coverage(mapper) -> float:
    try:
        occ = mapper.map.occupied()
        free = mapper.map.free_mask()
        return float((np.asarray(occ) | np.asarray(free)).mean())
    except Exception:
        pass
    try:
        return float(mapper.coverage())
    except Exception:
        return 0.0


def _map_cells(mapper):
    try:
        return list(np.asarray(mapper.map.occupied()).shape)
    except Exception:
        pass
    try:
        return mapper.shape()
    except Exception:
        return [0, 0]


# ------------------------------------------------------------------ slam eval
def _fly_tour(scene_name: str, seconds: float, use_slam: bool):
    sim = RoadSim(scene_name, seed=C.DEFAULT_SEED, noise=True)
    scene = sim.scene
    if use_slam:
        mapper, mapper_source = _make_slam(scene)
    else:
        mapper, mapper_source = None, "odometry_only"
    mission, mission_source = _make_mission(scene)
    goal = sim.goal()
    dt = 1.0 / C.CONTROL_HZ
    truth, est, cover = [], [], []
    if hasattr(mission, "reset"):
        try:
            mission.reset(sim.pose)
        except Exception:
            pass
    for _ in range(int(seconds * C.CONTROL_HZ)):
        pose = sim.pose.copy()
        scan = sim.lidar()
        odom = sim.integrate_odometry(dt)
        if mapper is not None:
            try:
                p = np.asarray(mapper.update(odom, scan), dtype=np.float64)
            except Exception as exc:
                print("  [nav] SLAM update failed (%s); falling back to "
                      "odometry" % type(exc).__name__)
                mapper = None
                mapper_source = "fallback_odometry_mapper"
                p = odom
            cover.append(_slam_coverage(mapper) if mapper is not None else 0.0)
        else:
            p = odom
            cover.append(0.0)
        if hasattr(mission, "set_speed"):
            try:
                mission.set_speed(sim.speed)
            except Exception:
                pass
        try:
            try:
                cmd = mission.update(pose, goal, scan, sim.lidar_beam_angles(),
                                     dt, t=sim.time)
            except TypeError:
                cmd = mission.update(pose, goal, scan, sim.lidar_beam_angles(),
                                     dt)
        except Exception:
            cmd = GeometricExpert(scene_name).update(
                pose, goal, scan, sim.lidar_beam_angles(), dt, t=sim.time)
        sim.step(cmd, dt)
        truth.append(sim.pose.copy())
        est.append(np.asarray(p, dtype=np.float64))
        if math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1]) <= 3.0:
            break
    truth = np.asarray(truth)
    est = np.asarray(est)
    if len(truth) == 0:
        return None
    err = np.asarray([math.hypot(a[0] - b[0], a[1] - b[1])
                      for a, b in zip(truth, est)])
    coverage_drivable = 0.0
    try:
        occ = np.asarray(mapper.map.occupied()) if mapper is not None else None
        free = np.asarray(mapper.map.free_mask()) if mapper is not None else None
        if occ is not None:
            drivable = scene.drivable_mask(C.MAP_RESOLUTION)
            hh = min(occ.shape[0], drivable.shape[0])
            ww = min(occ.shape[1], drivable.shape[1])
            mapped = (occ[:hh, :ww] | free[:hh, :ww])
            dr = drivable[:hh, :ww]
            coverage_drivable = float(mapped[dr].mean()) if dr.any() else 0.0
    except Exception:
        coverage_drivable = 0.0
    return {
        "scene": scene_name,
        "seconds": round(float(sim.time), 2),
        "coverage_drivable": round(coverage_drivable, 4),
        "ate_rmse_m": round(float(np.sqrt(np.mean(err ** 2))), 4),
        "ate_max_m": round(float(err.max()), 4),
        "final_position_error_m": round(float(err[-1]), 4),
        "coverage_mean": round(float(np.mean(cover)), 4),
        "coverage_final": round(float(cover[-1]), 4),
        "map_cells": _map_cells(mapper) if mapper is not None else [0, 0],
        "resolution_m": C.MAP_RESOLUTION,
        "collisions": int(sim.collisions),
        "red_light_violations": int(sim.red_light_violations),
        "slam_source": mapper_source,
        "controller_source": mission_source,
    }


def _odometry_reference(scene_name: str, seconds: float):
    sim = RoadSim(scene_name, seed=C.DEFAULT_SEED, noise=True)
    mission, source = _make_mission(sim.scene)
    goal = sim.goal()
    dt = 1.0 / C.CONTROL_HZ
    truth, odom_est = [], []
    if hasattr(mission, "reset"):
        try:
            mission.reset(sim.pose)
        except Exception:
            pass
    for _ in range(int(seconds * C.CONTROL_HZ)):
        pose = sim.pose.copy()
        scan = sim.lidar()
        odom = sim.integrate_odometry(dt)
        if hasattr(mission, "set_speed"):
            try:
                mission.set_speed(sim.speed)
            except Exception:
                pass
        try:
            try:
                cmd = mission.update(pose, goal, scan, sim.lidar_beam_angles(),
                                     dt, t=sim.time)
            except TypeError:
                cmd = mission.update(pose, goal, scan, sim.lidar_beam_angles(),
                                     dt)
        except Exception:
            cmd = GeometricExpert(scene_name).update(
                pose, goal, scan, sim.lidar_beam_angles(), dt, t=sim.time)
        sim.step(cmd, dt)
        truth.append(sim.pose.copy())
        odom_est.append(np.asarray(odom, dtype=np.float64))
        if math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1]) <= 3.0:
            break
    truth = np.asarray(truth)
    odom_est = np.asarray(odom_est)
    err = np.asarray([math.hypot(a[0] - b[0], a[1] - b[1])
                      for a, b in zip(truth, odom_est)])
    return {"scene": scene_name,
            "ate_rmse_m": round(float(np.sqrt(np.mean(err ** 2))), 4)
            if len(err) else 0.0,
            "final_position_error_m": round(float(err[-1]), 4) if len(err)
            else 0.0,
            "seconds": round(float(sim.time), 2),
            "controller_source": source}


# -------------------------------------------------------------- planning eval
def _clearance(scene, path) -> float:
    """Minimum distance from path vertices to the analytic obstacle shapes."""
    best = float("inf")
    for x, y in path:
        for cx, cy, hw, hh in list(scene.parked) + list(scene.barriers):
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            best = min(best, math.hypot(dx, dy))
        for cx, cy, r in scene.cones:
            best = min(best, max(0.0, math.hypot(x - cx, y - cy) - r))
        best = min(best, x - 0.2, scene.width - 0.2 - x,
                   y - 0.2, scene.height - 0.2 - y)
    return float(best)


def _make_grid_planner(scene):
    """Prefer the navigation engineer's GridPlanner on the scene occupancy."""
    mod = _try_import("planning", ["GridPlanner"])
    factory = getattr(mod, "GridPlanner", None) if mod else None
    if factory is not None:
        grid = scene.coarse_map(C.PLAN_RESOLUTION)
        for args, kwargs in (((grid, C.PLAN_RESOLUTION), {}),
                             ((grid,), {}),
                             ((), {"occupancy_bool": grid,
                                   "resolution": C.PLAN_RESOLUTION})):
            try:
                planner = factory(*args, **kwargs)
                if hasattr(planner, "plan"):
                    return planner, "GridPlanner"
            except Exception:
                continue
        print("  [nav] GridPlanner unusable; using the built-in weighted A*")
    grid = build_planning_grid(scene, C.PLAN_RESOLUTION)
    cost = build_cost_map(scene, C.PLAN_RESOLUTION)
    return AStarPlanner(grid, C.PLAN_RESOLUTION, cost), "fallback_astar"


def _path_len(planner, path) -> float:
    fn = getattr(planner, "path_length", None)
    if callable(fn):
        try:
            return float(fn(path))
        except Exception:
            pass
    return float(polyline_length(path))


def eval_planning() -> dict:
    scenes = {}
    source = None
    for name in scene_names():
        scene = get_scene(name)
        planner, source = _make_grid_planner(scene)
        t0 = time.time()
        try:
            path = planner.plan(scene.start[:2], scene.goal[:2])
        except Exception as exc:
            print("  [nav] planner failed on %s (%s)" % (name,
                                                         type(exc).__name__))
            path = None
        elapsed = time.time() - t0
        if not path:
            scenes[name] = {"path_found": False,
                            "plan_ms": round(elapsed * 1000, 2)}
            continue
        path = [tuple(float(v) for v in p[:2]) for p in path]
        length = _path_len(planner, path)
        straight = math.hypot(scene.goal[0] - scene.start[0],
                              scene.goal[1] - scene.start[1])
        scenes[name] = {
            "path_found": True,
            "waypoints": len(path),
            "length_m": round(length, 3),
            "straight_line_m": round(straight, 3),
            "detour_ratio": round(length / max(straight, 1e-6), 3),
            "min_clearance_m": round(_clearance(scene, path), 3),
            "plan_ms": round(elapsed * 1000, 2),
        }
    return {"scenes": scenes, "planner_source": source}


def eval_local_planner(scene_name: str = "downtown", seconds: float = 40.0
                       ) -> dict:
    """Closed-loop sanity run of the local planner (DWA if available)."""
    sim = RoadSim(scene_name, seed=C.DEFAULT_SEED, noise=True)
    start = sim.pose.copy()
    goal = np.array([min(sim.scene.width - 5.0, start[0] + 40.0),
                     start[1]], dtype=np.float64)
    mod = _try_import("planning", ["DWAPlanner"])
    factory = getattr(mod, "DWAPlanner", None) if mod else None
    planner, source = None, "reactive_lane_following"
    if factory is not None:
        for kwargs in ({"scene": sim.scene}, {}, {"scene": sim.scene,
                                                  "max_surge": C.MAX_SPEED}):
            try:
                planner = factory(**kwargs)
                source = "DWAPlanner"
                break
            except Exception:
                planner = None
        if planner is None:
            print("  [nav] DWAPlanner unusable; using the reactive planner")
    dt = 1.0 / C.CONTROL_HZ
    done = False
    goal3 = np.array([goal[0], goal[1], 0.0])
    for _ in range(int(seconds * C.CONTROL_HZ)):
        scan = sim.lidar()
        pose = sim.pose.copy()
        out = None
        if planner is not None:
            try:
                out = np.asarray(planner.compute(pose, goal, scan,
                                                 sim.lidar_beam_angles()),
                                 dtype=np.float64).reshape(-1)
            except Exception:
                planner = None
                source = "reactive_lane_following"
        if out is None:
            out = reactive_action(pose, goal3, scan, sim.scene, sim.speed,
                                  sim.time)
        if out.size >= 2:
            throttle, steer = float(out[0]), float(out[1])
        else:
            throttle, steer = float(out[0]), 0.0
        sim.step([throttle, steer], dt)
        if math.hypot(sim.pose[0] - goal[0], sim.pose[1] - goal[1]) <= 3.0:
            done = True
            break
    return {
        "scene": scene_name,
        "source": source,
        "done": bool(done),
        "start_goal_distance_m": round(float(math.hypot(goal[0] - start[0],
                                                        goal[1] - start[1])), 2),
        "final_distance_m": round(float(math.hypot(sim.pose[0] - goal[0],
                                                   sim.pose[1] - goal[1])), 3),
        "collisions": int(sim.collisions),
        "red_light_violations": int(sim.red_light_violations),
        "pedestrians_hit": int(sim.pedestrians_hit),
        "off_road_time": round(float(sim.off_road_time), 2),
        "seconds": round(float(sim.time), 2),
    }


def main() -> int:
    os.makedirs(C.RESULTS_DIR, exist_ok=True)
    t0 = time.time()

    per_scene = {}
    odom_per_scene = {}
    for name in scene_names():
        print("SLAM tour: %s" % name)
        slam = _fly_tour(name, seconds=35.0, use_slam=True)
        odom = _odometry_reference(name, seconds=35.0)
        if slam is None:
            continue
        per_scene[name] = slam
        odom_per_scene[name] = odom
        print("  ATE %.3f m  coverage %.3f  source %s"
              % (slam["ate_rmse_m"], slam["coverage_mean"],
                 slam["slam_source"]))
    if per_scene:
        ate = [v["ate_rmse_m"] for v in per_scene.values()]
        cov = [v["coverage_mean"] for v in per_scene.values()]
        odom_ate = [v["ate_rmse_m"] for v in odom_per_scene.values()]
        first = next(iter(per_scene.values()))
        slam_out = {
            "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "scene": ",".join(per_scene.keys()),
            "seconds": round(float(np.mean([v["seconds"]
                                            for v in per_scene.values()])), 2),
            "ate_rmse_m": round(float(np.mean(ate)), 4),
            "ate_max_m": round(float(np.max([v["ate_max_m"]
                                             for v in per_scene.values()])), 4),
            "final_position_error_m": round(float(np.mean(
                [v["final_position_error_m"] for v in per_scene.values()])), 4),
            "coverage_mean": round(float(np.mean(cov)), 4),
            "coverage_final": round(float(np.mean(
                [v["coverage_final"] for v in per_scene.values()])), 4),
            "coverage_drivable": round(float(np.mean(
                [v.get("coverage_drivable", 0.0)
                 for v in per_scene.values()])), 4),
            "map_cells": first["map_cells"],
            "resolution_m": C.MAP_RESOLUTION,
            "collisions": int(sum(v["collisions"] for v in per_scene.values())),
            "slam_source": first["slam_source"],
            "controller_source": first["controller_source"],
            "scenes": per_scene,
            "odometry_only": {
                "scene": ",".join(odom_per_scene.keys()),
                "ate_rmse_m": round(float(np.mean(odom_ate)), 4),
                "final_position_error_m": round(float(np.mean(
                    [v["final_position_error_m"]
                     for v in odom_per_scene.values()])), 4),
                "scenes": odom_per_scene,
            },
        }
    else:  # pragma: no cover - every scene failed
        slam_out = {"generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "scene": "", "ate_rmse_m": 0.0, "coverage_mean": 0.0,
                    "odometry_only": {}}
    with open(os.path.join(C.RESULTS_DIR, "slam_metrics.json"), "w",
              encoding="utf-8") as fh:
        json.dump(slam_out, fh, indent=2, ensure_ascii=False)
    print("slam_metrics: ATE %.3f m (odom %.3f m), coverage %.3f"
          % (slam_out["ate_rmse_m"],
             slam_out.get("odometry_only", {}).get("ate_rmse_m", 0.0),
             slam_out["coverage_mean"]))

    planning = eval_planning()
    local = eval_local_planner("downtown")
    planning["dwa_downtown"] = local
    planning["dwa_pool"] = local          # legacy alias for the shared docs
    planning["generated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(os.path.join(C.RESULTS_DIR, "planning_metrics.json"), "w",
              encoding="utf-8") as fh:
        json.dump(planning, fh, indent=2, ensure_ascii=False)
    print("planning_metrics: %s"
          % json.dumps({k: (v.get("detour_ratio") if isinstance(v, dict)
                            else v)
                        for k, v in planning["scenes"].items()}))
    print("wall seconds", round(time.time() - t0, 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
