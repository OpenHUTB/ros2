#!/usr/bin/env python3
"""Closed-loop benchmark: classical mission controller vs neural vs end-to-end.

Runs every controller in each of the three scenes in the MuJoCo simulator and
records success, time, collisions, path length and final goal error.  Writes
``results/benchmark.json`` and ``results/benchmark.csv``.

Usage (from the repo root)::

    python tools/evaluate.py --scenes pool,harbor,cave
"""

from __future__ import annotations

import argparse
import csv
import inspect
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.datasets import (GeometricExpert, _try_mission_controller,
                                 build_planning_grid, goal_features,
                                 image_to_input, sector_features)
from rov_numpy_nn_slam.nn.cnn import CNN
from rov_numpy_nn_slam.nn.mlp import MLP
from rov_numpy_nn_slam.scenes import get_scene
from rov_numpy_nn_slam.sim import ROVSim


# ------------------------------------------------------------------ helpers
def fallback_safety(cmd, scan, beam_angles):
    """Reactive forward-cone safety filter used when SLAM/control is absent."""
    cmd = np.asarray(cmd, dtype=np.float64).copy()
    if scan is None:
        return cmd
    scan = np.asarray(scan, dtype=np.float64)
    ang = np.asarray(beam_angles, dtype=np.float64)
    front = np.abs(ang) <= 0.50
    if not bool(front.any()):
        return cmd
    front_min = float(scan[front].min())
    if front_min < 1.4:
        cmd[0] *= max(0.0, min(1.0, (front_min - 0.50) / 0.90))
    if front_min < 0.75:
        left = scan[ang > 0.35]
        right = scan[ang < -0.35]
        lm = float(left.min()) if left.size else C.SONAR_RANGE_MAX
        rm = float(right.min()) if right.size else C.SONAR_RANGE_MAX
        cmd[3] = C.MAX_YAW_RATE * 0.9 * (1.0 if lm >= rm else -1.0)
        cmd[0] = 0.0
    return cmd


def make_safety_filter():
    """Return (callable, source); use nav-eng SafetyFilter when available."""
    try:
        from rov_numpy_nn_slam import control as nav_control  # type: ignore
        factory = getattr(nav_control, "SafetyFilter", None)
        if factory is not None:
            filt = factory()
            if hasattr(filt, "apply"):
                def apply(cmd, scan, angles):
                    return np.asarray(filt.apply(cmd, scan, angles),
                                      dtype=np.float64)
                return apply, "SafetyFilter"
    except Exception:
        pass
    return fallback_safety, "fallback_forward_cone"


class ClassicalController:
    """MissionController when available, otherwise the geometric expert."""

    needs_image = False

    def __init__(self, scene_name: str):
        self.scene_name = scene_name
        self.impl = None
        self.source = "fallback_geometric_expert"
        try:
            from rov_numpy_nn_slam import control as nav_control  # type: ignore
            factory = getattr(nav_control, "MissionController", None)
            if factory is not None:
                self.impl = _try_mission_controller(factory, scene_name)
                self.source = "MissionController"
        except Exception as exc:
            print("  [classical] MissionController unavailable (%s); "
                  "using geometric expert" % type(exc).__name__)
        if self.impl is None:
            self.impl = GeometricExpert(scene_name)
        self.fallback_used = False

    def reset(self, pose=None, goal=None):
        if hasattr(self.impl, "reset"):
            try:
                self.impl.reset(pose)
            except Exception:
                pass

    def update(self, pose, goal, scan, beam_angles, dt, image=None):
        try:
            return self.impl.update(pose, goal, scan, beam_angles, dt)
        except Exception as exc:
            if not self.fallback_used:
                print("  [classical] %s failed (%s); falling back to "
                      "geometric expert" % (self.source, type(exc).__name__))
                self.fallback_used = True
                self.source = "fallback_geometric_expert"
            self.impl = GeometricExpert(self.scene_name)
            self.impl.reset(np.asarray(pose, dtype=np.float64))
            return self.impl.update(pose, goal, scan, beam_angles, dt)


class NeuralController:
    """Control MLP (scan sectors + goal polar) + depth PD + safety filter."""

    needs_image = False

    def __init__(self, models_dir: str):
        self.net = MLP.load(os.path.join(models_dir, "control_mlp.npz"))
        self.safety, self.safety_source = make_safety_filter()
        self.source = "control_mlp"
        self.start_depth = 0.0
        self.total_dist = 1.0

    def reset(self, pose=None, goal=None):
        p = np.asarray(pose, dtype=np.float64)
        g = np.asarray(goal, dtype=np.float64)
        self.start_depth = float(p[2])
        self.total_dist = max(1e-6, float(np.hypot(g[0] - p[0], g[1] - p[1])))

    def update(self, pose, goal, scan, beam_angles, dt, image=None):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        if scan is not None:
            x = np.concatenate([
                sector_features(scan[None, :], 16),
                goal_features(pose, goal)[None, :],
            ], axis=1)
            out = self.net.predict(x)[0]
        else:
            out = np.zeros(2)
        vx = float(np.clip(out[0] * C.MAX_SURGE, -0.15, C.MAX_SURGE))
        yaw = float(np.clip(out[1] * C.MAX_YAW_RATE,
                            -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        dist = float(np.hypot(goal[0] - pose[0], goal[1] - pose[1]))
        progress = 1.0 - min(1.0, dist / self.total_dist)
        target_depth = self.start_depth + (float(goal[2]) - self.start_depth) * progress
        heave = float(np.clip(1.2 * (pose[2] - target_depth),
                              -C.MAX_HEAVE, C.MAX_HEAVE))
        return self.safety(np.array([vx, 0.0, heave, yaw]), scan, beam_angles)


class E2EController(NeuralController):
    """Camera CNN (image -> vx, yaw) + same depth PD and safety filter."""

    needs_image = True

    def __init__(self, models_dir: str):
        self.net = CNN.load(os.path.join(models_dir, "e2e_cnn.npz"))
        self.safety, self.safety_source = make_safety_filter()
        self.source = "e2e_cnn"
        self.start_depth = 0.0
        self.total_dist = 1.0

    def update(self, pose, goal, scan, beam_angles, dt, image=None):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        if image is None:
            out = np.zeros(2)
        else:
            out = self.net.predict(image_to_input(image))
        vx = float(np.clip(out[0] * C.MAX_SURGE, -0.15, C.MAX_SURGE))
        yaw = float(np.clip(out[1] * C.MAX_YAW_RATE,
                            -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        dist = float(np.hypot(goal[0] - pose[0], goal[1] - pose[1]))
        progress = 1.0 - min(1.0, dist / self.total_dist)
        target_depth = self.start_depth + (float(goal[2]) - self.start_depth) * progress
        heave = float(np.clip(1.2 * (pose[2] - target_depth),
                              -C.MAX_HEAVE, C.MAX_HEAVE))
        return self.safety(np.array([vx, 0.0, heave, yaw]), scan, beam_angles)


# ------------------------------------------------------------------- episode
_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (math.pi / 4.0)


class TwoLevelNeuralController(NeuralController):
    """Two-level neural controller: planner_mlp waypoint -> control_mlp.

    The planner MLP selects one of 8 local travel directions from the sonar
    sectors + goal polar features (the same inputs it was trained on).  That
    direction is turned into a short-range waypoint which the control MLP then
    tracks with the identical depth-hold and safety layers as ``neural``.
    """

    needs_image = False

    def __init__(self, models_dir: str):
        super().__init__(models_dir)
        self.planner = MLP.load(os.path.join(models_dir, "planner_mlp.npz"))
        self.source = "planner_mlp+control_mlp"
        self.waypoint_distance = 1.5
        self.scene = None

    def _clamp_waypoint(self, wx: float, wy: float):
        scene = self.scene
        if scene is None:
            return wx, wy
        margin = 0.40
        return (max(margin, min(float(scene.width) - margin, wx)),
                max(margin, min(float(scene.height) - margin, wy)))

    def update(self, pose, goal, scan, beam_angles, dt, image=None):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        sectors = None
        if scan is not None:
            sectors = sector_features(scan[None, :], 16)
            feat = np.concatenate(
                [sectors, goal_features(pose, goal)[None, :]], axis=1)
            action = int(np.argmax(self.planner.predict(feat)[0]))
        else:
            action = 0
        rel = float(_ACTION_ANGLES[action % 8])
        dist_goal = float(np.hypot(goal[0] - pose[0], goal[1] - pose[1]))
        d = min(self.waypoint_distance, max(0.40, dist_goal))
        heading = float(pose[3]) + rel
        wx, wy = self._clamp_waypoint(pose[0] + d * math.cos(heading),
                                      pose[1] + d * math.sin(heading))
        waypoint = np.array([wx, wy, float(goal[2])])
        if sectors is None:
            out = np.zeros(2)
        else:
            feat2 = np.concatenate(
                [sectors, goal_features(pose, waypoint)[None, :]], axis=1)
            out = self.net.predict(feat2)[0]
        vx = float(np.clip(out[0] * C.MAX_SURGE, -0.15, C.MAX_SURGE))
        yaw = float(np.clip(out[1] * C.MAX_YAW_RATE,
                            -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        progress = 1.0 - min(1.0, dist_goal / self.total_dist)
        target_depth = self.start_depth + (float(goal[2]) - self.start_depth) * progress
        heave = float(np.clip(1.2 * (pose[2] - target_depth),
                              -C.MAX_HEAVE, C.MAX_HEAVE))
        return self.safety(np.array([vx, 0.0, heave, yaw]), scan, beam_angles)


# ------------------------------------------------------------------- episode
def run_episode(scene_name: str, controller, seed: int, max_seconds: float,
                sample_hz: float = 10.0) -> Dict[str, object]:
    sim = ROVSim(scene_name, seed=seed, noise=True)
    sim.reset()
    goal = sim.goal()
    pose = sim.pose.copy()
    controller.reset(pose, goal)
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    beam_angles = sim.sonar_beam_angles()
    last_scan = None
    last_cmd = np.zeros(4)
    path_length = 0.0
    last_xy = pose[:2].copy()
    max_steps = max(1, int(round(float(max_seconds) / dt)))
    success = False
    final_error = float(math.hypot(goal[0] - pose[0], goal[1] - pose[1]))
    final_depth_error = abs(float(pose[2]) - float(goal[2]))
    for k in range(max_steps):
        pose = sim.pose.copy()
        if k % sample_every == 0:
            last_scan = sim.sonar()
            image = sim.camera() if controller.needs_image else None
            last_cmd = controller.update(pose, goal, last_scan, beam_angles,
                                         dt * sample_every, image=image)
        sim.step(last_cmd, dt)
        new_pose = sim.pose
        path_length += float(math.hypot(new_pose[0] - last_xy[0],
                                        new_pose[1] - last_xy[1]))
        last_xy = new_pose[:2].copy()
        final_error = float(math.hypot(goal[0] - new_pose[0],
                                       goal[1] - new_pose[1]))
        final_depth_error = abs(float(new_pose[2]) - float(goal[2]))
        if final_error < 0.7 and final_depth_error < 0.6:
            success = True
            break
    return {
        "scene": scene_name,
        "success": bool(success),
        "time": round(float(sim.time), 2),
        "collisions": int(sim.collisions),
        "path_length": round(path_length, 2),
        "final_error": round(final_error, 3),
        "final_depth_error": round(final_depth_error, 3),
    }


def aggregate(runs: List[Dict[str, object]]) -> Dict[str, object]:
    if not runs:
        return {}
    return {
        "episodes": len(runs),
        "success_rate": round(float(np.mean([1.0 if r["success"] else 0.0
                                             for r in runs])), 3),
        "mean_time": round(float(np.mean([r["time"] for r in runs])), 2),
        "mean_collisions": round(float(np.mean([r["collisions"] for r in runs])), 2),
        "mean_path_length": round(float(np.mean([r["path_length"] for r in runs])), 2),
        "mean_final_error": round(float(np.mean([r["final_error"] for r in runs])), 3),
        "mean_final_depth_error": round(float(np.mean([r["final_depth_error"] for r in runs])), 3),
    }


def load_model_metrics(models_dir: str) -> Dict[str, object]:
    out = {}
    for name in ("perception_mlp", "control_mlp", "planner_mlp", "e2e_cnn",
                 "rl_dqn", "rl_pg"):
        path = os.path.join(models_dir, name + ".json")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as fh:
                out[name] = json.load(fh)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=2,
                        help="episodes per scene and controller")
    parser.add_argument("--max-seconds", type=float, default=60.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=C.DEFAULT_SEED)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    parser.add_argument("--out", default=os.path.join(C.RESULTS_DIR,
                                                      "benchmark.json"))
    parser.add_argument("--csv", default=os.path.join(C.RESULTS_DIR,
                                                      "benchmark.csv"))
    args = parser.parse_args()

    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    for name in ("control_mlp.npz", "e2e_cnn.npz", "planner_mlp.npz"):
        path = os.path.join(args.models_dir, name)
        if not os.path.exists(path):
            print("ERROR: missing model %s (train it first)" % path)
            return 2

    t_start = time.time()
    runs: List[Dict[str, object]] = []
    per_controller: Dict[str, Dict[str, object]] = {}
    for scene_idx, scene_name in enumerate(scenes):
        get_scene(scene_name)
        print("scene %s" % scene_name)
        controllers = {
            "classical": ClassicalController(scene_name),
            "neural": NeuralController(args.models_dir),
            "neural_two_level": TwoLevelNeuralController(args.models_dir),
            "e2e": E2EController(args.models_dir),
        }
        controllers["neural_two_level"].scene = get_scene(scene_name)
        for ctrl_idx, (name, controller) in enumerate(controllers.items()):
            scene_runs = []
            for ep in range(args.episodes):
                seed = (args.seed + scene_idx * 1000 + ctrl_idx * 101
                        + ep * 17)
                result = run_episode(scene_name, controller, seed,
                                     args.max_seconds, args.sample_hz)
                result.update({"controller": name, "episode": ep,
                               "seed": seed})
                runs.append(result)
                scene_runs.append(result)
                print("  %-9s ep %d  success=%-5s time=%5.1fs collisions=%d "
                      "path=%5.1fm error=%.2fm"
                      % (name, ep, result["success"], result["time"],
                         result["collisions"], result["path_length"],
                         result["final_error"]))
            entry = aggregate(scene_runs)
            entry["source"] = getattr(controller, "source", "unknown")
            if name in ("neural", "e2e"):
                entry["safety"] = getattr(controller, "safety_source", "unknown")
            per_controller.setdefault(name, {"scenes": {}})["scenes"][scene_name] = entry

    elapsed = time.time() - t_start
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
    summary = {
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "scenes": scenes,
        "episodes_per_scene": int(args.episodes),
        "max_seconds": float(args.max_seconds),
        "sample_hz": float(args.sample_hz),
        "seed": int(args.seed),
        "controllers": per_controller,
        "model_metrics": load_model_metrics(args.models_dir),
        "runs": runs,
        "seconds": round(elapsed, 2),
    }
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)

    with open(args.csv, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["scene", "controller", "episode", "success", "time_s",
                         "collisions", "path_length_m", "final_error_m",
                         "final_depth_error_m", "seed"])
        for r in runs:
            writer.writerow([r["scene"], r["controller"], r["episode"],
                             int(r["success"]), r["time"], r["collisions"],
                             r["path_length"], r["final_error"],
                             r["final_depth_error"], r["seed"]])

    print("\nbenchmark summary (%.1f s):" % elapsed)
    print("  %-11s %-8s %8s %8s %8s %8s %8s"
          % ("controller", "scene", "success", "time", "coll", "path", "error"))
    for name in ("classical", "neural", "neural_two_level", "e2e"):
        entry = per_controller.get(name, {}).get("scenes", {})
        for scene_name in scenes:
            s = entry.get(scene_name)
            if not s:
                continue
            print("  %-11s %-8s %8.2f %8.2f %8.2f %8.2f %8.2f"
                  % (name, scene_name, s["success_rate"], s["mean_time"],
                     s["mean_collisions"], s["mean_path_length"],
                     s["mean_final_error"]))
    print("wrote %s and %s" % (args.out, args.csv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
