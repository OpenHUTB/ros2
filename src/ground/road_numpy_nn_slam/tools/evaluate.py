#!/usr/bin/env python3
"""Closed-loop benchmark: classical road controller vs neural vs end-to-end.

Runs every controller in each of the three road scenes in the bicycle
simulator and records success (<= 3 m from the goal), time, collisions,
red-light violations, pedestrians hit, off-road time, path length and final
goal error.  Writes ``results/benchmark.json`` and ``results/benchmark.csv``.

Controllers
-----------
classical          ``road_numpy_nn_slam.control.MissionController`` when importable,
                   otherwise the self-contained geometric A* expert
neural             control MLP (lidar sectors + goal + speed + light) ->
                   (throttle, steer) with a forward-cone collision guard
neural_two_level   planner MLP (8 directions) -> waypoint -> control MLP
e2e                camera CNN (48x64x3) -> (throttle, steer)

Usage (from the repo root)::

    python tools/evaluate.py --scenes downtown,campus,construction
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.datasets import (GeometricExpert, _try_mission_controller,
                                  control_features, image_to_input)
from road_numpy_nn_slam.nn.cnn import CNN
from road_numpy_nn_slam.nn.mlp import MLP
from road_numpy_nn_slam.scenes import get_scene
from road_numpy_nn_slam.sim import RoadSim

SUCCESS_RADIUS = 3.0          # m, goal tolerance

_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (math.pi / 4.0)


# ------------------------------------------------------------------ helpers
def clip_cmd(cmd) -> np.ndarray:
    cmd = np.asarray(cmd, dtype=np.float64).reshape(-1)[:2].copy()
    if cmd.size < 2:
        cmd = np.concatenate([cmd, np.zeros(2 - cmd.size)])
    return np.clip(cmd, -1.0, 1.0)


def forward_guard(cmd, scan, beam_angles, yaw: float,
                  scene=None, pose=None) -> np.ndarray:
    """Forward-cone collision guard + curb guard for the learned controllers.

    Only acts when something is really close in the direction of travel:
    throttles down below 6 m and brakes + steers away below 2.2 m.  A second
    rule keeps the car on the carriageway: off-road drag equals the maximum
    acceleration in this simulator, so a vehicle that leaves the road can
    never drive back -- the guard therefore slows to a crawl when the point
    ahead is not drivable.
    """
    cmd = np.asarray(cmd, dtype=np.float64).copy()
    if cmd.size < 2:
        return cmd
    if scene is not None and pose is not None:
        for ahead in (2.5, 4.0):
            ax = float(pose[0]) + ahead * math.cos(yaw)
            ay = float(pose[1]) + ahead * math.sin(yaw)
            if not scene.is_drivable(ax, ay):
                cmd[0] = min(float(cmd[0]), 0.15)
                break
    if scan is None:
        return cmd
    scan = np.asarray(scan, dtype=np.float64)
    ang = np.asarray(beam_angles, dtype=np.float64)
    steer_angle = float(cmd[1]) * C.MAX_STEER
    travel = steer_angle
    delta = (ang - travel + math.pi) % (2.0 * math.pi) - math.pi
    cone = np.abs(delta) <= 0.55
    if not bool(cone.any()):
        return cmd
    front = float(scan[cone].min())
    if front < 6.0:
        cmd[0] *= max(0.0, min(1.0, (front - 1.50) / 4.50))
    if front < 2.20:
        left = scan[ang > 0.35]
        right = scan[ang < -0.35]
        lm = float(left.min()) if left.size else C.LIDAR_RANGE_MAX
        rm = float(right.min()) if right.size else C.LIDAR_RANGE_MAX
        cmd[0] = -1.0
        cmd[1] = 0.9 if lm >= rm else -0.9
    return clip_cmd(cmd)


class AltitudePID:      # kept for API compatibility with the shared tools
    def reset(self, pose=None, goal=None):
        pass

    def command(self, pose, goal) -> float:
        return 0.0


# -------------------------------------------------------------- controllers
class ClassicalController:
    """MissionController when available, otherwise the geometric expert."""

    needs_image = False
    needs_time = True

    def __init__(self, scene_name: str):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.impl = None
        self.source = "fallback_geometric_expert"
        self.fallback_used = False
        try:
            from road_numpy_nn_slam import control as nav_control  # type: ignore
            factory = getattr(nav_control, "MissionController", None)
            if factory is not None:
                self.impl = _try_mission_controller(factory, scene_name)
                self.source = "MissionController"
        except Exception as exc:
            print("  [classical] MissionController unavailable (%s); using "
                  "geometric expert" % type(exc).__name__)
        if self.impl is None:
            self.impl = GeometricExpert(scene_name)

    def reset(self, pose=None, goal=None):
        if hasattr(self.impl, "reset"):
            try:
                self.impl.reset(pose)
            except Exception:
                pass

    def update(self, pose, goal, scan, beam_angles, dt, image=None, t=0.0):
        if hasattr(self.impl, "set_speed"):
            try:
                self.impl.set_speed(float(getattr(self, "_speed", 0.0)))
            except Exception:
                pass
        if hasattr(self.impl, "set_time"):
            try:
                self.impl.set_time(float(t))
            except Exception:
                pass
        # aero-nav's MissionController expects the traffic-light dict and the
        # pedestrian positions of the current instant (same call shape as
        # tests/test_control.py); fall back to the 5-argument form otherwise.
        light = None
        pedestrians = None
        if self.scene is not None:
            try:
                light = self.scene.nearest_light(float(pose[0]), float(pose[1]),
                                                 float(pose[3]), float(t))
                pedestrians = self.scene.pedestrian_positions(float(t))
            except Exception:
                light = pedestrians = None
        try:
            try:
                return clip_cmd(self.impl.update(pose, goal, scan, beam_angles,
                                                 dt, light=light,
                                                 pedestrians=pedestrians))
            except TypeError:
                pass
            try:
                return clip_cmd(self.impl.update(pose, goal, scan, beam_angles,
                                                 dt, t=t))
            except TypeError:
                return clip_cmd(self.impl.update(pose, goal, scan,
                                                 beam_angles, dt))
        except Exception as exc:
            if not self.fallback_used:
                print("  [classical] %s failed (%s); falling back to the "
                      "geometric expert" % (self.source, type(exc).__name__))
                self.fallback_used = True
                self.source = "fallback_geometric_expert"
            self.impl = GeometricExpert(self.scene_name)
            self.impl.reset(np.asarray(pose, dtype=np.float64))
            return clip_cmd(self.impl.update(pose, goal, scan, beam_angles, dt,
                                             t=t))


class NeuralController:
    """Control MLP (lidar sectors + goal + speed + light) -> (throttle, steer)."""

    needs_image = False
    needs_time = False

    def __init__(self, models_dir: str):
        self.net = MLP.load(os.path.join(models_dir, "control_mlp.npz"))
        self.source = "control_mlp"
        self.safety_source = "forward_cone_guard"
        self.scene = None

    def reset(self, pose=None, goal=None):
        pass

    def _policy(self, pose, goal, scan, speed, t):
        feats = control_features(pose, goal, scan, self.scene, speed, t)
        out = self.net.predict(feats)
        return float(out[0]), float(out[1])

    def update(self, pose, goal, scan, beam_angles, dt, image=None, t=0.0):
        pose = np.asarray(pose, dtype=np.float64)
        yaw = float(pose[3])
        speed = float(self._speed) if hasattr(self, "_speed") else 0.0
        if scan is not None:
            throttle, steer = self._policy(pose, goal, scan, speed, t)
        else:
            throttle = steer = 0.0
        cmd = forward_guard(np.array([throttle, steer]), scan, beam_angles, yaw, self.scene, pose)
        return clip_cmd(cmd)


class TwoLevelNeuralController(NeuralController):
    """Two-level neural controller: planner MLP waypoint -> control MLP."""

    def __init__(self, models_dir: str):
        super().__init__(models_dir)
        self.planner = MLP.load(os.path.join(models_dir, "planner_mlp.npz"))
        self.source = "planner_mlp+control_mlp"
        self.waypoint_distance = 8.0

    def _clamp_waypoint(self, wx: float, wy: float):
        scene = self.scene
        if scene is None:
            return wx, wy
        margin = 0.5
        return (max(margin, min(float(scene.width) - margin, wx)),
                max(margin, min(float(scene.height) - margin, wy)))

    def _policy(self, pose, goal, scan, speed, t):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        feats = control_features(pose, goal, scan, self.scene, speed, t)
        action = int(np.argmax(self.planner.predict(feats)))
        rel = float(_ACTION_ANGLES[action % 8])
        dist_goal = float(np.hypot(goal[0] - pose[0], goal[1] - pose[1]))
        d = min(self.waypoint_distance, max(1.0, dist_goal))
        heading = float(pose[3]) + rel
        wx, wy = self._clamp_waypoint(pose[0] + d * math.cos(heading),
                                      pose[1] + d * math.sin(heading))
        waypoint = np.array([wx, wy, 0.0])
        feats2 = control_features(pose, waypoint, scan, self.scene, speed, t)
        out = self.net.predict(feats2)
        return float(out[0]), float(out[1])


class E2EController(NeuralController):
    """Camera CNN (image -> throttle, steer) + the same collision guard."""

    needs_image = True

    def __init__(self, models_dir: str):
        self.net = CNN.load(os.path.join(models_dir, "e2e_cnn.npz"))
        self.source = "e2e_cnn"
        self.safety_source = "forward_cone_guard"
        self.scene = None

    def _policy(self, pose, goal, scan, speed, t):
        raise RuntimeError("e2e policy needs an image")

    def update(self, pose, goal, scan, beam_angles, dt, image=None, t=0.0):
        pose = np.asarray(pose, dtype=np.float64)
        yaw = float(pose[3])
        if image is None:
            throttle = steer = 0.0
        else:
            out = self.net.predict(image_to_input(np.asarray(image)))
            throttle, steer = float(out[0]), float(out[1])
        cmd = forward_guard(np.array([throttle, steer]), scan, beam_angles, yaw, self.scene, pose)
        return clip_cmd(cmd)


# ------------------------------------------------------------------- episode
def run_episode(scene_name: str, controller, seed: int, max_seconds: float,
                sample_hz: float = 10.0) -> Dict[str, object]:
    sim = RoadSim(scene_name, seed=seed, noise=True)
    sim.reset()
    goal = sim.goal()
    pose = sim.pose.copy()
    if controller.scene is None:
        controller.scene = sim.scene
    controller.reset(pose, goal)
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    beam_angles = sim.lidar_beam_angles()
    last_scan = None
    last_cmd = np.zeros(2)
    path_length = 0.0
    last_xy = pose[:2].copy()
    max_steps = max(1, int(round(float(max_seconds) / dt)))
    success = False
    final_error = float(math.hypot(goal[0] - pose[0], goal[1] - pose[1]))
    for k in range(max_steps):
        pose = sim.pose.copy()
        # the learned controllers need the measured speed (part of the input)
        controller._speed = sim.speed
        if k % sample_every == 0:
            last_scan = sim.lidar()
            image = sim.camera() if controller.needs_image else None
            last_cmd = controller.update(pose, goal, last_scan, beam_angles,
                                         dt * sample_every, image=image,
                                         t=sim.time)
        sim.step(last_cmd, dt)
        new_pose = sim.pose
        path_length += float(math.hypot(new_pose[0] - last_xy[0],
                                        new_pose[1] - last_xy[1]))
        last_xy = new_pose[:2].copy()
        final_error = float(math.hypot(goal[0] - new_pose[0],
                                       goal[1] - new_pose[1]))
        if final_error <= SUCCESS_RADIUS:
            success = True
            break
    return {
        "scene": scene_name,
        "success": bool(success),
        "time": round(float(sim.time), 2),
        "collisions": int(sim.collisions),
        "red_light_violations": int(sim.red_light_violations),
        "pedestrians_hit": int(sim.pedestrians_hit),
        "off_road_time": round(float(sim.off_road_time), 2),
        "path_length": round(path_length, 2),
        "final_error": round(final_error, 3),
    }


def aggregate(runs: List[Dict[str, object]]) -> Dict[str, object]:
    if not runs:
        return {}
    n = len(runs)
    return {
        "episodes": n,
        "success_rate": round(float(np.mean([1.0 if r["success"] else 0.0
                                             for r in runs])), 3),
        "mean_time": round(float(np.mean([r["time"] for r in runs])), 2),
        "mean_collisions": round(float(np.mean([r["collisions"]
                                                for r in runs])), 2),
        "mean_red_light_violations": round(float(np.mean(
            [r["red_light_violations"] for r in runs])), 2),
        "mean_pedestrians_hit": round(float(np.mean(
            [r["pedestrians_hit"] for r in runs])), 2),
        "mean_off_road_time": round(float(np.mean([r["off_road_time"]
                                                   for r in runs])), 2),
        "mean_path_length": round(float(np.mean([r["path_length"]
                                                 for r in runs])), 2),
        "mean_final_error": round(float(np.mean([r["final_error"]
                                                 for r in runs])), 3),
        # legacy alias used by the shared docs tables
        "mean_final_depth_error": 0.0,
    }


MODEL_NAMES = ("perception_mlp", "control_mlp", "planner_mlp", "e2e_cnn",
               "rl_dqn", "rl_dqn_3x64", "rl_pg")


def load_model_metrics(models_dir: str) -> Dict[str, object]:
    out = {}
    for name in MODEL_NAMES:
        path = os.path.join(models_dir, name + ".json")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as fh:
                out[name] = json.load(fh)
    return out


def controllers_order(per_controller: Dict[str, Dict[str, object]]) -> List[str]:
    order = ["classical", "neural", "neural_two_level", "e2e"]
    return [c for c in order if c in per_controller]


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
    parser.add_argument("--skip-e2e", action="store_true")
    args = parser.parse_args()

    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    for name in ("control_mlp.npz", "planner_mlp.npz"):
        path = os.path.join(args.models_dir, name)
        if not os.path.exists(path):
            print("ERROR: missing model %s (train it first)" % path)
            return 2

    t_start = time.time()
    runs: List[Dict[str, object]] = []
    per_controller: Dict[str, Dict[str, object]] = {}
    controllers: Dict[str, object] = {}
    for scene_idx, scene_name in enumerate(scenes):
        get_scene(scene_name)
        print("scene %s" % scene_name)
        controllers = {
            "classical": ClassicalController(scene_name),
            "neural": NeuralController(args.models_dir),
            "neural_two_level": TwoLevelNeuralController(args.models_dir),
        }
        if not args.skip_e2e and os.path.exists(
                os.path.join(args.models_dir, "e2e_cnn.npz")):
            controllers["e2e"] = E2EController(args.models_dir)
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
                print("  %-16s ep %d  success=%-5s time=%5.1fs coll=%d red=%d "
                      "peds=%d offroad=%5.1fs path=%5.1fm error=%.2fm"
                      % (name, ep, result["success"], result["time"],
                         result["collisions"],
                         result["red_light_violations"],
                         result["pedestrians_hit"], result["off_road_time"],
                         result["path_length"], result["final_error"]))
            entry = aggregate(scene_runs)
            entry["source"] = getattr(controller, "source", "unknown")
            if name in ("neural", "e2e", "neural_two_level"):
                entry["safety"] = getattr(controller, "safety_source",
                                          "unknown")
            per_controller.setdefault(name, {"scenes": {}}) \
                ["scenes"][scene_name] = entry

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
        "success_criteria": {"horizontal_m": SUCCESS_RADIUS},
        "beams": int(C.LIDAR_BEAMS),
        "controllers": per_controller,
        "model_metrics": load_model_metrics(args.models_dir),
        "runs": runs,
        "seconds": round(elapsed, 2),
    }
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)

    classical = per_controller.get("classical", {}).get("scenes", {})
    for scene_name, entry in classical.items():
        if entry.get("source") != "MissionController":
            print("WARNING: classical/%s used %r, expected MissionController "
                  "(the benchmark fell back to the geometric expert)"
                  % (scene_name, entry.get("source")))
    print("classical controller source(s): %s"
          % ", ".join("%s=%s" % (k, v.get("source"))
                      for k, v in classical.items()))

    with open(args.csv, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["scene", "controller", "episode", "success", "time_s",
                         "collisions", "red_light_violations",
                         "pedestrians_hit", "off_road_time_s",
                         "path_length_m", "final_error_m", "seed"])
        for r in runs:
            writer.writerow([r["scene"], r["controller"], r["episode"],
                             int(r["success"]), r["time"], r["collisions"],
                             r["red_light_violations"],
                             r["pedestrians_hit"], r["off_road_time"],
                             r["path_length"], r["final_error"], r["seed"]])

    print("\nbenchmark summary (%.1f s):" % elapsed)
    print("  %-16s %-13s %8s %8s %6s %5s %5s %8s %8s"
          % ("controller", "scene", "success", "time", "coll", "red",
             "peds", "offroad", "error"))
    for name in controllers_order(per_controller):
        entry = per_controller.get(name, {}).get("scenes", {})
        for scene_name in scenes:
            s = entry.get(scene_name)
            if not s:
                continue
            print("  %-16s %-13s %8.2f %8.2f %6.1f %5.1f %5.1f %8.2f %8.2f"
                  % (name, scene_name, s["success_rate"], s["mean_time"],
                     s["mean_collisions"], s["mean_red_light_violations"],
                     s["mean_pedestrians_hit"], s["mean_off_road_time"],
                     s["mean_final_error"]))
    print("wrote %s and %s" % (args.out, args.csv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
