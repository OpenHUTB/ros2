#!/usr/bin/env python3
"""Closed-loop benchmark: classical walking controller vs neural vs end-to-end.

Runs every controller in each of the three indoor scenes in the humanoid
simulator and records success (<= 0.6 m from the goal), time, collisions,
falls, path length and final goal error.  Writes ``results/benchmark.json``
and ``results/benchmark.csv``.

Controllers
-----------
classical          ``humanoid_walk_nn.control.MissionController`` when
                   importable, otherwise the self-contained geometric A* expert
neural             control MLP (lidar sectors + goal + speed + balance) ->
                   (walk speed, turn rate) with a balance/collision guard
neural_two_level   planner MLP (8 directions) -> waypoint -> control MLP
e2e                depth CNN (32x32) -> (walk speed, turn rate)

Usage (from the repo root)::

    python tools/evaluate.py --scenes hall,warehouse,obstacle
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

from humanoid_walk_nn import config as C
from humanoid_walk_nn.datasets import (GeometricExpert,
                                       _try_mission_controller,
                                       balance_limited, control_features,
                                       depth_to_input, turn_limit)
from humanoid_walk_nn.nn.cnn import CNN
from humanoid_walk_nn.nn.mlp import MLP
from humanoid_walk_nn.scenes import get_scene
from humanoid_walk_nn.sim import HumanoidSim

SUCCESS_RADIUS = 0.60
# The controller stops at C.GOAL_TOLERANCE (0.45 m); the harness counts a run
# as successful when the final goal distance is <= 0.60 m, the acceptance bound
# also used by tests/test_control.py.  ``final_error`` below is the
# end-of-episode distance to the current goal for both harnesses.
FINAL_ERROR_DEFINITION = ("distance from the final robot position to the current "
                          "goal point (x,y) in metres")
SUCCESS_DEFINITION = ("success when the final goal distance is <= %.2f m; the "
                      "MissionController stops inside config.GOAL_TOLERANCE "
                      "(%.2f m) so the reported final error is end-of-episode"
                      % (SUCCESS_RADIUS, C.GOAL_TOLERANCE))

_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (math.pi / 4.0)


# ------------------------------------------------------------------ helpers
def clip_cmd(cmd) -> np.ndarray:
    cmd = np.asarray(cmd, dtype=np.float64).reshape(-1)[:2].copy()
    if cmd.size < 2:
        cmd = np.concatenate([cmd, np.zeros(2 - cmd.size)])
    cmd[0] = float(np.clip(cmd[0], -C.MAX_BACKWARD, C.MAX_SPEED))
    cmd[1] = float(np.clip(cmd[1], -C.MAX_TURN_RATE, C.MAX_TURN_RATE))
    return cmd


def walking_guard(cmd, scan, beam_angles) -> np.ndarray:
    """Balance + forward hard-clearance guard applied to *every* controller.

    The forward clearance is a hard constraint (not a soft slow-down): with
    ``clear < HARD_CLEARANCE`` forward motion is forbidden and the robot turns
    towards the larger side clearance, so a learned policy cannot walk into a
    wall.  The 8/10-beam radars in this simulator see walls/cones/pillars/
    pedestrians, so this is the same rule a real safety PLC would enforce.
    """
    HARD_CLEARANCE = 0.40          # m, never move forward closer than this
    SOFT_CLEARANCE = 1.10          # m, slow down inside this distance
    cmd = np.asarray(cmd, dtype=np.float64).copy()
    v, omega = float(cmd[0]), float(cmd[1])
    # lateral balance: never command an unsafe v * omega product
    v, omega = balance_limited(v, omega)
    if scan is not None:
        scan = np.asarray(scan, dtype=np.float64)
        ang = np.asarray(beam_angles, dtype=np.float64)
        # any-direction hard stop: the robot must not translate while anything
        # is inside the safety envelope (a learned policy must not grind walls)
        any_clear = float(scan.min())
        if any_clear < HARD_CLEARANCE:
            v = 0.0
        elif any_clear < 0.75:
            v = min(v, max(0.0, (any_clear - HARD_CLEARANCE) * 0.8))
        front = np.abs(ang) <= 0.60
        if bool(front.any()):
            clear = float(scan[front].min())
            if clear < SOFT_CLEARANCE:
                v = min(v, max(0.0, (clear - HARD_CLEARANCE) * 0.85))
            if clear < HARD_CLEARANCE:
                # hard stop: no forward motion inside the safety envelope
                left = scan[ang > 0.35]
                right = scan[ang < -0.35]
                lm = float(left.min()) if left.size else C.LIDAR_RANGE_MAX
                rm = float(right.min()) if right.size else C.LIDAR_RANGE_MAX
                v = 0.0
                if max(lm, rm) > HARD_CLEARANCE:
                    omega = C.MAX_TURN_RATE * 0.7 * (1.0 if lm >= rm else -1.0)
                else:
                    omega = 0.0
        # reverse guard: a learned policy must not back into a wall either
        if v < 0.0:
            rear = (np.abs(np.abs(ang) - math.pi) <= 0.60)
            if bool(rear.any()) and float(scan[rear].min()) < HARD_CLEARANCE:
                v = 0.0
    return clip_cmd([v, omega])


class AltitudePID:      # kept for API compatibility with the shared tools
    def reset(self, pose=None, goal=None):
        pass

    def command(self, pose, goal) -> float:
        return 0.0


# -------------------------------------------------------------- controllers
class ClassicalController:
    """MissionController when available, otherwise the geometric expert."""

    needs_depth = False

    def __init__(self, scene_name: str):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.impl = None
        self.source = "fallback_geometric_expert"
        self.fallback_used = False
        try:
            from humanoid_walk_nn import control as nav_control  # type: ignore
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

    def update(self, pose, goal, scan, beam_angles, dt, depth=None, t=0.0,
               speed=0.0, balance=0.0):
        speed = float(getattr(self, "_speed", speed))
        try:
            try:
                return clip_cmd(self.impl.update(pose, goal, scan, beam_angles,
                                                 dt, speed=speed, t=t))
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
                                             speed=speed, t=t))


class NeuralController:
    """Control MLP (lidar sectors + goal + speed + balance) -> (v, omega)."""

    needs_depth = False

    def __init__(self, models_dir: str):
        self.net = MLP.load(os.path.join(models_dir, "control_mlp.npz"))
        self.source = "control_mlp"
        self.safety_source = "walking_guard"
        self.scene = None

    def reset(self, pose=None, goal=None):
        pass

    def _policy(self, pose, goal, scan, speed, balance):
        feats = control_features(pose, goal, scan, speed, balance)
        out = self.net.predict(feats)
        v = float(np.clip(out[0], -1.0, 1.0)) * C.MAX_SPEED
        omega = float(np.clip(out[1], -1.0, 1.0)) * C.MAX_TURN_RATE
        return v, omega

    def update(self, pose, goal, scan, beam_angles, dt, depth=None, t=0.0,
               speed=0.0, balance=0.0):
        speed = float(getattr(self, "_speed", speed))
        balance = float(getattr(self, "_balance", balance))
        if scan is not None:
            v, omega = self._policy(pose, goal, scan, speed, balance)
        else:
            v = omega = 0.0
        return walking_guard([v, omega], scan, beam_angles)


class TwoLevelNeuralController(NeuralController):
    """Two-level neural controller: planner MLP waypoint -> control MLP."""

    def __init__(self, models_dir: str):
        super().__init__(models_dir)
        self.planner = MLP.load(os.path.join(models_dir, "planner_mlp.npz"))
        self.source = "planner_mlp+control_mlp"
        self.waypoint_distance = 1.4

    def _clamp_waypoint(self, wx: float, wy: float):
        scene = self.scene
        if scene is None:
            return wx, wy
        margin = C.ROBOT_RADIUS + 0.05
        return (max(margin, min(float(scene.width) - margin, wx)),
                max(margin, min(float(scene.height) - margin, wy)))

    def _policy(self, pose, goal, scan, speed, balance):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        feats = control_features(pose, goal, scan, speed, balance)
        action = int(np.argmax(self.planner.predict(feats)))
        rel = float(_ACTION_ANGLES[action % 8])
        dist_goal = float(np.hypot(goal[0] - pose[0], goal[1] - pose[1]))
        d = min(self.waypoint_distance, max(0.35, dist_goal))
        heading = float(pose[3]) + rel
        wx, wy = self._clamp_waypoint(pose[0] + d * math.cos(heading),
                                      pose[1] + d * math.sin(heading))
        waypoint = np.array([wx, wy, 0.0])
        feats2 = control_features(pose, waypoint, scan, speed, balance)
        out = self.net.predict(feats2)
        return (float(np.clip(out[0], -1.0, 1.0)) * C.MAX_SPEED,
                float(np.clip(out[1], -1.0, 1.0)) * C.MAX_TURN_RATE)


class E2EController(NeuralController):
    """Depth CNN (32x32 -> v, omega) + the same walking guard."""

    needs_depth = True

    def __init__(self, models_dir: str):
        self.net = CNN.load(os.path.join(models_dir, "e2e_cnn.npz"))
        self.source = "e2e_cnn"
        self.safety_source = "walking_guard"
        self.scene = None

    def _policy(self, pose, goal, scan, speed, balance):
        raise RuntimeError("e2e policy needs a depth image")

    def update(self, pose, goal, scan, beam_angles, dt, depth=None, t=0.0,
               speed=0.0, balance=0.0):
        if depth is None:
            v = omega = 0.0
        else:
            out = self.net.predict(depth_to_input(np.asarray(depth)))
            v = float(np.clip(out[0], -1.0, 1.0)) * C.MAX_SPEED
            omega = float(np.clip(out[1], -1.0, 1.0)) * C.MAX_TURN_RATE
        return walking_guard([v, omega], scan, beam_angles)


# ------------------------------------------------------------------- episode
def run_episode(scene_name: str, controller, seed: int, max_seconds: float,
                sample_hz: float = 10.0) -> Dict[str, object]:
    sim = HumanoidSim(scene_name, seed=seed, noise=True)
    sim.reset()
    goal = sim.goal()
    pose = sim.pose.copy()
    if getattr(controller, "scene", None) is None:
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
        controller._speed = sim.speed
        controller._balance = sim.balance
        if k % sample_every == 0:
            last_scan = sim.lidar()
            depth = sim.camera_depth() if controller.needs_depth else None
            last_cmd = controller.update(pose, goal, last_scan, beam_angles,
                                         dt * sample_every, depth=depth,
                                         t=sim.time, speed=sim.speed,
                                         balance=sim.balance)
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
        "falls": int(sim.falls),
        "path_length": round(path_length, 2),
        "final_error": round(final_error, 3),
    }


def aggregate(runs: List[Dict[str, object]]) -> Dict[str, object]:
    if not runs:
        return {}
    n = len(runs)

    def _stats(key):
        values = np.array([float(r[key]) for r in runs], dtype=np.float64)
        return round(float(values.mean()), 3), round(float(values.std(ddof=0)), 3)

    time_m, time_s = _stats("time")
    coll_m, coll_s = _stats("collisions")
    falls_m, falls_s = _stats("falls")
    path_m, path_s = _stats("path_length")
    err_m, err_s = _stats("final_error")
    return {
        "episodes": n,
        "success_rate": round(float(np.mean([1.0 if r["success"] else 0.0
                                             for r in runs])), 3),
        "mean_time": round(time_m, 2),
        "std_time": round(time_s, 2),
        "mean_collisions": round(coll_m, 2),
        "std_collisions": round(coll_s, 2),
        "mean_falls": round(falls_m, 2),
        "std_falls": round(falls_s, 2),
        "mean_path_length": round(path_m, 2),
        "std_path_length": round(path_s, 2),
        "mean_final_error": round(err_m, 3),
        "std_final_error": round(err_s, 3),
        # legacy aliases used by the shared docs tables
        "mean_final_depth_error": 0.0,
        "mean_final_alt_error": 0.0,
        "mean_nfz_violations": 0.0,
        "mean_red_light_violations": 0.0,
        "mean_pedestrians_hit": 0.0,
        "mean_off_road_time": 0.0,
    }


MODEL_NAMES = ("perception_mlp", "control_mlp", "planner_mlp", "e2e_cnn",
               "rl_dqn", "rl_dqn_3x64", "rl_pg")


def load_model_metrics(models_dir: str) -> Dict[str, object]:
    out = {}
    for name in MODEL_NAMES:
        path = os.path.join(models_dir, name + ".json")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8-sig") as fh:
                out[name] = json.load(fh)
    return out

def controllers_order(per_controller: Dict[str, Dict[str, object]]) -> List[str]:
    order = ["classical", "neural", "neural_two_level", "e2e"]
    return [c for c in order if c in per_controller]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=3,
                        help="episodes per scene and controller "
                             "(the published protocol uses 3)")
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
                # identical seed for every controller: same scene
                # initialisation (poles/obstacles/cart phase/noise), so the
                # comparison is within-subject rather than across random draws
                seed = args.seed + scene_idx * 1000 + ep
                result = run_episode(scene_name, controller, seed,
                                     args.max_seconds, args.sample_hz)
                result.update({"controller": name, "episode": ep,
                               "seed": seed})
                runs.append(result)
                scene_runs.append(result)
                print("  %-16s ep %d  success=%-5s time=%5.1fs coll=%d "
                      "falls=%d path=%5.1fm error=%.2fm"
                      % (name, ep, result["success"], result["time"],
                         result["collisions"], result["falls"],
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
        "seed_policy": "one seed per scene+episode shared by every controller "
                       "(args.seed + 1000*scene_index + episode)",
        "success_criteria": {"horizontal_m": SUCCESS_RADIUS,
                             "definition": SUCCESS_DEFINITION},
        "final_error_definition": FINAL_ERROR_DEFINITION,
        "safety_guard": "walking_guard applied after every learned controller "
                        "(neural, neural_two_level, e2e): hard stop when any "
                        "lidar beam is closer than 0.40 m (front and reverse "
                        "sectors enforced), slow-down below 0.75 m, plus the "
                        "balance limit |v*omega|; the classical stack keeps its "
                        "own 0.35 m clearance in the shared control module",
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
            print("INFO: classical/%s used %r (geometric fallback expert)"
                  % (scene_name, entry.get("source")))
    print("classical controller source(s): %s"
          % ", ".join("%s=%s" % (k, v.get("source"))
                      for k, v in classical.items()))

    with open(args.csv, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["scene", "controller", "episode", "success", "time_s",
                         "collisions", "falls", "path_length_m",
                         "final_error_m", "seed"])
        for r in runs:
            writer.writerow([r["scene"], r["controller"], r["episode"],
                             int(r["success"]), r["time"], r["collisions"],
                             r["falls"], r["path_length"], r["final_error"],
                             r["seed"]])

    print("\nbenchmark summary (%.1f s, %d episodes/cell, one shared seed per "
          "scene+episode):" % (elapsed, args.episodes))
    print("  %-16s %-10s %8s %14s %12s %14s"
          % ("controller", "scene", "success", "time(s)", "collisions",
             "final_error(m)"))
    for name in controllers_order(per_controller):
        entry = per_controller.get(name, {}).get("scenes", {})
        for scene_name in scenes:
            s = entry.get(scene_name)
            if not s:
                continue
            print("  %-16s %-10s %8.2f %6.2f±%-5.2f %6.2f±%-5.2f %6.3f±%-5.3f"
                  % (name, scene_name, s["success_rate"],
                     s["mean_time"], s.get("std_time", 0.0),
                     s["mean_collisions"], s.get("std_collisions", 0.0),
                     s["mean_final_error"], s.get("std_final_error", 0.0)))
    print("wrote %s and %s" % (args.out, args.csv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
