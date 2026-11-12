#!/usr/bin/env python3
"""Closed-loop benchmark: classical mission controller vs neural vs end-to-end.

Runs every controller in each of the three UAV scenes in the MuJoCo simulator
and records success (<= 3 m horizontal, <= 2 m altitude), time, collisions,
no-fly-zone violations, path length and final goal error.  Writes
``results/benchmark.json`` and ``results/benchmark.csv``.

Controllers
-----------
classical          ``control.MissionController`` when importable, otherwise the
                   self-contained geometric A* expert
neural             control MLP (lidar sectors + goal polar + NFZ) -> body-frame
                   velocity + altitude PID + forward-cone safety filter
neural_two_level   planner MLP (8 directions) -> short waypoint -> control MLP
e2e                camera CNN (48x64x3) -> (forward speed, yaw rate) + alt PID

Usage (from the repo root)::

    python tools/evaluate.py --scenes city,canyon,airport
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

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.datasets import (GeometricExpert, _try_mission_controller,
                                 control_features, image_to_input, scan_angles)
from uav_numpy_nn_slam.nn.cnn import CNN
from uav_numpy_nn_slam.nn.mlp import MLP
from uav_numpy_nn_slam.scenes import get_scene
from uav_numpy_nn_slam.sim import UAVSim

SUCCESS_RADIUS = 3.0          # m, horizontal goal tolerance
SUCCESS_ALT = 2.0             # m, altitude tolerance

_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (math.pi / 4.0)


# ------------------------------------------------------------------ helpers
def body_to_world(vf: float, vl: float, yaw: float) -> tuple:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vf - s * vl, s * vf + c * vl


def clip_cmd(cmd) -> np.ndarray:
    cmd = np.asarray(cmd, dtype=np.float64).reshape(-1)[:4].copy()
    if cmd.size < 4:
        cmd = np.concatenate([cmd, np.zeros(4 - cmd.size)])
    return np.clip(cmd, [-C.MAX_VX, -C.MAX_VY, -C.MAX_VZ, -C.MAX_YAW_RATE],
                   [C.MAX_VX, C.MAX_VY, C.MAX_VZ, C.MAX_YAW_RATE])


def fallback_safety(cmd, scan, beam_angles, yaw: float = 0.0) -> np.ndarray:
    """Reactive forward-cone safety filter for world-frame velocity commands.

    Only prevents imminent collisions: it slows the vehicle when the corridor
    ahead is closing and stops + turns when something is directly in the way.
    """
    cmd = np.asarray(cmd, dtype=np.float64).copy()
    if scan is None or cmd.size < 4:
        return cmd
    scan = np.asarray(scan, dtype=np.float64)
    ang = np.asarray(beam_angles, dtype=np.float64)
    speed = math.hypot(cmd[0], cmd[1])
    if speed < 1e-6:
        return cmd
    travel = math.atan2(cmd[1], cmd[0]) - yaw
    delta = (ang - travel + math.pi) % (2.0 * math.pi) - math.pi
    cone = np.abs(delta) <= 0.45
    if not bool(cone.any()):
        return cmd
    front_min = float(scan[cone].min())
    if front_min < 5.0:
        cmd[0] *= max(0.0, min(1.0, (front_min - 1.20) / 3.80))
        cmd[1] *= max(0.0, min(1.0, (front_min - 1.20) / 3.80))
    if front_min < 2.20:
        left = scan[ang > 0.30]
        right = scan[ang < -0.30]
        lm = float(left.min()) if left.size else C.LIDAR_RANGE_MAX
        rm = float(right.min()) if right.size else C.LIDAR_RANGE_MAX
        turn = 1.0 if lm >= rm else -1.0
        cmd[0] = 0.0
        cmd[1] = 0.0
        cmd[3] = C.MAX_YAW_RATE * 0.9 * turn
    return cmd


def make_safety_filter():
    """Return ``(callable, source)``.

    The benchmark uses its own world-frame forward-cone filter: the ROV-era
    ``control.SafetyFilter`` API takes a *body frame* command, which would
    silently misinterpret the world-frame velocity commands used by the UAV
    simulator, so it is deliberately not used here.  The classical controller
    performs its own avoidance internally.
    """
    return fallback_safety, "fallback_forward_cone"


def escape_layer(cmd, pose, scan, beam_angles, scene, yaw: float) -> np.ndarray:
    """Emergency escape layer for the neural controllers (world-frame cmd).

    Two hard safety rules that the behaviour-cloned policy cannot guarantee:

    * if the corridor in the direction of travel is blocked at < 1.6 m, reverse
      away from it and yaw towards the more open side (this is what unsticks a
      reactive policy in a corner);
    * if the vehicle is inside / very close to a no-fly zone (< 3 m clearance)
      add a repulsive velocity pointing away from the zone centre.

    Both rules only use the current lidar scan and the known zone geometry, so
    they are geometry-independent reactive overrides rather than learned
    behaviour; the benchmark reports them explicitly through ``safety``.
    """
    cmd = np.asarray(cmd, dtype=np.float64).copy()
    if scan is not None and cmd.size >= 4:
        scan = np.asarray(scan, dtype=np.float64)
        ang = np.asarray(beam_angles, dtype=np.float64)
        speed = math.hypot(cmd[0], cmd[1])
        if speed > 1e-6:
            travel = math.atan2(cmd[1], cmd[0]) - yaw
            delta = (ang - travel + math.pi) % (2.0 * math.pi) - math.pi
            cone = np.abs(delta) <= 0.60
            if bool(cone.any()):
                front = float(scan[cone].min())
                if front < 2.20:
                    left = scan[ang > 0.30]
                    right = scan[ang < -0.30]
                    lm = float(left.min()) if left.size else C.LIDAR_RANGE_MAX
                    rm = float(right.min()) if right.size else C.LIDAR_RANGE_MAX
                    turn = 1.0 if lm >= rm else -1.0
                    back = yaw + travel + math.pi
                    v = 2.5
                    cmd[0] = v * math.cos(back)
                    cmd[1] = v * math.sin(back)
                    cmd[3] = C.MAX_YAW_RATE * 0.6 * turn
                    return clip_cmd(cmd)
    if scene is not None and scene.nfz:
        px, py = float(pose[0]), float(pose[1])
        clear = float(scene.nfz_clearance(px, py))
        if clear < 5.0:
            cx, cy, r = min(scene.nfz,
                            key=lambda z: math.hypot(px - z[0], py - z[1]) - z[2])
            dx, dy = px - float(cx), py - float(cy)
            d = max(1e-6, math.hypot(dx, dy))
            push = max(2.0, 5.0 - clear)
            cmd[0] += push * dx / d
            cmd[1] += push * dy / d
    return clip_cmd(cmd)


class AltitudePID:
    """Simple proportional altitude controller with a start->goal ramp."""

    def __init__(self) -> None:
        self.start_alt = 0.0
        self.goal_alt = 0.0
        self.total_dist = 1.0

    def reset(self, pose, goal) -> None:
        p = np.asarray(pose, dtype=np.float64)
        g = np.asarray(goal, dtype=np.float64)
        self.start_alt = float(p[2])
        self.goal_alt = float(g[2])
        self.total_dist = max(1e-6, float(np.hypot(g[0] - p[0], g[1] - p[1])))

    def command(self, pose, goal) -> float:
        p = np.asarray(pose, dtype=np.float64)
        dist = float(np.hypot(goal[0] - p[0], goal[1] - p[1]))
        progress = 1.0 - min(1.0, dist / self.total_dist)
        target = self.start_alt + (self.goal_alt - self.start_alt) * progress
        return float(np.clip(1.5 * (target - p[2]), -C.MAX_VZ, C.MAX_VZ))


# -------------------------------------------------------------- controllers
class ClassicalController:
    """MissionController when available, otherwise the geometric expert."""

    needs_image = False

    def __init__(self, scene_name: str):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.impl = None
        self.source = "fallback_geometric_expert"
        self.fallback_used = False
        try:
            from uav_numpy_nn_slam import control as nav_control  # type: ignore
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

    def update(self, pose, goal, scan, beam_angles, dt, image=None):
        try:
            return clip_cmd(self.impl.update(pose, goal, scan, beam_angles, dt))
        except Exception as exc:
            if not self.fallback_used:
                print("  [classical] %s failed (%s); falling back to the "
                      "geometric expert" % (self.source, type(exc).__name__))
                self.fallback_used = True
                self.source = "fallback_geometric_expert"
            self.impl = GeometricExpert(self.scene_name)
            self.impl.reset(np.asarray(pose, dtype=np.float64))
            return clip_cmd(self.impl.update(pose, goal, scan, beam_angles, dt))


class NeuralController:
    """Control MLP (lidar sectors + goal polar + NFZ) + altitude PID."""

    needs_image = False

    def __init__(self, models_dir: str):
        self.net = MLP.load(os.path.join(models_dir, "control_mlp.npz"))
        self.safety, self.safety_source = make_safety_filter()
        self.source = "control_mlp"
        self.alt = AltitudePID()
        self.scene = None

    def reset(self, pose=None, goal=None):
        self.alt.reset(pose, goal)

    def _policy(self, pose, goal, scan):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        feats = control_features(pose, goal, scan, self.scene)
        out = self.net.predict(feats)
        return float(out[0]), float(out[1]), float(out[2])

    def update(self, pose, goal, scan, beam_angles, dt, image=None):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        yaw = float(pose[3])
        if scan is not None:
            vf_n, vl_n, yaw_n = self._policy(pose, goal, scan)
        else:
            vf_n = vl_n = yaw_n = 0.0
        vf = float(np.clip(vf_n * C.MAX_VX, -C.MAX_VX, C.MAX_VX))
        vl = float(np.clip(vl_n * C.MAX_VY, -C.MAX_VY, C.MAX_VY))
        yaw_rate = float(np.clip(yaw_n * C.MAX_YAW_RATE,
                                 -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        wx, wy = body_to_world(vf, vl, yaw)
        vz = self.alt.command(pose, goal)
        cmd = np.array([wx, wy, vz, yaw_rate], dtype=np.float64)
        cmd = self.safety(cmd, scan, beam_angles, yaw)
        return escape_layer(cmd, pose, scan, beam_angles, self.scene, yaw)


class TwoLevelNeuralController(NeuralController):
    """Two-level neural controller: planner MLP waypoint -> control MLP."""

    needs_image = False

    def __init__(self, models_dir: str):
        super().__init__(models_dir)
        self.planner = MLP.load(os.path.join(models_dir, "planner_mlp.npz"))
        self.source = "planner_mlp+control_mlp"
        self.waypoint_distance = 3.0

    def _clamp_waypoint(self, wx: float, wy: float):
        scene = self.scene
        if scene is None:
            return wx, wy
        margin = 0.6
        return (max(margin, min(float(scene.width) - margin, wx)),
                max(margin, min(float(scene.height) - margin, wy)))

    def _policy(self, pose, goal, scan):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        feats = control_features(pose, goal, scan, self.scene)
        action = int(np.argmax(self.planner.predict(feats)))
        rel = float(_ACTION_ANGLES[action % 8])
        dist_goal = float(np.hypot(goal[0] - pose[0], goal[1] - pose[1]))
        d = min(self.waypoint_distance, max(0.6, dist_goal))
        heading = float(pose[3]) + rel
        wx, wy = self._clamp_waypoint(pose[0] + d * math.cos(heading),
                                      pose[1] + d * math.sin(heading))
        waypoint = np.array([wx, wy, float(goal[2])])
        feats2 = control_features(pose, waypoint, scan, self.scene)
        out = self.net.predict(feats2)
        return float(out[0]), float(out[1]), float(out[2])


class E2EController(NeuralController):
    """Camera CNN (image -> forward speed, yaw rate) + altitude PID."""

    needs_image = True

    def __init__(self, models_dir: str):
        self.net = CNN.load(os.path.join(models_dir, "e2e_cnn.npz"))
        self.safety, self.safety_source = make_safety_filter()
        self.source = "e2e_cnn"
        self.alt = AltitudePID()
        self.scene = None

    def _policy(self, pose, goal, scan):
        raise RuntimeError("e2e policy needs an image")

    def update(self, pose, goal, scan, beam_angles, dt, image=None):
        pose = np.asarray(pose, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        yaw = float(pose[3])
        if image is None:
            vf_n = yaw_n = 0.0
        else:
            out = self.net.predict(image_to_input(np.asarray(image)))
            vf_n, yaw_n = float(out[0]), float(out[1])
        vf = float(np.clip(vf_n * C.MAX_VX, -C.MAX_VX, C.MAX_VX))
        yaw_rate = float(np.clip(yaw_n * C.MAX_YAW_RATE,
                                 -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        wx, wy = body_to_world(vf, 0.0, yaw)
        vz = self.alt.command(pose, goal)
        cmd = np.array([wx, wy, vz, yaw_rate], dtype=np.float64)
        cmd = self.safety(cmd, scan, beam_angles, yaw)
        return escape_layer(cmd, pose, scan, beam_angles, self.scene, yaw)


# ------------------------------------------------------------------- episode
def run_episode(scene_name: str, controller, seed: int, max_seconds: float,
                sample_hz: float = 10.0) -> Dict[str, object]:
    sim = UAVSim(scene_name, seed=seed, noise=True)
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
    last_cmd = np.zeros(4)
    path_length = 0.0
    last_xy = pose[:2].copy()
    min_nfz_clearance = float("inf")
    max_steps = max(1, int(round(float(max_seconds) / dt)))
    success = False
    final_error = float(math.hypot(goal[0] - pose[0], goal[1] - pose[1]))
    final_alt_error = abs(float(pose[2]) - float(goal[2]))
    for k in range(max_steps):
        pose = sim.pose.copy()
        if k % sample_every == 0:
            last_scan = sim.lidar()
            image = sim.camera() if controller.needs_image else None
            last_cmd = controller.update(pose, goal, last_scan, beam_angles,
                                         dt * sample_every, image=image)
        sim.step(last_cmd, dt)
        new_pose = sim.pose
        path_length += float(math.hypot(new_pose[0] - last_xy[0],
                                        new_pose[1] - last_xy[1]))
        last_xy = new_pose[:2].copy()
        clear = sim.nfz_clearance()
        if math.isfinite(clear):
            min_nfz_clearance = min(min_nfz_clearance, float(clear))
        final_error = float(math.hypot(goal[0] - new_pose[0],
                                       goal[1] - new_pose[1]))
        final_alt_error = abs(float(new_pose[2]) - float(goal[2]))
        if final_error <= SUCCESS_RADIUS and final_alt_error <= SUCCESS_ALT:
            success = True
            break
    if not math.isfinite(min_nfz_clearance):
        min_nfz_clearance = -1.0
    return {
        "scene": scene_name,
        "success": bool(success),
        "time": round(float(sim.time), 2),
        "collisions": int(sim.collisions),
        "nfz_violations": int(sim.nfz_violations),
        "min_nfz_clearance": round(float(min_nfz_clearance), 2),
        "path_length": round(path_length, 2),
        "final_error": round(final_error, 3),
        "final_alt_error": round(final_alt_error, 3),
    }


def aggregate(runs: List[Dict[str, object]]) -> Dict[str, object]:
    if not runs:
        return {}
    n = len(runs)
    viol = [1.0 if r["nfz_violations"] > 0 else 0.0 for r in runs]
    return {
        "episodes": n,
        "success_rate": round(float(np.mean([1.0 if r["success"] else 0.0
                                             for r in runs])), 3),
        "mean_time": round(float(np.mean([r["time"] for r in runs])), 2),
        "mean_collisions": round(float(np.mean([r["collisions"]
                                                for r in runs])), 2),
        "mean_nfz_violations": round(float(np.mean([r["nfz_violations"]
                                                    for r in runs])), 2),
        "nfz_violation_rate": round(float(np.mean(viol)), 3),
        "mean_path_length": round(float(np.mean([r["path_length"]
                                                 for r in runs])), 2),
        "mean_final_error": round(float(np.mean([r["final_error"]
                                                 for r in runs])), 3),
        "mean_final_alt_error": round(float(np.mean([r["final_alt_error"]
                                                     for r in runs])), 3),
        # legacy alias used by the ROV-era docs tables
        "mean_final_depth_error": round(float(np.mean(
            [r["final_alt_error"] for r in runs])), 3),
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=2,
                        help="episodes per scene and controller")
    parser.add_argument("--max-seconds", type=float, default=70.0)
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
                print("  %-16s ep %d  success=%-5s time=%5.1fs coll=%d "
                      "nfz=%d path=%5.1fm error=%.2fm alt_err=%.2fm"
                      % (name, ep, result["success"], result["time"],
                         result["collisions"], result["nfz_violations"],
                         result["path_length"], result["final_error"],
                         result["final_alt_error"]))
            entry = aggregate(scene_runs)
            entry["source"] = getattr(controller, "source", "unknown")
            if name in ("neural", "e2e", "neural_two_level"):
                entry["safety"] = getattr(controller, "safety_source", "unknown")
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
        "success_criteria": {"horizontal_m": SUCCESS_RADIUS,
                             "altitude_m": SUCCESS_ALT},
        "beams": int(C.LIDAR_BEAMS),
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
                         "collisions", "nfz_violations", "path_length_m",
                         "final_error_m", "final_alt_error_m",
                         "final_depth_error_m", "seed"])
        for r in runs:
            writer.writerow([r["scene"], r["controller"], r["episode"],
                             int(r["success"]), r["time"], r["collisions"],
                             r["nfz_violations"], r["path_length"],
                             r["final_error"], r["final_alt_error"],
                             r["final_alt_error"], r["seed"]])

    print("\nbenchmark summary (%.1f s):" % elapsed)
    header = ("  %-16s %-8s %8s %8s %6s %6s %8s %8s %8s"
              % ("controller", "scene", "success", "time", "coll", "nfz",
                 "path", "error", "alt_err"))
    print(header)
    for name in controllers_order(per_controller):
        entry = per_controller.get(name, {}).get("scenes", {})
        for scene_name in scenes:
            s = entry.get(scene_name)
            if not s:
                continue
            print("  %-16s %-8s %8.2f %8.2f %6.1f %6.1f %8.2f %8.2f %8.2f"
                  % (name, scene_name, s["success_rate"], s["mean_time"],
                     s["mean_collisions"], s["mean_nfz_violations"],
                     s["mean_path_length"], s["mean_final_error"],
                     s["mean_final_alt_error"]))
    print("wrote %s and %s" % (args.out, args.csv))
    return 0


def controllers_order(per_controller: Dict[str, Dict[str, object]]) -> List[str]:
    order = ["classical", "neural", "neural_two_level", "e2e"]
    return [c for c in order if c in per_controller]


if __name__ == "__main__":
    raise SystemExit(main())
