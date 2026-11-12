#!/usr/bin/env python3
"""Closed-loop benchmark: classical arm controller vs neural vs end-to-end.

Runs every controller in each of the three reaching scenes on the real
musculoskeletal arm simulator and records success, time, collisions, reached
targets, final end-effector error, peak muscle torque and effort.  Writes
``results/benchmark.json`` and ``results/benchmark.csv``.

Controllers
-----------
classical       IK + Cartesian A* + joint PD + gravity compensation +
                muscle-activation mapping (the self-contained expert; a
                ``msk_arm_nn.control`` controller is used when importable)
neural          activation MLP: 28-dim features -> 4 muscle activations
neural_ik       IK network (target -> q) + analytic activation mapping
e2e             depth CNN (32x32) -> 4 muscle activations

Usage (from the repo root)::

    python tools/evaluate.py --scenes reach,shelf,draw
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

from msk_arm_nn import config as C
from msk_arm_nn.datasets import (GeometricExpert, arm_features, depth_to_input,
                                 ik_solve, make_expert, solve_activations)
from msk_arm_nn.nn.cnn import CNN
from msk_arm_nn.nn.mlp import MLP
from msk_arm_nn.scenes import get_scene
from msk_arm_nn.sim import MskArmSim

Q1_SCALE = max(abs(C.Q1_MIN), abs(C.Q1_MAX))
Q2_SCALE = max(abs(C.Q2_MIN), abs(C.Q2_MAX))


# ------------------------------------------------------------------ helpers
def clip_activation(u) -> np.ndarray:
    u = np.asarray(u, dtype=np.float64).reshape(-1)
    if u.size < 4:
        u = np.pad(u, (0, 4 - u.size), constant_values=0.0)
    return np.clip(u[:4], 0.0, 1.0)


class ClassicalController:
    """Geometric expert (with an optional nav controller in front)."""

    needs_depth = False

    def __init__(self, scene_name: str):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.impl = None
        self.source = "fallback_geometric_expert"
        self.fallback_used = False
        try:
            from msk_arm_nn import control as nav_control  # type: ignore
            for attr in ("ReachingController", "MissionController"):
                factory = getattr(nav_control, attr, None)
                if factory is not None:
                    try:
                        self.impl = factory(self.scene)
                        self.source = attr
                        break
                    except Exception:
                        continue
        except Exception as exc:
            print("  [classical] nav controller unavailable (%s); using the "
                  "geometric expert" % type(exc).__name__)
        if self.impl is None:
            self.impl = GeometricExpert(scene_name)

    def reset(self, pose=None, goal=None):
        if hasattr(self.impl, "reset"):
            try:
                self.impl.reset(pose)
            except Exception:
                pass

    def update(self, sim, goal, dt, depth=None):
        try:
            return clip_activation(self.impl.update(sim, goal, dt))
        except Exception as exc:
            if not self.fallback_used:
                print("  [classical] %s failed (%s); falling back to the "
                      "geometric expert" % (self.source, type(exc).__name__))
                self.fallback_used = True
                self.source = "fallback_geometric_expert"
            self.impl = GeometricExpert(self.scene_name)
            return clip_activation(self.impl.update(sim, goal, dt))


class NeuralController:
    """Activation MLP: 28-dim features -> 4 muscle activations."""

    needs_depth = True

    def __init__(self, models_dir: str):
        self.net = MLP.load(os.path.join(models_dir, "activation_mlp.npz"))
        self.source = "activation_mlp"
        self.scene = None

    def reset(self, pose=None, goal=None):
        pass

    def update(self, sim, goal, dt, depth=None):
        feats = arm_features(sim, goal, depth)
        return clip_activation(self.net.predict(feats))


class NeuralIKController:
    """IK network (target -> q) followed by the analytic activation mapping."""

    needs_depth = False

    def __init__(self, models_dir: str):
        self.net = MLP.load(os.path.join(models_dir, "ik_mlp.npz"))
        self.source = "ik_mlp+activation_map"
        self.scene = None

    def reset(self, pose=None, goal=None):
        pass

    def update(self, sim, goal, dt, depth=None):
        g = np.asarray(goal, dtype=np.float64)[:2]
        r = float(np.hypot(*g))
        ang = math.atan2(g[1], g[0])
        feats = np.array([g[0] / 0.70, g[1] / 0.70, r / 0.70,
                          math.sin(ang), math.cos(ang)], dtype=np.float64)
        p = self.net.predict(feats)
        q_des = np.array([p[0] * Q1_SCALE, p[1] * Q2_SCALE], dtype=np.float64)
        return clip_activation(solve_activations(sim, q_des))


class E2EController(NeuralController):
    """Depth CNN (32x32 -> 4 activations)."""

    needs_depth = True

    def __init__(self, models_dir: str):
        self.net = CNN.load(os.path.join(models_dir, "e2e_cnn.npz"))
        self.source = "e2e_cnn"
        self.scene = None

    def update(self, sim, goal, dt, depth=None):
        if depth is None:
            depth = sim.depth_camera()
        return clip_activation(self.net.predict(depth_to_input(depth)))


# ------------------------------------------------------------------- episode
def run_episode(scene_name: str, controller, seed: int, max_seconds: float,
                sample_hz: float = 50.0) -> Dict[str, object]:
    sim = MskArmSim(scene_name, seed=seed, noise=True)
    sim.reset()
    goal = sim.goal()
    controller.reset(sim.pose(), goal)
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    last_u = np.zeros(4)
    last_depth = None
    ee_path = 0.0
    last_ee = sim.ee().copy()
    max_steps = max(1, int(round(float(max_seconds) / dt)))
    success = False
    final_error = float(np.hypot(goal[0] - sim.ee()[0],
                                 goal[1] - sim.ee()[1]))
    for k in range(max_steps):
        if k % sample_every == 0:
            last_depth = sim.depth_camera()
            last_u = controller.update(sim, sim.goal(), dt, last_depth)
        sim.step(last_u, dt)
        ee = sim.ee()
        ee_path += float(np.hypot(ee[0] - last_ee[0], ee[1] - last_ee[1]))
        last_ee = ee.copy()
        cur_goal = sim.goal()
        final_error = float(np.hypot(cur_goal[0] - ee[0],
                                     cur_goal[1] - ee[1]))
        if sim.success:
            success = True
            break
    return {
        "scene": scene_name,
        "success": bool(success),
        "time": round(float(sim.time), 3),
        "collisions": int(sim.collisions),
        "reached_targets": int(sim.reached),
        "targets_total": int(len(sim.scene.targets)),
        "final_ee_error": round(final_error, 4),
        "max_torque": round(float(sim.max_torque), 3),
        "effort": round(float(sim.effort), 4),
        "path_length": round(ee_path, 4),
    }


def aggregate(runs: List[Dict[str, object]]) -> Dict[str, object]:
    if not runs:
        return {}
    n = len(runs)
    out = {
        "episodes": n,
        "success_rate": round(float(np.mean([1.0 if r["success"] else 0.0
                                             for r in runs])), 3),
        "mean_time": round(float(np.mean([r["time"] for r in runs])), 3),
        "mean_collisions": round(float(np.mean([r["collisions"]
                                                for r in runs])), 2),
        "mean_reached_targets": round(float(np.mean([r["reached_targets"]
                                                     for r in runs])), 2),
        "mean_final_ee_error": round(float(np.mean([r["final_ee_error"]
                                                    for r in runs])), 4),
        "mean_max_torque": round(float(np.mean([r["max_torque"]
                                                for r in runs])), 3),
        "mean_effort": round(float(np.mean([r["effort"] for r in runs])), 4),
        "mean_path_length": round(float(np.mean([r["path_length"]
                                                 for r in runs])), 4),
    }
    # legacy aliases used by the shared docs tables
    out["mean_final_error"] = out["mean_final_ee_error"]
    out["mean_falls"] = 0.0
    out["mean_final_depth_error"] = 0.0
    out["mean_final_alt_error"] = 0.0
    out["mean_nfz_violations"] = 0.0
    out["mean_red_light_violations"] = 0.0
    out["mean_pedestrians_hit"] = 0.0
    out["mean_off_road_time"] = 0.0
    return out


MODEL_NAMES = ("activation_mlp", "ik_mlp", "planner_mlp", "e2e_cnn",
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
    order = ["classical", "neural", "neural_ik", "e2e"]
    return [c for c in order if c in per_controller]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--max-seconds", type=float, default=20.0)
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument("--seed", type=int, default=C.DEFAULT_SEED)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    parser.add_argument("--out", default=os.path.join(C.RESULTS_DIR,
                                                      "benchmark.json"))
    parser.add_argument("--csv", default=os.path.join(C.RESULTS_DIR,
                                                      "benchmark.csv"))
    parser.add_argument("--skip-e2e", action="store_true")
    args = parser.parse_args()

    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    for name in ("activation_mlp.npz", "ik_mlp.npz"):
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
            "neural_ik": NeuralIKController(args.models_dir),
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
                print("  %-12s ep %d  success=%-5s time=%5.2fs coll=%d "
                      "reached=%d/%d err=%.3fm torque=%.2f effort=%.3f"
                      % (name, ep, result["success"], result["time"],
                         result["collisions"], result["reached_targets"],
                         result["targets_total"], result["final_ee_error"],
                         result["max_torque"], result["effort"]))
            entry = aggregate(scene_runs)
            entry["source"] = getattr(controller, "source", "unknown")
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
        "success_criteria": {"ee_tolerance_m": C.TARGET_TOLERANCE,
                             "hold_time_s": C.HOLD_TIME},
        "features": 28,
        "controllers": per_controller,
        "model_metrics": load_model_metrics(args.models_dir),
        "runs": runs,
        "seconds": round(elapsed, 2),
    }
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
    print("classical controller source(s): %s"
          % ", ".join("%s=%s" % (k, v.get("source"))
                      for k, v in per_controller.get("classical",
                                                     {}).get("scenes",
                                                             {}).items()))

    with open(args.csv, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["scene", "controller", "episode", "success",
                         "time_s", "collisions", "reached_targets",
                         "targets_total", "final_ee_error_m", "max_torque_nm",
                         "effort", "path_length_m", "seed"])
        for r in runs:
            writer.writerow([r["scene"], r["controller"], r["episode"],
                             int(r["success"]), r["time"], r["collisions"],
                             r["reached_targets"], r["targets_total"],
                             r["final_ee_error"], r["max_torque"],
                             r["effort"], r["path_length"], r["seed"]])

    print("\nbenchmark summary (%.1f s):" % elapsed)
    print("  %-12s %-8s %8s %8s %6s %8s %8s %8s"
          % ("controller", "scene", "success", "time", "coll", "reached",
             "err", "effort"))
    for name in controllers_order(per_controller):
        entry = per_controller.get(name, {}).get("scenes", {})
        for scene_name in scenes:
            s = entry.get(scene_name)
            if not s:
                continue
            print("  %-12s %-8s %8.2f %8.2f %6.1f %8.2f %8.4f %8.3f"
                  % (name, scene_name, s["success_rate"], s["mean_time"],
                     s["mean_collisions"], s["mean_reached_targets"],
                     s["mean_final_ee_error"], s["mean_effort"]))
    print("wrote %s and %s" % (args.out, args.csv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
