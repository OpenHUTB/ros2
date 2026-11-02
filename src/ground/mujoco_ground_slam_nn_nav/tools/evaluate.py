#!/usr/bin/env python3
"""Closed-loop benchmark: classical, neural planning/control and end-to-end."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from gvnn.config import MODELS_DIR, RESULTS_DIR
from gvnn.control.pure_pursuit import PurePursuit
from gvnn.control.safety import front_clearance, safety_filter
from gvnn.geometry import line_of_sight
from gvnn.nn.cnn import CNN
from gvnn.nn.mlp import MLP
from gvnn.planning.astar import GridPlanner
from gvnn.planning.neural_planner import NeuralPlanner
from gvnn.sim.mujoco_backend import MuJoCoSimulation
from gvnn.sim.world import SCENES
from gvnn.slam.frontend import SlamFrontend


def run_episode(scene_name: str, mode: str, seed: int, duration: float = 70.0) -> dict:
    sim = MuJoCoSimulation(scene_name, seed=seed, noise=True)
    scene = sim.scene
    goal = np.asarray(scene.goal, dtype=np.float64)
    pp = PurePursuit(max_speed=0.90)
    fe = None
    planner = None
    neural_planner = None
    neural_control = None
    cnn = None
    if mode in ("classical", "neural"):
        fe = SlamFrontend(scene.width, scene.height, initial_pose=sim.truth)
        planner = GridPlanner(scene.width, scene.height, 0.10, 0.42, 1.5)
        if mode == "neural":
            neural_planner = NeuralPlanner(os.path.join(MODELS_DIR, "planner_mlp.npz"))
            neural_control = MLP.load(os.path.join(MODELS_DIR, "control_mlp.npz"))
    elif mode == "e2e":
        cnn = CNN.load(os.path.join(MODELS_DIR, "e2e_cnn.npz"))
    else:
        raise ValueError(mode)

    path = None
    cmd_v = cmd_w = 0.0
    trail = [sim.truth[:2].copy()]
    slam_errs = []
    min_clear = 9.0
    t0 = time.time()
    reached = False
    self_stuck = 0
    self_recover = 0
    for k in range(int(duration * 20)):
        scan = sim.scan()
        fc = front_clearance(scan)
        min_clear = min(min_clear, fc)
        if mode in ("classical", "neural"):
            fe.update(sim.odom, scan)
            if k % 10 == 0 or path is None:
                planner.set_map(fe.grid.occupied_mask(), fe.grid.unknown_mask())
                new_path = None
                if mode == "neural":
                    wp, cls, conf = neural_planner.waypoint(
                        fe.pose, fe.grid.occupied_mask(), fe.grid.unknown_mask(), goal)
                    if wp is not None and conf >= 0.50 and line_of_sight(
                            ~planner.traversable, fe.pose[:2], wp, 0.10, 1):
                        new_path = [(float(fe.pose[0]), float(fe.pose[1])), wp]
                if new_path is None:
                    new_path = planner.plan(fe.pose, goal)
                if new_path is not None:
                    path = new_path
            if path is not None:
                c = pp.control(fe.pose, path, fc)
                if mode == "neural":
                    feat = np.array([[sim.state()["v"] / 1.1, c["cross_track"],
                                      c["heading_error"] / np.pi,
                                      c["curvature"] / 2.0, fc / 9.0]])
                    pred = neural_control.predict(feat)[0]
                    v_nn = float(max(0.0, (pred[0] + 1.0) * 0.5 * 1.1))
                    w_nn = float(pred[1] * 1.6)
                    # Neural controller is supervised by pure pursuit when the
                    # tracking error is large, which prevents oscillation.
                    if abs(c["cross_track"]) < 0.55 and abs(c["heading_error"]) < 0.65:
                        cmd_v, cmd_w = v_nn, w_nn
                    else:
                        cmd_v, cmd_w = c["v"], c["w"]
                    if c["distance"] > 1.5:
                        cmd_v = max(cmd_v, 0.25)
                else:
                    cmd_v, cmd_w = c["v"], c["w"]
            else:
                cmd_v = cmd_w = 0.0
            cmd_v, cmd_w, _ = safety_filter(scan, cmd_v, cmd_w)
            slam_errs.append(float(np.linalg.norm(fe.pose[:2] - sim.truth[:2])))
        else:
            if k % 2 == 0:
                bev = sim.bev_image()
                pred = cnn.predict(bev.astype(np.float64)[None] / 255.0)[0]
                v_nn = float(max(0.0, (pred[0] + 1.0) * 0.5 * 1.1))
                w_nn = float(pred[1] * 1.6)
                odom = sim.odom
                bearing = math.atan2(goal[1] - odom[1], goal[0] - odom[0]) - odom[2]
                bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi
                cmd_v = min(v_nn, 0.55)
                cmd_w = 0.70 * w_nn + 0.50 * max(-1.2, min(1.2, 1.2 * bearing))
                if abs(bearing) > 0.9:
                    cmd_v = min(cmd_v, 0.25)
                dist_goal = float(np.linalg.norm(sim.truth[:2] - goal))
                if dist_goal < 0.9:
                    cmd_v = 0.0
                elif dist_goal < 1.8:
                    cmd_v = min(cmd_v, 0.35)
                else:
                    cmd_v = max(cmd_v, 0.28)
                if abs(sim.state()["v"]) < 0.04 and cmd_v > 0.10:
                    self_stuck += 1
                else:
                    self_stuck = 0
                if self_stuck > 25:
                    self_recover = 24
                    self_stuck = 0
                if self_recover > 0:
                    cmd_v = -0.28
                    cmd_w = 0.9 * (1.0 if bearing >= 0 else -1.0)
                    self_recover -= 1
                cmd_v, cmd_w, _ = safety_filter(scan, cmd_v, cmd_w)

        sim.step(cmd_v, cmd_w, 0.05)
        trail.append(sim.truth[:2].copy())
        dist = float(np.linalg.norm(sim.truth[:2] - goal))
        if dist < 0.85 and sim.state()["v"] < 0.15:
            reached = True
            break
        if sim.collisions >= 3:
            break

    path_len = float(sum(np.linalg.norm(trail[i + 1] - trail[i])
                         for i in range(len(trail) - 1)))
    final = float(np.linalg.norm(sim.truth[:2] - goal))
    return {
        "scene": scene_name,
        "mode": mode,
        "seed": seed,
        "success": bool(reached),
        "collisions": int(sim.collisions),
        "time_s": round(k * 0.05, 2),
        "wall_s": round(time.time() - t0, 2),
        "final_distance_m": round(final, 3),
        "path_length_m": round(path_len, 3),
        "min_front_clearance_m": round(min_clear, 3),
        "slam_ate_rmse_m": round(float(np.sqrt(np.mean(np.square(slam_errs)))), 4)
        if slam_errs else None,
        "map_coverage": round(float(fe.coverage), 4) if fe is not None else None,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--modes", default="classical,neural,e2e")
    ap.add_argument("--scenes", default=",".join(SCENES.keys()))
    ap.add_argument("--seeds", default="1")
    ap.add_argument("--duration", type=float, default=70.0)
    args = ap.parse_args()
    os.makedirs(RESULTS_DIR, exist_ok=True)
    rows = []
    for mode in [m.strip() for m in args.modes.split(",") if m.strip()]:
        for scene in [s.strip() for s in args.scenes.split(",") if s.strip()]:
            for seed in [int(s) for s in args.seeds.split(",") if s.strip()]:
                print(f"running {mode:9s} {scene:15s} seed={seed}", flush=True)
                rows.append(run_episode(scene, mode, seed, args.duration))
                print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
    with open(os.path.join(RESULTS_DIR, "benchmark.json"), "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)
    with open(os.path.join(RESULTS_DIR, "benchmark.csv"), "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print("saved", os.path.join(RESULTS_DIR, "benchmark.json"))


if __name__ == "__main__":
    main()
