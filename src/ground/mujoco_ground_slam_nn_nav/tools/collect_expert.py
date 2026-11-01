#!/usr/bin/env python3
"""Collect a behaviour-cloning dataset from an A* + pure-pursuit expert."""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from gvnn.config import RESULTS_DIR
from gvnn.control.pure_pursuit import PurePursuit
from gvnn.control.safety import front_clearance, safety_filter
from gvnn.datasets import random_goal, sample_free_pose
from gvnn.planning.astar import GridPlanner
from gvnn.sim.mujoco_backend import MuJoCoSimulation
from gvnn.sim.world import SCENES, occupancy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes-per-scene", type=int, default=4)
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--seed", type=int, default=20261026)
    args = ap.parse_args()

    os.makedirs(RESULTS_DIR, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    images, features, commands, poses, goals, scenes, episodes = [], [], [], [], [], [], []
    ep_id = 0
    for scene_idx, (name, scene) in enumerate(SCENES.items()):
        occ = occupancy(scene)
        teacher = GridPlanner(scene.width, scene.height, 0.10, 0.38, 1.0)
        teacher.set_map(occ)
        for ep in range(args.episodes_per_scene):
            sim = MuJoCoSimulation(name, seed=int(rng.integers(1 << 30)), noise=True)
            start = sample_free_pose(scene, occ, rng, clearance=0.75,
                                     avoid=scene.goal, min_avoid_dist=5.0)
            sim.reset(start)
            if ep % 2 == 0:
                goal = scene.goal
            else:
                goal = random_goal(scene, occ, rng, start, min_distance=7.0)
            path = teacher.plan(start, goal)
            if path is None:
                continue
            pp = PurePursuit(max_speed=0.85)
            n = 0
            for k in range(int(args.duration * 20)):
                scan = sim.scan()
                fc = front_clearance(scan)
                cmd = pp.control(sim.truth, path, fc)
                v, w, _ = safety_filter(scan, cmd["v"], cmd["w"])
                if k % 2 == 0:
                    images.append(sim.bev_image())
                    state = sim.state()
                    features.append([
                        float(state["v"]) / 1.1,
                        float(cmd["cross_track"]),
                        float(cmd["heading_error"]) / np.pi,
                        float(cmd["curvature"]) / 2.0,
                        float(fc) / 9.0,
                    ])
                    commands.append([v, w])
                    poses.append(sim.truth.copy())
                    goals.append(goal)
                    scenes.append(scene_idx)
                    episodes.append(ep_id)
                sim.step(v, w, 0.05)
                n += 1
                if np.linalg.norm(sim.truth[:2] - np.asarray(goal)) < 0.6 and sim.state()["v"] < 0.2:
                    break
                if sim.collisions >= 2:
                    break
                if k % 100 == 99:
                    new_path = teacher.plan(sim.truth, goal)
                    if new_path is not None:
                        path = new_path
            print(f"[{name}] episode {ep}: frames={n // 2} collisions={sim.collisions} "
                  f"end_dist={np.linalg.norm(sim.truth[:2] - np.asarray(goal)):.2f}")
            ep_id += 1

    out = os.path.join(RESULTS_DIR, "expert_dataset.npz")
    np.savez_compressed(
        out,
        images=np.asarray(images, dtype=np.uint8),
        features=np.asarray(features, dtype=np.float32),
        commands=np.asarray(commands, dtype=np.float32),
        poses=np.asarray(poses, dtype=np.float32),
        goals=np.asarray(goals, dtype=np.float32),
        scenes=np.asarray(scenes, dtype=np.uint8),
        episodes=np.asarray(episodes, dtype=np.int32),
    )
    meta = {
        "frames": len(images),
        "episodes": ep_id,
        "scenes": list(SCENES.keys()),
        "image_shape": list(np.asarray(images[0]).shape) if images else [],
        "seed": args.seed,
    }
    with open(os.path.join(RESULTS_DIR, "expert_dataset.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print("saved", out, meta)


if __name__ == "__main__":
    main()
