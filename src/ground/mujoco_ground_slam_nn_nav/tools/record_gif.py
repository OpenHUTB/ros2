#!/usr/bin/env python3
"""Record reproducible GIF demos (teleop / SLAM navigation / end-to-end)."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from gvnn.config import ASSETS_DIR, MODELS_DIR, RESULTS_DIR
from gvnn.control.pure_pursuit import PurePursuit
from gvnn.control.safety import front_clearance, safety_filter
from gvnn.geometry import downsample_bool
from gvnn.nn.cnn import CNN
from gvnn.planning.astar import GridPlanner
from gvnn.sim.mujoco_backend import MuJoCoSimulation
from gvnn.sim.render import TopDownRenderer
from gvnn.slam.frontend import SlamFrontend


def save_gif(frames, path: str, fps: int = 8, size=(800, 500)) -> int:
    if not frames:
        return 0
    out = []
    for frame in frames:
        img = frame.convert("RGB").resize(size, Image.LANCZOS)
        out.append(img.quantize(colors=128, method=Image.MEDIANCUT))
    out[0].save(path, save_all=True, append_images=out[1:], duration=int(1000 / fps),
                loop=0, optimize=True, disposal=2)
    frames[-1].save(os.path.splitext(path)[0] + ".png")
    return os.path.getsize(path)


def teleop_script(t: float):
    if t < 2.0:
        return 0.60, 0.0, "W"
    if t < 3.4:
        return 0.55, 0.55, "W + A"
    if t < 5.0:
        return 0.55, -0.35, "W + D"
    if t < 6.2:
        return 0.0, 0.0, "SPACE"
    if t < 7.6:
        return -0.35, 0.0, "S"
    if t < 8.8:
        return 0.0, 0.7, "A (pivot)"
    if t < 10.4:
        return 0.55, 0.0, "W"
    return 0.0, 0.0, "SPACE"


def record_teleop(scene: str, seconds: float, fps: int, out: str) -> dict:
    sim = MuJoCoSimulation(scene, seed=20261026, noise=True)
    renderer = TopDownRenderer(sim.scene)
    frames = []
    trail = []
    for k in range(int(seconds * 20)):
        t = k * 0.05
        v, w, key = teleop_script(t)
        scan = sim.scan()
        sim.step(v, w, 0.05)
        trail.append(sim.truth[:2].copy())
        if k % 2 == 0:
            hud = [
                "key: %s" % key,
                "v = %.2f m/s   w = %.2f rad/s" % (v, w),
                "collisions = %d" % sim.collisions,
                "scene = %s" % scene,
            ]
            frames.append(renderer.render(
                truth=sim.truth, slam_pose=sim.truth, path=None,
                scan_points=sim.scan_points(ranges=scan),
                bev=sim.bev_image(), trail=trail[-120:], hud=hud,
                title="GVNN teleop: MuJoCo differential-drive vehicle"))
    size = save_gif(frames, out, fps=fps)
    return {"mode": "teleop", "scene": scene, "frames": len(frames), "bytes": size,
            "seconds": seconds, "collisions": sim.collisions}


def record_slam_nav(scene: str, seconds: float, fps: int, out: str) -> dict:
    sim = MuJoCoSimulation(scene, seed=7, noise=True)
    fe = SlamFrontend(sim.scene.width, sim.scene.height, initial_pose=sim.truth)
    planner = GridPlanner(sim.scene.width, sim.scene.height, 0.10, 0.42, 1.5)
    pp = PurePursuit(max_speed=0.90)
    renderer = TopDownRenderer(sim.scene)
    frames, trail, errs = [], [], []
    path = None
    for k in range(int(seconds * 20)):
        scan = sim.scan()
        fe.update(sim.odom, scan)
        if k % 10 == 0 or path is None:
            planner.set_map(*fe.planning_maps(2))
            p = planner.plan(fe.pose, sim.scene.goal)
            if p is not None:
                path = p
        if path is not None:
            cmd = pp.control(fe.pose, path, front_clearance(scan))
            v, w, _ = safety_filter(scan, cmd["v"], cmd["w"])
        else:
            v = w = 0.0
        sim.step(v, w, 0.05)
        trail.append(sim.truth[:2].copy())
        errs.append(float(np.linalg.norm(fe.pose[:2] - sim.truth[:2])))
        if k % 2 == 0:
            hud = [
                "t = %.1f s" % (k * 0.05),
                "v = %.2f m/s" % v,
                "map coverage = %.0f %%" % (100.0 * fe.coverage),
                "SLAM error = %.2f m" % errs[-1],
                "collisions = %d" % sim.collisions,
                "path length = %.1f m" % planner.path_cost(path) if path else "path length = 0 m",
            ]
            frames.append(renderer.render(
                truth=sim.truth, slam_pose=fe.pose, slam_data=fe.grid.data(),
                path=path, scan_points=sim.scan_points(fe.pose, scan),
                bev=sim.bev_image(), trail=trail[-200:], hud=hud,
                title="GVNN SLAM + A* navigation"))
        if np.linalg.norm(sim.truth[:2] - np.asarray(sim.scene.goal)) < 0.6 and \
                sim.state()["v"] < 0.15:
            break
    size = save_gif(frames, out, fps=fps)
    return {"mode": "slam_nav", "scene": scene, "frames": len(frames), "bytes": size,
            "ate_rmse": float(np.sqrt(np.mean(np.square(errs)))),
            "coverage": float(fe.coverage), "collisions": sim.collisions,
            "seconds": round(len(frames) / fps, 2)}


def record_e2e(scene: str, seconds: float, fps: int, out: str) -> dict:
    model_path = os.path.join(MODELS_DIR, "e2e_cnn.npz")
    if not os.path.exists(model_path):
        raise SystemExit("E2E model missing; run tools/train_e2e.py first")
    model = CNN.load(model_path)
    sim = MuJoCoSimulation(scene, seed=11, noise=True)
    renderer = TopDownRenderer(sim.scene)
    frames, trail = [], []
    cmd = (0.0, 0.0)
    for k in range(int(seconds * 20)):
        if k % 2 == 0:
            bev = sim.bev_image()
            pred = model.predict(bev.astype(np.float64)[None] / 255.0)[0]
            v = float(max(0.0, (pred[0] + 1.0) * 0.5 * 1.1))
            w = float(pred[1] * 1.6)
            scan = sim.scan()
            goal = np.asarray(sim.scene.goal, dtype=np.float64)
            bearing = math.atan2(goal[1] - sim.odom[1], goal[0] - sim.odom[0]) - sim.odom[2]
            bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi
            v = min(v, 0.55)
            w = 0.70 * w + 0.50 * max(-1.2, min(1.2, 1.2 * bearing))
            if abs(bearing) > 0.9:
                v = min(v, 0.25)
            dist_goal = float(np.linalg.norm(sim.truth[:2] - goal))
            if dist_goal < 0.9:
                v = 0.0
            elif dist_goal < 1.8:
                v = min(v, 0.35)
            else:
                v = max(v, 0.28)
            v, w, _ = safety_filter(scan, v, w)
            cmd = (v, w)
        else:
            bev = sim.bev_image()
            scan = sim.scan()
        sim.step(cmd[0], cmd[1], 0.05)
        trail.append(sim.truth[:2].copy())
        if k % 2 == 0:
            hud = [
                "policy: BEV image -> CNN -> (v, w)",
                "v = %.2f m/s   w = %.2f rad/s" % cmd,
                "collisions = %d" % sim.collisions,
                "goal distance = %.1f m" % np.linalg.norm(sim.truth[:2] - np.asarray(sim.scene.goal)),
            ]
            frames.append(renderer.render(
                truth=sim.truth, slam_pose=sim.truth, path=None,
                scan_points=sim.scan_points(ranges=scan),
                bev=bev, trail=trail[-160:], hud=hud,
                title="GVNN end-to-end CNN driving"))
        if np.linalg.norm(sim.truth[:2] - np.asarray(sim.scene.goal)) < 0.8:
            break
    size = save_gif(frames, out, fps=fps)
    return {"mode": "e2e", "scene": scene, "frames": len(frames), "bytes": size,
            "seconds": round(len(frames) / fps, 2), "collisions": sim.collisions}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["teleop", "slam_nav", "e2e"], required=True)
    ap.add_argument("--scene", default="warehouse")
    ap.add_argument("--seconds", type=float, default=16.0)
    ap.add_argument("--fps", type=int, default=8)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    os.makedirs(ASSETS_DIR, exist_ok=True)
    os.makedirs(RESULTS_DIR, exist_ok=True)
    out = args.out or os.path.join(ASSETS_DIR, f"{args.mode}.gif")
    if args.mode == "teleop":
        meta = record_teleop(args.scene, args.seconds, args.fps, out)
    elif args.mode == "slam_nav":
        meta = record_slam_nav(args.scene, args.seconds, args.fps, out)
    else:
        meta = record_e2e(args.scene, args.seconds, args.fps, out)
    meta["path"] = out
    with open(os.path.join(RESULTS_DIR, f"gif_{args.mode}.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(json.dumps(meta, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
