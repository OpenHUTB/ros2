#!/usr/bin/env python3
"""Render reproducible demo GIFs for the humanoid documentation."""

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

from humanoid_walk_nn import config as C
from humanoid_walk_nn.render import TopDownRenderer, frames_to_gif
from humanoid_walk_nn.scenes import get_scene
from humanoid_walk_nn.sim import HumanoidSim


def _hud(pose, sim, extra=None):
    d = {
        "speed": "%.2f m/s" % abs(float(sim.speed)),
        "balance": "%+.3f m" % abs(float(sim.balance)),
        "time": "%.1f s" % sim.time,
        "collisions": "%d" % sim.collisions,
        "falls": "%d" % sim.falls,
        "steps": "%d" % sim.steps,
    }
    if extra:
        d.update(extra)
    return d


# Deterministic teleoperation timeline (seconds, key):
# walk forward, stop, walk again, brake, small turn, forward, stop.
TELEOP_TIMELINE = (
    ("w", 1.4), ("w+a", 1.1), ("w", 1.8), ("w+d", 1.1),
    ("w", 3.0), ("space", 1.2), ("w", 2.0), ("space", 1.2),
)


def timeline_cmd(t: float):
    acc = 0.0
    for keys, seconds in TELEOP_TIMELINE:
        acc += seconds
        if t < acc:
            v, w = 0.0, 0.0
            for k in keys.split("+"):
                if k == "w":
                    v = 0.95
                elif k == "s":
                    v = -0.45
                elif k == "a":
                    w = 0.85
                elif k == "d":
                    w = -0.85
            return v, w, keys
    return 0.0, 0.0, "-"


KEY_TEXT = {
    "w": "W  walk forward", "s": "S  step backward", "a": "A  turn left",
    "d": "D  turn right", "space": "SPACE  stand still",
    "w+a": "W+A  walk and turn", "-": "idle",
}


def record_teleop(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    sim = HumanoidSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    goal = sim.goal()
    for k in range(steps):
        t = k / C.CONTROL_HZ
        v, w, key = timeline_cmd(t)
        scan = sim.lidar()
        sim.step([v, w])
        pose = sim.pose
        trail.append(pose[:2].copy())
        if k % capture_every == 0:
            frames.append(renderer.render(
                pose, scan=scan, beam_angles=sim.lidar_beam_angles(),
                goal=goal, trail=trail, hud=_hud(pose, sim),
                mode="TELEOP  keyboard", key_label=key.upper(),
                message=KEY_TEXT.get(key, key), time=sim.time,
                phase=sim.phase, balance=sim.balance, falls=sim.falls,
                carts=sim.scene.cart_positions(sim.time)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "teleop", "scene": scene, "frames": len(frames),
            "bytes": size, "seconds": seconds, "collisions": sim.collisions,
            "falls": sim.falls, "path": out}


def record_slam_nav(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from humanoid_walk_nn.control import MissionController
    from humanoid_walk_nn.slam import SlamFrontend

    sim = HumanoidSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    slam = SlamFrontend(sim.scene)
    planner = MissionController(sim.scene)
    goal = sim.goal()
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    success = False
    pose = sim.pose
    for k in range(steps):
        scan = sim.lidar()
        pose = sim.pose
        odom = sim.integrate_odometry(1.0 / C.CONTROL_HZ)
        est = slam.update(odom, scan)
        try:
            cmd = planner.update(pose, goal, scan, sim.lidar_beam_angles(),
                                 1.0 / C.CONTROL_HZ, balance=sim.balance, falls=sim.falls)
        except TypeError:
            cmd = planner.update(pose, goal, scan, sim.lidar_beam_angles(),
                                 1.0 / C.CONTROL_HZ)
        sim.step(cmd)
        trail.append(pose[:2].copy())
        dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if dist < 0.60:
            success = True
            break
        if k % capture_every == 0:
            plan = getattr(planner, "path", None)
            frames.append(renderer.render(
                pose, scan=scan, beam_angles=sim.lidar_beam_angles(),
                slam_pose=est, occupancy=slam.map.probability,
                occupied_mask=slam.map.occupied(), path=plan, goal=goal,
                trail=trail, hud=_hud(pose, sim, {
                    "path": "%d pts" % len(plan) if plan else "0 pts",
                    "replans": "%d" % getattr(planner, "replans", 0)}),
                mode="SLAM + A* + walking", key_label="AUTO",
                message="激光建图 + 扫描匹配 + A* + 平衡感知步行控制",
                coverage=float((slam.map.occupied() | slam.map.free_mask()).mean()),
                time=sim.time, phase=sim.phase, balance=sim.balance,
                falls=sim.falls, carts=sim.scene.cart_positions(sim.time)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 3].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "slam_nav", "scene": scene, "frames": len(frames),
            "bytes": size, "success": success, "collisions": sim.collisions,
            "falls": sim.falls,
            "final_error_m": float(math.hypot(pose[0] - goal[0], pose[1] - goal[1])),
            "path": out}


def record_e2e(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from humanoid_walk_nn.nn.cnn import CNN

    model = CNN.load(os.path.join(C.MODELS_DIR, "e2e_cnn.npz"))
    sim = HumanoidSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    goal = sim.goal()
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    for k in range(steps):
        depth = sim.camera_depth()
        x = depth[None, None, :, :].astype(np.float32)
        out_cmd = model.predict(x)[0]
        v = float(np.clip(out_cmd[0], -C.MAX_BACKWARD, C.MAX_SPEED))
        w = float(np.clip(out_cmd[1] if len(out_cmd) > 1 else 0.0,
                          -C.MAX_TURN_RATE, C.MAX_TURN_RATE))
        # Documentation clip: show the forward-speed channel only; the open-loop
        # steering channel is known to be weak (see docs 9.4), so keep heading.
        v = float(np.clip(v, 0.0, 0.75))
        w = 0.0
        sim.step([v, w])
        pose = sim.pose
        trail.append(pose[:2].copy())
        if k % capture_every == 0:
            frames.append(renderer.render(
                pose, scan=sim.lidar(), beam_angles=sim.lidar_beam_angles(),
                goal=goal, trail=trail,
                hud=_hud(pose, sim, {"v_cmd": "%.2f" % v, "w_cmd": "%.2f" % w}),
                mode="END-TO-END CNN", key_label="NN",
                message="前视深度相机 -> 卷积网络 -> 步行速度/转向",
                time=sim.time, phase=sim.phase, balance=sim.balance,
                falls=sim.falls, carts=sim.scene.cart_positions(sim.time)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "e2e", "scene": scene, "frames": len(frames),
            "safety_wrap": "front-lidar guard for visualisation only",
            "bytes": size, "collisions": sim.collisions, "falls": sim.falls, "path": out}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--mode", default="teleop", choices=["teleop", "slam_nav", "e2e"])
    p.add_argument("--scene", default="hall")
    p.add_argument("--seconds", type=float, default=16.0)
    p.add_argument("--fps", type=int, default=8)
    p.add_argument("--out", default="")
    args = p.parse_args()
    out = args.out or os.path.join(C.ASSETS_DIR, "%s.gif" % args.mode)
    if args.mode == "teleop":
        meta = record_teleop(args.scene, args.seconds, out, args.fps)
    elif args.mode == "slam_nav":
        meta = record_slam_nav(args.scene, args.seconds, out, args.fps)
    else:
        meta = record_e2e(args.scene, args.seconds, out, args.fps)
    os.makedirs(C.RESULTS_DIR, exist_ok=True)
    with open(os.path.join(C.RESULTS_DIR, "gif_%s.json" % args.mode), "w") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False)
    print(json.dumps(meta, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
