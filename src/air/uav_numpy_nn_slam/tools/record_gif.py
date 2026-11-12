#!/usr/bin/env python3
"""Render reproducible demo GIFs for the documentation.

Modes
-----
teleop   : scripted keyboard teleoperation in the city scene
slam_nav : SLAM + A* + pure-pursuit autonomous mission (needs slam/planning/control)
e2e      : end-to-end camera->command CNN closed loop (needs a trained model)

The frames are produced from the real simulation state, so the GIFs are
reproducible evidence of the algorithms rather than hand-drawn pictures.
"""

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

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.render import TopDownRenderer, frames_to_gif
from uav_numpy_nn_slam.scenes import get_scene
from uav_numpy_nn_slam.sim import UAVSim


def _hud(pose, sim, extra=None):
    v = sim.velocity()
    d = {
        "alt": "%.2f m" % pose[2],
        "yaw": "%+.0f deg" % math.degrees(pose[3]),
        "speed": "%.2f m/s" % float(np.hypot(v[0], v[1])),
        "time": "%.1f s" % sim.time,
        "collisions": "%d" % sim.collisions,
        "nfz": "%d" % sim.nfz_violations,
    }
    if extra:
        d.update(extra)
    return d


# Keyboard timeline shared with the recorded Ubuntu desktop demo
# (scripts/demo_desktop.sh passes the same string to the ROS teleop node).
TELEOP_TIMELINE = (
    ("w", 1.8), ("a", 0.9), ("w", 1.8), ("a", 0.9),
    ("w", 1.8), ("a", 0.9), ("w", 1.8), ("a", 0.9),
    ("r", 1.2), ("f", 1.2), ("q", 1.0), ("e", 1.0),
    ("a", 0.8), ("w", 1.2), ("d", 0.8), ("s", 1.2),
    ("w", 1.2), ("a", 0.9), ("w", 1.0),
)
_KEY_CMD = {
    "w": (0.85, 0.0, 0.0, 0.0), "s": (-0.50, 0.0, 0.0, 0.0),
    "a": (0.0, 0.0, 0.0, 0.85), "d": (0.0, 0.0, 0.0, -0.85),
    "r": (0.0, 0.0, 0.50, 0.0), "f": (0.0, 0.0, -0.50, 0.0),
    "q": (0.0, 0.40, 0.0, 0.0), "e": (0.0, -0.40, 0.0, 0.0),
}


def timeline_cmd(t: float):
    """Return (vx, vy, vz, wz, key) for the teleoperation demo timeline."""
    acc = 0.0
    for key, seconds in TELEOP_TIMELINE:
        acc += seconds
        if t < acc:
            cmd = _KEY_CMD.get(key, (0.0, 0.0, 0.0, 0.0))
            return cmd[0], cmd[1], cmd[2], cmd[3], key
    return 0.0, 0.0, 0.0, 0.0, "-"


def record_teleop(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    sim = UAVSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    goal = sim.goal()
    key_text = {"w": "W  forward", "r": "R  climb",
                "f": "F  descend", "a": "A  yaw left",
                "d": "D  yaw right", "q": "Q  sway left", "e": "E  sway right", "s": "S  reverse", "-": "idle"}
    for k in range(steps):
        t = k / C.CONTROL_HZ
        vx, vy, vz, wz, key = timeline_cmd(t)
        scan = sim.sonar()
        sim.step([vx, vy, vz, wz])
        pose = sim.pose
        trail.append(pose[:2].copy())
        if k % capture_every == 0:
            hud = _hud(pose, sim)
            frames.append(renderer.render(pose, scan=scan,
                                          beam_angles=sim.sonar_beam_angles(),
                                          goal=goal, trail=trail, hud=hud,
                                          mode="TELEOP  keyboard",
                                          key_label=key.upper() if key != "-" else "-",
                                          message=key_text.get(key, key)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps, size=(800, 672))
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    meta = {"mode": "teleop", "scene": scene, "frames": len(frames),
            "bytes": size, "seconds": seconds, "collisions": sim.collisions,
            "path": out}
    return meta


def record_slam_nav(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from uav_numpy_nn_slam.control import MissionController
    from uav_numpy_nn_slam.slam import SlamFrontend

    sim = UAVSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    slam = SlamFrontend(sim.scene)
    planner = MissionController(sim.scene)
    goal = sim.goal()
    frames, trail, truth_traj, est_traj = [], [], [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    plan = None
    success = False
    for k in range(steps):
        scan = sim.sonar()
        pose = sim.pose
        odom = sim.integrate_odometry(1.0 / C.CONTROL_HZ)
        est = slam.update(odom, scan)
        cmd = planner.update(est, goal, scan, sim.sonar_beam_angles(),
                             1.0 / C.CONTROL_HZ)
        plan = planner.path
        sim.step(cmd)
        trail.append(pose[:2].copy())
        truth_traj.append(pose.copy())
        est_traj.append(np.asarray(est))
        dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if dist < 0.7 and abs(pose[2] - goal[2]) < 0.6:
            success = True
            break
        if k % capture_every == 0:
            hud = _hud(pose, sim, {"path": "%d pts" % len(plan) if plan else "0 pts",
                                   "replans": "%d" % planner.replans})
            frames.append(renderer.render(
                pose, scan=scan, beam_angles=sim.sonar_beam_angles(),
                slam_pose=est, occupancy=slam.map.probability,
                occupied_mask=slam.map.occupied(), path=plan, goal=goal,
                trail=trail, hud=hud, mode="SLAM + A*  (NFZ-aware)", key_label="AUTO",
                coverage=float((slam.map.occupied() | slam.map.free_mask()).mean()),
                message="激光建图 + 扫描匹配定位 + A* 规划 + 纯跟踪控制 (禁飞区规避)"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps, size=(800, 672))
    frames[len(frames) // 3].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "slam_nav", "scene": scene, "frames": len(frames),
            "bytes": size, "success": success, "collisions": sim.collisions,
            "final_error_m": float(math.hypot(pose[0] - goal[0], pose[1] - goal[1])),
            "path": out}


def record_e2e(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from uav_numpy_nn_slam.control import MissionController
    from uav_numpy_nn_slam.nn.cnn import CNN

    model = CNN.load(os.path.join(C.MODELS_DIR, "e2e_cnn.npz"))
    sim = UAVSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    goal = sim.goal()
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    for k in range(steps):
        img = sim.camera()
        x = img.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        out_cmd = model.predict(x)[0]
        vx = float(np.clip(out_cmd[0], -C.MAX_SURGE, C.MAX_SURGE))
        wz = float(np.clip(out_cmd[1], -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        sim.step([vx, 0.0, 0.0, wz])
        pose = sim.pose
        trail.append(pose[:2].copy())
        if k % capture_every == 0:
            hud = _hud(pose, sim, {"网络输出 vx": "%.2f" % vx,
                                   "网络输出 wz": "%.2f" % wz})
            frames.append(renderer.render(pose, scan=sim.sonar(),
                                          beam_angles=sim.sonar_beam_angles(),
                                          goal=goal, trail=trail, hud=hud,
                                          mode="END-TO-END CNN", key_label="NN",
                                          message="相机图像 -> 卷积网络 -> 速度指令"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps, size=(800, 672))
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "e2e", "scene": scene, "frames": len(frames),
            "bytes": size, "path": out}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--mode", default="teleop", choices=["teleop", "slam_nav", "e2e"])
    p.add_argument("--scene", default="city")
    p.add_argument("--seconds", type=float, default=24.0)
    p.add_argument("--fps", type=int, default=8)
    p.add_argument("--out", default="")
    args = p.parse_args()
    out = args.out or os.path.join(
        C.ROOT, "..", "..", "..", "docs", "air", "images",
        "uav_numpy_nn_slam_%s.gif" % args.mode)
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
