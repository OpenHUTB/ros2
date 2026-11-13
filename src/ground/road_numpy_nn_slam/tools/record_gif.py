#!/usr/bin/env python3
"""Render reproducible demo GIFs for the road-vehicle documentation.

Modes
-----
teleop   : scripted keyboard teleoperation in the downtown scene
slam_nav : SLAM + A* + Stanley autonomous mission (needs slam/planning/control)
e2e      : end-to-end camera->command CNN closed loop (needs a trained model)

All frames come from the real simulation state.
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

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.render import TopDownRenderer, frames_to_gif
from road_numpy_nn_slam.scenes import get_scene
from road_numpy_nn_slam.sim import RoadSim


def _hud(pose, sim, extra=None):
    speed_kmh = abs(float(sim.speed)) * 3.6
    light = sim.traffic_light()
    d = {
        "speed": "%.1f km/h" % speed_kmh,
        "steer": "%+.0f deg" % math.degrees(float(sim.steering)),
        "time": "%.1f s" % sim.time,
        "collisions": "%d" % sim.collisions,
        "red-light": "%d" % sim.red_light_violations,
        "light": (light["state"].upper() if light else "-"),
    }
    if extra:
        d.update(extra)
    return d


# Keyboard timeline shared with the recorded Ubuntu desktop demo.
# Validated in the downtown scene: no collision, no red-light violation,
# no pedestrian hit, stays on the road (traffic-light stop-and-go demo).
TELEOP_TIMELINE = (
    ("w", 2.6), ("space", 2.0), ("w", 2.2), ("s", 1.6),
    ("space", 0.8), ("w", 1.8), ("space", 1.0),
)


def timeline_cmd(t: float):
    """Return (throttle, steer, key) for the teleoperation demo timeline."""
    acc = 0.0
    for keys, seconds in TELEOP_TIMELINE:
        acc += seconds
        if t < acc:
            throttle, steer = 0.0, 0.0
            for k in keys.split("+"):
                if k == "w":
                    throttle = 1.0
                elif k == "s":
                    throttle = -1.0
                elif k == "a":
                    steer = 1.0
                elif k == "d":
                    steer = -1.0
            return throttle, steer, keys
    return 0.0, 0.0, "-"


KEY_TEXT = {
    "w": "W  throttle", "s": "S  brake / reverse", "a": "A  steer left",
    "d": "D  steer right", "w+a": "W+A  left curve", "w+d": "W+D  right curve",
    "space": "SPACE  brake", "-": "idle",
}


def record_teleop(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    sim = RoadSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    goal = sim.goal()
    for k in range(steps):
        t = k / C.CONTROL_HZ
        throttle, steer, key = timeline_cmd(t)
        scan = sim.lidar()
        sim.step([throttle, steer])
        pose = sim.pose
        trail.append(pose[:2].copy())
        if k % capture_every == 0:
            light = sim.traffic_light()
            frames.append(renderer.render(
                pose, scan=scan, beam_angles=sim.lidar_beam_angles(),
                goal=goal, trail=trail, hud=_hud(pose, sim),
                mode="TELEOP  keyboard", key_label=key.upper(),
                message=KEY_TEXT.get(key, key), time=sim.time, light=light,
                pedestrians=sim.scene.pedestrian_positions(sim.time),
                steering=sim.steering))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "teleop", "scene": scene, "frames": len(frames),
            "bytes": size, "seconds": seconds, "collisions": sim.collisions,
            "red_light_violations": sim.red_light_violations, "path": out}


def record_slam_nav(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from road_numpy_nn_slam.control import MissionController
    from road_numpy_nn_slam.slam import SlamFrontend

    sim = RoadSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    slam = SlamFrontend(sim.scene)
    planner = MissionController(sim.scene)
    goal = sim.goal()
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    plan = None
    success = False
    for k in range(steps):
        scan = sim.lidar()
        pose = sim.pose
        odom = sim.integrate_odometry(1.0 / C.CONTROL_HZ)
        est = slam.update(odom, scan)
        light = sim.traffic_light()
        peds = sim.scene.pedestrian_positions(sim.time)
        # The offline visualisation keeps the SLAM map/estimate on screen but
        # steers with the ground-truth pose: the ROS closed loop (slam_node ->
        # nav_node) already runs the full SLAM-estimate chain and is verified
        # separately in results/ros_*_slam_nav.json.
        ctrl_pose = pose
        try:
            cmd = planner.update(ctrl_pose, goal, scan, sim.lidar_beam_angles(),
                                 1.0 / C.CONTROL_HZ, light=light, pedestrians=peds)
        except TypeError:
            cmd = planner.update(ctrl_pose, goal, scan, sim.lidar_beam_angles(),
                                 1.0 / C.CONTROL_HZ)
        plan = getattr(planner, "path", None)
        sim.step(cmd)
        trail.append(pose[:2].copy())
        dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if dist < 3.0:
            success = True
            break
        if k % capture_every == 0:
            frames.append(renderer.render(
                pose, scan=scan, beam_angles=sim.lidar_beam_angles(),
                slam_pose=est, occupancy=slam.map.probability,
                occupied_mask=slam.map.occupied(), path=plan, goal=goal,
                trail=trail, hud=_hud(pose, sim, {
                    "path": "%d pts" % len(plan) if plan else "0 pts",
                    "replans": "%d" % getattr(planner, "replans", 0)}),
                mode="SLAM + A*", key_label="AUTO",
                message="激光建图 + 扫描匹配定位 + A* 规划 + Stanley 控制",
                coverage=float((slam.map.occupied() | slam.map.free_mask()).mean()),
                time=sim.time, light=light, pedestrians=peds,
                steering=sim.steering))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 3].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "slam_nav", "scene": scene, "frames": len(frames),
            "controller_pose": "ground_truth (SLAM shown for visualisation)",
            "bytes": size, "success": success, "collisions": sim.collisions,
            "red_light_violations": sim.red_light_violations,
            "final_error_m": float(math.hypot(pose[0] - goal[0], pose[1] - goal[1])),
            "path": out}


def record_e2e(scene: str, seconds: float, out: str, fps: int = 8) -> dict:
    from road_numpy_nn_slam.nn.cnn import CNN

    model = CNN.load(os.path.join(C.MODELS_DIR, "e2e_cnn.npz"))
    sim = RoadSim(scene, seed=C.DEFAULT_SEED, noise=True)
    renderer = TopDownRenderer(sim.scene)
    goal = sim.goal()
    frames, trail = [], []
    steps = int(seconds * C.CONTROL_HZ)
    capture_every = max(1, int(round(C.CONTROL_HZ / fps)))
    for k in range(steps):
        img = sim.camera()
        x = img.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        out_cmd = model.predict(x)[0]
        throttle = float(np.clip(out_cmd[0], -1.0, 1.0))
        steer = float(np.clip(out_cmd[1] if len(out_cmd) > 1 else 0.0, -1.0, 1.0))
        sim.step([throttle, steer])
        pose = sim.pose
        trail.append(pose[:2].copy())
        if k % capture_every == 0:
            light = sim.traffic_light()
            frames.append(renderer.render(
                pose, scan=sim.lidar(), beam_angles=sim.lidar_beam_angles(),
                goal=goal, trail=trail,
                hud=_hud(pose, sim, {"throttle": "%.2f" % throttle,
                                     "steer": "%.2f" % steer}),
                mode="END-TO-END CNN", key_label="NN",
                message="前视相机 -> 卷积网络 -> 油门/转向",
                time=sim.time, light=light,
                pedestrians=sim.scene.pedestrian_positions(sim.time),
                steering=sim.steering))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    size = frames_to_gif(frames, out, fps=fps)
    frames[len(frames) // 2].save(os.path.splitext(out)[0] + ".png")
    return {"mode": "e2e", "scene": scene, "frames": len(frames),
            "bytes": size, "path": out}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--mode", default="teleop", choices=["teleop", "slam_nav", "e2e"])
    p.add_argument("--scene", default="downtown")
    p.add_argument("--seconds", type=float, default=24.0)
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
