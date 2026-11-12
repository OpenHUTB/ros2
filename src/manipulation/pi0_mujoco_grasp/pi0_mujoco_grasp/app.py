#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 单臂抓取/放置演示（ROS 2 ament_python 包入口）。（含 π0 推理接口）。

- 机器人：Franka Panda 7-DoF + 二指夹爪（MuJoCo Menagerie 模型）
- 场景：桌面 + 红色方块 + 白色圆盘 + 主相机 / 腕部相机
- 控制：pi0 策略接口（CPU 下用脚本化 IK 策略替身）-> 关节位置伺服
- 输出：可录制 640x480 主视图 + 腕部相机画中画的帧序列，用于生成 GIF

用法:
    python main.py                              # 仅仿真，不渲染
    python main.py --record /tmp/pi0_frames     # 渲染帧序列（约 330 帧）
    python main.py --policy pi0                 # 说明真实 pi0 所需的 GPU 环境

ROS 2 下由 launch/ros_node 调用时，Node 会追加 --ros-args 等参数；
main() 使用 parse_known_args() 忽略它们（见 cli 兼容性测试）。
"""
import argparse
import json
import os

import mujoco
import numpy as np
from PIL import Image

from .pi0_policy_stub import Pi0PolicyStub, CUBE_START, PLATE_XY, TOP_DOWN_R

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_ROOT = os.path.dirname(PACKAGE_DIR)


def _share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("pi0_mujoco_grasp")
    except Exception:
        return MODULE_ROOT


SHARE_DIR = _share_dir()
MODEL_DIR = os.path.join(SHARE_DIR, "models", "franka_emika_panda")
SCENE = os.path.join(MODEL_DIR, "scene_pickplace.xml")

GRIP_OPEN = 255.0
GRIP_CLOSE = 0.0


def build_parser():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", default="", help="directory to write rendered frames")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--policy", default="stub", choices=["stub", "pi0"],
                    help="stub = CPU 脚本策略替身; pi0 = 真正 pi0 推理（需要 GPU）")
    return ap


def _check_model_assets():
    """缺失模型/网格时给出可操作的提示（源码树与安装态都适用）。"""
    if not os.path.isfile(SCENE):
        raise SystemExit(
            "missing model: %s\n"
            "源码树：请先运行 python scripts/setup_model.py 并确认 models/ 目录完整；\n"
            "安装态：请确认 models/ 已随包安装（colcon build 前先下载网格资源）。" % SCENE)
    assets = os.path.join(MODEL_DIR, "assets")
    if not os.path.isdir(assets) or not os.listdir(assets):
        raise SystemExit(
            "missing mesh assets: %s\n"
            "请运行 python scripts/setup_model.py 下载 Panda 网格资源（约 33 MB，"
            "来自 MuJoCo Menagerie，固定 commit）。" % assets)


def rot_to_rotvec(R):
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(angle))
    return angle * axis


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


class PandaPickPlace:
    def __init__(self, scene=SCENE):
        _check_model_assets()
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        self.cube_qadr = self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")]
        self.jnt_range = np.array(self.model.jnt_range[:7])
        self.reset()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.data.ctrl[:] = 0
        self.data.ctrl[:7] = self.data.qpos[:7]
        self.data.ctrl[7] = GRIP_OPEN
        mujoco.mj_forward(self.model, self.data)

    def site_pose(self):
        pos = self.data.site_xpos[self.site_id].copy()
        R = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        return pos, R

    def ik(self, target_pos, target_R=TOP_DOWN_R, q0=None, max_iter=400, tol=1e-4):
        if q0 is None:
            q0 = self.data.qpos[:7].copy()
        q = np.array(q0, dtype=float)
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        err = None
        for _ in range(max_iter):
            self.data.qpos[:7] = q
            mujoco.mj_forward(self.model, self.data)
            pos, R = self.site_pose()
            perr = target_pos - pos
            rerr = rot_to_rotvec(target_R @ R.T)
            err = np.concatenate([perr, rerr])
            if np.linalg.norm(perr) < tol and np.linalg.norm(rerr) < 5 * tol:
                break
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
            J = np.vstack([jacp[:, :7], jacr[:, :7]])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-2 * np.eye(6), err)
            q = np.clip(q + dq, self.jnt_range[:, 0] + 1e-4, self.jnt_range[:, 1] - 1e-4)
        return q, err

    def cube_pos(self):
        return self.data.qpos[self.cube_qadr:self.cube_qadr + 3].copy()

    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)


def run_demo(env, policy, record_dir=None, fps=30):
    waypoints = policy.plan(env)
    q_by_name = {n: q for n, q, _ in waypoints}

    segments = [
        ("home", "pregrasp", 1.8, GRIP_OPEN, GRIP_OPEN),
        ("pregrasp", "grasp", 1.2, GRIP_OPEN, GRIP_OPEN),
        ("grasp", "grasp", 0.8, GRIP_OPEN, GRIP_CLOSE),
        ("grasp", "lift", 1.2, GRIP_CLOSE, GRIP_CLOSE),
        ("lift", "above_plate", 2.2, GRIP_CLOSE, GRIP_CLOSE),
        ("above_plate", "place", 1.2, GRIP_CLOSE, GRIP_CLOSE),
        ("place", "place", 0.8, GRIP_CLOSE, GRIP_OPEN),
        ("place", "retreat", 1.0, GRIP_OPEN, GRIP_OPEN),
        ("retreat", "retreat", 1.0, GRIP_OPEN, GRIP_OPEN),
    ]

    frames = []
    renderer = wrist_renderer = None
    if record_dir:
        os.makedirs(record_dir, exist_ok=True)
        for f in os.listdir(record_dir):
            if f.endswith(".png"):
                os.remove(os.path.join(record_dir, f))
        renderer = mujoco.Renderer(env.model, 480, 640)
        wrist_renderer = mujoco.Renderer(env.model, 240, 320)

    def render_frame():
        if renderer is None:
            return
        renderer.update_scene(env.data, camera="front")
        main = renderer.render()
        wrist_renderer.update_scene(env.data, camera="wrist_cam")
        wrist = wrist_renderer.render()
        frame = main.copy()
        inset = np.array(Image.fromarray(wrist).resize((220, 165)))
        x0 = frame.shape[1] - inset.shape[1] - 10
        y0 = 10
        frame[y0:y0 + inset.shape[0], x0:x0 + inset.shape[1]] = inset
        frame[y0 - 2:y0, x0 - 2:x0 + inset.shape[1] + 2] = 255
        frame[y0:y0 + inset.shape[0], x0 - 2:x0] = 255
        frame[y0:y0 + inset.shape[0], x0 + inset.shape[1]:x0 + inset.shape[1] + 2] = 255
        frame[y0 + inset.shape[0]:y0 + inset.shape[0] + 2, x0 - 2:x0 + inset.shape[1] + 2] = 255
        Image.fromarray(frame).save(os.path.join(record_dir, "f_%05d.png" % len(frames)))
        frames.append(frame)

    sim_t, step_count = 0.0, 0
    render_every = max(1, int(round(1.0 / fps / env.model.opt.timestep)))

    def advance(seconds, q0, q1, g0, g1):
        nonlocal sim_t, step_count
        n = int(seconds / env.model.opt.timestep)
        for i in range(n):
            s = smoothstep((i + 1) / n)
            env.data.ctrl[:7] = q0 * (1 - s) + q1 * s
            env.data.ctrl[7] = g0 * (1 - s) + g1 * s
            env.step(1)
            step_count += 1
            sim_t += env.model.opt.timestep
            if record_dir and step_count % render_every == 0:
                render_frame()
                if frames and len(frames) % 90 == 0:
                    print("  rendered %d frames, t=%.2fs" % (len(frames), sim_t))

    print("cube at", np.round(env.cube_pos(), 4), "tcp at", np.round(env.site_pose()[0], 4))
    render_frame()
    for a, b, dur, g0, g1 in segments:
        print("segment %s -> %s (%.1fs)" % (a, b, dur))
        advance(dur, q_by_name[a], q_by_name[b], g0, g1)

    final_cube = env.cube_pos()
    print("final cube at", np.round(final_cube, 4))
    return final_cube, frames


def main(argv=None):
    args, _ = build_parser().parse_known_args(argv)  # 忽略 ros2 launch 追加的 --ros-args 等参数

    if args.policy == "pi0":
        raise SystemExit(
            "真实 pi0 推理需要 GPU（≥8 GB 显存）与 openpi 权重/服务；\n"
            "请在有 GPU 的机器上启动 pi0 policy server，再把 pi0_policy_stub.py 中的\n"
            "plan()/act() 换成 openpi 客户端调用。本作业环境为无 GPU 的 CPU 虚拟机，\n"
            "因此默认使用 --policy stub 复现同一套观测->动作块->MuJoCo 执行流程。")

    policy = Pi0PolicyStub()
    print("policy:", policy.describe())
    env = PandaPickPlace()
    final_cube, frames = run_demo(env, policy, record_dir=args.record or None, fps=args.fps)
    dist = float(np.linalg.norm(final_cube[:2] - PLATE_XY))
    print("frames:", len(frames))
    print("distance_to_plate_xy: %.4f  success=%s" % (dist, dist < 0.08))
    print("RESULT_JSON=" + json.dumps({
        "cube_start": CUBE_START.tolist(),
        "cube_final": final_cube.tolist(),
        "distance_xy": dist,
        "success": bool(dist < 0.08),
        "frames": len(frames),
        "policy": policy.name,
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
