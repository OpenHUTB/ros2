#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形上下台阶步态仿真 —— 模块入口。

用法：
    python3 main.py                        # 打开 MuJoCo 窗口，实时演示
    python3 main.py --headless             # 无窗口，只打印指标并写 CSV
    python3 main.py --headless --save-dir results
    python3 main.py --test                 # 跑内置自检
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import humanoid_stair_climb.config as config
from humanoid_stair_climb import __version__
from humanoid_stair_climb.gait import StairGait
from humanoid_stair_climb.metrics import RunRecorder, format_summary
from humanoid_stair_climb.sim_core import StairSim


def positive_float(text):
    """argparse 类型：必须为正数，否则直接退出（避免 --control-hz 0 除零）。"""
    try:
        val = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError("需要数字，收到 %r" % text)
    if not math.isfinite(val) or val <= 0:
        raise argparse.ArgumentTypeError("必须为正数，收到 %r" % text)
    return val


def build_parser():
    p = argparse.ArgumentParser(description="人形上下台阶步态仿真")
    p.add_argument("--headless", action="store_true", help="不打开 MuJoCo 窗口")
    p.add_argument("--control-hz", type=positive_float, default=config.CONTROL_HZ,
                   help="控制频率（默认 %(default)s Hz）")
    p.add_argument("--time-limit", type=positive_float, default=config.TIME_LIMIT,
                   help="仿真时长上限，秒（默认 %(default)s）")
    p.add_argument("--hold", type=float, default=config.HOLD_TIME,
                   help="到达平台后保持的时间，秒（默认 %(default)s）")
    p.add_argument("--save-dir", default=None, help="CSV 输出目录")
    p.add_argument("--pace", action="store_true", help="按真实时间节流（录屏用）")
    p.add_argument("--pace-scale", type=positive_float, default=1.0,
                   help="--pace 的时间倍率，>1 更慢（默认 %(default)s）")
    p.add_argument("--record", default=None, metavar="DIR",
                   help="离屏渲染每一帧到 DIR（录 GIF 用）")
    p.add_argument("--record-every", type=positive_float, default=0.10,
                   help="每隔多少秒仿真时间存一帧（默认 %(default)s）")
    p.add_argument("--test", action="store_true", help="跑内置自检后退出")
    p.add_argument("--version", action="version",
                   version="humanoid_stair_climb %s" % __version__)
    return p


def _make_recorder(args, sim):
    """--record 时返回一个 (sim, t) -> None 的存帧回调，否则返回 None。"""
    if not args.record:
        return None
    import numpy as np
    from PIL import Image
    import mujoco

    os.makedirs(args.record, exist_ok=True)
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [1.05, 0.0, 0.34]
    cam.distance = 6.4
    cam.azimuth = 90.0
    cam.elevation = -24.0
    state = {"n": 0}

    def shoot(s, t):
        renderer.update_scene(s.data, camera=cam)
        Image.fromarray(renderer.render()).save(
            os.path.join(args.record, "f%04d.png" % state["n"]))
        state["n"] += 1

    return shoot


def run_one(args, viewer=None, sim=None):
    """跑一次仿真，返回 (recorder, summary)。

    sim 可以复用（有窗口时窗口和仿真必须共享同一份 MjData）。
    """
    gait = StairGait()
    sim = sim or StairSim()
    rec = RunRecorder()

    dt = 1.0 / args.control_hz
    base_x = gait.base_x(0.0)
    base_z = config.STAND_Z
    t = 0.0
    reached_at = None

    # t=0 先记录初始状态，保证 CSV 第一行就是初始位姿
    joints = gait.joint_targets(t, base_x, base_z)
    sim.set_state(base_x, base_z, joints)
    rec.add(t, sim.base_pos(), joints, sim.clearance(), gait.support_height(t))
    if viewer is not None:
        viewer.sync()

    recorder = _make_recorder(args, sim)
    next_shot = 0.0
    while t < args.time_limit:
        t += dt
        base_x = gait.base_x(t)

        # 基座高度跟随支撑脚（一阶低通，避免落脚瞬间跳变）
        target_z = config.STAND_Z + gait.support_height(t)
        alpha = min(1.0, dt / config.BASE_Z_TAU)
        base_z += (target_z - base_z) * alpha

        joints = gait.joint_targets(t, base_x, base_z)
        sim.set_state(base_x, base_z, joints)

        states = {s: gait.foot_state(t, s) for s in ("l", "r")}
        rec.detect_landings(t, states, sim)
        rec.add(t, sim.base_pos(), joints, sim.clearance(), gait.support_height(t))

        if viewer is not None:
            viewer.sync()
        if recorder is not None and t >= next_shot:
            recorder(sim, t)
            next_shot += args.record_every
        if args.pace and not args.headless:
            time.sleep(dt * args.pace_scale)

        if reached_at is None and sim.reached():
            reached_at = t
        if reached_at is not None and t - reached_at >= args.hold:
            break

    return rec, rec.summary()


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.test:
        import subprocess
        here = os.path.dirname(os.path.realpath(__file__))
        return subprocess.call([sys.executable, os.path.join(here, "test", "test_standalone.py")])

    sim = StairSim()
    viewer = None
    if not args.headless:
        import mujoco
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        # 侧视：看清台阶轮廓和抬腿动作（默认 free camera 落在正上方，只能看到台阶顶面）
        viewer.cam.lookat[:] = [1.00, 0.0, 0.36]
        viewer.cam.distance = 8.6
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -22.0
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = 0

    try:
        rec, summary = run_one(args, viewer, sim)
    finally:
        if viewer is not None:
            viewer.close()

    print(format_summary(summary))
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        path = os.path.join(args.save_dir, "stair_run.csv")
        rec.write_csv(path)
        print("CSV 已写入：%s" % path)
    return 0 if summary["reached"] else 1


if __name__ == "__main__":
    sys.exit(main())
