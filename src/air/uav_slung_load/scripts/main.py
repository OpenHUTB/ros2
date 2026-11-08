#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""吊挂负载四旋翼演示入口（无 ROS 依赖）。

演示模式：悬停 1 s 后向 +x 平移 2 m，跑 12 s 并打印摆角指标；
键盘模式：W/S 前后、A/D 左右、R 复位、Q 退出，目标点按 sim.dt 固定节拍推进。

``--no-viewer`` 用于无显示器的环境（CI / 虚拟机），只跑仿真并打印结果。
"""

import argparse
import os
import select
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controllers import CONTROLLERS  # noqa: E402
from sim_core import DT, SlungSim, run_episode, summarize  # noqa: E402

STEP_TIME = 1.0
DEMO_SECONDS = 12.0


def _pace(next_wall, dt):
    """按固定节拍推进，避免不同机器上仿真快慢不一致。"""
    next_wall += dt
    lag = next_wall - time.time()
    if lag > 0:
        time.sleep(lag)
    else:
        next_wall = time.time()
    return next_wall


def _make_viewer(sim, enabled):
    if not enabled:
        return None
    import mujoco.viewer
    return mujoco.viewer.launch_passive(sim.model, sim.data)


def run_demo(args):
    sim = SlungSim(payload_mass=args.mass)
    ctrl = CONTROLLERS[args.controller]()
    target = np.array([args.distance, 0.0, 1.5])
    viewer = _make_viewer(sim, not args.no_viewer)
    if viewer is None:
        log = run_episode(ctrl, target, args.seconds, payload_mass=args.mass, sim=sim,
                          step_time=STEP_TIME)
    else:
        # 打开窗口时按实时节拍逐步推进，最后再把记录折算成指标
        sim.reset(args.mass)
        ctrl.reset()
        hold = sim.base_pos.copy()
        rec = {"t": [], "pos": [], "target": [], "swing": [], "thrust": []}
        next_wall = time.time()
        for _ in range(int(round(args.seconds / DT))):
            tgt = hold if sim.time < STEP_TIME else target
            sim.step(tgt, ctrl)
            viewer.sync()
            rec["t"].append(sim.time)
            rec["pos"].append(sim.base_pos)
            rec["target"].append(tgt)
            rec["swing"].append(sim.swing_angle)
            rec["thrust"].append(sim._thrust.copy())
            next_wall = _pace(next_wall, DT)
        log = {k: np.asarray(v) for k, v in rec.items()}
    stats = summarize(log, settle_window=2.0, t_start=STEP_TIME)
    print("[demo] 控制器=%s 负载=%.1f kg 目标=+%0.1f m" % (
        args.controller, args.mass, args.distance))
    print("[demo] 摆角 RMS %.2f° / 峰值 %.2f° / 残余 %.2f° / 调节时间 %s s / 末端误差 %.3f m" % (
        stats["swing_rms_deg"], stats["swing_max_deg"], stats["residual_deg"],
        "nan" if np.isnan(stats["settle_s"]) else "%.2f" % stats["settle_s"],
        stats["pos_final_m"]))
    if viewer is not None:
        # 已有记录时用 viewer 回放不便，这里直接提示结果即可
        viewer.close()
    return 0


def run_keyboard(args):
    """键盘模式：目标点按 sim.dt 固定节拍推进，R 键复位。"""
    sim = SlungSim(payload_mass=args.mass)
    ctrl = CONTROLLERS[args.controller]()
    sim.reset(args.mass)
    target = sim.base_pos.copy()
    step = 0.02
    print("W/S 前后，A/D 左右，R 复位，Q 退出（无 GUI 时不显示窗口）")
    viewer = _make_viewer(sim, not args.no_viewer)
    next_wall = time.time()
    keys = ""
    try:
        while True:
            if sys.stdin.isatty():
                r, _, _ = select.select([sys.stdin], [], [], 0.002)
                if r:
                    keys = sys.stdin.read(1).lower()
            else:
                time.sleep(DT)
            if keys == "q":
                break
            if keys == "w":
                target[0] += step
            elif keys == "s":
                target[0] -= step
            elif keys == "a":
                target[1] += step
            elif keys == "d":
                target[1] -= step
            elif keys == "r":
                sim.reset(args.mass)
                target = sim.base_pos.copy()
            keys = ""

            sim.step(target, ctrl)
            if viewer is not None:
                viewer.sync()
            next_wall = _pace(next_wall, DT)
    finally:
        if viewer is not None:
            viewer.close()
    print("目标点 %.2f, %.2f, %.2f" % tuple(target))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="吊挂负载四旋翼演示")
    parser.add_argument("--mode", choices=["demo", "keyboard"], default="demo")
    parser.add_argument("--controller", choices=sorted(CONTROLLERS), default="swing")
    parser.add_argument("--mass", type=float, default=0.3, help="负载质量（kg）")
    parser.add_argument("--distance", type=float, default=2.0, help="演示平移距离（m）")
    parser.add_argument("--seconds", type=float, default=DEMO_SECONDS, help="演示时长（s）")
    parser.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 窗口")
    args = parser.parse_args(argv)

    if args.mode == "demo":
        return run_demo(args)
    return run_keyboard(args)


if __name__ == "__main__":
    raise SystemExit(main())
