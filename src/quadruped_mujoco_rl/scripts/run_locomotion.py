#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑一次四足运动控制实验并打印指标（纯 Python，不依赖 ROS）。

示例
----
站立基线（可选外部推力抗扰）::

    python3 run_locomotion.py --mode stand --seconds 3
    python3 run_locomotion.py --mode stand --seconds 3 --disturbance 60

调优后的对角步态::

    python3 run_locomotion.py --mode trot --seconds 8
    python3 run_locomotion.py --mode trot --seconds 8 --freq 3.97 \\
        --thigh-amp 0.15 --calf-amp 0.29

朴素开环步态（会翻倒，用于对照）::

    python3 run_locomotion.py --mode trot --seconds 3 --freq 1.0 \\
        --thigh-amp 0.35 --calf-amp 0.35 --kp-roll 0

加 --csv 保存逐帧轨迹 (t, x, y, z, tilt)。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys

from locomotion_core import (
    QuadrupedModel,
    StandController,
    TrotGait,
    default_model_path,
    format_metrics,
    simulate,
)


def main(argv=None):
    p = argparse.ArgumentParser(description="四足机器人运动控制实验")
    p.add_argument("--model", default=default_model_path(),
                   help="MJCF 场景文件（默认 ../models/quadruped.xml）")
    p.add_argument("--mode", choices=["stand", "trot"], default="trot")
    p.add_argument("--seconds", type=float, default=4.0)
    p.add_argument("--disturbance", type=float, default=0.0,
                   help="站立模式下在躯干施加的水平推力 (N)")
    p.add_argument("--disturbance-start", type=float, default=0.5)
    p.add_argument("--disturbance-duration", type=float, default=0.1)
    p.add_argument("--freq", type=float, default=3.97, help="步态频率 (Hz)")
    p.add_argument("--thigh-amp", type=float, default=0.15, help="大腿摆幅 (rad)")
    p.add_argument("--calf-amp", type=float, default=0.29, help="小腿摆幅 (rad)")
    p.add_argument("--phase-offset", type=float, default=math.pi / 2)
    p.add_argument("--kp-roll", type=float, default=0.01)
    p.add_argument("--kp-pitch", type=float, default=0.0)
    p.add_argument("--kp-height", type=float, default=0.0)
    p.add_argument("--csv", default=None, help="保存逐帧轨迹的 CSV 路径")
    ns = p.parse_args(sys.argv[1:] if argv is None else argv)

    robot = QuadrupedModel(ns.model)
    print("模型: %s" % ns.model)
    print("执行器: nu=%d（位置伺服，ctrl 即目标关节角）  时间步: %.4f s"
          % (robot.model.nu, robot.timestep))

    disturbance = None
    if ns.mode == "stand" and ns.disturbance > 0:
        disturbance = (ns.disturbance, ns.disturbance_start,
                       ns.disturbance_duration)

    if ns.mode == "stand":
        controller = StandController(robot)
        print("\n模式: 站立/姿态保持（目标角 = 关键帧姿态）")
        if disturbance:
            print("扰动: %.0f N 水平推力，%.1f s 起持续 %.1f s"
                  % disturbance)
    else:
        controller = TrotGait(
            robot, freq=ns.freq, a_thigh=ns.thigh_amp, a_calf=ns.calf_amp,
            phase_offset=ns.phase_offset, kp_roll=ns.kp_roll,
            kp_pitch=ns.kp_pitch, kp_height=ns.kp_height)
        print("\n模式: 对角步态 (trot)，参数: %s" % controller.params())

    metrics = simulate(robot, controller, seconds=ns.seconds,
                       disturbance=disturbance, log=bool(ns.csv))
    print("\n## 实测结果\n")
    print(format_metrics(metrics))

    if ns.csv:
        with open(ns.csv, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t", "x", "y", "z", "tilt_deg"])
            writer.writerows(metrics["trace"][::max(1, len(metrics["trace"]) // 500)])
        print("\n逐帧轨迹已写入: %s" % ns.csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
