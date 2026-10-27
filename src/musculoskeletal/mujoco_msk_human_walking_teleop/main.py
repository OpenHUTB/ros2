#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口（按仓库规范）。

用法：
  python3 main.py check                     # 模型自检（无 ROS）
  python3 main.py eval                      # 离线多场景性能评测（无 ROS）
  python3 main.py demo --script walk:8,squat:10   # 脚本化演示（无 ROS，用于录屏）
  python3 main.py sim --scene flat          # ROS: 仿真节点
  python3 main.py keyboard                  # ROS: 键盘遥操作节点
"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_SCRIPTS = os.path.join(_HERE, "scripts")
SCRIPTS = _SRC_SCRIPTS if os.path.isdir(_SRC_SCRIPTS) else _HERE
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, _HERE)


def cmd_check(_args):
    import runpy
    runpy.run_path(os.path.join(SCRIPTS, "msk_human_core.py"), run_name="__main__")


def cmd_eval(_args):
    import runpy
    sys.argv = ["msk_human_eval.py"]
    runpy.run_path(os.path.join(SCRIPTS, "msk_human_eval.py"), run_name="__main__")


def cmd_demo(args):
    """脚本化动作序列演示（无 ROS）：把动作名与时长编成序列回放。"""
    import time

    import numpy as np

    from msk_human_core import Composer, MskHumanSim, load_params

    scene = args.scene
    sim = MskHumanSim(scene, viewer=not args.headless)
    comp = Composer(load_params())
    dt = 0.002
    script = []
    for tok in args.script.split(","):
        name, sec = tok.split(":")
        script.append((name.strip(), float(sec)))
    print("演示序列:", script)
    # 先站稳 1 s
    comp.set_action("stand")
    for _ in range(int(1.0 / dt)):
        sim.step(comp.update(sim, dt))
    for name, sec in script:
        if name == "faster":
            comp.nudge_speed(+0.15)
            name = "walk"
        elif name == "slower":
            comp.nudge_speed(-0.15)
            name = "walk"
        comp.set_action(name)
        t_end = sim.time + sec
        while sim.time < t_end:
            u = comp.update(sim, dt)
            sim.step(u)
            if args.headless and sim.fallen():
                print("t=%.2f 摔倒，终止" % sim.time)
                break
        print("t=%.2f 完成 %-6s x=%.2f z=%.2f pitch=%+.3f" %
              (sim.time, name, sim.pelvis_x(), sim.pelvis_z(), sim.pitch()))
    if not args.headless:
        # 保持窗口 2 s 便于观察
        comp.set_action("stand")
        t_end = sim.time + 2.0
        while sim.time < t_end:
            sim.step(comp.update(sim, dt))
        time.sleep(0.2)
    sim.close()


def cmd_sim(args):
    import rospy
    rospy.init_node("msk_human_sim_node")
    from msk_human_sim_node import SimNode
    SimNode().spin()


def cmd_keyboard(_args):
    import rospy
    rospy.init_node("msk_human_keyboard_node")
    from msk_human_keyboard_node import KeyboardNode
    KeyboardNode().spin()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="模型自检（无 ROS）")
    sub.add_parser("eval", help="离线多场景性能评测（无 ROS）")

    p_demo = sub.add_parser("demo", help="脚本化动作演示（无 ROS）")
    p_demo.add_argument("--scene", default="flat",
                        choices=["flat", "slope5", "load"])
    p_demo.add_argument("--script", default="walk:8,kick:3,squat:10",
                        help="动作序列, 如 walk:6,kick:3,squat:8")
    p_demo.add_argument("--headless", action="store_true")

    p_sim = sub.add_parser("sim", help="ROS 仿真节点")
    p_sim.add_argument("--scene", default="flat")
    p_sim.add_argument("--viewer", default="true")

    sub.add_parser("keyboard", help="ROS 键盘遥操作节点")

    args = ap.parse_args()
    {"check": cmd_check, "eval": cmd_eval, "demo": cmd_demo,
     "sim": cmd_sim, "keyboard": cmd_keyboard}[args.cmd](args)


if __name__ == "__main__":
    main()
