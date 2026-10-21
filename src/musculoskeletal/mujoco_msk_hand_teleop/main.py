#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口（按仓库规范）。

用法：
  python3 main.py check                     # 模型自检（无 ROS）
  python3 main.py demo --scene button       # 脚本化演示（无 ROS）
  python3 main.py train [--n-train N ...]   # 训练 NN 逆模型（无 ROS）
  python3 main.py keyboard --scene button   # ROS: 键盘遥操作
  python3 main.py sim --scene button        # ROS: 仿真节点
  python3 main.py nn                        # ROS: NN 逆模型节点
"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
# 源码树：模块位于 scripts/；catkin 安装后：main.py 与各模块同级于 lib/<包名>/
_SRC_SCRIPTS = os.path.join(_HERE, "scripts")
SCRIPTS = _SRC_SCRIPTS if os.path.isdir(_SRC_SCRIPTS) else _HERE
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, _HERE)


def cmd_check(_args):
    import runpy
    runpy.run_path(os.path.join(SCRIPTS, "msk_hand_core.py"), run_name="__main__")


def cmd_demo(args):
    """脚本化按键序列演示（无 ROS，用于验证与录屏）。"""
    import numpy as np
    from msk_hand_core import MskHandSim, N_MUSCLE
    from msk_hand_keyboard import KeyboardPolicy

    sim = MskHandSim(args.scene)
    sim.reset()
    policy = KeyboardPolicy()
    dt = 0.01
    # 演示脚本：[(按键, 保持秒数), ...]
    script = [("u", 0.6), ("a", 0.9), ("a", 0.0), ("q", 0.7),
              ("u", 0.0), ("j", 0.6)]
    if args.keys:
        script = [(k, float(t)) for k, t in
                  (tok.split(":") for tok in args.keys.split(","))]
    for key, hold in script:
        if key:
            policy.toggle(key)
        for _ in range(max(1, int(hold / dt))):
            cmd = policy.update(dt)
            sim.step(cmd)
        print("t=%.2f held=%s tip=%s" %
              (sim.time, sorted(policy.held), np.round(sim.index_tip(), 3)))


def cmd_train(args):
    from msk_hand_nn import main as nn_main
    sys.argv = ["msk_hand_nn.py",
                "--n-train", str(args.n_train),
                "--epochs", str(args.epochs)]
    nn_main()


def _require_rospy():
    try:
        import rospy  # noqa: F401
    except ImportError:
        sys.exit("需要 ROS Noetic 环境（source /opt/ros/noetic/setup.bash）")


def cmd_sim(args):
    _require_rospy()
    import rospy
    from msk_hand_sim_node import SimNode
    rospy.init_node("msk_hand_sim_node")
    rospy.set_param("~scene", args.scene)
    SimNode().spin()


def cmd_keyboard(_args):
    _require_rospy()
    import rospy
    from msk_hand_keyboard_node import KeyboardNode
    rospy.init_node("msk_hand_keyboard_node")
    KeyboardNode().spin()


def cmd_nn(_args):
    _require_rospy()
    from msk_hand_nn_node import NNNode
    import rospy
    rospy.init_node("msk_hand_nn_node")
    NNNode()
    rospy.spin()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="模型自检（无 ROS）")

    p = sub.add_parser("demo", help="脚本化演示（无 ROS）")
    p.add_argument("--scene", default="button",
                   choices=["free", "button", "ball"])
    p.add_argument("--keys", default=None,
                   help="按键脚本，如 'u:0.6,a:0.9,q:0.7,j:0.6'")

    p = sub.add_parser("train", help="训练 NN 逆模型（无 ROS）")
    p.add_argument("--n-train", type=int, default=1200)
    p.add_argument("--epochs", type=int, default=250)

    p = sub.add_parser("sim", help="ROS 仿真节点")
    p.add_argument("--scene", default="free",
                   choices=["free", "button", "ball"])

    sub.add_parser("keyboard", help="ROS 键盘遥操作节点")
    sub.add_parser("nn", help="ROS NN 逆模型节点")

    args = ap.parse_args()
    {"check": cmd_check, "demo": cmd_demo, "train": cmd_train,
     "sim": cmd_sim, "keyboard": cmd_keyboard, "nn": cmd_nn}[args.cmd](args)


if __name__ == "__main__":
    main()
