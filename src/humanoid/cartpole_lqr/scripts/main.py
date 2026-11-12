#!/usr/bin/env python3
"""命令行入口：设计 LQR 并仿真倒立摆平衡。

用法:
    python3 scripts/main.py --outdir ../../../docs/img/humanoid/cartpole_lqr
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate


def strip_ros_args(argv):
    """把 ``--ros-args`` 及其后内容交给 rclpy，argparse 只解析它前面的部分。

    否则 ``ros2 run pkg main.py --ros-args -p foo:=1`` 会触发
    "unrecognized arguments: --ros-args ..."。
    """
    for i, a in enumerate(argv):
        if a == "--ros-args":
            return argv[:i]
    return argv


def main():
    ap = argparse.ArgumentParser(description="倒立摆 Cart-Pole 的 LQR 平衡控制")
    ap.add_argument("--outdir", default="../../../docs/img/humanoid/cartpole_lqr", help="配图输出目录")
    ap.add_argument("--theta0", type=float, default=0.1, help="初始摆角 (rad)")
    args = ap.parse_args(strip_ros_args(sys.argv[1:]))

    res = simulate.run(args.outdir, theta0=args.theta0)
    print("LQR 增益 K = [%.3f, %.3f, %.3f, %.3f]" % tuple(res["K"]))
    print("LQR   : 末端摆角 %.5f rad, 末端位置 %.5f m, 峰值控制力 %.2f N"
          % (res["theta_end_lqr"], res["pos_end_lqr"], res["max_force"]))
    print("无控  : 末端摆角 %.3f rad（已倒下）" % res["theta_end_passive"])
    print("已生成: %s  cartpole_angle.png  cartpole_state.png" % args.outdir)


if __name__ == "__main__":
    main()
