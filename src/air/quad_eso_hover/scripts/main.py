#!/usr/bin/env python3
"""命令行入口：四旋翼高度 PD + ESO 抗扰仿真。

用法:
    python3 scripts/main.py --outdir ../../../docs/img/air/quad_eso_hover
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
    ap = argparse.ArgumentParser(description="四旋翼高度 PD + ESO(ADRC) 抗扰控制")
    ap.add_argument("--outdir", default="../../../docs/img/air/quad_eso_hover", help="配图输出目录")
    args = ap.parse_args(strip_ros_args(sys.argv[1:]))

    res = simulate.run(args.outdir)
    print("PD-only : 末端高度 %.3f m, 稳态误差 %.3f m" % (res["pd_end_z"], res["pd_err"]))
    print("PD+ESO  : 末端高度 %.3f m, 稳态误差 %.3f m" % (res["eso_end_z"], res["eso_err"]))
    print("ESO 扰动估计: z3_hat=%.3f (真实 f=%.3f), 稳态平均估计误差 %.4f"
          % (res["z3_final"], res["f_final"], res["mean_est_err"]))
    print("已生成: %s  eso_altitude.png  eso_disturbance.png" % args.outdir)


if __name__ == "__main__":
    main()
