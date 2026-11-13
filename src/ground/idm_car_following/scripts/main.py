#!/usr/bin/env python3
"""命令行入口：IDM 跟车仿真。

用法:
    python3 scripts/main.py --outdir ../../../docs/img/ground/idm_car_following
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
    ap = argparse.ArgumentParser(description="智能驾驶员模型（IDM）跟车仿真")
    ap.add_argument("--outdir", default="../../../docs/img/ground/idm_car_following", help="配图输出目录")
    args = ap.parse_args(strip_ros_args(sys.argv[1:]))

    res = simulate.run(args.outdir)
    print("单车跟驰: 末端间距 %.2f m, 自车速度 %.2f m/s, 前车速度 %.2f m/s"
          % (res["gap_end"], res["ego_v_end"], res["lead_v_end"]))
    print("车队最终速度: %s (最大离散度 %.3f m/s)"
          % (["%.2f" % v for v in res["platoon_final_speeds"]],
             res["max_final_speed_spread"]))
    print("已生成: %s  idm_gap_speed.png  idm_platoon.png" % args.outdir)


if __name__ == "__main__":
    main()
