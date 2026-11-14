#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_speed_control 模块入口。

提供三种运行模式：

* ``node``    启动纵向速度控制节点（默认）
* ``demo``    在终端跑一遍仿真演示并输出量化对比（不需要 ROS / CARLA）
* ``check``   只做配置与算法自检，打印参数与限速表后退出（不需要 ROS）

用法::

    # 启动节点
    python3 main.py --mode node

    # 不带 ROS 跑演示（评审者验证用）
    python3 main.py --mode demo

    # 自检：打印不同曲率下的限速
    python3 main.py --mode check

经 launch 启动::

    roslaunch carla_speed_control main.launch
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def run_demo(extra_args):
    """调用演示脚本（独立进程，避免与 ROS 的 import 顺序互相影响）。"""
    script = os.path.join(HERE, "..", "tests", "demo_speed_control.py")
    script = os.path.abspath(script)
    if not os.path.isfile(script):
        print(f"[错误] 找不到演示脚本: {script}")
        return 1
    cmd = [sys.executable, script] + list(extra_args)
    return subprocess.call(cmd)


def run_check(cruise, radius, max_lateral_accel):
    """打印限速对照表，便于快速判断参数是否合理。"""
    from speed_control import (  # noqa: PLC0415
        GRAVITY,
        lateral_accel_at,
        speed_limit_from_curvature,
    )

    print()
    print("=" * 70)
    print("纵向速度控制 —— 参数自检")
    print("=" * 70)
    print(f"  巡航速度        : {cruise:.1f} m/s = {cruise * 3.6:.1f} km/h")
    print(f"  横向加速度上限  : {max_lateral_accel:.1f} m/s²"
          f" = {max_lateral_accel / GRAVITY:.2f} g")
    print()
    print("  不同曲率下的限速：")
    print(f"  {'半径(m)':>9}{'曲率(1/m)':>12}{'限速(m/s)':>11}{'限速(km/h)':>12}"
          f"{'横向加速度':>12}{'是否受曲率限制':>16}")
    print("  " + "-" * 72)
    for r in (10.0, 20.0, 40.0, 80.0, 150.0, 300.0, 1000.0):
        k = 1.0 / r
        v = speed_limit_from_curvature(k, max_lateral_accel, min_speed=1.0)
        v_eff = min(cruise, v)
        limited = "是" if v_eff < cruise - 1e-9 else "否"
        a_here = lateral_accel_at(v_eff, k)
        print(f"  {r:>9.0f}{k:>12.5f}{v_eff:>11.2f}{v_eff * 3.6:>12.1f}"
              f"{a_here:>12.3f}{limited:>16}")
    print()
    print(f"  参考：目标弯道半径 {radius:.0f} m 时限速 "
          f"{min(cruise, speed_limit_from_curvature(1.0 / radius, max_lateral_accel, 1.0)):.2f} m/s")
    print()
    return 0


def run_node():
    """启动 ROS 节点。"""
    try:
        import rospy  # noqa: F401, PLC0415
    except ImportError:
        print("[错误] 未找到 rospy。启动节点需要在 ROS 环境下运行：")
        print("       source /opt/ros/noetic/setup.bash")
        print("       或改用 --mode demo / --mode check（不需要 ROS）")
        return 2

    import speed_controller  # noqa: PLC0415

    rospy.init_node("speed_controller", anonymous=False)
    node = speed_controller.SpeedControllerNode()
    node.spin()
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="carla_speed_control：纵向速度控制（按曲率自动限速 + 速度跟踪）"
    )
    ap.add_argument("--mode", choices=["node", "demo", "check"], default="node",
                    help="运行模式：node=启动 ROS 节点，demo=终端仿真演示，check=参数自检")
    ap.add_argument("--cruise", type=float, default=12.0, help="巡航速度 m/s（demo/check）")
    ap.add_argument("--radius", type=float, default=40.0, help="弯道半径 m（demo/check）")
    ap.add_argument("--max-lateral-accel", type=float, default=3.0,
                    help="横向加速度上限 m/s²（check）")
    ap.add_argument("extra", nargs="*", help="透传给演示脚本的额外参数")
    args = ap.parse_args()

    if args.mode == "demo":
        extra = list(args.extra)
        if "--cruise" not in extra:
            extra += ["--cruise", str(args.cruise)]
        if "--radius" not in extra:
            extra += ["--radius", str(args.radius)]
        return run_demo(extra)
    if args.mode == "check":
        return run_check(args.cruise, args.radius, args.max_lateral_accel)
    return run_node()


if __name__ == "__main__":
    sys.exit(main())
