#!/usr/bin/env python3
"""入口：python3 main.py [--no-ros]

默认启动 ROS 2 节点。加 `--no-ros` 时用纯 Python 跑一遍与节点相同的控制律
（project_to_path -> lookahead_point(s + L_d) -> curvature(reverse) -> rate_limit，
再用 path_sim 的积分方式推进），不依赖 rclpy，便于没有 ROS 的环境复现横向误差。
"""

import math
import sys


def run_offline():
    """不依赖 ROS 的离线复现，参数与 node.py / path_sim.py 默认值一致。"""
    from pure_pursuit_reverse import core

    path = core.DEFAULT_PATH
    speed = 0.25
    lookahead = 0.9
    max_steer_rate = 1.2
    dt = 0.05

    pose = (0.6, 0.05, math.pi)
    steer = 0.0
    t = 0.0
    max_lateral = 0.0
    total = core.path_length(path)

    while True:
        s, lateral, _ = core.project_to_path(pose, path)
        max_lateral = max(max_lateral, abs(lateral))
        if s >= total - 0.15 or t > 120.0:
            break
        target = core.lookahead_point(path, s, lookahead)
        kappa, _ = core.pure_pursuit_curvature(pose, target, reverse=True)
        steer = core.rate_limit(steer, kappa * speed, max_steer_rate * dt)
        v, w = -speed, steer
        theta = pose[2] + w * dt
        x = pose[0] + v * math.cos(theta) * dt
        y = pose[1] + v * math.sin(theta) * dt
        pose = (x, y, theta)
        t += dt

    print("offline reverse pure pursuit")
    print("  t=%.2f s  s=%.2f/%.2f m  pos=(%.2f, %.2f)" % (t, s, total, pose[0], pose[1]))
    print("  max lateral error = %.4f m" % max_lateral)
    return 0


def main(args=None):
    if "--no-ros" in sys.argv[1:]:
        return run_offline()
    try:
        from pure_pursuit_reverse.node import main as ros_main
    except ModuleNotFoundError as exc:  # pragma: no cover - 仅用于友好提示
        raise SystemExit(
            "找不到 %s，请先 source /opt/ros/<distro>/setup.bash 并 "
            "colcon build --packages-select pure_pursuit_reverse；"
            "或加 --no-ros 跑离线复现" % exc.name
        )
    ros_main(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
