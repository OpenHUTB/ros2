#!/usr/bin/env python3
"""无动力被动行走（rimless wheel）模块的统一入口。

用法（不需要 ROS 也能跑）：

    python3 main.py                 # 默认跑 20 s 仿真并打印结果
    python3 main.py --seconds 30    # 跑 30 s
    python3 main.py --push 0.5      # 换初始推力
    python3 main.py --node          # 以 ROS 2 节点运行（需要 rclpy）

    ros2 run passive_walker main.py --seconds 20
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def cmd_run(args):
    from passive_walker.sim import format_summary, run_headless

    try:
        info = run_headless(args.seconds, args.push, args.xml)
    except Exception as exc:            # 缺 mujoco 或找不到模型时给出可读原因
        print("[FAIL] %s" % exc)
        return 1
    print(format_summary(info))
    if info["distance_m"] < 0.5:
        print("[FAIL] 20 s 内几乎没有前进，模型参数可能被改动")
        return 1
    print("[OK] 被动行走运行正常（无执行器，仅靠重力和落地碰撞前进）")
    return 0


def cmd_node(args):
    try:
        from passive_walker import sim_node
    except Exception as exc:
        print("[FAIL] 需要 ROS 2 环境（rclpy）：%s" % exc)
        return 1
    sys.argv = [sys.argv[0]]
    return sim_node.main(seconds=args.seconds, push=args.push)


def cmd_doctor(args):
    print("=== Environment Doctor ===")
    try:
        import mujoco
        print("[OK] mujoco %s" % mujoco.__version__)
    except Exception as exc:
        print("[FAIL] mujoco: %s" % exc)
    try:
        from passive_walker.sim import find_asset
        print("[OK] 模型文件 %s" % find_asset())
    except Exception as exc:
        print("[FAIL] assets: %s" % exc)
    try:
        import rclpy  # noqa: F401
        print("[OK] rclpy")
    except Exception as exc:
        print("[WARN] rclpy 不可用（只影响 --node 模式）：%s" % exc)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="无动力被动行走（rimless wheel）仿真")
    sub = parser.add_subparsers(dest="command")

    p_run = sub.add_parser("run", help="无头运行并打印结果（默认子命令）")
    p_run.add_argument("--seconds", type=float, default=20.0)
    p_run.add_argument("--push", type=float, default=0.3)
    p_run.add_argument("--xml", default=None)
    p_run.set_defaults(func=cmd_run)

    p_node = sub.add_parser("node", help="以 ROS 2 节点运行，发布累计步数")
    p_node.add_argument("--seconds", type=float, default=20.0)
    p_node.add_argument("--push", type=float, default=0.3)
    p_node.set_defaults(func=cmd_node)

    p_doc = sub.add_parser("doctor", help="检查 mujoco / 模型文件 / rclpy")
    p_doc.set_defaults(func=cmd_doctor)

    # 也允许不带子命令：main.py --seconds 20 / main.py --node
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--push", type=float, default=0.3)
    parser.add_argument("--xml", default=None)
    parser.add_argument("--node", action="store_true", help="以 ROS 2 节点运行")
    return parser


def main(argv=None):
    args = build_parser().parse_args(sys.argv[1:] if argv is None else argv)
    if getattr(args, "func", None):
        return args.func(args)
    return cmd_node(args) if args.node else cmd_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
