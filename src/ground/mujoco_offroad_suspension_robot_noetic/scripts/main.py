#!/usr/bin/env python3
"""模块统一入口（仓库规范要求模块提供 main.*）。

用法：
    python3 scripts/main.py                    # 纯物理仿真，出曲线图与指标（不需要 GPU / EGL）
    python3 scripts/main.py --duration 12      # 同上，指定仿真时长（秒）
    python3 scripts/main.py --headless [args]  # 无头渲染，出 MP4 / GIF / 关键帧 / 曲线（需 EGL 或 OSMesa）
    roslaunch mujoco_offroad_suspension_robot_noetic offroad_sim.launch   # ROS 节点（GUI，需显示器）

其余演示脚本保留为可单独运行的形式：
    scripts/physics_only_demo.py  与 main.py 默认模式等价
    scripts/headless_demo.py      与 main.py --headless 等价
    scripts/mujoco_offroad_node.py 由 launch 启动的 ROS 节点
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)

    if argv and argv[0] in ("-h", "--help"):
        print(__doc__.strip())
        return 0

    if argv and argv[0] == "--headless":
        import headless_demo
        sys.argv = [os.path.join(HERE, "headless_demo.py")] + argv[1:]
        headless_demo.main()
        return 0

    # 默认：纯物理仿真
    import argparse
    ap = argparse.ArgumentParser(prog="main.py", add_help=False)
    ap.add_argument("--duration", type=float, default=24.0)
    ap.add_argument("--out", default=None, help="输出目录（默认源码树 docs/img，装到空间后退到 ~/.cache）")
    ns, extra = ap.parse_known_args(argv)
    if extra:
        print("未知参数：%s\n" % " ".join(extra), file=sys.stderr)
        print(__doc__.strip(), file=sys.stderr)
        return 2
    import physics_only_demo
    physics_only_demo.main(duration=ns.duration, out_dir=ns.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
