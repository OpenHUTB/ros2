#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""main.py —— 多传感器融合跟踪模块统一入口（main.* 入口约定）

用法：
    python3 main.py doctor              # 依赖体检
    python3 main.py test                # 离线最小测试（无需 CARLA / ROS）
    python3 main.py sim2d [--frames 240] [--show] [--csv fusion_metrics.csv]
    python3 main.py carla [--host 127.0.0.1] [--map Town03] [--seconds 30]

    roslaunch carla_sensor_fusion main.launch            # ROS 工作空间内一键启动
    roslaunch carla_sensor_fusion main.launch mode:=sim2d

说明：
    - sim2d：2D 轻量合成传感器仿真，本机直接可跑，生成效果动图 + 指标 CSV。
    - carla：CARLA 渲染版，需本机安装并启动 CARLA（详见 carla_sensor_fusion.py）。
"""
import argparse
import os
import sys


ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def doctor():
    print(f"python: {sys.version.split()[0]} ({sys.executable})")
    try:
        import numpy as np
        print(f"numpy: {np.__version__}")
    except ImportError:
        print("numpy: 未安装")
    for name in ("matplotlib", "PIL"):
        try:
            __import__(name)
            print(f"{name}: ok")
        except ImportError:
            print(f"{name}: 未安装（2D 版需要）")
    try:
        import carla  # noqa: F401
        print(f"carla: ok ({carla.__file__})")
    except ImportError:
        print("carla: 未安装（2D 版与离线测试不需要；CARLA 版需本机安装）")
    print("doctor: OK")
    return 0


def test():
    import test_sensor_fusion
    return test_sensor_fusion.run()


def main(argv=None):
    ap = argparse.ArgumentParser(description="多传感器融合跟踪模块统一入口")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("sim2d", help="2D 轻量合成仿真（生成 gif + 指标 CSV）")
    p.add_argument("--frames", type=int, default=240)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--out", default="fusion_demo.gif",
                   help="动图输出路径（本地默认 fusion_demo.gif）")
    p.add_argument("--csv", default="fusion_metrics.csv")
    p.add_argument("--warmup-frames", type=int, default=0,
                   help="统计 RMSE 时跳过前 N 帧（默认 0=全程统计）")
    p.add_argument("--show", action="store_true")

    p = sub.add_parser("carla", help="CARLA 渲染版（需本机 CARLA）")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--map", default="Town03")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--radar-hfov", type=float, default=30.0)
    p.add_argument("--radar-range", type=float, default=40.0)
    p.add_argument("--radar-tick", type=float, default=0.05)
    p.add_argument("--csv", default="carla_metrics.csv")

    p = sub.add_parser("replay", help="无 CARLA 离线端到端回放（验证链路 + 产出指标）")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--csv", default="offline_replay_metrics.csv")

    sub.add_parser("test", help="离线最小测试（无需 CARLA）")
    sub.add_parser("doctor", help="依赖体检")

    # roslaunch 启动节点时会注入 __name:= / __log:= 等 ROS remap 参数，
    # 用 parse_known_args 忽略它们，避免 roslaunch 场景下 argparse 直接报错退出。
    ap.add_argument("--ros", action="store_true",
                     help="在 ROS 环境运行时发布 /usv/fusion/* 话题（供 roslaunch 验收）")
    args, _ros_remaps = ap.parse_known_args(argv)

    if args.cmd == "sim2d":
        from sim_2d_fusion import main as sim_main
        argv_s = ["--frames", str(args.frames), "--fps", str(args.fps),
                  "--out", args.out, "--csv", args.csv,
                  "--warmup-frames", str(args.warmup_frames)]
        if args.show:
            argv_s.append("--show")
        if getattr(args, "ros", False):
            argv_s.append("--ros")
        return sim_main(argv_s)
    if args.cmd == "carla":
        from carla_sensor_fusion import main as carla_main
        argv_c = ["--host", args.host, "--port", str(args.port),
                  "--timeout", str(args.timeout), "--map", args.map,
                  "--seconds", str(args.seconds), "--csv", args.csv,
                  "--radar-hfov", str(args.radar_hfov),
                  "--radar-range", str(args.radar_range),
                  "--radar-tick", str(args.radar_tick)]
        return carla_main(argv_c)
    if args.cmd == "test":
        return test()
    if args.cmd == "replay":
        from offline_replay import main as replay_main
        return replay_main(["--seconds", str(args.seconds), "--csv", args.csv])
    return doctor()


if __name__ == "__main__":
    sys.exit(main())
