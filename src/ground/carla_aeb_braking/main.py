#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_aeb_braking 统一入口。

用法：
    python3 main.py doctor            依赖体检（python / numpy / pyyaml / carla / ROS）
    python3 main.py test              离线单元测试（不需要 ROS 与 CARLA）
    python3 main.py build             在 ros2 工作区内 colcon 构建本包
    python3 main.py demo              一键演示（默认离线制动曲线；--live 连 CARLA 真跑）
    python3 main.py live [host]       ros2 launch 启动桥接节点 + AEB 节点连 CARLA 服务
    """
import argparse
import importlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WS = ROOT.parent.parent.parent  # .../ros2


def doctor():
    import numpy
    import yaml
    print(f"python: {sys.version.split()[0]} ({sys.executable})")
    print(f"numpy: {numpy.__version__}")
    print(f"pyyaml: ok")
    try:
        import carla  # noqa: F401
        print(f"carla: ok ({carla.__file__})")
    except ImportError:
        print("carla: 未安装（离线测试不需要；连 CARLA 前安装 "
              "CARLA/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl）")
    if os.path.isdir("/opt/ros"):
        vers = " ".join(sorted(os.listdir("/opt/ros")))
        print(f"ros: /opt/ros ({vers})")
    else:
        print("ros: 未安装（离线测试不需要；ROS 节点需在 ROS 2 环境运行）")
    print("doctor: OK")
    return 0


def test():
    code = (
        "import sys, unittest; sys.path.insert(0, r'%s');\n"
        "from pathlib import Path\n"
        "loader = unittest.TestLoader(); suite = loader.discover(r'%s/test', 'test_*.py')\n"
        "res = unittest.TextTestRunner(verbosity=2).run(suite)\n"
        "sys.exit(0 if res.wasSuccessful() else 1)\n" % (ROOT, ROOT)
    )
    return subprocess.call([sys.executable, "-c", code])


def build():
    colcon = shutil.which("colcon")
    if colcon is None:
        print("未找到 colcon；请先安装（apt install python3-colcon-common-extensions）")
        return 1
    return subprocess.call([colcon, "build", "--packages-select", "carla_aeb_braking"], cwd=str(WS))


def demo(args):
    from carla_aeb_braking import demo as demo_mod
    if args.live:
        return demo_mod.run_live(host=args.host, port=args.port, gap=args.gap)
    return demo_mod.run_offline()


def live(host):
    print(f"启动 ROS 2 节点（桥接 + AEB 控制）连 CARLA {host}：")
    print("  ros2 launch carla_aeb_braking main.launch.py host:=%s" % host)
    return subprocess.call(
        ["ros2", "launch", "carla_aeb_braking", "main.launch.py", f"host:={host}"])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("stage", choices=["doctor", "test", "build", "demo", "live"])
    ap.add_argument("--host", default="127.0.0.1",
                    help="demo --live / live 阶段的 CARLA 服务器地址")
    ap.add_argument("--port", type=int, default=2000, help="CARLA 端口")
    ap.add_argument("--live", action="store_true",
                    help="demo 阶段：连接 CARLA 跑真实场景（默认离线仿真）")
    ap.add_argument("--gap", type=float, default=50.0,
                    help="demo --live：障碍车初始距离（米，上限为车道干净距离）")
    a = ap.parse_args(argv)
    if a.stage == "doctor":
        return doctor()
    if a.stage == "test":
        return test()
    if a.stage == "build":
        return build()
    if a.stage == "demo":
        return demo(a)
    return live(a.host)


if __name__ == "__main__":
    raise SystemExit(main())
