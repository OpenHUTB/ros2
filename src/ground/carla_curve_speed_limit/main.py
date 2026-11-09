#!/usr/bin/env python3
"""carla_curve_speed_limit 统一入口。

阶段：
    doctor  依赖体检（python / numpy / pyyaml / carla / ROS）
    test    离线单元测试（不需要 ROS 与 CARLA）
    build   在 ros2 工作区内 colcon 构建本包
    demo    一键演示（默认离线弯道场景；--live 连 CARLA 真跑环路巡航）

示例：
    python3 main.py doctor
    python3 main.py test
    python3 main.py demo
    python3 main.py demo --live --host 192.168.1.100 --seconds 40
"""

import sys
import unittest
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent


def doctor() -> int:
    """依赖体检：逐项检查并给出安装指引。"""
    print("[doctor] Python:", sys.version.split()[0])
    ok = True

    def check(name, cond, hint):
        nonlocal ok
        mark = "OK  " if cond else "MISS"
        print(f"  [{mark}] {name}" + ("" if cond else f"  ← {hint}"))
        ok = ok and cond

    try:
        import numpy  # noqa: F401
        check("numpy", True, "")
    except ImportError:
        check("numpy", False, "pip install numpy")

    try:
        import yaml  # noqa: F401
        check("pyyaml", True, "")
    except ImportError:
        check("pyyaml", False, "pip install pyyaml")

    try:
        import carla  # noqa: F401
        check("carla（live/演示需要）", True, "")
    except ImportError:
        check("carla（live/演示需要）", False,
              "pip install carla==0.9.16（仅离线测试不需要）")

    try:
        import rclpy  # noqa: F401
        check("ROS 2 rclpy（launch 模式需要）", True, "")
    except ImportError:
        check("ROS 2 rclpy（launch 模式需要）", False,
              "安装 ROS 2 并 source /opt/ros/<distro>/setup.bash")

    print("[doctor]", "全部就绪" if ok else "存在缺失项（离线测试仅需要 numpy/pyyaml）")
    return 0 if ok else 1


def test() -> int:
    """发现并运行 tests/ 下全部单元测试。"""
    suite = unittest.defaultTestLoader.discover(
        str(PKG_ROOT / "tests"), top_level_dir=str(PKG_ROOT))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return 0 if result.wasSuccessful() else 1


def build() -> int:
    """定位 ros2 工作区并 colcon 构建本包。"""
    ws = PKG_ROOT.parent.parent.parent
    if not (ws / "src").is_dir():
        print(f"[build] 未找到 ros2 工作区（{ws}），请在工作区根目录执行："
              "colcon build --packages-select carla_curve_speed_limit")
        return 1
    import subprocess
    return subprocess.call(
        ["colcon", "build", "--packages-select", "carla_curve_speed_limit"],
        cwd=str(ws))


def main() -> int:
    stage = sys.argv[1] if len(sys.argv) > 1 else ""
    if stage == "doctor":
        return doctor()
    if stage == "test":
        return test()
    if stage == "build":
        return build()
    if stage == "demo":
        from carla_curve_speed_limit import demo
        return demo.main(sys.argv[2:])
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
