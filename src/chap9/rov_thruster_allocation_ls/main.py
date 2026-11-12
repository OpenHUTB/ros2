#!/usr/bin/env python3
"""直接运行入口：python3 main.py

未 source ROS 2 环境时给出明确提示，而不是直接抛 ModuleNotFoundError。
"""

try:
    from rov_thruster_allocation_ls.node import main
except ModuleNotFoundError as exc:  # pragma: no cover - 仅用于友好提示
    raise SystemExit(
        "找不到 %s，请先 source /opt/ros/<distro>/setup.bash "
        "并 colcon build --packages-select rov_thruster_allocation_ls" % exc.name
    )

if __name__ == "__main__":
    main()
