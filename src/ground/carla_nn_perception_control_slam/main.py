#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模块根目录薄包装入口。

等价于在已构建并 source 过的工作空间执行：
    ros2 run carla_nn_perception_control_slam main --task 1

前置步骤（工作空间根目录）：
    colcon build --packages-select carla_nn_perception_control_slam
    source install/setup.bash
"""
import subprocess
import sys


def main() -> int:
    cmd = ["ros2", "run", "carla_nn_perception_control_slam", "main"] + sys.argv[1:]
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
