#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_neural_residual_control 统一入口（作业要求入口以 main 开头）。

薄封装，逻辑在 ``uav_neural_residual_control/cli.py``：源码目录直接
``python main.py …`` 与安装后 ``ros2 run uav_neural_residual_control main …``
走同一份代码。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# 源码目录直接运行时（还没 colcon install），底座包 uav_mujoco_sim 在同级目录里；
# 把它也挂上，省掉手动设 PYTHONPATH。装好之后这一步是空操作。
_SIBLING = os.path.join(os.path.dirname(_HERE), "uav_mujoco_sim")
if os.path.isdir(os.path.join(_SIBLING, "uav_mujoco_sim")) and _SIBLING not in sys.path:
    sys.path.insert(0, _SIBLING)

from uav_neural_residual_control.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
