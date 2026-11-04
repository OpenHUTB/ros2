#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""向后兼容 shim：`from pid_controller import PIDController` 仍然可用。

模块重构后 PID 实现搬到了 `mujoco_balance_car/pid.py`（带积分限幅等改进），
但仓库文档与历史用法里可能有 `import pid_controller`。这里保留一层转发，
避免直接删文件把别人的用法弄坏。
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from mujoco_balance_car.pid import PIDController  # noqa: F401,E402

__all__ = ["PIDController"]
