"""humanoid_h1_walk：Unitree H1 / H1_2 人形机器人的神经网络行走控制。

模块分三层：

* :mod:`~humanoid_h1_walk.walker`：MuJoCo 模型 + TorchScript 策略的纯 Python 闭环；
* :mod:`~humanoid_h1_walk.sim_node` / :mod:`~humanoid_h1_walk.policy_node`：ROS 2 拆分；
* ``scripts/``：无头测试、机型对比、动图录制。
"""

from .walker import (
    GAIT_PERIOD,
    ROUTINE,
    RobotSpec,
    Walker,
    gravity_in_body,
    load_specs,
    package_root,
    routine_command,
)

__all__ = [
    "GAIT_PERIOD",
    "ROUTINE",
    "RobotSpec",
    "Walker",
    "gravity_in_body",
    "load_specs",
    "package_root",
    "routine_command",
]

__version__ = "0.1.0"
