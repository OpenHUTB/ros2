# -*- coding: utf-8 -*-
"""MuJoCo 单足弹跳机器人：SLIP 模型 + Raibert 落点控制。

子模块
------
config       常量
controller   纯 Python 控制器（可无 MuJoCo 单测）
sim          MuJoCo 封装
metrics      逐跳指标
experiments  三项实验
figures      曲线与动图
"""

from mujoco_hopper_slip_control.config import VERSION
from mujoco_hopper_slip_control.controller import (
    Command,
    HopperController,
    Phase,
    desired_liftoff_speed,
    foot_placement_angle,
    required_liftoff_speed,
)

__version__ = VERSION

__all__ = [
    "VERSION",
    "__version__",
    "Command",
    "HopperController",
    "Phase",
    "desired_liftoff_speed",
    "foot_placement_angle",
    "required_liftoff_speed",
]
