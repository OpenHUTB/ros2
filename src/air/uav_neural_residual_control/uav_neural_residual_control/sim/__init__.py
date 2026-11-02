#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把底座仿真服务端的机体换成带未建模效应的 ResidualPlant。

⚠️ 惰性导入：ROS 侧（Python 3.8）没有 mujoco。
"""

__all__ = ["ResidualSimCore", "ResidualSimServer", "serve"]


def __getattr__(name):
    if name in ("ResidualSimCore", "ResidualSimServer", "serve"):
        from . import plant_server
        return getattr(plant_server, name)
    raise AttributeError("module %r has no attribute %r" % (__name__, name))
