#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真世界构造：复用 ``uav_mujoco_sim`` 的四旋翼模型、几何控制器与走廊场景。

``uav_mujoco_sim`` 正常由 colcon 安装；在未安装的源码环境里退回
``src/air/uav_mujoco_sim``，保证本模块的离线实验可以直接 `python3` 跑。
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .mission import Mission


def ensure_sim_importable() -> None:
    """确保可以 import uav_mujoco_sim（colcon 安装或同仓源码）。"""
    try:
        import uav_mujoco_sim  # noqa: F401
        return
    except ImportError:
        pass
    air_root = Path(__file__).resolve().parents[3]
    candidate = air_root / "uav_mujoco_sim"
    if not candidate.exists():
        raise ImportError(
            "找不到 uav_mujoco_sim：请先 colcon build 该包，或把 "
            f"{candidate} 加入 PYTHONPATH")
    sys.path.insert(0, str(candidate))


@dataclass
class World:
    scene: object
    quad: object
    controller: object


def build_world(length: float = 24.0, width: float = 3.0, dt: float = 0.01):
    """走廊场景 + 四旋翼 + 几何控制器。"""
    ensure_sim_importable()
    from uav_mujoco_sim.core import GeometricController, Quadrotor
    from uav_mujoco_sim.core.scenes import corridor

    scene = corridor(length=length, width=width)
    quad = Quadrotor(scene=scene, dt=dt, substeps=4)
    controller = GeometricController(quad.params)
    return World(scene=scene, quad=quad, controller=controller)


def build_mission(start_x: float = -10.5, altitude: float = 1.5) -> Mission:
    """走廊内的巡检任务：四段去程 + 返航。"""
    home = np.array([start_x, 0.0, 1.2])
    waypoints = [
        np.array([start_x + 4.0, 0.0, altitude]),
        np.array([start_x + 9.0, 0.0, altitude]),
        np.array([start_x + 14.0, 0.0, altitude]),
        np.array([start_x + 16.5, 0.0, altitude]),
    ]
    return Mission(home=home, waypoints=waypoints)
