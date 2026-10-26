# -*- coding: utf-8 -*-
"""定位并导入底座 ``uav_mujoco_sim``（不修改它，只复用其公开 API）。

三种情形都要能用：

1. **colcon 安装后**：``source install/setup.bash`` → ``import uav_mujoco_sim`` 直接成功。
2. **源码树里直接跑**：本文件位于 ``src/air/uav_actuator_fault_sim/uav_actuator_fault_sim/``，
   底座的项目目录在 ``src/air/uav_mujoco_sim/``（其内层才是 ``uav_mujoco_sim/`` 包），
   把**项目目录**插进 ``sys.path`` 才能正确解析。
3. **底座被放到别处**：用环境变量 ``UAV_MUJOCO_SIM_SRC`` 指向其项目目录（或 ``src/air``）。

一个容易踩的坑：``src/air`` 本身不能放进 ``sys.path``。那样 ``uav_mujoco_sim`` 会被解析成
``src/air/uav_mujoco_sim/`` 这个**没有 ``__init__.py`` 的项目目录**（命名空间包），
``import uav_mujoco_sim.core`` 就会报 ``No module named 'uav_mujoco_sim.core'``。
本模块会先把这种错误的命名空间包从 ``sys.modules`` 里清掉再重试。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Iterator, NamedTuple

#: 底座的包名（import 名），与项目目录同名
BASE_PACKAGE = "uav_mujoco_sim"


class BaseAPI(NamedTuple):
    """底座暴露给本模块的全部符号。"""

    Quadrotor: object
    QuadParams: object
    QuadrotorState: object
    GeometricController: object
    mixer: object
    make_scene: object
    VisionSensors: object
    CameraConfig: object
    LidarSensor: object
    LidarConfig: object


def _project_dirs() -> Iterator[Path]:
    """产出可能**包含** ``uav_mujoco_sim/`` 包的项目目录。"""
    env = os.environ.get("UAV_MUJOCO_SIM_SRC")
    if env:
        p = Path(env)
        # 允许指向项目目录本身，或指向其上一级（src/air）
        yield p
        yield p / BASE_PACKAGE
    here = Path(__file__).resolve().parent      # .../uav_actuator_fault_sim/uav_actuator_fault_sim
    yield here.parent.parent / BASE_PACKAGE     # src/air/uav_mujoco_sim
    yield here.parent.parent.parent / BASE_PACKAGE  # src/uav_mujoco_sim（兜底）


def _ensure_on_path() -> None:
    """把第一个真正含 ``uav_mujoco_sim/__init__.py`` 的项目目录放到 ``sys.path`` 最前。"""
    for root in _project_dirs():
        if (root / BASE_PACKAGE / "__init__.py").is_file():
            path = str(root)
            if path not in sys.path:
                sys.path.insert(0, path)
            return
    # 一个都没找到：交给正常的 import 流程，让报错信息说话


def _drop_namespace_shadow() -> None:
    """清掉被误解析成命名空间包（``__file__`` 为 None）的 ``uav_mujoco_sim``。"""
    for name in [n for n in list(sys.modules)
                 if n == BASE_PACKAGE or n.startswith(BASE_PACKAGE + ".")]:
        mod = sys.modules.get(name)
        if mod is not None and getattr(mod, "__file__", None) is None:
            del sys.modules[name]


def _import_all() -> BaseAPI:
    from uav_mujoco_sim.core.quadrotor import (  # noqa: WPS433
        GeometricController,
        QuadParams,
        Quadrotor,
        QuadrotorState,
        mixer,
    )
    from uav_mujoco_sim.core.scenes import make_scene  # noqa: WPS433
    from uav_mujoco_sim.core.sensors import (  # noqa: WPS433
        CameraConfig,
        LidarConfig,
        LidarSensor,
        VisionSensors,
    )
    return BaseAPI(
        Quadrotor=Quadrotor,
        QuadParams=QuadParams,
        QuadrotorState=QuadrotorState,
        GeometricController=GeometricController,
        mixer=mixer,
        make_scene=make_scene,
        VisionSensors=VisionSensors,
        CameraConfig=CameraConfig,
        LidarSensor=LidarSensor,
        LidarConfig=LidarConfig,
    )


def load_base() -> BaseAPI:
    """导入底座并返回其公开符号。

    先按已配置的环境正常 import；失败则修路径、清掉误解析的命名空间包后重试一次。

    Raises
    ------
    ImportError
        底座不可用，附带排查提示（缺的是 pip 的 mujoco 还是源码路径）。
    """
    try:
        return _import_all()
    except ImportError:
        pass

    _drop_namespace_shadow()
    _ensure_on_path()
    try:
        return _import_all()
    except ImportError as exc:  # pragma: no cover - 环境问题分支
        raise ImportError(
            "无法导入底座 uav_mujoco_sim。请确认：\n"
            "  1) 已安装 MuJoCo：pip install 'mujoco>=3.0'\n"
            "  2) 已在同一 colcon 工作空间内 build 并 source，或设置环境变量\n"
            "     UAV_MUJOCO_SIM_SRC 指向底座项目目录，例如\n"
            "       $env:UAV_MUJOCO_SIM_SRC='E:\\openhutb\\ros2\\src\\air\\uav_mujoco_sim'\n"
            f"原始错误：{exc}"
        ) from exc
