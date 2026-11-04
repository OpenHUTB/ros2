#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""被控对象：底座四旋翼 + 未建模效应。

在 ``uav_mujoco_sim.core.Quadrotor`` 上叠加 drag / wind / payload / ground effect。
不重写推进循环，而是覆写两个钩子（``apply_rotor_thrust`` / ``apply_wind``），
推力饱和、自由关节读写、``mj_forward`` 等逻辑仍走底座实现，两边不会走偏。

两条质量口径：

    true_mass     含载荷的实际质量（MuJoCo 按几何+惯量算出）
    nominal_mass  控制器所"认为"的质量（不含载荷）

两者之差正是残差网络必须靠观测补回来的部分。
"""
from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from typing import Optional, Union

import numpy as np

from uav_mujoco_sim.core import Quadrotor, make_scene, scene_to_xml

from .attitude import QuadParams, quat_to_mat
from .disturbances import GRAVITY, PlantConfig, make_plant

__all__ = ["ResidualPlant", "inject_payload", "nominal_params_of", "as_quad_params"]


def as_quad_params(p) -> QuadParams:
    """底座 QuadParams → 本包的 QuadParams。

    两份定义字段完全相同（数值也一致），分开只是为了让 ROS 侧（Python 3.8、
    没有 mujoco）能独立导入控制器栈。
    """
    return QuadParams(mass=float(p.mass), arm=float(p.arm), k_m=float(p.k_m),
                      max_thrust=float(p.max_thrust),
                      hover_thrust=float(p.hover_thrust))


def inject_payload(scene_xml: str, payload_mass: float, offset=(0.0, 0.0, -0.075),
                   body_name: str = "base_link") -> str:
    """在机体下方挂一个载荷刚体，返回修改后的 MJCF。

    用 XML 后处理而不是直接改 ``model.body_mass``：后者绕开 MuJoCo 的
    惯量重算（``mj_setConst``），质量与惯量会静默不一致 —— 看着能跑，
    转动惯量却还是空机的，姿态响应完全不对。
    """
    if payload_mass <= 0.0:
        return scene_xml

    root = ET.fromstring(scene_xml)
    parent = None
    for body in root.find("worldbody").iter("body"):
        if body.get("name") == body_name:
            parent = body
            break
    if parent is None:
        raise ValueError("场景 XML 中找不到机体 %r" % body_name)

    a = 0.05                                   # 载荷按 5 cm 立方体建模
    I = payload_mass * a * a / 6.0             # 实心立方体 I = m·a²/6
    pb = ET.SubElement(parent, "body", {
        "name": "payload",
        "pos": "%.5f %.5f %.5f" % tuple(offset),
    })
    ET.SubElement(pb, "inertial", {
        "pos": "0 0 0",
        "mass": "%.6f" % payload_mass,
        "diaginertia": "%.8f %.8f %.8f" % (I, I, I),
    })
    ET.SubElement(pb, "geom", {
        "name": "payload_geom", "type": "box",
        "size": "%.4f %.4f %.4f" % (a / 2, a / 2, a / 2),
        "rgba": "0.85 0.55 0.15 1",
    })
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding="unicode")


class ResidualPlant(Quadrotor):
    """带未建模效应的四旋翼。

    扰动在每个物理子步重算并**赋值**到 ``xfrc_applied``：阻力依赖当前速度，
    子步之间速度会变；累加则会在同一控制周期内翻 ``substeps`` 倍。
    """

    def __init__(self, plant: PlantConfig = None, scene: Union[str, object] = "open_ground",
                 seed: int = 0, dt: float = 0.01, substeps: int = 4,
                 nominal_mass: float = None, scene_kwargs: dict = None,
                 base_mass: float = 0.95):
        self.plant_cfg = plant if plant is not None else make_plant()
        if isinstance(scene, str):
            scene = make_scene(scene, seed=seed, **(scene_kwargs or {}))
        xml = inject_payload(scene_to_xml(scene), self.plant_cfg.payload.mass,
                             self.plant_cfg.payload.offset)
        super().__init__(xml=xml, dt=dt, substeps=substeps)

        self.scene = scene
        self.base_mass = float(base_mass)
        self.true_mass = float(self.params.mass)
        self.nominal_mass = float(nominal_mass) if nominal_mass is not None \
            else self.true_mass - self.plant_cfg.payload.mass
        # 控制器只看到这一份参数：质量用标称值，其余几何量按空机读
        self.nominal_params = QuadParams(
            mass=self.nominal_mass, arm=self.params.arm, k_m=self.params.k_m,
            max_thrust=self.params.max_thrust,
            hover_thrust=self.nominal_mass * GRAVITY / 4.0)
        self.true_params = as_quad_params(self.params)

        self.plant_cfg.reset()
        self._last_acc = np.zeros(3)
        self._thrust_cmd = np.zeros(4)
        self._thrust_applied = np.zeros(4)
        self._ground_scale = 0.0

    # ------------------------------------------------------------ 底座钩子
    def apply_rotor_thrust(self, thrusts: np.ndarray) -> None:
        """地面效应在此叠加：同一转速近地推力更大。"""
        t = np.asarray(thrusts, dtype=float)
        self._ground_scale = self.plant_cfg.ground_scale(float(self.qpos[2]))
        if self._ground_scale > 0.0:
            t = t * (1.0 + self._ground_scale)
        super().apply_rotor_thrust(t)
        self._thrust_applied = self.data.ctrl[:4].copy()

    def apply_wind(self) -> None:
        """按当前状态重算阻力并**赋值**到 xfrc_applied 的力分量。

        覆盖底座的 ``+=`` 版本：阻力是状态的函数，必须每个子步重算；
        同时推进风场（OU 过程，dt 取物理子步长）。
        """
        self.plant_cfg.wind.update(self.dt / self.substeps)
        R = quat_to_mat(self.qpos[3:7])
        F = self.plant_cfg.drag_force(self.qvel[0:3], R)
        self.data.xfrc_applied[self.body_id, 0:3] = F

    def step_with_control(self, thrusts: np.ndarray, n: int = 1) -> None:
        for _ in range(n):
            v0 = self.qvel[0:3].copy()
            self._thrust_cmd = np.asarray(thrusts, dtype=float).copy()
            super().step_with_control(thrusts, 1)
            self._last_acc = (self.qvel[0:3] - v0) / self.dt

    # ------------------------------------------------------------ 真值接口
    @property
    def last_acc(self) -> np.ndarray:
        """上一控制周期的实测世界系加速度（速度差分）。"""
        return self._last_acc.copy()

    @property
    def thrust_cmd(self) -> np.ndarray:
        """上一控制周期控制器**要求**的四桨推力（未含地面效应、未饱和）。"""
        return self._thrust_cmd.copy()

    @property
    def thrust_applied(self) -> np.ndarray:
        """上一控制周期实际生效的四桨推力（含地面效应、含饱和）。两者不等
        即说明执行器已饱和或地面效应在起作用。"""
        return self._thrust_applied.copy()

    @property
    def ground_scale(self) -> float:
        return self._ground_scale

    def nominal_accel(self, thrusts: Optional[np.ndarray] = None,
                      quat: Optional[np.ndarray] = None) -> np.ndarray:
        """标称模型预测的世界系加速度。

        用控制器**要求**的推力而不是实际生效的推力：地面效应、饱和都是
        未被标称模型描述的效应，应当留在残差里，而不是被这里悄悄吸收。
        """
        q = self.qpos[3:7] if quat is None else quat
        t = self._thrust_cmd if thrusts is None else np.asarray(thrusts, dtype=float)
        R = quat_to_mat(q)
        return R @ np.array([0.0, 0.0, float(np.sum(t)) / self.nominal_mass]) + \
            np.array([0.0, 0.0, -GRAVITY])

    def true_disturbance_accel(self, thrusts: Optional[np.ndarray] = None) -> np.ndarray:
        """真值扰动加速度 Δa = a_实测 − a_标称，世界系。

        看似需要"上帝视角"，其实只用可观测量：``a_实测`` 由里程计速度差分得到，
        ``a_标称`` 由下达的推力与姿态算出。真正不可观测的只有后半段评估用的
        真值分解（阻力/风/载荷各占多少）。
        """
        return self._last_acc - self.nominal_accel(thrusts)

    def reset(self, pos=(0.0, 0.0, 1.5), yaw: float = 0.0) -> None:
        super().reset(pos=pos, yaw=yaw)
        self.plant_cfg.reset()
        self._last_acc = np.zeros(3)
        self._thrust_cmd = np.zeros(4)
        self._thrust_applied = np.zeros(4)
        self._ground_scale = 0.0


def nominal_params_of(base_mass: float = 0.95, arm: float = 0.106,
                      k_m: float = 0.015, max_thrust: float = 6.0) -> QuadParams:
    """空机参数 —— 所有基线控制器共用的一份"控制器眼中的机体"。"""
    return QuadParams(mass=base_mass, arm=arm, k_m=k_m, max_thrust=max_thrust,
                      hover_thrust=base_mass * GRAVITY / 4.0)


def clone_nominal(p: QuadParams) -> QuadParams:
    return copy.copy(p)
