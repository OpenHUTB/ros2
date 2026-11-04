# -*- coding: utf-8 -*-
"""MuJoCo 四旋翼 + 吊挂物资仿真。

世界系 ENU，机体系 FLU。飞行器与物资都是自由刚体；物资在初始阶段用
weld equality 挂在机腹，``release()`` 后关闭约束，物资进入自由落体并受风阻力。
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional

import mujoco
import numpy as np

from .mathutil import quat_to_euler
from .state import State


PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# __file__ = <pkg>/src/uav_wind_aware_delivery/sim.py；包根目录在其上一层
PKG_ROOT = os.path.dirname(PKG_ROOT)
DEFAULT_MODEL = os.path.join(PKG_ROOT, "models", "uav_delivery.xml")


@dataclass
class SimConfig:
    model_path: str = DEFAULT_MODEL
    dt: float = 0.01
    substeps: int = 4
    wind: np.ndarray = field(default_factory=lambda: np.array([3.0, 1.0, 0.0]))
    drag_k: float = 0.32         # 物资水平风阻 N/(m/s)
    drone_drag_k: float = 0.10   # 机体受风（让控制器有可见扰动）
    start_pos: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.60]))
    payload_drop: float = 0.30
    target: np.ndarray = field(default_factory=lambda: np.array([5.0, 2.0, 0.0]))


class DeliverySim:
    """封装 MuJoCo 模型和吊挂/释放逻辑。"""

    def __init__(self, cfg: Optional[SimConfig] = None, model_path: Optional[str] = None):
        self.cfg = cfg or SimConfig()
        path = model_path or self.cfg.model_path
        if not os.path.exists(path):
            raise FileNotFoundError(f"MuJoCo model not found: {path}")
        self.model_path = path
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.cfg.dt)
        self.substeps = int(self.cfg.substeps)
        self.time = 0.0

        self.base_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.payload_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "payload")
        self.weld_id = self._id(mujoco.mjtObj.mjOBJ_EQUALITY, "payload_weld")
        self.base_qadr = self._qadr("root")
        self.payload_qadr = self._qadr("payload_free")
        self.base_vadr = self._vadr("root")
        self.payload_vadr = self._vadr("payload_free")
        self._payload_geom = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "payload_geom")
        self.uav_mass = float(self.model.body_subtreemass[self.base_id])
        self.payload_mass = float(self.model.body_mass[self.payload_id])

        self.released = False
        self.landed = False
        self.wind = np.asarray(self.cfg.wind, dtype=float).copy()
        self.reset(wind=self.wind)

    # ------------------------------------------------------------------ ids
    def _id(self, objtype, name: str) -> int:
        i = mujoco.mj_name2id(self.model, objtype, name)
        if i < 0:
            raise ValueError(f"模型里找不到 {name!r}")
        return i

    def _qadr(self, joint: str) -> int:
        j = self._id(mujoco.mjtObj.mjOBJ_JOINT, joint)
        return int(self.model.jnt_qposadr[j])

    def _vadr(self, joint: str) -> int:
        j = self._id(mujoco.mjtObj.mjOBJ_JOINT, joint)
        return int(self.model.jnt_dofadr[j])

    # ---------------------------------------------------------------- reset
    def reset(self, wind: Optional[np.ndarray] = None, target: Optional[np.ndarray] = None):
        if wind is not None:
            self.wind = np.asarray(wind, dtype=float).copy()
        if target is not None:
            self.cfg.target = np.asarray(target, dtype=float).copy()
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.xfrc_applied[:] = 0.0

        start = self.cfg.start_pos
        self.data.qpos[self.base_qadr:self.base_qadr + 3] = start
        self.data.qpos[self.base_qadr + 3:self.base_qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        payload = start + np.array([0.0, 0.0, -self.cfg.payload_drop])
        self.data.qpos[self.payload_qadr:self.payload_qadr + 3] = payload
        self.data.qpos[self.payload_qadr + 3:self.payload_qadr + 7] = [1.0, 0.0, 0.0, 0.0]

        if hasattr(self.data, "eq_active"):
            self.data.eq_active[:] = self.model.eq_active0
        self.released = False
        self.landed = False
        self.time = 0.0
        mujoco.mj_forward(self.model, self.data)

    # --------------------------------------------------------------- action
    def release(self):
        """断开吊挂约束；只在第一次调用时生效。"""
        if self.released:
            return False
        if hasattr(self.data, "eq_active"):
            self.data.eq_active[self.weld_id] = 0
        self.released = True
        return True

    def apply_force_torque(self, force: np.ndarray, torque: np.ndarray):
        """在机体系原点施加世界系合力和世界系力矩。"""
        self.data.xfrc_applied[self.base_id, 0:3] = np.asarray(force, dtype=float)
        self.data.xfrc_applied[self.base_id, 3:6] = np.asarray(torque, dtype=float)

    def _wind_forces(self):
        """物资与机体的风阻力；机体力较小，用于体现扰动。"""
        # 水平风 + 垂直阻尼
        rel = self.wind[:2] - self.data.qvel[self.payload_vadr:self.payload_vadr + 2]
        fp = np.array([self.cfg.drag_k * rel[0],
                       self.cfg.drag_k * rel[1],
                       -0.02 * self.data.qvel[self.payload_vadr + 2]])
        self.data.xfrc_applied[self.payload_id, 0:3] = fp

        uv = self.data.qvel[self.base_vadr:self.base_vadr + 2]
        rel_u = self.wind[:2] - uv
        self.data.xfrc_applied[self.base_id, 0:2] += self.cfg.drone_drag_k * rel_u

    # ----------------------------------------------------------------- step
    def step(self, force: Optional[np.ndarray] = None, torque: Optional[np.ndarray] = None):
        if force is not None:
            self.apply_force_torque(force, torque if torque is not None else np.zeros(3))
        for _ in range(self.substeps):
            self._wind_forces()
            mujoco.mj_step(self.model, self.data)
        self.time += self.dt
        self._update_landed()

    def _update_landed(self):
        if not self.released:
            return
        pz = float(self.data.qpos[self.payload_qadr + 2])
        vz = float(self.data.qvel[self.payload_vadr + 2])
        if pz < 0.115 and abs(vz) < 0.25:
            self.landed = True

    # ---------------------------------------------------------------- state
    def state(self) -> State:
        qp = self.data.qpos[self.base_qadr:self.base_qadr + 7].copy()
        qv = self.data.qvel[self.base_vadr:self.base_vadr + 6].copy()
        pp = self.data.qpos[self.payload_qadr:self.payload_qadr + 3].copy()
        pv = self.data.qvel[self.payload_vadr:self.payload_vadr + 3].copy()
        # 物资接触：用 MuJoCo contact 里是否出现 payload 的 geom
        contact = False
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.geom1 == self._payload_geom or c.geom2 == self._payload_geom:
                contact = True
                break
        return State(
            t=self.time,
            pos=qp[0:3], quat=qp[3:7], vel=qv[0:3], omega=qv[3:6],
            euler=quat_to_euler(qp[3:7]),
            payload_pos=pp, payload_vel=pv,
            released=self.released, landed=self.landed,
            payload_contact=contact, wind=self.wind.copy(),
        )

    def payload_geom_id(self) -> int:
        g = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "payload_geom")
        return int(g)

    @property
    def mass(self) -> float:
        """总质量：吊挂时含物资；释放后减少，控制器随之减推力。"""
        return self.uav_mass + (0.0 if self.released else self.payload_mass)

    def hover_force(self) -> np.ndarray:
        return np.array([0.0, 0.0, self.mass * 9.81])
