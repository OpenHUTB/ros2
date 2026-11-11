"""双关节手臂模型：MuJoCo 刚体 + 6 条固定腱肌肉。

- 两条肌肉只跨肩、两条只跨肘、两条跨双关节（固定腱的 joint coef 同时非零）；
- 控制器只写肌肉激活（`ctrl`），关节力矩由 MuJoCo 按腱长/速度/力臂计算；
- 正运动学与文档一致：

      x = L1 cos q1 + L2 cos(q1+q2)
      z = -L1 sin q1 - L2 sin(q1+q2)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .config import MODEL, MUSCLES, Config


@dataclass
class ArmState:
    q1: float
    q2: float
    dq1: float
    dq2: float
    tip_x: float
    tip_z: float
    forces: np.ndarray
    activations: np.ndarray


class ArmModel:
    def __init__(self, cfg: Config, model_path: str | Path | None = None) -> None:
        path = Path(model_path) if model_path is not None else MODEL
        if not path.is_file():
            raise FileNotFoundError(f"找不到模型：{path}")
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.model.opt.timestep = float(cfg.sim.timestep)
        self.data = mujoco.MjData(self.model)
        self.sh = self.model.joint("shoulder")
        self.el = self.model.joint("elbow")
        self.tip = self.model.site("tip")
        self.actuator_ids = [self.model.actuator(n).id for n in MUSCLES]
        self.n_act = len(MUSCLES)
        # 末端坐标以肩关节为原点（模型基座抬高了 base_z）
        self.base_z = float(self.model.body("upper").pos[2])

    # ------------------------------------------------------------------
    @property
    def q1(self) -> float:
        return float(self.data.qpos[self.sh.qposadr[0]])

    @property
    def q2(self) -> float:
        return float(self.data.qpos[self.el.qposadr[0]])

    @property
    def dq1(self) -> float:
        return float(self.data.qvel[self.sh.dofadr[0]])

    @property
    def dq2(self) -> float:
        return float(self.data.qvel[self.el.dofadr[0]])

    @property
    def tip_position(self) -> np.ndarray:
        """末端在手臂平面内的 (x, z)（MuJoCo 的 y 轴垂直于该平面）。"""
        pos = self.data.site_xpos[self.tip.id]
        return np.array([pos[0], pos[2] - self.base_z], dtype=float)

    @property
    def activations(self) -> np.ndarray:
        return np.array([self.data.act[i] for i in self.actuator_ids])

    @property
    def forces(self) -> np.ndarray:
        return np.array([self.data.actuator_force[i] for i in self.actuator_ids])

    @property
    def joint_torques(self) -> np.ndarray:
        """两条肌肉（含双关节）对肩、肘产生的净力矩。"""
        return np.array([float(self.data.qfrc_actuator[self.sh.dofadr[0]]),
                         float(self.data.qfrc_actuator[self.el.dofadr[0]])])

    def reset(self, q1: float = 0.0, q2: float = 0.6) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.sh.qposadr[0]] = q1
        self.data.qpos[self.el.qposadr[0]] = q2
        self.data.ctrl[:] = 0.0
        self.data.act[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_activations(self, acts) -> None:
        acts = np.asarray(acts, dtype=float)
        if acts.shape != (self.n_act,):
            raise ValueError(f"激活维度必须是 ({self.n_act},)")
        for k, value in enumerate(np.clip(acts, 0.0, 1.0)):
            self.data.ctrl[self.actuator_ids[k]] = float(value)

    def step(self, n: int = 1) -> None:
        for _ in range(int(n)):
            mujoco.mj_step(self.model, self.data)

    def state(self) -> ArmState:
        tip = self.tip_position
        return ArmState(
            q1=self.q1, q2=self.q2, dq1=self.dq1, dq2=self.dq2,
            tip_x=float(tip[0]), tip_z=float(tip[1]),
            forces=self.forces, activations=self.activations,
        )

    # ------------------------------------------------------------------
    @staticmethod
    def forward_kinematics(q1: float, q2: float, l1: float,
                           l2: float) -> tuple[float, float]:
        x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
        z = -l1 * np.sin(q1) - l2 * np.sin(q1 + q2)
        return float(x), float(z)

    def forward_kinematics_current(self) -> tuple[float, float]:
        return self.forward_kinematics(self.q1, self.q2,
                                       self.cfg.model.upper_arm,
                                       self.cfg.model.forearm)
