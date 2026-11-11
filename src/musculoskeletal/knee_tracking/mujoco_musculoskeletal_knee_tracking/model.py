"""膝关节肌肉骨骼模型：MuJoCo 刚体 + Hill 肌肉 + PD 控制器 → 肌肉激活。

- 刚体与重力、限位由 MuJoCo 负责（`assets/knee.xml`）；
- 两条肌肉用 motor 执行器 + 恒定力臂 0.035 m 表示，`ctrl` 是肌力 [N]；
- 肌肉力 = 激活 a × F_max（力-长度/力-速度因子取 1，与文档默认参数一致）；
- 激活一阶动力学：上升 τ_act、下降 τ_deact。

控制律（与文档一致）：

    e   = θ_ref − θ
    τ   = kp·e − kd·θ̇
    u_flex = max(τ, 0) / τ_max,  u_ext = max(−τ, 0) / τ_max
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .config import ASSET, Config


@dataclass
class KneeState:
    theta_rad: float
    omega_rad_s: float
    flex: float
    ext: float
    tau_muscle: float


class ActivationDynamics:
    """两条肌肉的一阶激活动力学。"""

    def __init__(self, tau_act: float, tau_deact: float) -> None:
        if tau_act <= 0 or tau_deact <= 0:
            raise ValueError("时间常数必须 > 0")
        self.tau_act = float(tau_act)
        self.tau_deact = float(tau_deact)
        self.flex = 0.0
        self.ext = 0.0

    def reset(self) -> None:
        self.flex = 0.0
        self.ext = 0.0

    def update(self, u_flex: float, u_ext: float, dt: float) -> tuple[float, float]:
        if dt <= 0:
            raise ValueError("dt 必须 > 0")
        self.flex = self._step(self.flex, float(np.clip(u_flex, 0.0, 1.0)), dt)
        self.ext = self._step(self.ext, float(np.clip(u_ext, 0.0, 1.0)), dt)
        return self.flex, self.ext

    def _step(self, a: float, u: float, dt: float) -> float:
        tau = self.tau_act if u > a else self.tau_deact
        return float(np.clip(a + (u - a) * dt / tau, 0.0, 1.0))


class PDController:
    """PD → 肌肉激活的归一化映射，带关节限位保护。"""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.max_torque = float(cfg.ctrl.max_torque)
        self.theta_min = np.radians(cfg.ctrl.limit_low_deg)
        self.theta_max = np.radians(cfg.ctrl.limit_deg)

    def torque(self, theta: float, omega: float, theta_ref: float) -> float:
        e = theta_ref - theta
        return float(self.cfg.ctrl.kp * e - self.cfg.ctrl.kd * omega)

    def activations(self, theta: float, omega: float,
                    theta_ref: float) -> tuple[float, float, float]:
        tau = self.torque(theta, omega, theta_ref)
        if tau >= 0.0:
            u_flex, u_ext = min(tau / self.max_torque, 1.0), 0.0
        else:
            u_flex, u_ext = 0.0, min(-tau / self.max_torque, 1.0)
        # 到限位仍往限位方向用力 -> 松开对应肌肉
        margin = np.radians(0.01)
        if theta >= self.theta_max - margin and tau > 0.0:
            u_flex = 0.0
        if theta <= self.theta_min + margin and tau < 0.0:
            u_ext = 0.0
        return u_flex, u_ext, tau


class KneeSim:
    """MuJoCo 单关节仿真 + 肌肉驱动。"""

    def __init__(self, cfg: Config, model_path: str | Path | None = None) -> None:
        self.cfg = cfg
        path = Path(model_path) if model_path is not None else ASSET
        if not path.is_file():
            raise FileNotFoundError(f"找不到模型：{path}")
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.model.opt.timestep = float(cfg.sim.dt)
        self.data = mujoco.MjData(self.model)
        self.controller = PDController(cfg)
        self.activation = ActivationDynamics(cfg.muscle.tau_act, cfg.muscle.tau_deact)
        self.joint = self.model.joint("knee")
        self.qpos_idx = int(self.joint.qposadr[0])
        self.dof_idx = int(self.joint.dofadr[0])
        self.flex_id = self.model.actuator("flexor").id
        self.ext_id = self.model.actuator("extensor").id
        self.max_torque = cfg.muscle.f_max * cfg.muscle.moment_arm

    # ------------------------------------------------------------------
    @property
    def theta(self) -> float:
        return float(self.data.qpos[self.qpos_idx])

    @property
    def omega(self) -> float:
        return float(self.data.qvel[self.dof_idx])

    def reset(self, theta_deg: float) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_idx] = np.radians(theta_deg)
        self.data.qvel[self.dof_idx] = 0.0
        self.activation.reset()
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def state(self) -> KneeState:
        return KneeState(
            theta_rad=self.theta,
            omega_rad_s=self.omega,
            flex=self.activation.flex,
            ext=self.activation.ext,
            tau_muscle=self.max_torque * (self.activation.flex - self.activation.ext),
        )

    def step(self, theta_ref_rad: float) -> KneeState:
        """推进一个控制周期（dt）。"""
        u_flex, u_ext, _ = self.controller.activations(self.theta, self.omega,
                                                      theta_ref_rad)
        a_flex, a_ext = self.activation.update(u_flex, u_ext, self.cfg.sim.dt)
        self.data.ctrl[self.flex_id] = a_flex * self.cfg.muscle.f_max
        self.data.ctrl[self.ext_id] = a_ext * self.cfg.muscle.f_max
        mujoco.mj_step(self.model, self.data)
        return self.state()
