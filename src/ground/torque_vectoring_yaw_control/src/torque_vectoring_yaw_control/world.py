"""三自由度车辆动力学（纵向 + 侧向 + 横摆）与闭环仿真。

车身方程（车身坐标系，``u`` 纵向、``v`` 侧向、``r`` 横摆）：

.. math::

    m(\\dot u - v r) &= F_x - F_{yf}\\sin\\delta - F_{drag} \\\\
    m(\\dot v + u r) &= F_{yf}\\cos\\delta + F_{yr} \\\\
    I_z \\dot r &= a F_{yf}\\cos\\delta - b F_{yr} + M_{z,tv}

`M_{z,tv}` 就是四个轮子纵向力之差产生的横摆力矩——扭矩矢量的全部作用都从这里进来。
轮胎侧偏力用线性段并在 ``μFz`` 处截断，所以推头（饱和）这件事会自然发生，
不需要额外编一个"understeer"开关。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .control import ControllerConfig, YawController
from .vehicle import VehicleSpec, WheelState

__all__ = ["SimConfig", "SimResult", "VehicleSim"]


@dataclass
class SimConfig:
    duration: float = 8.0
    dt: float = 0.005
    u0: float = 25.0
    enable_tv: bool = True
    mu_left: float = 0.95
    mu_right: float = 0.95
    drive_force_limit: float = 0.0   # 驱动总力上限（0 表示不限），N
    brake_force_limit: float = 0.0   # 制动总力上限（0 表示不限），N
    dead_wheels: tuple[int, ...] = ()  # 失效电机（0/1/2/3 = 左前/右前/左后/右后）


@dataclass
class SimResult:
    t: np.ndarray
    u: np.ndarray
    v: np.ndarray
    r: np.ndarray
    r_ref: np.ndarray
    ax: np.ndarray
    ay: np.ndarray
    fx: np.ndarray          # (n, 4)
    mz_cmd: np.ndarray
    mz_actual: np.ndarray
    steer: np.ndarray
    x: np.ndarray
    y: np.ndarray
    utilization: np.ndarray
    saturated: np.ndarray

    @property
    def yaw_rms_error(self) -> float:
        return float(np.sqrt(np.mean((self.r - self.r_ref) ** 2)))

    @property
    def max_sideslip_deg(self) -> float:
        return float(np.degrees(np.max(np.abs(np.arctan2(self.v, np.maximum(self.u, 1.0))))))


class VehicleSim:
    """闭环仿真：转向输入 → 参考横摆率 → 扭矩矢量分配 → 车辆响应。"""

    def __init__(self, spec: VehicleSpec | None = None,
                 cfg: SimConfig | None = None,
                 ctrl_cfg: ControllerConfig | None = None,
                 steer_fn: Callable[[float], float] | None = None):
        self.spec = spec or VehicleSpec()
        self.cfg = cfg or SimConfig()
        if ctrl_cfg is None:
            ctrl_cfg = ControllerConfig(enabled=self.cfg.enable_tv)
        self.controller = YawController(self.spec, ctrl_cfg)
        self.steer_fn = steer_fn or (lambda t: 0.0)

        self.t = 0.0
        self.u = self.cfg.u0
        self.v = 0.0
        self.r = 0.0
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        self.steer = 0.0
        self.u_ref = self.cfg.u0

    # ------------------------------------------------------------------ 轮胎
    def _lateral_forces(self, delta: float, ax: float) -> tuple[float, float, float, float]:
        return self.spec.axle_forces(self.u, self.v, self.r, delta, ax)

    def _wheel_caps(self, fy_f: float, fy_r: float, fz_f: float, fz_r: float) -> list[float]:
        spec = self.spec
        caps = []
        for i, (load, fy_axle) in enumerate(((fz_f, fy_f), (fz_f, fy_f),
                                            (fz_r, fy_r), (fz_r, fy_r))):
            mu = self.cfg.mu_left if i in (0, 2) else self.cfg.mu_right
            cap = spec.force_cap(load / 2.0, fy_axle / 2.0, mu)
            if i in self.cfg.dead_wheels:
                cap = 0.0
            caps.append(min(cap, spec.motor_force_max()))
        return caps

    # ------------------------------------------------------------------ 单步
    def step(self) -> dict:
        spec, cfg, dt = self.spec, self.cfg, self.cfg.dt

        # 转向执行机构：速率限制
        steer_cmd = math.radians(self.steer_fn(self.t))
        rate = math.radians(spec.steer_rate_deg)
        self.steer += max(-rate * dt, min(rate * dt, steer_cmd - self.steer))

        fy_f, fy_r, fz_f, fz_r = self._lateral_forces(self.steer, 0.0)
        caps = self._wheel_caps(fy_f, fy_r, fz_f, fz_r)

        out = self.controller.step(
            u=self.u, r=self.r, steer=self.steer, u_ref=self.u_ref,
            ax_ff=0.0, caps=caps, dt=dt,
        )
        fx = list(out.fx)
        if cfg.drive_force_limit > 0:
            total = sum(fx)
            if total > cfg.drive_force_limit:
                scale = cfg.drive_force_limit / total
                fx = [f * scale for f in fx]
        if cfg.brake_force_limit > 0:
            total = sum(fx)
            if total < -cfg.brake_force_limit:
                scale = cfg.brake_force_limit / -total
                fx = [f * scale for f in fx]

        arms = WheelState.moment_arms(spec.track)
        mz_tv = sum(f * m for f, m in zip(fx, arms))

        fx_total = sum(fx) - fy_f * math.sin(self.steer) - spec.cd_a * self.u * abs(self.u)
        fy_total = fy_f * math.cos(self.steer) + fy_r

        ax = fx_total / spec.mass
        ay = fy_total / spec.mass
        u_dot = ax + self.v * self.r
        v_dot = ay - self.u * self.r
        r_dot = (spec.a * fy_f * math.cos(self.steer) - spec.b * fy_r + mz_tv) / spec.iz

        self.u += u_dot * dt
        self.v += v_dot * dt
        self.r += r_dot * dt
        self.psi += self.r * dt
        self.x += (self.u * math.cos(self.psi) - self.v * math.sin(self.psi)) * dt
        self.y += (self.u * math.sin(self.psi) + self.v * math.cos(self.psi)) * dt
        self.t += dt

        return {
            "t": self.t, "u": self.u, "v": self.v, "r": self.r, "ax": ax, "ay": ay,
            "r_ref": out.yaw_rate_ref, "fx": fx, "mz_cmd": out.moment_cmd,
            "mz_actual": mz_tv, "steer": self.steer,
            "utilization": out.utilization, "saturated": float(out.saturated),
            "x": self.x, "y": self.y,
        }

    def run(self) -> SimResult:
        n = int(round(self.cfg.duration / self.cfg.dt))
        keys = ["t", "u", "v", "r", "ax", "ay", "r_ref", "mz_cmd", "mz_actual",
                "steer", "utilization", "saturated", "x", "y"]
        buf: dict[str, list] = {k: [] for k in keys}
        fx_buf: list[list[float]] = []
        for _ in range(n):
            d = self.step()
            for k in keys:
                buf[k].append(d[k])
            fx_buf.append(d["fx"])
        arr = {k: np.asarray(v, dtype=float) for k, v in buf.items()}
        return SimResult(
            t=arr["t"], u=arr["u"], v=arr["v"], r=arr["r"], r_ref=arr["r_ref"],
            ax=arr["ax"], ay=arr["ay"], fx=np.asarray(fx_buf), mz_cmd=arr["mz_cmd"],
            mz_actual=arr["mz_actual"], steer=arr["steer"],
            x=arr["x"], y=arr["y"],
            utilization=arr["utilization"], saturated=arr["saturated"],
        )
