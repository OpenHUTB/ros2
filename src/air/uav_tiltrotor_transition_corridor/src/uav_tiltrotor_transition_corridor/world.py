"""纵向三自由度仿真：把气动、旋翼、控制器、执行机构与剖面串起来。

状态量是 ``(x, h, u, w)`` —— 水平位置、高度、地速水平/垂向分量。
气动力在**空速系**里算（相对风），加速度积分在地面系里，所以侧风会自动
体现在阻力与升力上，而不是简单叠加一个扰动量。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .aero import G
from .control import ActuatorState, Controller, ControllerConfig
from .corridor import Corridor
from .pilot import PilotConfig, TransitionPilot
from .vehicle import VehicleSpec

__all__ = ["TiltrotorSim", "SimResult"]

WindFn = Callable[[float, float], tuple[float, float]]


def _no_wind(t: float, h: float) -> tuple[float, float]:
    return 0.0, 0.0


@dataclass
class SimResult:
    t: np.ndarray
    h: np.ndarray
    u: np.ndarray
    w: np.ndarray
    v: np.ndarray
    tilt: np.ndarray
    thrust: np.ndarray
    cl: np.ndarray
    power: np.ndarray
    lift_margin: np.ndarray
    mode: list[str]
    aborted: bool
    abort_reason: str | None
    energy_wh: float

    @property
    def max_sink(self) -> float:
        return float(max(0.0, -self.w.min()))

    @property
    def alt_error(self) -> float:
        """相对起始高度的最大偏离，m。"""
        return float(np.max(np.abs(self.h - self.h[0])))

    @property
    def min_lift_margin(self) -> float:
        """整个剖面里最差的升力裕度（度）。小于 0 说明曾经托不住。"""
        return float(self.lift_margin.min())

    def transition_time(self, v_target: float) -> float | None:
        """从悬停加速到 ``v_target`` 所用的时间，s。"""
        idx = np.where(self.v >= v_target)[0]
        if idx.size == 0:
            return None
        return float(self.t[idx[0]] - self.t[0])

    def summary(self) -> dict[str, float | bool | str | None]:
        return {
            "aborted": self.aborted,
            "abort_reason": self.abort_reason,
            "energy_wh": round(self.energy_wh, 3),
            "alt_error_m": round(self.alt_error, 3),
            "min_height_m": round(float(self.h.min()), 3),
            "max_sink_mps": round(self.max_sink, 3),
            "min_lift_margin_deg": round(self.min_lift_margin, 2),
            "peak_power_w": round(float(self.power.max()), 1),
            "peak_speed_mps": round(float(self.v.max()), 2),
        }


class TiltrotorSim:
    def __init__(
        self,
        spec: VehicleSpec | None = None,
        cfg: ControllerConfig | None = None,
        pilot_cfg: PilotConfig | None = None,
        corridor_alt: float = 0.0,
        wind: WindFn | None = None,
        thrust_scale: Callable[[float], float] | None = None,
        dt: float = 0.01,
    ):
        self.spec = spec or VehicleSpec()
        self.corridor = Corridor(self.spec, alt_m=corridor_alt)
        self.controller = Controller(self.spec, self.corridor, cfg)
        self.pilot = TransitionPilot(pilot_cfg)
        self.wind = wind or _no_wind
        self.thrust_scale = thrust_scale or (lambda t: 1.0)
        self.dt = dt

        self.t = 0.0
        self.x = 0.0
        self.h = (pilot_cfg or PilotConfig()).hover_alt
        self.u = 0.0
        self.w = 0.0
        self.act = ActuatorState(tilt_deg=90.0, thrust=self.spec.weight, cl=0.0)
        self.energy_j = 0.0

    # ------------------------------------------------------------------ 积分
    def step(self) -> dict:
        dt = self.dt
        spec = self.spec
        rho = spec.density(self.h)
        wx, wz = self.wind(self.t, self.h)
        u_r, w_r = self.u - wx, self.w - wz
        v = math.hypot(u_r, w_r)

        cl = self.act.cl
        lift = 0.5 * rho * v * v * spec.wing.area * cl
        drag = spec.drag_at(v, cl, rho)

        h_ref, u_ref = self.pilot.update(self.t, self.h, self.u, self.w, dt)
        self.act, diag = self.controller.step(
            self.act, v=v, u=self.u, w=self.w, h=self.h, h_ref=h_ref, u_ref=u_ref,
            rho=rho, drag=drag, lift=lift, dt=dt, thrust_scale=self.thrust_scale(self.t),
        )

        d = math.radians(self.act.tilt_deg)
        thrust_vec_x = self.act.thrust * math.cos(d)
        thrust_vec_z = self.act.thrust * math.sin(d)

        drag_x = drag * u_r / v if v > 1e-6 else 0.0
        drag_z = drag * w_r / v if v > 1e-6 else 0.0

        a_u = (thrust_vec_x - drag_x) / spec.mass
        a_w = (thrust_vec_z + lift - spec.weight - drag_z) / spec.mass

        self.u += a_u * dt
        self.w += a_w * dt
        self.h += self.w * dt
        self.x += self.u * dt
        self.t += dt

        power = spec.rotor.shaft_power(self.act.thrust, v * abs(math.cos(d)), rho)
        self.energy_j += power * dt

        diag.update({
            "t": self.t, "x": self.x, "h": self.h, "u": self.u, "w": self.w,
            "v": v, "tilt": self.act.tilt_deg, "thrust": self.act.thrust,
            "cl": self.act.cl, "power": power, "mode": self.pilot.mode,
        })
        return diag

    def run(self, duration: float) -> SimResult:
        n = int(round(duration / self.dt))
        keys = ["t", "h", "u", "w", "v", "tilt", "thrust", "cl", "power", "lift_margin", "mode"]
        buf: dict[str, list] = {k: [] for k in keys}
        for _ in range(n):
            d = self.step()
            for k in keys:
                if k == "lift_margin":
                    buf[k].append(d["lift_margin"])
                elif k == "mode":
                    buf[k].append(d["mode"])
                else:
                    buf[k].append(d[k])
        arr = {k: np.asarray(v_, dtype=float) for k, v_ in buf.items() if k != "mode"}
        return SimResult(
            t=arr["t"], h=arr["h"], u=arr["u"], w=arr["w"], v=arr["v"],
            tilt=arr["tilt"], thrust=arr["thrust"], cl=arr["cl"],
            power=arr["power"], lift_margin=arr["lift_margin"], mode=buf["mode"],
            aborted=self.pilot.mode == "abort",
            abort_reason=self.pilot.abort_reason,
            energy_wh=self.energy_j / 3600.0,
        )
