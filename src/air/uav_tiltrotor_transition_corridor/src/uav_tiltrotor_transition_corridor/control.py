"""控制器：推力矢量分配 + 纵向速度/高度回路 + 走廊保护。

控制律的三层结构：

1. **外环**：高度 → 升降速度 → 垂向加速度；空速 → 纵向加速度。
2. **分配**：把期望合外力拆成旋翼推力与机翼升力。机翼拿"托不住的部分"，
   旋翼拿剩下的垂向分量与全部水平分量：

   .. math::

       F_z = W + m a_z - L,\\qquad F_x = D + m a_x
       T = \\sqrt{F_x^2 + F_z^2},\\qquad \\delta = \\operatorname{atan2}(F_z, F_x)

3. **执行机构**：推力一阶滞后（转速建立时间）、倾转角速率限制（舵机）、
   升力系数一阶滞后（俯仰回路等效），三者都在饱和后如实反映为掉高。

倾转角调度是可选的：默认交给分配律决定；也可以强制走一条速度-倾转角曲线，
用于演示"看起来合理、实际会掉出走廊"的调度。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .aero import G
from .corridor import Corridor
from .vehicle import VehicleSpec

__all__ = ["Controller", "ControllerConfig", "ActuatorState"]


@dataclass
class ControllerConfig:
    h_gain: float = 0.55          # 高度误差 → 升降速度指令，1/s
    vz_gain: float = 1.8          # 升降速度误差 → 垂向加速度，1/s
    u_gain: float = 0.85          # 空速误差 → 纵向加速度，(m/s²)/(m/s)
    a_z_limit: float = 3.0        # 垂向加速度指令限幅，m/s²
    corridor_margin_deg: float = 2.0
    corridor_guard: bool = True   # 是否启用走廊保护
    tilt_schedule: str | None = None  # None / "ramp" / "fixed:<deg>"


@dataclass
class ActuatorState:
    tilt_deg: float
    thrust: float
    cl: float


class Controller:
    def __init__(self, spec: VehicleSpec, corridor: Corridor,
                 cfg: ControllerConfig | None = None):
        self.spec = spec
        self.corridor = corridor
        self.cfg = cfg or ControllerConfig()

    # ------------------------------------------------------------------ 外环
    def vertical_accel_cmd(self, h: float, h_ref: float, w: float) -> float:
        cfg = self.cfg
        vz_cmd = max(-self.spec.vz_limit, min(self.spec.vz_limit, cfg.h_gain * (h_ref - h)))
        a_z = cfg.vz_gain * (vz_cmd - w)
        return max(-cfg.a_z_limit, min(cfg.a_z_limit, a_z))

    def forward_accel_cmd(self, u: float, u_ref: float) -> float:
        a_x = self.cfg.u_gain * (u_ref - u)
        return max(-self.spec.accel_limit, min(self.spec.accel_limit, a_x))

    # ------------------------------------------------------------------ 分配
    def allocate(self, v: float, rho: float, drag: float, lift: float,
                 a_x: float, a_z: float) -> tuple[float, float]:
        """返回 (推力指令, 倾转角指令/度)。"""
        m = self.spec.mass
        f_z = self.spec.weight + m * a_z - lift
        f_x = drag + m * a_x
        f_z = max(f_z, 0.0)          # 旋翼不产生向下的推力
        if f_x <= 1e-6:
            # 需要反向推力（减速过渡）。倾转角过 90° 时水平分量变负，
            # 前提是机型允许超过 90°（定距桨必须靠变距或反转才能做到）。
            if f_x < -1e-6 and self.spec.tilt_max_deg > 90.0:
                return math.hypot(f_x, f_z), math.degrees(math.atan2(f_z, f_x))
            return f_z, 90.0
        thrust = math.hypot(f_x, f_z)
        tilt = math.degrees(math.atan2(f_z, f_x))
        return thrust, tilt

    def schedule_tilt(self, u: float) -> float | None:
        """按空速给出参考倾转角；返回 ``None`` 表示不干预分配结果。"""
        mode = self.cfg.tilt_schedule
        if not mode:
            return None
        if mode.startswith("fixed:"):
            return float(mode.split(":", 1)[1])
        if mode in ("ramp", "ramp_corridor"):
            v0 = self.corridor.hover_speed_limit
            v1 = self.corridor.stall_speed * 1.15
            if u <= v0:
                return 90.0
            if u >= v1:
                return 0.0
            return 90.0 * (v1 - u) / (v1 - v0)
        raise ValueError(f"未知的 tilt_schedule：{mode}")

    # -------------------------------------------------------------- 单步更新
    def step(self, act: ActuatorState, *, v: float, u: float, w: float,
             h: float, h_ref: float, u_ref: float, rho: float,
             drag: float, lift: float, dt: float,
             thrust_scale: float = 1.0) -> tuple[ActuatorState, dict]:
        cfg = self.cfg
        a_z = self.vertical_accel_cmd(h, h_ref, w)
        a_x = self.forward_accel_cmd(u, u_ref)

        thrust_cmd, tilt_cmd = self.allocate(v, rho, drag, lift, a_x, a_z)
        t_max = self.corridor.thrust_max(v) * max(0.0, min(thrust_scale, 1.0))
        saturated = thrust_cmd > t_max
        thrust_cmd = min(thrust_cmd, t_max)

        tilt_cmd = max(0.0, min(self.spec.tilt_max_deg, tilt_cmd))
        sched = self.schedule_tilt(u)
        if sched is not None:
            tilt_cmd = sched
        if cfg.corridor_guard:
            tilt_cmd = self.corridor.clamp_tilt(v, tilt_cmd, cfg.corridor_margin_deg)

        # 倾转角速率限制
        rate = self.spec.rotor.tilt_rate_max_deg
        d_tilt = max(-rate * dt, min(rate * dt, tilt_cmd - act.tilt_deg))
        tilt = act.tilt_deg + d_tilt

        # 推力一阶滞后
        tau = self.spec.rotor.spool_tau
        alpha = 0.0 if tau <= 0 else min(1.0, dt / tau)
        thrust = act.thrust + alpha * (thrust_cmd - act.thrust)
        thrust = max(0.0, min(thrust, t_max))

        # 升力系数：由垂直平衡反解，带一阶滞后
        q_s = 0.5 * rho * v * v * self.spec.wing.area
        if q_s > 1e-6:
            # 机翼优先：先按"机翼独自承担全重"给配平升力，托不住的部分才交给旋翼。
            # 若反过来用旋翼先分，旋翼会一直保留上一拍的分担量，高速段白白多耗功率。
            cl_cmd = (self.spec.weight + self.spec.mass * a_z) / q_s
            cl_cmd = max(self.spec.wing.cl_min, min(self.spec.wing.cl_max, cl_cmd))
        else:
            cl_cmd = 0.0
        tl = self.spec.lift_lag
        beta = 0.0 if tl <= 0 else min(1.0, dt / tl)
        cl = act.cl + beta * (cl_cmd - act.cl)

        diag = {
            "a_x_cmd": a_x,
            "a_z_cmd": a_z,
            "thrust_cmd": thrust_cmd,
            "tilt_cmd": tilt_cmd,
            "saturated": saturated,
            "lift_margin": self.corridor.lift_margin_deg(v, tilt),
            "stalled": cl_cmd >= self.spec.wing.cl_max - 1e-9 and q_s > 1e-6,
        }
        return ActuatorState(tilt, thrust, cl), diag
