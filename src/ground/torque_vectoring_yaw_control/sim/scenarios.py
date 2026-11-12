"""闭环场景：阶跃转向、正弦扫掠、单电机失效、超附着极限。

前三个工况都留在**附着圆以内**（峰值附着利用率 ≤ 0.51，侧偏角 < 5°），
这样"横摆跟踪改善"比的是控制质量，而不是"谁甩得慢一点"。

``over_limit`` 是刻意越界的反例：30 m/s 打 5° 需要的侧向加速度接近 3 g，
路面根本给不出来，任何分配都救不回来。留着它是为了说明**扭矩矢量的权限有边界**，
不是所有失稳都能靠调力矩解决。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from torque_vectoring_yaw_control.control import ControllerConfig
from torque_vectoring_yaw_control.world import SimConfig, SimResult, VehicleSim

__all__ = ["Scenario", "SCENARIOS", "build", "run", "steer_fn"]


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    description: str
    u0: float = 20.0
    duration: float = 6.0
    mode: str = "step"            # step | sine
    steer_deg: float = 4.0        # 阶跃幅值 / 正弦幅值
    steer_start: float = 1.0
    steer_ramp: float = 30.0      # 阶跃的斜坡速率，°/s（低于执行机构上限）
    freq_hz: float = 0.5          # 正弦频率
    mu_left: float = 0.95
    mu_right: float = 0.95
    dead_wheels: tuple[int, ...] = ()
    ctrl: ControllerConfig = field(default_factory=ControllerConfig)


def steer_fn(sc: Scenario) -> Callable[[float], float]:
    """由场景生成方向盘转角输入（°）。"""
    if sc.mode == "sine":
        def f(t: float) -> float:
            if t < sc.steer_start:
                return 0.0
            # 0.5 s 渐入，避免正弦从 0 起步时导数跳变激起虚假瞬态
            ramp = min(1.0, (t - sc.steer_start) / 0.5)
            return sc.steer_deg * ramp * math.sin(2 * math.pi * sc.freq_hz * (t - sc.steer_start))
        return f

    def f(t: float) -> float:
        if t < sc.steer_start:
            return 0.0
        return min(sc.steer_deg, sc.steer_ramp * (t - sc.steer_start))
    return f


SCENARIOS: dict[str, Scenario] = {
    "step_steer": Scenario(
        key="step_steer", title="阶跃转向 4°",
        description="20 m/s 打 4°，附着裕度充足。差别全在瞬态：无扭矩矢量时横摆率靠轮胎侧偏力"
                    "慢慢建起来，有前馈则几乎跟着参考走。",
        u0=20.0, duration=6.0, steer_deg=4.0,
    ),
    "sine_steer": Scenario(
        key="sine_steer", title="正弦扫掠 5°",
        description="20 m/s、0.5 Hz 连续左右打方向，考察相位滞后与幅值衰减——这是 ESC 台架"
                    "最常用的评价工况。",
        u0=20.0, duration=8.0, mode="sine", steer_deg=5.0, freq_hz=0.5,
    ),
    "motor_failure": Scenario(
        key="motor_failure", title="右后电机失效",
        description="22 m/s 打 4°，右后轮额度归零。三轮驱动会自发产生一个把车往右拽的横摆力矩，"
                    "只能靠另外三轮重新分配来抵消。车速不能再往上堆：24 m/s 开始前后轴侧向力吃满，"
                    "附着椭圆留给纵向的额度趋近 0，那时任何分配都发不出驱动力。",
        u0=22.0, duration=6.0, steer_deg=4.0, dead_wheels=(3,),
    ),
    "over_limit": Scenario(
        key="over_limit", title="超附着极限 5°",
        description="30 m/s 打 5°，需要的侧向加速度接近 3 g。轮胎全部饱和后横摆力矩互相抵消，"
                    "车身横摆率会一直保持住、侧偏角持续增大——这是失控，扭矩矢量只能减缓恶化。",
        u0=30.0, duration=6.0, steer_deg=5.0,
    ),
}


def build(key: str, enable_tv: bool, u_ref: float | None = None) -> VehicleSim:
    sc = SCENARIOS[key]
    ctrl = ControllerConfig(
        kp=sc.ctrl.kp, ki=sc.ctrl.ki, i_limit=sc.ctrl.i_limit,
        moment_limit=sc.ctrl.moment_limit, speed_kp=sc.ctrl.speed_kp,
        speed_ki=sc.ctrl.speed_ki, enabled=enable_tv,
    )
    sim = VehicleSim(
        None,
        SimConfig(
            duration=sc.duration, u0=sc.u0, enable_tv=enable_tv,
            mu_left=sc.mu_left, mu_right=sc.mu_right, dead_wheels=sc.dead_wheels,
        ),
        ctrl_cfg=ctrl,
        steer_fn=steer_fn(sc),
    )
    if u_ref is not None:
        sim.u_ref = u_ref
    return sim


def run(key: str, enable_tv: bool) -> SimResult:
    return build(key, enable_tv).run()
