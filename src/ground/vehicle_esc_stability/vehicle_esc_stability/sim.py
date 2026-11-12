"""闭环仿真主循环：驾驶员输入 → ESC → 车辆动力学 → 指标统计。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .driver import PurePursuitDriver
from .dynamics import ForceBundle, RoadSpec, rk4_step
from .esc import Esc, EscStatus
from .params import ControlInput, EscConfig, TireSpec, VehicleSpec, VehicleState, WheelState
from .scenarios import Scenario

DT = 0.002
"""动力学积分步长，s。"""

CONTROL_DT = 0.02
"""ESC 与驾驶员决策周期，s（50 Hz）。"""

SAMPLE_DT = 0.05
"""记录一帧的间隔，s（20 Hz），用于画图与动画。"""

STOP_SPEED = 0.5
"""判定停车的速度阈值，m/s。用合速度 sqrt(u²+v²) 而不是纵向分量 u：

甩尾横滑时 u 会先过零而车身仍在以几十 km/h 侧滑，按 u 判会把这辆"还在动"的车
记成已停车，量出来的"制动距离"比真实值小一半以上。
"""

DEG = 180.0 / math.pi


@dataclass
class Frame:
    """一帧快照。"""

    t: float
    state: VehicleState
    steer: float
    throttle: float
    driver_brake: float
    wheel_brake: tuple[float, ...]
    esc: EscStatus
    wheels: tuple[WheelState, ...]
    ax: float = 0.0
    ay: float = 0.0


@dataclass
class SimResult:
    """一次仿真的完整结果。"""

    scenario: Scenario
    esc_enabled: bool
    frames: list[Frame] = field(default_factory=list)
    metrics: dict[str, float] = field(default_factory=dict)

    @property
    def title(self) -> str:
        return f"{self.scenario.title}{'（ESC 开）' if self.esc_enabled else '（ESC 关）'}"

    def series(self, key: str) -> list[float]:
        """按帧取一条曲线，方便画图。"""
        out = []
        for frame in self.frames:
            if key == "speed":
                out.append(frame.state.speed * 3.6)
            elif key == "yaw_rate":
                out.append(frame.state.r * DEG)
            elif key == "ref_yaw_rate":
                out.append(frame.esc.reference_yaw_rate * DEG)
            elif key == "sideslip":
                out.append(frame.state.sideslip * DEG)
            elif key == "steer":
                out.append(frame.steer * DEG)
            elif key == "y":
                out.append(frame.state.y)
            elif key == "x":
                out.append(frame.state.x)
            elif key == "esc":
                out.append(frame.esc.intensity)
            elif key == "brake_fl":
                out.append(frame.wheel_brake[0])
            elif key == "brake_fr":
                out.append(frame.wheel_brake[1])
            elif key == "brake_rl":
                out.append(frame.wheel_brake[2])
            elif key == "brake_rr":
                out.append(frame.wheel_brake[3])
            elif key == "beta_r":
                out.append(frame.state.sideslip * DEG)
            else:
                raise KeyError(key)
        return out

    def times(self) -> list[float]:
        return [frame.t for frame in self.frames]


def _cruise_throttle(state: VehicleState, target: float) -> float:
    """简易定速巡航：只用于把车速稳在工况初速附近。"""
    error = target - state.u
    return max(0.0, min(1.0, 0.55 * error))


def run(
    scenario: Scenario,
    esc_enabled: bool = True,
    esc_config: EscConfig | None = None,
    spec: VehicleSpec | None = None,
    tire: TireSpec | None = None,
) -> SimResult:
    """跑一个工况，返回轨迹与指标。"""
    spec = spec or VehicleSpec()
    tire = tire or TireSpec()
    config = EscConfig(enabled=esc_enabled) if esc_config is None else esc_config
    if esc_config is not None and esc_config.enabled != esc_enabled:
        config = EscConfig(**{**esc_config.__dict__, "enabled": esc_enabled})

    controller = Esc(config, spec)
    driver = (PurePursuitDriver(scenario.path, spec.wheelbase)
              if scenario.path is not None else None)
    state = VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        u=scenario.initial_speed,
        v=0.0,
        r=0.0,
    )

    result = SimResult(scenario=scenario, esc_enabled=esc_enabled)
    road: RoadSpec = scenario.road

    ax = 0.0
    ay = 0.0
    t = 0.0
    next_sample = 0.0
    next_control = 0.0
    control = ControlInput(steer=scenario.steer(0.0), driver_brake=scenario.brake(0.0))
    control.wheel_brake = [control.driver_brake] * 4
    bundle = ForceBundle()

    max_beta = 0.0
    max_yaw = 0.0
    max_offset = 0.0
    max_cross = 0.0
    yaw_error_sq = 0.0
    yaw_error_n = 0
    spin = False
    stop_distance = 0.0
    stop_time = 0.0
    stopped = False

    steps = int(round(scenario.duration / DT))
    for step in range(steps + 1):
        t = step * DT

        if t + 1e-9 >= next_control:
            driver_brake = scenario.brake(t)
            if driver is not None:
                steer = driver.steer(state.x, state.y, state.yaw, state.u, CONTROL_DT)
            else:
                steer = scenario.steer(t)
            control = ControlInput(
                steer=steer,
                driver_brake=driver_brake,
            )
            control.wheel_brake = [driver_brake] * 4
            control.throttle = (
                _cruise_throttle(state, scenario.initial_speed)
                if scenario.hold_speed and driver_brake < 0.02
                else 0.0
            )
            control = controller.step(state, control, road, CONTROL_DT)
            next_control += CONTROL_DT

        state, bundle = rk4_step(state, control, spec, tire, road, DT, ax, ay)
        ax, ay = bundle.ax, bundle.ay

        # 指标统计（每个积分步都算，避免采样漏掉峰值）
        beta = abs(state.sideslip) * DEG
        max_beta = max(max_beta, beta)
        max_yaw = max(max_yaw, abs(state.r) * DEG)
        max_offset = max(max_offset, abs(state.y))
        if driver is not None:
            _, cross, _ = driver.path.project(state.x, state.y)
            max_cross = max(max_cross, abs(cross))
        yaw_error_sq += (controller.status.yaw_rate_error * DEG) ** 2
        yaw_error_n += 1
        if beta > 20.0 or abs(state.r) > 1.6:
            spin = True
        if not stopped and state.speed < STOP_SPEED and scenario.brake(t) > 0.1:
            stop_distance = state.x
            stop_time = t
            stopped = True

        if t + 1e-9 >= next_sample:
            result.frames.append(
                Frame(
                    t=t,
                    state=VehicleState(state.x, state.y, state.yaw, state.u, state.v, state.r),
                    steer=control.steer,
                    throttle=control.throttle,
                    driver_brake=control.driver_brake,
                    wheel_brake=tuple(control.wheel_brake),
                    esc=controller.status,
                    wheels=bundle.wheels,
                    ax=bundle.ax,
                    ay=bundle.ay,
                )
            )
            next_sample += SAMPLE_DT

    stats = controller.stats
    result.metrics = {
        "max_beta_deg": max_beta,
        "max_yaw_rate_dps": max_yaw,
        "yaw_error_rms_dps": math.sqrt(yaw_error_sq / max(1, yaw_error_n)),
        "max_lateral_m": max_offset,
        "max_cross_track_m": max_cross,
        "final_offset_m": state.y,
        "final_heading_deg": state.yaw * DEG,
        "final_speed_kph": state.speed * 3.6,
        "distance_m": state.x,
        "stop_distance_m": stop_distance,
        "stop_time_s": stop_time,
        "stopped": 1.0 if stopped else 0.0,
        "spin": 1.0 if spin else 0.0,
        "esc_active_s": stats["active_time"],
        "esc_peak": stats["peak_intensity"],
        "esc_activations": stats["activations"],
    }
    return result


def run_pair(scenario: Scenario, **kwargs) -> tuple[SimResult, SimResult]:
    """同一工况跑两遍：ESC 关 / ESC 开。"""
    return run(scenario, esc_enabled=False, **kwargs), run(scenario, esc_enabled=True, **kwargs)
