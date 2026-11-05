"""闭环仿真：ATO/ATP 给指令，车按一阶滞后的执行机构响应，逐帧记录状态。

执行机构用一阶滞后是刻意的简化——真实列车的制动力不是瞬间建立，
空走时间加力上升大约 0.5~1 s。不做这个滞后，停车误差会小到不真实，
曲线留的那点余量也看不出作用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .ato import AtpState, Ato, AtoConfig
from .profile import SpeedProfile, atp_permit_curve, build_profile
from .scenarios import Scenario
from .train import TrainSpec, energy_kwh, grade_at


@dataclass
class Frame:
    t: float
    x: float
    v: float
    accel: float        # 实际作用在车上的加速度
    cmd: float          # 控制器给出的加速度指令
    atp: str
    note: str = ''


@dataclass
class SimResult:
    name: str
    title: str
    title_en: str
    frames: list[Frame]
    profile: SpeedProfile
    atp_profile: SpeedProfile
    stop_x: float
    segments: list = field(default_factory=list)
    metrics: dict = field(default_factory=dict)
    note: str = ''

    @property
    def final_x(self) -> float:
        return self.frames[-1].x

    @property
    def stop_error(self) -> float:
        return self.final_x - self.stop_x


def _manual_command(scenario: Scenario, spec: TrainSpec, x: float, v: float) -> float:
    """人工驾驶：不懂曲线，只按限速跑，到了指定距离才踩制动。"""
    remaining = scenario.stop_x - x
    if remaining > scenario.manual_brake_distance:
        limit = scenario.limit_at(x)
        accel = 1.2 * (limit - v)
        upper = max(0.0, spec.accel_capability(max(v, 0.5), grade_at(scenario.segments, x)))
        return max(-spec.service_brake_decel, min(upper, accel))
    return -spec.service_brake_decel


def run(scenario: Scenario, spec: TrainSpec | None = None,
        config: AtoConfig | None = None, dt: float = 0.02,
        dx: float = 1.0, t_max: float = 400.0) -> SimResult:
    spec = spec or TrainSpec()
    config = config or AtoConfig()

    profile = build_profile(scenario.segments, scenario.stop_x, spec,
                            a_plan=config.a_plan, dx=dx,
                            limit_margin=config.limit_margin_kph / 3.6)
    atp_profile = atp_permit_curve(scenario.segments, scenario.stop_x, spec, dx=dx)
    ato = Ato(profile, spec, config, atp_profile=atp_profile)

    x, v, applied = 0.0, 0.0, 0.0
    t = 0.0
    frames: list[Frame] = []
    energy = 0.0
    atp_time = 0.0
    atp_events = 0
    last_atp = AtpState.NORMAL
    overspeed_time = 0.0

    while t <= t_max:
        atp = ato.supervise(x, v)
        if scenario.manual_brake_distance is not None:
            cmd = _manual_command(scenario, spec, x, v)
            note = '人工驾驶'
            if atp is AtpState.INTERVENTION:
                cmd = -spec.emergency_brake_decel
                note = 'ATP 触发紧急制动'
        else:
            command = ato.command(x, v)
            cmd, note = command.accel, command.note

        if atp is not AtpState.NORMAL:
            # 停车点附近允许速度趋近 0，那一段的越界没有意义，会和最后几米的
            # 正常蠕动混在一起，所以与 over_permit 用同一条口径：距停车点 100 m 以外才计。
            if x < scenario.stop_x - 100.0:
                atp_time += dt
            if last_atp is not AtpState.INTERVENTION and atp is AtpState.INTERVENTION:
                atp_events += 1
        last_atp = atp

        limit = scenario.limit_at(x)
        if v > limit + 0.15:
            overspeed_time += dt

        frames.append(Frame(t=t, x=x, v=v, accel=applied, cmd=cmd,
                            atp=atp.value, note=note))

        # 执行机构一阶滞后
        tau = max(0.05, spec.brake_build_up)
        applied += (cmd - applied) * min(1.0, dt / tau)

        if applied > 0.0:
            energy += energy_kwh(spec, v, dt)

        v_next = v + applied * dt
        x_next = x + v * dt + 0.5 * applied * dt * dt
        if v_next < 0.0:
            # 停住时把残余的负速度截掉，避免在停车点附近来回抖
            v_next = 0.0
        x, v = x_next, v_next

        if v <= 0.0 and applied <= 0.0 and t > 1.0:
            frames.append(Frame(t=t + dt, x=x, v=0.0, accel=applied, cmd=cmd,
                                atp=atp.value, note='停车'))
            break
        t += dt

    # 停车点附近允许速度本身趋近于 0，拿那一段算"超出允许速度"没有意义，
    # 会把最后几米的正常蠕动也算成越界，所以只统计距停车点 100 m 以外的部分。
    over_permit = [(f.v - atp_profile.envelope_at(f.x)) * 3.6
                   for f in frames if f.x < scenario.stop_x - 100.0]

    metrics = {
        'run_time': frames[-1].t,
        'max_speed_kph': max(f.v for f in frames) * 3.6,
        'stop_error': x - scenario.stop_x,
        'energy_kwh': energy,
        'atp_time': atp_time,
        'atp_events': atp_events,
        'overspeed_time': overspeed_time,
        'max_over_permit': max(over_permit) if over_permit else 0.0,
    }
    return SimResult(name=scenario.name, title=scenario.title, title_en=scenario.title_en,
                     frames=frames, profile=profile, atp_profile=atp_profile,
                     stop_x=scenario.stop_x, segments=list(scenario.segments),
                     metrics=metrics, note=scenario.note)
