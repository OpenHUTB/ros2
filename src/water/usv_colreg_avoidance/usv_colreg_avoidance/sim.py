"""闭环仿真：船舶运动学 + LOS 制导 + COLREG 避让决策。

每条船都是独立的决策单元：自己观测周围目标、按规则决定让路还是保向、
自己操舵。没有任何集中调度，所以能看到两条船互相配合（或一条让一条保持）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from .colreg import (ColregConfig, Decision, Role, Vessel, cpa, decide,
                     required_turn, threat_cleared, wrap180, wrap360)


@dataclass
class ShipDynamics:
    """Nomoto 一阶操舵响应，参数取中小型无人艇的量级。"""

    k_rudder: float = 0.17    # 稳态转向率 / 舵角，(deg/s)/deg
    t_turn: float = 3.0       # 转向响应时间常数 (s)
    max_rudder: float = 35.0  # 最大舵角 (deg)
    kp: float = 1.5           # 航向误差 → 舵角，比例增益


@dataclass
class Controller:
    """单船的制导与避让状态机。

    避让一旦开始就锁定一个避让航向，直到威胁解除才回航线，
    避免 DCPA 刚回到安全值就把舵回过来、形成一连串小改动（Rule 8 禁止）。
    """

    goal: tuple[float, float]
    turn_rate: float = 0.0            # 当前转向率 r (deg/s)
    avoid_heading: float | None = None
    avoid_target: str | None = None


@dataclass
class Frame:
    t: float
    ships: list[tuple[float, float, float]]   # (x, y, heading)
    roles: list[str]
    situations: list[str]


@dataclass
class SimResult:
    name: str
    title: str
    title_en: str
    goal_names: list[str]
    initial: list[Vessel]
    goals: list[tuple[float, float]]
    frames: list[Frame]
    metrics: dict = field(default_factory=dict)


def los_heading(ship: Vessel, goal: tuple[float, float]) -> float:
    """指向目标航点的航向（Line-of-Sight 制导）。"""
    dx, dy = goal[0] - ship.x, goal[1] - ship.y
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return ship.heading
    return wrap360(math.degrees(math.atan2(dx, dy)))


def _choose_heading(ship: Vessel, ctrl: Controller, others: list[Vessel],
                    config: ColregConfig) -> tuple[float, Decision]:
    decision = decide(ship, others, config)

    # 已经锁定了避让目标：先看威胁是否解除
    if ctrl.avoid_target is not None:
        target = next((o for o in others if o.name == ctrl.avoid_target), None)
        if target is None or threat_cleared(ship, target, config):
            ctrl.avoid_target = None
            ctrl.avoid_heading = None
        else:
            return ctrl.avoid_heading, decision

    # 需要动作：锁定新的避让航向
    if decision.turn > 0.0 and decision.target is not None:
        ctrl.avoid_heading = wrap360(ship.heading + decision.turn)
        ctrl.avoid_target = decision.target
        return ctrl.avoid_heading, decision

    # 没有威胁，或处于直航船角色：回计划航线
    return los_heading(ship, ctrl.goal), decision


def _rudder(ship: Vessel, desired: float, dynamics: ShipDynamics) -> float:
    err = wrap180(desired - ship.heading)
    delta = dynamics.kp * err
    return max(-dynamics.max_rudder, min(dynamics.max_rudder, delta))


def run(scenario, config: ColregConfig | None = None,
        dynamics: ShipDynamics | None = None, dt: float = 0.2,
        avoid: bool = True) -> SimResult:
    """跑一个场景，返回逐帧状态与统计指标。

    avoid=False 时所有船一律直航，用来做「不避让会怎样」的对照基线。
    """
    config = config or ColregConfig()
    dynamics = dynamics or ShipDynamics()

    ships = [replace(v) for v in scenario.vessels]
    ctrls = [Controller(goal=g) for g in scenario.goals]
    initial = [replace(v) for v in ships]
    first_action = [None] * len(ships)
    arrived = [False] * len(ships)

    frames: list[Frame] = []
    t = 0.0
    while t <= scenario.t_max + 1e-9:
        desired: list[float] = []
        decisions: list[Decision] = []
        for i, ship in enumerate(ships):
            others = [s for j, s in enumerate(ships) if j != i]
            if avoid:
                heading, decision = _choose_heading(ship, ctrls[i], others, config)
            else:
                heading = los_heading(ship, ctrls[i].goal)
                decision = decide(ship, others, config)
            desired.append(heading)
            decisions.append(decision)
            if first_action[i] is None and decision.turn > 0.0:
                first_action[i] = t

        # 先算完所有船的舵角，再统一推进，避免「先动的船占便宜」
        rudders = [_rudder(ship, desired[i], dynamics) for i, ship in enumerate(ships)]
        for i, ship in enumerate(ships):
            if arrived[i]:
                continue
            ctrl = ctrls[i]
            if math.hypot(ship.x - ctrl.goal[0], ship.y - ctrl.goal[1]) < 60.0:
                # 到达航点就停在原地，免得转过头再掉回来
                arrived[i] = True
                ctrl.turn_rate = 0.0
                continue
            ctrl.turn_rate += (dynamics.k_rudder * rudders[i] - ctrl.turn_rate) / dynamics.t_turn * dt
            ship.heading = wrap360(ship.heading + ctrl.turn_rate * dt)
            h = math.radians(ship.heading)
            ship.x += ship.speed * math.sin(h) * dt
            ship.y += ship.speed * math.cos(h) * dt

        frames.append(Frame(
            t=t,
            ships=[(s.x, s.y, s.heading) for s in ships],
            roles=[d.role.value for d in decisions],
            situations=[d.situation.value for d in decisions],
        ))
        t += dt

        if all(math.hypot(ship.x - ctrl.goal[0], ship.y - ctrl.goal[1]) < 60.0
               for ship, ctrl in zip(ships, ctrls)):
            break

    return SimResult(
        name=scenario.name,
        title=scenario.title,
        title_en=scenario.title_en,
        goal_names=[s.name for s in ships],
        initial=initial,
        goals=list(scenario.goals),
        frames=frames,
        metrics=_metrics(frames, initial, first_action),
    )


def _metrics(frames: list[Frame], initial: list[Vessel], first_action: list) -> dict:
    n = len(initial)
    min_sep = math.inf
    min_sep_t = 0.0
    for frame in frames:
        for i in range(n):
            for j in range(i + 1, n):
                d = math.hypot(frame.ships[i][0] - frame.ships[j][0],
                               frame.ships[i][1] - frame.ships[j][1])
                if d < min_sep:
                    min_sep, min_sep_t = d, frame.t

    peak_turn = []
    for i in range(n):
        peak = max((abs(wrap180(f.ships[i][2] - initial[i].heading)) for f in frames), default=0.0)
        peak_turn.append(peak)

    hull = sum(v.length for v in initial[:2]) / 2.0 if n >= 2 else 0.0
    return {
        'min_separation': min_sep,
        'min_separation_time': min_sep_t,
        'collision': min_sep < hull,
        'peak_turn': peak_turn,
        'first_action': first_action,
        'duration': frames[-1].t if frames else 0.0,
    }
