"""速度曲线生成。

ATO 的核心就是这条曲线：在给定的线路限速和停车点下，先反推"再往前必须
减速"的制动脉络，再从起点按牵引能力正向积分，两者取小就得到一条既能跑
得起来、又一定停得住的目标速度。

两个细节容易漏：

1. 制动减速度按 brake_margin 打折（默认 1.25）。实际制动有 0.8 s 左右的力
   上升时间（空走时间），控制周期内也追不上目标速度，留出余量后规划曲线
   才不会算得太乐观。
2. 反推时要把基本阻力和坡道分力算进去。12‰ 下坡的分力能抵掉约 0.09 m/s²
   的制动力，按平直道算出来的制动点会偏晚——这正是坡道场景要暴露的问题。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .train import Segment, TrainSpec, grade_at, speed_limit_at


@dataclass
class SpeedProfile:
    """等间距里程网格上的几条曲线，单位均为 m 与 m/s。"""

    dx: float
    x: list[float] = field(default_factory=list)
    v_limit: list[float] = field(default_factory=list)
    v_envelope: list[float] = field(default_factory=list)
    v_ref: list[float] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.x)

    @property
    def stop_x(self) -> float:
        return self.x[-1] if self.x else 0.0

    def index(self, x: float) -> int:
        """里程换算成网格下标，越界时夹到两端。"""
        if not self.x:
            return 0
        i = int(round(x / self.dx))
        return max(0, min(len(self.x) - 1, i))

    def ref_at(self, x: float) -> float:
        return self.v_ref[self.index(x)]

    def limit_at(self, x: float) -> float:
        return self.v_limit[self.index(x)]

    def envelope_at(self, x: float) -> float:
        return self.v_envelope[self.index(x)]


def _grid(stop_x: float, dx: float) -> list[float]:
    n = int(math.floor(stop_x / dx)) + 1
    xs = [i * dx for i in range(n)]
    if xs[-1] < stop_x - 1e-9:
        xs.append(stop_x)
    return xs


def _effective_decel(spec: TrainSpec, v: float, grade: float, a_base: float) -> float:
    """制动时实际能拿到的减速度：基本阻力和坡道分力都会叠加到制动力上。"""
    extra = spec.resistance(v) + spec.grade_force(grade)
    a_eff = a_base + extra / (spec.rotary_factor * spec.mass)
    return max(a_eff, 0.05)


def _backward_envelope(xs: list[float], segments: list[Segment], spec: TrainSpec,
                       a_base: float, limit_margin: float = 0.0) -> list[float]:
    """从停车点倒推：每个位置最晚能有多快，还能在停车点前停住。

    limit_margin 是限速裕量（m/s）。限速是台阶函数，包络如果正好贴着台阶
    边界，进限速区时因为制动力上升有滞后，车头会短暂压过限速线。留一点
    裕量后包络本身就低于台阶，画出来的曲线也好读。
    """
    v_limit = [speed_limit_at(segments, x) for x in xs]
    v_env = [0.0] * len(xs)
    for i in range(len(xs) - 2, -1, -1):
        step = xs[i + 1] - xs[i]
        v_next = v_env[i + 1]
        a_eff = _effective_decel(spec, v_next, grade_at(segments, xs[i]), a_base)
        reachable = math.sqrt(v_next ** 2 + 2.0 * a_eff * step)
        v_env[i] = min(max(v_limit[i] - limit_margin, 0.0), reachable)
    return v_limit, v_env


def build_profile(segments: list[Segment], stop_x: float, spec: TrainSpec,
                  a_plan: float = 0.8, dx: float = 1.0,
                  limit_margin: float = 0.0) -> SpeedProfile:
    """生成 0 → stop_x 的目标速度曲线。"""
    if stop_x <= 0.0:
        raise ValueError('停车点里程必须为正')
    xs = _grid(stop_x, dx)
    v_limit, v_env = _backward_envelope(xs, segments, spec, a_plan, limit_margin)

    # 正向：按实际牵引能力积分，能跑多快就跑多快，但不超过包络
    v_ref = [0.0] * len(xs)
    for i in range(len(xs) - 1):
        step = xs[i + 1] - xs[i]
        grade = grade_at(segments, xs[i])
        a_cap = max(0.0, spec.accel_capability(max(v_ref[i], 0.2), grade))
        v_next = math.sqrt(v_ref[i] ** 2 + 2.0 * a_cap * step)
        v_ref[i + 1] = min(v_env[i + 1], v_next)

    return SpeedProfile(dx=dx, x=xs, v_limit=v_limit, v_envelope=v_env, v_ref=v_ref)


def braking_envelope_curve(segments: list[Segment], stop_x: float, spec: TrainSpec,
                           a_plan: float = 0.8, dx: float = 1.0) -> SpeedProfile:
    """只算包络不算牵引，用于把限速与制动曲线单独画出来做对照。"""
    xs = _grid(stop_x, dx)
    v_limit, v_env = _backward_envelope(xs, segments, spec, a_plan)
    return SpeedProfile(dx=dx, x=xs, v_limit=v_limit, v_envelope=v_env, v_ref=list(v_env))


def atp_permit_curve(segments: list[Segment], stop_x: float, spec: TrainSpec,
                     dx: float = 1.0) -> SpeedProfile:
    """ATP 的允许速度：用满紧急制动能力反推的安全上限。

    它比 ATO 的规划曲线更"贴边"（ATO 用打折的 0.8 m/s²，ATP 用 1.2 m/s²），
    这样正常运行时 ATO 永远在 ATP 之下，一旦司机或外部指令把速度顶上去，
    ATP 才介入——这才是监督该有的层次。
    """
    return braking_envelope_curve(segments, stop_x, spec,
                                  a_plan=spec.emergency_brake_decel, dx=dx)
