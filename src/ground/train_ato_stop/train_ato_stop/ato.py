"""ATO 速度跟踪与 ATP 超速监督。

分两层是有意的：
- ATO 负责"怎么开"：贴着目标曲线跑，最后一段按剩余距离反解允许速度，保证停在标尺内。
- ATP 负责"不许越过"：用满紧急制动能力算一条更贴边的允许速度，ATO 正常情况下
  永远在它下面；一旦被顶上去（司机手动驾驶、指令异常），ATP 才接管。
把两层混成一层，就分不清"没停准"到底是控制没做好还是安全防护没起作用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .profile import SpeedProfile
from .train import TrainSpec


class AtpState(Enum):
    NORMAL = 'normal'
    WARNING = 'warning'
    INTERVENTION = 'intervention'


@dataclass
class AtoConfig:
    """ATO 控制参数。"""

    a_plan: float = 0.8          # m/s²，规划用的制动减速度（比常用制动小，留余量）
    speed_gain: float = 1.2      # 速度误差到加速度指令的比例增益，1/s
    lookahead: float = 2.0       # m，取目标曲线时向前预瞄的距离
    limit_margin_kph: float = 2.0  # km/h，运行速度比限速压低这么多
    stop_tolerance: float = 0.3  # m，停车允许误差，对标站台标尺
    creep_speed: float = 0.8     # m/s，低于此速度进入蠕动对位
    creep_gain: float = 1.2      # 蠕动段的距离增益，1/s²
    creep_damp: float = 1.6      # 蠕动段的速度阻尼，1/s
    atp_warning_margin: float = 2.0 / 3.6   # km/h→m/s，超 2 km/h 报警
    atp_brake_margin: float = 4.0 / 3.6     # 超 4 km/h 触发紧急制动


@dataclass
class AtoCommand:
    accel: float          # m/s²，正为牵引
    target_speed: float   # m/s，本周期目标速度
    atp: AtpState
    note: str = ''


class Ato:
    def __init__(self, profile: SpeedProfile, spec: TrainSpec,
                 config: AtoConfig | None = None,
                 atp_profile: SpeedProfile | None = None):
        self.profile = profile
        self.spec = spec
        self.config = config or AtoConfig()
        self.atp_profile = atp_profile
        self.stop_x = profile.stop_x

    # ---------------------------------------------------------------- ATP
    def supervise(self, x: float, v: float) -> AtpState:
        if self.atp_profile is None:
            return AtpState.NORMAL
        permit = self.atp_profile.envelope_at(x)
        if v > permit + self.config.atp_brake_margin:
            return AtpState.INTERVENTION
        if v > permit + self.config.atp_warning_margin:
            return AtpState.WARNING
        return AtpState.NORMAL

    # ---------------------------------------------------------------- ATO
    def command(self, x: float, v: float) -> AtoCommand:
        cfg = self.config
        atp = self.supervise(x, v)

        if atp is AtpState.INTERVENTION:
            # 安全层接管，直接给紧急制动，不再理会目标曲线
            return AtoCommand(-self.spec.emergency_brake_decel, 0.0, atp,
                              'ATP 触发紧急制动')

        remaining = self.stop_x - x
        if remaining <= cfg.stop_tolerance and abs(v) < 0.3:
            return AtoCommand(-self.spec.service_brake_decel, 0.0, atp, '已停稳')

        # 目标速度同时受三条约束：线路限速、曲线建议值、以及"还来得及停住"。
        # 曲线在 x=0 处是 0（发车状态），直接取当点会让车永远起不来，
        # 所以向前预瞄一小段，取真正要走到的那个速度。
        v_curve = self.profile.ref_at(x + cfg.lookahead)
        # 比限速压低一点跑。贴着限速跑，进限速区时因为制动力上升有滞后，
        # 车头会短暂压过限速线，ATP 那边就报超速了。
        v_margin = self.profile.limit_at(x + cfg.lookahead) - cfg.limit_margin_kph / 3.6
        v_stop = math.sqrt(max(0.0, 2.0 * cfg.a_plan * max(0.0, remaining)))
        target = min(v_curve, v_stop, max(0.0, v_margin))

        if remaining < 40.0 and v < cfg.creep_speed:
            # 最后几十米按"距离 + 速度"一起闭环，把误差直接压到标尺以内。
            # 只按距离给一个比例项等于无阻尼弹簧，下坡时一定会冲过头；
            # 加上速度阻尼项才是收敛的，坡道上也能停在标尺内。
            accel = cfg.creep_gain * remaining - cfg.creep_damp * v
            accel = max(-self.spec.service_brake_decel, min(0.5, accel))
            return AtoCommand(accel, target, atp, '蠕动对位')

        accel = cfg.speed_gain * (target - v)
        upper = max(0.0, self.spec.accel_capability(max(v, 0.5)))
        accel = max(-self.spec.service_brake_decel, min(upper, accel))
        return AtoCommand(accel, target, atp, '速度跟踪')
