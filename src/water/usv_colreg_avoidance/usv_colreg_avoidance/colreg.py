"""COLREG（国际海上避碰规则）会遇局面判定与避让决策。

只覆盖开阔水域里最常见的三条规则：

- Rule 13 追越：从正横后方 22.5° 扇区接近，追越船为让路船；
- Rule 14 对遇：正前方左右各 6° 内、航向大致相反，双方都向右转；
- Rule 15 交叉相遇：目标在右舷则本船让路，在左舷则本船直航。

以及让路船 / 直航船的行为约束（Rule 16 大幅度早让、Rule 17 保向保速）。

全部是几何与规则判断，不依赖 ROS、numpy 或任何第三方库。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum

# Rule 14 的「对遇」判定扇区：正前方左右各半个扇区角
HEAD_ON_HALF_ANGLE = 6.0
# Rule 13 的「追越」判定扇区：正横后方 22.5° 以内
OVERTAKING_ANGLE = 112.5
# 判定航向大致相反的阈值
RECIPROCAL_COURSE = 170.0


def wrap360(deg: float) -> float:
    """角度归一化到 [0, 360)。航海习惯：0° 为正北，顺时针为正。"""
    return deg % 360.0


def wrap180(deg: float) -> float:
    """角度归一化到 (-180, 180]，用于相对方位与航向差。"""
    return (deg + 180.0) % 360.0 - 180.0


class Situation(str, Enum):
    """两船之间的会遇局面。"""

    HEAD_ON = 'head-on'
    CROSSING_GIVE_WAY = 'crossing-give-way'
    CROSSING_STAND_ON = 'crossing-stand-on'
    OVERTAKING = 'overtaking'
    OVERTAKEN = 'overtaken'
    CLEAR = 'clear'


class Role(str, Enum):
    """本船在该局面下的角色（Rule 16 / 17）。"""

    GIVE_WAY = 'give-way'
    STAND_ON = 'stand-on'
    FREE = 'free'


@dataclass
class Vessel:
    """一条船的状态。位置单位 m，航向单位 deg（0=正北，顺时针），速度单位 m/s。"""

    name: str
    x: float
    y: float
    heading: float
    speed: float
    length: float = 12.0
    beam: float = 4.0

    def velocity(self) -> tuple[float, float]:
        h = math.radians(self.heading)
        return self.speed * math.sin(h), self.speed * math.cos(h)

    def moved(self, heading: float | None = None, speed: float | None = None) -> 'Vessel':
        """返回改了航向或速度的副本，用于「假如这样走会怎样」的试算。"""
        return replace(
            self,
            heading=self.heading if heading is None else wrap360(heading),
            speed=self.speed if speed is None else speed,
        )


@dataclass
class ColregConfig:
    """避碰判据。默认值对应一条 12 m 长的中型无人艇在开阔水域的情形。"""

    d_safe: float = 400.0        # 安全会遇距离 D_safe (m)
    d_emergency: float = 250.0   # 紧迫危险距离 (m)
    d_action: float = 1200.0     # 进入这个距离内就开始判断，追越这类慢接近靠它触发
    horizon: float = 100.0       # 或者 TCPA 小于这个值就判断，对遇这类快接近靠它触发
    min_turn: float = 30.0       # Rule 16 要求「大幅度」，对遇与交叉的最小转向角 (deg)
    min_turn_overtaking: float = 15.0  # 追越没有幅度要求，小角度即可
    max_turn: float = 90.0       # 避让转向的搜索上限 (deg)
    turn_margin: float = 0.15    # 转向量按 (1+margin)·D_safe 来搜，补上操舵滞后
    release_factor: float = 2.5  # 让清判据里 DCPA 的松紧倍数


@dataclass
class Decision:
    """一条船在某一时刻的避让决策。"""

    role: Role
    situation: Situation
    target: str | None
    dcpa: float
    tcpa: float
    turn: float          # 建议向右转的角度 (deg)，0 表示保持航向
    reason: str


def distance(a: Vessel, b: Vessel) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def relative_bearing(own: Vessel, other: Vessel) -> float:
    """目标相对本船船首的方位，右舷为正、左舷为负，范围 (-180, 180]。"""
    rx, ry = other.x - own.x, other.y - own.y
    if abs(rx) < 1e-9 and abs(ry) < 1e-9:
        return 0.0
    bearing = math.degrees(math.atan2(rx, ry))  # 0=北，顺时针
    return wrap180(bearing - own.heading)


def cpa(own: Vessel, other: Vessel) -> tuple[float, float]:
    """最近会遇距离 DCPA 与到达时刻 TCPA。

    两条船都按当前航向匀速直航外推。TCPA 为负表示已经在远离。
    """
    rx, ry = other.x - own.x, other.y - own.y
    ovx, ovy = own.velocity()
    tvx, tvy = other.velocity()
    vx, vy = tvx - ovx, tvy - ovy
    vv = vx * vx + vy * vy
    if vv < 1e-12:
        return math.hypot(rx, ry), math.inf
    tcpa = -(rx * vx + ry * vy) / vv
    te = max(tcpa, 0.0)
    dcpa = math.hypot(rx + vx * te, ry + vy * te)
    return dcpa, tcpa


def classify(own: Vessel, other: Vessel) -> Situation:
    """按 Rule 13/14/15 判断本船与目标的会遇局面。"""
    _, tcpa = cpa(own, other)
    if tcpa <= 0.0:
        return Situation.CLEAR

    rb = relative_bearing(own, other)
    course_diff = abs(wrap180(other.heading - own.heading))

    if abs(rb) <= HEAD_ON_HALF_ANGLE and course_diff >= RECIPROCAL_COURSE:
        return Situation.HEAD_ON

    # Rule 13(d)：一旦构成追越，追越船的责任不因随后方位变化而解除。
    # 追越船从侧前方擦过去时，被追越船看到的方位会从「正后方」变成「右舷」，
    # 如果不加这一条，被追越船会误判成交叉局面跟着一起让路。
    if abs(rb) > 90.0 and other.speed > own.speed:
        return Situation.OVERTAKEN

    if abs(rb) > OVERTAKING_ANGLE:
        # 目标在正横后方：快的一方是追越船
        return Situation.OVERTAKING if own.speed > other.speed else Situation.OVERTAKEN

    if abs(rb) <= HEAD_ON_HALF_ANGLE:
        # 正前方但航向相近，属于追越前方慢船
        return Situation.OVERTAKING if own.speed > other.speed else Situation.CLEAR

    return Situation.CROSSING_GIVE_WAY if rb > 0 else Situation.CROSSING_STAND_ON


def required_turn(own: Vessel, other: Vessel, config: ColregConfig,
                  min_turn: float | None = None, step: float = 1.0) -> float:
    """向右转多少度才能把 DCPA 拉到安全值以上。

    假定目标保持匀速直航（COLREG 里的常规假设），只做一次几何搜索，
    不含船舶操纵滞后，所以目标值取 (1 + margin)·D_safe 留一点余量。
    找不到就在 max_turn 处截断。
    """
    target = config.d_safe * (1.0 + config.turn_margin)
    dcpa, _ = cpa(own, other)
    if dcpa >= target:
        return 0.0

    turn = config.min_turn if min_turn is None else min_turn
    while turn <= config.max_turn:
        if cpa(own.moved(heading=own.heading + turn), other)[0] >= target:
            return turn
        turn += step
    return config.max_turn


def _threats(own: Vessel, others: list[Vessel], config: ColregConfig) -> list[tuple[float, float, Vessel]]:
    """筛出会构成冲突的目标，按紧迫程度排序。

    触发条件用「时间」和「距离」两条并联：对遇、交叉这类快速接近由 TCPA 抓住，
    追越这种相对速度只有几 m/s 的慢接近，TCPA 一直很大，得靠距离兜住。
    """
    found = []
    for t in others:
        if t.name == own.name:
            continue
        dcpa, tcpa = cpa(own, t)
        if tcpa <= 0.0 or dcpa >= config.d_safe:
            continue
        if tcpa > config.horizon and distance(own, t) > config.d_action:
            continue
        found.append((tcpa, dcpa, t))
    found.sort(key=lambda item: (item[0], item[1]))
    return found


def decide(own: Vessel, others: list[Vessel],
           config: ColregConfig | None = None) -> Decision:
    """给出本船此刻应采取的避让动作。"""
    config = config or ColregConfig()
    threats = _threats(own, others, config)
    if not threats:
        return Decision(Role.FREE, Situation.CLEAR, None, math.inf, math.inf, 0.0,
                        '无冲突目标')

    tcpa, dcpa, target = threats[0]
    situation = classify(own, target)

    if situation in (Situation.HEAD_ON, Situation.CROSSING_GIVE_WAY, Situation.OVERTAKING):
        min_turn = (config.min_turn_overtaking if situation is Situation.OVERTAKING
                    else config.min_turn)
        turn = required_turn(own, target, config, min_turn=min_turn)
        rule = {
            Situation.HEAD_ON: 'Rule 14 对遇，双方右转',
            Situation.CROSSING_GIVE_WAY: 'Rule 15 右舷交叉，本船让路',
            Situation.OVERTAKING: 'Rule 13 追越，本船让路',
        }[situation]
        return Decision(Role.GIVE_WAY, situation, target.name, dcpa, tcpa, turn,
                        f'{rule}，右转 {turn:.0f}°')

    # 直航船：原则保向保速，只有对方明显没让、危险迫近时才自行右转（Rule 17 b）
    if distance(own, target) < config.d_emergency and dcpa < config.d_safe:
        turn = required_turn(own, target, config)
        return Decision(Role.STAND_ON, situation, target.name, dcpa, tcpa, turn,
                        f'Rule 17(b) 紧迫危险，自行右转 {turn:.0f}°')

    rule = 'Rule 17(a) 直航船保向保速'
    if situation is Situation.OVERTAKEN:
        rule = 'Rule 13 被追越船保向保速'
    return Decision(Role.STAND_ON, situation, target.name, dcpa, tcpa, 0.0, rule)


def threat_cleared(own: Vessel, other: Vessel, config: ColregConfig) -> bool:
    """判断此前的威胁目标是否已经让清了。

    航海上的经验是「让到目标过本船正横」才算让清，这里取 110° —— 比正横
    再多一点，避免目标刚好卡在正横线上时航向来回摆（Rule 8 禁止一连串小变动）。

    追越场景尤其需要这个余量：刚超过被追越船时两船纵向只差几十米，
    这时候切回原航线等于横穿对方船首。
    """
    dcpa, tcpa = cpa(own, other)
    separation = distance(own, other)

    if tcpa <= 0.0 and separation >= config.d_safe:
        return True

    if abs(relative_bearing(own, other)) >= 110.0 and separation > config.d_emergency:
        return True

    # 兜底：已经错开很远，不必继续咬着避让航向不放
    return (dcpa >= config.release_factor * config.d_safe
            and separation > 3.0 * config.d_safe)


def min_separation(positions_a: list[tuple[float, float]],
                   positions_b: list[tuple[float, float]]) -> float:
    """两条轨迹逐帧的最小中心距。"""
    n = min(len(positions_a), len(positions_b))
    return min((math.hypot(positions_b[i][0] - positions_a[i][0],
                           positions_b[i][1] - positions_a[i][1]) for i in range(n)),
               default=math.inf)
