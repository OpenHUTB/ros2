#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""换道可行性判定：给定车道宽度、横向加速度上限与前后车状态，算清"这次能换吗"。

坐标约定：x 为沿车道向前，y 为横向（左正右负）。换道用正弦横向加速度剖面：

    a_y(t) = A * sin(2*pi*t/T)          0 <= t <= T
    v_y(t) = (A*T/(2*pi)) * (1 - cos(2*pi*t/T))
    y(t)   = (A*T/(2*pi)) * (t - (T/(2*pi))*sin(2*pi*t/T))

由 y(T) = L 解出 A = 2*pi*L/T^2，于是给定 L 与 A 上限就能反解时长：

    T = sqrt(2*pi*L / A)        峰值横向速度 v_max = 2*L/T

这个剖面的好处：起点与终点的横向速度、横向加速度都为 0，方向盘与车身不会突变。
"""

from __future__ import division, print_function, unicode_literals

import math


DEFAULT_LANE_WIDTH = 3.50           # 车道中心到中心 (m)
DEFAULT_MAX_LATERAL_ACCEL = 1.50    # 横向加速度上限 (m/s^2)
DEFAULT_TTC_MIN = 3.00              # 后方最小碰撞时间 (s)
DEFAULT_FRONT_MARGIN = 5.00         # 前向安全余量 (m)
DEFAULT_REAR_MARGIN = 5.00          # 后向安全余量 (m)
DEFAULT_DURATION_LIMIT = 5.00       # 换道时长上限 (s)
DEFAULT_EGO_SPEED_KMH = 80.0        # 自车速度
DEFAULT_FRONT_SPEED_KMH = 60.0      # 目标车道前车速度
DEFAULT_FRONT_GAP = 40.0            # 目标车道前车间距 (m)
DEFAULT_REAR_SPEED_KMH = 90.0       # 目标车道后车速度
DEFAULT_REAR_GAP = 30.0             # 目标车道后车间距 (m)
DEFAULT_DIRECTION = 1               # 1 = 向左换道，-1 = 向右换道
DEFAULT_SIM_STEP = 0.02             # 轨迹采样步长 (s)
DEFAULT_PUBLISH_HZ = 10.0
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 14

DIRECTIONS = (1, -1)
ASSERT_NAMES = (u"换道时长", u"横向加速度", u"前向间隙", u"后向 TTC")

_EPS = 1e-9
_INF = float("inf")


class LaneChangeError(ValueError):
    """换道参数不合法。"""


# ------------------------------------------------------------------ 小工具
def _as_float(value, name, default=None):
    if value is None:
        if default is None:
            raise LaneChangeError(u"%s 不能为空" % (name,))
        return float(default)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise LaneChangeError(u"%s 必须是数字，实际 %r" % (name, value))
    if number != number or number in (_INF, -_INF):
        raise LaneChangeError(u"%s 不能是 NaN/Inf" % (name,))
    return number


def _positive(value, name):
    number = _as_float(value, name)
    if number <= 0.0:
        raise LaneChangeError(u"%s 必须大于 0，实际 %r" % (name, number))
    return number


def _non_negative(value, name):
    number = _as_float(value, name)
    if number < 0.0:
        raise LaneChangeError(u"%s 不能为负，实际 %r" % (name, number))
    return number


def kmh_to_mps(speed_kmh):
    return _as_float(speed_kmh, "speed_kmh") / 3.6


def mps_to_kmh(speed_mps):
    return _as_float(speed_mps, "speed_mps") * 3.6


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def offset_at(t, duration, offset):
    """换道至 t 时刻的横向偏移（正弦横向加速度剖面的解析解）。"""
    if duration <= 0.0:
        raise LaneChangeError(u"换道时长必须大于 0")
    if t <= 0.0:
        return 0.0
    if t >= duration:
        return offset
    omega = 2.0 * math.pi * t / duration
    return offset * (t / duration - math.sin(omega) / (2.0 * math.pi))


def lateral_accel_at(t, duration, offset):
    """t 时刻的横向加速度 (m/s^2)，幅值 A = 2*pi*L/T^2。"""
    if duration <= 0.0:
        raise LaneChangeError(u"换道时长必须大于 0")
    if t <= 0.0 or t >= duration:
        return 0.0
    amplitude = 2.0 * math.pi * offset / (duration * duration)
    return amplitude * math.sin(2.0 * math.pi * t / duration)


def lateral_velocity_at(t, duration, offset):
    if duration <= 0.0:
        raise LaneChangeError(u"换道时长必须大于 0")
    if t <= 0.0 or t >= duration:
        return 0.0
    amplitude = 2.0 * math.pi * offset / (duration * duration)
    base = amplitude * duration / (2.0 * math.pi)
    return base * (1.0 - math.cos(2.0 * math.pi * t / duration))


def duration_for(offset, max_accel):
    """由横向位移与横向加速度上限反解换道时长。"""
    offset = _positive(offset, "offset")
    max_accel = _positive(max_accel, "max_lateral_accel")
    return math.sqrt(2.0 * math.pi * offset / max_accel)


# ------------------------------------------------------------------ 规格与场景
class Spec(object):
    """换道几何与安全要求。"""

    __slots__ = ("lane_width", "max_lateral_accel", "ttc_min",
                 "front_margin", "rear_margin", "duration_limit")

    def __init__(self, lane_width=DEFAULT_LANE_WIDTH,
                 max_lateral_accel=DEFAULT_MAX_LATERAL_ACCEL,
                 ttc_min=DEFAULT_TTC_MIN,
                 front_margin=DEFAULT_FRONT_MARGIN,
                 rear_margin=DEFAULT_REAR_MARGIN,
                 duration_limit=DEFAULT_DURATION_LIMIT):
        self.lane_width = _positive(lane_width, "lane_width")
        self.max_lateral_accel = _positive(max_lateral_accel,
                                          "max_lateral_accel")
        self.ttc_min = _positive(ttc_min, "ttc_min")
        self.front_margin = _non_negative(front_margin, "front_margin")
        self.rear_margin = _non_negative(rear_margin, "rear_margin")
        self.duration_limit = _positive(duration_limit, "duration_limit")

    @property
    def offset(self):
        return self.lane_width

    def as_tuple(self):
        return (self.lane_width, self.max_lateral_accel, self.ttc_min,
                self.front_margin, self.rear_margin, self.duration_limit)

    def __repr__(self):
        return ("Spec(lane_width=%.2f, max_lateral_accel=%.2f, ttc_min=%.2f)"
                % (self.lane_width, self.max_lateral_accel, self.ttc_min))


class Traffic(object):
    """目标车道的前后车状态；间距都指车头/车尾到自车的纵向距离。"""

    __slots__ = ("ego_kmh", "front_kmh", "front_gap", "rear_kmh", "rear_gap")

    def __init__(self, ego_kmh=DEFAULT_EGO_SPEED_KMH,
                 front_kmh=DEFAULT_FRONT_SPEED_KMH,
                 front_gap=DEFAULT_FRONT_GAP,
                 rear_kmh=DEFAULT_REAR_SPEED_KMH,
                 rear_gap=DEFAULT_REAR_GAP):
        self.ego_kmh = _non_negative(ego_kmh, "ego_speed_kmh")
        self.front_kmh = _non_negative(front_kmh, "front_speed_kmh")
        self.front_gap = _non_negative(front_gap, "front_gap")
        self.rear_kmh = _non_negative(rear_kmh, "rear_speed_kmh")
        self.rear_gap = _non_negative(rear_gap, "rear_gap")

    @property
    def ego_mps(self):
        return kmh_to_mps(self.ego_kmh)

    @property
    def front_mps(self):
        return kmh_to_mps(self.front_kmh)

    @property
    def rear_mps(self):
        return kmh_to_mps(self.rear_kmh)

    def as_tuple(self):
        return (self.ego_kmh, self.front_kmh, self.front_gap,
                self.rear_kmh, self.rear_gap)

    def __repr__(self):
        return ("Traffic(ego=%.1f, front=%.1f@%.1fm, rear=%.1f@%.1fm)"
                % (self.ego_kmh, self.front_kmh, self.front_gap,
                   self.rear_kmh, self.rear_gap))


class Sample(object):
    """轨迹采样点。"""

    __slots__ = ("t", "y", "vy", "ay")

    def __init__(self, t, y, vy, ay):
        self.t = t
        self.y = y
        self.vy = vy
        self.ay = ay

    def as_tuple(self):
        return (self.t, self.y, self.vy, self.ay)

    def __repr__(self):
        return ("Sample(t=%.3f, y=%.3f, vy=%.3f, ay=%.3f)"
                % (self.t, self.y, self.vy, self.ay))


class Plan(object):
    """一次换道的规划结果。"""

    __slots__ = ("spec", "traffic", "direction", "duration", "distance",
                 "peak_accel", "peak_velocity", "required_front_gap",
                 "required_rear_gap", "ttc_front", "ttc_rear", "reasons")

    def __init__(self, spec, traffic, direction, duration, distance,
                 peak_accel, peak_velocity, required_front_gap,
                 required_rear_gap, ttc_front, ttc_rear, reasons):
        self.spec = spec
        self.traffic = traffic
        self.direction = direction
        self.duration = duration
        self.distance = distance
        self.peak_accel = peak_accel
        self.peak_velocity = peak_velocity
        self.required_front_gap = required_front_gap
        self.required_rear_gap = required_rear_gap
        self.ttc_front = ttc_front
        self.ttc_rear = ttc_rear
        self.reasons = reasons

    @property
    def offset(self):
        return self.spec.offset * self.direction

    @property
    def feasible(self):
        return not self.reasons

    @property
    def gap_shortfall(self):
        """前向间隙差多少（正数表示差这么多米）。"""
        return self.required_front_gap - self.traffic.front_gap

    @property
    def ttc_shortfall(self):
        """后向 TTC 差多少秒（正数表示差这么多秒）。"""
        if self.ttc_rear is _INF:
            return 0.0
        return self.spec.ttc_min - self.ttc_rear

    def offset_at(self, t):
        return offset_at(t, self.duration, self.offset)

    def lateral_accel_at(self, t):
        return lateral_accel_at(t, self.duration, self.offset)

    def lateral_velocity_at(self, t):
        return lateral_velocity_at(t, self.duration, self.offset)

    def samples(self, step=DEFAULT_SIM_STEP):
        step = _positive(step, "step")
        count = int(round(self.duration / step))
        points = []
        for index in range(count + 1):
            t = index * step
            if t > self.duration:
                t = self.duration
            points.append(Sample(t,
                                 self.offset_at(t),
                                 self.lateral_velocity_at(t),
                                 self.lateral_accel_at(t)))
        return points

    def as_tuple(self):
        return (self.direction, self.duration, self.distance, self.feasible)


def make_plan(spec=None, traffic=None, direction=DEFAULT_DIRECTION):
    """按规格与交通状态算一次换道：时长、距离、所需间隙与 TTC。"""
    spec = spec or Spec()
    traffic = traffic or Traffic()
    if direction not in DIRECTIONS:
        raise LaneChangeError(u"换道方向只能是 1 或 -1，实际 %r" % (direction,))

    duration = duration_for(spec.offset, spec.max_lateral_accel)
    ego = traffic.ego_mps
    front = traffic.front_mps
    rear = traffic.rear_mps

    distance = ego * duration
    # 换道期间自车比前车多走的那一段，就是额外需要的间隙
    closing_front = max(0.0, ego - front)
    closing_rear = max(0.0, rear - ego)
    required_front_gap = closing_front * duration + spec.front_margin
    required_rear_gap = closing_rear * duration + spec.rear_margin

    if ego > front + _EPS:
        ttc_front = traffic.front_gap / (ego - front)
    else:
        ttc_front = _INF
    if rear > ego + _EPS:
        ttc_rear = traffic.rear_gap / (rear - ego)
    else:
        ttc_rear = _INF

    reasons = []
    if duration > spec.duration_limit:
        reasons.append(u"换道时长 %.2f s 超过上限 %.2f s"
                       % (duration, spec.duration_limit))
    if traffic.front_gap < required_front_gap - _EPS:
        reasons.append(u"前向间隙 %.2f m 不足（需要 %.2f m）"
                       % (traffic.front_gap, required_front_gap))
    if traffic.rear_gap < required_rear_gap - _EPS:
        reasons.append(u"后向间隙 %.2f m 不足（需要 %.2f m）"
                       % (traffic.rear_gap, required_rear_gap))
    if ttc_rear < spec.ttc_min - _EPS:
        reasons.append(u"后车 TTC %.2f s 低于下限 %.2f s"
                       % (ttc_rear, spec.ttc_min))

    return Plan(spec, traffic, direction, duration, distance,
                2.0 * math.pi * spec.offset / (duration * duration),
                2.0 * spec.offset / duration,
                required_front_gap, required_rear_gap,
                ttc_front, ttc_rear, reasons)


# ------------------------------------------------------------------ 四条断言
def _result(name, passed, detail):
    return {"type": name, "passed": bool(passed), "detail": detail}


def check_plan(plan, duration_limit=None):
    """四条断言：换道时长、横向加速度、前向间隙、后向 TTC。"""
    spec = plan.spec
    limit = (spec.duration_limit if duration_limit is None
             else _positive(duration_limit, "duration_limit"))
    results = []

    results.append(_result(
        ASSERT_NAMES[0], plan.duration <= limit + _EPS,
        u"换道用时 %.2f s，上限 %.2f s" % (plan.duration, limit)))

    results.append(_result(
        ASSERT_NAMES[1], plan.peak_accel <= spec.max_lateral_accel + _EPS,
        u"峰值横向加速度 %.2f m/s^2，上限 %.2f m/s^2"
        % (plan.peak_accel, spec.max_lateral_accel)))

    results.append(_result(
        ASSERT_NAMES[2],
        plan.traffic.front_gap >= plan.required_front_gap - _EPS,
        u"前向间隙 %.2f m，需要 %.2f m（差 %.2f m）"
        % (plan.traffic.front_gap, plan.required_front_gap,
           plan.gap_shortfall)))

    if plan.ttc_rear is _INF:
        results.append(_result(ASSERT_NAMES[3], True,
                               u"后车不快于自车，无逼近风险"))
    else:
        results.append(_result(
            ASSERT_NAMES[3], plan.ttc_rear >= spec.ttc_min - _EPS,
            u"后车 TTC %.2f s，下限 %.2f s" % (plan.ttc_rear, spec.ttc_min)))

    return results


def all_passed(results):
    return all(item["passed"] for item in results)


def advice(plan):
    """给出可执行的建议：等多久、加减到多少、还是干脆别换。"""
    lines = []
    spec = plan.spec
    traffic = plan.traffic

    if plan.duration > spec.duration_limit + _EPS:
        need = 2.0 * math.pi * spec.offset / (spec.duration_limit ** 2)
        lines.append(u"缩短换道时间：横向加速度上限需提到 %.2f m/s^2 以上"
                     % (need,))

    if plan.traffic.front_gap < plan.required_front_gap - _EPS:
        short = plan.required_front_gap - plan.traffic.front_gap
        # 要么等前车拉开，要么把速度降到 gap 允许的水平
        allowed_ego = (traffic.front_mps
                       + (traffic.front_gap - spec.front_margin)
                       / max(plan.duration, _EPS))
        lines.append(u"前向间隙还差 %.1f m：把车速降到 %.1f km/h 以下，"
                     u"或等前车拉开后再换" % (short, mps_to_kmh(allowed_ego)))

    if plan.ttc_rear is not _INF and plan.ttc_rear < spec.ttc_min - _EPS:
        need_gap = spec.ttc_min * (traffic.rear_mps - traffic.ego_mps)
        lines.append(u"后车 %.1f km/h 正在逼近，TTC 只有 %.2f s："
                     u"等它拉开到 %.1f m 以外，或等它超车通过"
                     % (traffic.rear_kmh, plan.ttc_rear, need_gap))

    if plan.traffic.rear_gap < plan.required_rear_gap - _EPS:
        need_ego = (traffic.rear_mps
                    - (traffic.rear_gap - spec.rear_margin)
                    / max(plan.duration, _EPS))
        lines.append(u"提速到 %.1f km/h 以上可以缩小后向占用"
                     % (mps_to_kmh(need_ego),))

    if not lines:
        lines.append(u"当前时机可行，按 %.2f s 的剖面执行，"
                     u"换道期间推进 %.1f m" % (plan.duration, plan.distance))
    return lines


# ------------------------------------------------------------------ 字符图
def render_lane(plan, width=DEFAULT_GRID_WIDTH, height=DEFAULT_GRID_HEIGHT):
    """俯视示意图：三车道，o 自车，F 前车，R 后车。"""
    width = int(_positive(width, "grid_width"))
    height = int(_positive(height, "grid_height"))
    if width < 20 or height < 6:
        raise LaneChangeError(u"字符图尺寸太小")
    rows = [list(" " * width) for _ in range(height)]

    lanes = (-1, 0, 1)
    for index, lane in enumerate(lanes):
        column = int(round((index + 0.5) * width / 3.0))
        column = clamp(column, 1, width - 2)
        for row in range(1, height - 1):
            if rows[row][column] == " ":
                rows[row][column] = "|"

    middle = (height - 1) // 2
    middle = clamp(middle, 1, height - 2)
    center_column = int(round((1 + 0.5) * width / 3.0))
    target = 0 if plan.direction > 0 else 2
    target_column = int(round((target + 0.5) * width / 3.0))

    front_row = clamp(int(middle) - 2, 1, height - 2)
    rear_row = clamp(int(middle) + 2, 1, height - 2)
    rows[front_row][clamp(target_column, 1, width - 2)] = "F"
    rows[rear_row][clamp(target_column, 1, width - 2)] = "R"
    rows[middle][clamp(center_column, 1, width - 2)] = "o"

    for index in range(1, height, 2):
        rows[index][0] = "."

    return [u"".join(row) for row in rows]


# ------------------------------------------------------------------ 报告
def format_report(spec, traffic, plan, results, header=""):
    lines = []
    if header:
        lines.append(header)
    lines.append(u"换道方向：%s，车道宽度 %.2f m，横向加速度上限 %.2f m/s^2"
                 % (u"向左" if plan.direction > 0 else u"向右",
                    spec.lane_width, spec.max_lateral_accel))
    lines.append(u"自车 %.1f km/h（%.2f m/s），目标车道前车 %.1f km/h 距 %.1f m，"
                 u"后车 %.1f km/h 距 %.1f m"
                 % (traffic.ego_kmh, traffic.ego_mps, traffic.front_kmh,
                    traffic.front_gap, traffic.rear_kmh, traffic.rear_gap))
    lines.append(u"-" * 60)
    lines.append(u"换道时长：%.3f s（上限 %.2f s）" % (plan.duration,
                                                spec.duration_limit))
    lines.append(u"换道距离：%.3f m（自车 %.2f m/s 乘以时长）"
                 % (plan.distance, traffic.ego_mps))
    lines.append(u"峰值横向加速度：%.3f m/s^2，峰值横向速度：%.3f m/s"
                 % (plan.peak_accel, plan.peak_velocity))
    lines.append(u"所需前向间隙：%.3f m（实际 %.3f m）"
                 % (plan.required_front_gap, traffic.front_gap))
    lines.append(u"所需后向间隙：%.3f m（实际 %.3f m）"
                 % (plan.required_rear_gap, traffic.rear_gap))
    if plan.ttc_front is _INF:
        lines.append(u"前车 TTC：不适用（不快于自车）")
    else:
        lines.append(u"前车 TTC：%.3f s" % (plan.ttc_front,))
    if plan.ttc_rear is _INF:
        lines.append(u"后车 TTC：不适用（不快于自车）")
    else:
        lines.append(u"后车 TTC：%.3f s（下限 %.2f s）"
                     % (plan.ttc_rear, spec.ttc_min))
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (sum(1 for r in results if r["passed"]),
                    sum(1 for r in results if not r["passed"]), len(results)))
    for item in results:
        lines.append(u"  [%s] %-6s %s"
                     % (u"PASS" if item["passed"] else u"FAIL",
                        item["type"], item["detail"]))
    lines.append(u"决策：%s" % (u"可以换道" if plan.feasible else u"不建议换道",
                              ))
    if plan.reasons:
        for reason in plan.reasons:
            lines.append(u"  - %s" % (reason,))
    for line in advice(plan):
        lines.append(u"建议：%s" % (line,))
    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED",))
    return u"\n".join(lines)


def to_message_text(text):
    if not isinstance(text, str):
        text = u"%s" % (text,)
    try:
        return text.encode("utf-8")
    except AttributeError:
        return text


# ------------------------------------------------------------------ 参数
PARAM_KEYS = (
    "lane/width",
    "lane/max_lateral_accel",
    "lane/duration_limit",
    "lane/direction",
    "safety/ttc_min",
    "safety/front_margin",
    "safety/rear_margin",
    "traffic/ego_speed_kmh",
    "traffic/front_speed_kmh",
    "traffic/front_gap",
    "traffic/rear_speed_kmh",
    "traffic/rear_gap",
    "sim/step",
    "rate/publish_hz",
)

PARAM_DEFAULTS = {
    "lane/width": DEFAULT_LANE_WIDTH,
    "lane/max_lateral_accel": DEFAULT_MAX_LATERAL_ACCEL,
    "lane/duration_limit": DEFAULT_DURATION_LIMIT,
    "lane/direction": DEFAULT_DIRECTION,
    "safety/ttc_min": DEFAULT_TTC_MIN,
    "safety/front_margin": DEFAULT_FRONT_MARGIN,
    "safety/rear_margin": DEFAULT_REAR_MARGIN,
    "traffic/ego_speed_kmh": DEFAULT_EGO_SPEED_KMH,
    "traffic/front_speed_kmh": DEFAULT_FRONT_SPEED_KMH,
    "traffic/front_gap": DEFAULT_FRONT_GAP,
    "traffic/rear_speed_kmh": DEFAULT_REAR_SPEED_KMH,
    "traffic/rear_gap": DEFAULT_REAR_GAP,
    "sim/step": DEFAULT_SIM_STEP,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
}


class LaneChangeParams(object):
    """从参数服务器读到的全部配置。"""

    def __init__(self, **kwargs):
        for key in PARAM_KEYS:
            value = kwargs.get(key, PARAM_DEFAULTS[key])
            setattr(self, self._attr(key),
                    _as_float(value, key, PARAM_DEFAULTS[key]))
        self.spec = Spec(self.lane_width, self.lane_max_lateral_accel,
                         self.safety_ttc_min, self.safety_front_margin,
                         self.safety_rear_margin, self.lane_duration_limit)
        self.traffic = Traffic(self.traffic_ego_speed_kmh,
                               self.traffic_front_speed_kmh,
                               self.traffic_front_gap,
                               self.traffic_rear_speed_kmh,
                               self.traffic_rear_gap)
        direction = int(round(self.lane_direction))
        if direction not in DIRECTIONS:
            raise LaneChangeError(u"lane/direction 只能是 1 或 -1，实际 %r"
                                  % (direction,))
        self.direction = direction

    @staticmethod
    def _attr(key):
        return key.replace("/", "_")

    @classmethod
    def from_params(cls, get_param):
        values = {}
        for key in PARAM_KEYS:
            values[key] = get_param(key, PARAM_DEFAULTS[key])
        return cls(**values)

    def as_dict(self):
        return dict((key, getattr(self, self._attr(key))) for key in PARAM_KEYS)

    def __repr__(self):
        pairs = sorted(self.as_dict().items())
        return "LaneChangeParams(%s)" % ", ".join("%s=%r" % pair
                                                  for pair in pairs)

# ------------------------------------------------------------------ TF 广播
DEFAULT_TF_ENABLED = True
DEFAULT_TF_RATE_HZ = 10.0
DEFAULT_TF_WORLD_FRAME = u"map"
DEFAULT_TF_EGO_FRAME = u"ego"
DEFAULT_TF_KEYFRAMES = 24


def ego_frames(plan, count=DEFAULT_TF_KEYFRAMES, world=DEFAULT_TF_WORLD_FRAME,
               ego=DEFAULT_TF_EGO_FRAME):
    """换道过程中 ego 在 map 里的位姿关键帧：(parent, child, x, y, yaw)。"""
    count = max(2, int(count))
    duration = plan.duration if plan.duration > 0.0 else 1.0
    speed = plan.traffic.ego_mps
    step = duration / (count - 1)
    frames = []
    for index in range(count):
        t = step * index
        lateral = plan.offset_at(t)
        lateral_rate = plan.lateral_velocity_at(t)
        frames.append((world, ego, speed * t, lateral,
                       math.atan2(lateral_rate, max(speed, 1e-3))))
    return frames
