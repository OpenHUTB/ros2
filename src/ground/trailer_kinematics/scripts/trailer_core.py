#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""trailer_kinematics 的核心计算：牵引车-挂车运动学与折刀判定。

特点
----
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 两个状态量（牵引车航向、挂车航向）用显式欧拉积分推进，不是一代了事

模型
----
记牵引车后轴处航向为 theta1、挂车航向为 theta2、铰接角 phi = theta1 - theta2：

    theta1_dot = v * tan(delta) / L1
    theta2_dot = ( v * sin(phi) - d * theta1_dot * cos(phi) ) / L2

其中 L1 是牵引车轴距、L2 是铰接点到挂车轴的距离、d 是铰接点在牵引车
后轴后方的偏移量（d = 0 就是经典的「铰接点在轴上」模型）。

稳态（phi_dot = 0）时有闭式解。记 A = tan(delta) / L1：

    sin(phi) - d*A*cos(phi) = L2*A

写成 R*sin(phi - psi) = L2*A（R = sqrt(1 + (dA)^2)、psi = atan(dA)）后

    phi_steady = psi + asin( L2*A / sqrt(1 + (dA)^2) )

当 L2*A/sqrt(1+(dA)^2) > 1 时不存在稳态，铰接角会一直涨到折刀。
"""

from __future__ import division, print_function, unicode_literals

import io
import math


# --------------------------------------------------------------------------
# 默认参数
# --------------------------------------------------------------------------

DEFAULT_WHEELBASE = 2.70          # L1：牵引车轴距 (m)
DEFAULT_TRAILER_LENGTH = 4.20     # L2：铰接点到挂车轴 (m)
DEFAULT_HITCH_OFFSET = 0.60       # d：铰接点在牵引车后轴后方 (m)
DEFAULT_MAX_STEER_DEG = 35.0      # 前轮最大转角
DEFAULT_JACKKNIFE_DEG = 90.0      # 折刀角上限
DEFAULT_WARN_DEG = 60.0           # 预警角
DEFAULT_CHANNEL_WIDTH = 12.00     # 允许占用的通道宽度 (m)
DEFAULT_SPEED = 1.00              # 行驶速度 (m/s)
DEFAULT_STEER_DEG = 25.0          # 前轮转角
DEFAULT_SECONDS = 9.10            # 仿真时长 (s)，默认正好转 90 度
DEFAULT_STEP = 0.02               # 积分步长 (s)
DEFAULT_GEAR = 1                  # 1 前进、-1 倒车
DEFAULT_PUBLISH_HZ = 10.0         # 发布频率
DEFAULT_STEP_TOLERANCE = 0.01     # 步长减半后的位置差上限 (m)
DEFAULT_GRID_WIDTH = 60           # 字符相图宽度
DEFAULT_GRID_HEIGHT = 18          # 字符相图高度

GEARS = (1, -1)

ASSERT_NAMES = (u"折刀余量", u"积分精度", u"通道宽度", u"倒车稳定性")


class TrailerError(ValueError):
    """参数或场景不合法时抛出。"""


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------

def _as_float(value, name, default=None):
    """把参数转成 float，空值回落到 default。"""
    if value is None or value == "":
        if default is None:
            raise TrailerError(u"%s 不能为空" % name)
        return float(default)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise TrailerError(u"%s 必须是数字，收到 %r" % (name, value))
    if math.isnan(result) or math.isinf(result):
        raise TrailerError(u"%s 必须是有限数字，收到 %r" % (name, value))
    return result


def _positive(value, name, default=None):
    result = _as_float(value, name, default)
    if result <= 0.0:
        raise TrailerError(u"%s 必须大于 0，收到 %r" % (name, result))
    return result


def _non_negative(value, name, default=0.0):
    result = _as_float(value, name, default)
    if result < 0.0:
        raise TrailerError(u"%s 不能为负，收到 %r" % (name, result))
    return result


def as_bool(value, default=False):
    """兼容 py2 的 str/unicode 与 py3 的 str。"""
    if value is None or value == "":
        return bool(default)
    if isinstance(value, bool):
        return value
    text = value
    if not isinstance(text, type("")):
        text = u"%r" % (value,)
    return text.strip().lower() in (u"1", u"true", u"yes", u"on")


def normalize_angle(angle):
    """折算到 [-pi, pi)（对负数也要正确回卷；pi 与 -pi 视为同一角度）。"""
    result = math.fmod(angle + math.pi, 2.0 * math.pi)
    if result < 0.0:
        result += 2.0 * math.pi
    return result - math.pi


def deg(angle):
    return angle * 180.0 / math.pi


def rad(degree):
    return degree * math.pi / 180.0


def to_message_text(text):
    """py2 下转成 bytes，py3 下保持 str，供 std_msgs/String 使用。"""
    try:
        if isinstance(text, type(u"")) and not isinstance(text, type("")):
            return text.encode("utf-8")
    except Exception:
        pass
    return text


# --------------------------------------------------------------------------
# 车辆与采样
# --------------------------------------------------------------------------

class TrailerSpec(object):
    """牵引车 + 挂车的几何参数。"""

    def __init__(self, wheelbase=DEFAULT_WHEELBASE,
                 trailer_length=DEFAULT_TRAILER_LENGTH,
                 hitch_offset=DEFAULT_HITCH_OFFSET,
                 max_steer_deg=DEFAULT_MAX_STEER_DEG):
        self.wheelbase = _positive(wheelbase, u"wheelbase")
        self.trailer_length = _positive(trailer_length, u"trailer_length")
        self.hitch_offset = _non_negative(hitch_offset, u"hitch_offset")
        self.max_steer_deg = _positive(max_steer_deg, u"max_steer_deg")
        self.max_steer = rad(self.max_steer_deg)

    @property
    def min_turn_radius(self):
        """牵引车后轴最小转弯半径。"""
        return self.wheelbase / math.tan(self.max_steer)

    @property
    def total_length(self):
        """组合体的近似总长（牵引车轴距 + 挂车长度 + 铰接偏移）。"""
        return self.wheelbase + self.hitch_offset + self.trailer_length

    def as_tuple(self):
        return (self.wheelbase, self.trailer_length, self.hitch_offset,
                self.max_steer_deg)

    def __repr__(self):
        return ("TrailerSpec(wheelbase=%.3f, trailer_length=%.3f, "
                "hitch_offset=%.3f, max_steer_deg=%.3f)" % self.as_tuple())


class Sample(object):
    """一帧状态。"""

    def __init__(self, t, x, y, tractor_yaw, trailer_yaw, speed):
        self.t = t
        self.x = x
        self.y = y
        self.tractor_yaw = tractor_yaw
        self.trailer_yaw = trailer_yaw
        self.speed = speed

    @property
    def articulation(self):
        """铰接角（弧度），折算到 (-pi, pi]。"""
        return normalize_angle(self.tractor_yaw - self.trailer_yaw)

    @property
    def articulation_deg(self):
        return abs(deg(self.articulation))

    def hitch_point(self, spec):
        """铰接点位置（牵引车后轴沿牵引车航向往前 hitch_offset）。"""
        return (self.x + spec.hitch_offset * math.cos(self.tractor_yaw),
                self.y + spec.hitch_offset * math.sin(self.tractor_yaw))

    def trailer_point(self, spec):
        """挂车轴位置（从铰接点沿挂车航向往后 L2）。"""
        hx, hy = self.hitch_point(spec)
        return (hx - spec.trailer_length * math.cos(self.trailer_yaw),
                hy - spec.trailer_length * math.sin(self.trailer_yaw))

    def as_tuple(self):
        return (self.t, self.x, self.y, self.tractor_yaw, self.trailer_yaw,
                self.speed)

    def __repr__(self):
        return ("Sample(t=%.3f, x=%.3f, y=%.3f, art=%.3f deg)"
                % (self.t, self.x, self.y, self.articulation_deg))


class Trajectory(object):
    """一段仿真的采样序列。"""

    def __init__(self, samples, spec, name=""):
        self.samples = list(samples)
        self.spec = spec
        self.name = name

    @property
    def count(self):
        return len(self.samples)

    def is_empty(self):
        return not self.samples

    def first(self):
        return self.samples[0] if self.samples else None

    def last(self):
        return self.samples[-1] if self.samples else None

    def duration(self):
        if not self.samples:
            return 0.0
        return self.samples[-1].t - self.samples[0].t

    def articulation_max_deg(self):
        if not self.samples:
            return 0.0
        return max(s.articulation_deg for s in self.samples)

    def articulation_final_deg(self):
        if not self.samples:
            return 0.0
        return self.samples[-1].articulation_deg

    def articulation_signed(self):
        """带符号的铰接角序列（用于相图与增长判定）。"""
        return [deg(s.articulation) for s in self.samples]

    def turn_angle_deg(self):
        """牵引车总共转过的角度（绝对值）。"""
        if not self.samples:
            return 0.0
        return abs(deg(normalize_angle(self.samples[-1].tractor_yaw
                                       - self.samples[0].tractor_yaw)))

    def tail_articulation_deg(self, fraction=0.1):
        """末段（默认最后 10%）的铰接角绝对值平均。"""
        if not self.samples:
            return 0.0
        count = max(1, int(round(len(self.samples) * fraction)))
        tail = self.samples[-count:]
        return sum(s.articulation_deg for s in tail) / float(len(tail))

    def head_articulation_deg(self, fraction=0.1):
        """首段（默认前 10%）的铰接角绝对值平均。"""
        if not self.samples:
            return 0.0
        count = max(1, int(round(len(self.samples) * fraction)))
        head = self.samples[:count]
        return sum(s.articulation_deg for s in head) / float(len(head))

    def channel_needed(self):
        """整个过程中车身横向占用的跨度（起步沿 +x，取 y 的跨度）。"""
        if not self.samples:
            return 0.0
        points = []
        for sample in self.samples:
            points.extend(_body_points(sample, self.spec))
        ys = [point[1] for point in points]
        return max(ys) - min(ys)

    def as_rows(self):
        rows = []
        for sample in self.samples:
            rows.append((sample.t, sample.x, sample.y, sample.tractor_yaw,
                         sample.trailer_yaw, sample.articulation_deg))
        return rows


def _body_points(sample, spec, half_width=0.90):
    """牵引车与挂车各四个角点，用于算占用宽度。"""
    points = []
    for along, lateral in ((0.0, -half_width), (0.0, half_width),
                           (spec.wheelbase, half_width),
                           (spec.wheelbase, -half_width)):
        points.append(_place(sample.x, sample.y, sample.tractor_yaw,
                             along, lateral))

    hx, hy = sample.hitch_point(spec)
    for along, lateral in ((0.0, -half_width), (0.0, half_width),
                           (spec.trailer_length, half_width),
                           (spec.trailer_length, -half_width)):
        points.append(_place(hx, hy, sample.trailer_yaw, along, lateral))
    return points


def _place(x, y, yaw, along, lateral):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (x + along * cos_yaw - lateral * sin_yaw,
            y + along * sin_yaw + lateral * cos_yaw)


# --------------------------------------------------------------------------
# 积分
# --------------------------------------------------------------------------

def _clamp_steer(spec, steer_deg):
    steer = rad(_as_float(steer_deg, u"steer_deg", DEFAULT_STEER_DEG))
    if abs(steer) > spec.max_steer:
        raise TrailerError(u"转角 %.3f rad 超过前轮最大转角 %.3f rad"
                           % (abs(steer), spec.max_steer))
    return steer


def steady_articulation_deg(spec, steer_deg=DEFAULT_STEER_DEG):
    """稳态铰接角的闭式解（度）。不存在稳态时返回 180.0。"""
    steer = _clamp_steer(spec, steer_deg)
    a = math.tan(steer) / spec.wheelbase
    ratio = spec.trailer_length * a / math.sqrt(1.0
                                                + (spec.hitch_offset * a) ** 2)
    if abs(ratio) > 1.0:
        return 180.0
    psi = math.atan(spec.hitch_offset * a)
    return abs(deg(psi + math.asin(ratio)))


def simulate(spec=None, speed=DEFAULT_SPEED, steer_deg=DEFAULT_STEER_DEG,
             seconds=DEFAULT_SECONDS, step=DEFAULT_STEP, gear=DEFAULT_GEAR,
             name=""):
    """按显式欧拉积分推进组合体，返回 Trajectory。"""
    spec = spec or TrailerSpec()
    speed = _positive(speed, u"speed", DEFAULT_SPEED)
    seconds = _positive(seconds, u"seconds", DEFAULT_SECONDS)
    step = _positive(step, u"step", DEFAULT_STEP)
    gear = _as_float(gear, u"gear", DEFAULT_GEAR)
    if gear not in GEARS:
        raise TrailerError(u"gear 只能是 1（前进）或 -1（倒车），收到 %r" % gear)

    steer = _clamp_steer(spec, steer_deg)
    velocity = speed * gear
    count = int(round(seconds / step)) + 1
    yaw_rate1 = velocity * math.tan(steer) / spec.wheelbase

    x, y = 0.0, 0.0
    tractor_yaw, trailer_yaw = 0.0, 0.0
    samples = [Sample(0.0, x, y, tractor_yaw, trailer_yaw, velocity)]

    for index in range(1, count):
        phi = normalize_angle(tractor_yaw - trailer_yaw)
        yaw_rate2 = (velocity * math.sin(phi)
                     - spec.hitch_offset * yaw_rate1 * math.cos(phi)) \
            / spec.trailer_length

        x += velocity * math.cos(tractor_yaw) * step
        y += velocity * math.sin(tractor_yaw) * step
        tractor_yaw = normalize_angle(tractor_yaw + yaw_rate1 * step)
        trailer_yaw = normalize_angle(trailer_yaw + yaw_rate2 * step)

        samples.append(Sample(index * step, x, y, tractor_yaw, trailer_yaw,
                              velocity))

    return Trajectory(samples, spec, name=name)


def integration_gap(spec=None, speed=DEFAULT_SPEED, steer_deg=DEFAULT_STEER_DEG,
                    seconds=DEFAULT_SECONDS, step=DEFAULT_STEP,
                    gear=DEFAULT_GEAR):
    """步长减半后与原始步长的最大位置偏差（用来判断积分够不够细）。"""
    if step <= 0.0:
        raise TrailerError(u"step 必须大于 0")
    coarse = simulate(spec, speed, steer_deg, seconds, step, gear)
    fine = simulate(spec, speed, steer_deg, seconds, step / 2.0, gear)
    gap = 0.0
    ratio = int(round(1.0 / (step / 2.0 / step)))  # = 2
    for index in range(coarse.count):
        fine_index = index * ratio
        if fine_index >= fine.count:
            break
        a = coarse.samples[index]
        b = fine.samples[fine_index]
        gap = max(gap, math.hypot(a.x - b.x, a.y - b.y))
    return gap


# --------------------------------------------------------------------------
# 判定
# --------------------------------------------------------------------------

def _result(name, passed, detail):
    return {"type": name, "passed": bool(passed), "detail": detail}


def check_trailer(trajectory, jackknife_deg=DEFAULT_JACKKNIFE_DEG,
                  channel_width=DEFAULT_CHANNEL_WIDTH,
                  step_tolerance=DEFAULT_STEP_TOLERANCE,
                  gear=DEFAULT_GEAR, speed=DEFAULT_SPEED,
                  steer_deg=DEFAULT_STEER_DEG, seconds=DEFAULT_SECONDS,
                  step=DEFAULT_STEP):
    """四条断言：折刀余量 / 积分精度 / 通道宽度 / 倒车稳定性。"""
    if trajectory is None or trajectory.is_empty():
        raise TrailerError(u"轨迹为空，无法判定")

    results = []
    spec = trajectory.spec

    peak = trajectory.articulation_max_deg()
    jackknife_deg = _positive(jackknife_deg, u"jackknife_deg",
                              DEFAULT_JACKKNIFE_DEG)
    results.append(_result(
        ASSERT_NAMES[0], peak < jackknife_deg,
        u"最大铰接角 %.2f 度，折刀角 %.2f 度（余量 %.2f 度）"
        % (peak, jackknife_deg, jackknife_deg - peak)))

    gap = integration_gap(spec, speed, steer_deg, seconds, step, gear)
    step_tolerance = _positive(step_tolerance, u"step_tolerance",
                               DEFAULT_STEP_TOLERANCE)
    results.append(_result(
        ASSERT_NAMES[1], gap <= step_tolerance,
        u"步长 %.3f 与 %.3f 的位置差 %.5f m，上限 %.3f m"
        % (step, step / 2.0, gap, step_tolerance)))

    needed = trajectory.channel_needed()
    channel_width = _positive(channel_width, u"channel_width",
                              DEFAULT_CHANNEL_WIDTH)
    results.append(_result(
        ASSERT_NAMES[2], needed <= channel_width,
        u"占用宽度 %.2f m，通道 %.2f m" % (needed, channel_width)))

    head = trajectory.head_articulation_deg()
    tail = trajectory.tail_articulation_deg()
    if int(gear) < 0:
        growing = tail > head + 1.0
        results.append(_result(
            ASSERT_NAMES[3], not growing,
            u"倒车：首段 %.2f 度 → 末段 %.2f 度%s"
            % (head, tail, u"，铰接角持续增大" if growing else u"，未发散")))
    else:
        results.append(_result(
            ASSERT_NAMES[3], True,
            u"前进：首段 %.2f 度 → 末段 %.2f 度，不适用倒车发散判定"
            % (head, tail)))

    return results


def all_passed(results):
    return all(item["passed"] for item in results)


# --------------------------------------------------------------------------
# 渲染
# --------------------------------------------------------------------------

def render_phase(trajectory, width=DEFAULT_GRID_WIDTH,
                 height=DEFAULT_GRID_HEIGHT,
                 jackknife_deg=DEFAULT_JACKKNIFE_DEG):
    """把铰接角随时间的曲线画成字符相图，返回行列表。"""
    if trajectory is None or trajectory.is_empty():
        return []

    values = trajectory.articulation_signed()
    limit = max(jackknife_deg, max(abs(value) for value in values) * 1.05)
    center = (height - 1) / 2.0
    grid = [[" "] * width for _ in range(height)]
    zero_row = int(round(center))
    for column in range(width):
        grid[zero_row][column] = "-"

    count = len(values)
    for index, value in enumerate(values):
        column = int(round(index * (width - 1) / float(max(1, count - 1))))
        row = int(round(center - (value / limit) * center))
        row = min(height - 1, max(0, row))
        grid[row][column] = "*"

    return [u"".join(row) for row in grid]


def render_body(trajectory, width=DEFAULT_GRID_WIDTH,
                height=DEFAULT_GRID_HEIGHT):
    """把牵引车与挂车的位置画成俯视字符图，返回行列表。"""
    if trajectory is None or trajectory.is_empty():
        return []

    stride = max(1, len(trajectory.samples) // 150)
    points = []
    for sample in trajectory.samples[::stride]:
        points.extend(_body_points(sample, trajectory.spec))

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(1e-6, max_x - min_x)
    span_y = max(1e-6, max_y - min_y)

    grid = [[" "] * width for _ in range(height)]
    for point in points:
        column = int((point[0] - min_x) / span_x * (width - 1))
        row = height - 1 - int((point[1] - min_y) / span_y * (height - 1))
        column = min(width - 1, max(0, column))
        row = min(height - 1, max(0, row))
        grid[row][column] = "#"

    grid[height - 1][0] = "o"
    return [u"".join(row) for row in grid]


def format_report(spec, trajectory, results, header=""):
    """生成终端报告（文本）。"""
    lines = []
    if header:
        lines.append(header)
    lines.append(u"=" * 60)
    lines.append(u"牵引车-挂车运动学与折刀判定（纯几何 + 显式欧拉，不依赖 CARLA）")
    lines.append(u"牵引车：轴距 %.2f m，铰接偏移 %.2f m，最大转角 %.2f 度"
                 % (spec.wheelbase, spec.hitch_offset, spec.max_steer_deg))
    lines.append(u"挂车：铰接点到轴线 %.2f m，组合体约 %.2f m"
                 % (spec.trailer_length, spec.total_length))
    lines.append(u"最小转弯半径：%.3f m" % (spec.min_turn_radius,))
    if trajectory is not None and not trajectory.is_empty():
        first = trajectory.first()
        last = trajectory.last()
        lines.append(u"采样：%d 个点，时长 %.2f s，速度 %.2f m/s"
                     % (trajectory.count, trajectory.duration(), first.speed))
        lines.append(u"起点：(%.3f, %.3f)，终点：(%.3f, %.3f)"
                     % (first.x, first.y, last.x, last.y))
        lines.append(u"牵引车转角：%.2f 度"
                     % (trajectory.turn_angle_deg(),))
        lines.append(u"铰接角：最大 %.2f 度，末段 %.2f 度，首段 %.2f 度"
                     % (trajectory.articulation_max_deg(),
                        trajectory.tail_articulation_deg(),
                        trajectory.head_articulation_deg()))
        lines.append(u"占用通道宽度：%.2f m" % (trajectory.channel_needed(),))

    passed = sum(1 for item in results if item["passed"])
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(results) - passed, len(results)))
    for item in results:
        flag = u"[PASS]" if item["passed"] else u"[FAIL]"
        lines.append(u"%s %-8s %s" % (flag, item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED"))
    return u"\n".join(lines)


# --------------------------------------------------------------------------
# 参数
# --------------------------------------------------------------------------

PARAM_KEYS = (
    "trailer/wheelbase",
    "trailer/length",
    "trailer/hitch_offset",
    "trailer/max_steer_deg",
    "check/jackknife_deg",
    "check/channel_width",
    "check/step_tolerance",
    "sim/speed",
    "sim/steer_deg",
    "sim/seconds",
    "sim/step",
    "sim/gear",
    "rate/publish_hz",
)

PARAM_DEFAULTS = {
    "trailer/wheelbase": DEFAULT_WHEELBASE,
    "trailer/length": DEFAULT_TRAILER_LENGTH,
    "trailer/hitch_offset": DEFAULT_HITCH_OFFSET,
    "trailer/max_steer_deg": DEFAULT_MAX_STEER_DEG,
    "check/jackknife_deg": DEFAULT_JACKKNIFE_DEG,
    "check/channel_width": DEFAULT_CHANNEL_WIDTH,
    "check/step_tolerance": DEFAULT_STEP_TOLERANCE,
    "sim/speed": DEFAULT_SPEED,
    "sim/steer_deg": DEFAULT_STEER_DEG,
    "sim/seconds": DEFAULT_SECONDS,
    "sim/step": DEFAULT_STEP,
    "sim/gear": DEFAULT_GEAR,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
}


class TrailerParams(object):
    """从 ROS 参数服务器读到的全部配置。"""

    def __init__(self, **kwargs):
        for key in PARAM_KEYS:
            value = kwargs.get(key, PARAM_DEFAULTS[key])
            # ROS 参数可能以字符串传进来，统一在构造时转成 float
            setattr(self, self._attr(key),
                    _as_float(value, key, PARAM_DEFAULTS[key]))
        self.spec = TrailerSpec(self.trailer_wheelbase,
                                self.trailer_length,
                                self.trailer_hitch_offset,
                                self.trailer_max_steer_deg)

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
        return "TrailerParams(%s)" % ", ".join("%s=%r" % pair for pair in pairs)

# ------------------------------------------------------------------ TF 广播
DEFAULT_TF_ENABLED = True
DEFAULT_TF_RATE_HZ = 10.0
DEFAULT_TF_WORLD_FRAME = u"map"
DEFAULT_TF_TRACTOR_FRAME = u"tractor"
DEFAULT_TF_TRAILER_FRAME = u"trailer"


def trailer_in_tractor(sample, spec):
    """挂车轴在牵引车坐标系（原点在后轴、x 朝车头）里的位置与相对航向。"""
    axle_x, axle_y = sample.trailer_point(spec)
    dx = axle_x - sample.x
    dy = axle_y - sample.y
    cos_yaw = math.cos(sample.tractor_yaw)
    sin_yaw = math.sin(sample.tractor_yaw)
    local_x = dx * cos_yaw + dy * sin_yaw
    local_y = -dx * sin_yaw + dy * cos_yaw
    return (local_x, local_y,
            normalize_angle(sample.trailer_yaw - sample.tractor_yaw))


def frames_for(trajectory, world=DEFAULT_TF_WORLD_FRAME,
               tractor=DEFAULT_TF_TRACTOR_FRAME,
               trailer=DEFAULT_TF_TRAILER_FRAME):
    """两条变换：(parent, child, x, y, yaw)，可直接喂给 tf2 广播器。"""
    last = trajectory.last()
    local_x, local_y, local_yaw = trailer_in_tractor(last, trajectory.spec)
    return [
        (world, tractor, last.x, last.y, normalize_angle(last.tractor_yaw)),
        (tractor, trailer, local_x, local_y, local_yaw),
    ]
