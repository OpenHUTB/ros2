#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""steering_actuator_lag 的核心计算：转向执行器的滞后、死区与回正。

特点
----
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 模型不是「一代了事」：一阶惯性 + 死区 + 速率限制 + 独立回正速率

模型
----
记指令转角为 cmd(t)、执行器实际转角为 theta(t)，误差 e = cmd - theta：

    |e| <= dead_band           -> 不动（死区）
    e > dead_band               -> 目标速率 = min( (e - 0)/tau, max_rate )
    e < -dead_band              -> 目标速率 = max( (e - 0)/tau, -max_rate )
    回正阶段（cmd == 0）        -> 目标速率 = clamp( -theta/tau, +-center_rate )

每步 theta += rate * dt，并把 |theta| 夹在 max_angle 以内。

为什么需要这一层
----------------
规划出来的「理想转角」和执行器真正打出来的角度不是一回事：有惯性（跟不上）、
有死区（小指令没反应）、有速率上限（打不快）、松手后还要回正。这些量直接决定
「一条轨迹跟不跟得上」，但在实车和 CARLA 里都要跑起来才知道。本模块把这一段
单独抽出来，用阶跃响应指标在离线环境里量清楚。
"""

from __future__ import division, print_function, unicode_literals

import io
import math


# --------------------------------------------------------------------------
# 默认参数
# --------------------------------------------------------------------------

DEFAULT_TIME_CONSTANT = 0.25      # 一阶惯性的时间常数 tau (s)
DEFAULT_DEAD_BAND_DEG = 0.30      # 死区半宽 (度)
DEFAULT_MAX_RATE_DEG = 60.0       # 打角时最大角速度 (度/s)
DEFAULT_CENTER_RATE_DEG = 40.0    # 回正时最大角速度 (度/s)
DEFAULT_MAX_ANGLE_DEG = 35.0      # 机械限位 (度)
DEFAULT_STEP_DEG = 20.0           # 阶跃指令幅值 (度)
DEFAULT_DWELL_SECONDS = 5.0        # 阶跃保持时长 (s)，之后指令归零回正
DEFAULT_STEADY_TOLERANCE = 0.50   # 稳态误差上限 (度)
DEFAULT_RISE_LIMIT = 1.00         # 到 90% 的时间上限 (s)
DEFAULT_CENTER_LIMIT = 1.50       # 指令归零后回到容差内的时间上限 (s)
DEFAULT_CENTER_TOLERANCE = 0.50   # 判「回正完成」的角度容差 (度)
DEFAULT_SIM_STEP = 0.01           # 积分步长 (s)
DEFAULT_PUBLISH_HZ = 10.0         # 发布频率
DEFAULT_GRID_WIDTH = 60           # 字符曲线宽度
DEFAULT_GRID_HEIGHT = 14          # 字符曲线高度

ASSERT_NAMES = (u"上升时间", u"稳态误差", u"速率上限", u"回正时间")


class ActuatorError(ValueError):
    """参数或场景不合法时抛出。"""


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------

def _as_float(value, name, default=None):
    if value is None or value == "":
        if default is None:
            raise ActuatorError(u"%s 不能为空" % name)
        return float(default)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ActuatorError(u"%s 必须是数字，收到 %r" % (name, value))
    if math.isnan(result) or math.isinf(result):
        raise ActuatorError(u"%s 必须是有限数字，收到 %r" % (name, value))
    return result


def _positive(value, name, default=None):
    result = _as_float(value, name, default)
    if result <= 0.0:
        raise ActuatorError(u"%s 必须大于 0，收到 %r" % (name, result))
    return result


def _non_negative(value, name, default=0.0):
    result = _as_float(value, name, default)
    if result < 0.0:
        raise ActuatorError(u"%s 不能为负，收到 %r" % (name, result))
    return result


def as_bool(value, default=False):
    if value is None or value == "":
        return bool(default)
    if isinstance(value, bool):
        return value
    text = value
    if not isinstance(text, type("")):
        text = u"%r" % (value,)
    return text.strip().lower() in (u"1", u"true", u"yes", u"on")


def clamp(value, low, high):
    return min(high, max(low, value))


def to_message_text(text):
    try:
        if isinstance(text, type(u"")) and not isinstance(text, type("")):
            return text.encode("utf-8")
    except Exception:
        pass
    return text


# --------------------------------------------------------------------------
# 执行器与采样
# --------------------------------------------------------------------------

class ActuatorSpec(object):
    """执行器的物理特性。"""

    def __init__(self, time_constant=DEFAULT_TIME_CONSTANT,
                 dead_band_deg=DEFAULT_DEAD_BAND_DEG,
                 max_rate_deg=DEFAULT_MAX_RATE_DEG,
                 center_rate_deg=DEFAULT_CENTER_RATE_DEG,
                 max_angle_deg=DEFAULT_MAX_ANGLE_DEG):
        self.time_constant = _positive(time_constant, u"time_constant")
        self.dead_band_deg = _non_negative(dead_band_deg, u"dead_band_deg")
        self.max_rate_deg = _positive(max_rate_deg, u"max_rate_deg")
        self.center_rate_deg = _positive(center_rate_deg, u"center_rate_deg")
        self.max_angle_deg = _positive(max_angle_deg, u"max_angle_deg")

    @property
    def max_rate(self):
        return math.radians(self.max_rate_deg)

    @property
    def center_rate(self):
        return math.radians(self.center_rate_deg)

    @property
    def dead_band(self):
        return math.radians(self.dead_band_deg)

    @property
    def max_angle(self):
        return math.radians(self.max_angle_deg)

    def as_tuple(self):
        return (self.time_constant, self.dead_band_deg,
                self.max_rate_deg, self.center_rate_deg, self.max_angle_deg)

    def __repr__(self):
        return ("ActuatorSpec(time_constant=%.3f, dead_band_deg=%.3f, "
                "max_rate_deg=%.3f, center_rate_deg=%.3f, max_angle_deg=%.3f)"
                % self.as_tuple())


class Sample(object):
    """一帧状态（角度单位是度，方便阅读）。"""

    def __init__(self, t, command_deg, angle_deg, rate_deg, centering):
        self.t = t
        self.command_deg = command_deg
        self.angle_deg = angle_deg
        self.rate_deg = rate_deg
        self.centering = centering

    def as_tuple(self):
        return (self.t, self.command_deg, self.angle_deg, self.rate_deg)

    def __repr__(self):
        return ("Sample(t=%.3f, cmd=%.3f, angle=%.3f, rate=%.3f)"
                % self.as_tuple())


class Response(object):
    """一次阶跃+回正测试的完整采样。"""

    def __init__(self, samples, spec, step_deg, dwell, name=""):
        self.samples = list(samples)
        self.spec = spec
        self.step_deg = step_deg
        self.dwell = dwell
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

    def hold_samples(self):
        """阶跃保持阶段的采样。"""
        return [s for s in self.samples if s.t < self.dwell]

    def center_samples(self):
        """回正阶段的采样。"""
        return [s for s in self.samples if s.t >= self.dwell]

    def angle_at(self, t):
        """最接近时刻 t 的角度（度）。"""
        if not self.samples:
            return 0.0
        best = self.samples[0]
        for sample in self.samples:
            if abs(sample.t - t) <= abs(best.t - t):
                best = sample
        return best.angle_deg

    def rise_time(self, fraction=0.9):
        """从 0 到 fraction*指令 所需时间（s）；达不到返回 None。"""
        if not self.samples or self.step_deg <= 0.0:
            return None
        target = fraction * self.step_deg
        for sample in self.samples:
            if sample.t >= self.dwell:
                break
            if sample.command_deg > 0.0 and sample.angle_deg >= target:
                return sample.t
        return None

    def overshoot(self):
        """阶跃段超过指令的最大量（度），不超调返回 0。"""
        hold = self.hold_samples()
        if not hold:
            return 0.0
        return max(0.0, max(s.angle_deg for s in hold) - self.step_deg)

    def steady_error(self):
        """阶跃段末端的角度误差（度）。"""
        hold = self.hold_samples()
        if not hold:
            return 0.0
        return abs(hold[-1].angle_deg - self.step_deg)

    def max_rate_deg(self):
        """全程实测最大角速度（度/s）。"""
        best = 0.0
        previous = None
        for sample in self.samples:
            if previous is not None and sample.t > previous.t:
                rate = abs(sample.angle_deg - previous.angle_deg) \
                    / (sample.t - previous.t)
                best = max(best, rate)
            previous = sample
        return best

    def center_time(self, tolerance=DEFAULT_CENTER_TOLERANCE):
        """指令归零后回到容差内所需时间（s）；未回正返回 None。"""
        for sample in self.center_samples():
            if abs(sample.angle_deg) <= tolerance:
                return sample.t - self.dwell
        return None

    def center_final_deg(self):
        last = self.last()
        return abs(last.angle_deg) if last is not None else 0.0

    def as_rows(self):
        return [sample.as_tuple() for sample in self.samples]


# --------------------------------------------------------------------------
# 仿真
# --------------------------------------------------------------------------

def command_at(t, step_deg, dwell):
    """指令：阶跃段给 step_deg，回正段给 0。"""
    return step_deg if t < dwell else 0.0


def simulate(spec=None, step_deg=DEFAULT_STEP_DEG,
             dwell=DEFAULT_DWELL_SECONDS, dt=DEFAULT_SIM_STEP,
             settle_seconds=None, name=""):
    """按显式欧拉积分执行器的响应，返回 Response。"""
    spec = spec or ActuatorSpec()
    step_deg = _as_float(step_deg, u"step_deg", DEFAULT_STEP_DEG)
    if step_deg <= 0.0:
        raise ActuatorError(u"step_deg 必须大于 0，收到 %r" % (step_deg,))
    if abs(step_deg) > spec.max_angle_deg:
        raise ActuatorError(u"step_deg %.3f 度超过机械限位 %.3f 度"
                            % (step_deg, spec.max_angle_deg))
    dwell = _positive(dwell, u"dwell", DEFAULT_DWELL_SECONDS)
    dt = _positive(dt, u"dt", DEFAULT_SIM_STEP)
    if settle_seconds is None:
        settle_seconds = max(2.0, 3.0 * spec.time_constant + 1.0)
    total = dwell + _positive(settle_seconds, u"settle_seconds")

    count = int(round(total / dt)) + 1
    angle = 0.0
    samples = [Sample(0.0, command_at(0.0, step_deg, dwell), angle, 0.0, False)]

    for index in range(1, count):
        t = index * dt
        command = command_at(t, step_deg, dwell)
        centering = command == 0.0
        error = command - angle

        if centering:
            rate = -angle / spec.time_constant
            rate = clamp(rate, -spec.center_rate_deg, spec.center_rate_deg)
        elif abs(error) <= spec.dead_band_deg:
            rate = 0.0
        else:
            rate = error / spec.time_constant
            rate = clamp(rate, -spec.max_rate_deg, spec.max_rate_deg)

        angle = angle + rate * dt
        angle = clamp(angle, -spec.max_angle_deg, spec.max_angle_deg)
        samples.append(Sample(t, command, angle, math.degrees(rate),
                              centering))

    return Response(samples, spec, step_deg, dwell, name=name)


# --------------------------------------------------------------------------
# 判定
# --------------------------------------------------------------------------

def _result(name, passed, detail):
    return {"type": name, "passed": bool(passed), "detail": detail}


def check_response(response, rise_limit=DEFAULT_RISE_LIMIT,
                   steady_tolerance=DEFAULT_STEADY_TOLERANCE,
                   center_limit=DEFAULT_CENTER_LIMIT,
                   center_tolerance=DEFAULT_CENTER_TOLERANCE,
                   rate_margin=1e-6):
    """四条断言：上升时间 / 稳态误差 / 速率上限 / 回正时间。"""
    if response is None or response.is_empty():
        raise ActuatorError(u"响应为空，无法判定")

    results = []

    rise = response.rise_time()
    rise_limit = _positive(rise_limit, u"rise_limit", DEFAULT_RISE_LIMIT)
    if rise is None:
        results.append(_result(
            ASSERT_NAMES[0], False,
            u"在阶跃段内没有到 90%%，上限 %.2f s" % (rise_limit,)))
    else:
        results.append(_result(
            ASSERT_NAMES[0], rise <= rise_limit,
            u"到 90%% 用时 %.3f s，上限 %.2f s" % (rise, rise_limit)))

    error = response.steady_error()
    steady_tolerance = _positive(steady_tolerance, u"steady_tolerance",
                                 DEFAULT_STEADY_TOLERANCE)
    results.append(_result(
        ASSERT_NAMES[1], error <= steady_tolerance,
        u"稳态误差 %.3f 度（死区 %.2f 度），容差 %.2f 度"
        % (error, response.spec.dead_band_deg, steady_tolerance)))

    measured = response.max_rate_deg()
    allowed = max(response.spec.max_rate_deg,
                  response.spec.center_rate_deg) * (1.0 + rate_margin)
    results.append(_result(
        ASSERT_NAMES[2], measured <= allowed,
        u"最大角速度 %.2f 度/s，上限 %.2f 度/s（打角 %.1f、回正 %.1f）"
        % (measured, allowed, response.spec.max_rate_deg,
           response.spec.center_rate_deg)))

    center = response.center_time(center_tolerance)
    center_limit = _positive(center_limit, u"center_limit",
                             DEFAULT_CENTER_LIMIT)
    if center is None:
        results.append(_result(
            ASSERT_NAMES[3], False,
            u"回正段没有回到 %.2f 度以内，上限 %.2f s"
            % (center_tolerance, center_limit)))
    else:
        results.append(_result(
            ASSERT_NAMES[3], center <= center_limit,
            u"回正到 %.2f 度以内用时 %.3f s，上限 %.2f s"
            % (center_tolerance, center, center_limit)))

    return results


def all_passed(results):
    return all(item["passed"] for item in results)


# --------------------------------------------------------------------------
# 渲染
# --------------------------------------------------------------------------

def render_curve(response, width=DEFAULT_GRID_WIDTH,
                 height=DEFAULT_GRID_HEIGHT):
    """把「指令 vs 实际角度」画成两条字符曲线，返回行列表。

    `o` 表示指令、`*` 表示实际角度、`-` 表示 0 度中线。
    """
    if response is None or response.is_empty():
        return []

    peak = max([abs(s.command_deg) for s in response.samples]
               + [abs(s.angle_deg) for s in response.samples] + [1e-6])
    grid = [[" "] * width for _ in range(height)]
    middle = (height - 1) / 2.0
    zero_row = int(round(middle))
    for column in range(width):
        grid[zero_row][column] = "-"

    count = response.count
    for index, sample in enumerate(response.samples):
        column = int(round(index * (width - 1) / float(max(1, count - 1))))
        for marker, value in (("o", sample.command_deg),
                              ("*", sample.angle_deg)):
            row = int(round(middle - (value / peak) * middle))
            row = min(height - 1, max(0, row))
            if grid[row][column] in (" ", "-") or marker == "*":
                grid[row][column] = marker

    return [u"".join(row) for row in grid]


def render_rate(response, width=DEFAULT_GRID_WIDTH,
                height=DEFAULT_GRID_HEIGHT):
    """把角速度画成字符柱状图，返回行列表。"""
    if response is None or response.is_empty():
        return []

    rates = [sample.rate_deg for sample in response.samples]
    peak = max(max(abs(rate) for rate in rates), 1e-6)
    grid = [[" "] * width for _ in range(height)]

    for column in range(width):
        index = int(round(column * (len(rates) - 1) / float(max(1, width - 1))))
        rate = rates[min(len(rates) - 1, max(0, index))]
        bars = int(round(abs(rate) / peak * (height - 1)))
        for row in range(bars):
            grid[height - 1 - row][column] = "#" if rate >= 0 else "="

    return [u"".join(row) for row in grid]


def format_report(spec, response, results, header=""):
    """生成终端报告（文本）。"""
    lines = []
    if header:
        lines.append(header)
    lines.append(u"=" * 60)
    lines.append(u"转向执行器滞后与回正（一阶惯性 + 死区 + 速率限制，不依赖 CARLA）")
    lines.append(u"执行器：时间常数 %.2f s，死区 ±%.2f 度，打角上限 %.1f 度/s，"
                 u"回正上限 %.1f 度/s，机械限位 %.1f 度"
                 % (spec.time_constant, spec.dead_band_deg, spec.max_rate_deg,
                    spec.center_rate_deg, spec.max_angle_deg))
    if response is not None and not response.is_empty():
        rise = response.rise_time()
        center = response.center_time()
        lines.append(u"阶跃：指令 %.2f 度，保持 %.2f s，之后归零回正"
                     % (response.step_deg, response.dwell))
        lines.append(u"采样：%d 个点，总时长 %.2f s"
                     % (response.count, response.duration()))
        lines.append(u"到 90%% 用时：%s，超调 %.3f 度"
                     % (u"%.3f s" % rise if rise is not None else u"未达到",
                        response.overshoot()))
        lines.append(u"稳态角度：%.3f 度，稳态误差 %.3f 度"
                     % (response.angle_at(response.dwell - 1e-9),
                        response.steady_error()))
        lines.append(u"最大角速度：%.2f 度/s"
                     % (response.max_rate_deg(),))
        lines.append(u"回正：归零后 %s，最终 %.3f 度"
                     % (u"%.3f s 回到容差内" % center if center is not None
                        else u"未回到容差内",
                        response.center_final_deg()))

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
    "actuator/time_constant",
    "actuator/dead_band_deg",
    "actuator/max_rate_deg",
    "actuator/center_rate_deg",
    "actuator/max_angle_deg",
    "test/step_deg",
    "test/dwell_seconds",
    "test/steady_tolerance_deg",
    "test/rise_limit_seconds",
    "test/center_limit_seconds",
    "test/center_tolerance_deg",
    "sim/step",
    "rate/publish_hz",
)

PARAM_DEFAULTS = {
    "actuator/time_constant": DEFAULT_TIME_CONSTANT,
    "actuator/dead_band_deg": DEFAULT_DEAD_BAND_DEG,
    "actuator/max_rate_deg": DEFAULT_MAX_RATE_DEG,
    "actuator/center_rate_deg": DEFAULT_CENTER_RATE_DEG,
    "actuator/max_angle_deg": DEFAULT_MAX_ANGLE_DEG,
    "test/step_deg": DEFAULT_STEP_DEG,
    "test/dwell_seconds": DEFAULT_DWELL_SECONDS,
    "test/steady_tolerance_deg": DEFAULT_STEADY_TOLERANCE,
    "test/rise_limit_seconds": DEFAULT_RISE_LIMIT,
    "test/center_limit_seconds": DEFAULT_CENTER_LIMIT,
    "test/center_tolerance_deg": DEFAULT_CENTER_TOLERANCE,
    "sim/step": DEFAULT_SIM_STEP,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
}


class ActuatorParams(object):
    """从 ROS 参数服务器读到的全部配置。"""

    def __init__(self, **kwargs):
        for key in PARAM_KEYS:
            value = kwargs.get(key, PARAM_DEFAULTS[key])
            setattr(self, self._attr(key),
                    _as_float(value, key, PARAM_DEFAULTS[key]))
        self.spec = ActuatorSpec(self.actuator_time_constant,
                                 self.actuator_dead_band_deg,
                                 self.actuator_max_rate_deg,
                                 self.actuator_center_rate_deg,
                                 self.actuator_max_angle_deg)

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
        return "ActuatorParams(%s)" % ", ".join("%s=%r" % pair
                                                for pair in pairs)
