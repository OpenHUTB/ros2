#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""里程计轨迹记录/回放/比对的核心逻辑（纯标准库，不 import rospy）。

流程是「记录 -> 落盘 -> 重采样 -> 回放 -> 与参考轨迹比对」五段，
与「算一次就打印」的工具有本质区别：

* 有状态：轨迹是一串带时间戳的采样点，边收边记；
* 有磁盘：记录存成 CSV，可读回来再比对（回放不依赖原始话题）；
* 有反馈：比对结果给出最大误差与最差点位置，可判断记录是否可信。
"""

from __future__ import division
from __future__ import print_function

import math


class TraceError(ValueError):
    """轨迹相关参数或数据不合法时抛出。"""


DEFAULT_RECORD_SECONDS = 8.0
DEFAULT_RECORD_RATE = 20.0
DEFAULT_MOCK_RADIUS = 6.0
DEFAULT_MOCK_SPEED = 2.0
DEFAULT_MIN_SAMPLES = 100
DEFAULT_MAX_ERROR = 0.20
DEFAULT_LENGTH_TOLERANCE = 0.50
DEFAULT_REFERENCE_RADIUS = DEFAULT_MOCK_RADIUS
DEFAULT_CSV_PATH = "/tmp/trace.csv"
DEFAULT_HISTOGRAM_BINS = 8
DEFAULT_HISTOGRAM_WIDTH = 40
DEFAULT_TOPIC_ODOM = "/odom"
DEFAULT_TOPIC_REPLAY = "/trace/replay"
DEFAULT_TOPIC_REPORT = "/trace/report"

ASSERT_NAMES = (u"采样完整", u"里程合理", u"回放一致", u"误差在限")

PARAM_KEYS = (
    "record/seconds",
    "record/rate",
    "mock/enabled",
    "mock/radius",
    "mock/speed",
    "check/min_samples",
    "check/max_error",
    "check/length_tolerance",
    "check/reference_radius",
    "trace/csv_path",
    "topic/odom",
    "topic/replay",
    "topic/report",
)

PARAM_DEFAULTS = {
    "record/seconds": DEFAULT_RECORD_SECONDS,
    "record/rate": DEFAULT_RECORD_RATE,
    "mock/enabled": True,
    "mock/radius": DEFAULT_MOCK_RADIUS,
    "mock/speed": DEFAULT_MOCK_SPEED,
    "check/min_samples": DEFAULT_MIN_SAMPLES,
    "check/max_error": DEFAULT_MAX_ERROR,
    "check/length_tolerance": DEFAULT_LENGTH_TOLERANCE,
    "check/reference_radius": DEFAULT_REFERENCE_RADIUS,
    "trace/csv_path": DEFAULT_CSV_PATH,
    "topic/odom": DEFAULT_TOPIC_ODOM,
    "topic/replay": DEFAULT_TOPIC_REPLAY,
    "topic/report": DEFAULT_TOPIC_REPORT,
}

_EPS = 1e-9


# ------------------------------------------------------------------ 小工具


def _as_float(value, name, default=None):
    if value is None:
        if default is None:
            raise TraceError(u"参数 %s 不能为空" % name)
        return float(default)
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        result = float(value)
    else:
        text = u"%s" % value
        text = text.strip()
        if not text:
            if default is None:
                raise TraceError(u"参数 %s 不能为空" % name)
            return float(default)
        try:
            result = float(text)
        except ValueError:
            raise TraceError(u"参数 %s 不是数字：%r" % (name, value))
    if result != result or result in (float("inf"), float("-inf")):
        raise TraceError(u"参数 %s 不是有限数：%r" % (name, value))
    return result


def _positive(value, name):
    result = _as_float(value, name)
    if result <= 0.0:
        raise TraceError(u"参数 %s 必须大于 0，当前 %g" % (name, result))
    return result


def _non_negative(value, name):
    result = _as_float(value, name)
    if result < 0.0:
        raise TraceError(u"参数 %s 不能为负，当前 %g" % (name, result))
    return result


def as_bool(value, default=False):
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    text = u"%s" % value
    text = text.strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise TraceError(u"参数不是布尔值：%r" % value)


def normalize_angle(angle):
    value = math.fmod(angle + math.pi, 2.0 * math.pi)
    if value < 0.0:
        value += 2.0 * math.pi
    return value - math.pi


def to_message_text(text):
    try:
        return text.encode("utf-8")
    except (AttributeError, UnicodeDecodeError):
        return text


# ------------------------------------------------------------------ 采样点


class TracePoint(object):
    """一个采样点：时间、位置、朝向。"""

    __slots__ = ("t", "x", "y", "yaw")

    def __init__(self, t, x, y, yaw=0.0):
        self.t = _as_float(t, "t", 0.0)
        self.x = _as_float(x, "x")
        self.y = _as_float(y, "y")
        self.yaw = normalize_angle(_as_float(yaw, "yaw", 0.0))

    def as_row(self):
        return [self.t, self.x, self.y, self.yaw]

    def __repr__(self):
        return "TracePoint(%.3f, %.3f, %.3f, %.3f)" % (self.t, self.x, self.y, self.yaw)


class Trace(object):
    """一串按时间排序的采样点。"""

    __slots__ = ("points", "name")

    def __init__(self, points, name=""):
        collected = []
        for item in points:
            if isinstance(item, TracePoint):
                collected.append(item)
            else:
                collected.append(TracePoint(*item))
        if not collected:
            raise TraceError(u"轨迹至少要有一个点")
        for index in range(len(collected) - 1):
            if collected[index + 1].t + _EPS < collected[index].t:
                raise TraceError(u"第 %d 个点的时间倒退了" % (index + 1))
        self.points = collected
        self.name = name

    @property
    def count(self):
        return len(self.points)

    def duration(self):
        return self.points[-1].t - self.points[0].t

    def length(self):
        total = 0.0
        for index in range(len(self.points) - 1):
            first = self.points[index]
            second = self.points[index + 1]
            total += math.hypot(second.x - first.x, second.y - first.y)
        return total

    def bounds(self):
        xs = [item.x for item in self.points]
        ys = [item.y for item in self.points]
        return (min(xs), max(xs), min(ys), max(ys))

    def first(self):
        return self.points[0]

    def last(self):
        return self.points[-1]

    def errors_against(self, reference):
        """逐点与参考轨迹比位置误差（要求点数相同）。"""
        if self.count != reference.count:
            raise TraceError(u"点数不等：%d vs %d，请先重采样" % (
                self.count, reference.count))
        return [math.hypot(actual.x - expected.x, actual.y - expected.y)
                for actual, expected in zip(self.points, reference.points)]

    def to_csv_lines(self):
        lines = [u"t,x,y,yaw"]
        for item in self.points:
            lines.append(u"%.4f,%.4f,%.4f,%.4f" % (item.t, item.x, item.y, item.yaw))
        return lines

    def csv_text(self):
        return u"\n".join(self.to_csv_lines()) + u"\n"

    def as_rows(self):
        return [item.as_row() for item in self.points]

    def __repr__(self):
        return "Trace(%d points, %.3f m)" % (self.count, self.length())


def parse_trace_text(text, name=""):
    """从 CSV 文本读回轨迹；第一行可以是表头。"""
    if not text:
        raise TraceError(u"CSV 内容是空的")
    rows = []
    for index, line in enumerate(text.splitlines()):
        stripped = line.strip()
        if not stripped:
            continue
        parts = [item.strip() for item in stripped.split(",")]
        if len(parts) < 3:
            raise TraceError(u"第 %d 行列数不足：%r" % (index + 1, line))
        if index == 0 and not parts[0].replace(".", "").replace("-", "").isdigit():
            continue
        rows.append(TracePoint(parts[0], parts[1], parts[2],
                               parts[3] if len(parts) > 3 else 0.0))
    if not rows:
        raise TraceError(u"CSV 里没有数据行")
    return Trace(rows, name)


# ------------------------------------------------------------------ 模拟真值


def mock_trace(radius=DEFAULT_MOCK_RADIUS, speed=DEFAULT_MOCK_SPEED,
               seconds=DEFAULT_RECORD_SECONDS, rate=DEFAULT_RECORD_RATE):
    """生成一条左转圆弧作为「真值」：x = R sin θ，y = R (1 - cos θ)。"""
    radius_value = _positive(radius, "radius")
    speed_value = _positive(speed, "speed")
    duration = _positive(seconds, "seconds")
    sample_rate = _positive(rate, "rate")

    count = int(round(duration * sample_rate)) + 1
    points = []
    for index in range(count):
        time = index / sample_rate
        if time > duration:
            time = duration
        theta = speed_value * time / radius_value
        points.append(TracePoint(time,
                                 radius_value * math.sin(theta),
                                 radius_value * (1.0 - math.cos(theta)),
                                 theta))
    return Trace(points, "mock_r%.2f" % radius_value)


def integration_trace(radius=DEFAULT_MOCK_RADIUS, speed=DEFAULT_MOCK_SPEED,
                      seconds=DEFAULT_RECORD_SECONDS, rate=DEFAULT_RECORD_RATE):
    """用定步长积分「开」一条圆弧轨迹（模拟里程计输出，带离散误差）。

    与 mock_trace 的解析真值相比会有 O(dt) 量级的偏差，正好用来验证比对环节
    真的在工作，而不是两边用同一个公式自证。
    """
    radius_value = _positive(radius, "radius")
    speed_value = _positive(speed, "speed")
    duration = _positive(seconds, "seconds")
    sample_rate = _positive(rate, "rate")

    dt = 1.0 / sample_rate
    count = int(round(duration * sample_rate)) + 1
    yaw_rate = speed_value / radius_value
    x = 0.0
    y = 0.0
    yaw = 0.0
    points = [TracePoint(0.0, x, y, yaw)]
    for index in range(1, count):
        yaw += yaw_rate * dt
        x += speed_value * math.cos(yaw) * dt
        y += speed_value * math.sin(yaw) * dt
        points.append(TracePoint(index * dt, x, y, yaw))
    return Trace(points, "integration_r%.2f" % radius_value)


def resample(trace, count):
    """按等时间步长重采样（线性插值位置与朝向）。"""
    wanted = int(count)
    if wanted < 2:
        raise TraceError(u"重采样点数至少为 2")
    if trace.count == 1:
        return Trace([trace.points[0]] * wanted, trace.name)

    start = trace.points[0]
    end = trace.points[-1]
    span = end.t - start.t
    result = []
    cursor = 0
    for index in range(wanted):
        ratio = index / (wanted - 1.0)
        time = start.t + span * ratio
        while cursor < trace.count - 2 and trace.points[cursor + 1].t < time:
            cursor += 1
        left = trace.points[cursor]
        right = trace.points[cursor + 1]
        step = right.t - left.t
        if step <= _EPS:
            weight = 0.0
        else:
            weight = (time - left.t) / step
        result.append(TracePoint(
            time,
            left.x + (right.x - left.x) * weight,
            left.y + (right.y - left.y) * weight,
            left.yaw + normalize_angle(right.yaw - left.yaw) * weight))
    return Trace(result, trace.name)


def compare(reference, actual):
    """比较两条点数相同的轨迹。"""
    errors = actual.errors_against(reference)
    worst = 0
    for index in range(1, len(errors)):
        if errors[index] > errors[worst]:
            worst = index
    return {
        "max_error": max(errors),
        "mean_error": sum(errors) / len(errors),
        "worst_index": worst,
        "worst_point": actual.points[worst],
    }


# ------------------------------------------------------------------ 判定


def check_trace(trace, reference, min_samples=DEFAULT_MIN_SAMPLES,
                max_error=DEFAULT_MAX_ERROR,
                length_tolerance=DEFAULT_LENGTH_TOLERANCE):
    """四条断言：采样完整 / 里程合理 / 回放一致 / 误差在限。"""
    required = int(_positive(min_samples, "min_samples"))
    allowance = _positive(max_error, "max_error")
    length_tolerance_value = _non_negative(length_tolerance, "length_tolerance")

    results = []

    enough = trace.count >= required
    results.append({
        "type": ASSERT_NAMES[0],
        "passed": enough,
        "detail": u"记录 %d 个点，要求 >= %d" % (trace.count, required),
    })

    reference_length = reference.length()
    actual_length = trace.length()
    length_ok = abs(actual_length - reference_length) <= length_tolerance_value
    results.append({
        "type": ASSERT_NAMES[1],
        "passed": length_ok,
        "detail": u"里程 %.3f m，参考 %.3f m（容差 %.3f m）" % (
            actual_length, reference_length, length_tolerance_value),
    })

    doubled = resample(trace, max(2, trace.count * 2))
    replay = resample(doubled, trace.count)
    replay_stats = compare(trace, replay)
    replay_ok = replay_stats["max_error"] <= allowance
    results.append({
        "type": ASSERT_NAMES[2],
        "passed": replay_ok,
        "detail": u"加密 2 倍再回采样，往返偏差 %.4f m（阈值 %.3f m）" % (
            replay_stats["max_error"], allowance),
    })

    if trace.count == reference.count:
        stats = compare(reference, trace)
        worst = stats["worst_point"]
        worst_ok = stats["max_error"] <= allowance
        detail = u"最大误差 %.4f m（第 %d 点，%.2f, %.2f），阈值 %.3f m" % (
            stats["max_error"], stats["worst_index"] + 1, worst.x, worst.y, allowance)
    else:
        stats = {"max_error": float("inf"), "mean_error": float("inf"),
                 "worst_index": -1, "worst_point": trace.last()}
        worst_ok = False
        detail = u"点数与参考不一致（%d vs %d），无法逐点比对" % (
            trace.count, reference.count)
    results.append({"type": ASSERT_NAMES[3], "passed": worst_ok, "detail": detail})

    return results


def all_passed(results):
    return all(item["passed"] for item in results)


def histogram(values, bins=DEFAULT_HISTOGRAM_BINS, width=DEFAULT_HISTOGRAM_WIDTH):
    """把误差画成横向柱状图（这一段的形态与其他模块的曲线图不同）。"""
    count = int(bins)
    span = int(width)
    if count < 2:
        raise TraceError(u"分箱数至少为 2")
    if span < 10:
        raise TraceError(u"柱状宽度至少为 10")
    if not values:
        return [u"（没有数据）"]

    low = min(values)
    high = max(values)
    if high - low < _EPS:
        high = low + 1e-6
    step = (high - low) / count
    buckets = [0] * count
    for value in values:
        index = int((value - low) / step)
        if index >= count:
            index = count - 1
        if index < 0:
            index = 0
        buckets[index] += 1

    peak = max(buckets) or 1
    lines = []
    for index in range(count):
        left = low + step * index
        right = left + step
        bar = int(round(buckets[index] / peak * span))
        lines.append(u"[%6.3f, %6.3f) %-*s %d" % (
            left, right, span, u"#" * bar, buckets[index]))
    return lines


def format_report(trace, reference, results, stats, csv_path=u"", header=u""):
    lines = []
    if header:
        lines.append(header)
    first = trace.first()
    last = trace.last()
    left, right, bottom, top = trace.bounds()
    lines.append(u"记录：%d 个点，时长 %.2f s，里程 %.3f m" % (
        trace.count, trace.duration(), trace.length()))
    lines.append(u"起点：(%.3f, %.3f)，终点：(%.3f, %.3f)" % (
        first.x, first.y, last.x, last.y))
    lines.append(u"包围盒：x ∈ [%.3f, %.3f]，y ∈ [%.3f, %.3f]" % (
        left, right, bottom, top))
    if csv_path:
        lines.append(u"落盘：%s（%d 行数据）" % (csv_path, trace.count))
    lines.append(u"参考：%s（%d 个点，%.3f m）" % (
        reference.name or u"内置真值", reference.count, reference.length()))
    lines.append(u"比对：最大误差 %.4f m，平均误差 %.4f m，最差点第 %d 个" % (
        stats["max_error"], stats["mean_error"], stats["worst_index"] + 1))
    passed = len([item for item in results if item["passed"]])
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）" % (
        passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %-4s %s" % (
            u"PASS" if item["passed"] else u"FAIL", item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED"))
    return u"\n".join(lines)


# ------------------------------------------------------------------ 参数


class TraceParams(object):
    """从参数服务器读到的配置。"""

    __slots__ = ("seconds", "rate", "use_mock", "mock_radius", "mock_speed",
                 "min_samples", "max_error", "length_tolerance",
                 "reference_radius", "csv_path", "topic_odom", "topic_replay",
                 "topic_report")

    def __init__(self, **kwargs):
        self.seconds = _positive(
            kwargs.get("seconds", PARAM_DEFAULTS["record/seconds"]), "seconds")
        self.rate = _positive(kwargs.get("rate", PARAM_DEFAULTS["record/rate"]),
                              "rate")
        self.use_mock = as_bool(kwargs.get("use_mock", PARAM_DEFAULTS["mock/enabled"]),
                                True)
        self.mock_radius = _positive(
            kwargs.get("mock_radius", PARAM_DEFAULTS["mock/radius"]), "mock_radius")
        self.mock_speed = _positive(
            kwargs.get("mock_speed", PARAM_DEFAULTS["mock/speed"]), "mock_speed")
        self.min_samples = int(_positive(
            kwargs.get("min_samples", PARAM_DEFAULTS["check/min_samples"]),
            "min_samples"))
        self.max_error = _positive(
            kwargs.get("max_error", PARAM_DEFAULTS["check/max_error"]), "max_error")
        self.length_tolerance = _non_negative(
            kwargs.get("length_tolerance", PARAM_DEFAULTS["check/length_tolerance"]),
            "length_tolerance")
        self.reference_radius = _positive(
            kwargs.get("reference_radius", PARAM_DEFAULTS["check/reference_radius"]),
            "reference_radius")
        self.csv_path = u"%s" % kwargs.get("csv_path", PARAM_DEFAULTS["trace/csv_path"])
        self.topic_odom = u"%s" % kwargs.get("topic_odom", PARAM_DEFAULTS["topic/odom"])
        self.topic_replay = u"%s" % kwargs.get(
            "topic_replay", PARAM_DEFAULTS["topic/replay"])
        self.topic_report = u"%s" % kwargs.get(
            "topic_report", PARAM_DEFAULTS["topic/report"])

    @classmethod
    def from_params(cls, get_param):
        values = {}
        mapping = {
            "record/seconds": "seconds",
            "record/rate": "rate",
            "mock/enabled": "use_mock",
            "mock/radius": "mock_radius",
            "mock/speed": "mock_speed",
            "check/min_samples": "min_samples",
            "check/max_error": "max_error",
            "check/length_tolerance": "length_tolerance",
            "check/reference_radius": "reference_radius",
            "trace/csv_path": "csv_path",
            "topic/odom": "topic_odom",
            "topic/replay": "topic_replay",
            "topic/report": "topic_report",
        }
        for key, attribute in mapping.items():
            values[attribute] = get_param(key, PARAM_DEFAULTS[key])
        return cls(**values)

    def reference(self):
        """参考轨迹：与记录同样的时长与采样率，但用参考半径。"""
        return mock_trace(self.reference_radius, self.mock_speed,
                          self.seconds, self.rate)

    def as_tuple(self):
        return (self.seconds, self.rate, self.use_mock, self.mock_radius,
                self.mock_speed, self.min_samples, self.max_error,
                self.length_tolerance, self.reference_radius, self.csv_path,
                self.topic_odom, self.topic_replay, self.topic_report)

    def __repr__(self):
        return "TraceParams(seconds=%.2f, rate=%.2f, radius=%.2f)" % (
            self.seconds, self.rate, self.mock_radius)
