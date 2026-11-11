#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""速度曲线规划的核心逻辑（纯标准库，不 import rospy）。

给定「要开多远、巡航速度多少、加/减速度上限多少」，规划出一条
加速 → 匀速 → 减速 的速度曲线，并判断它是否满足限速、加速度、
距离与时间预算四条要求。

距离太短、来不及加到巡航速度时，会自动退化成三角形的
「加速 → 减速」曲线，峰值速度由距离反解得到。
"""

from __future__ import division
from __future__ import print_function


class ProfileError(ValueError):
    """参数不合法时抛出。"""


DEFAULT_DISTANCE = 120.0
DEFAULT_CRUISE_KMH = 40.0
DEFAULT_START_KMH = 0.0
DEFAULT_ACCEL = 1.2
DEFAULT_DECEL = 2.0
DEFAULT_SPEED_LIMIT_KMH = 50.0
DEFAULT_TIME_BUDGET = 30.0
DEFAULT_TOLERANCE = 0.5
DEFAULT_PUBLISH_HZ = 5.0
DEFAULT_DT = 0.05
DEFAULT_TOPIC_PROFILE = "/speed/profile"
DEFAULT_TOPIC_REPORT = "/speed/report"
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 12

ASSERT_NAMES = (u"限速", u"加速度", u"距离", u"时间")

PARAM_KEYS = (
    "scene/distance",
    "scene/cruise_speed_kmh",
    "scene/start_speed_kmh",
    "vehicle/accel",
    "vehicle/decel",
    "check/speed_limit_kmh",
    "check/time_budget",
    "check/tolerance",
    "sim/dt",
    "rate/publish_hz",
    "topic/profile",
    "topic/report",
)

PARAM_DEFAULTS = {
    "scene/distance": DEFAULT_DISTANCE,
    "scene/cruise_speed_kmh": DEFAULT_CRUISE_KMH,
    "scene/start_speed_kmh": DEFAULT_START_KMH,
    "vehicle/accel": DEFAULT_ACCEL,
    "vehicle/decel": DEFAULT_DECEL,
    "check/speed_limit_kmh": DEFAULT_SPEED_LIMIT_KMH,
    "check/time_budget": DEFAULT_TIME_BUDGET,
    "check/tolerance": DEFAULT_TOLERANCE,
    "sim/dt": DEFAULT_DT,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
    "topic/profile": DEFAULT_TOPIC_PROFILE,
    "topic/report": DEFAULT_TOPIC_REPORT,
}

_EPS = 1e-9


# ------------------------------------------------------------------ 小工具


def _as_float(value, name, default=None):
    if value is None:
        if default is None:
            raise ProfileError(u"参数 %s 不能为空" % name)
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
                raise ProfileError(u"参数 %s 不能为空" % name)
            return float(default)
        try:
            result = float(text)
        except ValueError:
            raise ProfileError(u"参数 %s 不是数字：%r" % (name, value))
    if result != result or result in (float("inf"), float("-inf")):
        raise ProfileError(u"参数 %s 不是有限数：%r" % (name, value))
    return result


def _positive(value, name):
    result = _as_float(value, name)
    if result <= 0.0:
        raise ProfileError(u"参数 %s 必须大于 0，当前 %g" % (name, result))
    return result


def _non_negative(value, name):
    result = _as_float(value, name)
    if result < 0.0:
        raise ProfileError(u"参数 %s 不能为负，当前 %g" % (name, result))
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
    raise ProfileError(u"参数不是布尔值：%r" % value)


def kmh_to_mps(kmh):
    return _as_float(kmh, "speed_kmh") / 3.6


def mps_to_kmh(mps):
    return _as_float(mps, "speed_mps") * 3.6


def to_message_text(text):
    """给 std_msgs/String 用：py2 返回 byte str，py3 保持 str。"""
    try:
        return text.encode("utf-8")
    except (AttributeError, UnicodeDecodeError):
        return text


# ------------------------------------------------------------------ 曲线


class Profile(object):
    """一条加速 → 匀速 → 减速的速度曲线。"""

    __slots__ = (
        "kind",
        "distance",
        "cruise_kmh",
        "start_kmh",
        "accel",
        "decel",
        "accel_time",
        "accel_distance",
        "cruise_time",
        "cruise_distance",
        "decel_time",
        "decel_distance",
    )

    def __init__(self, kind, distance, cruise_kmh, start_kmh, accel, decel,
                 accel_time, accel_distance, cruise_time, cruise_distance,
                 decel_time, decel_distance):
        self.kind = kind
        self.distance = distance
        self.cruise_kmh = cruise_kmh
        self.start_kmh = start_kmh
        self.accel = accel
        self.decel = decel
        self.accel_time = accel_time
        self.accel_distance = accel_distance
        self.cruise_time = cruise_time
        self.cruise_distance = cruise_distance
        self.decel_time = decel_time
        self.decel_distance = decel_distance

    @property
    def total_time(self):
        return self.accel_time + self.cruise_time + self.decel_time

    @property
    def average_speed_mps(self):
        total = self.total_time
        if total <= 0.0:
            return 0.0
        return self.distance / total

    @property
    def peak_speed_mps(self):
        return self.cruise_kmh / 3.6

    @property
    def peak_speed_kmh(self):
        return self.cruise_kmh

    def speed_at(self, elapsed):
        """返回 elapsed 秒时的车速（m/s），超出区间按端点值取。"""
        time = _as_float(elapsed, "elapsed", 0.0)
        if time < 0.0:
            time = 0.0
        start = self.start_kmh / 3.6
        peak = self.peak_speed_mps
        if self.accel_time > 0.0 and time <= self.accel_time:
            return start + self.accel * time
        if time <= self.accel_time + self.cruise_time:
            return peak
        rest = time - self.accel_time - self.cruise_time
        if self.decel_time > 0.0 and rest <= self.decel_time:
            return max(0.0, peak - self.decel * rest)
        return 0.0

    def distance_at(self, elapsed):
        """返回 elapsed 秒时已经驶过的距离（m）。"""
        time = _as_float(elapsed, "elapsed", 0.0)
        if time < 0.0:
            time = 0.0
        start = self.start_kmh / 3.6
        peak = self.peak_speed_mps
        if self.accel_time > 0.0 and time <= self.accel_time:
            return start * time + 0.5 * self.accel * time * time
        travelled = self.accel_distance
        rest = time - self.accel_time
        if rest <= self.cruise_time:
            return travelled + peak * rest
        travelled += self.cruise_distance
        rest -= self.cruise_time
        if self.decel_time > 0.0 and rest <= self.decel_time:
            return travelled + peak * rest - 0.5 * self.decel * rest * rest
        return self.distance

    def samples(self, dt=DEFAULT_DT):
        """按 dt 采样，返回 [(t, speed, distance)]。"""
        step = _positive(dt, "dt")
        count = int(round(self.total_time / step)) + 1
        result = []
        for index in range(count):
            time = index * step
            if time > self.total_time:
                time = self.total_time
            result.append((time, self.speed_at(time), self.distance_at(time)))
        return result

    def as_tuple(self):
        return (self.kind, self.distance, self.cruise_kmh, self.start_kmh,
                self.accel, self.decel, self.total_time)

    def __repr__(self):
        return "Profile(%s, %.3f m, peak %.3f km/h, %.3f s)" % (
            self.kind, self.distance, self.cruise_kmh, self.total_time)


def _distances(start_speed, cruise_speed, accel, decel):
    accel_distance = max(0.0, (cruise_speed * cruise_speed
                               - start_speed * start_speed) / (2.0 * accel))
    decel_distance = cruise_speed * cruise_speed / (2.0 * decel)
    return accel_distance, decel_distance


def _make(kind, distance, cruise_kmh, start_kmh, accel, decel, cruise_time):
    cruise_speed = cruise_kmh / 3.6
    start_speed = start_kmh / 3.6
    accel_time = (cruise_speed - start_speed) / accel if cruise_speed > start_speed else 0.0
    accel_distance = max(0.0, (cruise_speed * cruise_speed
                               - start_speed * start_speed) / (2.0 * accel))
    decel_time = cruise_speed / decel
    decel_distance = cruise_speed * cruise_speed / (2.0 * decel)
    return Profile(kind, distance, cruise_kmh, start_kmh, accel, decel,
                   accel_time, accel_distance, cruise_time,
                   cruise_speed * cruise_time, decel_time, decel_distance)


def build_profile(distance, cruise_kmh, start_kmh=0.0, accel=DEFAULT_ACCEL,
                  decel=DEFAULT_DECEL):
    """规划速度曲线；距离不够时自动降峰值（三角形）。"""
    length = _positive(distance, "distance")
    cruise = _positive(cruise_kmh, "cruise_speed_kmh")
    start = _non_negative(start_kmh, "start_speed_kmh")
    accel_value = _positive(accel, "accel")
    decel_value = _positive(decel, "decel")
    if start > cruise:
        raise ProfileError(u"初速 %g km/h 高于巡航速度 %g km/h" % (start, cruise))

    cruise_speed = cruise / 3.6
    accel_distance, decel_distance = _distances(start / 3.6, cruise_speed,
                                                accel_value, decel_value)
    if accel_distance + decel_distance <= length + _EPS:
        cruise_distance = length - accel_distance - decel_distance
        return _make("trapezoid", length, cruise, start, accel_value, decel_value,
                     cruise_distance / cruise_speed)

    # 三角形：峰值速度由「加速段 + 减速段刚好覆盖整段距离」反解
    start_speed = start / 3.6
    coefficient = 1.0 / (2.0 * accel_value) + 1.0 / (2.0 * decel_value)
    peak_squared = (length + start_speed * start_speed / (2.0 * accel_value)) / coefficient
    if peak_squared <= 0.0:
        raise ProfileError(u"距离 %g m 太短，规划不出速度曲线" % length)
    peak = peak_squared ** 0.5
    if peak > cruise_speed:
        peak = cruise_speed
    return _make("triangle", length, peak * 3.6, start, accel_value, decel_value, 0.0)


# ------------------------------------------------------------------ 判定


def check_profile(profile, speed_limit_kmh=DEFAULT_SPEED_LIMIT_KMH,
                  time_budget=DEFAULT_TIME_BUDGET,
                  tolerance=DEFAULT_TOLERANCE):
    """四条断言：限速 / 加速度 / 距离 / 时间。"""
    limit = _positive(speed_limit_kmh, "speed_limit_kmh")
    budget = _positive(time_budget, "time_budget")
    allowance = _non_negative(tolerance, "tolerance")

    results = []

    peak_ok = profile.peak_speed_kmh <= limit + _EPS
    results.append({
        "type": EXPECT_LIMIT,
        "passed": peak_ok,
        "detail": u"峰值 %.2f km/h，限速 %.2f km/h" % (profile.peak_speed_kmh, limit),
    })

    accel_ok = profile.accel > 0.0 and profile.decel > 0.0
    results.append({
        "type": EXPECT_ACCEL,
        "passed": accel_ok,
        "detail": u"加速 %.2f m/s^2、减速 %.2f m/s^2" % (profile.accel, profile.decel),
    })

    planned = profile.accel_distance + profile.cruise_distance + profile.decel_distance
    distance_ok = abs(planned - profile.distance) <= allowance
    results.append({
        "type": EXPECT_DISTANCE,
        "passed": distance_ok,
        "detail": u"规划 %.2f m，目标 %.2f m（容差 %.2f m）" % (
            planned, profile.distance, allowance),
    })

    time_ok = profile.total_time <= budget + _EPS
    results.append({
        "type": EXPECT_TIME,
        "passed": time_ok,
        "detail": u"用时 %.2f s，预算 %.2f s" % (profile.total_time, budget),
    })

    return results


EXPECT_LIMIT = u"限速"
EXPECT_ACCEL = u"加速度"
EXPECT_DISTANCE = u"距离"
EXPECT_TIME = u"时间"


def candidate_cruises(cruise_kmh, speed_limit_kmh):
    """对比用的巡航阶梯：当前值上下各取一点，再带上限速本身。"""
    base = _positive(cruise_kmh, "cruise_kmh")
    limit = _positive(speed_limit_kmh, "speed_limit_kmh")
    raw = [base - 20.0, base - 10.0, base, base + 20.0, base + 30.0, limit]
    result = []
    for value in raw:
        if value < 5.0:
            value = 5.0
        value = round(value, 1)
        if any(abs(value - seen) < 1e-6 for seen in result):
            continue
        result.append(value)
    result.sort()
    return result


def compare_plans(distance, cruise_kmh, accel, decel, speed_limit_kmh,
                  start_kmh=0.0):
    """同一段路按几种巡航速度各排一次，返回按用时升序的对比表。"""
    limit = _positive(speed_limit_kmh, "speed_limit_kmh")
    plans = []
    for candidate in candidate_cruises(cruise_kmh, limit):
        profile = build_profile(distance, candidate, start_kmh, accel, decel)
        plans.append({
            "cruise_kmh": candidate,
            "peak_kmh": profile.peak_speed_kmh,
            "total_time": profile.total_time,
            "average_kmh": profile.average_speed_mps * 3.6,
            "kind": profile.kind,
            "within_limit": profile.peak_speed_kmh <= limit + 1e-9,
        })
    plans.sort(key=lambda item: item["cruise_kmh"])
    return plans


def recommend_plan(plans):
    """推荐：不超限的方案里用时最短的；并列则取巡航更低的那条（更保守）。"""
    best = None
    for item in plans:
        if not item["within_limit"]:
            continue
        if best is None or item["total_time"] < best["total_time"] - 1e-9:
            best = item
    return best


def format_plans(plans, speed_limit_kmh, distance):
    lines = [u"方案对比（同一段 %.2f m，限速 %.2f km/h）：" % (
        distance, speed_limit_kmh)]
    for item in plans:
        lines.append(u"  巡航 %5.1f km/h -> 峰值 %5.1f km/h，"
                     u"用时 %6.2f s，平均 %5.2f km/h，%s%s" % (
                         item["cruise_kmh"], item["peak_kmh"],
                         item["total_time"], item["average_kmh"],
                         u"梯形" if item["kind"] == "trapezoid" else u"三角形",
                         u"" if item["within_limit"] else u"（超限）"))
    best = recommend_plan(plans)
    if best is None:
        lines.append(u"推荐：没有不超限的方案，请降低巡航速度或放宽限速")
    else:
        lines.append(u"推荐：%.2f km/h（用时 %.2f s，平均 %.2f km/h，"
                     u"峰值 %.2f km/h 未超限）" % (
                         best["cruise_kmh"], best["total_time"],
                         best["average_kmh"], best["peak_kmh"]))
    return lines


def all_passed(results):
    return all(item["passed"] for item in results)


def render_profile(profile, width=DEFAULT_GRID_WIDTH, height=DEFAULT_GRID_HEIGHT):
    """画速度曲线：`*` 是巡航高度，`#` 是曲线以下的部分。"""
    columns = int(width)
    rows = int(height)
    if columns < 20:
        raise ProfileError(u"宽度太小：%d" % columns)
    if rows < 4:
        raise ProfileError(u"高度太小：%d" % rows)

    peak = profile.peak_speed_mps
    if peak <= 0.0:
        raise ProfileError(u"峰值速度为 0，画不出曲线")

    grid = [[" "] * columns for _ in range(rows)]
    for column in range(columns):
        ratio = column / (columns - 1.0)
        time = ratio * profile.total_time
        speed = profile.speed_at(time)
        level = int(round(speed / peak * (rows - 1)))
        if level < 0:
            level = 0
        for row in range(rows):
            index = rows - 1 - row
            if index <= level:
                grid[row][column] = u"#" if index < level else u"*"

    lines = [u"".join(row) for row in grid]
    lines.append(u"0" + u"-" * (columns - 6) + u"%.0f s" % profile.total_time)
    return lines


def format_report(profile, results, header=u"", plans=None,
                  speed_limit_kmh=DEFAULT_SPEED_LIMIT_KMH):
    lines = []
    if header:
        lines.append(header)
    lines.append(u"曲线类型：%s" % (u"梯形（含匀速段）" if profile.kind == "trapezoid"
                              else u"三角形（距离不够，达不到巡航速度）"))
    lines.append(u"巡航：峰值 %.2f km/h，初速 %.2f km/h，规划距离 %.2f m" % (
        profile.peak_speed_kmh, profile.start_kmh, profile.distance))
    lines.append(u"分段：加速 %.2f s / %.2f m，匀速 %.2f s / %.2f m，"
                 u"减速 %.2f s / %.2f m" % (
                     profile.accel_time, profile.accel_distance,
                     profile.cruise_time, profile.cruise_distance,
                     profile.decel_time, profile.decel_distance))
    lines.append(u"总时长 %.2f s，平均车速 %.2f km/h" % (
        profile.total_time, profile.average_speed_mps * 3.6))
    passed = len([item for item in results if item["passed"]])
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）" % (
        passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %-4s %s" % (
            u"PASS" if item["passed"] else u"FAIL", item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED"))
    if plans:
        lines.append(u"")
        lines.extend(format_plans(plans, speed_limit_kmh, profile.distance))
    return u"\n".join(lines)


# ------------------------------------------------------------------ 参数


class SpeedParams(object):
    """从参数服务器读到的配置。"""

    __slots__ = ("distance", "cruise_kmh", "start_kmh", "accel", "decel",
                 "speed_limit_kmh", "time_budget", "tolerance", "dt",
                 "publish_hz", "topic_profile", "topic_report")

    def __init__(self, **kwargs):
        self.distance = _positive(
            kwargs.get("distance", PARAM_DEFAULTS["scene/distance"]), "distance")
        self.cruise_kmh = _positive(
            kwargs.get("cruise_kmh", PARAM_DEFAULTS["scene/cruise_speed_kmh"]),
            "cruise_kmh")
        self.start_kmh = _non_negative(
            kwargs.get("start_kmh", PARAM_DEFAULTS["scene/start_speed_kmh"]),
            "start_kmh")
        self.accel = _positive(
            kwargs.get("accel", PARAM_DEFAULTS["vehicle/accel"]), "accel")
        self.decel = _positive(
            kwargs.get("decel", PARAM_DEFAULTS["vehicle/decel"]), "decel")
        self.speed_limit_kmh = _positive(
            kwargs.get("speed_limit_kmh", PARAM_DEFAULTS["check/speed_limit_kmh"]),
            "speed_limit_kmh")
        self.time_budget = _positive(
            kwargs.get("time_budget", PARAM_DEFAULTS["check/time_budget"]),
            "time_budget")
        self.tolerance = _non_negative(
            kwargs.get("tolerance", PARAM_DEFAULTS["check/tolerance"]), "tolerance")
        self.dt = _positive(kwargs.get("dt", PARAM_DEFAULTS["sim/dt"]), "dt")
        self.publish_hz = _positive(
            kwargs.get("publish_hz", PARAM_DEFAULTS["rate/publish_hz"]), "publish_hz")
        self.topic_profile = u"%s" % kwargs.get(
            "topic_profile", PARAM_DEFAULTS["topic/profile"])
        self.topic_report = u"%s" % kwargs.get(
            "topic_report", PARAM_DEFAULTS["topic/report"])

    @classmethod
    def from_params(cls, get_param):
        values = {}
        mapping = {
            "scene/distance": "distance",
            "scene/cruise_speed_kmh": "cruise_kmh",
            "scene/start_speed_kmh": "start_kmh",
            "vehicle/accel": "accel",
            "vehicle/decel": "decel",
            "check/speed_limit_kmh": "speed_limit_kmh",
            "check/time_budget": "time_budget",
            "check/tolerance": "tolerance",
            "sim/dt": "dt",
            "rate/publish_hz": "publish_hz",
            "topic/profile": "topic_profile",
            "topic/report": "topic_report",
        }
        for key, attribute in mapping.items():
            values[attribute] = get_param(key, PARAM_DEFAULTS[key])
        return cls(**values)

    def profile(self):
        return build_profile(self.distance, self.cruise_kmh, self.start_kmh,
                             self.accel, self.decel)

    def as_tuple(self):
        return (self.distance, self.cruise_kmh, self.start_kmh, self.accel,
                self.decel, self.speed_limit_kmh, self.time_budget,
                self.tolerance, self.dt, self.publish_hz,
                self.topic_profile, self.topic_report)

    def __repr__(self):
        return "SpeedParams(distance=%.3f, cruise=%.3f, accel=%.3f)" % (
            self.distance, self.cruise_kmh, self.accel)
