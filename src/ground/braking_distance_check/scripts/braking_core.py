#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""制动距离与跟车间距的物理核算（纯标准库，不 import rospy）。

模型（教科书版本的停车距离公式）：

    反应距离 = v0 * t_r
    制动减速度 a = μ * g * cosθ + g * sinθ      （θ > 0 表示上坡，帮着刹车）
    制动距离 = v0^2 / (2a)
    停车距离 = 反应距离 + 制动距离
    跟车安全距离 = v0 * 2 s + 余量

全程只用到中学物理，所以结果可以手算复核，这也是本模块能被离线测试的原因。
"""

from __future__ import division
from __future__ import print_function

import io
import math
import os

try:  # Python 2 兼容
    text_type = unicode  # noqa: F821
except NameError:  # Python 3
    text_type = str

__all__ = [
    "DEFAULT_SPEED_KMH", "DEFAULT_REACTION_TIME", "DEFAULT_SPEED_LIMIT_KMH",
    "DEFAULT_FRICTION", "DEFAULT_SLOPE_DEG", "DEFAULT_AVAILABLE_DISTANCE", "DEFAULT_GAP",
    "DEFAULT_GAP_SECONDS", "DEFAULT_GAP_MARGIN", "DEFAULT_MIN_FRICTION",
    "DEFAULT_PUBLISH_HZ", "DEFAULT_TOPIC_REPORT", "DEFAULT_TOPIC_BARS",
    "DEFAULT_GRAVITY", "PARAM_KEYS", "PARAM_DEFAULTS", "ASSERT_NAMES", "SPEED_TABLE_KMH",
    "BrakingError", "RoadSpec", "BrakingParams", "kmh_to_mps", "mps_to_kmh",
    "measure", "safety_gap", "check_braking", "all_passed", "speed_table",
    "render_bars", "format_report", "to_message_text",
]

DEFAULT_SPEED_KMH = 50.0
DEFAULT_REACTION_TIME = 1.0
DEFAULT_SPEED_LIMIT_KMH = 60.0
DEFAULT_FRICTION = 0.70
DEFAULT_SLOPE_DEG = 0.0
DEFAULT_AVAILABLE_DISTANCE = 40.0
DEFAULT_GAP = 32.0
DEFAULT_GAP_SECONDS = 2.0
DEFAULT_GAP_MARGIN = 3.0
DEFAULT_MIN_FRICTION = 0.30
DEFAULT_PUBLISH_HZ = 1.0
DEFAULT_TOPIC_REPORT = "/braking/report"
DEFAULT_TOPIC_BARS = "/braking/bars"
DEFAULT_GRAVITY = 9.81
SPEED_TABLE_KMH = (30.0, 50.0, 80.0, 100.0)

PARAM_DEFAULTS = {
    "speed_kmh": DEFAULT_SPEED_KMH,
    "reaction_time": DEFAULT_REACTION_TIME,
    "speed_limit_kmh": DEFAULT_SPEED_LIMIT_KMH,
    "friction": DEFAULT_FRICTION,
    "slope_deg": DEFAULT_SLOPE_DEG,
    "available_distance": DEFAULT_AVAILABLE_DISTANCE,
    "gap": DEFAULT_GAP,
    "publish_hz": DEFAULT_PUBLISH_HZ,
    "topic_report": DEFAULT_TOPIC_REPORT,
    "topic_bars": DEFAULT_TOPIC_BARS,
}

PARAM_KEYS = (
    "scene/speed_kmh",
    "vehicle/reaction_time",
    "vehicle/speed_limit_kmh",
    "road/friction",
    "road/slope_deg",
    "scene/available_distance",
    "scene/gap",
    "rate/publish_hz",
    "topic/report",
    "topic/bars",
)

ASSERT_NAMES = (u"停车距离", u"跟车间距", u"路面附着", u"限速")


class BrakingError(ValueError):
    """参数不合法。"""


# ------------------------------------------------------------------ 小工具


def _as_float(value, name, default=None):
    if value is None or (isinstance(value, text_type) and not value.strip()):
        if default is None:
            raise BrakingError(u"%s 不能为空" % name)
        return float(default)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise BrakingError(u"%s 不是数字：%r" % (name, value))
    if math.isnan(result) or math.isinf(result):
        raise BrakingError(u"%s 不是有限数：%r" % (name, value))
    return result


def _positive(value, name):
    result = _as_float(value, name)
    if result <= 0.0:
        raise BrakingError(u"%s 必须大于 0：%r" % (name, result))
    return result


def _non_negative(value, name):
    result = _as_float(value, name)
    if result < 0.0:
        raise BrakingError(u"%s 不能为负：%r" % (name, result))
    return result


def kmh_to_mps(speed_kmh):
    return _as_float(speed_kmh, "speed_kmh") / 3.6


def mps_to_kmh(speed_mps):
    return _as_float(speed_mps, "speed_mps") * 3.6


def to_message_text(text):
    """std_msgs/String 用：Py2 要 bytes，Py3 要 str。"""
    if isinstance(text, bytes):
        return text
    if text_type is not str:  # Python 2
        return text.encode("utf-8")
    return text


# ------------------------------------------------------------------ 路面


class RoadSpec(object):
    """路面：附着系数与坡度。"""

    __slots__ = ("friction", "slope_deg", "gravity")

    def __init__(self, friction=DEFAULT_FRICTION, slope_deg=DEFAULT_SLOPE_DEG,
                 gravity=DEFAULT_GRAVITY):
        self.friction = _positive(friction, "road.friction")
        self.slope_deg = _as_float(slope_deg, "road.slope_deg")
        self.gravity = _positive(gravity, "road.gravity")
        if self.friction > 1.5:
            raise BrakingError(u"road.friction 大于 1.5 不现实：%r" % self.friction)
        if abs(self.slope_deg) >= 45.0:
            raise BrakingError(u"road.slope_deg 绝对值必须小于 45 度：%r" % self.slope_deg)

    @property
    def slope_rad(self):
        return math.radians(self.slope_deg)

    @property
    def deceleration(self):
        """可用制动减速度：μ g cosθ + g sinθ（上坡为正、下坡为负）。"""
        value = (self.friction * self.gravity * math.cos(self.slope_rad)
                 + self.gravity * math.sin(self.slope_rad))
        if value <= 0.0:
            raise BrakingError(u"该坡度下车辆无法靠制动停下：坡度 %.2f 度" % self.slope_deg)
        return value

    def as_tuple(self):
        return (self.friction, self.slope_deg, self.gravity)

    def __repr__(self):
        return "RoadSpec(friction=%.3f, slope_deg=%.2f)" % (self.friction, self.slope_deg)


# ------------------------------------------------------------------ 核心计算


def measure(speed_kmh, reaction_time=DEFAULT_REACTION_TIME, road=None):
    """给定车速、反应时间与路面，返回停车距离的分解。"""
    road = road or RoadSpec()
    speed = kmh_to_mps(speed_kmh)
    if speed <= 0.0:
        raise BrakingError(u"车速必须大于 0：%r" % speed_kmh)
    reaction_time = _non_negative(reaction_time, "vehicle.reaction_time")

    deceleration = road.deceleration
    reaction_distance = speed * reaction_time
    brake_distance = speed * speed / (2.0 * deceleration)
    stop_distance = reaction_distance + brake_distance

    return {
        "speed_kmh": speed * 3.6,
        "speed_mps": speed,
        "reaction_time": reaction_time,
        "deceleration": deceleration,
        "reaction_distance": reaction_distance,
        "brake_distance": brake_distance,
        "stop_distance": stop_distance,
        "brake_time": speed / deceleration,
        "road": road,
    }


def safety_gap(speed_kmh, seconds=DEFAULT_GAP_SECONDS, margin=DEFAULT_GAP_MARGIN):
    """两秒法则 + 固定余量。"""
    seconds = _positive(seconds, "gap_seconds")
    margin = _non_negative(margin, "gap_margin")
    return kmh_to_mps(speed_kmh) * seconds + margin


def check_braking(result, available_distance=DEFAULT_AVAILABLE_DISTANCE,
                  gap=DEFAULT_GAP, speed_limit_kmh=DEFAULT_SPEED_LIMIT_KMH,
                  min_friction=DEFAULT_MIN_FRICTION):
    """四条互相独立的断言。"""
    results = []

    available_distance = _positive(available_distance, "scene.available_distance")
    passed = result["stop_distance"] <= available_distance + 1e-9
    results.append({
        "type": u"停车距离",
        "passed": passed,
        "detail": u"需要 %.2f m，前方可用 %.2f m" % (result["stop_distance"], available_distance),
    })

    gap = _non_negative(gap, "scene.gap")
    required = safety_gap(result["speed_kmh"])
    passed = gap + 1e-9 >= required
    results.append({
        "type": u"跟车间距",
        "passed": passed,
        "detail": u"当前 %.2f m，两秒法则要求 %.2f m" % (gap, required),
    })

    min_friction = _positive(min_friction, "road.min_friction")
    passed = result["road"].friction + 1e-9 >= min_friction
    results.append({
        "type": u"路面附着",
        "passed": passed,
        "detail": u"μ = %.2f，对应减速度 %.2f m/s^2，下限 μ = %.2f"
                  % (result["road"].friction, result["deceleration"], min_friction),
    })

    speed_limit_kmh = _positive(speed_limit_kmh, "vehicle.speed_limit_kmh")
    passed = result["speed_kmh"] <= speed_limit_kmh + 1e-9
    results.append({
        "type": u"限速",
        "passed": passed,
        "detail": u"当前 %.1f km/h，限速 %.1f km/h"
                  % (result["speed_kmh"], speed_limit_kmh),
    })
    return results


def all_passed(results):
    for item in results:
        if not item["passed"]:
            return False
    return True


def speed_table(road=None, reaction_time=DEFAULT_REACTION_TIME, speeds=SPEED_TABLE_KMH,
                available_distance=DEFAULT_AVAILABLE_DISTANCE):
    """一组车速下的停车距离表，用来说明「速度翻倍距离翻四倍」。"""
    rows = []
    for speed in speeds:
        item = measure(speed, reaction_time, road)
        rows.append({
            "speed_kmh": item["speed_kmh"],
            "reaction_distance": item["reaction_distance"],
            "brake_distance": item["brake_distance"],
            "stop_distance": item["stop_distance"],
            "fits": item["stop_distance"] <= available_distance + 1e-9,
        })
    return rows


# ------------------------------------------------------------------ 字符图


def render_bars(result, available_distance=DEFAULT_AVAILABLE_DISTANCE,
                width=60):
    """把「反应距离 | 制动距离 | 剩余」画成一行字符条。"""
    width = int(width)
    if width < 20:
        raise BrakingError(u"字符条太窄：%d" % width)
    scale = max(available_distance, result["stop_distance"])
    if scale <= 0.0:
        raise BrakingError(u"距离必须大于 0")

    reacted = int(round(result["reaction_distance"] / scale * width))
    braked = int(round(result["brake_distance"] / scale * width))
    reacted = max(0, min(width, reacted))
    braked = max(0, min(width - reacted, braked))
    stop_cells = reacted + braked
    limit_cells = max(0, min(width, int(round(available_distance / scale * width))))

    line = []
    for index in range(width):
        if index < reacted:
            line.append(u"=")
        elif index < stop_cells:
            line.append(u"#")
        elif index < limit_cells:
            line.append(u"-")
        else:
            line.append(u" ")
    bar = u"".join(line)
    ruler = u"0" + u" " * (width - len("%.1f" % scale) - 1) + u"%.1f m" % scale
    return [
        u"  " + bar,
        u"  " + ruler,
        u"  图例：= 反应距离（%.2f m）  # 制动距离（%.2f m）  - 前方尚可用的距离"
        % (result["reaction_distance"], result["brake_distance"]),
    ]


# ------------------------------------------------------------------ 报告


def format_report(result, results, rows=None, header=""):
    lines = []
    if header:
        lines.append(header)
    road = result["road"]
    lines.append(u"工况：车速 %.1f km/h（%.3f m/s），反应时间 %.2f s，路面 μ = %.2f，坡度 %+.1f 度"
                 % (result["speed_kmh"], result["speed_mps"], result["reaction_time"],
                    road.friction, road.slope_deg))
    lines.append(u"减速：可用制动减速度 %.2f m/s^2，刹停耗时 %.2f s"
                 % (result["deceleration"], result["brake_time"]))
    lines.append(u"距离：反应 %.2f m + 制动 %.2f m = 停车 %.2f m"
                 % (result["reaction_distance"], result["brake_distance"],
                    result["stop_distance"]))

    passed = sum(1 for item in results if item["passed"])
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %-6s %s" % (u"PASS" if item["passed"] else u"FAIL",
                                          item["type"], item["detail"]))

    if rows:
        lines.append(u"车速对照（同一路面、同一反应时间）：")
        for row in rows:
            lines.append(u"  %5.0f km/h -> 反应 %5.2f + 制动 %5.2f = %5.2f m  %s"
                         % (row["speed_kmh"], row["reaction_distance"],
                            row["brake_distance"], row["stop_distance"],
                            u"够" if row["fits"] else u"不够"))

    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED"))
    return u"\n".join(lines)


# ------------------------------------------------------------------ 参数


class BrakingParams(object):
    """节点参数（与 launch 参数一一对应）。"""

    __slots__ = tuple(PARAM_DEFAULTS.keys())

    def __init__(self, **kwargs):
        for key, default in PARAM_DEFAULTS.items():
            setattr(self, key, kwargs.get(key, default))

    @classmethod
    def from_params(cls, get_param):
        return cls(
            speed_kmh=_positive(get_param("scene/speed_kmh", DEFAULT_SPEED_KMH),
                                "scene/speed_kmh"),
            reaction_time=_non_negative(get_param("vehicle/reaction_time",
                                                  DEFAULT_REACTION_TIME),
                                        "vehicle/reaction_time"),
            speed_limit_kmh=_positive(get_param("vehicle/speed_limit_kmh",
                                                DEFAULT_SPEED_LIMIT_KMH),
                                      "vehicle/speed_limit_kmh"),
            friction=_positive(get_param("road/friction", DEFAULT_FRICTION), "road/friction"),
            slope_deg=_as_float(get_param("road/slope_deg", DEFAULT_SLOPE_DEG),
                                "road/slope_deg"),
            available_distance=_positive(get_param("scene/available_distance",
                                                   DEFAULT_AVAILABLE_DISTANCE),
                                         "scene/available_distance"),
            gap=_non_negative(get_param("scene/gap", DEFAULT_GAP), "scene/gap"),
            publish_hz=_positive(get_param("rate/publish_hz", DEFAULT_PUBLISH_HZ),
                                 "rate/publish_hz"),
            topic_report=get_param("topic/report", DEFAULT_TOPIC_REPORT),
            topic_bars=get_param("topic/bars", DEFAULT_TOPIC_BARS),
        )

    def road(self):
        return RoadSpec(self.friction, self.slope_deg)

    def as_tuple(self):
        return tuple(getattr(self, key) for key in PARAM_DEFAULTS)

    def __repr__(self):
        return "BrakingParams(%s)" % ", ".join("%s=%r" % (key, getattr(self, key))
                                              for key in PARAM_DEFAULTS)
