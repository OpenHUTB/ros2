#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""车辆转弯扫掠包络的计算与校验（纯标准库，不 import rospy）。

术语（一律以「后轴中心到转弯中心的距离」R 为基准，单位米）：

    内后轮半径   = R - 半轮距
    外后轮半径   = R + 半轮距
    内前轮半径   = hypot(R - 半轮距, 轴距)
    外前轮半径   = hypot(R + 半轮距, 轴距)
    外前角半径   = hypot(R + 半轮距, 轴距 + 前悬)
    内前角半径   = hypot(R - 半轮距, 轴距 + 前悬)
    外后角半径   = hypot(R + 半轮距, 后悬)
    内后角半径   = hypot(R - 半轮距, 后悬)

车身绕转弯中心扫过的是一个圆环：外半径取上面所有半径的最大值，
内半径取最小值，两者之差就是「走这个弯至少需要多宽的通道」。
内轮差按教科书定义取「内前轮半径 - 内后轮半径」。
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
    "DEFAULT_WHEELBASE", "DEFAULT_TRACK", "DEFAULT_FRONT_OVERHANG", "DEFAULT_REAR_OVERHANG",
    "DEFAULT_VEHICLE_WIDTH", "DEFAULT_MAX_STEER", "DEFAULT_CHANNEL_WIDTH",
    "DEFAULT_INNER_CLEARANCE", "DEFAULT_MAX_INNER_DIFF", "DEFAULT_PUBLISH_HZ",
    "DEFAULT_TOPIC_REPORT", "DEFAULT_TOPIC_RING", "DEFAULT_GRID_WIDTH", "DEFAULT_GRID_HEIGHT",
    "PARAM_KEYS",
    "PARAM_DEFAULTS", "TURN_ERROR", "TURN_RADII", "ASSERT_NAMES",
    "TurningError", "VehicleSpec", "TurningParams", "sweep", "radii_at_limit",
    "check_sweep", "all_passed", "render_ring", "format_report", "to_message_text",
]

DEFAULT_WHEELBASE = 2.70
DEFAULT_TRACK = 1.54
DEFAULT_FRONT_OVERHANG = 0.90
DEFAULT_REAR_OVERHANG = 0.85
DEFAULT_VEHICLE_WIDTH = 1.80
DEFAULT_MAX_STEER = 0.55
DEFAULT_CHANNEL_WIDTH = 6.0
DEFAULT_INNER_CLEARANCE = 0.30
DEFAULT_MAX_INNER_DIFF = 1.00
DEFAULT_PUBLISH_HZ = 1.0
DEFAULT_TOPIC_REPORT = "/turning/sweep"
DEFAULT_TOPIC_RING = "/turning/ring"
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 30

PARAM_DEFAULTS = {
    "wheelbase": DEFAULT_WHEELBASE,
    "track": DEFAULT_TRACK,
    "front_overhang": DEFAULT_FRONT_OVERHANG,
    "rear_overhang": DEFAULT_REAR_OVERHANG,
    "vehicle_width": DEFAULT_VEHICLE_WIDTH,
    "max_steer": DEFAULT_MAX_STEER,
    "turn_radius": 8.0,
    "channel_width": DEFAULT_CHANNEL_WIDTH,
    "inner_clearance": DEFAULT_INNER_CLEARANCE,
    "max_inner_diff": DEFAULT_MAX_INNER_DIFF,
    "publish_hz": DEFAULT_PUBLISH_HZ,
    "topic_report": DEFAULT_TOPIC_REPORT,
    "topic_ring": DEFAULT_TOPIC_RING,
}

PARAM_KEYS = (
    "vehicle/wheelbase",
    "vehicle/track",
    "vehicle/front_overhang",
    "vehicle/rear_overhang",
    "vehicle/width",
    "vehicle/max_steer",
    "turning/radius",
    "check/channel_width",
    "check/inner_clearance",
    "check/max_inner_diff",
    "rate/publish_hz",
    "topic/report",
    "topic/ring",
)

TURN_RADII = (
    "inner_rear",
    "outer_rear",
    "inner_front",
    "outer_front",
    "outer_front_corner",
    "inner_front_corner",
    "outer_rear_corner",
    "inner_rear_corner",
)

ASSERT_NAMES = (u"转弯可行性", u"通道宽度", u"内轮差", u"内侧余量")


class TurningError(ValueError):
    """参数不合法或场景无法计算。"""


# ------------------------------------------------------------------ 小工具


def _as_float(value, name, default=None):
    if value is None or (isinstance(value, text_type) and not value.strip()):
        if default is None:
            raise TurningError(u"%s 不能为空" % name)
        return float(default)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise TurningError(u"%s 不是数字：%r" % (name, value))
    if math.isnan(result) or math.isinf(result):
        raise TurningError(u"%s 不是有限数：%r" % (name, value))
    return result


def _positive(value, name):
    result = _as_float(value, name)
    if result <= 0.0:
        raise TurningError(u"%s 必须大于 0：%r" % (name, result))
    return result


def _non_negative(value, name):
    result = _as_float(value, name)
    if result < 0.0:
        raise TurningError(u"%s 不能为负：%r" % (name, result))
    return result


def to_message_text(text):
    """std_msgs/String 用：Py2 要 bytes，Py3 要 str。"""
    if isinstance(text, bytes):
        return text
    if text_type is not str:  # Python 2
        return text.encode("utf-8")
    return text


# ------------------------------------------------------------------ 车辆


class VehicleSpec(object):
    """车辆几何参数。"""

    __slots__ = ("wheelbase", "track", "front_overhang", "rear_overhang", "width", "max_steer")

    def __init__(self, wheelbase=DEFAULT_WHEELBASE, track=DEFAULT_TRACK,
                 front_overhang=DEFAULT_FRONT_OVERHANG, rear_overhang=DEFAULT_REAR_OVERHANG,
                 width=DEFAULT_VEHICLE_WIDTH, max_steer=DEFAULT_MAX_STEER):
        self.wheelbase = _positive(wheelbase, "vehicle.wheelbase")
        self.track = _positive(track, "vehicle.track")
        self.front_overhang = _non_negative(front_overhang, "vehicle.front_overhang")
        self.rear_overhang = _non_negative(rear_overhang, "vehicle.rear_overhang")
        self.width = _positive(width, "vehicle.width")
        self.max_steer = _positive(max_steer, "vehicle.max_steer")
        if self.max_steer >= math.pi / 2.0:
            raise TurningError(u"vehicle.max_steer 必须小于 90 度")
        if self.width + 1e-9 < self.track:
            raise TurningError(u"车宽 %.3f 小于轮距 %.3f" % (self.width, self.track))

    # -- 派生量 ------------------------------------------------------------
    @property
    def length(self):
        return self.front_overhang + self.wheelbase + self.rear_overhang

    @property
    def half_track(self):
        return self.track / 2.0

    @property
    def min_radius(self):
        """后轴中心的最小转弯半径（按最大前轮转角算）。"""
        return self.wheelbase / math.tan(self.max_steer)

    @property
    def body_half_width(self):
        return self.width / 2.0

    def as_tuple(self):
        return (self.wheelbase, self.track, self.front_overhang, self.rear_overhang,
                self.width, self.max_steer)

    def __repr__(self):
        return ("VehicleSpec(wheelbase=%.2f, track=%.2f, front_overhang=%.2f, "
                "rear_overhang=%.2f, width=%.2f, max_steer=%.3f)" % self.as_tuple())


# ------------------------------------------------------------------ 扫掠计算


def sweep(vehicle, radius):
    """给定后轴中心转弯半径，返回各关键点的扫掠半径与派生指标。"""
    radius = _positive(radius, "turning.radius")
    half = vehicle.half_track
    inner = radius - half
    outer = radius + half
    if inner <= 0.0:
        raise TurningError(u"转弯半径 %.3f 小于半轮距 %.3f，几何上不可能" % (radius, half))

    front = vehicle.wheelbase
    front_edge = vehicle.wheelbase + vehicle.front_overhang
    rear_edge = vehicle.rear_overhang

    radii = {
        "inner_rear": inner,
        "outer_rear": outer,
        "inner_front": math.hypot(inner, front),
        "outer_front": math.hypot(outer, front),
        "outer_front_corner": math.hypot(outer, front_edge),
        "inner_front_corner": math.hypot(inner, front_edge),
        "outer_rear_corner": math.hypot(outer, rear_edge),
        "inner_rear_corner": math.hypot(inner, rear_edge),
    }
    outer_max = max(radii.values())
    inner_min = min(radii.values())
    result = {
        "radius": radius,
        "radii": radii,
        "outer_max": outer_max,
        "inner_min": inner_min,
        "swept_width": outer_max - inner_min,
        "inner_diff": radii["inner_front"] - radii["inner_rear"],
        "outer_swing": radii["outer_front_corner"] - radii["outer_front"],
        "steer_angle": math.atan(vehicle.wheelbase / radius),
        "vehicle": vehicle,
    }
    return result


def radii_at_limit(vehicle):
    """最小转弯半径下的一组扫掠量，用来对比「打满方向盘」时的占用宽度。"""
    return sweep(vehicle, vehicle.min_radius + 1e-6)


def check_sweep(vehicle, result, channel_width=DEFAULT_CHANNEL_WIDTH,
                inner_clearance=DEFAULT_INNER_CLEARANCE,
                max_inner_diff=DEFAULT_MAX_INNER_DIFF):
    """四条断言，返回 [{type, passed, detail}]。"""
    radius = result["radius"]
    results = []

    passed = radius + 1e-9 >= vehicle.min_radius
    results.append({
        "type": u"转弯可行性",
        "passed": passed,
        "detail": u"后轴半径 %.3f m，最小可行 %.3f m" % (radius, vehicle.min_radius),
    })

    channel_width = _positive(channel_width, "check.channel_width")
    passed = result["swept_width"] <= channel_width + 1e-9
    results.append({
        "type": u"通道宽度",
        "passed": passed,
        "detail": u"需要 %.3f m，通道 %.3f m" % (result["swept_width"], channel_width),
    })

    max_inner_diff = _positive(max_inner_diff, "check.max_inner_diff")
    passed = result["inner_diff"] <= max_inner_diff + 1e-9
    results.append({
        "type": u"内轮差",
        "passed": passed,
        "detail": u"内轮差 %.3f m，上限 %.3f m" % (result["inner_diff"], max_inner_diff),
    })

    inner_clearance = _non_negative(inner_clearance, "check.inner_clearance")
    passed = (result["inner_min"] - inner_clearance) > -1e-9
    results.append({
        "type": u"内侧余量",
        "passed": passed,
        "detail": u"内侧最小半径 %.3f m，要求 >= %.3f m" % (result["inner_min"], inner_clearance),
    })
    return results


def all_passed(results):
    for item in results:
        if not item["passed"]:
            return False
    return True


# ------------------------------------------------------------------ 字符图


def render_ring(vehicle, result, width=DEFAULT_GRID_WIDTH, height=DEFAULT_GRID_HEIGHT):
    """把扫掠圆环画成字符图：'#' 内圈之内，'.' 圆环（扫掠区），' ' 外圈之外。

    转弯中心固定在网格中心，第一象限里车在 (R, 0) 处（车头朝 +y 方向绕行）。
    """
    width = int(width)
    height = int(height)
    outer = result["outer_max"]
    inner = result["inner_min"]
    if width < 8 or height < 4:
        raise TurningError(u"字符图尺寸太小：%d x %d" % (width, height))

    scale = outer * 1.05
    rows = []
    for row in range(height):
        line = []
        y = scale * (1.0 - 2.0 * row / (height - 1))
        for col in range(width):
            x = scale * (2.0 * col / (width - 1) - 1.0)
            distance = math.hypot(x, y)
            if distance < inner:
                line.append(u"#")
            elif distance <= outer:
                line.append(u".")
            else:
                line.append(u" ")
        rows.append(u"".join(line))
    return rows


# ------------------------------------------------------------------ 报告


def format_report(vehicle, result, results, header=""):
    lines = []
    if header:
        lines.append(header)
    lines.append(u"车辆：轴距 %.2f m，轮距 %.2f m，车长 %.2f m，车宽 %.2f m，最大转角 %.3f rad"
                 % (vehicle.wheelbase, vehicle.track, vehicle.length, vehicle.width,
                    vehicle.max_steer))
    lines.append(u"转弯：后轴中心半径 %.3f m，等效前轮转角 %.3f rad"
                 % (result["radius"], result["steer_angle"]))
    lines.append(u"半径：内后轮 %.3f / 外后轮 %.3f / 内前轮 %.3f / 外前轮 %.3f"
                 % (result["radii"]["inner_rear"], result["radii"]["outer_rear"],
                    result["radii"]["inner_front"], result["radii"]["outer_front"]))
    lines.append(u"扫掠：内圈 %.3f m，外圈 %.3f m，需要通道宽度 %.3f m"
                 % (result["inner_min"], result["outer_max"], result["swept_width"]))
    lines.append(u"其他：内轮差 %.3f m，外摆 %.3f m，最小可行半径 %.3f m"
                 % (result["inner_diff"], result["outer_swing"], vehicle.min_radius))

    passed = sum(1 for item in results if item["passed"])
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %-6s %s" % (u"PASS" if item["passed"] else u"FAIL",
                                          item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED"))
    return u"\n".join(lines)


# ------------------------------------------------------------------ 参数


class TurningParams(object):
    """节点参数（与 launch 参数一一对应）。"""

    __slots__ = tuple(PARAM_DEFAULTS.keys())

    def __init__(self, **kwargs):
        for key, default in PARAM_DEFAULTS.items():
            setattr(self, key, kwargs.get(key, default))

    @classmethod
    def from_params(cls, get_param):
        return cls(
            wheelbase=_positive(get_param("vehicle/wheelbase", DEFAULT_WHEELBASE),
                                "vehicle/wheelbase"),
            track=_positive(get_param("vehicle/track", DEFAULT_TRACK), "vehicle/track"),
            front_overhang=_non_negative(get_param("vehicle/front_overhang",
                                                   DEFAULT_FRONT_OVERHANG),
                                         "vehicle/front_overhang"),
            rear_overhang=_non_negative(get_param("vehicle/rear_overhang", DEFAULT_REAR_OVERHANG),
                                        "vehicle/rear_overhang"),
            vehicle_width=_positive(get_param("vehicle/width", DEFAULT_VEHICLE_WIDTH),
                                    "vehicle/width"),
            max_steer=_positive(get_param("vehicle/max_steer", DEFAULT_MAX_STEER),
                                "vehicle/max_steer"),
            turn_radius=_positive(get_param("turning/radius", PARAM_DEFAULTS["turn_radius"]),
                                  "turning/radius"),
            channel_width=_positive(get_param("check/channel_width", DEFAULT_CHANNEL_WIDTH),
                                    "check/channel_width"),
            inner_clearance=_non_negative(get_param("check/inner_clearance",
                                                    DEFAULT_INNER_CLEARANCE),
                                          "check/inner_clearance"),
            max_inner_diff=_positive(get_param("check/max_inner_diff", DEFAULT_MAX_INNER_DIFF),
                                     "check/max_inner_diff"),
            publish_hz=_positive(get_param("rate/publish_hz", DEFAULT_PUBLISH_HZ),
                                 "rate/publish_hz"),
            topic_report=get_param("topic/report", DEFAULT_TOPIC_REPORT),
            topic_ring=get_param("topic/ring", DEFAULT_TOPIC_RING),
        )

    def vehicle(self):
        return VehicleSpec(self.wheelbase, self.track, self.front_overhang,
                           self.rear_overhang, self.vehicle_width, self.max_steer)

    def as_tuple(self):
        return tuple(getattr(self, key) for key in PARAM_DEFAULTS)

    def __repr__(self):
        return "TurningParams(%s)" % ", ".join("%s=%r" % (key, getattr(self, key))
                                              for key in PARAM_DEFAULTS)
