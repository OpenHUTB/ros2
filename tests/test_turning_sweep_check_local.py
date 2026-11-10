#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""turning_sweep_check 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_turning_sweep_check_local.py

覆盖范围：
A 默认参数        B 小工具          C 车辆几何
D 扫掠半径        E 派生指标        F 四条断言
G 字符图          H 参数读取        I 报告格式
J 工程文件约定
"""

from __future__ import division
from __future__ import print_function

import io
import math
import os
import sys

try:  # Python 2 兼容
    text_type = unicode  # noqa: F821
except NameError:  # Python 3
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PKG = os.path.join(ROOT, "src", "ground", "turning_sweep_check")
SCRIPTS = os.path.join(PKG, "scripts")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "turning_sweep_check.md")

sys.path.insert(0, SCRIPTS)

import turning_core as core  # noqa: E402


PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(u"%s %s" % (name, detail))
    return bool(condition)


def check_close(name, actual, expected, tolerance=1e-6):
    try:
        same = abs(float(actual) - float(expected)) <= tolerance
    except (TypeError, ValueError):
        same = False
    return check(name, same, u"实测 %r，期望 %r（容差 %g）" % (actual, expected, tolerance))


def check_raises(name, expected, func, *args, **kwargs):
    """expected 是被期待的异常类型。"""
    try:
        func(*args, **kwargs)
    except expected:
        return check(name, True)
    except Exception as error:  # noqa: BLE001
        return check(name, False, u"抛出了 %s: %s" % (type(error).__name__, error))
    return check(name, False, u"没有抛出 %s" % expected.__name__)


def make_vehicle(**kwargs):
    return core.VehicleSpec(**kwargs)


def make_params(**kwargs):
    merged = dict(core.PARAM_DEFAULTS)
    merged.update(kwargs)
    return core.TurningParams(**merged)


def fake_getter(values, default_values=None):
    """模拟 rospy.get_param(name, default)。"""
    table = dict(default_values or {})

    def get_param(name, default=None):
        if name in values:
            return values[name]
        if name in table:
            return table[name]
        return default

    return get_param


# ------------------------------------------------------------------ A 默认参数


def group_defaults():
    check_close("A1 DEFAULT_WHEELBASE", core.DEFAULT_WHEELBASE, 2.70)
    check_close("A2 DEFAULT_TRACK", core.DEFAULT_TRACK, 1.54)
    check_close("A3 DEFAULT_FRONT_OVERHANG", core.DEFAULT_FRONT_OVERHANG, 0.90)
    check_close("A4 DEFAULT_REAR_OVERHANG", core.DEFAULT_REAR_OVERHANG, 0.85)
    check_close("A5 DEFAULT_VEHICLE_WIDTH", core.DEFAULT_VEHICLE_WIDTH, 1.80)
    check_close("A6 DEFAULT_MAX_STEER", core.DEFAULT_MAX_STEER, 0.55)
    check_close("A7 DEFAULT_CHANNEL_WIDTH", core.DEFAULT_CHANNEL_WIDTH, 6.0)
    check_close("A8 DEFAULT_INNER_CLEARANCE", core.DEFAULT_INNER_CLEARANCE, 0.30)
    check_close("A9 DEFAULT_MAX_INNER_DIFF", core.DEFAULT_MAX_INNER_DIFF, 1.00)
    check_close("A10 DEFAULT_PUBLISH_HZ", core.DEFAULT_PUBLISH_HZ, 1.0)
    check("A11 DEFAULT_TOPIC_REPORT", core.DEFAULT_TOPIC_REPORT == "/turning/sweep",
          core.DEFAULT_TOPIC_REPORT)
    check("A12 DEFAULT_TOPIC_RING", core.DEFAULT_TOPIC_RING == "/turning/ring",
          core.DEFAULT_TOPIC_RING)
    check("A13 DEFAULT_GRID", (core.DEFAULT_GRID_WIDTH, core.DEFAULT_GRID_HEIGHT) == (60, 30),
          (core.DEFAULT_GRID_WIDTH, core.DEFAULT_GRID_HEIGHT))
    check("A14 PARAM_KEYS 数量", len(core.PARAM_KEYS) == 13, len(core.PARAM_KEYS))
    check("A15 PARAM_KEYS 无重复", len(set(core.PARAM_KEYS)) == len(core.PARAM_KEYS))
    check("A16 PARAM_DEFAULTS 键数", len(core.PARAM_DEFAULTS) == 13, len(core.PARAM_DEFAULTS))
    check("A17 ASSERT_NAMES 四条", len(core.ASSERT_NAMES) == 4, core.ASSERT_NAMES)
    check("A18 TURN_RADII 八项", len(core.TURN_RADII) == 8, core.TURN_RADII)


# ------------------------------------------------------------------ B 小工具


def group_helpers():
    check("B1 _as_float 整数", core._as_float(3, "x") == 3.0)
    check("B2 _as_float 字符串", core._as_float("2.5", "x") == 2.5)
    check("B3 _as_float 默认值", core._as_float(None, "x", 1.5) == 1.5)
    check("B4 _as_float 空字符串", core._as_float("  ", "x", 2.0) == 2.0)
    check_raises("B5 _as_float 空值报错", core.TurningError, core._as_float, None, "x")
    check_raises("B6 _as_float 非数字报错", core.TurningError, core._as_float, "abc", "x")
    check_raises("B7 _as_float NaN 报错", core.TurningError, core._as_float, float("nan"), "x")
    check_raises("B8 _as_float Inf 报错", core.TurningError, core._as_float, float("inf"), "x")
    check_close("B9 _positive 正常", core._positive(2.0, "x"), 2.0)
    check_raises("B10 _positive 零报错", core.TurningError, core._positive, 0.0, "x")
    check_raises("B11 _positive 负报错", core.TurningError, core._positive, -1.0, "x")
    check_close("B12 _non_negative 零可以", core._non_negative(0.0, "x"), 0.0)
    check_raises("B13 _non_negative 负报错", core.TurningError, core._non_negative, -0.1, "x")
    text = core.to_message_text(u"转弯")
    check("B14 to_message_text 返回 str/bytes", isinstance(text, (str, bytes)))


# ------------------------------------------------------------------ C 车辆几何


def group_vehicle():
    vehicle = make_vehicle()
    check_close("C1 轴距", vehicle.wheelbase, 2.70)
    check_close("C2 轮距", vehicle.track, 1.54)
    check_close("C3 半轮距", vehicle.half_track, 0.77)
    check_close("C4 车长", vehicle.length, 4.45)
    check_close("C5 车长公式", vehicle.length,
                vehicle.front_overhang + vehicle.wheelbase + vehicle.rear_overhang)
    check_close("C6 半车宽", vehicle.body_half_width, 0.90)
    check_close("C7 最小转弯半径", vehicle.min_radius, 2.70 / math.tan(0.55), 1e-9)
    check_close("C8 最小转弯半径数值", vehicle.min_radius, 4.403812, 1e-5)
    check("C9 as_tuple 六项", len(vehicle.as_tuple()) == 6, vehicle.as_tuple())
    check("C10 repr 含 VehicleSpec", "VehicleSpec" in repr(vehicle), repr(vehicle))

    check_raises("C11 轴距为零报错", core.TurningError, make_vehicle, wheelbase=0.0)
    check_raises("C12 轮距为负报错", core.TurningError, make_vehicle, track=-1.0)
    check_raises("C13 前悬为负报错", core.TurningError, make_vehicle, front_overhang=-0.1)
    check_raises("C14 后悬为负报错", core.TurningError, make_vehicle, rear_overhang=-0.1)
    check_raises("C15 最大转角为零报错", core.TurningError, make_vehicle, max_steer=0.0)
    check_raises("C16 最大转角超 90 度报错", core.TurningError, make_vehicle, max_steer=1.6)
    check("C17 前悬后悬可为零", make_vehicle(front_overhang=0.0, rear_overhang=0.0) is not None)
    check_raises("C18 车宽小于轮距报错", core.TurningError, make_vehicle,
                 width=1.50, track=1.54)
    check("C19 车宽等于轮距可以", make_vehicle(width=1.54, track=1.54) is not None)


# ------------------------------------------------------------------ D 扫掠半径


def group_sweep_radii():
    vehicle = make_vehicle()
    result = core.sweep(vehicle, 8.0)
    radii = result["radii"]

    check_close("D1 记录半径", result["radius"], 8.0)
    check_close("D2 内后轮", radii["inner_rear"], 7.23)
    check_close("D3 外后轮", radii["outer_rear"], 8.77)
    check_close("D4 内前轮", radii["inner_front"], math.hypot(7.23, 2.70), 1e-9)
    check_close("D5 外前轮", radii["outer_front"], math.hypot(8.77, 2.70), 1e-9)
    check_close("D6 外前角", radii["outer_front_corner"], math.hypot(8.77, 3.60), 1e-9)
    check_close("D7 内前角", radii["inner_front_corner"], math.hypot(7.23, 3.60), 1e-9)
    check_close("D8 外后角", radii["outer_rear_corner"], math.hypot(8.77, 0.85), 1e-9)
    check_close("D9 内后角", radii["inner_rear_corner"], math.hypot(7.23, 0.85), 1e-9)
    check("D10 八项齐全", sorted(radii.keys()) == sorted(core.TURN_RADII), sorted(radii.keys()))
    check("D11 内外有序", all(radii["inner_" + key] < radii["outer_" + key]
                              for key in ("rear", "front", "front_corner", "rear_corner")),
          radii)
    check_close("D12 内前轮数值", radii["inner_front"], 7.717700, 1e-5)
    check_close("D13 外前轮数值", radii["outer_front"], 9.176214, 1e-5)
    check_close("D14 外前角数值", radii["outer_front_corner"], 9.480132, 1e-5)

    check_raises("D15 半径为零报错", core.TurningError, core.sweep, vehicle, 0.0)
    check_raises("D16 半径小于半轮距报错", core.TurningError, core.sweep, vehicle, 0.5)
    tight = core.sweep(vehicle, 1.0)
    check("D17 半径略大于半轮距可算", tight["radii"]["inner_rear"] > 0, tight["radii"])


# ------------------------------------------------------------------ E 派生指标


def group_metrics():
    vehicle = make_vehicle()
    result = core.sweep(vehicle, 8.0)

    check_close("E1 外圈取最大值", result["outer_max"], 9.480132, 1e-5)
    check_close("E2 内圈取最小值", result["inner_min"], 7.23)
    check_close("E3 扫掠宽度 = 外圈 - 内圈", result["swept_width"],
                result["outer_max"] - result["inner_min"], 1e-12)
    check_close("E4 扫掠宽度数值", result["swept_width"], 2.250132, 1e-5)
    check_close("E5 内轮差", result["inner_diff"], result["radii"]["inner_front"]
                - result["radii"]["inner_rear"], 1e-12)
    check_close("E6 内轮差数值", result["inner_diff"], 0.487700, 1e-5)
    check_close("E7 外摆", result["outer_swing"],
                result["radii"]["outer_front_corner"] - result["radii"]["outer_front"], 1e-12)
    check_close("E8 外摆数值", result["outer_swing"], 0.303918, 1e-5)
    check_close("E9 等效转角", result["steer_angle"], math.atan(2.70 / 8.0), 1e-12)
    check_close("E10 等效转角数值", result["steer_angle"], 0.325496, 1e-5)
    check("E11 结果里带车辆", result["vehicle"] is vehicle)

    larger = core.sweep(vehicle, 12.0)
    check("E12 半径变大扫掠变窄", larger["swept_width"] < result["swept_width"],
          (larger["swept_width"], result["swept_width"]))
    check("E13 半径变大内轮差变小", larger["inner_diff"] < result["inner_diff"],
          (larger["inner_diff"], result["inner_diff"]))
    check("E14 半径变大转角变小", larger["steer_angle"] < result["steer_angle"],
          (larger["steer_angle"], result["steer_angle"]))

    limit = core.radii_at_limit(vehicle)
    check_close("E15 极限半径", limit["radius"], vehicle.min_radius, 1e-4)
    check("E16 极限时扫掠更宽", limit["swept_width"] > result["swept_width"],
          (limit["swept_width"], result["swept_width"]))
    check_close("E17 极限时转角接近最大转角", limit["steer_angle"], vehicle.max_steer, 1e-4)


# ------------------------------------------------------------------ F 四条断言


def group_checks():
    vehicle = make_vehicle()
    result = core.sweep(vehicle, 8.0)
    results = core.check_sweep(vehicle, result)

    check("F1 四条断言", len(results) == 4, len(results))
    check("F2 断言名齐全",
          [item["type"] for item in results] == list(core.ASSERT_NAMES),
          [item["type"] for item in results])
    check("F3 每条含 passed", all("passed" in item for item in results))
    check("F4 每条含 detail", all(item["detail"] for item in results))
    check("F5 默认全部通过", core.all_passed(results),
          [item for item in results if not item["passed"]])
    check("F6 detail 带数字", "m" in results[0]["detail"], results[0]["detail"])

    narrow = core.check_sweep(vehicle, result, channel_width=2.0)
    check("F7 通道太窄只失败一条",
          [item["type"] for item in narrow if not item["passed"]] == [u"通道宽度"],
          [item["type"] for item in narrow if not item["passed"]])
    check("F8 通道太窄结论 FAILED", not core.all_passed(narrow))

    too_tight = core.check_sweep(vehicle, result, max_inner_diff=0.3)
    check("F9 内轮差超限失败",
          [item["type"] for item in too_tight if not item["passed"]] == [u"内轮差"],
          [item["type"] for item in too_tight if not item["passed"]])

    big_clearance = core.check_sweep(vehicle, result, inner_clearance=7.5)
    check("F10 内侧余量不足失败",
          [item["type"] for item in big_clearance if not item["passed"]] == [u"内侧余量"],
          [item["type"] for item in big_clearance if not item["passed"]])

    tight_radius = core.sweep(vehicle, 4.0)
    tight_result = core.check_sweep(vehicle, tight_radius)
    check("F11 半径小于最小可行失败",
          [item["type"] for item in tight_result if not item["passed"]] == [u"转弯可行性"],
          [item["type"] for item in tight_result if not item["passed"]])

    both = core.check_sweep(vehicle, tight_radius, channel_width=1.0)
    check("F12 可同时失败两条",
          len([item for item in both if not item["passed"]]) == 2,
          [item["type"] for item in both if not item["passed"]])

    check_raises("F13 通道宽度为零报错", core.TurningError, core.check_sweep,
                 vehicle, result, 0.0)
    check_raises("F14 内轮差上限为零报错", core.TurningError, core.check_sweep,
                 vehicle, result, 6.0, 0.3, 0.0)
    check_raises("F15 内侧余量为负报错", core.TurningError, core.check_sweep,
                 vehicle, result, 6.0, -0.1)
    check("F16 all_passed 空列表为真", core.all_passed([]))


# ------------------------------------------------------------------ G 字符图


def group_render():
    vehicle = make_vehicle()
    result = core.sweep(vehicle, 8.0)
    rows = core.render_ring(vehicle, result)

    check("G1 行数", len(rows) == core.DEFAULT_GRID_HEIGHT, len(rows))
    check("G2 每行列数一致", all(len(line) == core.DEFAULT_GRID_WIDTH for line in rows),
          [len(line) for line in rows][:3])
    flat = u"".join(rows)
    check("G3 含内圈字符", u"#" in flat)
    check("G4 含圆环字符", u"." in flat)
    check("G5 含空白字符", u" " in flat)
    check("G6 中心是内圈", rows[core.DEFAULT_GRID_HEIGHT // 2][core.DEFAULT_GRID_WIDTH // 2] == u"#",
          rows[core.DEFAULT_GRID_HEIGHT // 2][core.DEFAULT_GRID_WIDTH // 2])

    small = core.render_ring(vehicle, result, width=20, height=10)
    check("G7 自定义尺寸", len(small) == 10 and all(len(line) == 20 for line in small),
          (len(small), len(small[0])))
    check_raises("G8 尺寸过小报错", core.TurningError, core.render_ring, vehicle, result, 5, 3)


# ------------------------------------------------------------------ H 参数读取


def group_params():
    params = make_params()
    check("H1 默认 vehicle_width", params.vehicle_width == core.DEFAULT_VEHICLE_WIDTH)
    check("H2 默认 turn_radius", params.turn_radius == 8.0)
    check("H3 默认 topic_report", params.topic_report == core.DEFAULT_TOPIC_REPORT)
    check("H4 默认 topic_ring", params.topic_ring == core.DEFAULT_TOPIC_RING)
    check("H5 as_tuple 十三项", len(params.as_tuple()) == 13, len(params.as_tuple()))
    check("H6 repr 含 TurningParams", "TurningParams" in repr(params), repr(params))

    vehicle = params.vehicle()
    check("H7 由参数造车辆", isinstance(vehicle, core.VehicleSpec))
    check_close("H8 参数里的轴距传进去了", vehicle.wheelbase, params.wheelbase)

    getter = fake_getter({
        "vehicle/wheelbase": 3.0,
        "vehicle/track": 1.6,
        "vehicle/width": 1.9,
        "vehicle/max_steer": 0.6,
        "turning/radius": 9.0,
        "check/channel_width": 5.0,
        "check/inner_clearance": 0.4,
        "check/max_inner_diff": 0.8,
        "rate/publish_hz": 2.0,
        "topic/report": "/custom/report",
        "topic/ring": "/custom/ring",
    })
    parsed = core.TurningParams.from_params(getter)
    check_close("H9 读到轴距", parsed.wheelbase, 3.0)
    check_close("H10 读到轮距", parsed.track, 1.6)
    check_close("H11 读到车宽", parsed.vehicle_width, 1.9)
    check_close("H12 读到 max_steer", parsed.max_steer, 0.6)
    check_close("H13 读到半径", parsed.turn_radius, 9.0)
    check_close("H14 读到通道宽度", parsed.channel_width, 5.0)
    check_close("H15 读到内侧余量", parsed.inner_clearance, 0.4)
    check_close("H16 读到内轮差上限", parsed.max_inner_diff, 0.8)
    check_close("H17 读到频率", parsed.publish_hz, 2.0)
    check("H18 读到报告话题", parsed.topic_report == "/custom/report", parsed.topic_report)
    check("H19 读到字符图话题", parsed.topic_ring == "/custom/ring", parsed.topic_ring)

    fallback = core.TurningParams.from_params(fake_getter({}, core.PARAM_DEFAULTS))
    check_close("H20 缺键回落默认", fallback.turn_radius, 8.0)
    check("H21 缺话题回落默认", fallback.topic_report == core.DEFAULT_TOPIC_REPORT)

    check_raises("H22 频率为零报错", core.TurningError, core.TurningParams.from_params,
                 fake_getter({"rate/publish_hz": 0.0}))
    check_raises("H23 轴距非数字报错", core.TurningError, core.TurningParams.from_params,
                 fake_getter({"vehicle/wheelbase": "abc"}))


# ------------------------------------------------------------------ I 报告格式


def group_report():
    vehicle = make_vehicle()
    result = core.sweep(vehicle, 8.0)
    results = core.check_sweep(vehicle, result)
    report = core.format_report(vehicle, result, results, "测试标题")

    check("I1 含标题", report.splitlines()[0] == "测试标题", report.splitlines()[0])
    check("I2 含车辆行", u"车辆：" in report)
    check("I3 含转弯行", u"转弯：" in report)
    check("I4 含半径行", u"半径：" in report)
    check("I5 含扫掠行", u"扫掠：" in report)
    check("I6 含其他行", u"其他：" in report)
    check("I7 含断言统计", u"断言：4 通过 / 0 失败（共 4 条）" in report, report)
    check("I8 含 PASS 标记", report.count(u"[PASS]") == 4, report.count(u"[PASS]"))
    check("I9 含结论 PASSED", u"结论：PASSED" in report)
    check("I10 四条断言名都在",
          all(name in report for name in core.ASSERT_NAMES), core.ASSERT_NAMES)

    failed = core.check_sweep(vehicle, result, channel_width=1.0)
    failed_report = core.format_report(vehicle, result, failed)
    check("I11 失败含 FAIL 标记", u"[FAIL]" in failed_report)
    check("I12 失败结论 FAILED", u"结论：FAILED" in failed_report)
    check("I13 失败统计正确", u"断言：3 通过 / 1 失败" in failed_report, failed_report)
    check("I14 无标题时不空行", not core.format_report(vehicle, result, results)
          .startswith(u"\n"))


# ------------------------------------------------------------------ J 工程文件


def group_project_files():
    check("J1 包目录存在", os.path.isdir(PKG), PKG)
    check("J2 脚本目录存在", os.path.isdir(SCRIPTS), SCRIPTS)

    package_xml = os.path.join(PKG, "package.xml")
    check("J3 package.xml 存在", os.path.isfile(package_xml))
    if os.path.isfile(package_xml):
        text = io.open(package_xml, encoding="utf-8").read()
        check("J4 包名正确", u"<name>turning_sweep_check</name>" in text)
        check("J5 exec_depend rospy", u"<exec_depend>rospy</exec_depend>" in text)
        check("J6 exec_depend std_msgs", u"<exec_depend>std_msgs</exec_depend>" in text)
        check("J7 有 buildtool catkin", u"<buildtool_depend>catkin</buildtool_depend>" in text)

    cmake = os.path.join(PKG, "CMakeLists.txt")
    check("J8 CMakeLists 存在", os.path.isfile(cmake))
    if os.path.isfile(cmake):
        text = io.open(cmake, encoding="utf-8").read()
        check("J9 find_package catkin", "find_package(catkin REQUIRED" in text)
        check("J10 安装 main.py", "scripts/main.py" in text)
        check("J11 安装 turning_core.py", "scripts/turning_core.py" in text)
        check("J12 安装 turning_node.py", "scripts/turning_node.py" in text)
        check("J13 安装 launch 与 config", "DIRECTORY launch config" in text)
        check("J14 保留源权限", "USE_SOURCE_PERMISSIONS" in text)

    launch = os.path.join(PKG, "launch", "main.launch")
    check("J15 main.launch 存在", os.path.isfile(launch))
    if os.path.isfile(launch):
        text = io.open(launch, encoding="utf-8").read()
        check("J16 入口是 main.py", 'type="main.py"' in text)
        check("J17 加载 yaml", 'command="load"' in text)
        check("J18 带 doc", text.count('doc="') >= 13, text.count('doc="'))
        for index, arg in enumerate(("wheelbase", "track", "front_overhang", "rear_overhang",
                                     "vehicle_width", "max_steer", "turn_radius",
                                     "channel_width", "inner_clearance", "max_inner_diff",
                                     "publish_hz", "report_topic", "ring_topic")):
            check("J19.%d arg %s" % (index, arg), 'name="%s"' % arg in text, arg)

    config = os.path.join(PKG, "config", "turning_sweep_check.yaml")
    check("J20 config yaml 存在", os.path.isfile(config))
    if os.path.isfile(config):
        text = io.open(config, encoding="utf-8").read()
        for index, key in enumerate(("wheelbase", "track", "front_overhang", "rear_overhang",
                                     "max_steer", "channel_width", "inner_clearance",
                                     "max_inner_diff", "publish_hz")):
            check("J21.%d yaml 键 %s" % (index, key), key + ":" in text, key)

    core_source = io.open(os.path.join(SCRIPTS, "turning_core.py"), encoding="utf-8").read()
    check("J22 核心不 import rospy", "\nimport rospy" not in core_source)
    check("J23 核心不 import numpy", "\nimport numpy" not in core_source)
    node_source = io.open(os.path.join(SCRIPTS, "turning_node.py"), encoding="utf-8").read()
    check("J24 节点 import turning_core", "import turning_core" in node_source)
    check("J25 节点使用 unicode_literals", "unicode_literals" in node_source)
    check("J26 节点有 _log 助手", "def _log(" in node_source)
    check("J27 节点没直接 loginfo(fmt, args)", 'rospy.loginfo("' not in node_source)
    main_source = io.open(os.path.join(SCRIPTS, "main.py"), encoding="utf-8").read()
    check("J28 main.py 调 turning_node", "from turning_node import main" in main_source)

    check("J29 文档页存在", os.path.isfile(DOCS_PAGE), DOCS_PAGE)
    if os.path.isfile(DOCS_PAGE):
        page = io.open(DOCS_PAGE, encoding="utf-8").read()
        check("J30 文档页标题正确", page.startswith(u"# 转弯扫掠校验"), page[:40])
        check("J31 文档页有公式表", u"内后轮" in page and u"外前角" in page)
        check("J32 文档页有断言表", u"内轮差" in page and u"通道宽度" in page)
        check("J33 文档页有运行命令", u"roslaunch turning_sweep_check main.launch" in page)
        check("J34 文档页有离线测试命令", u"test_turning_sweep_check_local.py" in page)
        check("J35 文档页有大模型声明", u"大模型" in page)

    readme = os.path.join(PKG, "README.md")
    check("J36 README 存在", os.path.isfile(readme))
    if os.path.isfile(readme):
        text = io.open(readme, encoding="utf-8").read()
        check("J37 README 有启动命令", u"roslaunch turning_sweep_check main.launch" in text)
        check("J38 README 有测试命令", u"test_turning_sweep_check_local.py" in text)


# ------------------------------------------------------------------ 主流程


def _pytest_group(func):
    def runner():
        before = len(FAILED)
        func()
        new = FAILED[before:]
        if new:
            raise AssertionError(u"本组失败 %d 项：\n%s" % (len(new), u"\n".join(new)))
    runner.__name__ = "test_" + func.__name__[len("group_"):]
    return runner


test_defaults = _pytest_group(group_defaults)
test_helpers = _pytest_group(group_helpers)
test_vehicle = _pytest_group(group_vehicle)
test_sweep_radii = _pytest_group(group_sweep_radii)
test_metrics = _pytest_group(group_metrics)
test_checks = _pytest_group(group_checks)
test_render = _pytest_group(group_render)
test_params = _pytest_group(group_params)
test_report = _pytest_group(group_report)
test_project_files = _pytest_group(group_project_files)


def main():
    print(u"turning_sweep_check 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"-" * 60)
    group_defaults()
    group_helpers()
    group_vehicle()
    group_sweep_radii()
    group_metrics()
    group_checks()
    group_render()
    group_params()
    group_report()
    group_project_files()
    print(u"=" * 60)
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    for item in FAILED:
        print(u"  [失败] %s" % item)
    if FAILED:
        print(u"有失败项")
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
