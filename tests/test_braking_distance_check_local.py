#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""braking_distance_check 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_braking_distance_check_local.py

覆盖范围：
A 默认参数      B 小工具        C 路面模型
D 停车距离      E 跟车距离      F 四条断言
G 速度对照表    H 字符条        I 报告格式
J 参数读取      K 工程文件约定
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

PKG = os.path.join(ROOT, "src", "ground", "braking_distance_check")
SCRIPTS = os.path.join(PKG, "scripts")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "braking_distance_check.md")

sys.path.insert(0, SCRIPTS)

import braking_core as core  # noqa: E402


PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(u"%s %s" % (name, detail))
    return bool(condition)


def check_close(name, actual, expected, tolerance=1e-3):
    try:
        same = abs(float(actual) - float(expected)) <= tolerance
    except (TypeError, ValueError):
        same = False
    return check(name, same, u"实测 %r，期望 %r（容差 %g）" % (actual, expected, tolerance))


def check_raises(name, expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        return check(name, True)
    except Exception as error:  # noqa: BLE001
        return check(name, False, u"抛出了 %s: %s" % (type(error).__name__, error))
    return check(name, False, u"没有抛出 %s" % expected.__name__)


def make_params(**kwargs):
    merged = dict(core.PARAM_DEFAULTS)
    merged.update(kwargs)
    return core.BrakingParams(**merged)


def fake_getter(values, default_values=None):
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
    check_close("A1 DEFAULT_SPEED_KMH", core.DEFAULT_SPEED_KMH, 50.0)
    check_close("A2 DEFAULT_REACTION_TIME", core.DEFAULT_REACTION_TIME, 1.0)
    check_close("A3 DEFAULT_SPEED_LIMIT_KMH", core.DEFAULT_SPEED_LIMIT_KMH, 60.0)
    check_close("A4 DEFAULT_FRICTION", core.DEFAULT_FRICTION, 0.70)
    check_close("A5 DEFAULT_SLOPE_DEG", core.DEFAULT_SLOPE_DEG, 0.0)
    check_close("A6 DEFAULT_AVAILABLE_DISTANCE", core.DEFAULT_AVAILABLE_DISTANCE, 40.0)
    check_close("A7 DEFAULT_GAP", core.DEFAULT_GAP, 32.0)
    check_close("A8 DEFAULT_GAP_SECONDS", core.DEFAULT_GAP_SECONDS, 2.0)
    check_close("A9 DEFAULT_GAP_MARGIN", core.DEFAULT_GAP_MARGIN, 3.0)
    check_close("A10 DEFAULT_MIN_FRICTION", core.DEFAULT_MIN_FRICTION, 0.30)
    check_close("A11 DEFAULT_GRAVITY", core.DEFAULT_GRAVITY, 9.81)
    check("A12 DEFAULT_TOPIC_REPORT", core.DEFAULT_TOPIC_REPORT == "/braking/report",
          core.DEFAULT_TOPIC_REPORT)
    check("A13 DEFAULT_TOPIC_BARS", core.DEFAULT_TOPIC_BARS == "/braking/bars",
          core.DEFAULT_TOPIC_BARS)
    check("A14 PARAM_KEYS 十个", len(core.PARAM_KEYS) == 10, len(core.PARAM_KEYS))
    check("A15 PARAM_KEYS 无重复", len(set(core.PARAM_KEYS)) == len(core.PARAM_KEYS))
    check("A16 PARAM_DEFAULTS 键数", len(core.PARAM_DEFAULTS) == 10, len(core.PARAM_DEFAULTS))
    check("A17 ASSERT_NAMES 四条", len(core.ASSERT_NAMES) == 4, core.ASSERT_NAMES)
    check("A18 速度表四档", core.SPEED_TABLE_KMH == (30.0, 50.0, 80.0, 100.0),
          core.SPEED_TABLE_KMH)


# ------------------------------------------------------------------ B 小工具


def group_helpers():
    check_close("B1 _as_float 整数", core._as_float(3, "x"), 3.0)
    check_close("B2 _as_float 字符串", core._as_float("2.5", "x"), 2.5)
    check_close("B3 _as_float 默认值", core._as_float(None, "x", 1.5), 1.5)
    check_close("B4 _as_float 空串", core._as_float("  ", "x", 2.0), 2.0)
    check_raises("B5 _as_float 空值报错", core.BrakingError, core._as_float, None, "x")
    check_raises("B6 _as_float 非数字报错", core.BrakingError, core._as_float, "abc", "x")
    check_raises("B7 _as_float NaN 报错", core.BrakingError, core._as_float, float("nan"), "x")
    check_raises("B8 _as_float Inf 报错", core.BrakingError, core._as_float, float("inf"), "x")
    check_close("B9 _positive 正常", core._positive(2.0, "x"), 2.0)
    check_raises("B10 _positive 零报错", core.BrakingError, core._positive, 0.0, "x")
    check_raises("B11 _positive 负报错", core.BrakingError, core._positive, -1.0, "x")
    check_close("B12 _non_negative 零可以", core._non_negative(0.0, "x"), 0.0)
    check_raises("B13 _non_negative 负报错", core.BrakingError, core._non_negative, -0.1, "x")
    check_close("B14 kmh_to_mps 36", core.kmh_to_mps(36.0), 10.0, 1e-9)
    check_close("B15 kmh_to_mps 50", core.kmh_to_mps(50.0), 13.88888888888889, 1e-9)
    check_close("B16 mps_to_kmh 10", core.mps_to_kmh(10.0), 36.0, 1e-9)
    check_close("B17 换算是互逆的", core.mps_to_kmh(core.kmh_to_mps(72.0)), 72.0, 1e-9)
    text = core.to_message_text(u"刹车")
    check("B18 to_message_text 可用", isinstance(text, (str, bytes)))
    check_raises("B19 kmh_to_mps 非数字报错", core.BrakingError, core.kmh_to_mps, "abc")


# ------------------------------------------------------------------ C 路面模型


def group_road():
    road = core.RoadSpec()
    check_close("C1 默认附着", road.friction, 0.70)
    check_close("C2 默认坡度", road.slope_deg, 0.0)
    check_close("C3 默认减速度", road.deceleration, 0.70 * 9.81, 1e-9)
    check_close("C4 减速度数值", road.deceleration, 6.867, 1e-9)
    check_close("C5 坡度弧度", core.RoadSpec(0.7, 30.0).slope_rad, math.radians(30.0), 1e-12)
    check("C6 as_tuple 三项", len(road.as_tuple()) == 3, road.as_tuple())
    check("C7 repr 含 RoadSpec", "RoadSpec" in repr(road), repr(road))

    uphill = core.RoadSpec(0.70, 10.0)
    downhill = core.RoadSpec(0.70, -10.0)
    check("C8 上坡减速度更大", uphill.deceleration > road.deceleration,
          (uphill.deceleration, road.deceleration))
    check("C9 下坡减速度更小", downhill.deceleration < road.deceleration,
          (downhill.deceleration, road.deceleration))
    check_close("C10 上坡减速度数值", uphill.deceleration,
                0.70 * 9.81 * math.cos(math.radians(10.0))
                + 9.81 * math.sin(math.radians(10.0)), 1e-9)
    check_close("C11 下坡减速度数值", downhill.deceleration,
                0.70 * 9.81 * math.cos(math.radians(10.0))
                - 9.81 * math.sin(math.radians(10.0)), 1e-9)

    check_raises("C12 附着为零报错", core.BrakingError, core.RoadSpec, 0.0, 0.0)
    check_raises("C13 附着为负报错", core.BrakingError, core.RoadSpec, -0.1, 0.0)
    check_raises("C14 附着大于 1.5 报错", core.BrakingError, core.RoadSpec, 1.6, 0.0)
    check_raises("C15 坡度 45 度报错", core.BrakingError, core.RoadSpec, 0.7, 45.0)
    check_raises("C16 陡下坡停不住报错", core.BrakingError,
                 lambda: core.RoadSpec(0.05, -30.0).deceleration)
    check("C17 附着 1.5 可以", core.RoadSpec(1.5, 0.0) is not None)


# ------------------------------------------------------------------ D 停车距离


def group_measure():
    result = core.measure(50.0, 1.0, core.RoadSpec(0.70, 0.0))

    check_close("D1 车速 m/s", result["speed_mps"], 13.88888888888889, 1e-9)
    check_close("D2 车速回换算", result["speed_kmh"], 50.0, 1e-9)
    check_close("D3 减速度", result["deceleration"], 6.867, 1e-9)
    check_close("D4 反应距离", result["reaction_distance"], 13.88888888888889, 1e-9)
    check_close("D5 反应距离数值", result["reaction_distance"], 13.888889, 1e-5)
    check_close("D6 制动距离", result["brake_distance"], 14.045, 1e-3)
    check_close("D7 制动距离公式", result["brake_distance"],
                result["speed_mps"] ** 2 / (2.0 * result["deceleration"]), 1e-9)
    check_close("D8 停车距离", result["stop_distance"], 27.934, 1e-3)
    check_close("D9 停车距离可加和", result["stop_distance"],
                result["reaction_distance"] + result["brake_distance"], 1e-12)
    check_close("D10 刹停耗时", result["brake_time"], 2.0225, 1e-3)
    check("D11 结果里带路面", result["road"] is not None)

    zero_reaction = core.measure(50.0, 0.0, core.RoadSpec(0.70, 0.0))
    check_close("D12 反应时间为零", zero_reaction["reaction_distance"], 0.0, 1e-12)
    check_close("D13 反应为零时停车=制动", zero_reaction["stop_distance"],
                zero_reaction["brake_distance"], 1e-12)

    slow = core.measure(25.0, 1.0, core.RoadSpec(0.70, 0.0))
    check("D14 速度减半制动距离约为四分之一",
          abs(slow["brake_distance"] - result["brake_distance"] / 4.0) < 1e-3,
          (slow["brake_distance"], result["brake_distance"] / 4.0))

    wet = core.measure(50.0, 1.0, core.RoadSpec(0.35, 0.0))
    check_close("D15 雨路减速度", wet["deceleration"], 3.4335, 1e-4)
    check_close("D16 雨路制动距离", wet["brake_distance"], 28.091, 1e-3)
    check_close("D17 雨路停车距离", wet["stop_distance"], 41.980, 1e-3)
    check("D18 雨路停车距离翻倍", wet["stop_distance"] > result["stop_distance"] * 1.4,
          (wet["stop_distance"], result["stop_distance"]))

    check_raises("D19 车速为零报错", core.BrakingError, core.measure, 0.0)
    check_raises("D20 车速为负报错", core.BrakingError, core.measure, -10.0)
    check_raises("D21 反应时间为负报错", core.BrakingError, core.measure, 50.0, -1.0)


# ------------------------------------------------------------------ E 跟车距离


def group_gap():
    check_close("E1 默认两秒法则", core.safety_gap(50.0), 30.7777777, 1e-4)
    check_close("E2 两秒法则公式", core.safety_gap(50.0), 50.0 / 3.6 * 2.0 + 3.0, 1e-9)
    check_close("E3 自定义秒数", core.safety_gap(50.0, 3.0, 3.0), 50.0 / 3.6 * 3.0 + 3.0, 1e-9)
    check_close("E4 余量为零", core.safety_gap(50.0, 2.0, 0.0), 50.0 / 3.6 * 2.0, 1e-9)
    check_close("E5 车速翻倍距离翻倍", core.safety_gap(100.0, 2.0, 0.0),
                core.safety_gap(50.0, 2.0, 0.0) * 2.0, 1e-9)
    check_raises("E6 秒数为零报错", core.BrakingError, core.safety_gap, 50.0, 0.0)
    check_raises("E7 余量为负报错", core.BrakingError, core.safety_gap, 50.0, 2.0, -1.0)


# ------------------------------------------------------------------ F 四条断言


def group_checks():
    result = core.measure(50.0, 1.0, core.RoadSpec(0.70, 0.0))
    results = core.check_braking(result)

    check("F1 四条断言", len(results) == 4, len(results))
    check("F2 断言名齐全", [item["type"] for item in results] == list(core.ASSERT_NAMES),
          [item["type"] for item in results])
    check("F3 每条含 passed", all("passed" in item for item in results))
    check("F4 每条含 detail", all(item["detail"] for item in results))
    check("F5 默认全部通过", core.all_passed(results),
          [item for item in results if not item["passed"]])
    check("F6 detail 带单位", "m" in results[0]["detail"], results[0]["detail"])

    narrow = core.check_braking(result, available_distance=20.0)
    check("F7 前方太近只失败一条",
          [item["type"] for item in narrow if not item["passed"]] == [u"停车距离"],
          [item["type"] for item in narrow if not item["passed"]])

    tight = core.check_braking(result, gap=20.0)
    check("F8 跟车太近只失败一条",
          [item["type"] for item in tight if not item["passed"]] == [u"跟车间距"],
          [item["type"] for item in tight if not item["passed"]])

    fast = core.measure(70.0, 1.0, core.RoadSpec(0.70, 0.0))
    overspeed = core.check_braking(fast, available_distance=100.0, gap=100.0)
    check("F9 超速只失败一条",
          [item["type"] for item in overspeed if not item["passed"]] == [u"限速"],
          [item["type"] for item in overspeed if not item["passed"]])

    icy = core.measure(50.0, 1.0, core.RoadSpec(0.20, 0.0))
    icy_checks = core.check_braking(icy, available_distance=100.0, gap=100.0)
    check("F10 冰面附着失败",
          u"路面附着" in [item["type"] for item in icy_checks if not item["passed"]],
          [item["type"] for item in icy_checks if not item["passed"]])

    combo = core.check_braking(icy, available_distance=40.0, gap=100.0)
    check("F11 可同时失败两条",
          len([item for item in combo if not item["passed"]]) == 2,
          [item["type"] for item in combo if not item["passed"]])

    check("F12 all_passed 空列表为真", core.all_passed([]))
    check_raises("F13 可用距离为零报错", core.BrakingError, core.check_braking,
                 result, 0.0)
    check_raises("F14 车距为负报错", core.BrakingError, core.check_braking,
                 result, 40.0, -1.0)
    check_raises("F15 限速为零报错", core.BrakingError, core.check_braking,
                 result, 40.0, 32.0, 0.0)


# ------------------------------------------------------------------ G 速度对照表


def group_table():
    rows = core.speed_table(core.RoadSpec(0.70, 0.0), 1.0)

    check("G1 四行", len(rows) == 4, len(rows))
    check("G2 速度顺序", [round(row["speed_kmh"], 6) for row in rows] == [30.0, 50.0, 80.0, 100.0],
          [row["speed_kmh"] for row in rows])
    check_close("G3 30 km/h 停车距离", rows[0]["stop_distance"], 13.389, 1e-3)
    check_close("G4 50 km/h 停车距离", rows[1]["stop_distance"], 27.934, 1e-3)
    check_close("G5 80 km/h 停车距离", rows[2]["stop_distance"], 58.178, 1e-3)
    check_close("G6 100 km/h 停车距离", rows[3]["stop_distance"], 83.9599, 1e-3)
    check("G7 前两档够用", rows[0]["fits"] and rows[1]["fits"],
          [(row["speed_kmh"], row["fits"]) for row in rows])
    check("G8 后两档不够", not rows[2]["fits"] and not rows[3]["fits"],
          [(row["speed_kmh"], row["fits"]) for row in rows])
    check("G9 距离随速度单调增加",
          all(rows[index]["stop_distance"] < rows[index + 1]["stop_distance"]
              for index in range(len(rows) - 1)),
          [row["stop_distance"] for row in rows])

    wet_rows = core.speed_table(core.RoadSpec(0.35, 0.0), 1.0)
    check("G10 雨路更长的距离", wet_rows[1]["stop_distance"] > rows[1]["stop_distance"],
          (wet_rows[1]["stop_distance"], rows[1]["stop_distance"]))
    check("G11 雨路 30 km/h 够、50 km/h 不够",
          wet_rows[0]["fits"] and not wet_rows[1]["fits"],
          [row["stop_distance"] for row in wet_rows])


# ------------------------------------------------------------------ H 字符条


def group_bars():
    result = core.measure(50.0, 1.0, core.RoadSpec(0.70, 0.0))
    lines = core.render_bars(result, 40.0)

    check("H1 三行", len(lines) == 3, len(lines))
    check("H2 含等号", u"=" in lines[0])
    check("H3 含井号", u"#" in lines[0])
    check("H4 含减号", u"-" in lines[0])
    check("H5 有刻度行", u"0" in lines[1] and u"40.0 m" in lines[1], lines[1])
    check("H6 有图例", u"图例" in lines[2], lines[2])
    check("H7 字符条长度合理", 60 <= len(lines[0]) <= 64, len(lines[0]))

    narrow = core.render_bars(result, 40.0, width=24)
    check("H8 自定义宽度", len(narrow[0]) <= 30, len(narrow[0]))
    check_raises("H9 宽度过小报错", core.BrakingError, core.render_bars, result, 40.0, 10)

    long_stop = core.measure(100.0, 1.0, core.RoadSpec(0.35, 0.0))
    over = core.render_bars(long_stop, 40.0)
    check("H10 停不住时几乎全是填充", over[0].count(u"-") <= 2,
          (over[0].count(u"-"), over[0]))


# ------------------------------------------------------------------ I 报告格式


def group_report():
    result = core.measure(50.0, 1.0, core.RoadSpec(0.70, 0.0))
    results = core.check_braking(result)
    rows = core.speed_table(core.RoadSpec(0.70, 0.0), 1.0)
    report = core.format_report(result, results, rows, "测试标题")

    check("I1 首行是标题", report.splitlines()[0] == "测试标题", report.splitlines()[0])
    check("I2 含工况行", u"工况：" in report)
    check("I3 含减速行", u"减速：" in report)
    check("I4 含距离行", u"距离：" in report)
    check("I5 距离行带加和", u" + " in report and u" = " in report)
    check("I6 含断言统计", u"断言：4 通过 / 0 失败（共 4 条）" in report, report)
    check("I7 PASS 数量", report.count(u"[PASS]") == 4, report.count(u"[PASS]"))
    check("I8 含结论 PASSED", u"结论：PASSED" in report)
    check("I9 含速度对照表", u"车速对照" in report)
    check("I10 对照表四行", report.count(u"km/h -> ") == 4, report.count(u"km/h -> "))
    check("I11 对照表有够/不够", u"够" in report and u"不够" in report)

    failed = core.check_braking(result, gap=20.0)
    failed_report = core.format_report(result, failed, None)
    check("I12 失败含 FAIL", u"[FAIL]" in failed_report)
    check("I13 失败结论 FAILED", u"结论：FAILED" in failed_report)
    check("I14 失败统计正确", u"断言：3 通过 / 1 失败" in failed_report, failed_report)
    check("I15 无表格时不出现对照标题",
          u"车速对照" not in core.format_report(result, results))
    check("I16 无标题时不空行", not core.format_report(result, results).startswith(u"\n"))


# ------------------------------------------------------------------ J 参数读取


def group_params():
    params = make_params()
    check_close("J1 默认车速", params.speed_kmh, 50.0)
    check_close("J2 默认车距", params.gap, 32.0)
    check_close("J3 默认附着", params.friction, 0.70)
    check("J4 默认报告话题", params.topic_report == core.DEFAULT_TOPIC_REPORT)
    check("J5 as_tuple 十项", len(params.as_tuple()) == 10, len(params.as_tuple()))
    check("J6 repr 含 BrakingParams", "BrakingParams" in repr(params), repr(params))

    road = params.road()
    check("J7 由参数造路面", isinstance(road, core.RoadSpec))
    check_close("J8 参数里的附着传进去了", road.friction, params.friction)

    getter = fake_getter({
        "scene/speed_kmh": 70.0,
        "vehicle/reaction_time": 1.5,
        "vehicle/speed_limit_kmh": 80.0,
        "road/friction": 0.5,
        "road/slope_deg": 5.0,
        "scene/available_distance": 60.0,
        "scene/gap": 50.0,
        "rate/publish_hz": 2.0,
        "topic/report": "/custom/report",
        "topic/bars": "/custom/bars",
    })
    parsed = core.BrakingParams.from_params(getter)
    check_close("J9 读到车速", parsed.speed_kmh, 70.0)
    check_close("J10 读到反应时间", parsed.reaction_time, 1.5)
    check_close("J11 读到限速", parsed.speed_limit_kmh, 80.0)
    check_close("J12 读到附着", parsed.friction, 0.5)
    check_close("J13 读到坡度", parsed.slope_deg, 5.0)
    check_close("J14 读到可用距离", parsed.available_distance, 60.0)
    check_close("J15 读到车距", parsed.gap, 50.0)
    check_close("J16 读到频率", parsed.publish_hz, 2.0)
    check("J17 读到报告话题", parsed.topic_report == "/custom/report", parsed.topic_report)
    check("J18 读到字符条话题", parsed.topic_bars == "/custom/bars", parsed.topic_bars)

    fallback = core.BrakingParams.from_params(fake_getter({}, core.PARAM_DEFAULTS))
    check_close("J19 缺键回落默认", fallback.speed_kmh, 50.0)
    check_raises("J20 车速为零报错", core.BrakingError, core.BrakingParams.from_params,
                 fake_getter({"scene/speed_kmh": 0.0}))
    check_raises("J21 反应时间为负报错", core.BrakingError, core.BrakingParams.from_params,
                 fake_getter({"vehicle/reaction_time": -0.5}))
    check_raises("J22 附着非数字报错", core.BrakingError, core.BrakingParams.from_params,
                 fake_getter({"road/friction": "abc"}))


# ------------------------------------------------------------------ K 工程文件


def group_project_files():
    package_xml = os.path.join(PKG, "package.xml")
    check("K1 package.xml 存在", os.path.isfile(package_xml), package_xml)
    if os.path.isfile(package_xml):
        text = io.open(package_xml, encoding="utf-8").read()
        check("K2 包名正确", u"<name>braking_distance_check</name>" in text)
        check("K3 exec_depend rospy", u"<exec_depend>rospy</exec_depend>" in text)
        check("K4 exec_depend std_msgs", u"<exec_depend>std_msgs</exec_depend>" in text)
        check("K5 有 buildtool", u"<buildtool_depend>catkin</buildtool_depend>" in text)

    cmake = os.path.join(PKG, "CMakeLists.txt")
    check("K6 CMakeLists 存在", os.path.isfile(cmake))
    if os.path.isfile(cmake):
        text = io.open(cmake, encoding="utf-8").read()
        check("K7 find_package catkin", "find_package(catkin REQUIRED" in text)
        check("K8 安装 main.py", "scripts/main.py" in text)
        check("K9 安装 braking_core.py", "scripts/braking_core.py" in text)
        check("K10 安装 braking_node.py", "scripts/braking_node.py" in text)
        check("K11 安装 launch 与 config", "DIRECTORY launch config" in text)
        check("K12 保留源权限", "USE_SOURCE_PERMISSIONS" in text)

    launch = os.path.join(PKG, "launch", "main.launch")
    check("K13 main.launch 存在", os.path.isfile(launch))
    if os.path.isfile(launch):
        text = io.open(launch, encoding="utf-8").read()
        check("K14 入口是 main.py", 'type="main.py"' in text)
        check("K15 加载 yaml", 'command="load"' in text)
        check("K16 arg 带 doc", text.count('doc="') >= 10, text.count('doc="'))
        for index, arg in enumerate(("speed_kmh", "available_distance", "gap",
                                     "reaction_time", "speed_limit_kmh", "friction",
                                     "slope_deg", "publish_hz", "report_topic",
                                     "bars_topic")):
            check("K17.%d arg %s" % (index, arg), 'name="%s"' % arg in text, arg)

    config = os.path.join(PKG, "config", "braking_distance_check.yaml")
    check("K18 config yaml 存在", os.path.isfile(config))
    if os.path.isfile(config):
        text = io.open(config, encoding="utf-8").read()
        for index, key in enumerate(("speed_kmh", "available_distance", "gap",
                                     "reaction_time", "speed_limit_kmh", "friction",
                                     "slope_deg", "publish_hz")):
            check("K19.%d yaml 键 %s" % (index, key), key + ":" in text, key)

    core_source = io.open(os.path.join(SCRIPTS, "braking_core.py"), encoding="utf-8").read()
    check("K20 核心不 import rospy", "\nimport rospy" not in core_source)
    check("K21 核心不 import numpy", "\nimport numpy" not in core_source)
    node_source = io.open(os.path.join(SCRIPTS, "braking_node.py"), encoding="utf-8").read()
    check("K22 节点 import braking_core", "import braking_core" in node_source)
    check("K23 节点用 unicode_literals", "unicode_literals" in node_source)
    check("K24 节点有 _log 助手", "def _log(" in node_source)
    check("K25 节点没直接 loginfo(fmt, args)", 'rospy.loginfo("' not in node_source)
    main_source = io.open(os.path.join(SCRIPTS, "main.py"), encoding="utf-8").read()
    check("K26 main.py 调 braking_node", "from braking_node import main" in main_source)

    check("K27 文档页存在", os.path.isfile(DOCS_PAGE), DOCS_PAGE)
    if os.path.isfile(DOCS_PAGE):
        page = io.open(DOCS_PAGE, encoding="utf-8").read()
        check("K28 文档页标题", page.startswith(u"# 刹车距离核算"), page[:40])
        check("K29 文档页有公式表", u"反应距离" in page and u"制动距离" in page)
        check("K30 文档页有断言表", u"跟车间距" in page and u"路面附着" in page)
        check("K31 文档页有运行命令",
              u"roslaunch braking_distance_check main.launch" in page)
        check("K32 文档页有离线测试命令",
              u"test_braking_distance_check_local.py" in page)
        check("K33 文档页有大模型声明", u"大模型" in page)

    readme = os.path.join(PKG, "README.md")
    check("K34 README 存在", os.path.isfile(readme))
    if os.path.isfile(readme):
        text = io.open(readme, encoding="utf-8").read()
        check("K35 README 有启动命令",
              u"roslaunch braking_distance_check main.launch" in text)
        check("K36 README 有测试命令", u"test_braking_distance_check_local.py" in text)


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
test_road = _pytest_group(group_road)
test_measure = _pytest_group(group_measure)
test_gap = _pytest_group(group_gap)
test_checks = _pytest_group(group_checks)
test_table = _pytest_group(group_table)
test_bars = _pytest_group(group_bars)
test_report = _pytest_group(group_report)
test_params = _pytest_group(group_params)
test_project_files = _pytest_group(group_project_files)


def main():
    print(u"braking_distance_check 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"-" * 60)
    group_defaults()
    group_helpers()
    group_road()
    group_measure()
    group_gap()
    group_checks()
    group_table()
    group_bars()
    group_report()
    group_params()
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
