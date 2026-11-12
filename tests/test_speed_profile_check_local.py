#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""speed_profile_check 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_speed_profile_check_local.py

覆盖范围：
A 默认参数      B 小工具        C 梯形曲线
D 三角形曲线    E 带初速        F 非法参数
G 曲线取点      H 采样          I 四条断言
J 字符图与报告  K 参数与工程文件
"""

from __future__ import division
from __future__ import print_function

import io
import os
import sys

try:  # Python 2 兼容
    text_type = unicode  # noqa: F821
except NameError:  # Python 3
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PKG = os.path.join(ROOT, "src", "ground", "speed_profile_check")
SCRIPTS = os.path.join(PKG, "scripts")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "speed_profile_check.md")

sys.path.insert(0, SCRIPTS)

import speed_core as core  # noqa: E402


PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(u"%s %s" % (name, detail))
    return bool(condition)


def check_close(name, actual, expected, tolerance=1e-4):
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
    defaults = {
        "distance": core.DEFAULT_DISTANCE,
        "cruise_kmh": core.DEFAULT_CRUISE_KMH,
        "start_kmh": core.DEFAULT_START_KMH,
        "accel": core.DEFAULT_ACCEL,
        "decel": core.DEFAULT_DECEL,
        "speed_limit_kmh": core.DEFAULT_SPEED_LIMIT_KMH,
        "time_budget": core.DEFAULT_TIME_BUDGET,
        "tolerance": core.DEFAULT_TOLERANCE,
        "dt": core.DEFAULT_DT,
        "publish_hz": core.DEFAULT_PUBLISH_HZ,
        "topic_profile": core.DEFAULT_TOPIC_PROFILE,
        "topic_report": core.DEFAULT_TOPIC_REPORT,
    }
    defaults.update(kwargs)
    return core.SpeedParams(**defaults)


def fake_getter(values):
    def get_param(name, default=None):
        if name in values:
            return values[name]
        return default

    return get_param


# ------------------------------------------------------------------ A 默认参数


def group_defaults():
    check_close("A1 DEFAULT_DISTANCE", core.DEFAULT_DISTANCE, 120.0)
    check_close("A2 DEFAULT_CRUISE_KMH", core.DEFAULT_CRUISE_KMH, 40.0)
    check_close("A3 DEFAULT_START_KMH", core.DEFAULT_START_KMH, 0.0)
    check_close("A4 DEFAULT_ACCEL", core.DEFAULT_ACCEL, 1.2)
    check_close("A5 DEFAULT_DECEL", core.DEFAULT_DECEL, 2.0)
    check_close("A6 DEFAULT_SPEED_LIMIT_KMH", core.DEFAULT_SPEED_LIMIT_KMH, 50.0)
    check_close("A7 DEFAULT_TIME_BUDGET", core.DEFAULT_TIME_BUDGET, 30.0)
    check_close("A8 DEFAULT_TOLERANCE", core.DEFAULT_TOLERANCE, 0.5)
    check_close("A9 DEFAULT_DT", core.DEFAULT_DT, 0.05)
    check_close("A10 DEFAULT_PUBLISH_HZ", core.DEFAULT_PUBLISH_HZ, 5.0)
    check("A11 默认曲线话题", core.DEFAULT_TOPIC_PROFILE == "/speed/profile",
          core.DEFAULT_TOPIC_PROFILE)
    check("A12 默认报告话题", core.DEFAULT_TOPIC_REPORT == "/speed/report",
          core.DEFAULT_TOPIC_REPORT)
    check("A13 PARAM_KEYS 十二个", len(core.PARAM_KEYS) == 12, len(core.PARAM_KEYS))
    check("A14 PARAM_KEYS 无重复", len(set(core.PARAM_KEYS)) == len(core.PARAM_KEYS))
    check("A15 PARAM_DEFAULTS 键数", len(core.PARAM_DEFAULTS) == 12,
          len(core.PARAM_DEFAULTS))
    check("A16 PARAM_DEFAULTS 与 KEYS 对齐",
          sorted(core.PARAM_DEFAULTS.keys()) == sorted(core.PARAM_KEYS))
    check("A17 ASSERT_NAMES 四条", len(core.ASSERT_NAMES) == 4, core.ASSERT_NAMES)
    check("A18 断言名内容", core.ASSERT_NAMES == (u"限速", u"加速度", u"距离", u"时间"))


# ------------------------------------------------------------------ B 小工具


def group_helpers():
    check_close("B1 _as_float 整数", core._as_float(3, "x"), 3.0)
    check_close("B2 _as_float 字符串", core._as_float("2.5", "x"), 2.5)
    check_close("B3 _as_float 默认值", core._as_float(None, "x", 1.5), 1.5)
    check_close("B4 _as_float 空串回落", core._as_float("  ", "x", 2.0), 2.0)
    check_close("B5 _as_float 布尔", core._as_float(True, "x"), 1.0)
    check_raises("B6 _as_float 空值报错", core.ProfileError, core._as_float, None, "x")
    check_raises("B7 _as_float 非数字报错", core.ProfileError, core._as_float, "abc", "x")
    check_raises("B8 _as_float NaN 报错", core.ProfileError, core._as_float,
                 float("nan"), "x")
    check_raises("B9 _as_float Inf 报错", core.ProfileError, core._as_float,
                 float("inf"), "x")
    check_close("B10 _positive 正常", core._positive(2.0, "x"), 2.0)
    check_raises("B11 _positive 零报错", core.ProfileError, core._positive, 0.0, "x")
    check_raises("B12 _positive 负报错", core.ProfileError, core._positive, -1.0, "x")
    check_close("B13 _non_negative 零可以", core._non_negative(0.0, "x"), 0.0)
    check_raises("B14 _non_negative 负报错", core.ProfileError, core._non_negative,
                 -0.1, "x")
    check("B15 as_bool True", core.as_bool(True) is True)
    check("B16 as_bool 字符串 yes", core.as_bool("YES") is True)
    check("B17 as_bool 字符串 0", core.as_bool("0") is False)
    check("B18 as_bool 空值取默认", core.as_bool(None, False) is False)
    check_raises("B19 as_bool 非法值报错", core.ProfileError, core.as_bool, "maybe")
    check_close("B20 kmh_to_mps 36", core.kmh_to_mps(36.0), 10.0, 1e-9)
    check_close("B21 kmh_to_mps 40", core.kmh_to_mps(40.0), 11.11111111111111, 1e-9)
    check_close("B22 mps_to_kmh 10", core.mps_to_kmh(10.0), 36.0, 1e-9)
    check_close("B23 换算是互逆的", core.mps_to_kmh(core.kmh_to_mps(72.0)), 72.0, 1e-9)
    check_raises("B24 kmh_to_mps 非数字", core.ProfileError, core.kmh_to_mps, "abc")
    text = core.to_message_text(u"速度")
    check("B25 to_message_text 可用", isinstance(text, (str, bytes)))


# ------------------------------------------------------------------ C 梯形曲线


def group_trapezoid():
    profile = core.build_profile(120.0, 40.0)

    check("C1 类型是梯形", profile.kind == "trapezoid", profile.kind)
    check_close("C2 峰值速度", profile.peak_speed_kmh, 40.0, 1e-9)
    check_close("C3 峰值 m/s", profile.peak_speed_mps, 11.11111111111111, 1e-9)
    check_close("C4 初速", profile.start_kmh, 0.0)
    check_close("C5 加速时长", profile.accel_time, 9.259259, 1e-5)
    check_close("C6 加速距离", profile.accel_distance, 51.440329, 1e-5)
    check_close("C7 匀速时长", profile.cruise_time, 3.392593, 1e-5)
    check_close("C8 匀速距离", profile.cruise_distance, 37.695473, 1e-5)
    check_close("C9 减速时长", profile.decel_time, 5.555556, 1e-5)
    check_close("C10 减速距离", profile.decel_distance, 30.864198, 1e-5)
    check_close("C11 总时长", profile.total_time, 18.207407, 1e-5)
    check_close("C12 平均速度 m/s", profile.average_speed_mps, 6.590724, 1e-5)
    check_close("C13 平均速度 km/h", profile.average_speed_mps * 3.6, 23.726607, 1e-4)
    check_close("C14 三段距离之和",
                profile.accel_distance + profile.cruise_distance + profile.decel_distance,
                120.0, 1e-9)
    check_close("C15 加速距离公式", profile.accel_distance,
                profile.peak_speed_mps ** 2 / (2.0 * core.DEFAULT_ACCEL), 1e-9)
    check_close("C16 减速距离公式", profile.decel_distance,
                profile.peak_speed_mps ** 2 / (2.0 * core.DEFAULT_DECEL), 1e-9)
    check("C17 as_tuple 七项", len(profile.as_tuple()) == 7, profile.as_tuple())
    check("C18 repr 含 Profile", "Profile(" in repr(profile), repr(profile))

    longer = core.build_profile(200.0, 40.0)
    check("C19 距离更长则匀速段更长",
          longer.cruise_distance > profile.cruise_distance,
          (longer.cruise_distance, profile.cruise_distance))
    check("C20 距离更长则用时更长", longer.total_time > profile.total_time,
          (longer.total_time, profile.total_time))

    quicker = core.build_profile(120.0, 40.0, 0.0, 2.0, 2.0)
    check("C21 加速度更大则加速段更短", quicker.accel_time < profile.accel_time,
          (quicker.accel_time, profile.accel_time))


# ------------------------------------------------------------------ D 三角形曲线


def group_triangle():
    profile = core.build_profile(60.0, 40.0)

    check("D1 类型是三角形", profile.kind == "triangle", profile.kind)
    check("D2 峰值低于巡航", profile.peak_speed_kmh < 40.0, profile.peak_speed_kmh)
    check_close("D3 峰值速度", profile.peak_speed_kmh, 34.152599, 1e-5)
    check_close("D4 匀速时长为零", profile.cruise_time, 0.0, 1e-12)
    check_close("D5 匀速距离为零", profile.cruise_distance, 0.0, 1e-12)
    check_close("D6 加速距离", profile.accel_distance, 37.5, 1e-5)
    check_close("D7 减速距离", profile.decel_distance, 22.5, 1e-5)
    check_close("D8 总时长", profile.total_time, 12.649111, 1e-5)
    check_close("D9 两段距离之和", profile.accel_distance + profile.decel_distance,
                60.0, 1e-9)
    check("D10 峰值被反解拉低", profile.peak_speed_kmh < core.DEFAULT_CRUISE_KMH)

    peak = profile.peak_speed_mps
    check_close("D11 反解关系成立",
                (peak ** 2) / (2.0 * core.DEFAULT_ACCEL)
                + (peak ** 2) / (2.0 * core.DEFAULT_DECEL), 60.0, 1e-9)

    tiny = core.build_profile(5.0, 40.0)
    check("D12 极短距离仍是三角形", tiny.kind == "triangle", tiny.kind)
    check("D13 极短距离峰值很低", tiny.peak_speed_kmh < 15.0, tiny.peak_speed_kmh)
    check_close("D14 极短距离走完", tiny.accel_distance + tiny.decel_distance, 5.0, 1e-9)


# ------------------------------------------------------------------ E 带初速


def group_start_speed():
    profile = core.build_profile(120.0, 40.0, 20.0)

    check("E1 类型仍是梯形", profile.kind == "trapezoid", profile.kind)
    check_close("E2 初速保留", profile.start_kmh, 20.0, 1e-9)
    check_close("E3 加速时长变短", profile.accel_time, 4.629630, 1e-5)
    check_close("E4 加速距离", profile.accel_distance, 38.580247, 1e-5)
    check_close("E5 匀速距离变长", profile.cruise_distance, 50.555556, 1e-5)
    check_close("E6 总时长", profile.total_time, 14.735185, 1e-5)
    check_close("E7 起点速度", profile.speed_at(0.0), 5.555555555555555, 1e-9)
    check("E8 带初速比静止起步快", profile.total_time < core.build_profile(120.0, 40.0).total_time)

    same = core.build_profile(120.0, 40.0, 40.0)
    check_close("E9 初速等于巡航时加速段为零", same.accel_time, 0.0, 1e-12)
    check_close("E10 初速等于巡航时加速距离为零", same.accel_distance, 0.0, 1e-12)


# ------------------------------------------------------------------ F 非法参数


def group_errors():
    check_raises("F1 距离为零报错", core.ProfileError, core.build_profile, 0.0, 40.0)
    check_raises("F2 距离为负报错", core.ProfileError, core.build_profile, -5.0, 40.0)
    check_raises("F3 巡航为零报错", core.ProfileError, core.build_profile, 100.0, 0.0)
    check_raises("F4 巡航为负报错", core.ProfileError, core.build_profile, 100.0, -10.0)
    check_raises("F5 初速为负报错", core.ProfileError, core.build_profile,
                 100.0, 40.0, -1.0)
    check_raises("F6 初速高于巡航报错", core.ProfileError, core.build_profile,
                 100.0, 40.0, 50.0)
    check_raises("F7 加速度为零报错", core.ProfileError, core.build_profile,
                 100.0, 40.0, 0.0, 0.0)
    check_raises("F8 减速度为零报错", core.ProfileError, core.build_profile,
                 100.0, 40.0, 0.0, 1.0, 0.0)
    check_raises("F9 限速为零报错", core.ProfileError, core.check_profile,
                 core.build_profile(120.0, 40.0), 0.0)
    check_raises("F10 时间预算为零报错", core.ProfileError, core.check_profile,
                 core.build_profile(120.0, 40.0), 50.0, 0.0)
    check_raises("F11 容差为负报错", core.ProfileError, core.check_profile,
                 core.build_profile(120.0, 40.0), 50.0, 30.0, -1.0)
    check_raises("F12 采样步长为零报错", core.ProfileError,
                 lambda: core.build_profile(120.0, 40.0).samples(0.0))


# ------------------------------------------------------------------ G 曲线取点


def group_sampling_speed():
    profile = core.build_profile(120.0, 40.0)

    check_close("G1 t=0 时速度为零", profile.speed_at(0.0), 0.0, 1e-9)
    check_close("G2 t=加速段一半", profile.speed_at(profile.accel_time / 2.0),
                5.555555555555555, 1e-6)
    check_close("G3 t=加速段结束就是巡航", profile.speed_at(profile.accel_time),
                profile.peak_speed_mps, 1e-9)
    check_close("G4 t=匀速段中仍巡航",
                profile.speed_at(profile.accel_time + profile.cruise_time / 2.0),
                profile.peak_speed_mps, 1e-9)
    check_close("G5 t=全程结束速度为零", profile.speed_at(profile.total_time),
                0.0, 1e-6)
    check_close("G6 负时间按 0 处理", profile.speed_at(-5.0), 0.0, 1e-9)
    check_close("G7 超过终点按 0 处理", profile.speed_at(999.0), 0.0, 1e-9)
    check("G8 减速段速度单调不增",
          profile.speed_at(profile.accel_time + profile.cruise_time + 1.0)
          >= profile.speed_at(profile.accel_time + profile.cruise_time + 2.0))
    check_close("G9 加速段速度递增",
                profile.speed_at(2.0) - profile.speed_at(1.0), core.DEFAULT_ACCEL, 1e-9)

    check_close("G10 t=0 距离为零", profile.distance_at(0.0), 0.0, 1e-9)
    check_close("G11 加速段结束的距离", profile.distance_at(profile.accel_time),
                51.440329, 1e-5)
    check_close("G12 匀速段结束的距离",
                profile.distance_at(profile.accel_time + profile.cruise_time),
                89.135802, 1e-5)
    check_close("G13 全程结束的距离", profile.distance_at(profile.total_time),
                120.0, 1e-6)
    check_close("G14 超过终点仍是总距离", profile.distance_at(999.0), 120.0, 1e-9)
    check_close("G15 负时间距离为零", profile.distance_at(-1.0), 0.0, 1e-9)
    check("G16 距离单调不减",
          profile.distance_at(5.0) <= profile.distance_at(6.0))
    check_close("G17 加速段距离与匀加速公式一致",
                profile.distance_at(3.0), 0.5 * core.DEFAULT_ACCEL * 9.0, 1e-9)


# ------------------------------------------------------------------ H 采样


def group_samples():
    profile = core.build_profile(120.0, 40.0)
    samples = profile.samples(0.05)

    check("H1 采样数量", len(samples) == 365, len(samples))
    check("H2 每条三个值", len(samples[0]) == 3, samples[0])
    check_close("H3 第一个采样 t=0", samples[0][0], 0.0, 1e-12)
    check_close("H4 第一个采样速度为零", samples[0][1], 0.0, 1e-12)
    check_close("H5 第一个采样距离为零", samples[0][2], 0.0, 1e-12)
    check_close("H6 最后一个采样接近终点", samples[-1][2], 120.0, 0.05)
    check("H7 采样时间递增",
          all(samples[index][0] <= samples[index + 1][0]
              for index in range(len(samples) - 1)))
    check("H8 采样距离单调不减",
          all(samples[index][2] <= samples[index + 1][2] + 1e-9
              for index in range(len(samples) - 1)))
    check("H9 采样速度不超过峰值",
          all(item[1] <= profile.peak_speed_mps + 1e-9 for item in samples))
    middle = samples[len(samples) // 2]
    check_close("H10 中间采样 t", middle[0], 9.1, 1e-9)
    check_close("H11 中间采样速度", middle[1], 10.92, 1e-9)
    check_close("H12 中间采样距离", middle[2], 49.686, 1e-9)

    coarse = profile.samples(0.5)
    check("H13 粗采样数量更少", len(coarse) < len(samples), len(coarse))
    triangle = core.build_profile(60.0, 40.0)
    check("H14 三角形采样数量", len(triangle.samples(0.05)) == 254,
          len(triangle.samples(0.05)))


# ------------------------------------------------------------------ I 四条断言


def group_checks():
    profile = core.build_profile(120.0, 40.0)
    results = core.check_profile(profile)

    check("I1 四条断言", len(results) == 4, len(results))
    check("I2 断言名顺序", [item["type"] for item in results] == list(core.ASSERT_NAMES),
          [item["type"] for item in results])
    check("I3 每条含 passed", all("passed" in item for item in results))
    check("I4 每条含 detail", all(item["detail"] for item in results))
    check("I5 默认全部通过", core.all_passed(results),
          [item for item in results if not item["passed"]])
    check("I6 detail 带数字", "40.00" in results[0]["detail"], results[0]["detail"])

    over = core.check_profile(profile, speed_limit_kmh=35.0)
    check("I7 限速提升只失败限速",
          [item["type"] for item in over if not item["passed"]] == [u"限速"],
          [item["type"] for item in over if not item["passed"]])

    tight = core.check_profile(profile, time_budget=15.0)
    check("I8 时间预算不足只失败时间",
          [item["type"] for item in tight if not item["passed"]] == [u"时间"],
          [item["type"] for item in tight if not item["passed"]])

    loose = core.check_profile(profile, tolerance=0.001)
    check("I9 容差收紧仍然通过", core.all_passed(loose),
          [item for item in loose if not item["passed"]])

    shorter = core.check_profile(profile, speed_limit_kmh=35.0, time_budget=15.0)
    check("I10 可以同时失败两条",
          len([item for item in shorter if not item["passed"]]) == 2,
          [item["type"] for item in shorter if not item["passed"]])

    triangle = core.build_profile(60.0, 40.0)
    check("I11 三角形也全过", core.all_passed(core.check_profile(triangle)),
          [item for item in core.check_profile(triangle) if not item["passed"]])
    check("I12 三角形峰值进 detail", "34.15" in core.check_profile(triangle)[0]["detail"],
          core.check_profile(triangle)[0]["detail"])

    check("I13 all_passed 空列表为真", core.all_passed([]))
    check("I14 all_passed 有假为假",
          core.all_passed([{"type": u"x", "passed": True, "detail": u""},
                           {"type": u"y", "passed": False, "detail": u""}]) is False)


# ------------------------------------------------------------------ J 字符图与报告


def group_render_report():
    profile = core.build_profile(120.0, 40.0)
    results = core.check_profile(profile)
    lines = core.render_profile(profile)

    check("J1 行数=高度+1", len(lines) == core.DEFAULT_GRID_HEIGHT + 1, len(lines))
    check("J2 每行等宽", all(len(line) == core.DEFAULT_GRID_WIDTH for line in lines[:-1]),
          [len(line) for line in lines[:-1]])
    check("J3 有曲线顶端标记", any(u"*" in line for line in lines))
    check("J4 有填充标记", any(u"#" in line for line in lines))
    check("J5 有横轴", u"0" in lines[-1] and u"s" in lines[-1], lines[-1])
    check("J6 顶部比底部稀疏",
          lines[0].count(u"#") < lines[-2].count(u"#"),
          (lines[0].count(u"#"), lines[-2].count(u"#")))

    small = core.render_profile(profile, 40, 6)
    check("J7 自定义尺寸", len(small) == 7 and len(small[0]) == 40,
          (len(small), len(small[0])))
    check_raises("J8 宽度过小报错", core.ProfileError, core.render_profile, profile, 10)
    check_raises("J9 高度过小报错", core.ProfileError, core.render_profile, profile, 60, 3)

    report = core.format_report(profile, results, u"标题行")
    check("J10 首行是标题", report.splitlines()[0] == u"标题行", report.splitlines()[0])
    check("J11 含曲线类型", u"曲线类型：梯形" in report, report)
    check("J12 含分段", u"分段：" in report)
    check("J13 含总时长", u"总时长 18.21 s" in report, report)
    check("J14 含断言统计", u"断言：4 通过 / 0 失败（共 4 条）" in report, report)
    check("J15 PASS 计数", report.count(u"[PASS]") == 4, report.count(u"[PASS]"))
    check("J16 含结论 PASSED", u"结论：PASSED" in report)
    check("J17 无标题时不空行", not core.format_report(profile, results).startswith(u"\n"))

    failed = core.format_report(profile, core.check_profile(profile, time_budget=10.0))
    check("J18 失败含 FAIL", u"[FAIL]" in failed)
    check("J19 失败结论 FAILED", u"结论：FAILED" in failed)
    check("J20 失败统计正确", u"断言：3 通过 / 1 失败" in failed, failed)

    triangle = core.build_profile(60.0, 40.0)
    triangle_report = core.format_report(triangle, core.check_profile(triangle))
    check("J21 三角形报告类型", u"曲线类型：三角形" in triangle_report, triangle_report)


# ------------------------------------------------------------------ K 参数与工程文件


def group_params_files():
    params = make_params()
    check_close("K1 默认距离", params.distance, 120.0)
    check_close("K2 默认巡航", params.cruise_kmh, 40.0)
    check_close("K3 默认加速度", params.accel, 1.2)
    check_close("K4 默认减速度", params.decel, 2.0)
    check("K5 默认曲线话题", params.topic_profile == core.DEFAULT_TOPIC_PROFILE)
    check("K6 as_tuple 十二项", len(params.as_tuple()) == 12, len(params.as_tuple()))
    check("K7 repr 含类名", "SpeedParams(" in repr(params), repr(params))

    profile = params.profile()
    check("K8 参数能造出曲线", isinstance(profile, core.Profile))
    check_close("K9 参数距离进曲线", profile.distance, params.distance)

    getter = fake_getter({
        "scene/distance": 90.0,
        "scene/cruise_speed_kmh": 30.0,
        "scene/start_speed_kmh": 10.0,
        "vehicle/accel": 0.8,
        "vehicle/decel": 1.5,
        "check/speed_limit_kmh": 40.0,
        "check/time_budget": 25.0,
        "check/tolerance": 0.2,
        "sim/dt": 0.1,
        "rate/publish_hz": 2.0,
        "topic/profile": "/p",
        "topic/report": "/r",
    })
    parsed = core.SpeedParams.from_params(getter)
    check_close("K10 读到距离", parsed.distance, 90.0)
    check_close("K11 读到巡航", parsed.cruise_kmh, 30.0)
    check_close("K12 读到初速", parsed.start_kmh, 10.0)
    check_close("K13 读到加速度", parsed.accel, 0.8)
    check_close("K14 读到减速度", parsed.decel, 1.5)
    check_close("K15 读到限速", parsed.speed_limit_kmh, 40.0)
    check_close("K16 读到预算", parsed.time_budget, 25.0)
    check_close("K17 读到容差", parsed.tolerance, 0.2)
    check_close("K18 读到步长", parsed.dt, 0.1)
    check_close("K19 读到频率", parsed.publish_hz, 2.0)
    check("K20 读到曲线话题", parsed.topic_profile == "/p", parsed.topic_profile)
    check("K21 读到报告话题", parsed.topic_report == "/r", parsed.topic_report)

    fallback = core.SpeedParams.from_params(fake_getter({}))
    check_close("K22 缺键回落默认", fallback.distance, core.DEFAULT_DISTANCE)
    check_raises("K23 距离为零报错", core.ProfileError, core.SpeedParams, distance=0.0)
    check_raises("K24 巡航非数字报错", core.ProfileError, core.SpeedParams,
                 cruise_kmh="abc")

    package_xml = os.path.join(PKG, "package.xml")
    check("K25 package.xml 存在", os.path.isfile(package_xml), package_xml)
    if os.path.isfile(package_xml):
        text = io.open(package_xml, encoding="utf-8").read()
        check("K26 包名正确", u"<name>speed_profile_check</name>" in text)
        check("K27 exec_depend rospy", u"<exec_depend>rospy</exec_depend>" in text)
        check("K28 exec_depend std_msgs", u"<exec_depend>std_msgs</exec_depend>" in text)
        check("K29 有 buildtool_depend", u"<buildtool_depend>catkin</buildtool_depend>" in text)

    cmake = os.path.join(PKG, "CMakeLists.txt")
    check("K30 CMakeLists 存在", os.path.isfile(cmake))
    if os.path.isfile(cmake):
        text = io.open(cmake, encoding="utf-8").read()
        check("K31 find_package catkin", "find_package(catkin REQUIRED" in text)
        check("K32 安装 main.py", "scripts/main.py" in text)
        check("K33 安装 speed_core.py", "scripts/speed_core.py" in text)
        check("K34 安装 speed_node.py", "scripts/speed_node.py" in text)
        check("K35 安装 launch 与 config", "DIRECTORY launch config" in text)
        check("K36 保留源权限", "USE_SOURCE_PERMISSIONS" in text)

    launch = os.path.join(PKG, "launch", "main.launch")
    check("K37 main.launch 存在", os.path.isfile(launch))
    if os.path.isfile(launch):
        text = io.open(launch, encoding="utf-8").read()
        check("K38 入口是 main.py", 'type="main.py"' in text)
        check("K39 加载 yaml", 'command="load"' in text)
        check("K40 arg 带 doc", text.count('doc="') >= 12, text.count('doc="'))
        for index, arg in enumerate(("distance", "cruise_speed_kmh", "start_speed_kmh",
                                     "accel", "decel", "speed_limit_kmh", "time_budget",
                                     "tolerance", "dt", "publish_hz", "profile_topic",
                                     "report_topic")):
            check("K41.%d arg %s" % (index, arg), 'name="%s"' % arg in text, arg)

    config = os.path.join(PKG, "config", "speed_profile_check.yaml")
    check("K42 config yaml 存在", os.path.isfile(config))
    if os.path.isfile(config):
        text = io.open(config, encoding="utf-8").read()
        for index, key in enumerate(("distance", "cruise_speed_kmh", "start_speed_kmh",
                                     "accel", "decel", "speed_limit_kmh", "time_budget",
                                     "tolerance", "dt", "publish_hz", "profile",
                                     "report")):
            check("K43.%d yaml 键 %s" % (index, key), key + ":" in text, key)

    core_source = io.open(os.path.join(SCRIPTS, "speed_core.py"), encoding="utf-8").read()
    check("K44 核心不 import rospy", "\nimport rospy" not in core_source)
    check("K45 核心不 import numpy", "\nimport numpy" not in core_source)
    node_source = io.open(os.path.join(SCRIPTS, "speed_node.py"), encoding="utf-8").read()
    check("K46 节点 import speed_core", "import speed_core" in node_source)
    check("K47 节点用 unicode_literals", "unicode_literals" in node_source)
    check("K48 节点有 _log", "def _log(" in node_source)
    check("K49 节点没直接 loginfo(fmt, args)", 'rospy.loginfo("' not in node_source)
    main_source = io.open(os.path.join(SCRIPTS, "main.py"), encoding="utf-8").read()
    check("K50 main.py 调 speed_node", "from speed_node import main" in main_source)

    check("K51 文档页存在", os.path.isfile(DOCS_PAGE), DOCS_PAGE)
    if os.path.isfile(DOCS_PAGE):
        page = io.open(DOCS_PAGE, encoding="utf-8").read()
        check("K52 文档页标题", page.startswith(u"# 速度曲线规划"), page[:40])
        check("K53 文档页有公式表", u"加速" in page and u"减速" in page)
        check("K54 文档页有断言表", u"限速" in page and u"时间" in page)
        check("K55 文档页有运行命令", u"roslaunch speed_profile_check main.launch" in page)
        check("K56 文档页有离线测试命令",
              u"test_speed_profile_check_local.py" in page)
        check("K57 文档页有大模型声明", u"大模型" in page)

    readme = os.path.join(PKG, "README.md")
    check("K58 README 存在", os.path.isfile(readme))
    if os.path.isfile(readme):
        text = io.open(readme, encoding="utf-8").read()
        check("K59 README 有启动命令",
              u"roslaunch speed_profile_check main.launch" in text)
        check("K60 README 有测试命令", u"test_speed_profile_check_local.py" in text)


# ------------------------------------------------------------------ 主流程


def group_plans():
    plans = core.compare_plans(120.0, 40.0, 1.2, 2.0, 60.0)
    check("L1 给出 5 个候选", len(plans) == 5, str(len(plans)))
    check("L2 按巡航升序",
          [round(item["cruise_kmh"], 1) for item in plans] ==
          [20.0, 30.0, 40.0, 60.0, 70.0],
          repr([round(item["cruise_kmh"], 1) for item in plans]))
    check_close("L3 20 km/h 用时 25.3037", plans[0]["total_time"], 25.3037, 1e-3)
    check_close("L4 30 km/h 用时 19.9556", plans[1]["total_time"], 19.9556, 1e-3)
    check_close("L5 40 km/h 用时 18.2074", plans[2]["total_time"], 18.2074, 1e-3)
    check_close("L6 60 km/h 退化成三角形，峰值 48.2991",
                plans[3]["peak_kmh"], 48.2991, 1e-3)
    check("L7 60 km/h 的曲线类型是三角形",
          plans[3]["kind"] == "triangle", plans[3]["kind"])
    check_close("L8 距离不够时 60 与 70 峰值相同",
                plans[4]["peak_kmh"], plans[3]["peak_kmh"], 1e-9)
    check("L9 默认场景全部未超限",
          all(item["within_limit"] for item in plans),
          repr([item["within_limit"] for item in plans]))

    best = core.recommend_plan(plans)
    check_close("L10 推荐 60 km/h", best["cruise_kmh"], 60.0, 1e-9)
    check_close("L11 推荐方案的用时是最短的",
                best["total_time"], min(item["total_time"] for item in plans), 1e-9)

    lines = core.format_plans(plans, 60.0, 120.0)
    check("L12 表头 + 5 行 + 推荐 = 7 行", len(lines) == 7, str(len(lines)))
    check("L13 首行写明同一段与限速",
          "120.00" in lines[0] and "60.00" in lines[0], lines[0])
    check("L14 每行都有巡航与用时",
          all("巡航" in line and "用时" in line for line in lines[1:6]),
          lines[1])
    check("L15 推荐行写明 60.00", "推荐：60.00 km/h" in lines[-1], lines[-1])

    over = [{"cruise_kmh": 90.0, "peak_kmh": 90.0, "total_time": 10.0,
             "average_kmh": 43.2, "kind": "trapezoid", "within_limit": False}]
    check("L16 全都超限时推荐为 None", core.recommend_plan(over) is None)
    over_lines = core.format_plans(over, 60.0, 120.0)
    check("L17 超限行带（超限）标注", "（超限）" in over_lines[1], over_lines[1])
    check("L18 无能推荐时给出提示", "没有不超限" in over_lines[-1], over_lines[-1])

    profile = core.build_profile(120.0, 40.0, 0.0, 1.2, 2.0)
    results = core.check_profile(profile, 60.0, 30.0, 0.05)
    text = core.format_report(profile, results, plans=plans, speed_limit_kmh=60.0)
    check("L19 报告里出现方案对比", "方案对比" in text, text[-80:])
    check("L20 报告里出现推荐行", "推荐：60.00 km/h" in text)
    plain = core.format_report(profile, results)
    check("L21 不传 plans 时没有对比段", "方案对比" not in plain)
    check_close("L22 候选阶梯最高到 cruise+30",
                max(item["cruise_kmh"] for item in plans), 70.0, 1e-9)
    check_raises("L23 限速为 0 时报错", core.ProfileError,
                 core.compare_plans, 120.0, 40.0, 1.2, 2.0, 0.0)


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
test_trapezoid = _pytest_group(group_trapezoid)
test_triangle = _pytest_group(group_triangle)
test_start_speed = _pytest_group(group_start_speed)
test_errors = _pytest_group(group_errors)
test_sampling_speed = _pytest_group(group_sampling_speed)
test_samples = _pytest_group(group_samples)
test_checks = _pytest_group(group_checks)
test_render_report = _pytest_group(group_render_report)
test_params_files = _pytest_group(group_params_files)
test_plans = _pytest_group(group_plans)


def main():
    print(u"speed_profile_check 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"-" * 60)
    group_defaults()
    group_helpers()
    group_trapezoid()
    group_triangle()
    group_start_speed()
    group_errors()
    group_sampling_speed()
    group_samples()
    group_checks()
    group_render_report()
    group_params_files()
    group_plans()
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
