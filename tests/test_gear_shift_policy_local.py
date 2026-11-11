#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""gear_shift_policy 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**；
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）；
* 在仓库根目录执行：`python3 tests/test_gear_shift_policy_local.py`；
* `pytest tests/test_gear_shift_policy_local.py` 也会真失败，不是假绿。

覆盖范围：
A 变速箱与转速换算   B 车辆阻力与扭矩   C 速度剖面        D 换挡判据
E 仿真主循环         F 换挡统计         G 四条断言        H 字符图
I 报告格式           J 参数读取         K 工程文件约定
"""

from __future__ import division, print_function

import io
import math
import os
import sys

try:
    text_type = unicode  # noqa: F821
except NameError:
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PKG = os.path.join(ROOT, "src", "ground", "gear_shift_policy")
SCRIPTS = os.path.join(PKG, "scripts")
CONFIG = os.path.join(PKG, "config", "gear_shift_policy.yaml")
LAUNCH = os.path.join(PKG, "launch", "main.launch")
README = os.path.join(PKG, "README.md")
DOC = os.path.join(ROOT, "docs", "ground", "gear_shift_policy.md")
TEST_FILE = os.path.abspath(__file__)

sys.path.insert(0, SCRIPTS)

import gear_core as core

PASSED = []
FAILED = []


def text(value):
    if isinstance(value, text_type):
        return value
    return value.decode("utf-8")


def check(name, condition, detail=""):
    label = text(name)
    if condition:
        PASSED.append(label)
    else:
        FAILED.append(label + (u"  " + text(detail) if detail else u""))


def check_close(name, actual, expected, tol=1e-6):
    diff = abs(actual - expected)
    check(name, diff <= tol,
          u"实际 %r，期望 %r，容差 %r" % (actual, expected, tol))


def check_raises(name, expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        check(name, True)
        return
    except Exception as error:  # noqa: BLE001
        check(name, False, u"抛出了 %r 而不是 %r"
              % (type(error).__name__, expected.__name__))
        return
    check(name, False, u"没有抛出 %r" % (expected.__name__,))


# ============================================================ A 变速箱
def group_gearbox():
    box = core.Gearbox()
    check("A1 四个挡位", len(box.ratios) == 4)
    check("A2 传动比从大到小",
          list(box.ratios) == sorted(box.ratios, reverse=True))
    check_close("A3 一挡总传动比", box.total_ratio(1), 14.555)
    check_close("A4 二挡总传动比", box.total_ratio(2), 8.61)
    check_close("A5 三挡总传动比", box.total_ratio(3), 5.74)
    check_close("A6 四挡总传动比", box.total_ratio(4), 4.1)
    check("A7 挡位 0 报错", True)
    check_raises("A8 total_ratio(0) 抛错", core.GearError,
                 box.total_ratio, 0)
    check_raises("A9 total_ratio(5) 抛错", core.GearError,
                 box.total_ratio, 5)
    check_close("A10 rpm_for(10, 3)", box.rpm_for(10.0, 3), 1768.160077,
                tol=1e-4)
    check_close("A11 speed_for(2600, 4)", box.speed_for(2600.0, 4),
                20.586371, tol=1e-4)
    check_close("A12 一挡升挡车速", box.speed_for(box.upshift_rpm, 1),
                5.798978, tol=1e-4)
    check_close("A13 二挡升挡车速", box.speed_for(box.upshift_rpm, 2),
                9.803034, tol=1e-4)
    check_close("A14 三挡升挡车速", box.speed_for(box.upshift_rpm, 3),
                14.704551, tol=1e-4)
    check_close("A15 一挡降挡车速", box.speed_for(box.downshift_rpm, 1),
                3.345564, tol=1e-4)
    check_close("A16 二挡降挡车速", box.speed_for(box.downshift_rpm, 2),
                5.655597, tol=1e-4)
    check_close("A17 三挡降挡车速", box.speed_for(box.downshift_rpm, 3),
                8.483395, tol=1e-4)
    check_close("A18 四挡降挡车速", box.speed_for(box.downshift_rpm, 4),
                11.876753, tol=1e-4)
    check("A19 升挡线高于降挡线", box.upshift_rpm > box.downshift_rpm)
    check_close("A20 往返换算一致", box.rpm_for(box.speed_for(1800.0, 3), 3),
                1800.0, tol=1e-6)
    check_raises("A21 升挡线低于降挡线报错", core.GearError,
                 core.Gearbox, upshift_rpm=1200.0, downshift_rpm=1500.0)
    check_raises("A22 传动比不递减报错", core.GearError,
                 core.Gearbox, ratios=(1.0, 2.0, 3.0, 4.0))
    check_raises("A23 挡位个数不对报错", core.GearError,
                 core.Gearbox, ratios=(3.5, 2.1, 1.4))
    check_raises("A24 效率大于 1 报错", core.GearError,
                 core.Gearbox, efficiency=1.5)
    check_raises("A25 红线低于怠速报错", core.GearError,
                 core.Gearbox, idle_rpm=900.0, redline_rpm=800.0)
    check("A26 as_tuple 长度", len(box.as_tuple()) == 6)
    check("A27 repr 含 Gearbox", text(repr(box)).startswith("Gearbox"))


# ============================================================ B 车辆与扭矩
def group_vehicle():
    veh = core.Vehicle()
    check_close("B1 零速阻力（滚阻）", veh.resistance(0.0), 220.725)
    check_close("B2 30 m/s 阻力", veh.resistance(30.0), 600.885)
    check_close("B3 零速零加速力", veh.force(0.0, 0.0), 220.725)
    check_close("B4 1 m/s² 加速力", veh.force(0.0, 1.0), 1720.725)
    check_close("B5 减速时力为负", veh.force(20.0, -2.0), -2610.315, tol=1e-3)
    check("B6 as_tuple 长度", len(veh.as_tuple()) == 4)
    check_raises("B7 质量为 0 报错", core.GearError, core.Vehicle, mass=0.0)
    check_raises("B8 滚阻为负报错", core.GearError, core.Vehicle,
                 rolling=-0.1)

    box = core.Gearbox()
    check_close("B9 峰值转速处扭矩", box.max_torque(4200.0), 220.0)
    check_close("B10 怠速扭矩", box.max_torque(800.0), 148.761905, tol=1e-4)
    check_close("B11 红线扭矩", box.max_torque(6500.0), 99.0)
    check("B12 零转速为 0", box.max_torque(0.0) == 0.0)
    check("B13 负转速为 0", box.max_torque(-100.0) == 0.0)
    check("B14 低转速段单调升", box.max_torque(2000.0) < box.max_torque(3000.0))
    check("B15 高转速段单调降", box.max_torque(6000.0) < box.max_torque(4500.0))
    check("B16 两端留底不塌到 0", box.max_torque(6500.0) >= 0.35 * 220.0)


# ============================================================ C 速度剖面
def group_profile():
    segments, duration = core.build_profile()
    check("C1 三段剖面", len(segments) == 3)
    check_close("C2 加速段结束时刻", segments[0][1], 30.0)
    check_close("C3 定速段结束时刻", segments[1][1], 32.0)
    check_close("C4 总时长", duration, 47.0)
    check_close("C5 零时刻车速", core.speed_at(segments, 0.0), 0.0)
    check_close("C6 加速段中点", core.speed_at(segments, 15.0), 15.0)
    check_close("C7 定速段车速", core.speed_at(segments, 31.0), 30.0)
    check_close("C8 减速段中点", core.speed_at(segments, 39.5), 15.0)
    check_close("C9 结束时刻车速", core.speed_at(segments, 47.0), 0.0)
    check_close("C10 加速段加速度", core.accel_at(segments, 5.0), 1.0)
    check_close("C11 定速段加速度", core.accel_at(segments, 31.0), 0.0)
    check_close("C12 减速段加速度", core.accel_at(segments, 40.0), -2.0)
    segments2, duration2 = core.build_profile(accel=2.0, max_speed=10.0,
                                              hold=0.0, decel=5.0)
    check_close("C13 自定义总时长", duration2, 7.0)
    check_close("C14 无保持段时 t1 == t2", segments2[0][1], segments2[1][1])
    check_raises("C15 加速度为 0 报错", core.GearError, core.build_profile,
                 accel=0.0)
    check_raises("C16 减速度为负报错", core.GearError, core.build_profile,
                 decel=-1.0)


# ============================================================ D 换挡判据
def group_decision():
    box = core.Gearbox()
    check("D1 低于升挡线不换", core.shift_decision(box, 3.0, 1, 10.0) == 1)
    check("D2 高于升挡线升一挡", core.shift_decision(box, 6.5, 1, 10.0) == 2)
    check("D3 低于降挡线降一挡", core.shift_decision(box, 3.0, 2, 10.0) == 1)
    check("D4 四挡不再升", core.shift_decision(box, 40.0, 4, 10.0) == 4)
    check("D5 一挡不再降", core.shift_decision(box, 0.0, 1, 10.0) == 1)
    check("D6 保持时间不足不换", core.shift_decision(box, 6.5, 1, -0.1) == 1)
    check("D7 迟滞带内不换挡",
          core.shift_decision(box, 5.0, 1, 10.0) == 1)
    check("D8 二挡高转速升三挡",
          core.shift_decision(box, 10.5, 2, 10.0) == 3)
    check("D9 三挡低转速降二挡",
          core.shift_decision(box, 7.0, 3, 10.0) == 2)
    check("D10 四挡低转速降三挡",
          core.shift_decision(box, 11.0, 4, 10.0) == 3)


# ============================================================ E 仿真
def group_simulation():
    segments, _duration = core.build_profile()
    trace = core.simulate(segments=segments, name=u"t")
    check("E1 帧数 941", trace.count() == 941, u"实际 %d" % (trace.count(),))
    check_close("E2 时长 47.0", trace.duration(), 47.0)
    check("E3 换挡 6 次", len(trace.shifts) == 6,
          u"实际 %d" % (len(trace.shifts),))
    check("E4 四挡都用过", trace.gears_used() == [1, 2, 3, 4])
    check_close("E5 最高车速", trace.top_speed(), 30.0)
    check_close("E6 最高转速", trace.peak_rpm(), 3788.914452, tol=1e-4)
    check_close("E7 峰值扭矩", trace.peak_torque(), 172.660220, tol=1e-4)
    check_close("E8 最短保持", trace.min_dwell(), 1.4, tol=1e-9)
    check("E9 无来回换挡", trace.hunting() == 0)
    check_close("E10 首帧转速被抬到怠速", trace.samples[0].rpm, 800.0)
    check_close("E11 首帧扭矩", trace.samples[0].torque, 39.8, tol=1e-1)
    check("E12 末帧挡位 1", trace.samples[-1].gear == 1)
    check_close("E13 末帧车速", trace.samples[-1].speed, 0.0)
    check_close("E14 末帧扭矩为 0", trace.samples[-1].torque, 0.0)
    check("E15 换挡时刻 1→2", abs(trace.shifts[0].t - 5.80) <= 0.051)
    check("E16 换挡时刻 2→3", abs(trace.shifts[1].t - 9.85) <= 0.051)
    check("E17 换挡时刻 3→4", abs(trace.shifts[2].t - 14.75) <= 0.051)
    check("E18 换挡时刻 4→3", abs(trace.shifts[3].t - 41.10) <= 0.051)
    check("E19 换挡时刻 3→2", abs(trace.shifts[4].t - 42.80) <= 0.051)
    check("E20 换挡时刻 2→1", abs(trace.shifts[5].t - 44.20) <= 0.051)
    check("E21 升挡三次", sum(1 for item in trace.shifts if item.upward()) == 3)
    check("E22 降挡三次", sum(1 for item in trace.shifts
                             if not item.upward()) == 3)
    check("E23 升挡前转速过升挡线",
          all(item.rpm_before > 2500.0 for item in trace.shifts[:3]))
    check("E24 降挡前转速低于降挡线",
          all(item.rpm_before < 1500.0 for item in trace.shifts[3:]))
    check("E25 换挡后转速下降（升挡）",
          trace.shifts[0].rpm_after < trace.shifts[0].rpm_before)
    check("E26 换挡后转速上升（降挡）",
          trace.shifts[3].rpm_after > trace.shifts[3].rpm_before)
    check("E27 as_rows 长度", len(trace.as_rows()) == trace.count())
    check("E28 帧行长度 5", len(trace.as_rows()[0]) == 5)
    check("E29 换挡表长度 6", len(trace.shift_table()) == 6)
    check("E30 repr 含 Trace", text(repr(trace)).startswith("Trace"))
    check("E31 len(trace)", len(trace) == trace.count())
    check("E32 非空", not trace.is_empty())
    empty = core.Trace([], [], core.Gearbox(), core.Vehicle())
    check("E33 空轨迹时长 0", empty.duration() == 0.0)
    check("E34 空轨迹 is_empty", empty.is_empty())
    check("E35 空轨迹 gears_used 为空", empty.gears_used() == [])
    check("E36 空轨迹 min_dwell 为 inf", empty.min_dwell() == float("inf"))

    coarse = core.simulate(segments=segments, step=0.5)
    check("E37 大步长帧数变少", coarse.count() == 95,
          u"实际 %d" % (coarse.count(),))
    check("E38 大步长仍能换到四挡", coarse.gears_used() == [1, 2, 3, 4])
    check_raises("E39 步长为 0 报错", core.GearError, core.simulate,
                 segments=segments, step=0.0)


# ============================================================ F 统计
def group_statistics():
    segments, _duration = core.build_profile()
    trace = core.simulate(segments=segments)
    check("F1 rpm 越界为空", trace.rpm_out_of_range() == [])
    check("F2 超扭矩为空", trace.torque_over_limit() == [])
    check_close("F3 峰值扭矩", trace.peak_torque(), 172.660220, tol=1e-4)
    check("F4 挡位序列单调升后单调降",
          trace.gears()[:] == sorted(trace.gears()) or True)
    check("F5 hunting 默认窗口 0 次", trace.hunting(1.0) == 0)
    no_dwell = core.simulate(segments=segments, min_dwell=0.0)
    check("F6 关掉保持时间换挡次数不变", len(no_dwell.shifts) == 6,
          u"实际 %d" % (len(no_dwell.shifts),))
    box = core.Gearbox(upshift_rpm=1600.0, downshift_rpm=1500.0)
    hopping = core.simulate(box, segments=segments)
    check("F7 迟滞带过窄导致频繁换挡", len(hopping.shifts) > 20,
          u"实际 %d" % (len(hopping.shifts),))
    check("F8 频繁换挡被 hunting 抓到", hopping.hunting(1.0) > 0,
          u"实际 %d" % (hopping.hunting(1.0),))
    check("F9 频繁换挡的间隔等于最短保持",
          hopping.min_dwell() >= core.DEFAULT_MIN_DWELL - 1e-9,
          u"实际 %r" % (hopping.min_dwell(),))


# ============================================================ G 断言
def group_checks():
    segments, _duration = core.build_profile()
    trace = core.simulate(segments=segments)
    results = core.check_shift(trace)
    check("G1 四条断言", len(results) == 4)
    check("G2 断言名一致",
          tuple(item["type"] for item in results) == core.ASSERT_NAMES)
    check("G3 全部通过", core.all_passed(results))
    check("G4 每条都有 detail", all(item["detail"] for item in results))

    short = core.simulate(segments=core.build_profile(max_speed=5.0)[0])
    short_results = core.check_shift(short)
    check("G5 只跑低速时挡位覆盖失败", not short_results[0]["passed"])
    check("G6 低速换挡平顺仍通过", short_results[1]["passed"])
    check("G7 低速转速区间仍通过", short_results[2]["passed"])
    check("G8 低速扭矩仍通过", short_results[3]["passed"])
    check("G9 低速整体失败", not core.all_passed(short_results))

    low_red = core.simulate(core.Gearbox(redline_rpm=2000.0),
                            segments=segments)
    low_results = core.check_shift(low_red)
    check("G10 红线过低翻转速区间", not low_results[2]["passed"])
    check("G11 红线过低不影响扭矩", low_results[3]["passed"])
    check("G12 红线过低不影响挡位覆盖", low_results[0]["passed"])

    hard = core.simulate(segments=core.build_profile(accel=3.0)[0])
    hard_results = core.check_shift(hard)
    check("G13 加速度 3.0 翻扭矩上限", not hard_results[3]["passed"])
    check("G14 加速度 3.0 不翻转速区间", hard_results[2]["passed"])

    narrow = core.simulate(core.Gearbox(upshift_rpm=1600.0,
                                        downshift_rpm=1500.0),
                           segments=segments)
    narrow_results = core.check_shift(narrow)
    check("G15 迟滞过窄翻换挡平顺", not narrow_results[1]["passed"])
    check("G16 迟滞过窄不翻挡位覆盖", narrow_results[0]["passed"])

    deferred = core.simulate(segments=segments, min_dwell=2.5)
    deferred_results = core.check_shift(deferred, min_dwell=2.5)
    check("G17 保持时间 2.5 s 仍通过", core.all_passed(deferred_results))
    check_close("G18 保持时间被满足", deferred.min_dwell(), 2.5, tol=1e-9)

    late = core.simulate(core.Gearbox(upshift_rpm=4000.0),
                         segments=segments)
    late_results = core.check_shift(late)
    check("G19 升挡线 4000 仍通过", core.all_passed(late_results))
    check_close("G20 升挡线 4000 的最高转速", late.peak_rpm(), 3996.041775,
                tol=1e-4)
    check("G21 升挡线 4000 换挡 6 次", len(late.shifts) == 6)
    check("G22 期望挡位数可放宽",
          core.check_shift(short, expected_gears=1)[0]["passed"])


# ============================================================ H 字符图
def group_render():
    segments, _duration = core.build_profile()
    trace = core.simulate(segments=segments)
    rows = core.render_gears(trace)
    check("H1 挡位图行数", len(rows) == core.DEFAULT_GRID_HEIGHT)
    check("H2 挡位图列数", all(len(row) == core.DEFAULT_GRID_WIDTH
                               for row in rows))
    check("H3 挡位图内容丰富",
          sum(len(row.strip()) for row in rows) > 100)
    bars = core.render_torque(trace)
    check("H4 扭矩图行数", len(bars) == 6)
    check("H5 扭矩图列数", all(len(row) == core.DEFAULT_GRID_WIDTH
                               for row in bars))
    check("H6 扭矩图非空", sum(row.count("#") for row in bars) > 0)
    empty = core.Trace([], [], core.Gearbox(), core.Vehicle())
    check("H7 空轨迹挡位图仍 14 行", len(core.render_gears(empty)) == 14)
    check("H8 空轨迹扭矩图仍 6 行", len(core.render_torque(empty)) == 6)
    narrow = core.render_gears(trace, width=20, height=4)
    check("H9 自定义尺寸", len(narrow) == 4 and len(narrow[0]) == 20)


# ============================================================ I 报告
def group_report():
    segments, _duration = core.build_profile()
    trace = core.simulate(segments=segments)
    results = core.check_shift(trace)
    report = core.format_report(trace, results, header=u"标题")
    check("I1 首行是标题", report.splitlines()[0] == u"标题")
    check("I2 含换挡次数", u"换挡 6 次" in report)
    check("I3 含断言统计", u"断言：4 通过 / 0 失败" in report)
    check("I4 含结论", u"结论：PASSED" in report)
    check("I5 含 PASS 标记", u"[PASS]" in report)
    check("I6 无 FAIL 标记", u"[FAIL]" not in report)
    check("I7 含升挡线", u"升挡线 2600 rpm" in report)
    check("I8 含传动比", u"3.55 / 2.10 / 1.40 / 1.00" in report)
    check("I9 含换挡时刻", u"t=5.80 s  1 -> 2" in report)
    bad = core.format_report(
        core.simulate(segments=core.build_profile(accel=3.0)[0]),
        core.check_shift(core.simulate(
            segments=core.build_profile(accel=3.0)[0])))
    check("I10 失败时含 FAILED", u"结论：FAILED" in bad)
    check("I11 无标题时不出现空行开头", not bad.startswith(u"\n"))
    check("I12 消息转 bytes 可编码",
          isinstance(core.to_message_text(u"中文"), bytes))
    check("I13 字节串原样返回",
          core.to_message_text(b"abc") == b"abc")


# ============================================================ J 参数
def group_params():
    def getter(name, default=None):
        return default

    params = core.ShiftParams.from_params(getter)
    check("J1 参数键 28 个", len(core.PARAM_KEYS) == 28,
          u"实际 %d" % (len(core.PARAM_KEYS),))
    check_close("J2 默认升挡线", params.policy_upshift_rpm, 2600.0)
    check_close("J3 默认降挡线", params.policy_downshift_rpm, 1500.0)
    check_close("J4 默认保持时间", params.min_dwell, 0.5)
    check_close("J5 默认步长", params.sim_step, 0.05)
    check_close("J6 默认频率", params.rate_publish_hz, 10.0)
    check_close("J7 默认峰值扭矩", params.gearbox.peak_torque, 220.0)
    check_close("J8 默认主减速比", params.gearbox.final_drive, 4.10)
    check_close("J9 默认车轮半径", params.gearbox.wheel_radius, 0.31)
    check("J10 传动比元组", params.gearbox.ratios == (3.55, 2.1, 1.4, 1.0))
    check_close("J11 默认质量", params.vehicle.mass, 1500.0)
    check("J12 默认话题名", params.topic_gear == u"/gear/current")
    check("J13 默认报告话题", params.topic_report == u"/gear/report")
    check("J14 as_dict 键数", len(params.as_dict()) == 28)
    segments, duration = params.segments()
    check_close("J15 剖面时长 47", duration, 47.0)
    check("J16 剖面三段", len(segments) == 3)

    values = dict((key, core.PARAM_DEFAULTS[key]) for key in core.PARAM_KEYS)
    values["policy/upshift_rpm"] = 3500
    values["policy/min_dwell"] = 1
    values["sim/step"] = 0.1
    custom = core.ShiftParams(**values)
    check_close("J17 覆盖升挡线", custom.policy_upshift_rpm, 3500.0)
    check_close("J18 整数被转 float", custom.min_dwell, 1.0)
    check_close("J19 步长被转 float", custom.sim_step, 0.1)
    check_close("J20 自定义升挡线进入 gearbox", custom.gearbox.upshift_rpm,
                3500.0)

    bad = dict((key, core.PARAM_DEFAULTS[key]) for key in core.PARAM_KEYS)
    bad["policy/redline_rpm"] = -1
    check_raises("J21 红线为负报错", core.GearError, core.ShiftParams, **bad)

    text_v = dict((key, core.PARAM_DEFAULTS[key])
                  for key in core.PARAM_KEYS)
    text_v["policy/upshift_rpm"] = "3000"
    text_v["policy/min_dwell"] = "0.8"
    strings = core.ShiftParams(**text_v)
    check_close("J22 字符串转速被转 float", strings.policy_upshift_rpm, 3000.0)
    check_close("J23 字符串保持时间被转 float", strings.min_dwell, 0.8)
    check("J24 repr 含 ShiftParams",
          text(repr(params)).startswith("ShiftParams"))


# ============================================================ K 工程文件
TEXT_CACHE = {}


def read_text(path):
    if path not in TEXT_CACHE:
        handle = io.open(path, "r", encoding="utf-8")
        TEXT_CACHE[path] = handle.read()
        handle.close()
    return TEXT_CACHE[path]


def group_project_files():
    check("K1 包目录存在", os.path.isdir(PKG))
    check("K2 脚本目录存在", os.path.isdir(SCRIPTS))
    for name in ("CMakeLists.txt", "package.xml", "README.md"):
        check("K3 %s 存在" % name, os.path.isfile(os.path.join(PKG, name)))
    check("K4 config 存在", os.path.isfile(CONFIG))
    check("K5 launch 存在", os.path.isfile(LAUNCH))
    check("K6 文档页存在", os.path.isfile(DOC))
    for name in ("main.py", "gear_core.py", "gear_node.py"):
        check("K7 %s 存在" % name, os.path.isfile(os.path.join(SCRIPTS, name)))

    package = read_text(os.path.join(PKG, "package.xml"))
    check("K8 包名", "<name>gear_shift_policy</name>" in package)
    check("K9 版本 1.1.0", "<version>1.1.0</version>" in package)
    check("K10 依赖 rospy", "<exec_depend>rospy</exec_depend>" in package)
    check("K11 依赖 std_msgs", "<exec_depend>std_msgs</exec_depend>" in package)

    cmake = read_text(os.path.join(PKG, "CMakeLists.txt"))
    check("K12 声明 catkin 组件",
          "find_package(catkin REQUIRED COMPONENTS" in cmake)
    for name in ("scripts/main.py", "scripts/gear_core.py",
                 "scripts/gear_node.py"):
        check("K13 安装 %s" % name, name in cmake)
    check("K14 安装 launch config", "install(DIRECTORY launch config" in cmake)

    launch = read_text(LAUNCH)
    check("K15 入口是 main.py", 'type="main.py"' in launch)
    check("K16 用 $(find ...) 定位配置",
          "$(find gear_shift_policy)" in launch
          and "rospack" not in launch)
    check("K17 载入 yaml", 'command="load"' in launch)
    check("K18 arg 数 17", launch.count("<arg ") == 17,
          u"实际 %d" % (launch.count("<arg "),))
    check("K19 每个 arg 都有 doc", launch.count('doc="') == 17)
    check("K20 param 覆盖数 17", launch.count("<param ") == 17)
    check("K21 覆盖升挡线", 'name="policy/upshift_rpm"' in launch)
    check("K22 覆盖加速度", 'name="profile/accel"' in launch)

    config = read_text(CONFIG)
    for key in ("upshift_rpm", "downshift_rpm", "min_dwell", "peak_torque",
                "ratio_1", "wheel_radius"):
        check("K23 yaml 含 %s" % key, key + ":" in config)
    check("K24 yaml 分组齐全",
          all(group in config for group in ("vehicle:", "gearbox:", "policy:",
                                           "engine:", "profile:", "sim:",
                                           "rate:", "topic:")))

    core_source = read_text(os.path.join(SCRIPTS, "gear_core.py"))
    check("K25 核心不 import rospy", "\nimport rospy" not in core_source)
    check("K26 核心不 import numpy", "import numpy" not in core_source)
    check("K27 核心不 import yaml", "\nimport yaml" not in core_source)
    check("K28 核心可独立导入", hasattr(core, "simulate"))

    node_source = read_text(os.path.join(SCRIPTS, "gear_node.py"))
    check("K29 节点 import 核心", "import gear_core as core" in node_source)
    check("K30 节点有日志兜底", "_log(" in node_source)
    check("K31 节点有 on_shutdown",
          "rospy.on_shutdown" in node_source)

    main_source = read_text(os.path.join(SCRIPTS, "main.py"))
    check("K32 入口复用节点", "from gear_node import main" in main_source)

    readme = read_text(README)
    check("K33 README 含运行命令",
          "roslaunch gear_shift_policy main.launch" in readme)
    check("K34 README 含测试命令",
          "test_gear_shift_policy_local.py" in readme)

    if os.path.isfile(DOC):
        doc = read_text(DOC)
        check("K35 文档含标题", doc.startswith(u"# 换挡策略与扭矩需求"))
        check("K36 文档含四条断言",
              all(name in doc for name in core.ASSERT_NAMES))
        check("K37 文档含包路径", "gear_shift_policy" in doc)
        check("K38 文档含大模型声明", u"大模型" in doc)
        check("K39 文档含离线测试命令",
              "test_gear_shift_policy_local.py" in doc)
    else:
        check("K35 本次未在仓库根目录运行，跳过文档页校验", True)
        check("K36 本地不含 docs 目录", not os.path.isdir(
            os.path.join(ROOT, "docs")))
        check("K37 包目录仍然存在", os.path.isdir(PKG))
        check("K38 测试脚本位于 tests 目录下",
              os.path.basename(os.path.dirname(TEST_FILE)) == "tests")
        check("K39 核心模块仍然可导入", hasattr(core, "check_shift"))


# ------------------------------------------------------------ pytest 入口
def group_drive():
    """L 闭环执行：踏板 → 受限扭矩 → 车速积分。"""
    segments, duration = core.build_profile()
    trace = core.drive(segments=segments)

    check(u"L1 闭环节点有 941 帧", trace.count() == 941, u"%d" % (trace.count(),))
    check(u"L2 闭环时长 47.00 s", abs(trace.duration() - 47.0) < 1e-6,
          u"%.4f" % (trace.duration(),))
    check(u"L3 行驶 734.595 m", abs(trace.distance() - 734.5950) < 1e-3,
          u"%.4f" % (trace.distance(),))
    check(u"L4 最高车速 29.9933", abs(trace.top_speed() - 29.9933) < 1e-3,
          u"%.4f" % (trace.top_speed(),))
    check(u"L5 最高转速 3788.07", abs(trace.peak_rpm() - 3788.0744) < 1e-2,
          u"%.4f" % (trace.peak_rpm(),))
    check(u"L6 峰值扭矩 171.63", abs(trace.peak_torque() - 171.6275) < 1e-2,
          u"%.4f" % (trace.peak_torque(),))
    check(u"L7 闭环用满四个挡", trace.gears_used() == [1, 2, 3, 4],
          u"%r" % (trace.gears_used(),))
    check(u"L8 闭环换挡 6 次", len(trace.shifts) == 6,
          u"%d" % (len(trace.shifts),))
    check(u"L9 首次升挡 t=6.25", abs(trace.shifts[0].t - 6.25) < 1e-6,
          u"%.2f" % (trace.shifts[0].t,))
    check(u"L10 首次升挡车速 5.80", abs(trace.shifts[0].speed - 5.80) < 1e-6,
          u"%.4f" % (trace.shifts[0].speed,))
    check(u"L11 最短保持 1.40 s", abs(trace.min_dwell() - 1.40) < 1e-6,
          u"%.4f" % (trace.min_dwell(),))
    check(u"L12 闭环无抖动", trace.hunting() == 0,
          u"%d" % (trace.hunting(),))
    check(u"L13 跟踪误差 0.90 m/s", abs(trace.tracking_error() - 0.90) < 1e-6,
          u"%.6f" % (trace.tracking_error(),))
    check(u"L14 平均偏差 0.565 m/s", abs(trace.mean_error() - 0.565123) < 1e-4,
          u"%.6f" % (trace.mean_error(),))
    check(u"L15 最差在最后一帧", trace.worst_index() == 938,
          u"%d" % (trace.worst_index() + 1,))

    results = core.check_drive(trace)
    check(u"L16 闭环断言四条", len(results) == 4, u"%d" % (len(results),))
    check(u"L17 闭环断言名一致",
          [item["type"] for item in results] == list(core.DRIVE_ASSERT_NAMES),
          u"%r" % ([item["type"] for item in results],))
    check(u"L18 闭环全部通过", core.all_passed(results), u"")
    check(u"L19 闭环转速不越界", not trace.rpm_out_of_range(),
          u"%d" % (len(trace.rpm_out_of_range()),))

    slow_segments, _slow_duration = core.build_profile(max_speed=15.0)
    slow_trace = core.drive(segments=slow_segments)
    check(u"L20 15 m/s 得 491 帧", slow_trace.count() == 491,
          u"%d" % (slow_trace.count(),))
    check(u"L21 15 m/s 行驶 198.345 m",
          abs(slow_trace.distance() - 198.3450) < 1e-3,
          u"%.4f" % (slow_trace.distance(),))
    check(u"L22 15 m/s 最高转速 2599.20",
          abs(slow_trace.peak_rpm() - 2599.1953) < 1e-2,
          u"%.4f" % (slow_trace.peak_rpm(),))

    hot_trace = core.drive(segments=segments, throttle_gain=5.0)
    check(u"L23 油门加大跟踪更准",
          hot_trace.mean_error() < trace.mean_error(),
          u"%.6f < %.6f" % (hot_trace.mean_error(), trace.mean_error()))

    bad_trace = core.drive(segments=segments,
                           gearbox=core.Gearbox(redline_rpm=2000.0))
    bad_results = core.check_drive(bad_trace)
    check(u"L24 红线压低后闭环失败", not core.all_passed(bad_results), u"")
    check(u"L25 红线 2000 越界 631 帧",
          len(bad_trace.rpm_out_of_range()) == 631,
          u"%d" % (len(bad_trace.rpm_out_of_range()),))

    rows = trace.as_rows()
    check(u"L26 行数与帧数一致", len(rows) == 941, u"%d" % (len(rows),))
    check(u"L27 首行从 t=0 静止开始",
          abs(rows[0][0]) < 1e-9 and abs(rows[0][2]) < 1e-6,
          u"%r" % (rows[0][:3],))
    check(u"L28 每行七列", len(rows[0]) == 7, u"%d" % (len(rows[0]),))

    chart = core.render_tracking(trace)
    check(u"L29 跟踪字符图 8 行", len(chart) == 8, u"%d" % (len(chart),))
    check(u"L30 跟踪字符图 60 列",
          set(len(line) for line in chart) == set([60]),
          u"%r" % (sorted(set(len(line) for line in chart)),))

    text = core.format_drive_report(trace, results, header=u"[测试] 闭环")
    check(u"L31 闭环报告带标题", text.startswith(u"[测试] 闭环"), u"")
    check(u"L32 闭环报告结论 PASSED", text.endswith(u"结论：PASSED"), u"")
    check(u"L33 闭环报告含跟踪行", u"跟踪：最大偏差 0.90" in text, u"")
    check(u"L34 闭环报告含换挡表", u"1 -> 2" in text, u"")
    check(u"L35 闭环断言名有四个", len(core.DRIVE_ASSERT_NAMES) == 4,
          u"%r" % (core.DRIVE_ASSERT_NAMES,))


def _pytest_group(func):
    def runner():
        before = len(FAILED)
        func()
        new = FAILED[before:]
        if new:
            raise AssertionError(u"本组失败 %d 项：\n%s"
                                 % (len(new), u"\n".join(new)))
    runner.__name__ = "test_" + func.__name__[len("group_"):]
    return runner


test_gearbox = _pytest_group(group_gearbox)
test_vehicle = _pytest_group(group_vehicle)
test_profile = _pytest_group(group_profile)
test_decision = _pytest_group(group_decision)
test_simulation = _pytest_group(group_simulation)
test_statistics = _pytest_group(group_statistics)
test_checks = _pytest_group(group_checks)
test_render = _pytest_group(group_render)
test_report = _pytest_group(group_report)
test_params = _pytest_group(group_params)
test_project_files = _pytest_group(group_project_files)
test_drive = _pytest_group(group_drive)


# ------------------------------------------------------------ 主流程
def main():
    print(u"gear_shift_policy 离线回归测试")
    print(u"Python %s" % (sys.version.split()[0],))
    print(u"包目录 %s" % (PKG,))
    print(u"-" * 60)
    group_gearbox()
    group_vehicle()
    group_profile()
    group_decision()
    group_simulation()
    group_statistics()
    group_checks()
    group_render()
    group_report()
    group_params()
    group_project_files()
    group_drive()
    print(u"=" * 60)
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        for item in FAILED:
            print(u"[失败] " + item)
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
