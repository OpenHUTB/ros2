#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ultrasonic_rear_alert 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**；
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）；
* 在仓库根目录执行：python3 tests/test_ultrasonic_rear_alert_local.py

覆盖范围：
A 阈值与数值工具        B 读数解析            C 判定器基础
D 迟滞与毛刺            E 轨迹指标            F 四条断言
G 字符图                H 报告格式            I 参数读取
J 演示场景端到端        K 工程文件约定
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
PKG = os.path.join(ROOT, "src", "ground", "ultrasonic_rear_alert")
SCRIPTS = os.path.join(PKG, "scripts")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "ultrasonic_rear_alert.md")

sys.path.insert(0, SCRIPTS)

import alert_core as core  # noqa: E402

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(u"%s %s" % (name, detail))


def close(actual, expected, tolerance=1e-9):
    return abs(actual - expected) <= tolerance


def raises(expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        return True
    except Exception:
        return False
    return False


def read_text(path):
    return io.open(path, "r", encoding="utf-8").read()


# ---------------------------------------------------------------- A 阈值与工具
def group_thresholds():
    detector = core.Detector()
    check(u"A1 默认进入阈值从大到小", detector.enter == (2.0, 1.5, 1.0, 0.5),
          u"实得 %r" % (detector.enter,))
    check(u"A2 退出阈值 = 进入 + 0.20",
          close(detector.exit_threshold(1), 2.20)
          and close(detector.exit_threshold(2), 1.70)
          and close(detector.exit_threshold(3), 1.20)
          and close(detector.exit_threshold(4), 0.70),
          u"实得 %r" % ([detector.exit_threshold(i) for i in (1, 2, 3, 4)],))
    check(u"A3 退出阈值越界报错",
          raises(core.AlertError, detector.exit_threshold, 0)
          and raises(core.AlertError, detector.exit_threshold, 5))
    check(u"A4 2.60 m 是 0 级", detector.raw_level(2.60) == 0,
          u"实得 %d" % (detector.raw_level(2.60),))
    check(u"A5 2.00 m 是 1 级", detector.raw_level(2.00) == 1,
          u"实得 %d" % (detector.raw_level(2.00),))
    check(u"A6 1.50 m 是 2 级", detector.raw_level(1.50) == 2,
          u"实得 %d" % (detector.raw_level(1.50),))
    check(u"A7 1.00 m 是 3 级", detector.raw_level(1.00) == 3,
          u"实得 %d" % (detector.raw_level(1.00),))
    check(u"A8 0.50 m 是 4 级", detector.raw_level(0.50) == 4,
          u"实得 %d" % (detector.raw_level(0.50),))
    check(u"A9 0.42 m 仍是 4 级", detector.raw_level(0.42) == 4,
          u"实得 %d" % (detector.raw_level(0.42),))
    check(u"A10 无回波是 0 级", detector.raw_level(None) == 0,
          u"实得 %d" % (detector.raw_level(None),))
    check(u"A11 低于量程下限按无回波", detector.raw_level(0.05) == 0,
          u"实得 %d" % (detector.raw_level(0.05),))
    check(u"A12 高于量程上限按无回波", detector.raw_level(3.50) == 0,
          u"实得 %d" % (detector.raw_level(3.50),))
    check(u"A13 阈值必须从大到小",
          raises(core.AlertError, core.Detector, (1.0, 2.0, 1.5, 0.5)))
    check(u"A14 阈值个数必须是 4",
          raises(core.AlertError, core.Detector, (2.0, 1.0, 0.5)))
    check(u"A15 确认帧数至少 1",
          raises(core.AlertError, core.Detector, confirm_frames=0))
    check(u"A16 释放帧数至少 1",
          raises(core.AlertError, core.Detector, release_frames=0))
    check(u"A17 量程上限必须大于下限",
          raises(core.AlertError, core.Detector, min_range=1.0, max_range=1.0))
    check(u"A18 负的退出余量报错",
          raises(core.AlertError, core.Detector, exit_margin=-0.1))

    check(u"A19 _as_float 认字符串", close(core._as_float(u"1.5", "x"), 1.5))
    check(u"A20 _as_float 空值回落默认",
          close(core._as_float(None, "x", 2.5), 2.5))
    check(u"A21 _as_float 拒绝 nan",
          raises(core.AlertError, core._as_float, u"nan", "x"))
    check(u"A22 _as_int 认浮点整数", core._as_int(3.0, "x") == 3)
    check(u"A23 _as_int 拒绝小数", raises(core.AlertError, core._as_int, 3.7, "x"))
    check(u"A24 _positive 拒绝 0", raises(core.AlertError, core._positive, 0.0, "x"))


# ---------------------------------------------------------------- B 读数解析
def group_reading():
    text = u"0.0 2.60 2.70 2.80 2.90\n0.1 1.10 1.20 1.30 1.40\n"
    readings = core.parse_readings_text(text, "两帧")
    check(u"B1 解析出两帧", len(readings) == 2, u"实得 %d" % (len(readings),))
    check(u"B2 第一帧最近距离 2.60", close(readings[0].nearest(), 2.60),
          u"实得 %r" % (readings[0].nearest(),))
    check(u"B3 第一帧最近通道是 0", readings[0].nearest_index() == 0,
          u"实得 %d" % (readings[0].nearest_index(),))
    check(u"B4 第二帧最近通道是 0", readings[1].nearest_index() == 0,
          u"实得 %d" % (readings[1].nearest_index(),))

    dashed = core.parse_readings_text(u"0.0 - - - -\n", "全无回波")
    check(u"B5 横杠表示无回波", dashed[0].nearest() is None,
          u"实得 %r" % (dashed[0].nearest(),))
    check(u"B6 全无回波时通道号是 -1", dashed[0].nearest_index() == -1,
          u"实得 %d" % (dashed[0].nearest_index(),))

    mixed = core.parse_readings_text(u"0.0 2.00 - 1.20 -\n", "混合")
    check(u"B7 混合时跳过无回波", close(mixed[0].nearest(), 1.20),
          u"实得 %r" % (mixed[0].nearest(),))
    check(u"B8 混合时通道号是 2", mixed[0].nearest_index() == 2,
          u"实得 %d" % (mixed[0].nearest_index(),))

    check(u"B9 字段不足时报错",
          raises(core.AlertError, core.parse_readings_text, u"0.0 1.0 2.0\n", "缺字段"))
    check(u"B10 空文本报错",
          raises(core.AlertError, core.parse_readings_text, u"# 只有注释\n", "空"))
    check(u"B11 注释行被跳过",
          len(core.parse_readings_text(u"# 注释\n0.0 1 1 1 1\n", "带注释")) == 1)

    demo = core.demo_readings()
    check(u"B12 内置演示 23 帧", len(demo) == 23, u"实得 %d" % (len(demo),))
    check(u"B13 首帧最近 2.90", close(demo[0].nearest(), 2.90),
          u"实得 %r" % (demo[0].nearest(),))
    check(u"B14 末帧最近 3.00", close(demo[-1].nearest(), 3.00),
          u"实得 %r" % (demo[-1].nearest(),))
    check(u"B15 第 13 帧左后通道丢回波",
          demo[13].distances[0] is not None and demo[13].distances[0] > 2.0,
          u"实得 %r" % (demo[13].distances,))


# ---------------------------------------------------------------- C 判定器基础
def group_detector():
    detector = core.Detector()
    grades = [detector.feed(t, [d] * 4)
              for t, d in ((0.0, 2.60), (0.1, 2.60), (0.2, 2.60))]
    check(u"C1 一直很远时保持 0 级", all(g.level == 0 for g in grades),
          u"实得 %r" % ([g.level for g in grades],))

    detector = core.Detector()
    grades = [detector.feed(0.1 * i, [1.90] * 4) for i in range(4)]
    check(u"C2 1.90 m 三帧后升到 1 级",
          [g.level for g in grades] == [0, 0, 1, 1],
          u"实得 %r" % ([g.level for g in grades],))

    detector = core.Detector()
    grades = [detector.feed(0.1 * i, [0.40] * 4) for i in range(3)]
    check(u"C3 直接跳到 4 级（不逐级播报）",
          [g.level for g in grades] == [0, 0, 4],
          u"实得 %r" % ([g.level for g in grades],))

    detector = core.Detector()
    detector.feed(0.0, [0.40] * 4)
    detector.feed(0.1, [0.40] * 4)
    first = detector.feed(0.2, [0.40] * 4)
    check(u"C4 升级那一帧 changed 为真", first.changed is True)
    second = detector.feed(0.3, [0.40] * 4)
    check(u"C5 保持不变时 changed 为假", second.changed is False)

    detector = core.Detector()
    detector.reset()
    check(u"C6 reset 后回到 0 级", detector.level == 0)
    check(u"C7 reset 后通道号回到 -1", detector.sensor == -1)
    check(u"C8 reset 后距离清空", detector.distance is None)

    detector = core.Detector()
    detector.feed(0.0, [0.40, 2.90, 2.90, 2.90])
    check(u"C9 记录触发通道", detector.sensor == 0,
          u"实得 %d" % (detector.sensor,))
    check(u"C10 记录最近距离", close(detector.distance, 0.40),
          u"实得 %r" % (detector.distance,))

    detector = core.Detector()
    grade = detector.feed(0.5, [1.20] * 4)
    check(u"C11 raw_level 写在结果里", grade.raw_level == 2,
          u"实得 %d" % (grade.raw_level,))
    check(u"C12 首次触线时级别仍是 0", grade.level == 0,
          u"实得 %d" % (grade.level,))


# ---------------------------------------------------------------- D 迟滞与毛刺
def group_hysteresis():
    detector = core.Detector()
    grades = []
    for index in range(3):
        grades.append(detector.feed(0.1 * index, [0.40] * 4))
    base = grades[-1].level
    spike = detector.feed(0.3, [2.90] * 4)
    check(u"D1 单帧跳远不改变级别", spike.level == base == 4,
          u"实得 %d（之前 %d）" % (spike.level, base))
    recover = detector.feed(0.4, [0.40] * 4)
    check(u"D2 毛刺后级别仍是 4", recover.level == 4,
          u"实得 %d" % (recover.level,))

    detector = core.Detector()
    for index in range(3):
        detector.feed(0.1 * index, [0.40] * 4)
    check(u"D3 升级后级别为 4", detector.level == 4)
    detector.feed(0.3, [0.55] * 4)
    check(u"D4 0.55 m 未过退出阈值仍保持 4", detector.level == 4,
          u"实得 %d" % (detector.level,))
    detector.feed(0.4, [0.80] * 4)
    check(u"D5 0.80 m 过了退出阈值也开始数帧但还没降", detector.level == 4,
          u"实得 %d" % (detector.level,))
    detector.feed(0.5, [0.80] * 4)
    detector.feed(0.6, [0.80] * 4)
    check(u"D6 连续三帧过退出阈值后降级", detector.level == 3,
          u"实得 %d" % (detector.level,))

    no_margin = core.Detector(exit_margin=0.0)
    for index in range(3):
        no_margin.feed(0.1 * index, [0.40] * 4)
    check(u"D7 无余量时升级到 4", no_margin.level == 4)
    no_margin.feed(0.4, [1.10] * 4)
    no_margin.feed(0.5, [1.10] * 4)
    no_margin.feed(0.6, [1.10] * 4)
    check(u"D8 无余量时三帧即降级到即时级别", no_margin.level == 2,
          u"实得 %d" % (no_margin.level,))

    loose = core.Detector(confirm_frames=1, release_frames=1)
    grades = [loose.feed(0.1 * i, [d] * 4)
              for i, d in enumerate((2.60, 1.90, 0.90, 1.90, 2.60))]
    check(u"D9 确认 1 帧时逐帧跟随",
          [g.level for g in grades] == [0, 1, 3, 1, 0],
          u"实得 %r" % ([g.level for g in grades],))


# ---------------------------------------------------------------- E 轨迹指标
def group_trace():
    trace = core.run_detector(core.demo_readings())
    check(u"E1 轨迹 23 帧", trace.count() == 23, u"实得 %d" % (trace.count(),))
    check(u"E2 时长 2.20 s", close(trace.duration(), 2.20, 1e-6),
          u"实得 %r" % (trace.duration(),))
    check(u"E3 最高级别 4", trace.max_level() == 4,
          u"实得 %d" % (trace.max_level(),))
    check(u"E4 末帧级别 0", trace.final().level == 0,
          u"实得 %d" % (trace.final().level,))
    check(u"E5 切换 3 次", trace.switches() == 3,
          u"实得 %d" % (trace.switches(),))
    check(u"E6 振荡 0 次", trace.chatter() == 0,
          u"实得 %d" % (trace.chatter(),))
    check(u"E7 首次到 4 级在第 11 帧", trace.first_at(4) == 11,
          u"实得 %d" % (trace.first_at(4),))
    check(u"E8 首次到 1 级也在第 11 帧", trace.first_at(1) == 11,
          u"实得 %d" % (trace.first_at(1),))

    levels = trace.levels()
    check(u"E9 级别序列长度正确", len(levels) == 23)
    check(u"E10 前 11 帧都是 0 级", levels[:11] == [0] * 11,
          u"实得 %r" % (levels[:11],))
    check(u"E11 1.1~1.7 s 是 4 级", levels[11:18] == [4] * 7,
          u"实得 %r" % (levels[11:18],))

    timeline = trace.timeline()
    check(u"E12 时间线首项 t=0", close(timeline[0][0], 0.0))
    check(u"E13 时间线末项 t=2.2", close(timeline[-1][0], 2.2, 1e-6))
    check(u"E14 时间线级别数一致", len(timeline) == len(levels))

    used = trace.sensors_used()
    check(u"E15 触发过的通道在 0..3 之间", all(0 <= i <= 3 for i in used),
          u"实得 %r" % (used,))
    check(u"E16 空轨迹的 max_level 是 0", core.Trace([]).max_level() == 0)
    check(u"E17 空轨迹的 switches 是 0", core.Trace([]).switches() == 0)
    check(u"E18 空轨迹取末帧报错",
          raises(core.AlertError, core.Trace([]).final))
    check(u"E19 kept 统计 4 级的帧数", trace.kept(4) == 7,
          u"实得 %d" % (trace.kept(4),))


# ---------------------------------------------------------------- F 四条断言
def group_checks():
    readings = core.demo_readings()
    detector = core.Detector()
    trace = core.run_detector(readings, detector)
    results = core.check_alert(trace, readings, detector, 6)
    check(u"F1 四条断言齐全", len(results) == 4, u"实得 %d" % (len(results),))
    check(u"F2 断言名字与顺序", [item["type"] for item in results]
          == [u"分级准确", u"无抖动", u"响应及时", u"覆盖全部通道"],
          u"实得 %r" % ([item["type"] for item in results],))
    check(u"F3 默认全部通过", core.all_passed(results),
          u"失败项 %r" % ([i["type"] for i in results if not i["passed"]],))
    check(u"F4 分级准确的细节含最高级别",
          u"最高级别 4" in results[0]["detail"], results[0]["detail"])
    check(u"F5 响应及时的细节含延迟",
          u"延迟 2 帧" in results[2]["detail"], results[2]["detail"])

    loose = core.Detector(confirm_frames=1, release_frames=1)
    loose_trace = core.run_detector(readings, loose)
    loose_results = core.check_alert(loose_trace, readings, loose, 6)
    check(u"F6 关掉迟滞后无抖动断言失败",
          not loose_results[1]["passed"], loose_results[1]["detail"])
    check(u"F7 关掉迟滞后切换升到 8 次", loose_trace.switches() == 8,
          u"实得 %d" % (loose_trace.switches(),))

    slow = core.Detector(confirm_frames=5)
    slow_trace = core.run_detector(readings, slow)
    slow_results = core.check_alert(slow_trace, readings, slow, 6)
    check(u"F8 确认 5 帧时延迟变 4", u"延迟 4 帧" in slow_results[2]["detail"],
          slow_results[2]["detail"])
    check(u"F9 确认 5 帧时仍然通过", core.all_passed(slow_results),
          u"失败项 %r" % ([i["type"] for i in slow_results if not i["passed"]],))

    narrow = core.parse_readings_text(
        u"0.0 2.60 2.70 2.50 2.40\n0.2 1.10 2.70 2.50 2.40\n"
        u"0.3 1.05 2.70 2.50 2.40\n0.4 1.02 2.70 2.50 2.40\n"
        u"0.5 2.60 2.70 2.50 2.40\n", "只有一路近")
    narrow_detector = core.Detector()
    narrow_trace = core.run_detector(narrow, narrow_detector)
    narrow_results = core.check_alert(narrow_trace, narrow, narrow_detector, 6)
    check(u"F10 只有一路进区间时覆盖断言失败",
          not narrow_results[3]["passed"], narrow_results[3]["detail"])
    check(u"F11 覆盖断言的细节写明 1 个",
          u"有 1 个" in narrow_results[3]["detail"], narrow_results[3]["detail"])

    check(u"F12 all_passed 对空列表为真", core.all_passed([]) is True)


# ---------------------------------------------------------------- G 字符图
def group_render():
    trace = core.run_detector(core.demo_readings())
    rows = core.render_levels(trace)
    check(u"G1 级别条带 12 行", len(rows) == core.DEFAULT_GRID_HEIGHT,
          u"实得 %d" % (len(rows),))
    check(u"G2 每行 60 列", all(len(row) == core.DEFAULT_GRID_WIDTH for row in rows))
    check(u"G3 条带里有 # 号", any(u"#" in row for row in rows))
    check(u"G4 0 级行用 - 打底", any(u"-" in rows[-1] for _ in (0,)),
          u"末行 %r" % (rows[-1][:20],))

    empty_rows = core.render_levels(core.Trace([]))
    check(u"G5 空轨迹条带仍 12 行", len(empty_rows) == 12)
    check(u"G6 空轨迹条带全是空格",
          all(row.strip() == u"" for row in empty_rows))

    radar = core.render_radar([0.40, 0.90, 1.50, 2.80])
    check(u"G7 雷达条 4 行", len(radar) == 4, u"实得 %d" % (len(radar),))
    check(u"G8 每行都带通道名",
          all(core.DEFAULT_SENSOR_NAMES[i] in radar[i] for i in range(4)))
    check(u"G9 每行都带距离数值", all(u" m" in row for row in radar))
    check(u"G10 无回波时标 无回波",
          u"无回波" in core.render_radar([None, None, None, None])[0])


# ---------------------------------------------------------------- H 报告
def group_report():
    readings = core.demo_readings()
    detector = core.Detector()
    trace = core.run_detector(readings, detector)
    results = core.check_alert(trace, readings, detector, 6)
    report = core.format_report(trace, readings, detector, results)
    check(u"H1 报告含标题", u"倒车雷达分级告警" in report)
    check(u"H2 报告含进入阈值", u"进入阈值：2.00 / 1.50 / 1.00 / 0.50 m" in report)
    check(u"H3 报告含帧数与时长", u"帧数：23，时长：2.20 s" in report)
    check(u"H4 报告含最高级别", u"最高级别：4（停止）" in report)
    check(u"H5 报告含切换次数", u"级别切换：3 次" in report)
    check(u"H6 报告列出三次切换",
          report.count(u"t=") >= 3, u"t= 出现 %d 次" % (report.count(u"t="),))
    check(u"H7 报告含断言统计", u"断言：4 通过 / 0 失败（共 4 条）" in report)
    check(u"H8 报告含结论", u"结论：PASSED" in report)
    check(u"H9 报告含四条 PASS",
          report.count(u"[PASS]") == 4, u"实得 %d" % (report.count(u"[PASS]"),))

    headed = core.format_report(trace, readings, detector, results, u"前缀行")
    check(u"H10 可加表头行", headed.startswith(u"前缀行"))

    failed_report = core.format_report(
        trace, readings, detector,
        [{"type": u"示例", "passed": False, "detail": u"故意失败"}])
    check(u"H11 失败时结论是 FAILED", u"结论：FAILED" in failed_report)
    check(u"H12 失败项带 [FAIL] 前缀", u"[FAIL] 示例" in failed_report)


# ---------------------------------------------------------------- I 参数读取
def group_params():
    params = core.AlertParams()
    check(u"I1 默认确认帧数 3", params.alert_confirm_frames == 3,
          u"实得 %r" % (params.alert_confirm_frames,))
    check(u"I2 默认释放帧数 3", params.alert_release_frames == 3)
    check(u"I3 默认读数节拍 10", close(params.rate_read_hz, 10.0))
    check(u"I4 默认级别话题", params.topic_level == u"/rear_alert/level",
          u"实得 %r" % (params.topic_level,))
    check(u"I5 默认有四个话题键",
          all(hasattr(params, name) for name in
              ("topic_level", "topic_report", "topic_levels", "topic_radar")))
    check(u"I6 参数个数与 PARAM_KEYS 一致",
          len(params.as_dict()) == len(core.PARAM_KEYS),
          u"实得 %d / %d" % (len(params.as_dict()), len(core.PARAM_KEYS)))
    check(u"I7 构造出的判定器阈值正确",
          params.detector.enter == (2.0, 1.5, 1.0, 0.5),
          u"实得 %r" % (params.detector.enter,))

    values = {}

    def get_param(name, default=None):
        values[name] = True
        return u"5" if name == "alert/confirm_frames" else default

    loaded = core.AlertParams.from_params(get_param)
    check(u"I8 from_params 读全部键", len(values) == len(core.PARAM_KEYS),
          u"实得 %d" % (len(values),))
    check(u"I9 字符串帧数被转成整数", loaded.alert_confirm_frames == 5,
          u"实得 %r" % (loaded.alert_confirm_frames,))

    custom = core.AlertParams(**{"alert/max_switches": 2})
    check(u"I10 完整键名可覆盖默认值", custom.max_switches == 2,
          u"实得 %r" % (custom.max_switches,))
    check(u"I11 非法阈值顺序报错",
          raises(core.AlertError, core.AlertParams,
                 **{"alert/enter_info_m": 1.0, "alert/enter_warn_m": 2.0}))


# ---------------------------------------------------------------- J 端到端
def group_end_to_end():
    readings = core.demo_readings()
    detector = core.Detector()
    trace = core.run_detector(readings, detector, u"默认")
    results = core.check_alert(trace, readings, detector, 6)
    check(u"J1 默认场景 4 条全过", sum(1 for i in results if i["passed"]) == 4,
          u"实得 %d" % (sum(1 for i in results if i["passed"]),))
    check(u"J2 默认场景切换 3 次", trace.switches() == 3)
    check(u"J3 默认场景最高 4 级", trace.max_level() == 4)
    check(u"J4 默认场景末帧 0 级", trace.final().level == 0)
    changes = [(item.t, item.level) for item in trace.grades if item.changed]
    check(u"J5 三次切换分别在 1.1/1.8/2.1 s",
          close(changes[0][0], 1.1, 1e-6) and close(changes[1][0], 1.8, 1e-6)
          and close(changes[2][0], 2.1, 1e-6),
          u"实得 %r" % (changes,))
    check(u"J6 切换的级别是 4/1/0",
          [item[1] for item in changes] == [4, 1, 0],
          u"实得 %r" % ([item[1] for item in changes],))

    no_margin = core.Detector(exit_margin=0.0)
    trace2 = core.run_detector(readings, no_margin, u"无余量")
    check(u"J7 无余量时末帧仍是 0 级", trace2.final().level == 0)
    check(u"J8 无余量时切换仍是 3 次", trace2.switches() == 3,
          u"实得 %d" % (trace2.switches(),))
    check(u"J9 无余量时 1.7 s 降到 2 级",
          trace2.grades[17].level == 2, u"实得 %d" % (trace2.grades[17].level,))

    switching = core.parse_readings_text(
        u"0.0 2.20 2.30 2.25 2.35\n0.1 1.20 1.30 1.25 1.35\n"
        u"0.2 2.20 2.30 2.25 2.35\n0.3 1.20 1.30 1.25 1.35\n"
        u"0.4 2.20 2.30 2.25 2.35\n0.5 1.20 1.30 1.25 1.35\n"
        u"0.6 0.60 0.70 0.65 0.75\n0.7 0.60 0.70 0.65 0.75\n"
        u"0.8 0.60 0.70 0.65 0.75\n0.9 0.60 0.70 0.65 0.75\n", "反复进出")
    switching_detector = core.Detector()
    switching_trace = core.run_detector(switching, switching_detector)
    check(u"J10 反复进出时只切换 1 次", switching_trace.switches() == 1,
          u"实得 %d" % (switching_trace.switches(),))
    check(u"J11 反复进出时最高 3 级", switching_trace.max_level() == 3,
          u"实得 %d" % (switching_trace.max_level(),))
    switching_results = core.check_alert(switching_trace, switching,
                                         switching_detector, 6)
    check(u"J12 反复进出时四条断言全过",
          sum(1 for i in switching_results if i["passed"]) == 4,
          u"失败项 %r" % ([i["type"] for i in switching_results
                          if not i["passed"]],))

    one_side = core.parse_readings_text(
        u"0.0 2.60 2.70 2.50 2.40\n0.2 1.10 2.70 2.50 2.40\n"
        u"0.3 1.05 2.70 2.50 2.40\n0.4 1.02 2.70 2.50 2.40\n"
        u"0.5 2.60 2.70 2.50 2.40\n", "单侧")
    one_detector = core.Detector()
    one_trace = core.run_detector(one_side, one_detector)
    check(u"J13 单侧场景最高 2 级", one_trace.max_level() == 2,
          u"实得 %d" % (one_trace.max_level(),))
    check(u"J14 单侧场景切换 1 次", one_trace.switches() == 1,
          u"实得 %d" % (one_trace.switches(),))


# ---------------------------------------------------------------- K 工程文件
def group_project_files():
    package_path = os.path.join(PKG, "package.xml")
    cmake_path = os.path.join(PKG, "CMakeLists.txt")
    launch_path = os.path.join(PKG, "launch", "main.launch")
    config_path = os.path.join(PKG, "config", "ultrasonic_rear_alert.yaml")
    readme_path = os.path.join(PKG, "README.md")
    core_path = os.path.join(SCRIPTS, "alert_core.py")
    node_path = os.path.join(SCRIPTS, "alert_node.py")
    main_path = os.path.join(SCRIPTS, "main.py")

    for name, path in (("K1 package.xml", package_path),
                       ("K2 CMakeLists.txt", cmake_path),
                       ("K3 main.launch", launch_path),
                       ("K4 参数文件", config_path),
                       ("K5 README", readme_path),
                       ("K6 alert_core.py", core_path),
                       ("K7 alert_node.py", node_path),
                       ("K8 main.py", main_path),
                       ("K9 文档页", DOCS_PAGE)):
        check(name, os.path.isfile(path), path)

    if not os.path.isfile(package_path):
        return
    package = read_text(package_path)
    check(u"K10 package 名字正确", u"<name>ultrasonic_rear_alert</name>" in package)
    check(u"K11 package 依赖 rospy", u"<exec_depend>rospy</exec_depend>" in package)
    check(u"K12 package 依赖 std_msgs",
          u"<exec_depend>std_msgs</exec_depend>" in package)

    cmake = read_text(cmake_path)
    check(u"K13 CMake 安装三个脚本",
          cmake.count(u"scripts/") == 3, u"实得 %d" % (cmake.count(u"scripts/"),))
    check(u"K14 CMake 安装 launch 目录", u"launch" in cmake)
    check(u"K15 CMake 安装 config 目录", u"config" in cmake)

    launch = read_text(launch_path)
    check(u"K16 launch 用 main.py 作为入口", u'type="main.py"' in launch)
    check(u"K17 launch 加载参数文件",
          u'<rosparam command="load"' in launch)
    check(u"K18 launch 有 17 个 arg", launch.count(u"<arg ") == 17,
          u"实得 %d" % (launch.count(u"<arg "),))
    check(u"K19 每个 arg 都带 doc", launch.count(u'doc="') == 17,
          u"实得 %d" % (launch.count(u'doc="'),))
    check(u"K20 launch 转发 17 个 param", launch.count(u"<param ") == 17,
          u"实得 %d" % (launch.count(u"<param "),))

    config = read_text(config_path)
    for key in ("enter_info_m", "enter_warn_m", "enter_danger_m",
                "enter_stop_m", "exit_margin_m", "confirm_frames",
                "release_frames", "min_range_m", "max_range_m",
                "max_switches", "read_hz", "publish_hz",
                "level", "report", "levels", "radar", "simulate"):
        check(u"K21 参数文件含 %s" % (key,), key in config)

    core_source = read_text(core_path)
    check(u"K22 核心模块不 import rospy",
          u"\nimport rospy" not in core_source, u"核心模块里出现了 import rospy")
    check(u"K23 核心模块不 import yaml", u"\nimport yaml" not in core_source)
    check(u"K24 核心模块不 import numpy", u"\nimport numpy" not in core_source)
    check(u"K25 核心模块声明四条断言",
          u'ASSERT_NAMES = (u"分级准确", u"无抖动", u"响应及时", u"覆盖全部通道")'
          in core_source)

    node_source = read_text(node_path)
    check(u"K26 节点 import 核心模块", u"import alert_core" in node_source)
    check(u"K27 节点日志走 _log", u"def _log(" in node_source)
    check(u"K28 节点发布级别话题", u"Int32(" in node_source)
    check(u"K29 节点注册 on_shutdown", u"on_shutdown" in node_source)

    main_source = read_text(main_path)
    check(u"K30 入口复用节点逻辑", u"from alert_node import main" in main_source)
    check(u"K31 入口有 __main__ 保护", u'if __name__ == "__main__":' in main_source)

    readme = read_text(readme_path)
    check(u"K32 README 含 roslaunch 命令",
          u"roslaunch ultrasonic_rear_alert main.launch" in readme)
    check(u"K33 README 含离线测试命令",
          u"test_ultrasonic_rear_alert_local.py" in readme)

    if os.path.isfile(DOCS_PAGE):
        page = read_text(DOCS_PAGE)
        check(u"K34 文档页标题正确", page.startswith(u"# 倒车雷达分级告警"),
              page[:20])
        check(u"K35 文档页含四条断言", u"分级准确" in page and u"无抖动" in page
              and u"响应及时" in page and u"覆盖全部通道" in page)
        check(u"K36 文档页含真实输出", u"最高级别：4（停止）" in page)
        check(u"K37 文档页含大模型声明", u"大模型" in page)
    else:
        check(u"K34 本次未在仓库根目录运行，跳过文档页校验", True)
        check(u"K35 跳过文档页断言检查", True)
        check(u"K36 跳过文档页输出检查", True)
        check(u"K37 跳过文档页声明检查", True)


# ---------------------------------------------------------------- 主流程
def group_service():
    """L 服务形态：把「重算一次」暴露成 std_srvs/Trigger。"""
    check(u"L1 服务默认名", core.DEFAULT_SERVICE_SIMULATE == u"/rear_alert/simulate",
          u"%r" % (core.DEFAULT_SERVICE_SIMULATE,))
    check(u"L2 服务名进参数表", u"service/simulate" in core.PARAM_KEYS,
          u"%d 个键" % (len(core.PARAM_KEYS),))
    check(u"L3 参数对象暴露服务名",
          core.AlertParams().service_simulate == core.DEFAULT_SERVICE_SIMULATE,
          u"%r" % (core.AlertParams().service_simulate,))

    node_source = read_text(os.path.join(SCRIPTS, "alert_node.py"))
    check(u"L4 节点注册 rospy.Service", u"rospy.Service(" in node_source)
    check(u"L5 节点用 Trigger 类型", u"std_srvs.srv" in node_source
          and u"Trigger" in node_source)
    check(u"L6 节点返回 TriggerResponse",
          u"TriggerResponse(" in node_source and u"success=" in node_source)

    package = read_text(os.path.join(PKG, "package.xml"))
    check(u"L7 package 依赖 std_srvs",
          u"<exec_depend>std_srvs</exec_depend>" in package)

    launch = read_text(os.path.join(PKG, "launch", "main.launch"))
    check(u"L8 launch 暴露服务名参数", u'name="simulate_service"' in launch)
    check(u"L9 launch 转发 service/simulate", u'name="service/simulate"' in launch)


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


test_thresholds = _pytest_group(group_thresholds)
test_reading = _pytest_group(group_reading)
test_detector = _pytest_group(group_detector)
test_hysteresis = _pytest_group(group_hysteresis)
test_trace = _pytest_group(group_trace)
test_checks = _pytest_group(group_checks)
test_render = _pytest_group(group_render)
test_report = _pytest_group(group_report)
test_params = _pytest_group(group_params)
test_end_to_end = _pytest_group(group_end_to_end)
test_project_files = _pytest_group(group_project_files)
test_service = _pytest_group(group_service)


def main():
    print(u"ultrasonic_rear_alert 离线回归测试")
    print(u"Python %s" % (sys.version.split()[0],))
    print(u"包目录 %s" % (PKG,))
    print(u"-" * 60)
    group_thresholds()
    group_reading()
    group_detector()
    group_hysteresis()
    group_trace()
    group_checks()
    group_render()
    group_report()
    group_params()
    group_end_to_end()
    group_project_files()
    group_service()
    print(u"=" * 60)
    for item in FAILED:
        print(u"[失败] %s" % (item,))
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
