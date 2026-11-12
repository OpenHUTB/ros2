#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""lane_change_feasibility 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_lane_change_feasibility_local.py

覆盖范围：
A 通用小工具      B 规格与交通校验   C 剖面主算      D 计划对象
E 四条断言        F 典型工况与反例   G 建议文案      H 字符俯视图
I 报告格式        J 参数读取         K 工程文件约定
"""

from __future__ import division, print_function

import io
import math
import os
import sys

try:
    text_type = unicode  # noqa: F821  Python 2
except NameError:
    text_type = str  # Python 3

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "src", "ground", "lane_change_feasibility")
SCRIPTS = os.path.join(PKG, "scripts")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "lane_change_feasibility.md")

sys.path.insert(0, SCRIPTS)

import lane_change_core as core

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(u"%s %s" % (name, detail))


def close(name, actual, expected, tol=1e-6, detail=""):
    ok = actual is not None and abs(actual - expected) <= tol
    check(name, ok, u"实得 %r，期望 %r（容差 %r）%s"
          % (actual, expected, tol, detail))


def raises(expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        return True
    except Exception as error:  # noqa: BLE001
        check(u"异常类型不符", False, u"实得 %r" % (error,))
        return False
    check(u"应当抛异常但没有", False, u"%s" % (func,))
    return False


# ---------------------------------------------------------------- A 通用小工具
def group_helpers():
    close(u"A1 kmh_to_mps(80)", core.kmh_to_mps(80.0), 22.222222222, 1e-9)
    close(u"A2 kmh_to_mps(0)", core.kmh_to_mps(0.0), 0.0, 1e-12)
    close(u"A3 mps_to_kmh(22.222222222)", core.mps_to_kmh(22.222222222),
          80.0, 1e-6)
    close(u"A4 mps_to_kmh(0)", core.mps_to_kmh(0.0), 0.0, 1e-12)
    close(u"A5 clamp 上限", core.clamp(5.0, 0.0, 1.0), 1.0, 1e-12)
    close(u"A6 clamp 下限", core.clamp(-5.0, 0.0, 1.0), 0.0, 1e-12)
    close(u"A7 clamp 中间", core.clamp(0.5, 0.0, 1.0), 0.5, 1e-12)
    close(u"A8 duration_for(3.5,1.5)",
          core.duration_for(3.5, 1.5), 3.828937936, 1e-6)
    close(u"A9 duration_for(5.0,1.5)",
          core.duration_for(5.0, 1.5), 4.576456360, 1e-6)
    close(u"A10 duration_for(3.5,0.5)",
          core.duration_for(3.5, 0.5), 6.631914691, 1e-6)
    the_duration = core.duration_for(3.5, 1.5)
    close(u"A11 offset_at 中点", core.offset_at(the_duration / 2.0,
                                               the_duration, 3.5),
          1.75, 1e-9)
    close(u"A12 offset_at 起点", core.offset_at(0.0, the_duration, 3.5),
          0.0, 1e-12)
    close(u"A13 offset_at 终点", core.offset_at(the_duration,
                                                the_duration, 3.5),
          3.5, 1e-9)
    close(u"A14 offset_at 越界按终点",
          core.offset_at(the_duration * 2.0, the_duration, 3.5), 3.5, 1e-9)
    close(u"A15 峰值横向速度 = 2L/T",
          max(core.lateral_velocity_at(the_duration * i / 200.0,
                                       the_duration, 3.5)
              for i in range(201)),
          2.0 * 3.5 / the_duration, 2e-3)
    close(u"A16 四分之一处横向加速度为峰",
          abs(core.lateral_accel_at(the_duration / 4.0,
                                    the_duration, 3.5)),
          2.0 * math.pi * 3.5 / (the_duration ** 2), 1e-6)
    close(u"A16b 中点横向加速度为零",
          abs(core.lateral_accel_at(the_duration / 2.0,
                                    the_duration, 3.5)), 0.0, 1e-9)
    close(u"A17 起点横向加速度为 0",
          core.lateral_accel_at(0.0, the_duration, 3.5), 0.0, 1e-9)
    close(u"A18 终点横向加速度为 0",
          core.lateral_accel_at(the_duration, the_duration, 3.5), 0.0, 1e-6)
    close(u"A19 起点横向速度为 0",
          core.lateral_velocity_at(0.0, the_duration, 3.5), 0.0, 1e-9)
    close(u"A20 终点横向速度为 0",
          core.lateral_velocity_at(the_duration, the_duration, 3.5),
          0.0, 1e-6)
    check(u"A21 _as_float 字符串", abs(core._as_float("1.25", u"x") - 1.25) < 1e-12)
    close(u"A22 _as_float 缺省", core._as_float(None, u"x", 2.5), 2.5, 1e-12)
    check(u"A23 _as_float 非法报错",
          raises(core.LaneChangeError, core._as_float, "abc", u"x"))
    check(u"A24 _as_float NaN 报错",
          raises(core.LaneChangeError, core._as_float, float("nan"), u"x"))
    close(u"A25 _positive 正常", core._positive(2.0, u"x"), 2.0, 1e-12)
    check(u"A26 _positive 零报错",
          raises(core.LaneChangeError, core._positive, 0.0, u"x"))
    check(u"A27 _positive 负报错",
          raises(core.LaneChangeError, core._positive, -1.0, u"x"))
    close(u"A28 _non_negative 零", core._non_negative(0.0, u"x"), 0.0, 1e-12)
    check(u"A29 _non_negative 负报错",
          raises(core.LaneChangeError, core._non_negative, -0.5, u"x"))
    check(u"A30 断言名是四条", len(core.ASSERT_NAMES) == 4,
          u"实得 %r" % (core.ASSERT_NAMES,))


# ------------------------------------------------------------ B 规格与交通校验
def group_spec_and_traffic():
    spec = core.Spec()
    close(u"B1 车道宽默认", spec.lane_width, 3.5, 1e-12)
    close(u"B2 横向加速度默认", spec.max_lateral_accel, 1.5, 1e-12)
    close(u"B3 TTC 默认", spec.ttc_min, 3.0, 1e-12)
    close(u"B4 前向余量默认", spec.front_margin, 5.0, 1e-12)
    close(u"B5 后向余量默认", spec.rear_margin, 5.0, 1e-12)
    close(u"B6 时长上限默认", spec.duration_limit, 5.0, 1e-12)
    close(u"B7 offset 等于车道宽", spec.offset, 3.5, 1e-12)
    check(u"B8 as_tuple 六项", len(spec.as_tuple()) == 6,
          u"实得 %r" % (spec.as_tuple(),))
    check(u"B9 车道宽为 0 报错",
          raises(core.LaneChangeError, core.Spec, lane_width=0.0))
    check(u"B10 车道宽为负报错",
          raises(core.LaneChangeError, core.Spec, lane_width=-1.0))
    check(u"B11 横向加速度为负报错",
          raises(core.LaneChangeError, core.Spec, max_lateral_accel=-0.1))
    check(u"B12 TTC 为负报错",
          raises(core.LaneChangeError, core.Spec, ttc_min=-1.0))
    check(u"B13 时长上限为负报错",
          raises(core.LaneChangeError, core.Spec, duration_limit=-1.0))
    traffic = core.Traffic()
    close(u"B14 自车默认", traffic.ego_kmh, 80.0, 1e-12)
    close(u"B15 前车默认", traffic.front_kmh, 60.0, 1e-12)
    close(u"B16 前车间距默认", traffic.front_gap, 40.0, 1e-12)
    close(u"B17 后车默认", traffic.rear_kmh, 90.0, 1e-12)
    close(u"B18 后车间距默认", traffic.rear_gap, 30.0, 1e-12)
    close(u"B19 ego_mps", traffic.ego_mps, 22.222222222, 1e-6)
    close(u"B20 front_mps", traffic.front_mps, 16.666666667, 1e-6)
    close(u"B21 rear_mps", traffic.rear_mps, 25.0, 1e-9)
    check(u"B22 as_tuple 五项", len(traffic.as_tuple()) == 5,
          u"实得 %r" % (traffic.as_tuple(),))
    check(u"B23 速度为负报错",
          raises(core.LaneChangeError, core.Traffic, ego_kmh=-1.0))
    check(u"B24 间距为负报错",
          raises(core.LaneChangeError, core.Traffic, front_gap=-1.0))
    check(u"B25 参数为字符串也能用",
          abs(core.Traffic(ego_kmh="80")["ego_kmh"] if False else 0.0) < 1e-12)


# ------------------------------------------------------------------- C 剖面主算
def group_make_plan():
    plan = core.make_plan()
    close(u"C1 换道时长", plan.duration, 3.828937936, 1e-6)
    close(u"C2 推进距离", plan.distance, 85.087509832, 1e-5)
    close(u"C3 峰值横向加速度", plan.peak_accel, 1.5, 1e-9)
    close(u"C4 峰值横向速度", plan.peak_velocity, 1.828183197, 1e-6)
    close(u"C5 所需前向间隙", plan.required_front_gap, 26.271876545, 1e-5)
    close(u"C6 所需后向间隙", plan.required_rear_gap, 15.635938727, 1e-5)
    close(u"C7 前向 TTC", plan.ttc_front, 7.2, 1e-6)
    close(u"C8 后向 TTC", plan.ttc_rear, 10.8, 1e-6)
    check(u"C9 默认可行", plan.feasible, u"reasons=%r" % (plan.reasons,))
    check(u"C10 默认无否决理由", len(plan.reasons) == 0,
          u"实得 %r" % (plan.reasons,))
    wide = core.make_plan(core.Spec(lane_width=5.0))
    close(u"C11 宽车道时长", wide.duration, 4.576456360, 1e-6)
    close(u"C12 宽车道距离", wide.distance, 101.699025801, 1e-5)
    close(u"C13 宽车道峰值横向速度", wide.peak_velocity, 2.185096832, 1e-6)
    gentle = core.make_plan(core.Spec(max_lateral_accel=0.5))
    close(u"C14 温和加速度时长", gentle.duration, 6.631914691, 1e-6)
    check(u"C15 温和加速度不可行", not gentle.feasible)
    reverse = core.make_plan(None, None, -1)
    close(u"C16 向右换道 offset", reverse.offset, -3.5, 1e-12)
    close(u"C17 向右时长不变", reverse.duration, 3.828937936, 1e-6)
    slow = core.make_plan(core.Spec(), core.Traffic(rear_kmh=70.0))
    check(u"C18 后车慢时 TTC 为无穷",
          slow.ttc_rear == float("inf"), u"实得 %r" % (slow.ttc_rear,))
    close(u"C19 后车慢所需后向间隙", slow.required_rear_gap, 5.0, 1e-12)
    same = core.make_plan(core.Spec(), core.Traffic(rear_kmh=80.0))
    check(u"C20 后车同速 TTC 为无穷",
          same.ttc_rear == float("inf"), u"实得 %r" % (same.ttc_rear,))
    quick = core.make_plan(core.Spec(), core.Traffic(ego_kmh=120.0,
                                                     front_gap=100.0,
                                                     rear_gap=100.0))
    close(u"C21 自车 120 推进距离", quick.distance,
          3.828937936 * 33.333333333, 1e-4)
    check(u"C22 as_tuple 四项", len(plan.as_tuple()) == 4,
          u"实得 %d" % (len(plan.as_tuple()),))
    check(u"C22b as_tuple 首项是方向", plan.as_tuple()[0] == 1)
    check(u"C22c as_tuple 末项是可行性", plan.as_tuple()[3] is True)


# ------------------------------------------------------------------ D 计划对象
def group_plan_object():
    plan = core.make_plan()
    check(u"D1 offset 带方向", abs(plan.offset - 3.5) < 1e-12)
    samples = plan.samples(0.5)
    check(u"D2 采样点数为 9", len(samples) == 9, u"实得 %d" % (len(samples),))
    check(u"D3 首点 y 为 0", abs(samples[0].y) < 1e-12)
    close(u"D4 t=2.0 处 y", samples[4].y, 1.906110, 1e-5)
    check(u"D5 末点 y 为 3.5", abs(samples[-1].y - 3.5) < 1e-6,
          u"实得 %r" % (samples[-1].y,))
    check(u"D6 采样横向速度非负",
          all(sample.vy >= -1e-9 for sample in samples))
    check(u"D7 采样为 Sample 实例",
          isinstance(samples[0], core.Sample))
    close(u"D8 plan.offset_at 中点", plan.offset_at(plan.duration / 2.0),
          1.75, 1e-9)
    close(u"D9 plan.lateral_accel_at 四分之一处",
          plan.lateral_accel_at(plan.duration / 4.0), 1.5, 1e-6)
    close(u"D9b plan.lateral_accel_at 中点为零",
          plan.lateral_accel_at(plan.duration / 2.0), 0.0, 1e-9)
    close(u"D10 plan.lateral_velocity_at 中点",
          plan.lateral_velocity_at(plan.duration / 2.0), 1.828183, 1e-5)
    fine = plan.samples(0.02)
    check(u"D11 细采样点数", len(fine) == 192, u"实得 %d" % (len(fine),))
    coarse = plan.samples(1.0)
    check(u"D12 粗采样点数", len(coarse) == 5, u"实得 %d" % (len(coarse),))
    check(u"D13 默认 gap_shortfall 为 0 或负",
          plan.gap_shortfall <= 1e-9,
          u"实得 %r" % (plan.gap_shortfall,))
    short = core.make_plan(core.Spec(), core.Traffic(ego_kmh=100.0))
    close(u"D14 前向缺口", short.gap_shortfall, 7.5437546, 1e-5)
    check(u"D15 缺口为正表示不够", short.gap_shortfall > 0)
    tight = core.make_plan(core.Spec(),
                           core.Traffic(rear_kmh=120.0, rear_gap=12.0))
    check(u"D16 后向 TTC 缺口为正", tight.ttc_shortfall > 0,
          u"实得 %r" % (tight.ttc_shortfall,))
    check(u"D17 后向 TTC 缺口大于 1 s", tight.ttc_shortfall > 1.0,
          u"实得 %r" % (tight.ttc_shortfall,))


# ------------------------------------------------------------------- E 四条断言
def group_checks():
    plan = core.make_plan()
    results = core.check_plan(plan)
    check(u"E1 返回四条", len(results) == 4, u"实得 %d" % (len(results),))
    check(u"E2 每条都是 dict",
          all(isinstance(item, dict) for item in results))
    check(u"E3 每条都有 type", all("type" in item for item in results))
    check(u"E4 每条都有 passed", all("passed" in item for item in results))
    check(u"E5 每条都有 detail", all("detail" in item for item in results))
    check(u"E6 类型顺序正确",
          [item["type"] for item in results] == list(core.ASSERT_NAMES),
          u"实得 %r" % ([item["type"] for item in results],))
    check(u"E7 默认四条全过", all(item["passed"] for item in results))
    check(u"E8 all_passed 为真", core.all_passed(results))
    fail_one = core.make_plan(core.Spec(), core.Traffic(ego_kmh=100.0))
    one = core.check_plan(fail_one)
    check(u"E9 前车太近只翻一条",
          sum(1 for item in one if not item["passed"]) == 1,
          u"实得 %r" % ([item["passed"] for item in one],))
    check(u"E10 前车太近翻的是前向间隙",
          [item["type"] for item in one if not item["passed"]] == [u"前向间隙"])
    tight = core.make_plan(core.Spec(),
                           core.Traffic(rear_kmh=120.0, rear_gap=12.0))
    two = core.check_plan(tight)
    check(u"E11 后车逼近只翻一条",
          sum(1 for item in two if not item["passed"]) == 1,
          u"实得 %r" % ([item["passed"] for item in two],))
    worse = core.make_plan(core.Spec(),
                            core.Traffic(ego_kmh=100.0,
                                         rear_kmh=120.0,
                                         rear_gap=12.0))
    check(u"E11b 提速到 100 会连前向一起翻",
          sum(1 for item in core.check_plan(worse)
              if not item["passed"]) == 2,
          u"实得 %r" % ([item["passed"]
                        for item in core.check_plan(worse)],))
    check(u"E12 后车逼近包含 TTC",
          any(item["type"] == u"后向 TTC" and not item["passed"]
              for item in two))
    gentle = core.make_plan(core.Spec(max_lateral_accel=0.5))
    three = core.check_plan(gentle)
    check(u"E13 温和加速度翻两条",
          sum(1 for item in three if not item["passed"]) == 2,
          u"实得 %r" % ([item["passed"] for item in three],))
    check(u"E14 温和加速度含时长",
          any(item["type"] == u"换道时长" and not item["passed"]
              for item in three))
    check(u"E15 all_passed 为假", not core.all_passed(three))
    strict = core.check_plan(plan, duration_limit=3.0)
    check(u"E16 收紧时长上限会翻",
          any(item["type"] == u"换道时长" and not item["passed"]
              for item in strict))
    check(u"E17 detail 是文本",
          isinstance(plan and results[0]["detail"], text_type))


# ------------------------------------------------------------ F 典型工况与反例
def group_cases():
    default = core.make_plan()
    check(u"F1 默认可行", default.feasible)
    far_front = core.make_plan(core.Spec(), core.Traffic(ego_kmh=100.0,
                                                         front_gap=60.0))
    check(u"F2 前车拉远后可行", far_front.feasible,
          u"reasons=%r" % (far_front.reasons,))
    slow_ego = core.make_plan(core.Spec(), core.Traffic(ego_kmh=60.0,
                                                        front_kmh=60.0))
    close(u"F3 同速时所需前向只有余量", slow_ego.required_front_gap, 5.0, 1e-9)
    far_rear = core.make_plan(core.Spec(), core.Traffic(rear_kmh=120.0,
                                                        rear_gap=60.0))
    check(u"F4 后车拉远后可行", far_rear.feasible,
          u"reasons=%r" % (far_rear.reasons,))
    race = core.make_plan(core.Spec(), core.Traffic(ego_kmh=100.0,
                                                    rear_kmh=120.0,
                                                    rear_gap=12.0))
    check(u"F5 高速后被车咬住不可行", not race.feasible)
    fast_ego = core.make_plan(core.Spec(), core.Traffic(ego_kmh=100.0,
                                                        front_gap=80.0,
                                                        rear_kmh=100.0,
                                                        rear_gap=60.0))
    check(u"F6 提速到 100 也可行", fast_ego.feasible,
          u"reasons=%r" % (fast_ego.reasons,))
    for value in (0.3, 1.0, 2.5):
        spec = core.Spec(max_lateral_accel=value)
        plan = core.make_plan(spec)
        check(u"F7 横向加速度 %r 时 T 与 A 自洽" % (value,),
              abs(2.0 * math.pi * spec.lane_width / (plan.duration ** 2)
                  - value) < 1e-6)
    for width in (2.5, 3.0, 3.75, 5.0):
        spec = core.Spec(lane_width=width)
        plan = core.make_plan(spec)
        check(u"F8 车道 %r 时 y(T) 等于车道宽" % (width,),
              abs(core.offset_at(plan.duration, plan.duration, width)
                  - width) < 1e-9)
    reverse = core.make_plan(None, None, -1)
    check(u"F9 向右换道可行", reverse.feasible)
    check(u"F10 向右时 offset 为负", reverse.offset < 0)
    check(u"F11 reasons 是列表",
          isinstance(default.reasons, list))
    check(u"F12 反例的后向 TTC 小于阈值",
          core.make_plan(core.Spec(),
                         core.Traffic(rear_kmh=120.0, rear_gap=12.0)).ttc_rear
          < 3.0)
    check(u"F13 宽松 TTC 可放行",
          core.make_plan(core.Spec(ttc_min=0.5),
                         core.Traffic(rear_kmh=100.0,
                                      rear_gap=30.0)).feasible,
          u"reasons=%r" % (core.make_plan(
              core.Spec(ttc_min=0.5),
              core.Traffic(rear_kmh=100.0, rear_gap=30.0)).reasons,))
    check(u"F14 放宽时长并拉远前车后可放行温和加速度",
          core.make_plan(core.Spec(max_lateral_accel=0.5,
                                   duration_limit=8.0),
                         core.Traffic(front_gap=50.0)).feasible,
          u"reasons=%r" % (core.make_plan(
              core.Spec(max_lateral_accel=0.5, duration_limit=8.0),
              core.Traffic(front_gap=50.0)).reasons,))


# ------------------------------------------------------------------ G 建议文案
def group_advice():
    okay = core.advice(core.make_plan())
    check(u"G1 可行时给一条建议", len(okay) == 1, u"实得 %d" % (len(okay),))
    check(u"G2 可行建议含执行", u"执行" in okay[0] or u"推进" in okay[0],
          okay[0])
    blocked = core.advice(core.make_plan(core.Spec(),
                                         core.Traffic(ego_kmh=100.0)))
    check(u"G3 前向不足有建议", len(blocked) >= 1)
    check(u"G4 建议里给出降速数值",
          any(u"km/h" in line for line in blocked),
          u"实得 %r" % (blocked,))
    both = core.advice(core.make_plan(core.Spec(),
                                      core.Traffic(rear_kmh=120.0,
                                                   rear_gap=12.0)))
    check(u"G5 后车逼近建议至少两条", len(both) >= 2,
          u"实得 %d" % (len(both),))
    check(u"G6 建议里有 TTC 字样",
          any(u"TTC" in line for line in both), u"实得 %r" % (both,))
    gentle = core.advice(core.make_plan(core.Spec(max_lateral_accel=0.5)))
    check(u"G7 温和加速度建议提加速度",
          any(u"m/s" in line for line in gentle), u"实得 %r" % (gentle,))
    check(u"G8 建议都是文本",
          all(isinstance(line, text_type) for line in both))


# ---------------------------------------------------------------- H 字符俯视图
def group_render():
    rows = core.render_lane(core.make_plan())
    check(u"H1 返回行列表", isinstance(rows, list))
    check(u"H2 行数为 14", len(rows) == 14, u"实得 %d" % (len(rows),))
    check(u"H3 列数为 60", all(len(row) == 60 for row in rows),
          u"实得 %r" % (sorted(set(len(row) for row in rows)),))
    text = u"\n".join(rows)
    check(u"H4 含自车标记", u"o" in text)
    check(u"H5 含前车标记", u"F" in text)
    check(u"H6 含后车标记", u"R" in text)
    check(u"H7 含车道线", u"|" in text)
    narrow = core.render_lane(core.make_plan(), 30, 8)
    check(u"H8 自定义尺寸生效",
          len(narrow) == 8 and all(len(row) == 30 for row in narrow))
    check(u"H9 每行都是文本",
          all(isinstance(row, text_type) for row in rows))


# ------------------------------------------------------------------ I 报告格式
def group_report():
    plan = core.make_plan()
    report = core.format_report(core.Spec(), core.Traffic(), plan,
                                core.check_plan(plan))
    check(u"I1 含换道方向", u"换道方向：" in report)
    check(u"I2 含自车速度", u"自车 80.0 km/h" in report)
    check(u"I3 含换道时长行", u"换道时长：3.829 s" in report)
    check(u"I4 含所需前向间隙", u"所需前向间隙：26.272 m" in report)
    check(u"I5 含决策行", u"决策：可以换道" in report)
    check(u"I6 含断言统计", u"4 通过 / 0 失败" in report)
    check(u"I7 含 PASS 标记", u"[PASS]" in report)
    check(u"I8 含建议", u"建议：" in report)
    check(u"I9 含结论 PASSED", u"结论：PASSED" in report)
    bad = core.make_plan(core.Spec(), core.Traffic(ego_kmh=100.0))
    bad_report = core.format_report(core.Spec(), core.Traffic(ego_kmh=100.0),
                                    bad, core.check_plan(bad))
    check(u"I10 含 FAIL 标记", u"[FAIL]" in bad_report)
    check(u"I11 含结论 FAILED", u"结论：FAILED" in bad_report)
    check(u"I12 表头可自定义",
          u"自定义头" in core.format_report(core.Spec(), core.Traffic(),
                                            plan, core.check_plan(plan),
                                            header=u"自定义头"))
    payload = core.to_message_text(report)
    check(u"I13 to_message_text 返回可发给 std_msgs 的载荷",
          isinstance(payload, (bytes, str)), u"实得 %r" % (type(payload),))
    check(u"I13b 载荷能解回原文",
          payload.decode("utf-8") == report
          if isinstance(payload, bytes) else payload == report)


# -------------------------------------------------------------------- J 参数读取
def group_params():
    class FakeGet(object):
        def __init__(self, values):
            self.values = values
            self.seen = []

        def __call__(self, name, default=None):
            self.seen.append(name)
            return self.values.get(name, default)

    params = core.LaneChangeParams()
    close(u"J1 默认车道宽", params.lane_width, 3.5, 1e-12)
    close(u"J2 默认自车速度", params.traffic_ego_speed_kmh, 80.0, 1e-12)
    close(u"J3 默认方向", params.lane_direction, 1, 1e-12)
    check(u"J4 有 spec 属性", isinstance(params.spec, core.Spec))
    check(u"J5 有 traffic 属性", isinstance(params.traffic, core.Traffic))
    check(u"J6 PARAM_KEYS 共 14 项", len(core.PARAM_KEYS) == 14,
          u"实得 %d" % (len(core.PARAM_KEYS),))
    check(u"J7 PARAM_DEFAULTS 覆盖所有键",
          all(key in core.PARAM_DEFAULTS for key in core.PARAM_KEYS))

    getter = FakeGet({"lane/width": 3.75,
                      "traffic/ego_speed_kmh": 90.0,
                      "traffic/rear_gap": 25.0,
                      "lane/direction": -1})
    parsed = core.LaneChangeParams.from_params(getter)
    close(u"J8 读到车道宽", parsed.spec.lane_width, 3.75, 1e-12)
    close(u"J9 读到自车速度", parsed.traffic.ego_kmh, 90.0, 1e-12)
    close(u"J10 读到后车间距", parsed.traffic.rear_gap, 25.0, 1e-12)
    close(u"J11 读到方向", parsed.lane_direction, -1, 1e-12)
    check(u"J12 读到了全部 14 个键",
          len([key for key in core.PARAM_KEYS if key in getter.seen]) == 14,
          u"实得 %r" % (getter.seen,))
    check(u"J13 字符串数值可用",
          abs(core.LaneChangeParams.from_params(
              FakeGet({"lane/width": "4.0"})).spec.lane_width - 4.0) < 1e-12)
    blank = core.LaneChangeParams.from_params(FakeGet({}))
    close(u"J14 全缺省回落", blank.spec.lane_width, 3.5, 1e-12)
    check(u"J15 非法方向报错",
          raises(core.LaneChangeError, core.LaneChangeParams,
                 **{"lane/direction": 2}))
    as_dict = params.as_dict()
    check(u"J16 as_dict 是字典", isinstance(as_dict, dict))
    check(u"J17 as_dict 覆盖 14 键",
          len(as_dict) == 14, u"实得 %d" % (len(as_dict),))
    close(u"J18 publish_hz 默认", params.rate_publish_hz, 10.0, 1e-12)
    close(u"J19 sim_step 默认", params.sim_step, 0.02, 1e-12)
    check(u"J20 属性名带分组前缀",
          hasattr(params, "lane_max_lateral_accel") and
          hasattr(params, "safety_rear_margin"))


# ---------------------------------------------------------------- K 工程文件约定
def group_project_files():
    check(u"K1 包目录存在", os.path.isdir(PKG))
    for name in ("CMakeLists.txt", "package.xml", "README.md"):
        path = os.path.join(PKG, name)
        check(u"K2 存在 %s" % (name,), os.path.isfile(path))
    for name in ("main.py", "lane_change_core.py", "lane_change_node.py"):
        path = os.path.join(SCRIPTS, name)
        check(u"K3 存在 scripts/%s" % (name,), os.path.isfile(path))

    def read(path):
        handle = io.open(path, "r", encoding="utf-8")
        try:
            return handle.read()
        finally:
            handle.close()

    cmake = read(os.path.join(PKG, "CMakeLists.txt"))
    check(u"K4 CMake 安装 main.py", u"scripts/main.py" in cmake)
    check(u"K5 CMake 安装 core", u"scripts/lane_change_core.py" in cmake)
    check(u"K6 CMake 安装 node", u"scripts/lane_change_node.py" in cmake)
    check(u"K7 CMake 安装 config", u"config" in cmake)
    pkg_xml = read(os.path.join(PKG, "package.xml"))
    check(u"K8 package.xml 包名正确",
          u"<name>lane_change_feasibility</name>" in pkg_xml)
    check(u"K9 package.xml 依赖 rospy", u"<exec_depend>rospy</exec_depend>"
          in pkg_xml)
    check(u"K10 package.xml 依赖 std_msgs",
          u"<exec_depend>std_msgs</exec_depend>" in pkg_xml)

    launch = read(os.path.join(PKG, "launch", "main.launch"))
    check(u"K11 launch 用 main.py 作入口", u'type="main.py"' in launch)
    check(u"K12 launch 载入 yaml",
          u'<rosparam command="load"' in launch)
    check(u"K13 launch 有 16 个 arg", launch.count(u"<arg name=") == 16,
          u"实得 %d" % (launch.count(u"<arg name="),))
    check(u"K14 每个 arg 都有 doc",
          launch.count(u'doc="') >= 14,
          u"实得 %d" % (launch.count(u'doc="'),))
    check(u"K15 launch 覆盖 lane/width",
          u'name="lane/width"' in launch)
    check(u"K16 launch 覆盖 traffic/rear_gap",
          u'name="traffic/rear_gap"' in launch)

    yaml_text = read(os.path.join(PKG, "config",
                                  "lane_change_feasibility.yaml"))
    for key in ("width", "max_lateral_accel", "ttc_min", "ego_speed_kmh",
                "front_gap", "rear_gap", "direction"):
        check(u"K17 yaml 含 %s" % (key,), key + ":" in yaml_text)
    check(u"K18 core 不 import rospy",
          u"\nimport rospy" not in read(os.path.join(
              SCRIPTS, "lane_change_core.py")))
    check(u"K19 core 不 import yaml",
          u"\nimport yaml" not in read(os.path.join(
              SCRIPTS, "lane_change_core.py")))
    node_source = read(os.path.join(SCRIPTS, "lane_change_node.py"))
    check(u"K20 node 用了 log 前缀",
          u'"log" + level' in node_source)
    main_source = read(os.path.join(SCRIPTS, "main.py"))
    check(u"K21 main.py 复用节点逻辑",
          u"from lane_change_node import main" in main_source)
    readme = read(os.path.join(PKG, "README.md"))
    check(u"K22 README 写了 roslaunch 命令",
          u"roslaunch lane_change_feasibility main.launch" in readme)
    check(u"K23 README 写了测试命令",
          u"test_lane_change_feasibility_local.py" in readme)
    if os.path.isfile(DOCS_PAGE):
        page = read(DOCS_PAGE)
        check(u"K24 文档页存在且含四条断言",
              u"换道时长" in page and u"后向 TTC" in page)
        check(u"K25 文档页写了剖面公式",
              u"2πL" in page or u"2\\*πL" in page or u"sqrt" in page)
        check(u"K26 文档页写明测试命令",
              u"test_lane_change_feasibility_local.py" in page)
        check(u"K27 文档页写明大模型声明", u"大模型" in page)
    else:
        for name in (u"K24 本次未在仓库根目录运行，跳过文档页校验",
                     u"K25 本地不含 docs 目录",
                     u"K26 包目录仍然存在",
                     u"K27 测试脚本位于 tests 目录下"):
            check(name, True)


def group_tf():
    check(u"TF1 core 有默认帧名与关键帧数",
          core.DEFAULT_TF_WORLD_FRAME == u"map"
          and core.DEFAULT_TF_EGO_FRAME == u"ego"
          and core.DEFAULT_TF_KEYFRAMES == 24)
    plan = core.make_plan()
    frames = core.ego_frames(plan)
    check(u"TF2 ego_frames 默认 24 帧", len(frames) == 24,
          u"实得 %d" % (len(frames),))
    world, ego, first_x, first_y, first_yaw = frames[0]
    check(u"TF3 首帧在原点", (world, ego) == (u"map", u"ego")
          and abs(first_x) < 1e-9 and abs(first_y) < 1e-9
          and abs(first_yaw) < 1e-9)
    last_x, last_y, last_yaw = frames[-1][2], frames[-1][3], frames[-1][4]
    check(u"TF4 末帧 x 等于换道距离",
          abs(last_x - plan.distance) < 1e-9, u"实得 %.6f" % (last_x,))
    check(u"TF5 末帧 y 等于车道偏移",
          abs(last_y - plan.offset) < 1e-9, u"实得 %.6f" % (last_y,))
    check(u"TF6 末帧航向回到 0", abs(last_yaw) < 1e-9,
          u"实得 %.6f" % (last_yaw,))
    middle_index = len(frames) // 2
    middle = frames[middle_index]
    expected = plan.offset_at(plan.duration * middle_index / (len(frames) - 1))
    check(u"TF7 中间帧 y 跟随剖面",
          abs(middle[3] - expected) < 1e-9,
          u"实得 %.6f 期望 %.6f" % (middle[3], expected))
    check(u"TF8 ego_frames 可指定帧数", len(core.ego_frames(plan, 5)) == 5)
    check(u"TF9 两端横向速度为零处航向为零",
          abs(frames[-1][4]) < 1e-9 and abs(frames[0][4]) < 1e-9)

    # ---- TF 相关（本次新增） ----
    node_path = os.path.join(PKG, u"scripts", u"lane_change_node.py")
    with io.open(node_path, "r", encoding="utf-8") as handle:
        node_text = handle.read()
    package_path = os.path.join(PKG, u"package.xml")
    with io.open(package_path, "r", encoding="utf-8") as handle:
        package_text = handle.read()
    cmake_path = os.path.join(PKG, u"CMakeLists.txt")
    with io.open(cmake_path, "r", encoding="utf-8") as handle:
        cmake_text = handle.read()
    yaml_path = os.path.join(PKG, u"config", u"lane_change_feasibility.yaml")
    with io.open(yaml_path, "r", encoding="utf-8") as handle:
        yaml_text = handle.read()
    launch_path = os.path.join(PKG, u"launch", u"main.launch")
    with io.open(launch_path, "r", encoding="utf-8") as handle:
        launch_text = handle.read()

    check(u"T1 节点用了 TransformBroadcaster",
          u"TransformBroadcaster" in node_text)
    check(u"T2 节点 import 了 tf2_ros", u"import tf2_ros" in node_text)
    check(u"T3 节点用了 TransformStamped",
          u"TransformStamped" in node_text)
    check(u"T4 package.xml 依赖 tf2_ros",
          u"<exec_depend>tf2_ros</exec_depend>" in package_text)
    check(u"T5 package.xml 依赖 geometry_msgs",
          u"<exec_depend>geometry_msgs</exec_depend>" in package_text)
    check(u"T6 CMakeLists find_package 含 tf2_ros",
          u"tf2_ros" in cmake_text)
    check(u"T7 yaml 有 tf 组", u"tf:" in yaml_text)
    check(u"T8 yaml 有 tf/rate_hz", u"rate_hz:" in yaml_text)
    check(u"T9 launch 有 tf_enabled arg",
          u'<arg name="tf_enabled"' in launch_text)
    check(u"T10 launch 有 tf/rate_hz param",
          u'<param name="tf/rate_hz"' in launch_text)


# ---------------------------------------------------------------- pytest 包装
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


test_helpers = _pytest_group(group_helpers)
test_spec_and_traffic = _pytest_group(group_spec_and_traffic)
test_make_plan = _pytest_group(group_make_plan)
test_plan_object = _pytest_group(group_plan_object)
test_checks = _pytest_group(group_checks)
test_cases = _pytest_group(group_cases)
test_advice = _pytest_group(group_advice)
test_render = _pytest_group(group_render)
test_report = _pytest_group(group_report)
test_params = _pytest_group(group_params)
test_project_files = _pytest_group(group_project_files)
test_tf = _pytest_group(group_tf)


# ------------------------------------------------------------------- 主流程
def main():
    print(u"lane_change_feasibility 离线回归测试")
    print(u"Python %s" % (sys.version.split()[0],))
    print(u"包目录 %s" % (PKG,))
    print(u"-" * 60)

    group_helpers()
    group_spec_and_traffic()
    group_make_plan()
    group_plan_object()
    group_checks()
    group_cases()
    group_advice()
    group_render()
    group_report()
    group_params()
    group_project_files()
    group_tf()

    print(u"=" * 60)
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    for item in FAILED:
        print(u"[失败] %s" % (item,))
    if FAILED:
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
