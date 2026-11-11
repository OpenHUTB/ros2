#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""passability_check 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要仿真器、不需要显示器**
* 在仓库根目录执行：python3 tests/test_passability_check_local.py
* 也可以交给 pytest：python3 -m pytest tests/test_passability_check_local.py -q

覆盖范围：
A 默认值与常量      B 车辆参数校验      C 三个通过性角度
D 剖面查询与折点    E 弦长求解与位姿    F 侵入量与扫描
G 五条断言          H 报告与字符图      I 场景文件
J 结构化输出        K 工程文件约定
"""

from __future__ import print_function

import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "src", "ground", "passability_check")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "passability_check.md")

sys.path.insert(0, PKG)

import passability_check as pc  # noqa: E402
from passability_check import (  # noqa: E402
    VehicleSpec, VehicleError, Terrain, TerrainError, ClearanceError,
    Scenario, ScenarioError, seat, scan_profile, penetration_at,
    contact_front_x, get_vehicle, load_scenario, list_scenarios,
    evaluate, format_report, render_profile, build_assertions, all_passed,
    angle_passed, numeric_passed, verdict,
    VERDICT_FAILED, VERDICT_PASSED, VERDICT_PASS_WITH_NOTE,
    scenario_matrix_csv, to_json_payload,
)

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append("%s %s" % (name, detail))
    return bool(condition)


def check_close(name, actual, expected, tolerance=1e-6):
    try:
        same = abs(float(actual) - float(expected)) <= tolerance
    except (TypeError, ValueError):
        same = False
    return check(name, same, "实测 %r，期望 %r（容差 %g）" % (actual, expected, tolerance))


def check_raises(name, expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        return check(name, True)
    except Exception as error:  # noqa: BLE001
        return check(name, False, "抛出了 %s: %s" % (type(error).__name__, error))
    return check(name, False, "没有抛出 %s" % expected.__name__)


def read_text(path):
    with io.open(path, "r", encoding="utf-8") as handle:
        return handle.read()


# ------------------------------------------------------------------ A 默认值


def group_defaults():
    check("A01 版本号", isinstance(pc.__version__, str) and pc.__version__.count(".") >= 1,
          pc.__version__)
    check_close("A02 默认轴距", pc.PRESET_VEHICLES["sedan"].wheelbase, 2.70)
    check_close("A03 轿车车长", get_vehicle("sedan").length, 4.53, 1e-9)
    check_close("A04 SUV 车长", get_vehicle("suv").length, 4.43, 1e-9)
    check("A05 预置两款车", sorted(pc.PRESET_VEHICLES) == ["sedan", "suv"],
          sorted(pc.PRESET_VEHICLES))
    check("A06 断言名 5 条", len(pc.ASSERT_NAMES) == 5, pc.ASSERT_NAMES)
    check("A07 断言名唯一", len(set(pc.ASSERT_NAMES)) == 5)

    # 下缘采样点必须既足数又不重复（评审 @KKK555-MAX：41 个点里只有 40 个唯一）
    for name in ("sedan", "suv"):
        vehicle = get_vehicle(name)
        for count in (5, 41, 101):
            pts = vehicle.lower_contour(count)
            check("A08 %s 下缘 %d 点且唯一" % (name, count),
                  len(pts) == count and len(set(pts)) == count,
                  "%d 点 / %d 唯一" % (len(pts), len(set(pts))))
        check("A09 %s 下缘末点落在最后折点" % name,
              vehicle.lower_contour(41)[-1] == vehicle.lower_knots()[-1])


# ------------------------------------------------------------------ B 参数校验


def group_vehicle_validation():
    check_raises("B01 负轴距", VehicleError, VehicleSpec, wheelbase=-1.0)
    check_raises("B02 零前悬", VehicleError, VehicleSpec, front_overhang=0.0)
    check_raises("B03 负离地", VehicleError, VehicleSpec, clearance_mid=-0.01)
    check_raises("B04 轴距非数字", VehicleError, VehicleSpec, wheelbase="abc")
    check_raises("B05 轴距为 NaN", VehicleError, VehicleSpec, wheelbase=float("nan"))
    check_raises("B06 轴距为 inf", VehicleError, VehicleSpec, wheelbase=float("inf"))
    check_raises("B07 mid_ratio 越界", VehicleError, VehicleSpec, mid_ratio=1.2)
    check_raises("B08 mid_ratio 为零", VehicleError, VehicleSpec, mid_ratio=0.0)
    check_raises("B09 车宽小于轮距", VehicleError, VehicleSpec, width=1.0, track=1.54)
    check_raises("B10 未知车型", VehicleError, get_vehicle, "truck")
    check("B11 字符串数字可用", VehicleSpec(wheelbase="2.7").wheelbase == 2.7)
    check("B12 get_vehicle 忽略大小写", get_vehicle("SUV").name == "suv")


# ------------------------------------------------------------------ C 角度


def group_angles():
    sedan = get_vehicle("sedan")
    suv = get_vehicle("suv")
    check_close("C01 轿车接近角", sedan.approach_deg, 9.0395, 1e-3)
    check_close("C02 轿车离去角", sedan.departure_deg, 11.8887, 1e-3)
    check_close("C03 轿车纵向通过角", sedan.breakover_deg, 11.8412, 1e-3)
    check_close("C04 SUV 接近角", suv.approach_deg, 14.5111, 1e-3)
    check_close("C05 SUV 离去角", suv.departure_deg, 16.9275, 1e-3)
    check_close("C06 SUV 纵向通过角", suv.breakover_deg, 17.1037, 1e-3)
    check("C07 SUV 接近角大于轿车", suv.approach_deg > sedan.approach_deg)
    check("C08 SUV 纵向通过角大于轿车", suv.breakover_deg > sedan.breakover_deg)

    # 手算校核：atan(0.14 / 0.88)
    manual = math.degrees(math.atan(0.14 / 0.88))
    check_close("C09 接近角与手算一致", sedan.approach_deg, manual, 1e-9)
    manual = 2.0 * math.degrees(math.atan(2.0 * 0.14 / 2.70))
    check_close("C10 中点近似与通式一致", sedan.breakover_deg, manual, 1e-9)

    # 最低点越靠中间，两条切线越平，纵向通过角越小（中点最不利）
    shifted = VehicleSpec(mid_ratio=0.35)
    check("C11 最低点偏离中点后纵向通过角变大",
          shifted.breakover_deg > sedan.breakover_deg,
          "%.3f vs %.3f" % (shifted.breakover_deg, sedan.breakover_deg))
    check_close("C12 最低点位置", shifted.mid_position, 0.945, 1e-9)
    # 离地越高角度越大（单调）
    higher = VehicleSpec(clearance_front=0.28, clearance_mid=0.28, clearance_rear=0.28)
    check("C13 抬高离地后三者都变大",
          higher.approach_deg > sedan.approach_deg
          and higher.departure_deg > sedan.departure_deg
          and higher.breakover_deg > sedan.breakover_deg)
    check_close("C14 最低离地", sedan.min_clearance, 0.14, 1e-9)


# ------------------------------------------------------------------ D 剖面


def group_terrain():
    ramp = Terrain([[0.0, 0.0], [3.0, 0.0], [9.0, 1.2], [12.0, 1.2]])
    check_close("D01 平坦处高度", ramp.height(1.5), 0.0, 1e-12)
    check_close("D02 坡中高度", ramp.height(6.0), 0.6, 1e-12)
    check_close("D03 坡顶高度", ramp.height(10.0), 1.2, 1e-12)
    check_close("D04 坡中坡度", ramp.slope_deg(6.0), 11.3099, 1e-3)
    check_close("D05 左端延长", ramp.height(-5.0), 0.0, 1e-12)
    check_close("D06 右端延长", ramp.height(50.0), 1.2, 1e-12)
    check_close("D07 下凹折角", ramp.max_concave_deg(), 11.3099, 1e-3)
    check_close("D08 上凸折角", ramp.max_convex_deg(), 11.3099, 1e-3)
    breaks = ramp.breaks()
    check("D09 折点两个", len(breaks) == 2, len(breaks))
    check("D10 第一个折点是下凹", breaks[0]["kind"] == "concave", breaks[0])
    check("D11 第二个折点是上凸", breaks[1]["kind"] == "convex", breaks[1])
    check("D12 折点位置", abs(breaks[0]["x"] - 3.0) < 1e-9 and abs(breaks[1]["x"] - 9.0) < 1e-9)

    check_raises("D13 少于两点", TerrainError, Terrain, [[0.0, 0.0]])
    check_raises("D14 x 不递增", TerrainError, Terrain, [[0.0, 0.0], [0.0, 1.0]])
    check_raises("D15 点格式错误", TerrainError, Terrain, [[0.0, 0.0], [1.0]])
    check_raises("D16 非数字", TerrainError, Terrain, [[0.0, 0.0], [1.0, "x"]])

    crest = pc.terrain.crest_profile(up_deg=8.0, down_deg=4.0)
    check_close("D17 坡顶最大上凸折角", crest.max_convex_deg(), 12.0, 1e-9)
    check_close("D18 坡顶最高点", crest.max_z, 5.0 * math.tan(math.radians(8.0)), 1e-9)

    bump = pc.terrain.bump_profile(height=0.10, top_width=0.70, slope_width=0.35)
    check_close("D19 减速带最高点", bump.max_z, 0.10, 1e-12)
    check_close("D20 减速带坡面角", bump.max_concave_deg(),
                math.degrees(math.atan2(0.10, 0.35)), 1e-9)


# ------------------------------------------------------------------ E 弦长与位姿


def group_seating():
    sedan = get_vehicle("sedan")
    flat = Terrain([[0.0, 0.0], [20.0, 0.0]])
    pose = seat(flat, sedan, 5.0)
    check_close("E01 平地弦长等于轴距", pose["chord"], sedan.wheelbase, 1e-9)
    check_close("E02 平地俯仰角为 0", pose["pitch_deg"], 0.0, 1e-9)
    check_close("E03 平地前轴位置", pose["front_x"], 7.70, 1e-9)

    ramp = Terrain([[0.0, 0.0], [3.0, 0.0], [9.0, 1.2], [12.0, 1.2]])
    pose = seat(ramp, sedan, 5.0)
    check_close("E04 坡上弦长仍等于轴距", pose["chord"], sedan.wheelbase, 1e-6)
    check_close("E05 坡上俯仰角等于坡度", pose["pitch_deg"], 11.3099, 1e-3)
    check("E06 前轴在后轴之前", pose["front_x"] > pose["rear_x"])

    # 二分求解在陡坡上也要收敛
    steep = Terrain([[0.0, 0.0], [2.0, 0.0], [6.0, 2.0], [8.0, 2.0]])
    pose = seat(steep, sedan, 3.5)
    check_close("E07 陡坡弦长收敛", pose["chord"], sedan.wheelbase, 1e-6)
    check("E08 陡坡俯仰角显著", pose["pitch_deg"] > 15.0, pose["pitch_deg"])

    front_x = contact_front_x(flat, 0.0, 2.7)
    check_close("E09 接地点求解", front_x, 2.7, 1e-6)
    check_raises("E10 轴距为 0", ClearanceError, contact_front_x, flat, 0.0, 0.0)


# ------------------------------------------------------------------ F 侵入与扫描


def group_penetration():
    sedan = get_vehicle("sedan")
    suv = get_vehicle("suv")
    flat = Terrain([[0.0, 0.0], [20.0, 0.0]])
    pose = seat(flat, sedan, 4.0)
    record = penetration_at(flat, sedan, pose)
    check_close("F01 平地最大侵入为负", record["depth"], -0.14, 1e-6)
    check_close("F02 平地最小余量", record["margin"], 0.14, 1e-6)
    check("F03 平地最危险区段是车底", record["section"] == "车底中部", record["section"])
    check("F04 采样点数", record["points"] >= 20, record["points"])

    ramp = Terrain([[0.0, 0.0], [3.0, 0.0], [9.0, 1.2], [12.0, 1.2]])
    sedan_scan = scan_profile(ramp, sedan, steps=480)
    suv_scan = scan_profile(ramp, suv, steps=480)
    check("F05 轿车在地库坡底托底", sedan_scan["max_depth"] > 0.02, sedan_scan["max_depth"])
    check("F06 轿车最小余量为负", sedan_scan["min_margin"] < 0.0, sedan_scan["min_margin"])
    check("F07 SUV 同坡道不托底", suv_scan["max_depth"] <= 1e-9, suv_scan["max_depth"])
    check("F08 轿车侵入发生在车头", sedan_scan["worst"]["section"] == "车头",
          sedan_scan["worst"]["section"])
    check_close("F09 轿车最大侵入量", sedan_scan["max_depth"], 0.0351, 2e-3)
    check("F10 扫描步长合理", 0.0 < sedan_scan["step"] < 0.05, sedan_scan["step"])
    check("F11 扫描区间覆盖剖面",
          sedan_scan["start_x"] <= ramp.min_x and sedan_scan["end_x"] >= ramp.max_x)

    crest = Terrain([[0.0, 0.0], [5.0, 0.7027], [10.0, 0.3527]])
    crest_scan = scan_profile(crest, sedan, steps=480)
    check("F12 轿车在坡顶托底", crest_scan["max_depth"] > 0.0, crest_scan["max_depth"])
    check("F13 坡顶侵入发生在地板", crest_scan["worst"]["section"] == "车底中部",
          crest_scan["worst"]["section"])
    check("F14 SUV 坡顶不托底", scan_profile(crest, suv, steps=480)["max_depth"] <= 1e-9)

    bump = pc.terrain.bump_profile()
    bump_scan = scan_profile(bump, sedan, steps=480)
    check("F15 减速带不托底", bump_scan["max_depth"] <= 1e-9, bump_scan["max_depth"])
    check_close("F16 减速带最小余量", bump_scan["min_margin"], 0.04, 2e-3)

    # 采样越密，侵入量只会更准，不会消失
    dense = scan_profile(ramp, sedan, steps=960, samples=81)
    check("F17 加密采样仍托底", dense["max_depth"] > 0.02, dense["max_depth"])
    check_close("F18 加密采样侵入量接近", dense["max_depth"], sedan_scan["max_depth"], 3e-3)


# ------------------------------------------------------------------ G 断言


def group_assertions():
    sedan = get_vehicle("sedan")
    suv = get_vehicle("suv")
    garage = load_scenario(os.path.join(PKG, "config", "scenarios", "garage_ramp.json"))

    report, assertions = evaluate(sedan, garage)
    check("G01 五条断言", len(assertions) == 5, len(assertions))
    check("G02 轿车结论失败", not all_passed(assertions))
    failed = [item["type"] for item in assertions if not item["passed"]]
    check("G03 失败项是接近角与托底",
          failed == ["接近角", "全程不托底"], failed)
    check("G04 断言带 detail", all(item["detail"] for item in assertions))
    check("G05 断言带数值", all("value" in item and "need" in item for item in assertions))

    report_suv, assertions_suv = evaluate(suv, garage)
    check("G06 SUV 结论通过", all_passed(assertions_suv),
          [(item["type"], item["detail"]) for item in assertions_suv if not item["passed"]])

    # 安全余量抬高后 SUV 也不通过
    tight = Scenario(name="tight", terrain=garage.terrain.to_dict()["terrain"],
                     title="同一坡道但要求 30 cm 余量", min_clearance_m=0.12,
                     safety_margin_m=0.30)
    _, tight_assertions = evaluate(suv, tight)
    check("G07 安全余量生效", not all_passed(tight_assertions))

    # 每一条断言都能单独失败：把某个量抬到不可能
    harsh = Scenario(name="harsh", terrain=garage.terrain.to_dict()["terrain"],
                     title="苛刻门槛", min_clearance_m=0.60)
    _, harsh_assertions = evaluate(suv, harsh)
    clearance_assert = [item for item in harsh_assertions if item["type"] == "离地间隙"][0]
    check("G08 离地间隙可单独失败", not clearance_assert["passed"])

    # 结论三档：数值判据说了算，角度判据只降级成备注（评审 @KKK555-MAX 指出
    # 旧实现「结论 FAILED、备注说以数值复核为准」自相矛盾）
    check("G09 地库轿车结论 FAILED", verdict(assertions) == VERDICT_FAILED,
          verdict(assertions))
    check("G10 地库 SUV 结论 PASSED", verdict(assertions_suv) == VERDICT_PASSED,
          verdict(assertions_suv))

    bump = load_scenario(os.path.join(PKG, "config", "scenarios", "speed_bump.json"))
    bump_report, bump_assertions = evaluate(suv, bump)
    check("G11 减速带 SUV：数值通过但角度不通过",
          numeric_passed(bump_assertions) and not angle_passed(bump_assertions))
    check("G12 减速带 SUV 结论 PASS_WITH_NOTE",
          verdict(bump_assertions) == VERDICT_PASS_WITH_NOTE, verdict(bump_assertions))
    bump_payload = to_json_payload(suv, bump, bump_report, bump_assertions)
    check("G13 JSON 里 result 与 numeric_ok 一致",
          (bump_payload["result"] == VERDICT_FAILED) != bump_payload["numeric_ok"],
          bump_payload)
    bump_text = format_report(suv, bump, bump_report, bump_assertions)
    conclusion = [line for line in bump_text.splitlines() if line.startswith("结论：")][0]
    check("G14 结论行与备注不打架（不再出现 FAILED）",
          "FAILED" not in conclusion and "PASS_WITH_NOTE" in conclusion, conclusion)


# ------------------------------------------------------------------ H 报告与图


def group_report():
    sedan = get_vehicle("sedan")
    garage = load_scenario(os.path.join(PKG, "config", "scenarios", "garage_ramp.json"))
    report, assertions = evaluate(sedan, garage)
    text = format_report(sedan, garage, report, assertions, "标题行")

    check("H01 报告含标题", text.startswith("标题行"))
    for keyword in ("接近角", "离去角", "纵向通过角", "断言", "结论", "FAILED"):
        check("H02 报告含 %s" % keyword, keyword in text)
    check("H03 报告行数合理", 10 <= len(text.splitlines()) <= 20, len(text.splitlines()))
    check("H04 报告带差值", "差 -2.27°" in text, [line for line in text.splitlines()
                                                if "接近角" in line])

    figure = render_profile(garage.terrain, sedan, report["worst"]["pose"],
                            worst=report["worst"])
    lines = figure.splitlines()
    drawing = "\n".join([line for line in lines
                         if not line.startswith("-")
                         and "图例" not in line and "窗口" not in line])
    check("H05 图有内容", len(lines) >= 10, len(lines))
    check("H06 图含地面", "#" in figure)
    check("H07 图含车身", "o" in figure)
    check("H08 图含车轮", "@" in figure)
    check("H09 图标记侵入点", "X" in drawing)
    check("H10 图有图例", "图例" in figure)
    check("H11 图宽度一致", max(len(line) for line in lines) <= 92,
          max(len(line) for line in lines))

    suv = get_vehicle("suv")
    suv_report, _ = evaluate(suv, garage)
    suv_figure = render_profile(garage.terrain, suv, suv_report["worst"]["pose"],
                                worst=suv_report["worst"])
    suv_drawing = "\n".join([line for line in suv_figure.splitlines()
                              if not line.startswith("-")
                              and "图例" not in line and "窗口" not in line])
    check("H12 通过的场景不画侵入点", "X" not in suv_drawing)

    # 窄图也不能崩
    small = render_profile(garage.terrain, sedan, report["worst"]["pose"],
                           worst=report["worst"], width=40, height=10)
    check("H13 小尺寸图可用", len(small.splitlines()) >= 4)


# ------------------------------------------------------------------ I 场景


def group_scenarios():
    paths = list_scenarios()
    check("I01 场景不少于 3 个", len(paths) >= 3, len(paths))
    names = [os.path.basename(path) for path in paths]
    check("I02 场景都是 json", all(name.endswith(".json") for name in names), names)
    check("I03 场景文件名英文", all(name.replace(".json", "").replace("_", "").isalnum()
                                    for name in names), names)

    for path in paths:
        scenario = load_scenario(path)
        check("I04 %s 有标题" % scenario.name, bool(scenario.title))
        check("I05 %s 剖面合法" % scenario.name, len(scenario.terrain.points) >= 2)
        check("I06 %s 门槛非负" % scenario.name,
              scenario.min_clearance_m >= 0.0 and scenario.safety_margin_m >= 0.0)
        check("I07 %s 折点存在" % scenario.name,
              scenario.terrain.max_concave_deg() > 0.0 or scenario.terrain.max_convex_deg() > 0.0)

    check_raises("I08 不存在的场景", ScenarioError, load_scenario, "no_such_scene")
    check_raises("I09 缺 name", ScenarioError, Scenario.from_dict, {"terrain": [[0, 0], [1, 0]]})
    check_raises("I10 缺 terrain", ScenarioError, Scenario.from_dict, {"name": "x"})
    check_raises("I11 门槛为负", ScenarioError, Scenario.from_dict,
                 {"name": "x", "terrain": [[0, 0], [1, 0]], "min_clearance_m": -1.0})
    check_raises("I12 顶层不是对象", ScenarioError, Scenario.from_dict, [1, 2, 3])

    scenario = Scenario.from_dict({"name": "mini", "terrain": [[0, 0], [3, 0], [6, 1]]})
    check("I13 默认门槛", abs(scenario.min_clearance_m - 0.10) < 1e-12)
    check("I14 to_dict 往返", json.loads(json.dumps(scenario.to_dict()))["name"] == "mini")

    # 场景里的地形与手算折角一致
    garage = load_scenario(os.path.join(PKG, "config", "scenarios", "garage_ramp.json"))
    check_close("I15 地库坡道 20%", garage.terrain.max_concave_deg(), 11.3099, 1e-3)
    bump = load_scenario(os.path.join(PKG, "config", "scenarios", "speed_bump.json"))
    check_close("I16 减速带高度", bump.terrain.max_z, 0.10, 1e-9)


# ------------------------------------------------------------------ J 输出


def group_output():
    sedan = get_vehicle("sedan")
    garage = load_scenario(os.path.join(PKG, "config", "scenarios", "garage_ramp.json"))
    report, assertions = evaluate(sedan, garage)
    payload = to_json_payload(sedan, garage, report, assertions)

    check("J01 JSON 顶层键",
          set(payload) == {"vehicle", "scenario", "scan", "assertions", "result",
                           "numeric_ok", "angle_ok"},
          sorted(payload))
    check("J02 结论为 FAILED", payload["result"] == "FAILED", payload["result"])
    check("J03 断言条目 5 个", len(payload["assertions"]) == 5)
    check("J04 角度写进 JSON", abs(payload["vehicle"]["approach_deg"] - 9.0395) < 1e-3)
    check("J05 扫描最坏区段", payload["scan"]["worst_section"] == "车头",
          payload["scan"]["worst_section"])
    check("J06 JSON 可序列化", json.loads(json.dumps(payload, ensure_ascii=False))["result"]
          == "FAILED")

    rows = [{"scenario": "garage_ramp", "vehicle": "sedan", "approach_deg": "9.04",
             "departure_deg": "11.89", "breakover_deg": "11.84", "min_clearance_m": "0.140",
             "max_depth_m": "0.035", "min_margin_m": "-0.035", "result": "FAILED"}]
    csv_text = scenario_matrix_csv(rows)
    check("J07 CSV 表头", csv_text.splitlines()[0].startswith("scenario,vehicle"))
    check("J08 CSV 行数", len(csv_text.splitlines()) == 2, csv_text)
    check("J09 CSV 含结论", "FAILED" in csv_text)


# ------------------------------------------------------------------ K 工程约定


def group_project_layout():
    for name in ("package.xml", "setup.py", "setup.cfg", "README.md", "main.py",
                 "main.sh", "main.bat", "config", "launch", "passability_check"):
        check("K01 存在 %s" % name, os.path.exists(os.path.join(PKG, name)), name)

    check("K02 入口以 main 开头", os.path.isfile(os.path.join(PKG, "main.py")))
    launch = os.path.join(PKG, "launch", "main.launch.py")
    check("K03 launch 入口", os.path.isfile(launch), launch)
    if os.path.isfile(launch):
        content = read_text(launch)
        check("K04 launch 声明参数", "DeclareLaunchArgument" in content)
        check("K05 launch 起节点", "Node(" in content)

    # 文件名：全英文、无空格、无中文、下划线分隔
    bad = []
    for root, dirs, files in os.walk(PKG):
        for name in list(dirs) + files:
            stem = os.path.splitext(name)[0]
            if name.startswith("."):
                continue
            if any(ord(char) > 127 for char in name) or " " in name:
                bad.append(os.path.join(root, name))
            # README.md 是仓库约定的固定写法，其余文件一律小写下划线
            if stem != stem.lower() and name != "README.md":
                bad.append(os.path.join(root, name))
    check("K06 文件名合规", not bad, bad)

    # 文档页 + 首页 + 导航
    check("K07 文档页存在", os.path.isfile(DOCS_PAGE), DOCS_PAGE)
    if os.path.isfile(DOCS_PAGE):
        page = read_text(DOCS_PAGE)
        check("K08 文档页有标题", page.lstrip().startswith("#"))
        check("K09 文档页有公式表", "接近角" in page and "纵向通过角" in page)
        check("K10 文档页有运行命令", "python3 main.py" in page)
        check("K11 文档页有测试命令", "test_passability_check_local.py" in page)
        check("K12 文档页声明大模型", "大模型" in page)

    index_path = os.path.join(ROOT, "docs", "index.md")
    if os.path.isfile(index_path):
        check("K13 首页有链接", "./ground/passability_check.md" in read_text(index_path))
    mkdocs = os.path.join(ROOT, "mkdocs.yml")
    if os.path.isfile(mkdocs):
        check("K14 导航有条目", "ground/passability_check.md" in read_text(mkdocs))

    # package.xml 基本字段
    package_xml = os.path.join(PKG, "package.xml")
    if os.path.isfile(package_xml):
        content = read_text(package_xml)
        for tag in ("<name>passability_check</name>", "<license>", "<description>",
                    "ament_python"):
            check("K15 package.xml 含 %s" % tag, tag in content)


# ------------------------------------------------------------------ 入口


GROUPS = [
    ("A 默认值", group_defaults),
    ("B 参数校验", group_vehicle_validation),
    ("C 三个角度", group_angles),
    ("D 剖面", group_terrain),
    ("E 位姿", group_seating),
    ("F 侵入", group_penetration),
    ("G 断言", group_assertions),
    ("H 报告", group_report),
    ("I 场景", group_scenarios),
    ("J 输出", group_output),
    ("K 工程约定", group_project_layout),
]


def run_all():
    for _, func in GROUPS:
        func()
    print("通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        for item in FAILED:
            print("  [FAIL] %s" % item)
        print("存在失败项")
        return 1
    print("全部通过")
    return 0


def _make_test(func):
    def test_wrapper():
        before = len(FAILED)
        func()
        new = FAILED[before:]
        assert not new, "\n".join(new)
    test_wrapper.__name__ = "test_" + func.__name__
    return test_wrapper


for _name, _func in GROUPS:
    globals()[_make_test(_func).__name__] = _make_test(_func)


if __name__ == "__main__":
    sys.exit(run_all())
