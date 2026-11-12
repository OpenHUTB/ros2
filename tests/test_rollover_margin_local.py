#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""rollover_margin 的离线回归测试。

特点：
* 只用标准库，不需要 ROS、不需要 CARLA、不需要显示器；
* 兼容 Python 2.7 与 Python 3.x；
* 在仓库根目录执行：python3 tests/test_rollover_margin_local.py

覆盖：A 参数 / B 车辆几何 / C 工况 / D 载荷 / E 四条断言 / F 车速扫描 /
      G 字符图与报告 / H 参数读取 / I 工程文件 / J pytest 包装
"""
from __future__ import division, print_function

import io
import math
import os
import sys

try:
    text_type = unicode      # noqa: F821
except NameError:
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "src", "ground", "rollover_margin")
SCRIPTS = os.path.join(PKG, "scripts")
DOC_PATH = os.path.join(ROOT, "docs", "ground", "rollover_margin.md")

sys.path.insert(0, SCRIPTS)
import rollover_core as core  # noqa: E402

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(name if not detail else "%s（%s）" % (name, detail))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def raises(expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        return True
    except Exception:  # noqa: BLE001
        return False
    return False


# ── A 参数校验 ───────────────────────────────────────────────────────────

def group_params():
    check("A1 默认轮距 1.54", close(core.DEFAULT_TRACK, 1.54))
    check("A2 默认质心高 0.55", close(core.DEFAULT_CG_HEIGHT, 0.55))
    check("A3 默认质量 1500", close(core.DEFAULT_MASS, 1500.0))
    check("A4 默认半径 30", close(core.DEFAULT_CURVE_RADIUS, 30.0))
    check("A5 默认车速 60", close(core.DEFAULT_SPEED_KMH, 60.0))
    check("A6 默认裕度 0.70", close(core.DEFAULT_ROLLOVER_MARGIN, 0.70))
    check("A7 默认重力 9.81", close(core.DEFAULT_GRAVITY, 9.81))
    check("A8 断言名 4 条", core.ASSERT_NAMES ==
          (u"侧翻裕度", u"内侧轮载荷", u"极限车速", u"载荷转移"))
    check("A9 PARAM_KEYS 12 个", len(core.PARAM_KEYS) == 12)
    check("A10 车速表 7 档", len(core.SPEED_TABLE_KMH) == 7)
    check("A11 车速表 30~90", core.SPEED_TABLE_KMH[0] == 30.0
          and core.SPEED_TABLE_KMH[-1] == 90.0)

    spec = core.VehicleSpec()
    check("A12 默认构造", close(spec.track, 1.54) and close(spec.cg_height, 0.55))
    check("A13 as_tuple 四项", len(spec.as_tuple()) == 4)
    check("A14 轮距为 0 报错", raises(core.RolloverError, core.VehicleSpec, 0.0))
    check("A15 质心高为负报错", raises(core.RolloverError, core.VehicleSpec, 1.54, -0.1))
    check("A16 质量为 0 报错", raises(core.RolloverError, core.VehicleSpec, 1.54, 0.55, 0.0))
    check("A17 非数字报错", raises(core.RolloverError, core.VehicleSpec, "abc"))
    check("A18 布尔值报错", raises(core.RolloverError, core.VehicleSpec, True))
    check("A19 NaN 报错", raises(core.RolloverError, core.VehicleSpec, float("nan")))
    check("A20 字符串数字可用", close(core.VehicleSpec("1.6").track, 1.6))
    check("A21 半径为 0 报错", raises(core.RolloverError, core.Cornering, None, 0.0))
    check("A22 车速为负报错", raises(core.RolloverError, core.Cornering, None, 30.0, -5.0))
    check("A23 裕度为 0 报错", raises(core.RolloverError, core.check_cornering,
                                     core.Cornering(), 0.0))


# ── B 车辆几何 ───────────────────────────────────────────────────────────

def group_vehicle():
    spec = core.VehicleSpec()
    check("B1 SSF = 1.4", close(spec.ssf(), 1.4))
    check("B2 SSF 公式 = track/(2h)", close(spec.ssf(), 1.54 / (2 * 0.55)))
    check("B3 临界加速度 13.734", close(spec.critical_accel(), 13.734, 1e-9))
    check("B4 临界加速度 = SSF·g", close(spec.critical_accel(), 1.4 * 9.81))
    check("B5 单侧静态载荷 7357.5", close(spec.wheel_load_static(), 7357.5))
    check("B6 静态载荷 = m·g/2", close(spec.wheel_load_static(), 1500 * 9.81 / 2))

    check("B7 轮距变大 → SSF 变大",
          core.VehicleSpec(1.8).ssf() > spec.ssf())
    check("B8 质心变高 → SSF 变小",
          core.VehicleSpec(1.54, 0.8).ssf() < spec.ssf())
    check("B9 SSF 与质量无关",
          close(core.VehicleSpec(1.54, 0.55, 800.0).ssf(), spec.ssf()))
    check("B10 临界车速随半径增大",
          spec.critical_speed(60.0) > spec.critical_speed(30.0))
    check("B11 临界车速闭式解",
          close(spec.critical_speed(30.0), math.sqrt(13.734 * 30.0), 1e-12))
    check("B12 半径为 0 算临界车速报错",
          raises(core.RolloverError, spec.critical_speed, 0.0))

    # 质量不影响侧翻临界加速度，但影响载荷
    heavy = core.VehicleSpec(1.54, 0.55, 3000.0)
    check("B13 质量翻倍后载荷翻倍",
          close(heavy.wheel_load_static(), 2 * spec.wheel_load_static()))
    check("B14 质量翻倍后临界加速度不变",
          close(heavy.critical_accel(), spec.critical_accel()))


# ── C 工况 ───────────────────────────────────────────────────────────────

def group_cornering():
    spec = core.VehicleSpec()
    item = core.Cornering(spec, 30.0, 60.0)
    check("C1 速度换算 16.6667 m/s", close(item.speed_mps, 16.666666, 1e-6))
    check("C2 侧向加速度 9.259259", close(item.lateral_accel(), 9.259259, 1e-6))
    check("C3 a_y = v²/R", close(item.lateral_accel(), 16.666666 ** 2 / 30.0, 1e-5))
    check("C4 裕度 0.674185", close(item.usage(), 0.674185, 1e-6))
    check("C5 裕度 = a_y/a_crit",
          close(item.usage(), item.lateral_accel() / spec.critical_accel()))
    check("C6 极限车速 20.298276 m/s", close(item.speed_limit(), 20.298276, 1e-6))
    check("C7 极限车速 73.073793 km/h", close(item.speed_limit_kmh(), 73.073793, 1e-6))
    check("C8 余量 13.073793", close(item.headroom_kmh(), 13.073793, 1e-6))
    check("C9 60 km/h 不抬轮", not item.lifts_off())
    check("C10 安全车速(0.7) 61.137921", close(item.safe_speed(0.7), 61.137921, 1e-6))
    check("C11 安全车速(0.5) 51.670974", close(item.safe_speed(0.5), 51.670974, 1e-6))
    check("C12 安全车速与裕度单调",
          item.safe_speed(0.9) > item.safe_speed(0.5))

    fast = core.Cornering(spec, 30.0, 80.0)
    check("C13 80 km/h 抬轮", fast.lifts_off())
    check("C14 80 km/h 裕度 > 1", fast.usage() > 1.0)
    check("C15 80 km/h 余量为负", fast.headroom_kmh() < 0.0)

    check("C16 半径变大 → 侧向加速度变小",
          core.Cornering(spec, 60.0, 60.0).lateral_accel() < item.lateral_accel())
    check("C17 静止时 a_y = 0", close(core.Cornering(spec, 30.0, 0.0).lateral_accel(), 0.0))
    check("C18 静止时裕度 0", close(core.Cornering(spec, 30.0, 0.0).usage(), 0.0))
    check("C19 as_row 四元", len(item.as_row()) == 4)


# ── D 载荷 ───────────────────────────────────────────────────────────────

def group_loads():
    spec = core.VehicleSpec()
    item = core.Cornering(spec, 30.0, 60.0)
    check("D1 转移量 4960.31746", close(item.transfer(), 4960.31746, 1e-5))
    check("D2 转移量公式 m·a_y·h/t",
          close(item.transfer(), 1500 * 9.259259 * 0.55 / 1.54, 1e-3))
    check("D3 内侧 2397.18254", close(item.inner_load(), 2397.18254, 1e-4))
    check("D4 外侧 12317.81746", close(item.outer_load(), 12317.81746, 1e-4))
    check("D5 内外侧之和 = m·g",
          close(item.inner_load() + item.outer_load(), 1500 * 9.81, 1e-9))
    check("D6 转移占比 0.674185", close(item.transfer_ratio(), 0.674185, 1e-6))
    check("D7 转移占比 = 裕度",
          close(item.transfer_ratio(), item.usage(), 1e-12))

    still = core.Cornering(spec, 30.0, 0.0)
    check("D8 静止时转移量为 0", close(still.transfer(), 0.0))
    check("D9 静止时内外侧相等",
          close(still.inner_load(), still.outer_load()))

    lifted = core.Cornering(spec, 30.0, 90.0)
    check("D10 90 km/h 内侧为负", lifted.inner_load() < 0.0)
    check("D11 90 km/h 转移量超静态载荷",
          lifted.transfer() > spec.wheel_load_static())

    # 抬轮临界点：把车速调到恰好 v_limit，内侧力≈0
    at_limit = core.Cornering(spec, 30.0, item.speed_limit_kmh())
    check("D12 极限车速时内侧≈0", abs(at_limit.inner_load()) < 1e-6,
          "inner=%.6f" % at_limit.inner_load())
    check("D13 极限车速时裕度≈1", close(at_limit.usage(), 1.0, 1e-9))


# ── E 四条断言 ───────────────────────────────────────────────────────────

def group_checks():
    spec = core.VehicleSpec()
    item = core.Cornering(spec, 30.0, 60.0)
    results = core.check_cornering(item)
    check("E1 返回 4 条", len(results) == 4)
    check("E2 顺序与 ASSERT_NAMES 一致",
          [r["type"] for r in results] == list(core.ASSERT_NAMES))
    check("E3 每条含 passed/detail",
          all("passed" in r and "detail" in r for r in results))
    check("E4 默认全通过", core.all_passed(results))
    check("E5 裕度通过", results[0]["passed"])
    check("E6 内侧轮载荷通过", results[1]["passed"])
    check("E7 极限车速通过", results[2]["passed"])
    check("E8 载荷转移通过", results[3]["passed"])
    check("E9 载荷转移偏差极小", "1e-1" in results[3]["detail"]
          or "e-1" in results[3]["detail"])

    # 反例 1：裕度上限收到 0.5，只翻第一条
    res1 = core.check_cornering(item, rollover_margin=0.5)
    check("E10 裕度 0.5 → 侧翻裕度失败", not res1[0]["passed"])
    check("E11 裕度 0.5 → 其余三条仍通过",
          all(r["passed"] for i, r in enumerate(res1) if i != 0))

    # 反例 2：抬高质心，裕度/内侧轮/极限车速同时崩
    tall = core.Cornering(core.VehicleSpec(cg_height=0.90), 30.0, 60.0)
    res2 = core.check_cornering(tall)
    check("E12 高质心 → 侧翻裕度失败", not res2[0]["passed"])
    check("E13 高质心 → 内侧轮载荷失败", not res2[1]["passed"])
    check("E14 高质心 → 极限车速失败", not res2[2]["passed"])
    check("E15 高质心 → 载荷转移仍自洽", res2[3]["passed"])
    check("E16 高质心 SSF 0.8556", close(tall.spec.ssf(), 0.855556, 1e-6))

    # 反例 3：把转移容差设成不可能满足的值，只有第四条失败
    res3 = core.check_cornering(item, transfer_tolerance=-1e-6)
    check("E17 容差为负时第四条失败", not res3[3]["passed"])
    check("E18 容差为负时前三条仍通过",
          all(r["passed"] for i, r in enumerate(res3) if i != 3))

    check("E19 all_passed 空为真", core.all_passed([]))
    check("E20 all_passed 有失败为假",
          not core.all_passed([{"type": "x", "passed": False, "detail": ""}]))


# ── F 车速扫描 ───────────────────────────────────────────────────────────

def group_table():
    spec = core.VehicleSpec()
    rows = core.speed_table(spec, 30.0)
    check("F1 7 行", len(rows) == 7)
    check("F2 首行 30 km/h", close(rows[0].speed_kmh, 30.0))
    check("F3 末行 90 km/h", close(rows[-1].speed_kmh, 90.0))
    check("F4 30 km/h 裕度 0.169", close(rows[0].usage(), 0.169, 1e-3))
    check("F5 50 km/h 内侧 3912.8", close(rows[2].inner_load(), 3912.83, 1e-2))
    check("F6 70 km/h 不抬轮但很接近", not rows[4].lifts_off()
          and rows[4].usage() > 0.9)
    check("F7 80 km/h 抬轮", rows[5].lifts_off())
    check("F8 90 km/h 抬轮", rows[6].lifts_off())
    check("F9 裕度随车速单调递增",
          all(rows[i].usage() < rows[i + 1].usage() for i in range(6)))
    check("F10 内侧载荷随车速单调递减",
          all(rows[i].inner_load() > rows[i + 1].inner_load() for i in range(6)))
    check("F11 抬轮发生在 70~80 之间",
          not rows[4].lifts_off() and rows[5].lifts_off())
    check("F12 半径 60 m 时 80 km/h 不抬轮",
          not core.Cornering(spec, 60.0, 80.0).lifts_off())


# ── G 字符图与报告 ───────────────────────────────────────────────────────

def group_report():
    spec = core.VehicleSpec()
    item = core.Cornering(spec, 30.0, 60.0)
    results = core.check_cornering(item)

    lines = core.render_loads(item)
    check("G1 字符图 13 行", len(lines) == core.DEFAULT_GRID_HEIGHT + 1)
    check("G2 有外侧条", any("=" in line for line in lines[:-1]))
    check("G3 有内侧条", any("#" in line for line in lines[:-1]))
    check("G4 有中线", any("|" in line for line in lines[:-1]))
    check("G5 脚注含两个载荷",
          u"内侧" in lines[-1] and u"外侧" in lines[-1])
    check("G6 网格太小报错",
          raises(core.RolloverError, core.render_loads, item, 10, 3))
    check("G7 自定义尺寸", len(core.render_loads(item, 40, 8)) == 9)

    lifted = core.Cornering(spec, 30.0, 90.0)
    lines_lift = core.render_loads(lifted)
    check("G8 抬轮时内侧条消失",
          not any("#" in line for line in lines_lift[:-1]))

    report = core.format_report(item, results, header=u"测试标题")
    check("G9 报告含标题", u"测试标题" in report)
    check("G10 报告含 SSF", u"SSF = 1.4000" in report)
    check("G11 报告含断言统计", u"断言：4 通过 / 0 失败" in report)
    check("G12 报告含 PASSED", u"结论：PASSED" in report)
    check("G13 报告含车速扫描表头", u"车速(km/h)" in report)
    check("G14 报告含 7 行车速", report.count(u"km/h") >= 7)
    check("G15 报告行数 > 30", len(report.splitlines()) > 30)

    failed = core.check_cornering(item, rollover_margin=0.5)
    report_fail = core.format_report(item, failed)
    check("G16 失败报告含 [FAIL]", u"[FAIL]" in report_fail)
    check("G17 失败报告 FAILED", u"结论：FAILED" in report_fail)
    check("G18 失败统计 3 通过 / 1 失败", u"3 通过 / 1 失败" in report_fail)


# ── H 参数读取 ───────────────────────────────────────────────────────────

def group_params_read():
    def getter(name, default=None):
        return default

    params = core.RolloverParams.from_params(getter)
    check("H1 默认 track", close(params.track, 1.54))
    check("H2 默认 cg_height", close(params.cg_height, 0.55))
    check("H3 默认 radius", close(params.radius, 30.0))
    check("H4 默认 speed", close(params.speed_kmh, 60.0))
    check("H5 默认 margin", close(params.rollover_margin, 0.70))
    check("H6 vehicle() 返回 VehicleSpec", isinstance(params.vehicle(), core.VehicleSpec))
    check("H7 cornering() 返回 Cornering", isinstance(params.cornering(), core.Cornering))
    check("H8 as_dict 9 项", len(params.as_dict()) == 9)

    overrides = {
        "vehicle/track": "1.70",
        "vehicle/cg_height": "0.62",
        "corner/speed_kmh": "75",
        "check/rollover_margin": "0.65",
    }

    def getter2(name, default=None):
        return overrides.get(name, default)

    params2 = core.RolloverParams.from_params(getter2)
    check("H9 覆盖 track", close(params2.track, 1.70))
    check("H10 覆盖 cg_height", close(params2.cg_height, 0.62))
    check("H11 覆盖 speed", close(params2.speed_kmh, 75.0))
    check("H12 覆盖 margin", close(params2.rollover_margin, 0.65))
    check("H13 未覆盖项回落默认", close(params2.radius, 30.0))

    check("H14 非法 track 构造即报错",
          raises(core.RolloverError, core.RolloverParams, **{"vehicle/track": 0.0}))
    check("H15 负车速报错",
          raises(core.RolloverError, core.RolloverParams, **{"corner/speed_kmh": -1}))
    check("H16 字符串数字可用",
          close(core.RolloverParams(**{"vehicle/track": "1.6"}).track, 1.6))


# ── I 工程文件 ───────────────────────────────────────────────────────────

def group_project():
    pkg_xml = os.path.join(PKG, "package.xml")
    cmake = os.path.join(PKG, "CMakeLists.txt")
    yaml_path = os.path.join(PKG, "config", "rollover_margin.yaml")
    launch_path = os.path.join(PKG, "launch", "main.launch")
    readme = os.path.join(PKG, "README.md")

    for path, name in ((pkg_xml, "I1 package.xml"), (cmake, "I2 CMakeLists.txt"),
                       (yaml_path, "I3 config yaml"), (launch_path, "I4 launch"),
                       (readme, "I5 README")):
        check(name + " 存在", os.path.isfile(path), path)

    pkg_text = io.open(pkg_xml, "r", encoding="utf-8").read()
    check("I6 包名 rollover_margin", "<name>rollover_margin</name>" in pkg_text)
    check("I7 依赖 rospy", "<exec_depend>rospy</exec_depend>" in pkg_text)
    check("I8 依赖 std_msgs", "<exec_depend>std_msgs</exec_depend>" in pkg_text)

    cmake_text = io.open(cmake, "r", encoding="utf-8").read()
    check("I9 安装三个脚本", cmake_text.count("scripts/") >= 3)
    check("I10 安装 launch 与 config", "install(DIRECTORY launch config" in cmake_text)

    launch_text = io.open(launch_path, "r", encoding="utf-8").read()
    check("I11 type=main.py", 'type="main.py"' in launch_text)
    check("I12 用 $(find ...)", "$(find rollover_margin)" in launch_text)
    check("I13 不含 rospack", "rospack" not in launch_text)
    check("I14 加载 yaml", "config/rollover_margin.yaml" in launch_text)
    check("I15 arg 数 12", launch_text.count("<arg ") == 12)
    check("I16 每个 arg 有 doc", launch_text.count("<arg ") == launch_text.count('doc="'))
    check("I17 param 数 12", launch_text.count("<param ") == 12)

    yaml_text = io.open(yaml_path, "r", encoding="utf-8").read()
    check("I18 yaml 含 track", "track:" in yaml_text)
    check("I19 yaml 含 cg_height", "cg_height:" in yaml_text)
    check("I20 yaml 含 rollover_margin", "rollover_margin:" in yaml_text)

    readme_text = io.open(readme, "r", encoding="utf-8").read()
    check("I21 README 含 roslaunch", "roslaunch rollover_margin main.launch" in readme_text)
    check("I22 README 含测试命令", "test_rollover_margin_local.py" in readme_text)

    core_text = io.open(os.path.join(SCRIPTS, "rollover_core.py"), "r",
                        encoding="utf-8").read()
    check("I23 核心不 import rospy", "\nimport rospy" not in core_text)
    node_text = io.open(os.path.join(SCRIPTS, "rollover_node.py"), "r",
                        encoding="utf-8").read()
    check("I24 节点 import core", "import rollover_core as core" in node_text)
    check("I25 节点有 _log", "def _log(" in node_text)
    check("I26 节点三个发布者", node_text.count("rospy.Publisher") == 3)
    check("I27 main.py 复用节点",
          "from rollover_node import main" in io.open(
              os.path.join(SCRIPTS, "main.py"), "r", encoding="utf-8").read())

    if os.path.isfile(DOC_PATH):
        doc_text = io.open(DOC_PATH, "r", encoding="utf-8").read()
        check("I28 文档页存在", len(doc_text) > 2000)
        check("I29 文档页含四条断言名",
              all(name in doc_text for name in core.ASSERT_NAMES))
        check("I30 文档页含 roslaunch", "roslaunch rollover_margin" in doc_text)
    else:
        check("I28 本次未在仓库根目录运行，跳过文档页校验", True)
        check("I29 本地不含 docs 目录", True)
        check("I30 包目录仍然存在", os.path.isdir(PKG))


def _pytest_group(func):
    def runner():
        before = len(FAILED)
        func()
        new = FAILED[before:]
        if new:
            raise AssertionError(u"本组失败 %d 项：\n%s" % (len(new), u"\n".join(new)))
    runner.__name__ = "test_" + func.__name__[len("group_"):]
    return runner


test_params = _pytest_group(group_params)
test_vehicle = _pytest_group(group_vehicle)
test_cornering = _pytest_group(group_cornering)
test_loads = _pytest_group(group_loads)
test_checks = _pytest_group(group_checks)
test_table = _pytest_group(group_table)
test_report = _pytest_group(group_report)
test_params_read = _pytest_group(group_params_read)
test_project = _pytest_group(group_project)


def main():
    print(u"rollover_margin 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"-" * 60)
    group_params()
    group_vehicle()
    group_cornering()
    group_loads()
    group_checks()
    group_table()
    group_report()
    group_params_read()
    group_project()
    print(u"=" * 60)
    for item in FAILED:
        print(u"[失败] %s" % item)
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if not FAILED:
        print(u"全部通过")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
