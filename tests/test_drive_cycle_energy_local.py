#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""drive_cycle_energy 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**；
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）；
* 在仓库根目录执行：python3 tests/test_drive_cycle_energy_local.py

覆盖范围:
A VehicleSpec 参数       B 阻力计算          C 工况解析
D 工况积分               E 四条断言          F 能量恒等式
G 续航与百公里能耗       H 字符图            I 报告文本
J 参数对象               K 工程文件约定（含脚本编码声明的回归检查）

脚本编码声明这一组（K）是专门为本项目踩过的坑加的：Melodic 的 Python 2.7
要求每个 .py 的第 1 或第 2 行有 `# -*- coding: utf-8 -*-`，而 `sed` 改 shebang
时若把第 1 行整行覆盖，编码声明就会丢失，节点一启动就 SyntaxError。
下面 K 组会逐个文件检查「第 1 行是 shebang、第 2 行是编码声明」。
"""

from __future__ import division, print_function

import io
import os
import sys

try:
    text_type = unicode  # noqa: F821  # Python 2
except NameError:
    text_type = str  # Python 3

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PKG = os.path.join(ROOT, "src", "ground", "drive_cycle_energy")
SCRIPTS = os.path.join(PKG, "scripts")

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import energy_core as core  # noqa: E402

PASSED = []
FAILED = []

SPEC = core.VehicleSpec()
CYCLE = core.DEFAULT_CYCLE
RESULT = core.simulate(SPEC, CYCLE)
ASSERTS = core.check_energy(SPEC, RESULT)


def check(desc, expected, actual, tol=1e-9):
    ok = False
    if isinstance(expected, (list, tuple)) and isinstance(actual, (list, tuple)):
        if expected and isinstance(expected[0], text_type):
            ok = list(expected) == list(actual)
        else:
            ok = len(expected) == len(actual) and all(
                abs(a - b) <= tol for a, b in zip(expected, actual))
    elif isinstance(expected, bool) or isinstance(actual, bool):
        ok = bool(expected) == bool(actual)
    elif isinstance(expected, text_type) or isinstance(actual, text_type):
        ok = expected == actual
    else:
        ok = abs(float(expected) - float(actual)) <= max(
            tol, abs(float(expected)) * 1e-12)
    if ok:
        PASSED.append(desc)
        return True
    FAILED.append(u"%s（期望 %r，实际 %r）" % (desc, expected, actual))
    return False


def raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    except Exception as other:  # noqa: BLE001
        FAILED.append(u"应抛 %s，实际抛 %r" % (exc.__name__, other))
        return False
    FAILED.append(u"应抛 %s，实际没有抛" % (exc.__name__,))
    return False


def read(path):
    with io.open(path, "rb") as fh:
        return fh.read().decode("utf-8")


def read_first_lines(path, count=2):
    with io.open(path, "rb") as fh:
        text = fh.read().decode("utf-8")
    return text.splitlines()[:count]


# ---------------------------------------------------------------- A 参数 ----
def group_params():
    s = core.VehicleSpec()
    check(u"A1 质量", 1800.0, s.mass)
    check(u"A2 滚阻系数", 0.012, s.c_rr)
    check(u"A3 风阻系数", 0.31, s.cd)
    check(u"A4 迎风面积", 2.2, s.area)
    check(u"A5 电池容量", 60.0, s.battery_kwh)
    check(u"A6 回收效率", 0.6, s.regen_eff)
    check(u"A7 电池焦耳", 2.16e8, s.battery_joules())
    check(u"A8 质量必须为正", True,
          raises(core.EnergyError, core.VehicleSpec, mass=0.0))
    check(u"A9 电池必须为正", True,
          raises(core.EnergyError, core.VehicleSpec, battery_kwh=0.0))
    check(u"A10 风阻系数必须为正", True,
          raises(core.EnergyError, core.VehicleSpec, cd=0.0))
    check(u"A11 回收效率不能大于 1", True,
          raises(core.EnergyError, core.VehicleSpec, regen_eff=1.2))
    check(u"A12 回收效率可以为 0", 0.0, core.VehicleSpec(regen_eff=0.0).regen_eff)
    check(u"A13 滚阻系数可以为 0", 0.0, core.VehicleSpec(c_rr=0.0).c_rr)
    check(u"A14 滚阻系数不能为负", True,
          raises(core.EnergyError, core.VehicleSpec, c_rr=-0.01))
    check(u"A15 repr 含 mass", True, "mass=1800" in repr(s))


# ---------------------------------------------------------------- B 阻力 ----
def group_forces():
    s = core.VehicleSpec()
    check(u"B1 滚阻力恒定", 211.896, s.rolling_force())
    check(u"B2 10 m/s 风阻", 40.92, s.aero_force(10.0))
    check(u"B3 30 m/s 风阻", 368.28, s.aero_force(30.0))
    check(u"B4 0 m/s 风阻为零", 0.0, s.aero_force(0.0))
    check(u"B5 风阻按平方增长", 9.0, s.aero_force(30.0) / s.aero_force(10.0))
    check(u"B6 总阻力 = 滚阻 + 风阻", 252.816, s.resistance(10.0))
    check(u"B7 50 km/h 总阻力", 290.831185, s.resistance(50.0 / 3.6), tol=1e-5)
    check(u"B8 牵引力 = m·a + 阻力", 2090.831185,
          s.traction_force(50.0 / 3.6, 1.0), tol=1e-5)
    check(u"B9 匀减速时牵引力为负", True,
          s.traction_force(50.0 / 3.6, -2.0) < 0)
    check(u"B10 动能 50 km/h", 173611.1111, s.kinetic(50.0 / 3.6), tol=1e-3)
    check(u"B11 动能按平方增长", 4.0, s.kinetic(20.0) / s.kinetic(10.0))
    check(u"B12 静止动能为零", 0.0, s.kinetic(0.0))


# ------------------------------------------------------------ C 工况解析 ----
def group_parse():
    segs = core.parse_segments([(5.0, 0.0, 30.0), (10.0, 30.0, 30.0)])
    check(u"C1 段数", 2, len(segs))
    check(u"C2 时长保留", 5.0, segs[0][0])
    check(u"C3 km/h 转 m/s", 30.0 / 3.6, segs[0][2], tol=1e-12)
    check(u"C4 匀速段起止相同", segs[1][1], segs[1][2])
    check(u"C5 空工况报错", True,
          raises(core.EnergyError, core.parse_segments, []))
    check(u"C6 段长必须为三元素", True,
          raises(core.EnergyError, core.parse_segments, [(1.0, 2.0)]))
    check(u"C7 时长必须为正", True,
          raises(core.EnergyError, core.parse_segments, [(0.0, 0.0, 10.0)]))
    check(u"C8 车速不能为负", True,
          raises(core.EnergyError, core.parse_segments, [(1.0, -5.0, 10.0)]))
    check(u"C9 默认工况八段", 8, len(core.DEFAULT_CYCLE))
    check(u"C10 默认工况总时长", 60.0,
          sum(seg[0] for seg in core.DEFAULT_CYCLE))
    check(u"C11 默认工况起点静止", 0.0, core.DEFAULT_CYCLE[0][1])
    check(u"C12 默认工况终点 40 km/h", 40.0, core.DEFAULT_CYCLE[-1][2])


# ------------------------------------------------------------ D 工况积分 ----
def group_simulate():
    check(u"D1 总时长", 60.0, RESULT["duration_s"])
    check(u"D2 行驶距离", 512.5, RESULT["distance_m"], tol=1e-6)
    check(u"D3 最高车速 km/h", 50.0, RESULT["v_max"] * 3.6, tol=1e-9)
    check(u"D4 驱动能量", 407321.394176, RESULT["drive_j"], tol=1e-3)
    check(u"D5 制动能量", 159646.509903, RESULT["brake_j"], tol=1e-3)
    check(u"D6 回收能量 = η·制动", 0.6 * RESULT["brake_j"], RESULT["regen_j"])
    check(u"D7 滚阻能量", 108596.7, RESULT["roll_j"], tol=1e-3)
    check(u"D8 风阻能量", 27967.073162, RESULT["aero_j"], tol=1e-3)
    check(u"D9 动能净变化", 111111.1111, RESULT["ke_j"], tol=1e-3)
    check(u"D10 净耗电 = 驱动 − 回收", 311533.488234, RESULT["net_j"], tol=1e-3)
    check(u"D11 步长影响可忽略", RESULT["net_j"],
          core.simulate(SPEC, CYCLE, dt=0.02)["net_j"], tol=2.0)
    check(u"D12 静止工况无驱动能量", 0.0,
          core.simulate(SPEC, [(5.0, 0.0, 0.0)])["drive_j"])
    check(u"D13 静止工况无距离", 0.0,
          core.simulate(SPEC, [(5.0, 0.0, 0.0)])["distance_m"])
    check(u"D14 步长必须为正", True,
          raises(core.EnergyError, core.simulate, SPEC, CYCLE, 0.0))
    check(u"D15 减速越快制动能量越大", True,
          core.simulate(SPEC, [(5.0, 0.0, 20.0), (4.0, 20.0, 0.0)])["brake_j"]
          > core.simulate(SPEC, [(5.0, 0.0, 20.0), (8.0, 20.0, 0.0)])["brake_j"])
    check(u"D16 匀速工况驱动 = 阻力做功", 0.0,
          core.simulate(SPEC, [(10.0, 40.0, 40.0)])["ke_j"])


# ------------------------------------------------------------ E 四条断言 ----
def group_asserts():
    check(u"E1 断言条数", 4, len(ASSERTS))
    check(u"E2 断言名称", list(core.ASSERT_NAMES),
          [a["type"] for a in ASSERTS])
    check(u"E3 默认全部通过", True, all(a["passed"] for a in ASSERTS))
    check(u"E4 每条都有 detail", True,
          all(a["detail"] for a in ASSERTS))
    check(u"E5 detail 是文本", True,
          all(isinstance(a["detail"], text_type) for a in ASSERTS))
    bad = core.check_energy(SPEC, RESULT, regen_limit=0.5)
    check(u"E6 回收上限收紧后第一条失败", True,
          not [a for a in bad if a["type"] == core.ASSERT_NAMES[1]][0]["passed"])
    check(u"E7 其余三条仍然通过", 3,
          len([a for a in bad if a["passed"]]))
    bad2 = core.check_energy(SPEC, RESULT, breakdown_tolerance=-0.01)
    check(u"E8 分解容差不可满足时该条失败", True,
          not [a for a in bad2 if a["type"] == core.ASSERT_NAMES[0]][0]["passed"])
    check(u"E9 分解失败其余通过", 3, len([a for a in bad2 if a["passed"]]))
    bad3 = core.check_energy(SPEC, RESULT, aero_rising=False)
    check(u"E10 反向期望时风阻条失败", True,
          not [a for a in bad3 if a["type"] == core.ASSERT_NAMES[3]][0]["passed"])
    zero = core.simulate(SPEC, [(5.0, 0.0, 0.0)])
    a_zero = core.check_energy(SPEC, zero)
    check(u"E11 零驱动工况分解条失败", True,
          not [a for a in a_zero if a["type"] == core.ASSERT_NAMES[0]][0]["passed"])
    no_regen = core.simulate(core.VehicleSpec(regen_eff=0.0), CYCLE)
    a_nr = core.check_energy(core.VehicleSpec(regen_eff=0.0), no_regen)
    check(u"E12 无回收时回收比为零", 0.0,
          no_regen["regen_j"] / no_regen["brake_j"])
    check(u"E13 无回收仍然四条通过", 4,
          len([a for a in a_nr if a["passed"]]))
    check(u"E14 回收比与 η 一致", 0.6,
          RESULT["regen_j"] / RESULT["brake_j"], tol=1e-12)
    rich = core.simulate(core.VehicleSpec(regen_eff=0.95), CYCLE)
    check(u"E15 η=0.95 时回收比 0.95", 0.95,
          rich["regen_j"] / rich["brake_j"], tol=1e-12)
    check(u"E16 η=0.95 时回收条通过", True,
          all(a["passed"] for a in core.check_energy(
              core.VehicleSpec(regen_eff=0.95), rich)))
    check(u"E17 η=0.95 净耗低于默认", True, rich["net_j"] < RESULT["net_j"])
    heavy = core.VehicleSpec(mass=3000.0)
    r_heavy = core.simulate(heavy, CYCLE)
    check(u"E18 车更重则驱动能量更大", True,
          r_heavy["drive_j"] > RESULT["drive_j"])
    check(u"E19 车更重仍然自洽", True,
          [a for a in core.check_energy(heavy, r_heavy)
           if a["type"] == core.ASSERT_NAMES[0]][0]["passed"])
    slick = core.VehicleSpec(c_rr=0.006)
    r_slick = core.simulate(slick, CYCLE)
    check(u"E20 滚阻减半则滚阻能量约为一半", 0.5,
          r_slick["roll_j"] / RESULT["roll_j"], tol=1e-12)
    check(u"E21 滚阻减半续航更长", True,
          core.range_km(slick, r_slick) > core.range_km(SPEC, RESULT))
    check(u"E22 断言结果带 passed 布尔", True,
          all(isinstance(a["passed"], bool) for a in ASSERTS))


# ------------------------------------------------------------ F 恒等式 ----
def group_identity():
    total = (RESULT["roll_j"] + RESULT["aero_j"] + RESULT["brake_j"]
             + RESULT["ke_j"])
    check(u"F1 分解和等于驱动能量", RESULT["drive_j"], total, tol=1e-6)
    check(u"F2 驱动减制动等于净机械能", RESULT["drive_j"] - RESULT["brake_j"],
          RESULT["roll_j"] + RESULT["aero_j"] + RESULT["ke_j"], tol=1e-6)
    check(u"F3 动能等于 0.5·m·v²", 0.5 * 1800.0 * (40.0 / 3.6) ** 2,
          RESULT["ke_j"], tol=1e-6)
    check(u"F4 滚阻能量等于滚阻力乘距离", 211.896 * RESULT["distance_m"],
          RESULT["roll_j"], tol=1e-6)
    check(u"F5 风阻能量等于 ∫F·v", 27967.073162, RESULT["aero_j"], tol=1e-3)
    check(u"F6 净耗等于驱动减回收", RESULT["drive_j"] - RESULT["regen_j"],
          RESULT["net_j"], tol=1e-9)
    check(u"F7 回收是制动的 60%", 0.6 * RESULT["brake_j"],
          RESULT["regen_j"], tol=1e-9)
    check(u"F8 三项占比之和为 1", 1.0,
          sum(core.share(RESULT, k) for k in ("roll_j", "aero_j", "brake_j", "ke_j")),
          tol=1e-9)
    check(u"F9 滚阻占比", 0.266612, core.share(RESULT, "roll_j"), tol=1e-5)
    check(u"F10 风阻占比", 0.068661, core.share(RESULT, "aero_j"), tol=1e-5)
    check(u"F11 制动占比", 0.391933, core.share(RESULT, "brake_j"), tol=1e-5)
    check(u"F12 动能占比", 0.272785, core.share(RESULT, "ke_j"), tol=1e-5)
    check(u"F13 占比非法时抛错", True,
          raises(core.EnergyError, core.share,
                 {"drive_j": 0.0, "roll_j": 0.0}, "roll_j"))


# ------------------------------------------------------ G 续航与百公里 ----
def group_range():
    check(u"G1 百公里电耗", 16.885284, core.per_100km(RESULT), tol=1e-5)
    check(u"G2 满电续航", 355.339006, core.range_km(SPEC, RESULT), tol=1e-4)
    check(u"G3 续航 = 电池/百公里×100", 60.0 / 16.885284 * 100.0,
          core.range_km(SPEC, RESULT), tol=1e-3)
    hw = core.simulate(SPEC, [(20.0, 100.0, 100.0)])
    check(u"G4 高速匀速百公里电耗", 14.656576, core.per_100km(hw), tol=1e-5)
    check(u"G5 高速续航", 60.0 / core.per_100km(hw) * 100.0,
          core.range_km(SPEC, hw), tol=1e-9)
    check(u"G6 城市工况百公里高于高速", True,
          core.per_100km(RESULT) > core.per_100km(hw))
    check(u"G7 电池减半续航减半", 0.5,
          core.range_km(core.VehicleSpec(battery_kwh=30.0), RESULT)
          / core.range_km(SPEC, RESULT), tol=1e-9)
    check(u"G8 零距离无法折算", True,
          raises(core.EnergyError, core.per_100km, {"distance_m": 0.0,
                                                    "net_j": 0.0}))
    check(u"G9 净耗非正无法算续航", True,
          raises(core.EnergyError, core.range_km, SPEC,
                 {"distance_m": 100.0, "net_j": 0.0}))
    check(u"G10 无回收时续航变短", True,
          core.range_km(core.VehicleSpec(regen_eff=0.0),
                        core.simulate(core.VehicleSpec(regen_eff=0.0), CYCLE))
          < core.range_km(SPEC, RESULT))


# ------------------------------------------------------------ H 字符图 ----
def group_render():
    art = core.render_energy(RESULT)
    lines = art.splitlines()
    check(u"H1 行数 = 2 + 6 + 1", 9, len(lines))
    check(u"H2 含各项标签", True, all(k in art for k in (u"滚阻", u"风阻", u"制动",
                                                         u"动能", u"驱动", u"回收")))
    check(u"H3 驱动条最长（42 格）", 42,
          max(line.count("#") for line in lines))
    check(u"H4 含千焦单位", True, u"kJ" in art)
    check(u"H5 自定义宽度生效", True,
          core.render_energy(RESULT, width=10).splitlines()[1].count("-") == 10)
    check(u"H6 空能量不崩", True, u"kJ" in core.render_energy(
        {"roll_j": 0.0, "aero_j": 0.0, "brake_j": 0.0, "ke_j": 0.0,
         "drive_j": 0.0, "regen_j": 0.0}))


# ------------------------------------------------------------ I 报告 ----
def group_report():
    text = core.format_report(SPEC, RESULT, ASSERTS)
    check(u"I1 含工况行", True, u"工况：60.0 s" in text)
    check(u"I2 含整车行", True, u"电池 60.0 kWh" in text)
    check(u"I3 含分解行", True, u"分解：" in text)
    check(u"I4 含续航行", True, u"续航：百公里" in text)
    check(u"I5 四条 PASS", 4, text.count(u"[PASS]"))
    check(u"I6 结论 PASSED", True, u"结论：PASSED" in text)
    check(u"I7 断言统计行", True, u"4 通过 / 0 失败" in text)
    bad = core.check_energy(SPEC, RESULT, regen_limit=0.5)
    bad_text = core.format_report(SPEC, RESULT, bad)
    check(u"I8 失败时结论 FAILED", True, u"结论：FAILED" in bad_text)
    check(u"I9 失败时统计正确", True, u"3 通过 / 1 失败" in bad_text)


# ------------------------------------------------------------ J 参数对象 ----
def group_params_obj():
    p = core.EnergyParams()
    check(u"J1 默认质量", 1800.0, p.vehicle.mass)
    check(u"J2 默认回收效率", 0.6, p.vehicle.regen_eff)
    check(u"J3 默认分解容差", 0.02, p.breakdown_tolerance)
    check(u"J4 默认回收上限", 1.0, p.regen_limit)
    check(u"J5 默认步长", 0.05, p.dt)
    check(u"J6 默认工况段数", 8, len(p.cycle))
    q = core.EnergyParams(**{"vehicle/mass": 2400.0, "vehicle/regen_eff": 0.8,
                             "check/regen_limit": 0.9, "cycle/dt": 0.1})
    check(u"J7 覆盖质量", 2400.0, q.vehicle.mass)
    check(u"J8 覆盖回收", 0.8, q.vehicle.regen_eff)
    check(u"J9 覆盖上限", 0.9, q.regen_limit)
    check(u"J10 覆盖步长", 0.1, q.dt)


# --------------------------------------------------- K 工程文件约定 ----
def group_files():
    script_names = ("main.py", "energy_core.py", "energy_node.py")
    for name in script_names:
        path = os.path.join(SCRIPTS, name)
        exists = os.path.isfile(path)
        check(u"K1 %s 存在" % name, True, exists)
        if not exists:
            continue
        first = read_first_lines(path, 2)
        check(u"K2 %s 第 1 行是 shebang" % name, True,
              len(first) >= 1 and first[0].startswith("#!"))
        check(u"K3 %s 第 2 行是编码声明" % name, True,
              len(first) >= 2 and u"coding: utf-8" in first[1])
        check(u"K4 %s 含 __future__" % name, True,
              u"from __future__" in read(path))

    check(u"K5 core 不 import rospy", False,
          "import rospy" in read(os.path.join(SCRIPTS, "energy_core.py")))
    check(u"K6 core 不 import numpy", False,
          "import numpy" in read(os.path.join(SCRIPTS, "energy_core.py")))
    check(u"K7 node 含 _log 辅助", True,
          "def _log" in read(os.path.join(SCRIPTS, "energy_node.py")))
    check(u"K8 node 用 latched 发布", True,
          "latch=True" in read(os.path.join(SCRIPTS, "energy_node.py")))
    check(u"K9 yaml 存在", True,
          os.path.isfile(os.path.join(PKG, "config", "drive_cycle_energy.yaml")))
    check(u"K10 launch 存在", True,
          os.path.isfile(os.path.join(PKG, "launch", "main.launch")))
    launch = read(os.path.join(PKG, "launch", "main.launch"))
    check(u"K11 launch 用 main.py", True, 'type="main.py"' in launch)
    check(u"K12 launch 加载 yaml", True, "rosparam command=\"load\"" in launch)
    check(u"K13 launch 里 12 个 param", 12, launch.count("<param "))
    check(u"K14 launch 里 12 个 arg", 12, launch.count("<arg "))
    check(u"K15 文档页存在", True,
          os.path.isfile(os.path.join(ROOT, "docs", "ground", "drive_cycle_energy.md")))
    check(u"K16 README 存在", True, os.path.isfile(os.path.join(PKG, "README.md")))
    check(u"K17 package.xml 存在", True,
          os.path.isfile(os.path.join(PKG, "package.xml")))
    check(u"K18 CMakeLists 存在", True,
          os.path.isfile(os.path.join(PKG, "CMakeLists.txt")))
    check(u"K19 CMakeLists 安装三个脚本", 3,
          read(os.path.join(PKG, "CMakeLists.txt")).count("scripts/"))


# ------------------------------------------------------------ 主体 ----
GROUP_FUNCS = [
    (u"A VehicleSpec 参数", group_params),
    (u"B 阻力计算", group_forces),
    (u"C 工况解析", group_parse),
    (u"D 工况积分", group_simulate),
    (u"E 四条断言", group_asserts),
    (u"F 能量恒等式", group_identity),
    (u"G 续航与百公里能耗", group_range),
    (u"H 字符图", group_render),
    (u"I 报告文本", group_report),
    (u"J 参数对象", group_params_obj),
    (u"K 工程文件约定", group_files),
]


def _pytest_group(fn):
    def runner():
        before_fail = len(FAILED)
        fn()
        if len(FAILED) > before_fail:
            raise AssertionError(
                u"本组失败 %d 项：%s" % (len(FAILED) - before_fail,
                                       FAILED[-1]))
    runner.__name__ = fn.__name__
    return runner


def main():
    lines = []
    lines.append(u"drive_cycle_energy 离线回归测试")
    lines.append(u"Python %s" % sys.version.split()[0])
    lines.append(u"包目录 %s" % PKG)
    lines.append(u"-" * 60)
    for title, fn in GROUP_FUNCS:
        before = len(FAILED)
        fn()
        bad = len(FAILED) - before
        lines.append(u"%-22s %s" % (title, u"PASS" if bad == 0 else u"FAIL(%d)" % bad))
    lines.append(u"=" * 60)
    lines.append(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    for item in FAILED:
        lines.append(u"  [FAIL] %s" % item)
    lines.append(u"全部通过" if not FAILED else u"结论：FAILED")
    print(u"\n".join(lines))
    return 0 if not FAILED else 1


test_A_parameters = _pytest_group(group_params)
test_B_forces = _pytest_group(group_forces)
test_C_parse = _pytest_group(group_parse)
test_D_simulate = _pytest_group(group_simulate)
test_E_asserts = _pytest_group(group_asserts)
test_F_identity = _pytest_group(group_identity)
test_G_range = _pytest_group(group_range)
test_H_render = _pytest_group(group_render)
test_I_report = _pytest_group(group_report)
test_J_params = _pytest_group(group_params_obj)
test_K_files = _pytest_group(group_files)


if __name__ == "__main__":
    sys.exit(main())
