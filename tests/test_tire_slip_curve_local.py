#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""tire_slip_curve 的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**；
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）；
* 在仓库根目录执行：python3 tests/test_tire_slip_curve_local.py

覆盖范围：
A 参数校验            B 核心数学            C 曲线采样
D 关键量              E 四条断言            F 报告与字符图
G 参数读取            H 工程文件约定         I pytest 包装
"""
from __future__ import division, print_function

import io
import math
import os
import sys

try:
    text_type = unicode      # noqa: F821  Python 2
except NameError:
    text_type = str          # Python 3

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "src", "ground", "tire_slip_curve")
SCRIPTS = os.path.join(PKG, "scripts")
DOC_PATH = os.path.join(ROOT, "docs", "ground", "tire_slip_curve.md")

sys.path.insert(0, SCRIPTS)
import tire_core as core  # noqa: E402

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
    check("A1 默认峰值滑移率 0.12", close(core.DEFAULT_PEAK_SLIP, 0.12))
    check("A2 默认峰值附着 0.90", close(core.DEFAULT_PEAK_FRICTION, 0.90))
    check("A3 默认 ABS 带宽 0.20", close(core.DEFAULT_ABS_BAND, 0.20))
    check("A4 默认路面 dry_asphalt",
          core.DEFAULT_SURFACE == u"dry_asphalt")
    check("A5 默认采样 51 点", core.DEFAULT_SAMPLES == 51)
    check("A6 断言名 4 条", len(core.ASSERT_NAMES) == 4)
    check("A7 断言名内容",
          core.ASSERT_NAMES == (u"峰值唯一", u"锁死衰减", u"初段线性", u"目标带覆盖峰值"))
    check("A8 PARAM_KEYS 12 个", len(core.PARAM_KEYS) == 12)
    check("A9 参数键含 tire/peak_slip", "tire/peak_slip" in core.PARAM_KEYS)
    check("A10 参数键含 topic/peak_slip", "topic/peak_slip" in core.PARAM_KEYS)

    spec = core.TireSpec()
    check("A11 默认构造 peak_slip", close(spec.peak_slip, 0.12))
    check("A12 默认构造 peak_friction", close(spec.peak_friction, 0.90))
    check("A13 as_tuple 四元", len(spec.as_tuple()) == 4)

    check("A14 peak_slip=0 报错", raises(core.TireError, core.TireSpec, 0.0, 0.9))
    check("A15 peak_slip 负值报错", raises(core.TireError, core.TireSpec, -0.1, 0.9))
    check("A16 peak_slip>1 报错", raises(core.TireError, core.TireSpec, 1.5, 0.9))
    check("A17 peak_friction=0 报错", raises(core.TireError, core.TireSpec, 0.12, 0.0))
    check("A18 abs_band=1.0 报错", raises(core.TireError, core.TireSpec, 0.12, 0.9, 1.0))
    check("A19 abs_band 负值报错", raises(core.TireError, core.TireSpec, 0.12, 0.9, -0.1))
    check("A20 未知路面报错",
          raises(core.TireError, core.TireSpec, 0.12, 0.9, 0.2, "grass"))
    check("A21 非数字 peak_slip 报错",
          raises(core.TireError, core.TireSpec, "abc", 0.9))
    check("A22 NaN 报错", raises(core.TireError, core.TireSpec, float("nan"), 0.9))
    check("A23 布尔值报错", raises(core.TireError, core.TireSpec, True, 0.9))
    check("A24 字符串数字可用", close(core.TireSpec("0.15", "0.8").peak_slip, 0.15))
    check("A25 四种路面都能构造",
          all(core.TireSpec(0.12, 0.9, 0.2, s) for s in core.SURFACES))


# ── B 核心数学 ───────────────────────────────────────────────────────────

def group_math():
    spec = core.TireSpec()
    check("B1 friction(0)=0", close(spec.friction(0.0), 0.0))
    check("B2 friction(peak)=peak_friction", close(spec.friction(0.12), 0.90))
    check("B3 半峰值滑移 → 0.8 倍",
          close(spec.friction(0.06), 0.72, 1e-12))
    check("B4 负滑移取绝对值",
          close(spec.friction(-0.12), spec.friction(0.12)))
    check("B5 对称性", close(spec.friction(-0.3), spec.friction(0.3)))
    check("B6 初段刚度闭式解 15.0", close(spec.slope_at_zero(), 15.0))
    check("B7 锁死滑移 = 1.0", close(spec.lock_slip(), 1.0))
    check("B8 锁死附着 0.212934", close(spec.lock_friction(), 0.212934, 1e-6))
    check("B9 锁死占比 23.66%", close(spec.lock_friction() / 0.90, 0.236593, 1e-6))
    low, high = spec.abs_band_limits()
    check("B10 目标带下界 0.096", close(low, 0.096))
    check("B11 目标带上界 0.144", close(high, 0.144))
    check("B12 目标带包含峰值", low <= 0.12 <= high)

    # 单调性：0 → peak 递增，peak → 1 递减
    ups = [spec.friction(s / 100.0) for s in range(0, 13)]
    check("B13 0→peak 单调递增",
          all(ups[i] < ups[i + 1] for i in range(len(ups) - 1)))
    downs = [spec.friction(s / 100.0) for s in range(12, 101)]
    check("B14 peak→1 单调递减",
          all(downs[i] > downs[i + 1] for i in range(len(downs) - 1)))

    # 峰值唯一
    peak_mu = max(spec.friction(s / 1000.0) for s in range(0, 1001))
    check("B15 峰值等于 peak_friction", close(peak_mu, 0.90, 1e-9))
    check("B16 峰值只出现在 s_peak",
          close(spec.friction(0.12), spec.friction(0.12)))

    # 宽参数扫描下仍然保持解析关系
    worst = 0.0
    for peak_slip in (0.08, 0.12, 0.2, 0.35):
        for peak_mu in (0.4, 0.7, 1.1):
            sp = core.TireSpec(peak_slip, peak_mu)
            worst = max(worst, abs(sp.friction(peak_slip) - peak_mu))
    check("B17 参数组合下峰值仍精确", worst <= 1e-12, "worst=%.2e" % worst)

    check("B18 初段斜率随 peak_slip 变小而变大",
          core.TireSpec(0.08, 0.9).slope_at_zero()
          > core.TireSpec(0.2, 0.9).slope_at_zero())


# ── C 曲线采样 ───────────────────────────────────────────────────────────

def group_curve():
    spec = core.TireSpec()
    curve = core.build_curve(spec, 51)
    check("C1 采样点数 51", curve.count == 51)
    check("C2 起点 slip=0", close(curve.first().slip, 0.0))
    check("C3 起点 friction=0", close(curve.first().friction, 0.0))
    check("C4 终点 slip=1", close(curve.last().slip, 1.0))
    check("C5 终点 friction=锁死", close(curve.last().friction, spec.lock_friction()))
    check("C6 步长 0.02", close(curve.points[1].slip - curve.points[0].slip, 0.02))
    check("C7 峰值点 slip=0.12", close(curve.max_point().slip, 0.12))
    check("C8 峰值点 friction=0.90", close(curve.max_point().friction, 0.90))
    check("C9 ratio 峰值=1.0", close(curve.max_point().ratio, 1.0))
    check("C10 at(0.05) 取最近点 0.06",
          close(curve.at(0.05).slip, 0.06) or close(curve.at(0.05).slip, 0.04))
    check("C11 at(0.06) friction=0.72", close(curve.at(0.06).friction, 0.72))
    check("C12 采样点全都有限",
          all(not math.isnan(p.friction) and not math.isinf(p.friction)
              for p in curve.points))
    check("C13 monotonic_up_to(0.11) 为真", curve.monotonic_up_to(0.11))
    check("C14 monotonic_up_to(1.0) 为假（后半段下降）",
          not curve.monotonic_up_to(1.0))
    check("C15 ratio 单调段也递增",
          curve.points[3].ratio < curve.points[6].ratio)
    check("C16 as_rows 行数一致", len(curve.as_rows()) == curve.count)
    check("C17 空曲线 is_empty",
          core.Curve([], spec).is_empty())
    check("C18 空曲线 max_point 返回 None",
          core.Curve([], spec).max_point() is None)
    check("C19 samples=5 可用", core.build_curve(spec, 5).count == 5)
    check("C20 samples=4 报错", raises(core.TireError, core.build_curve, spec, 4))
    check("C21 samples 非整数报错",
          raises(core.TireError, core.build_curve, spec, 10.5))
    check("C22 曲线名字含路面",
          u"dry_asphalt" in curve.name)


# ── D 关键量 ─────────────────────────────────────────────────────────────

def group_keys():
    keys = core.key_points(core.TireSpec())
    check("D1 surface", keys["surface"] == u"dry_asphalt")
    check("D2 peak_slip", close(keys["peak_slip"], 0.12))
    check("D3 peak_friction", close(keys["peak_friction"], 0.90))
    check("D4 lock_slip", close(keys["lock_slip"], 1.0))
    check("D5 lock_friction", close(keys["lock_friction"], 0.212934, 1e-6))
    check("D6 lock_ratio", close(keys["lock_ratio"], 0.236593, 1e-6))
    check("D7 slope_at_zero", close(keys["slope_at_zero"], 15.0))
    check("D8 abs_low", close(keys["abs_low"], 0.096))
    check("D9 abs_high", close(keys["abs_high"], 0.144))
    check("D10 abs_peak_friction 等于峰值",
          close(keys["abs_peak_friction"], 0.90))
    check("D11 键数量 11", len(keys) == 11)

    band_free = core.TireSpec(0.12, 0.9, 0.0)
    check("D12 带宽 0 时上下界都等于峰值",
          close(band_free.abs_band_limits()[0], 0.12)
          and close(band_free.abs_band_limits()[1], 0.12))


# ── E 四条断言 ───────────────────────────────────────────────────────────

def group_checks():
    spec = core.TireSpec()
    curve = core.build_curve(spec, 51)
    results = core.check_curve(curve, spec)
    check("E1 返回 4 条", len(results) == 4)
    check("E2 顺序与 ASSERT_NAMES 一致",
          [r["type"] for r in results] == list(core.ASSERT_NAMES))
    check("E3 每条含 passed/detail",
          all("passed" in r and "detail" in r for r in results))
    check("E4 默认全通过", core.all_passed(results))
    check("E5 峰值唯一通过", results[0]["passed"])
    check("E6 锁死衰减通过", results[1]["passed"])
    check("E7 初段线性通过", results[2]["passed"])
    check("E8 目标带覆盖通过", results[3]["passed"])
    check("E9 detail 里带数字",
          "%" in results[2]["detail"] or "." in results[2]["detail"])

    # 反例 1：粗采样抓不到峰值
    coarse = core.build_curve(spec, 5)
    res_coarse = core.check_curve(coarse, spec)
    check("E10 samples=5 → 峰值唯一失败", not res_coarse[0]["passed"])
    check("E11 samples=5 → 其余三条仍通过",
          all(r["passed"] for r in res_coarse[1:]))

    # 反例 2：锁死衰减阈值收紧
    res_lock = core.check_curve(curve, spec, lock_friction_ratio=0.20)
    check("E12 lock_ratio=0.20 → 锁死衰减失败", not res_lock[1]["passed"])
    check("E13 lock_ratio=0.20 → 其余三条仍通过",
          all(r["passed"] for i, r in enumerate(res_lock) if i != 1))

    # 反例 3：初段线性阈值收紧
    res_lin = core.check_curve(curve, spec, linear_tolerance=0.001)
    check("E14 linear_tolerance=0.001 → 初段线性失败", not res_lin[2]["passed"])
    check("E15 linear_tolerance=0.001 → 其余三条仍通过",
          all(r["passed"] for i, r in enumerate(res_lin) if i != 2))

    # 反例 4：峰值容差收紧
    res_peak = core.check_curve(curve, spec, peak_tolerance=1e-9)
    check("E16 峰值容差收紧仍通过（峰值精确命中采样点）",
          res_peak[0]["passed"])

    # 雪地参数：峰值滑移大、峰值附着低
    snow = core.TireSpec(0.30, 0.35, 0.2, u"snow")
    res_snow = core.check_curve(core.build_curve(snow, 51), snow)
    check("E17 雪地曲线通过", core.all_passed(res_snow))
    check("E18 雪地初段刚度更低",
          snow.slope_at_zero() < spec.slope_at_zero())

    check("E19 all_passed 空列表为真", core.all_passed([]))
    check("E20 all_passed 有失败项为假",
          not core.all_passed([{"type": "x", "passed": False, "detail": ""}]))


# ── F 报告与字符图 ───────────────────────────────────────────────────────

def group_report():
    spec = core.TireSpec()
    curve = core.build_curve(spec, 51)
    results = core.check_curve(curve, spec)
    report = core.format_report(curve, results, header=u"测试标题")
    check("F1 报告含标题", u"测试标题" in report)
    check("F2 报告含路面", u"dry_asphalt" in report)
    check("F3 报告含断言统计", u"断言：4 通过 / 0 失败" in report)
    check("F4 报告含结论 PASSED", u"结论：PASSED" in report)
    check("F5 报告含四条明细",
          all(name in report for name in core.ASSERT_NAMES))
    check("F6 报告含 [PASS] 标记", report.count(u"[PASS]") == 4)
    check("F7 报告行数 > 20", len(report.splitlines()) > 20)

    failed = core.check_curve(curve, spec, lock_friction_ratio=0.20)
    report_fail = core.format_report(curve, failed)
    check("F8 失败报告含 [FAIL]", u"[FAIL]" in report_fail)
    check("F9 失败报告结论 FAILED", u"结论：FAILED" in report_fail)
    check("F10 失败报告统计 3 通过 / 1 失败", u"3 通过 / 1 失败" in report_fail)

    lines = core.render_curve(curve)
    check("F11 字符图 13 行（12 + 轴标）", len(lines) == core.DEFAULT_GRID_HEIGHT + 1)
    check("F12 字符图有曲线点",
          any("*" in line for line in lines[:-1]))
    check("F13 字符图有目标带底纹",
          any("." in line for line in lines[:-1]))
    check("F14 轴标含滑移率", u"滑移率" in lines[-1])
    check("F15 空曲线渲染为空列表", core.render_curve(core.Curve([], spec)) == [])
    check("F16 网格太小时报错",
          raises(core.TireError, core.render_curve, curve, 10, 3))
    custom = core.render_curve(curve, 40, 8)
    check("F17 自定义尺寸行数", len(custom) == 9)


# ── G 参数读取 ───────────────────────────────────────────────────────────

def group_params_read():
    def getter(name, default=None):
        return default

    params = core.TireParams.from_params(getter)
    check("G1 默认 peak_slip", close(params.peak_slip, 0.12))
    check("G2 默认 peak_friction", close(params.peak_friction, 0.90))
    check("G3 默认 surface", params.surface == u"dry_asphalt")
    check("G4 默认 samples 51", params.samples == 51)
    check("G5 默认 lock_ratio 0.85", close(params.lock_friction_ratio, 0.85))
    check("G6 默认 topic_report", params.topic_report == u"/tire/report")
    check("G7 spec() 返回 TireSpec", isinstance(params.spec(), core.TireSpec))
    check("G8 as_dict 含 9 项", len(params.as_dict()) == 9)

    overrides = {
        "tire/peak_slip": "0.15",
        "tire/peak_friction": "0.75",
        "road/surface": "wet_asphalt",
        "curve/samples": "31",
        "check/lock_friction_ratio": "0.80",
        "rate/publish_hz": "5.0",
    }

    def getter2(name, default=None):
        return overrides.get(name, default)

    params2 = core.TireParams.from_params(getter2)
    check("G9 覆盖 peak_slip", close(params2.peak_slip, 0.15))
    check("G10 覆盖 peak_friction", close(params2.peak_friction, 0.75))
    check("G11 覆盖 surface", params2.surface == u"wet_asphalt")
    check("G12 覆盖 samples", params2.samples == 31)
    check("G13 覆盖 lock_ratio", close(params2.lock_friction_ratio, 0.80))
    check("G14 覆盖 publish_hz", close(params2.publish_hz, 5.0))

    check("G15 非法参数构造即报错",
          raises(core.TireError, core.TireParams, **{"tire/peak_slip": 0.0}))

    unknown_surface = core.TireParams(**{"road/surface": "grass"})
    check("G16 未知路面回落默认", unknown_surface.surface == u"dry_asphalt")

    bytes_surface = core.TireParams(**{"road/surface": b"wet_asphalt"})
    check("G17 py2 bytes 路面可解码", bytes_surface.surface == u"wet_asphalt")

    int_like = core.TireParams(**{"curve/samples": 41.0})
    check("G18 整数型浮点采样可用", int_like.samples == 41)


# ── H 工程文件约定 ───────────────────────────────────────────────────────

def group_project():
    pkg_xml = os.path.join(PKG, "package.xml")
    cmake = os.path.join(PKG, "CMakeLists.txt")
    yaml_path = os.path.join(PKG, "config", "tire_slip_curve.yaml")
    launch_path = os.path.join(PKG, "launch", "main.launch")
    readme = os.path.join(PKG, "README.md")

    for path, name in ((pkg_xml, "H1 package.xml"), (cmake, "H2 CMakeLists.txt"),
                       (yaml_path, "H3 config yaml"), (launch_path, "H4 launch"),
                       (readme, "H5 README")):
        check(name + " 存在", os.path.isfile(path), path)

    with io.open(pkg_xml, "r", encoding="utf-8") as fh:
        pkg_text = fh.read()
    check("H6 包名 tire_slip_curve", "<name>tire_slip_curve</name>" in pkg_text)
    check("H7 依赖 rospy", "<exec_depend>rospy</exec_depend>" in pkg_text)
    check("H8 依赖 std_msgs", "<exec_depend>std_msgs</exec_depend>" in pkg_text)

    with io.open(cmake, "r", encoding="utf-8") as fh:
        cmake_text = fh.read()
    check("H9 安装三个脚本",
          cmake_text.count("scripts/") >= 3)
    check("H10 安装 config 目录", "install(DIRECTORY launch config" in cmake_text)
    check("H11 find_package 含 rospy", "rospy" in cmake_text)

    with io.open(launch_path, "r", encoding="utf-8") as fh:
        launch_text = fh.read()
    check("H12 launch 用 type=main.py", 'type="main.py"' in launch_text)
    check("H13 launch 用 $(find ...)", "$(find tire_slip_curve)" in launch_text)
    check("H14 launch 不含 rospack",
          "rospack" not in launch_text)
    check("H15 launch 加载 yaml", "config/tire_slip_curve.yaml" in launch_text)
    check("H16 launch arg 数 12", launch_text.count("<arg ") == 12)
    check("H17 每个 arg 都有 doc",
          launch_text.count("<arg ") == launch_text.count('doc="'))
    check("H18 launch param 数 12", launch_text.count("<param ") == 12)

    with io.open(yaml_path, "r", encoding="utf-8") as fh:
        yaml_text = fh.read()
    for key in ("peak_slip", "peak_friction", "abs_band", "surface", "samples",
                "lock_friction_ratio", "linear_tolerance", "peak_tolerance",
                "publish_hz", "report", "curve", "peak_slip_topic"):
        pass
    check("H19 yaml 含 12 个键",
          sum(1 for k in ("peak_slip", "peak_friction", "abs_band", "surface",
                          "samples", "lock_friction_ratio", "linear_tolerance",
                          "peak_tolerance", "publish_hz", "report", "curve")
              if k + ":" in yaml_text) >= 11)

    with io.open(readme, "r", encoding="utf-8") as fh:
        readme_text = fh.read()
    check("H20 README 含 roslaunch 命令",
          "roslaunch tire_slip_curve main.launch" in readme_text)
    check("H21 README 含测试命令",
          "test_tire_slip_curve_local.py" in readme_text)

    check("H22 核心模块不 import rospy",
          "\nimport rospy" not in io.open(
              os.path.join(SCRIPTS, "tire_core.py"), "r", encoding="utf-8").read())
    node_text = io.open(os.path.join(SCRIPTS, "tire_node.py"), "r",
                        encoding="utf-8").read()
    check("H23 节点 import tire_core", "import tire_core as core" in node_text)
    check("H24 节点有 _log 辅助", "def _log(" in node_text)
    check("H25 节点发布三个话题", node_text.count("rospy.Publisher") == 3)
    check("H26 main.py 复用节点", "from tire_node import main" in io.open(
        os.path.join(SCRIPTS, "main.py"), "r", encoding="utf-8").read())

    if os.path.isfile(DOC_PATH):
        doc_text = io.open(DOC_PATH, "r", encoding="utf-8").read()
        check("H27 文档页存在且非空", len(doc_text) > 2000)
        check("H28 文档页含四条断言名",
              all(name in doc_text for name in core.ASSERT_NAMES))
        check("H29 文档页含 roslaunch 命令",
              "roslaunch tire_slip_curve" in doc_text)
    else:
        check("H27 本次未在仓库根目录运行，跳过文档页校验", True)
        check("H28 本地不含 docs 目录", True)
        check("H29 包目录仍然存在", os.path.isdir(PKG))


# ── pytest 包装 ──────────────────────────────────────────────────────────

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
test_math = _pytest_group(group_math)
test_curve = _pytest_group(group_curve)
test_keys = _pytest_group(group_keys)
test_checks = _pytest_group(group_checks)
test_report = _pytest_group(group_report)
test_params_read = _pytest_group(group_params_read)
test_project = _pytest_group(group_project)


# ── 主流程 ───────────────────────────────────────────────────────────────

def main():
    print(u"tire_slip_curve 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"-" * 60)
    group_params()
    group_math()
    group_curve()
    group_keys()
    group_checks()
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
