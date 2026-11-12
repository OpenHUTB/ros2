#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ride_comfort_transmissibility 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x
* 在仓库根目录执行：python3 tests/test_ride_comfort_transmissibility_local.py

覆盖范围：
A 传递率基本形   B 闭式峰位峰值   C 舒适等级   D 四条断言
E 扫描表         F 字符图         G 参数读取   H 报告排版
I 闭式核对       J 工程文件约定   K 编码声明回归
"""

from __future__ import division, print_function, unicode_literals

import io
import math
import os
import sys

try:  # Python 2 兼容
    text_type = unicode  # noqa: F821
except NameError:
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PKG = os.path.join(ROOT, "src", "ground", "ride_comfort_transmissibility")
SCRIPTS = os.path.join(PKG, "scripts")

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import ride_core as core  # noqa: E402

FAILS = []
OK = [0]


def out(text):
    if isinstance(text, text_type):
        sys.stdout.write(text + u"\n")
    else:
        sys.stdout.write(text + "\n")


def check(name, cond, detail=""):
    OK[0] += 1
    if cond:
        out(u"    ok   %s" % name)
    else:
        out(u"    FAIL %s  %s" % (name, detail))
        FAILS.append(name)


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


def raises(exc, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except exc:
        return True
    except Exception:
        return False
    return False


# ------------------------------------------------------------ A 传递率基本形
def group_shape():
    out(u"A 传递率基本形")
    spec = core.RideSpec()
    check(u"A1 零频传递率为 1",
          near(spec.transmissibility_at_ratio(1e-6), 1.0, 1e-6),
          repr(spec.transmissibility_at_ratio(1e-6)))
    check(u"A2 共振比处 T=2.236", near(spec.transmissibility_at_ratio(1.0), 2.236068, 1e-6))
    check(u"A3 半频比处 T=1.304", near(spec.transmissibility_at_ratio(0.5), 1.303840, 1e-6))
    check(u"A4 两倍频比处 T=0.447", near(spec.transmissibility_at_ratio(2.0), 0.447214, 1e-6))
    check(u"A5 十倍频比处 T=0.0514", near(spec.transmissibility_at_ratio(10.0), 0.051440, 1e-6))
    check(u"A6 峰前单调增",
          spec.transmissibility_at_ratio(0.3) < spec.transmissibility_at_ratio(0.6)
          < spec.transmissibility_at_ratio(0.9))
    check(u"A7 峰后单调减",
          spec.transmissibility_at_ratio(1.2) > spec.transmissibility_at_ratio(1.8)
          > spec.transmissibility_at_ratio(3.0))
    check(u"A8 峰值大于共振比处值",
          spec.peak_value() > spec.transmissibility_at_ratio(1.0))
    check(u"A9 频率比可换算", near(spec.ratio(2.4), 2.0, 1e-12))
    check(u"A10 传递率恒为正",
          all(spec.transmissibility_at_ratio(r) > 0 for r in core.DEFAULT_RATIOS))


# ------------------------------------------------------------ B 闭式峰位峰值
def group_peak():
    out(u"B 闭式峰位峰值")
    spec = core.RideSpec()
    check(u"B1 峰位闭式 sqrt(1-2ζ²)",
          near(spec.peak_ratio(), math.sqrt(1 - 2 * 0.25 ** 2), 1e-12))
    check(u"B2 峰位数值", near(spec.peak_ratio(), 0.935414, 1e-6))
    check(u"B3 峰值数值", near(spec.peak_value(), 2.280351, 1e-6))
    peak_r, peak_t = core.find_peak(spec)
    check(u"B4 扫描峰位", near(peak_r, 0.948625, 1e-6))
    check(u"B5 扫描峰值", near(peak_t, 2.283149, 1e-6))
    check(u"B6 扫描峰值与闭式相对偏差 < 1%",
          abs(peak_t - spec.peak_value()) / spec.peak_value() < 0.01)
    check(u"B7 扫描峰位与闭式偏差 < 容差",
          abs(peak_r - spec.peak_ratio()) < 0.02)
    check(u"B8 阻尼变大峰值变小",
          core.RideSpec(damping=0.4).peak_value() < spec.peak_value())
    check(u"B9 阻尼变小峰值变大",
          core.RideSpec(damping=0.1).peak_value() > spec.peak_value())
    check(u"B10 扫描区间非法被拒",
          raises(core.RideError, core.find_peak, spec, 0.0, 1.0, 10))


# ------------------------------------------------------------ C 舒适等级
def group_grade():
    out(u"C 舒适等级")
    check(u"C1 0.3 舒适", core.comfort_grade(0.3) == u"舒适", core.comfort_grade(0.3))
    check(u"C2 0.315 边界进下一档",
          core.comfort_grade(0.315) == u"略有不适")
    check(u"C3 0.5 略有不适", core.comfort_grade(0.5) == u"略有不适")
    check(u"C4 0.8 不适", core.comfort_grade(0.8) == u"不适")
    check(u"C5 1.2 很不适", core.comfort_grade(1.2) == u"很不适")
    check(u"C6 2.0 极不适", core.comfort_grade(2.0) == u"极不适")
    check(u"C7 3.0 无法忍受", core.comfort_grade(3.0) == u"无法忍受")
    check(u"C8 负值被拒", raises(core.RideError, core.comfort_grade, -0.1))
    check(u"C9 等级单调",
          core.comfort_grade(0.2) == u"舒适"
          and core.comfort_grade(2.4) == u"极不适")
    check(u"C10 默认工况等级",
          core.comfort_grade(core.RideSpec().weighted_accel(6.0)) == u"舒适")


# ------------------------------------------------------------ D 四条断言
def group_asserts():
    out(u"D 四条断言")
    spec = core.RideSpec()
    asserts = core.check_ride(spec)
    check(u"D1 断言条数为 4", len(asserts) == 4, repr(len(asserts)))
    check(u"D2 断言名齐全",
          tuple(a["type"] for a in asserts) == core.ASSERT_NAMES,
          repr([a["type"] for a in asserts]))
    check(u"D3 默认全部通过", all(a["passed"] for a in asserts),
          repr([a["type"] for a in asserts if not a["passed"]]))
    check(u"D4 每条有 detail",
          all(isinstance(a["detail"], (u"").__class__) and a["detail"]
              for a in asserts))
    check(u"D5 共振位置 detail 含扫描值", u"0.9486" in asserts[0]["detail"])
    check(u"D6 峰值 detail 含闭式值", u"2.2804" in asserts[1]["detail"])
    check(u"D7 高频 detail 含 10 倍频", u"r=10" in asserts[2]["detail"])
    check(u"D8 线性 detail 含加倍",
          u"加倍" in asserts[3]["detail"], asserts[3]["detail"])

    tight = core.check_ride(spec, pos_tolerance=0.001)
    flipped = [a["type"] for a in tight if not a["passed"]]
    check(u"D9 收紧位置容差只翻共振位置",
          flipped == [core.ASSERT_NAMES[0]], repr(flipped))

    strict = core.check_ride(spec, high_freq_limit=0.001)
    flipped = [a["type"] for a in strict if not a["passed"]]
    check(u"D10 收紧高频上限只翻高频衰减",
          flipped == [core.ASSERT_NAMES[2]], repr(flipped))

    soft = core.check_ride(core.RideSpec(damping=0.02))
    check(u"D11 软阻尼下四条仍全过", all(a["passed"] for a in soft))


# ------------------------------------------------------------ E 扫描表
def group_scan():
    out(u"E 扫描表")
    spec = core.RideSpec()
    rows = core.scan(spec, core.DEFAULT_RATIOS)
    check(u"E1 行数", len(rows) == 10, repr(len(rows)))
    check(u"E2 首行频率比", near(rows[0][0], 0.25))
    check(u"E3 末行频率比", near(rows[-1][0], 10.0))
    check(u"E4 频率换算", near(rows[3][1], 1.2, 1e-12))
    check(u"E5 共振比行传递率", near(rows[3][2], 2.236068, 1e-6))
    check(u"E6 加权等于传递率乘输入",
          all(near(r[3], r[2] * spec.input_accel, 1e-12) for r in rows))
    check(u"E7 峰值出现在 0.75~1.25 之间",
          rows[2][2] < rows[3][2] and rows[4][2] < rows[3][2])
    check(u"E8 非法频率比被拒",
          raises(core.RideError, core.scan, spec, (0.0,)))
    check(u"E9 自定义频率比可用",
          len(core.scan(spec, (1.0, 2.0, 3.0))) == 3)


# ------------------------------------------------------------ F 字符图
def group_render():
    out(u"F 字符图")
    art = core.render_transmissibility(core.RideSpec())
    lines = art.split("\n")
    check(u"F1 有标题", u"传递率曲线" in lines[0], lines[0])
    check(u"F2 有分隔线", lines[1].startswith("-") and lines[-1].startswith("-"))
    check(u"F3 数据行数 10", len(lines) == 13, repr(len(lines)))
    check(u"F4 共振行有峰值标记", any(u"<峰值" in ln for ln in lines))
    bars = [ln.count("#") for ln in lines[2:-1]]
    check(u"F5 条形最宽在峰值行", bars.index(max(bars)) == 3, repr(bars))
    check(u"F6 宽度可调",
          max(ln.count("#") for ln in
              core.render_transmissibility(core.RideSpec(), width=12).split("\n")) <= 12)


# ------------------------------------------------------------ G 参数读取
def group_params():
    out(u"G 参数读取")
    params = core.RideParams()
    check(u"G1 默认固有频率", near(params.spec.fn_hz, 1.2))
    check(u"G2 默认阻尼比", near(params.spec.damping, 0.25))
    check(u"G3 默认输入", near(params.spec.input_accel, 0.5))
    check(u"G4 默认激励频率", near(params.spec.road_freq_hz, 6.0))
    check(u"G5 默认位置容差", near(params.pos_tolerance, 0.02))
    check(u"G6 默认峰值容差", near(params.peak_tolerance, 0.05))
    check(u"G7 默认高频上限", near(params.high_freq_limit, 0.1))
    check(u"G8 默认线性容差", near(params.linear_tolerance, 0.02))

    custom = core.RideParams(**{
        "suspension/fn_hz": 1.6,
        "suspension/damping": 0.3,
        "road/input_accel": 0.8,
        "check/high_freq_limit": 0.2,
    })
    check(u"G9 固有频率可覆盖", near(custom.spec.fn_hz, 1.6))
    check(u"G10 阻尼可覆盖", near(custom.spec.damping, 0.3))
    check(u"G11 输入可覆盖", near(custom.spec.input_accel, 0.8))
    check(u"G12 阈值可覆盖", near(custom.high_freq_limit, 0.2))
    check(u"G13 未覆盖项保持默认", near(custom.spec.road_freq_hz, 6.0))
    check(u"G14 非法阻尼被拒",
          raises(core.RideError, core.RideSpec, damping=0.0))
    check(u"G15 过大阻尼被拒",
          raises(core.RideError, core.RideSpec, damping=0.75))
    check(u"G16 非法固有频率被拒",
          raises(core.RideError, core.RideSpec, fn_hz=-1.0))


# ------------------------------------------------------------ H 报告排版
def group_report():
    out(u"H 报告排版")
    spec = core.RideSpec()
    asserts = core.check_ride(spec)
    text = core.format_report(spec, asserts)
    check(u"H1 含悬架行", u"悬架：固有频率 1.200 Hz" in text)
    check(u"H2 含理论行", u"理论：峰位 r=0.9354，峰值 2.2804" in text)
    check(u"H3 含扫描行", u"扫描：峰位 r=0.9486" in text)
    check(u"H4 含舒适性行", u"等级「舒适」" in text)
    check(u"H5 含断言汇总", u"断言：4 通过 / 0 失败（共 4 条）" in text)
    check(u"H6 含结论 PASSED", u"结论：PASSED" in text)
    check(u"H7 每条断言都出现", all(a["type"] in text for a in asserts))
    rows = [line for line in text.split(u"\n")
            if line.strip().startswith((u"0.25", u"0.50", u"0.75", u"1.00",
                                        u"1.25", u"1.50", u"2.00", u"3.00",
                                        u"5.00", u"10.00"))]
    check(u"H8 表格有 10 行数据",
          len(rows) == 10 and u"10.00" in text and u"12.000" in text)

    bad = core.format_report(spec, core.check_ride(spec, pos_tolerance=0.001))
    check(u"H9 反例结论 FAILED", u"结论：FAILED" in bad)
    check(u"H10 反例含 1 失败", u"断言：3 通过 / 1 失败" in bad)


# ------------------------------------------------------------ I 闭式核对
def group_identity():
    out(u"I 闭式核对")
    spec = core.RideSpec()
    zeta = spec.damping
    # r=1 处的解析值：sqrt(1+(2ζ)²)/(2ζ)
    analytic = math.sqrt(1 + (2 * zeta) ** 2) / (2 * zeta)
    check(u"I1 共振比处解析值", near(spec.transmissibility_at_ratio(1.0), analytic, 1e-12))
    # 峰位处解析值（与 peak_value 同源，但用另一条式子写）
    r = spec.peak_ratio()
    alt = math.sqrt((1 + (2 * zeta * r) ** 2) /
                    ((1 - r * r) ** 2 + (2 * zeta * r) ** 2))
    check(u"I2 峰值与峰位处取值一致", near(alt, spec.peak_value(), 1e-12))
    # 高频渐近：ζ 很小时 T(10) ≈ 1/(100−1)
    tiny = core.RideSpec(damping=1e-4)
    check(u"I3 高频渐近 1/(r²-1)",
          near(tiny.transmissibility_at_ratio(10.0), 1.0 / 99.0, 2e-3),
          repr(tiny.transmissibility_at_ratio(10.0)))
    # 加权线性
    a1 = spec.weighted_accel(2.4)
    a2 = core.RideSpec(input_accel=1.0).weighted_accel(2.4)
    check(u"I4 输入加倍加权加倍", near(a2, 2 * a1, 1e-12))
    # 固有频率只改变频率轴，不改变传递率
    other = core.RideSpec(fn_hz=2.4)
    check(u"I5 固有频率不改变传递率",
          near(other.transmissibility(2.4), spec.transmissibility(1.2), 1e-12))
    check(u"I6 峰值与阻尼反相关",
          core.RideSpec(damping=0.05).peak_value()
          > core.RideSpec(damping=0.5).peak_value())


# ------------------------------------------------------------ J 工程文件约定
def group_files():
    out(u"J 工程文件约定")
    for rel in ("CMakeLists.txt", "package.xml", "README.md",
                "config/ride_comfort_transmissibility.yaml", "launch/main.launch",
                "scripts/main.py", "scripts/ride_core.py",
                "scripts/ride_node.py"):
        check(u"J %s 存在" % rel, os.path.isfile(os.path.join(PKG, rel)))

    launch = io.open(os.path.join(PKG, "launch", "main.launch"),
                     encoding="utf-8").read()
    check(u"J launch 有 rosparam load", u'<rosparam command="load"' in launch)
    check(u"J launch 有 node", u'type="main.py"' in launch)
    check(u"J launch 参数个数 >= 12", launch.count("<arg ") >= 12,
          repr(launch.count("<arg ")))

    node = io.open(os.path.join(PKG, "scripts", "ride_node.py"),
                   encoding="utf-8").read()
    check(u"J node 有 _log", u"def _log(" in node)
    check(u"J node 打中文前转 byte str", u".encode(" in node)
    check(u"J node 调 init_node", u"init_node(" in node)
    check(u"J node 有保活循环", u"is_shutdown" in node and u"rate.sleep" in node)

    main = io.open(os.path.join(PKG, "scripts", "main.py"),
                   encoding="utf-8").read()
    check(u"J main 转发到 node", u"ride_node" in main and u"main()" in main)

    readme = io.open(os.path.join(PKG, "README.md"), encoding="utf-8").read()
    check(u"J README 含公式", u"传递率" in readme)
    check(u"J README 含话题", u"话题" in readme)


# ------------------------------------------------------------ K 编码声明回归
def group_encoding():
    out(u"K 编码声明回归")
    for name in ("main.py", "ride_core.py", "ride_node.py"):
        path = os.path.join(PKG, "scripts", name)
        lines = io.open(path, encoding="utf-8").read().split("\n")
        check(u"K %s 第 1 行 shebang" % name,
              lines[0].startswith("#!") and "python" in lines[0], lines[0])
        check(u"K %s 第 2 行 coding" % name,
              "coding" in lines[1] and "utf-8" in lines[1], lines[1])


def main():
    out(u"ride_comfort_transmissibility 离线回归测试")
    out(u"Python %s" % sys.version.split()[0])
    out(u"包目录 %s" % PKG)
    out(u"-" * 60)
    group_shape()
    group_peak()
    group_grade()
    group_asserts()
    group_scan()
    group_render()
    group_params()
    group_report()
    group_identity()
    group_files()
    group_encoding()
    out(u"-" * 60)

    out(u"通过 %d 项，失败 %d 项" % (OK[0] - len(FAILS), len(FAILS)))
    if FAILS:
        out(u"失败项：")
        for name in FAILS:
            out(u"  - %s" % name)
        out(u"结论：FAILED")
        return 1
    out(u"全部通过")
    out(u"结论：PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
