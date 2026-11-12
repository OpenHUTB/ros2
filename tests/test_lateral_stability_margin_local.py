#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""lateral_stability_margin 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_lateral_stability_margin_local.py

覆盖范围：
A 几何与轴荷       B 梯度与转向特性   C 稳态圆周
D 四条断言         E 车速扫描         F 字符图
G 参数读取         H 报告排版         I 闭式核对
J 工程文件约定     K 编码声明回归
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

PKG = os.path.join(ROOT, "src", "ground", "lateral_stability_margin")
SCRIPTS = os.path.join(PKG, "scripts")

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import stability_core as core  # noqa: E402

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


# ---------------------------------------------------------------- A 几何与轴荷
def group_geometry():
    out(u"A 几何与轴荷")
    spec = core.VehicleSpec()
    check(u"A1 质心到后轴", near(spec.cg_to_rear(), 1.5), repr(spec.cg_to_rear()))
    check(u"A2 前轴载荷", near(spec.front_load(), 8175.0, 1e-6), repr(spec.front_load()))
    check(u"A3 后轴载荷", near(spec.rear_load(), 6540.0, 1e-6), repr(spec.rear_load()))
    check(u"A4 轴荷之和等于整车重力", near(spec.front_load() + spec.rear_load(),
                                          spec.total_load(), 1e-9))
    check(u"A5 前轴载荷大于后轴", spec.front_load() > spec.rear_load())
    check(u"A6 轴荷比等于力臂反比",
          near(spec.front_load() / spec.rear_load(),
               spec.cg_to_rear() / spec.cg_to_front, 1e-12))

    heavy = core.VehicleSpec(mass=2000.0)
    check(u"A7 质量放大后轴荷比不变",
          near(heavy.front_load() / heavy.rear_load(),
               spec.front_load() / spec.rear_load(), 1e-12))

    forward = core.VehicleSpec(cg_to_front=1.5)
    check(u"A8 质心后移前轴变轻", forward.front_load() < spec.front_load())
    check(u"A9 质心后移后轴变重", forward.rear_load() > spec.rear_load())
    check(u"A10 非法轴距被拒", raises(core.StabilityError, core.VehicleSpec, wheelbase=0.0))
    check(u"A11 质心超轴距被拒",
          raises(core.StabilityError, core.VehicleSpec, cg_to_front=3.0))
    check(u"A12 刚度为负被拒",
          raises(core.StabilityError, core.VehicleSpec, front_cornering=-1.0))


# ------------------------------------------------------------ B 梯度与转向特性
def group_gradient():
    out(u"B 梯度与转向特性")
    spec = core.VehicleSpec()
    k = spec.gradient()
    manual = (spec.front_load() / spec.front_cornering - spec.rear_load() / spec.rear_cornering) / core.GRAVITY
    check(u"B1 梯度闭式一致", near(k, manual, 1e-12), repr(k))
    check(u"B2 默认车不足转向", k > 0, repr(k))
    check(u"B3 特性名不足转向", spec.character() == u"不足转向", spec.character())
    vchar = spec.characteristic_speed()
    check(u"B4 特征车速闭式", near(vchar, math.sqrt(spec.wheelbase / k), 1e-12))
    check(u"B5 特征车速数值", near(vchar, 28.1837, 1e-3), repr(vchar))
    check(u"B6 不足转向无临界车速", spec.critical_speed() is None)

    over = core.VehicleSpec(front_cornering=160000.0, rear_cornering=40000.0)
    check(u"B7 硬前软后过多转向", over.gradient() < 0, repr(over.gradient()))
    check(u"B8 特性名过多转向", over.character() == u"过多转向", over.character())
    vcrit = over.critical_speed()
    check(u"B9 临界车速闭式",
          near(vcrit, math.sqrt(-over.wheelbase / over.gradient()), 1e-12))
    check(u"B10 临界车速数值", near(vcrit, 15.3504, 1e-3), repr(vcrit))
    check(u"B11 过多转向无特征车速", over.characteristic_speed() is None)

    soft = core.VehicleSpec(front_cornering=110000.0)
    check(u"B12 前轮变软梯度变小", soft.gradient() < k, repr(soft.gradient()))
    check(u"B13 前轮变软特征车速变大", soft.characteristic_speed() > vchar)


# ---------------------------------------------------------------- C 稳态圆周
def group_cornering():
    out(u"C 稳态圆周")
    spec = core.VehicleSpec()
    corner = core.Cornering(spec)
    check(u"C1 车速换算",
          near(corner.speed(), 40.0 / 3.6, 1e-12), repr(corner.speed()))
    check(u"C2 侧向加速度", near(corner.lateral_accel(), 2.469136, 1e-6))
    check(u"C3 阿克曼角", near(corner.ackermann_angle(), 0.054, 1e-12))
    check(u"C4 稳态转向角", near(corner.steer_angle(), 0.0623929, 1e-6))
    check(u"C5 转向角大于阿克曼角",
          corner.steer_angle() > corner.ackermann_angle())
    check(u"C6 超出量等于 K·a_y",
          near(corner.deflection(), spec.gradient() * corner.lateral_accel(), 1e-12))
    check(u"C7 转向灵敏度", near(corner.sensitivity(), 39.5740, 1e-3))

    slow = core.Cornering(spec, speed_kmh=2.0)
    check(u"C8 低速退化为阿克曼",
          near(slow.steer_angle(), slow.ackermann_angle(), 6e-4))

    fast = core.Cornering(spec, speed_kmh=120.0)
    check(u"C9 高速转向角显著变大", fast.steer_angle() > 2 * slow.steer_angle())

    over = core.VehicleSpec(front_cornering=160000.0, rear_cornering=40000.0)
    diverged = core.Cornering(over, radius=50.0, speed_kmh=100.0)
    check(u"C10 过临界灵敏度退化", diverged.sensitivity() is None)
    check(u"C11 非法半径被拒",
          raises(core.StabilityError, core.Cornering, spec, radius=0.0))
    check(u"C12 缺整车参数被拒", raises(core.StabilityError, core.Cornering, None))


# ---------------------------------------------------------------- D 四条断言
def group_asserts():
    out(u"D 四条断言")
    spec = core.VehicleSpec()
    corner = core.Cornering(spec)
    asserts = core.check_stability(corner)
    check(u"D1 断言条数是 4", len(asserts) == 4, repr(len(asserts)))
    check(u"D2 断言名齐全",
          tuple(a["type"] for a in asserts) == core.ASSERT_NAMES,
          repr([a["type"] for a in asserts]))
    check(u"D3 默认全部通过", all(a["passed"] for a in asserts),
          repr([a["type"] for a in asserts if not a["passed"]]))
    check(u"D4 每条都有 detail",
          all(isinstance(a["detail"], (u"").__class__) and a["detail"]
              for a in asserts))
    check(u"D5 轴荷 detail 含数字", u"8175.0" in asserts[0]["detail"])

    over = core.check_stability(core.Cornering(
        core.VehicleSpec(front_cornering=160000.0, rear_cornering=40000.0),
        speed_kmh=20.0))
    flipped = [a["type"] for a in over if not a["passed"]]
    check(u"D6 过多转向只翻不足转向裕度",
          flipped == [core.ASSERT_NAMES[1]], repr(flipped))

    tight = core.check_stability(corner, ackermann_speed_kmh=60.0)
    flipped = [a["type"] for a in tight if not a["passed"]]
    check(u"D7 高速阿克曼只翻低速阿克曼",
          flipped == [core.ASSERT_NAMES[3]], repr(flipped))

    high = core.check_stability(corner, understeer_min=0.1)
    flipped = [a["type"] for a in high if not a["passed"]]
    check(u"D8 阈值抬高翻不足转向裕度",
          core.ASSERT_NAMES[1] in flipped, repr(flipped))


# ---------------------------------------------------------------- E 车速扫描
def group_scan():
    out(u"E 车速扫描")
    spec = core.VehicleSpec()
    rows = core.speed_table(spec)
    check(u"E1 扫描行数", len(rows) == 6, repr(len(rows)))
    check(u"E2 首行车速", near(rows[0][0], 20.0))
    check(u"E3 末行车速", near(rows[-1][0], 120.0))
    deltas = [r[2] for r in rows]
    check(u"E4 转向角单调增大",
          all(deltas[i] < deltas[i + 1] for i in range(len(deltas) - 1)))
    check(u"E5 40 km/h 行与工况一致",
          near(rows[1][2], core.Cornering(spec).steer_angle(), 1e-12))
    sens = [r[4] for r in rows]
    check(u"E6 不足转向灵敏度递增",
          all(sens[i] < sens[i + 1] for i in range(len(sens) - 1)))
    less = core.speed_table(spec, radius=80.0)
    check(u"E7 半径变大转向角变小", less[0][2] < rows[0][2])
    over = core.VehicleSpec(front_cornering=160000.0, rear_cornering=40000.0)
    over_rows = core.speed_table(over)
    check(u"E8 过多转向高速发散",
          any(r[4] is None for r in over_rows), repr([r[4] for r in over_rows]))


# ---------------------------------------------------------------- F 字符图
def group_render():
    out(u"F 字符图")
    spec = core.VehicleSpec()
    art = core.render_speed_table(spec)
    lines = art.split("\n")
    check(u"F1 有标题行", u"转向角随车速" in lines[0], lines[0])
    check(u"F2 有分隔线", lines[1].startswith("-") and lines[-1].startswith("-"))
    check(u"F3 图内有 6 行数据", len(lines) == 9, repr(len(lines)))
    bars = [ln.count("#") for ln in lines[2:-1]]
    check(u"F4 条形单调变宽",
          all(bars[i] < bars[i + 1] for i in range(len(bars) - 1)), repr(bars))
    narrow = core.render_speed_table(spec, width=8)
    check(u"F5 宽度可调",
          max(ln.count("#") for ln in narrow.split("\n")) <= 8)
    check(u"F6 含车速单位", u"km/h" in art)


# ---------------------------------------------------------------- G 参数读取
def group_params():
    out(u"G 参数读取")
    params = core.StabilityParams()
    check(u"G1 默认质量", near(params.spec.mass, 1500.0))
    check(u"G2 默认半径", near(params.radius, 50.0))
    check(u"G3 默认车速", near(params.speed_kmh, 40.0))
    check(u"G4 默认下限", near(params.understeer_min, 0.0))
    check(u"G5 默认阿克曼车速", near(params.ackermann_speed_kmh, 2.0))
    check(u"G6 默认容差", near(params.ackermann_tolerance, 0.02))
    check(u"G7 默认扫描车速", params.speeds == core.DEFAULT_SPEEDS_KMH)

    custom = core.StabilityParams(**{
        "vehicle/mass": 1800.0,
        "tire/rear_cornering": 70000.0,
        "scene/radius": 80.0,
        "check/understeer_min": 0.02,
    })
    check(u"G8 质量可覆盖", near(custom.spec.mass, 1800.0))
    check(u"G9 刚度可覆盖", near(custom.spec.rear_cornering, 70000.0))
    check(u"G10 半径可覆盖", near(custom.radius, 80.0))
    check(u"G11 下限可覆盖", near(custom.understeer_min, 0.02))
    check(u"G12 未覆盖项保持默认", near(custom.spec.front_cornering, 80000.0))
    check(u"G13 非法质量被拒",
          raises(core.StabilityError, core.VehicleSpec, mass=-1.0))
    check(u"G14 非法阻尼被拒",
          raises(core.StabilityError, core.VehicleSpec, rear_cornering=0.0))


# ---------------------------------------------------------------- H 报告排版
def group_report():
    out(u"H 报告排版")
    spec = core.VehicleSpec()
    corner = core.Cornering(spec)
    asserts = core.check_stability(corner)
    text = core.format_report(spec, corner, asserts)
    check(u"H1 含整车行", u"整车：质量 1500.0 kg" in text)
    check(u"H2 含轴荷行", u"轴荷：前 8175.0 N" in text)
    check(u"H3 含梯度行", u"K = 0.003399" in text)
    check(u"H4 含特征车速行", u"特征车速 28.184 m/s（101.5 km/h）" in text)
    check(u"H5 含工况行", u"δ 0.062393 rad（3.57°）" in text)
    check(u"H6 含断言汇总", u"断言：4 通过 / 0 失败（共 4 条）" in text)
    check(u"H7 含结论 PASSED", u"结论：PASSED" in text)
    check(u"H8 每条断言都出现",
          all(a["type"] in text for a in asserts))

    over = core.VehicleSpec(front_cornering=160000.0, rear_cornering=40000.0)
    over_corner = core.Cornering(over, speed_kmh=20.0)
    over_asserts = core.check_stability(over_corner)
    bad = core.format_report(over, over_corner, over_asserts)
    check(u"H9 反例结论 FAILED", u"结论：FAILED" in bad)
    check(u"H10 反例含临界车速行", u"临界车速 15.350 m/s（55.3 km/h）" in bad)


# ---------------------------------------------------------------- I 闭式核对
def group_identity():
    out(u"I 闭式核对")
    spec = core.VehicleSpec()
    corner = core.Cornering(spec)
    k = spec.gradient()
    # δ = L/R + K·a_y  ⇒  (δ − L/R)/K = a_y
    back = (corner.steer_angle() - spec.wheelbase / corner.radius) / k
    check(u"I1 稳态转向角反解", near(back, corner.lateral_accel(), 1e-12))
    # a_y/δ 与 v²/(L + K·v²) 两种写法
    v = corner.speed()
    alt = v * v / (spec.wheelbase + k * v * v)
    check(u"I2 灵敏度闭式一致", near(alt, corner.sensitivity(), 1e-12))
    # 转向角（°）换算
    check(u"I3 弧度转度",
          near(math.degrees(corner.steer_angle()), 3.5748, 1e-3))
    # 特征车速处：L + K·v² = 2L
    vchar = spec.characteristic_speed()
    check(u"I4 特征车速使分母翻倍",
          near(spec.wheelbase + k * vchar ** 2, 2 * spec.wheelbase, 1e-9))
    # 临界车速处：L + K·v² = 0
    over = core.VehicleSpec(front_cornering=160000.0, rear_cornering=40000.0)
    vcrit = over.critical_speed()
    check(u"I5 临界车速使分母为零",
          near(over.wheelbase + over.gradient() * vcrit ** 2, 0.0, 1e-9))
    # 轴荷合力矩：W_f·b = W_r·a
    check(u"I6 轴荷满足力矩平衡",
          near(spec.front_load() * spec.cg_to_front,
               spec.rear_load() * spec.cg_to_rear(), 1e-9))


# ------------------------------------------------------------ J 工程文件约定
def group_files():
    out(u"J 工程文件约定")
    for rel in ("CMakeLists.txt", "package.xml", "README.md",
                "config/lateral_stability_margin.yaml", "launch/main.launch",
                "scripts/main.py", "scripts/stability_core.py",
                "scripts/stability_node.py"):
        check(u"J %s 存在" % rel, os.path.isfile(os.path.join(PKG, rel)))

    launch = io.open(os.path.join(PKG, "launch", "main.launch"),
                     encoding="utf-8").read()
    check(u"J launch 有 rosparam load", u'<rosparam command="load"' in launch)
    check(u"J launch 有 node", u'type="main.py"' in launch)
    check(u"J launch 参数个数 >= 12", launch.count("<arg ") >= 12,
          repr(launch.count("<arg ")))

    main = io.open(os.path.join(PKG, "scripts", "main.py"),
                   encoding="utf-8").read()
    check(u"J main 转发到 node", u"stability_node" in main and u"main()" in main)

    node = io.open(os.path.join(PKG, "scripts", "stability_node.py"),
                   encoding="utf-8").read()
    check(u"J node 有 _log", u"def _log(" in node)
    check(u"J node 打中文前转 byte str", u".encode(" in node)
    check(u"J node 调 init_node", u"init_node(" in node)
    check(u"J node 有保活循环", u"is_shutdown" in node and u"rate.sleep" in node)

    readme = io.open(os.path.join(PKG, "README.md"), encoding="utf-8").read()
    check(u"J README 含公式", u"不足转向梯度" in readme)
    check(u"J README 含话题表", u"话题" in readme)


# ------------------------------------------------------------ K 编码声明回归
def group_encoding():
    out(u"K 编码声明回归")
    for name in ("main.py", "stability_core.py", "stability_node.py"):
        path = os.path.join(PKG, "scripts", name)
        lines = io.open(path, encoding="utf-8").read().split("\n")
        check(u"K %s 第 1 行 shebang" % name,
              lines[0].startswith("#!") and "python" in lines[0], lines[0])
        check(u"K %s 第 2 行 coding" % name,
              "coding" in lines[1] and "utf-8" in lines[1], lines[1])
        check(u"K %s 没有中文在编码声明之前" % name,
              "coding" in "".join(lines[:2]))


def main():
    out(u"lateral_stability_margin 离线回归测试")
    out(u"Python %s" % sys.version.split()[0])
    out(u"包目录 %s" % PKG)
    out(u"-" * 60)
    group_geometry()
    group_gradient()
    group_cornering()
    group_asserts()
    group_scan()
    group_render()
    group_params()
    group_report()
    group_identity()
    group_files()
    group_encoding()
    out(u"-" * 60)

    total = 0
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
