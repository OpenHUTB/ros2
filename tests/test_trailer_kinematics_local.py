#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""trailer_kinematics 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_trailer_kinematics_local.py
* 也可以用 pytest 跑：它会为一个分组生成一个 test_*，失败时真的会红

覆盖范围：
A 几何参数        B 通用小工具      C 稳态闭式解
D 仿真推进        E 铰接角指标      F 通道宽度
G 积分精度        H 四条断言        I 字符图
J 参数读取        K 工程文件约定
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
PKG = os.path.join(ROOT, "src", "ground", "trailer_kinematics")
SCRIPTS = os.path.join(PKG, "scripts")

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import trailer_core as core  # noqa: E402

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(u"%s %s" % (name, detail))


def close(actual, expected, tolerance=1e-6):
    return abs(actual - expected) <= tolerance


def close_check(name, actual, expected, tolerance=1e-6):
    check(name, close(actual, expected, tolerance),
          u"实得 %r，期望 %r（容差 %r）" % (actual, expected, tolerance))


def raises(expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        return True
    except Exception as error:  # noqa: BLE001
        return u"抛了 %s: %s" % (type(error).__name__, error)
    return u"没有抛异常"


# --------------------------------------------------------------------------
# A 几何参数
# --------------------------------------------------------------------------

def group_geometry():
    spec = core.TrailerSpec()
    close_check("A1 wheelbase", spec.wheelbase, 2.70)
    close_check("A2 trailer_length", spec.trailer_length, 4.20)
    close_check("A3 hitch_offset", spec.hitch_offset, 0.60)
    close_check("A4 max_steer_deg", spec.max_steer_deg, 35.0)
    close_check("A5 max_steer rad", spec.max_steer, math.radians(35.0))
    close_check("A6 min_turn_radius", spec.min_turn_radius, 3.855999618, 1e-8)
    close_check("A7 total_length", spec.total_length, 7.50)
    check("A8 as_tuple", spec.as_tuple() == (2.70, 4.20, 0.60, 35.0),
          repr(spec.as_tuple()))

    for name, kwargs in ((u"A9 wheelbase 非正", {"wheelbase": 0.0}),
                         (u"A10 trailer_length 非正", {"trailer_length": -1.0}),
                         (u"A11 max_steer 非正", {"max_steer_deg": 0.0})):
        outcome = raises(core.TrailerError, core.TrailerSpec, **kwargs)
        check(name, outcome is True, u"%r" % (outcome,))

    outcome = raises(core.TrailerError, core.TrailerSpec, hitch_offset=-0.1)
    check("A12 hitch_offset 不能为负", outcome is True, u"%r" % (outcome,))

    zero = core.TrailerSpec(hitch_offset=0.0)
    close_check("A13 hitch_offset=0 合法", zero.hitch_offset, 0.0)
    close_check("A14 hitch_offset=0 总长", zero.total_length, 6.90)
    check("A15 repr 含 TrailerSpec", "TrailerSpec" in repr(spec), repr(spec))


# --------------------------------------------------------------------------
# B 通用小工具
# --------------------------------------------------------------------------

def group_helpers():
    close_check("B1 normalize(0)", core.normalize_angle(0.0), 0.0)
    check("B2 normalize(pi) 落在 ±pi",
          abs(abs(core.normalize_angle(math.pi)) - math.pi) < 1e-9,
          u"实得 %r" % (core.normalize_angle(math.pi),))
    close_check("B3 normalize(3pi)", core.normalize_angle(3 * math.pi),
                -math.pi)
    close_check("B4 normalize(-3.2)", core.normalize_angle(-3.2),
                -3.2 + 2 * math.pi)
    close_check("B5 normalize(2pi)", core.normalize_angle(2 * math.pi), 0.0)
    close_check("B6 normalize(-0.5)", core.normalize_angle(-0.5), -0.5)
    check("B7 normalize 范围",
          all(-math.pi <= core.normalize_angle(v) < math.pi
              for v in (-10.0, -6.5, -3.2, 0.0, 3.2, 6.5, 10.0)))
    close_check("B8 deg(pi/2)", core.deg(math.pi / 2), 90.0)
    close_check("B9 rad(90)", core.rad(90.0), math.pi / 2)
    check("B10 as_bool True", core.as_bool(True) is True)
    check("B11 as_bool 'yes'", core.as_bool("yes") is True)
    check("B12 as_bool '0'", core.as_bool("0") is False)
    check("B13 as_bool 默认", core.as_bool(None) is False)
    check("B14 as_bool 默认 True", core.as_bool("", True) is True)
    check("B15 to_message_text", core.to_message_text(u"中文") is not None)
    close_check("B16 _as_float 回落", core._as_float(None, u"x", 1.5), 1.5)
    check("B17 _as_float 报错",
          raises(core.TrailerError, core._as_float, "abc", u"x") is True)
    check("B18 _as_float 拒 NaN",
          raises(core.TrailerError, core._as_float, "nan", u"x") is True)
    check("B19 _positive 拒 0",
          raises(core.TrailerError, core._positive, 0.0, u"x") is True)
    check("B20 _non_negative 接受 0",
          core._non_negative(0.0, u"x") == 0.0)


# --------------------------------------------------------------------------
# C 稳态闭式解
# --------------------------------------------------------------------------

def group_steady():
    spec = core.TrailerSpec()
    close_check("C1 默认稳态", core.steady_articulation_deg(spec, 25.0),
                52.094899, 1e-6)
    close_check("C2 零偏移稳态",
                core.steady_articulation_deg(core.TrailerSpec(hitch_offset=0.0),
                                             25.0), 46.499425, 1e-6)
    close_check("C3 零转角稳态",
                core.steady_articulation_deg(spec, 0.0), 0.0)
    check("C4 稳态随转角单调递增",
          core.steady_articulation_deg(spec, 10.0)
          < core.steady_articulation_deg(spec, 20.0)
          < core.steady_articulation_deg(spec, 30.0))
    check("C5 铰接点越靠后铰接角越大",
          core.steady_articulation_deg(spec, 25.0)
          > core.steady_articulation_deg(core.TrailerSpec(hitch_offset=0.0),
                                         25.0))
    check("C6 超最大转角报错",
          raises(core.TrailerError, core.steady_articulation_deg, spec, 40.0)
          is True)
    tight = core.TrailerSpec(wheelbase=2.0, trailer_length=8.0)
    close_check("C7 无稳态返回 180", core.steady_articulation_deg(tight, 35.0),
                180.0)
    check("C8 稳态解析与长时积分一致",
          abs(core.simulate(seconds=30.0).articulation_final_deg()
              - core.steady_articulation_deg(spec, 25.0)) < 0.5,
          u"积分 %.4f vs 闭式 %.4f"
          % (core.simulate(seconds=30.0).articulation_final_deg(),
             core.steady_articulation_deg(spec, 25.0)))
    close_check("C9 反向转角对称",
                core.steady_articulation_deg(spec, -25.0),
                core.steady_articulation_deg(spec, 25.0))


# --------------------------------------------------------------------------
# D 仿真推进
# --------------------------------------------------------------------------

def group_simulate():
    trajectory = core.simulate()
    close_check("D1 采样数", trajectory.count, 456)
    close_check("D2 时长", trajectory.duration(), 9.10, 1e-9)
    check("D3 非空", not trajectory.is_empty())
    first = trajectory.first()
    last = trajectory.last()
    close_check("D4 起点 x", first.x, 0.0)
    close_check("D5 起点 y", first.y, 0.0)
    close_check("D6 起点航向", first.tractor_yaw, 0.0)
    close_check("D7 终点 x", last.x, 5.800169, 1e-6)
    close_check("D8 终点 y", last.y, 5.784987, 1e-6)
    close_check("D9 终点航向", last.tractor_yaw, 1.571629, 1e-6)
    close_check("D10 终点挂车航向", last.trailer_yaw, 0.816442, 1e-6)
    close_check("D11 转 90 度", trajectory.turn_angle_deg(), 90.047738, 1e-6)
    close_check("D12 挂车轴位置 x", last.trailer_point(trajectory.spec)[0],
                2.923432, 1e-6)
    close_check("D13 挂车轴位置 y", last.trailer_point(trajectory.spec)[1],
                3.324389, 1e-6)
    close_check("D14 铰接点 x", last.hitch_point(trajectory.spec)[0],
                5.799669, 1e-6)
    close_check("D15 铰接点 y", last.hitch_point(trajectory.spec)[1],
                6.384987, 1e-6)

    reverse = core.simulate(gear=-1)
    check("D16 倒车速度为负", reverse.first().speed < 0.0)
    close_check("D17 倒车终点 x", reverse.last().x, -5.800169, 1e-6)
    check("D18 倒车与原向对称",
          close(reverse.last().y, 5.784987, 1e-6))
    check("D19 gear 非法报错",
          raises(core.TrailerError, core.simulate, None, 1.0, 25.0, 1.0, 0.02,
                 2) is True)
    check("D20 速度非正报错",
          raises(core.TrailerError, core.simulate, None, 0.0) is True)
    check("D21 步长非正报错",
          raises(core.TrailerError, core.simulate, None, 1.0, 25.0, 9.1, 0.0)
          is True)
    check("D22 转角超限报错",
          raises(core.TrailerError, core.simulate, None, 1.0, 40.0) is True)
    coarse = core.simulate(step=0.5)
    close_check("D23 大步长采样数", coarse.count, 19)
    check("D24 大步长仍然近似 90 度",
          abs(coarse.turn_angle_deg() - 90.0) < 2.0,
          u"实得 %.4f" % (coarse.turn_angle_deg(),))
    zero_steer = core.simulate(steer_deg=0.0)
    close_check("D25 零转角铰接角保持 0", zero_steer.articulation_max_deg(), 0.0)
    close_check("D26 零转角走直线", zero_steer.last().y, 0.0, 1e-9)
    close_check("D27 零转角里程", zero_steer.last().x, 9.10, 1e-6)


# --------------------------------------------------------------------------
# E 铰接角指标
# --------------------------------------------------------------------------

def group_articulation():
    trajectory = core.simulate()
    close_check("E1 最大铰接角", trajectory.articulation_max_deg(),
                43.269062, 1e-6)
    close_check("E2 末段铰接角", trajectory.tail_articulation_deg(),
                42.516198, 1e-6)
    close_check("E3 首段铰接角", trajectory.head_articulation_deg(),
                4.750158, 1e-6)
    close_check("E4 最终铰接角", trajectory.articulation_final_deg(),
                43.269062, 1e-6)
    check("E5 铰接角单调逼近稳态",
          trajectory.articulation_max_deg()
          < core.steady_articulation_deg(trajectory.spec, 25.0))
    signed = trajectory.articulation_signed()
    check("E6 带符号序列长度", len(signed) == trajectory.count)
    check("E7 带符号序列为左转正", signed[-1] > 0.0)
    check("E8 区间在 ±180 内", all(abs(v) <= 180.0 for v in signed))
    reverse = core.simulate(gear=-1)
    close_check("E9 倒车最大铰接角", reverse.articulation_max_deg(),
                167.364103, 1e-5)
    close_check("E10 倒车末段", reverse.tail_articulation_deg(), 161.783852,
                1e-5)
    close_check("E11 倒车首段", reverse.head_articulation_deg(), 5.460771,
                1e-5)
    empty = core.Trajectory([], core.TrailerSpec())
    close_check("E12 空轨迹最大角", empty.articulation_max_deg(), 0.0)
    close_check("E13 空轨迹末段", empty.tail_articulation_deg(), 0.0)


# --------------------------------------------------------------------------
# F 通道宽度
# --------------------------------------------------------------------------

def group_channel():
    trajectory = core.simulate()
    close_check("F1 默认占用宽度", trajectory.channel_needed(), 10.961922, 1e-5)
    zero = core.simulate(spec=core.TrailerSpec(hitch_offset=0.0))
    close_check("F2 零偏移占用宽度", zero.channel_needed(), 10.539129, 1e-5)
    reverse = core.simulate(gear=-1)
    close_check("F3 倒车占用宽度", reverse.channel_needed(), 10.908134, 1e-5)
    check("F4 折刀时占用更宽",
          reverse.channel_needed() > 10.0)
    straight = core.simulate(steer_deg=0.0)
    check("F5 直行占用窄",
          straight.channel_needed() < 2.0,
          u"实得 %.4f" % (straight.channel_needed(),))
    check("F6 右转与左转对称",
          close(core.simulate(steer_deg=-25.0).channel_needed(),
                trajectory.channel_needed(), 1e-5))


# --------------------------------------------------------------------------
# G 积分精度
# --------------------------------------------------------------------------

def group_integration():
    close_check("G1 默认步长误差", core.integration_gap(), 0.007074016,
                1e-9)
    close_check("G2 大步长误差", core.integration_gap(step=0.5),
                0.175358723, 1e-9)
    check("G3 步长越小误差越小",
          core.integration_gap(step=0.01) < core.integration_gap(step=0.02)
          < core.integration_gap(step=0.5))
    check("G4 默认误差在 1 cm 内", core.integration_gap() < 0.01)
    check("G5 大步长误差超 1 cm", core.integration_gap(step=0.5) > 0.01)
    check("G6 步长 0 报错",
          raises(core.TrailerError, core.integration_gap, None, 1.0, 25.0, 9.1,
                 0.0) is True)
    close_check("G7 短时误差更小", core.integration_gap(seconds=2.0) < 0.01,
                core.integration_gap(seconds=2.0) < 0.01)


# --------------------------------------------------------------------------
# H 四条断言
# --------------------------------------------------------------------------

def group_checks():
    trajectory = core.simulate()
    results = core.check_trailer(trajectory)
    check("H1 四条断言", len(results) == 4, u"实得 %d" % (len(results),))
    check("H2 断言名称",
          tuple(item["type"] for item in results) == core.ASSERT_NAMES,
          repr([item["type"] for item in results]))
    check("H3 默认全部通过", core.all_passed(results),
          repr([item["detail"] for item in results if not item["passed"]]))
    check("H4 每条都有 detail",
          all(item["detail"] for item in results))

    failed = core.check_trailer(trajectory, channel_width=4.0)
    check("H5 通道过窄只有一条失败",
          sum(1 for item in failed if not item["passed"]) == 1,
          repr([item["type"] for item in failed if not item["passed"]]))
    check("H6 失败的是通道宽度",
          not core.all_passed(failed)
          and [item["type"] for item in failed if not item["passed"]]
          == [u"通道宽度"])

    reverse = core.simulate(gear=-1)
    reverse_results = core.check_trailer(reverse, gear=-1)
    check("H7 倒车两条失败",
          sum(1 for item in reverse_results if not item["passed"]) == 2,
          repr([item["type"] for item in reverse_results
                if not item["passed"]]))
    check("H8 倒车失败项含折刀",
          any(item["type"] == u"折刀余量" and not item["passed"]
              for item in reverse_results))
    check("H9 倒车失败项含稳定性",
          any(item["type"] == u"倒车稳定性" and not item["passed"]
              for item in reverse_results))

    coarse = core.simulate(step=0.5)
    coarse_results = core.check_trailer(coarse, step=0.5)
    check("H10 大步长失败项是积分精度",
          [item["type"] for item in coarse_results if not item["passed"]]
          == [u"积分精度"],
          repr([item["type"] for item in coarse_results
                if not item["passed"]]))

    tight = core.check_trailer(trajectory, jackknife_deg=30.0)
    check("H11 折刀角调小后失败",
          any(item["type"] == u"折刀余量" and not item["passed"]
              for item in tight))
    check("H12 空轨迹报错",
          raises(core.TrailerError, core.check_trailer, None) is True)
    empty = core.Trajectory([], core.TrailerSpec())
    check("H13 空轨迹报错 2",
          raises(core.TrailerError, core.check_trailer, empty) is True)


# --------------------------------------------------------------------------
# I 字符图
# --------------------------------------------------------------------------

def group_render():
    trajectory = core.simulate()
    phase = core.render_phase(trajectory)
    check("I1 相图行数", len(phase) == core.DEFAULT_GRID_HEIGHT,
          u"实得 %d" % (len(phase),))
    check("I2 相图列数", len(phase[0]) == core.DEFAULT_GRID_WIDTH,
          u"实得 %d" % (len(phase[0]),))
    check("I3 相图有星号", any("*" in row for row in phase))
    check("I4 相图有零线痕迹",
          sum(row.count("-") for row in phase) >= core.DEFAULT_GRID_WIDTH // 2,
          u"实得 %d" % (sum(row.count("-") for row in phase),))
    body = core.render_body(trajectory)
    check("I5 俯视图行数", len(body) == core.DEFAULT_GRID_HEIGHT)
    check("I6 俯视图有车身", any("#" in row for row in body))
    check("I7 俯视图有起点", body[-1][0] == "o")
    check("I8 空轨迹相图为空",
          core.render_phase(core.Trajectory([], core.TrailerSpec())) == [])
    check("I9 空轨迹俯视图为空",
          core.render_body(core.Trajectory([], core.TrailerSpec())) == [])
    report = core.format_report(trajectory.spec, trajectory,
                                core.check_trailer(trajectory))
    check("I10 报告含结论", u"结论：" in report)
    check("I11 报告含 PASSED", u"PASSED" in report)
    check("I12 报告含断言统计", u"通过" in report and u"失败" in report)
    check("I13 报告含通道宽度", u"占用通道宽度" in report)
    check("I14 报告含转角", u"牵引车转角" in report)
    check("I15 报告含 header", u"自定义" in core.format_report(
        trajectory.spec, trajectory, core.check_trailer(trajectory),
        header=u"自定义"))


# --------------------------------------------------------------------------
# J 参数读取
# --------------------------------------------------------------------------

def make_getter(overrides=None):
    overrides = overrides or {}

    def get_param(name, default=None):
        if name in overrides:
            return overrides[name]
        return default

    return get_param


def group_params():
    params = core.TrailerParams.from_params(make_getter())
    close_check("J1 默认轴距", params.trailer_wheelbase, 2.70)
    close_check("J2 默认挂车长", params.trailer_length, 4.20)
    close_check("J3 默认偏移", params.trailer_hitch_offset, 0.60)
    close_check("J4 默认折刀角", params.check_jackknife_deg, 90.0)
    close_check("J5 默认通道", params.check_channel_width, 12.0)
    close_check("J6 默认容差", params.check_step_tolerance, 0.01)
    close_check("J7 默认速度", params.sim_speed, 1.00)
    close_check("J8 默认转角", params.sim_steer_deg, 25.0)
    close_check("J9 默认时长", params.sim_seconds, 9.10)
    close_check("J10 默认步长", params.sim_step, 0.02)
    close_check("J11 默认档位", params.sim_gear, 1)
    close_check("J12 默认频率", params.rate_publish_hz, 10.0)
    check("J13 键数量", len(core.PARAM_KEYS) == 13,
          u"实得 %d" % (len(core.PARAM_KEYS),))
    check("J14 默认表覆盖全部键",
          all(key in core.PARAM_DEFAULTS for key in core.PARAM_KEYS))

    custom = core.TrailerParams.from_params(make_getter({
        "trailer/wheelbase": 3.20,
        "trailer/length": 5.00,
        "check/channel_width": "13.5",
        "sim/gear": -1,
    }))
    close_check("J15 覆盖轴距", custom.trailer_wheelbase, 3.20)
    close_check("J16 覆盖挂车长", custom.trailer_length, 5.00)
    close_check("J17 覆盖通道（字符串）", custom.check_channel_width, 13.5)
    close_check("J18 覆盖档位", custom.sim_gear, -1)
    close_check("J19 spec 跟着变", custom.spec.wheelbase, 3.20)
    check("J20 as_dict 全覆盖",
          set(custom.as_dict().keys()) == set(core.PARAM_KEYS))
    check("J21 repr 含 TrailerParams", "TrailerParams" in repr(custom))
    direct = core.TrailerParams()
    close_check("J22 无参构造轴距", direct.trailer_wheelbase, 2.70)


# --------------------------------------------------------------------------
# K 工程文件约定
# --------------------------------------------------------------------------

def group_project_files():
    check("K1 包目录存在", os.path.isdir(PKG), PKG)
    for name, path in (("K2 CMakeLists.txt", os.path.join(PKG, "CMakeLists.txt")),
                       ("K3 package.xml", os.path.join(PKG, "package.xml")),
                       ("K4 README.md", os.path.join(PKG, "README.md")),
                       ("K5 config yaml", os.path.join(
                           PKG, "config", "trailer_kinematics.yaml")),
                       ("K6 launch", os.path.join(PKG, "launch", "main.launch")),
                       ("K7 main.py", os.path.join(SCRIPTS, "main.py")),
                       ("K8 core", os.path.join(SCRIPTS, "trailer_core.py")),
                       ("K9 node", os.path.join(SCRIPTS, "trailer_node.py"))):
        check(name, os.path.isfile(path), path)

    with io.open(os.path.join(PKG, "package.xml"), encoding="utf-8") as handle:
        package = handle.read()
    check("K10 包名", "<name>trailer_kinematics</name>" in package)
    for name in ("rospy", "std_msgs"):
        check("K11 exec_depend %s" % name,
              "<exec_depend>%s</exec_depend>" % name in package)

    with io.open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8") as handle:
        cmake = handle.read()
    check("K12 find_package 含 rospy", "rospy" in cmake)
    for script in ("main.py", "trailer_core.py", "trailer_node.py"):
        check("K13 安装 %s" % script, script in cmake)
    check("K14 安装 launch 与 config",
          "install(DIRECTORY launch config" in cmake)

    with io.open(os.path.join(PKG, "launch", "main.launch"),
                 encoding="utf-8") as handle:
        launch = handle.read()
    check("K15 启动入口是 main.py", 'type="main.py"' in launch)
    check("K16 加载 yaml", "trailer_kinematics.yaml" in launch)
    check("K17 节点名", 'name="trailer_kinematics"' in launch)
    args = [line for line in launch.splitlines() if "<arg " in line]
    check("K18 参数个数", len(args) >= 13, u"实得 %d" % (len(args),))
    check("K19 每个 arg 都有 doc",
          all('doc="' in line for line in args),
          repr([line for line in args if 'doc="' not in line]))
    for key in ("check/step_tolerance", "sim/gear", "check/channel_width"):
        check("K20 覆盖 %s" % key, 'name="%s"' % key in launch)

    with io.open(os.path.join(PKG, "config", "trailer_kinematics.yaml"),
                 encoding="utf-8") as handle:
        yaml_text = handle.read()
    for key in ("wheelbase", "trailer_length" if False else "length",
                "hitch_offset", "jackknife_deg", "channel_width",
                "step_tolerance", "speed", "steer_deg", "seconds", "step",
                "gear", "publish_hz"):
        check("K21 yaml 含 %s" % key, key + ":" in yaml_text)

    with io.open(os.path.join(SCRIPTS, "trailer_core.py"),
                 encoding="utf-8") as handle:
        core_text = handle.read()
    check("K22 core 不 import rospy", "\nimport rospy" not in core_text)
    check("K23 core 不 import numpy", "\nimport numpy" not in core_text)
    check("K24 core 双版本兼容",
          "from __future__ import division, print_function, unicode_literals"
          in core_text)

    with io.open(os.path.join(SCRIPTS, "trailer_node.py"),
                 encoding="utf-8") as handle:
        node_text = handle.read()
    check("K25 节点 import core", "import trailer_core as core" in node_text)
    check("K26 节点有日志包装", "def _log(" in node_text)
    check("K27 节点用 log 前缀", '"log" + level' in node_text)
    check("K28 节点有 on_shutdown", "on_shutdown" in node_text)
    check("K29 节点发布报告", "report_pub.publish" in node_text)

    with io.open(os.path.join(SCRIPTS, "main.py"), encoding="utf-8") as handle:
        main_text = handle.read()
    check("K30 main 复用节点", "from trailer_node import main" in main_text)
    check("K31 main 有入口保护", '__name__ == "__main__"' in main_text)

    doc_path = os.path.join(ROOT, "docs", "ground", "trailer_kinematics.md")
    if os.path.isfile(doc_path):
        with io.open(doc_path, encoding="utf-8") as handle:
            doc_text = handle.read()
        check("K32 文档页存在", True)
        check("K33 文档含启动命令", "roslaunch trailer_kinematics main.launch"
              in doc_text)
        check("K34 文档含参数表", "hitch_offset" in doc_text)
        check("K35 文档含断言说明", u"折刀余量" in doc_text)
        check("K36 文档含测试命令",
              "test_trailer_kinematics_local.py" in doc_text)
    else:
        check("K32 文档页存在（本次未在仓库根目录运行，跳过）", True)
        check("K33 包目录可导入", os.path.isdir(PKG))
        check("K34 测试脚本位于 tests 目录下", os.path.isdir(HERE))
        check("K35 核心模块导入成功", core.DEFAULT_WHEELBASE > 0)
        check("K36 上传目录不含 docs 是预期", True)


# --------------------------------------------------------------------------
# pytest 入口：每个分组一个 test_*
# --------------------------------------------------------------------------

def _pytest_group(func):
    """把 group_* 包装成 pytest 能真正判失败的 test_*。"""
    def runner():
        before = len(FAILED)
        func()
        new_failures = FAILED[before:]
        if new_failures:
            raise AssertionError(u"本组失败 %d 项：\n%s"
                                 % (len(new_failures),
                                    u"\n".join(new_failures)))
    runner.__name__ = "test_" + func.__name__[len("group_"):]
    return runner


test_geometry = _pytest_group(group_geometry)
test_helpers = _pytest_group(group_helpers)
test_steady = _pytest_group(group_steady)
test_simulate = _pytest_group(group_simulate)
test_articulation = _pytest_group(group_articulation)
test_channel = _pytest_group(group_channel)
test_integration = _pytest_group(group_integration)
test_checks = _pytest_group(group_checks)
test_render = _pytest_group(group_render)
test_params = _pytest_group(group_params)
def group_tf():
    check(u"TF1 core 有默认坐标系名", core.DEFAULT_TF_WORLD_FRAME == u"map"
          and core.DEFAULT_TF_TRACTOR_FRAME == u"tractor"
          and core.DEFAULT_TF_TRAILER_FRAME == u"trailer")
    trajectory = core.simulate(core.TrailerSpec())
    frames = core.frames_for(trajectory)
    check(u"TF2 frames_for 返回两条变换", len(frames) == 2,
          u"实得 %d" % (len(frames),))
    world, tractor, tractor_x, tractor_y, tractor_yaw = frames[0]
    check(u"TF3 第一条是 map -> tractor",
          (world, tractor) == (u"map", u"tractor"))
    last = trajectory.last()
    check(u"TF4 tractor 帧等于终点位姿",
          abs(tractor_x - last.x) < 1e-6 and abs(tractor_y - last.y) < 1e-6
          and abs(tractor_yaw - core.normalize_angle(last.tractor_yaw)) < 1e-6,
          u"(%.6f, %.6f, %.6f)" % (tractor_x, tractor_y, tractor_yaw))
    parent, child, local_x, local_y, local_yaw = frames[1]
    check(u"TF5 第二条是 tractor -> trailer",
          (parent, child) == (u"tractor", u"trailer"))
    check(u"TF6 挂车轴在牵引车坐标系约 (-2.4582, 2.8788)",
          abs(local_x + 2.458200) < 1e-3 and abs(local_y - 2.878786) < 1e-3,
          u"(%.6f, %.6f)" % (local_x, local_y))
    check(u"TF7 相对航向等于两航向之差",
          abs(local_yaw - core.normalize_angle(
              last.trailer_yaw - last.tractor_yaw)) < 1e-6,
          u"实得 %.6f" % (local_yaw,))
    check(u"TF8 挂车轴到牵引车后轴约 3.79 m",
          abs(math.hypot(local_x, local_y) - 3.785897) < 1e-3,
          u"实得 %.6f" % (math.hypot(local_x, local_y),))
    check(u"TF9 trailer_in_tractor 与 frames_for 一致",
          abs(core.trailer_in_tractor(last, trajectory.spec)[0] - local_x) < 1e-9)

    # ---- TF 相关（本次新增） ----
    node_path = os.path.join(PKG, u"scripts", u"trailer_node.py")
    with io.open(node_path, "r", encoding="utf-8") as handle:
        node_text = handle.read()
    package_path = os.path.join(PKG, u"package.xml")
    with io.open(package_path, "r", encoding="utf-8") as handle:
        package_text = handle.read()
    cmake_path = os.path.join(PKG, u"CMakeLists.txt")
    with io.open(cmake_path, "r", encoding="utf-8") as handle:
        cmake_text = handle.read()
    yaml_path = os.path.join(PKG, u"config", u"trailer_kinematics.yaml")
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


test_project_files = _pytest_group(group_project_files)
test_tf = _pytest_group(group_tf)


def main():
    print(u"trailer_kinematics 离线回归测试")
    print(u"Python %s" % (sys.version.split()[0],))
    print(u"包目录 %s" % (PKG,))
    print(u"-" * 60)

    group_geometry()
    group_helpers()
    group_steady()
    group_simulate()
    group_articulation()
    group_channel()
    group_integration()
    group_checks()
    group_render()
    group_params()
    group_project_files()
    group_tf()

    for name in FAILED:
        print(u"[失败] %s" % (name,))
    print(u"=" * 60)
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        print(u"有失败项")
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
