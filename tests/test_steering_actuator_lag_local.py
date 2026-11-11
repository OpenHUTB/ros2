#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""steering_actuator_lag 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_steering_actuator_lag_local.py
* 也可以用 pytest 跑：pytest tests/test_steering_actuator_lag_local.py

覆盖范围：
A 规格与常量      B 小工具         C 指令生成
D 仿真基本事实    E 上升时间       F 稳态误差
G 速率上限        H 回正           I 四条断言
J 参数读取        K 工程文件约定
"""

from __future__ import division
from __future__ import print_function

import io
import math
import os
import sys

try:                       # Python 2 兼容
    text_type = unicode    # noqa: F821
except NameError:          # Python 3
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "src", "ground", "steering_actuator_lag")
SCRIPTS = os.path.join(PKG, "scripts")

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import actuator_core as core


PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
        if detail:
            print(u"  [通过] %s %s" % (name, detail))
    else:
        FAILED.append(u"%s %s" % (name, detail))
        print(u"  [失败] %s %s" % (name, detail))


def check_close(name, actual, expected, tolerance=1e-6, detail=""):
    ok = abs(actual - expected) <= tolerance
    check(name, ok, u"%s 实际 %r 期望 %r%s"
          % (detail or name, actual, expected,
             u"（容差 %g）" % tolerance if not ok else u""))


def make_spec(**kwargs):
    args = dict(
        time_constant=core.DEFAULT_TIME_CONSTANT,
        dead_band_deg=core.DEFAULT_DEAD_BAND_DEG,
        max_rate_deg=core.DEFAULT_MAX_RATE_DEG,
        center_rate_deg=core.DEFAULT_CENTER_RATE_DEG,
        max_angle_deg=core.DEFAULT_MAX_ANGLE_DEG,
    )
    args.update(kwargs)
    return core.ActuatorSpec(**args)


def make_response(spec=None, step_deg=core.DEFAULT_STEP_DEG,
                  dwell=core.DEFAULT_DWELL_SECONDS, dt=core.DEFAULT_SIM_STEP):
    return core.simulate(spec=spec or make_spec(), step_deg=step_deg,
                         dwell=dwell, dt=dt)


def raises(expected, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except expected:
        return True
    except Exception:
        return False
    return False


# ---------------------------------------------------------------- A 规格与常量
def group_spec():
    print(u"\nA 规格与常量")
    check("A1", core.DEFAULT_TIME_CONSTANT == 0.25, u"时间常数默认 0.25")
    check("A2", core.DEFAULT_DEAD_BAND_DEG == 0.30, u"死区默认 0.30")
    check("A3", core.DEFAULT_MAX_RATE_DEG == 60.0, u"打角上限 60")
    check("A4", core.DEFAULT_CENTER_RATE_DEG == 40.0, u"回正上限 40")
    check("A5", core.DEFAULT_MAX_ANGLE_DEG == 35.0, u"最大转角 35")
    check("A6", core.DEFAULT_STEP_DEG == 20.0, u"阶跃 20 度")
    check("A7", core.DEFAULT_DWELL_SECONDS == 5.0, u"保持 5 s")
    check("A8", core.DEFAULT_RISE_LIMIT == 1.00, u"上升上限 1 s")
    check("A9", core.DEFAULT_CENTER_LIMIT == 1.50, u"回正上限 1.5 s")
    check("A10", core.DEFAULT_SIM_STEP == 0.01, u"步长 0.01")
    check("A11", core.ASSERT_NAMES == (u"上升时间", u"稳态误差",
                                      u"速率上限", u"回正时间"),
          u"四条断言名字顺序固定")

    spec = make_spec()
    check("A12", spec.time_constant == 0.25, u"spec.time_constant")
    check("A13", spec.dead_band_deg == 0.30, u"spec.dead_band_deg")
    check("A14", spec.max_rate_deg == 60.0, u"spec.max_rate_deg")
    check("A15", spec.center_rate_deg == 40.0, u"spec.center_rate_deg")
    check("A16", spec.max_angle_deg == 35.0, u"spec.max_angle_deg")
    check_close("A17", spec.max_rate, math.radians(60.0), 1e-12, u"弧度属性")
    check_close("A18", spec.center_rate, math.radians(40.0), 1e-12,
                u"回正弧度属性")
    check_close("A19", spec.dead_band, math.radians(0.30), 1e-12,
                u"死区弧度属性")
    check_close("A20", spec.max_angle, math.radians(35.0), 1e-12,
                u"转角弧度属性")
    check("A21", spec.as_tuple() == (0.25, 0.30, 60.0, 40.0, 35.0),
          u"as_tuple 顺序")

    check("A22", raises(core.ActuatorError, make_spec, time_constant=0.0),
          u"时间常数 0 应报错")
    check("A23", raises(core.ActuatorError, make_spec, time_constant=-1.0),
          u"时间常数负数应报错")
    check("A24", raises(core.ActuatorError, make_spec, dead_band_deg=-0.1),
          u"死区负数应报错")
    check("A25", raises(core.ActuatorError, make_spec, max_rate_deg=0.0),
          u"打角上限 0 应报错")
    check("A26", raises(core.ActuatorError, make_spec, center_rate_deg=-5.0),
          u"回正上限负数应报错")
    check("A27", raises(core.ActuatorError, make_spec, max_angle_deg=0.0),
          u"最大转角 0 应报错")
    check("A28", raises(core.ActuatorError, make_spec,
                        time_constant=float("nan")), u"NaN 应报错")
    check("A29", raises(core.ActuatorError, make_spec,
                        dead_band_deg=float("inf")), u"Inf 应报错")
    check("A30", make_spec(time_constant=2.0).time_constant == 2.0,
          u"合法值可改")


# ---------------------------------------------------------------- B 小工具
def group_helpers():
    print(u"\nB 小工具")
    check("B1", core.clamp(5.0, 0.0, 3.0) == 3.0, u"clamp 上界")
    check("B2", core.clamp(-5.0, 0.0, 3.0) == 0.0, u"clamp 下界")
    check("B3", core.clamp(2.0, 0.0, 3.0) == 2.0, u"clamp 区间内")
    check("B4", core._as_float("1.5", "x") == 1.5, u"字符串转 float")
    check("B5", core._as_float(2, "x") == 2.0, u"整数转 float")
    check("B6", core._as_float(None, "x", 3.0) == 3.0, u"None 用默认值")
    check("B7", raises(core.ActuatorError, core._as_float, "abc", "x"),
          u"非法字符串报错")
    check("B8", raises(core.ActuatorError, core._as_float, None, "x"),
          u"缺默认值的 None 报错")
    check("B9", core.to_message_text(u"中文") is not None, u"消息文本不抛异常")
    text = core.to_message_text(u"中文")
    check("B10", not isinstance(text, text_type) or sys.version_info[0] >= 3,
          u"py2 下已转成字节串")


# ---------------------------------------------------------------- C 指令生成
def group_command():
    print(u"\nC 指令生成")
    check("C1", core.command_at(0.0, 20.0, 5.0) == 20.0, u"t=0 给阶跃")
    check("C2", core.command_at(2.5, 20.0, 5.0) == 20.0, u"保持段内给阶跃")
    check("C3", core.command_at(4.99, 20.0, 5.0) == 20.0, u"保持段末端仍给阶跃")
    check("C4", core.command_at(5.0, 20.0, 5.0) == 0.0, u"到点后归零")
    check("C5", core.command_at(7.0, 20.0, 5.0) == 0.0, u"回正段指令为 0")
    check("C6", core.command_at(1.0, -15.0, 5.0) == -15.0, u"负指令也支持")
    check("C7", core.command_at(6.0, -15.0, 5.0) == 0.0, u"负指令同样归零")


# ---------------------------------------------------------------- D 仿真基本事实
def group_simulation():
    print(u"\nD 仿真基本事实")
    response = make_response()
    check("D1", response.count == 701,
          u"采样点数 %d（7 s / 0.01 + 1）" % response.count)
    check_close("D2", response.duration(), 7.0, 1e-9, u"总时长")
    check("D3", response.first().t == 0.0, u"首点时间 0")
    check("D4", response.first().angle_deg == 0.0, u"首点角度 0")
    check("D5", response.first().command_deg == 20.0, u"首点指令 20")
    check("D6", not response.first().centering, u"首点不在回正段")
    check("D7", response.last().centering, u"末点在回正段")
    check("D8", response.last().command_deg == 0.0, u"末点指令 0")
    hold = response.hold_samples()
    check("D9", len(hold) == 500, u"保持段 %d 个点" % len(hold))
    check("D10", len(response.center_samples()) == 201,
          u"回正段 %d 个点" % len(response.center_samples()))
    check("D11", all(s.angle_deg >= 0.0 for s in response.samples),
          u"正指令下角度不出现负值")
    peak = max(s.angle_deg for s in response.samples)
    check("D12", peak <= 20.0 + 1e-9, u"峰值不超指令 %.6f" % peak)
    check("D13", hold[-1].angle_deg > hold[0].angle_deg, u"保持段角度在增长")
    check("D14", response.samples[50].angle_deg > 0.0, u"0.5 s 后已明显打角")
    response_up = make_response(dt=0.05)
    check("D15", response_up.count == 141,
          u"步长 0.05 时 %d 点" % response_up.count)
    check("D16", raises(core.ActuatorError, core.simulate,
                        spec=make_spec(), dt=0.0), u"步长 0 报错")
    check("D17", raises(core.ActuatorError, core.simulate,
                        spec=make_spec(), step_deg=0.0), u"阶跃 0 报错")
    check("D18", raises(core.ActuatorError, core.simulate,
                        spec=make_spec(), dwell=0.0), u"保持 0 报错")


# ---------------------------------------------------------------- E 上升时间
def group_rise():
    print(u"\nE 上升时间")
    response = make_response()
    check_close("E1", response.rise_time(), 0.58, 1e-9, u"默认上升到 90%")
    check("E2", response.rise_time(0.5) < response.rise_time(0.9),
          u"到 50% 比到 90% 快")
    check_close("E3", response.overshoot(), 0.0, 1e-9, u"一阶响应无超调")
    slow = make_response(spec=make_spec(time_constant=1.0))
    check_close("E4", slow.rise_time(), 2.30, 1e-9, u"时间常数 1.0 时上升")
    check("E5", slow.rise_time() > response.rise_time(), u"慢执行器上升更久")
    fast = make_response(spec=make_spec(time_constant=0.10))
    check("E6", fast.rise_time() < response.rise_time(), u"快执行器上升更快")
    tiny = make_response(step_deg=0.10)
    check("E7", tiny.rise_time() is None, u"指令小于死区时永远到不了 90%")
    check("E8", tiny.max_rate_deg() == 0.0, u"指令小于死区时完全不动")


# ---------------------------------------------------------------- F 稳态误差
def group_steady():
    print(u"\nF 稳态误差")
    response = make_response()
    check_close("F1", response.steady_error(), 0.289995, 1e-6, u"默认稳态误差")
    check("F2", abs(response.steady_error() -
                    core.DEFAULT_DEAD_BAND_DEG) < 0.02,
          u"稳态误差应贴近死区宽度")
    check_close("F3", response.hold_samples()[-1].angle_deg, 19.710005, 1e-6,
                u"保持段末角度")
    wide = make_response(spec=make_spec(dead_band_deg=2.0))
    check_close("F4", wide.steady_error(), 1.975346, 1e-6, u"死区 2 度时误差")
    check_close("F5", wide.hold_samples()[-1].angle_deg, 18.024654, 1e-6,
                u"死区 2 度时保持段末角度")
    narrow = make_response(spec=make_spec(dead_band_deg=0.0))
    check("F6", narrow.steady_error() < response.steady_error(),
          u"无死区时误差更小")
    check("F7", response.steady_error() >= 0.0, u"稳态误差非负")


# ---------------------------------------------------------------- G 速率上限
def group_rate():
    print(u"\nG 速率上限")
    response = make_response()
    check_close("G1", response.max_rate_deg(), 60.0, 1e-6, u"默认最大速率")
    check("G2", response.max_rate_deg() <= core.DEFAULT_MAX_RATE_DEG + 1e-6,
          u"最大速率不超过打角上限")
    slow = make_response(spec=make_spec(time_constant=1.0))
    check_close("G3", slow.max_rate_deg(), 20.0, 1e-6, u"惯性大时速率只有 20")
    limited = make_response(spec=make_spec(max_rate_deg=20.0))
    check_close("G4", limited.max_rate_deg(), 40.0, 1e-6,
                u"打角限 20 但回正仍可 40")
    weak = make_response(spec=make_spec(center_rate_deg=5.0))
    check("G5", weak.max_rate_deg() >= 60.0 - 1e-6, u"回正弱时打角仍到 60")
    check("G6", all(s.rate_deg >= -1e-9 for s in response.hold_samples()),
          u"保持段速率非负")


# ---------------------------------------------------------------- H 回正
def group_center():
    print(u"\nH 回正")
    response = make_response()
    check_close("H1", response.center_time(), 0.97, 1e-9, u"默认回正时间")
    check_close("H2", response.center_final_deg(), 0.007361, 1e-6,
                u"回正终值")
    check("H3", response.center_final_deg() < core.DEFAULT_CENTER_TOLERANCE,
          u"回正终值在容差内")
    slow = make_response(spec=make_spec(time_constant=1.0))
    check_close("H4", slow.center_time(), 3.65, 1e-9, u"惯性大时回正 3.65 s")
    check_close("H5", slow.center_final_deg(), 0.350097, 1e-6, u"慢回正终值")
    weak = make_response(spec=make_spec(center_rate_deg=5.0))
    check("H6", weak.center_time() is None, u"回正太弱时算不到回正时间")
    check_close("H7", weak.center_final_deg(), 9.660005, 1e-6, u"弱回正终值")
    wide = make_response(spec=make_spec(dead_band_deg=2.0))
    check_close("H8", wide.center_time(), 0.93, 1e-9, u"死区 2 度时回正")
    check("H9", response.center_time() > 0.0, u"回正时间是正的")
    check("H10", response.center_time() < response.duration(),
          u"回正在保活时间内完成")


# ---------------------------------------------------------------- I 四条断言
def group_checks():
    print(u"\nI 四条断言")
    response = make_response()
    results = core.check_response(response)
    check("I1", len(results) == 4, u"共 4 条断言")
    check("I2", [r["type"] for r in results] == list(core.ASSERT_NAMES),
          u"断言名字与顺序")
    check("I3", all(r["passed"] for r in results), u"默认全部通过")
    check("I4", core.all_passed(results), u"all_passed 为真")
    for index in range(4):
        check("I5_%d" % index, results[index]["detail"] != u"",
              u"第 %d 条有说明" % (index + 1))

    slow = core.check_response(
        make_response(spec=make_spec(time_constant=1.0)))
    check("I6", not slow[0]["passed"], u"慢执行器上升时间失败")
    check("I7", slow[1]["passed"], u"慢执行器稳态误差仍通过")
    check("I8", slow[2]["passed"], u"慢执行器速率上限仍通过")
    check("I9", not slow[3]["passed"], u"慢执行器回正时间失败")
    check("I10", not core.all_passed(slow), u"all_passed 为假")

    wide = core.check_response(
        make_response(spec=make_spec(dead_band_deg=2.0)))
    check("I11", wide[0]["passed"], u"大死区不影响上升时间")
    check("I12", not wide[1]["passed"], u"大死区稳态误差失败")
    check("I13", wide[2]["passed"], u"大死区速率上限仍通过")
    check("I14", wide[3]["passed"], u"大死区回正仍通过")

    weak = core.check_response(
        make_response(spec=make_spec(center_rate_deg=5.0)))
    check("I15", weak[0]["passed"], u"弱回正不影响上升时间")
    check("I16", weak[1]["passed"], u"弱回正不影响稳态误差")
    check("I17", weak[2]["passed"], u"弱回正速率上限仍通过")
    check("I18", not weak[3]["passed"], u"弱回正回正时间失败")

    tight = core.check_response(response, rise_limit=0.10)
    check("I19", not tight[0]["passed"], u"收紧上限后上升时间失败")
    loose = core.check_response(response, rise_limit=99.0, center_limit=99.0,
                                steady_tolerance=99.0)
    check("I20", core.all_passed(loose), u"放宽上限后全部通过")

    failed = [r for r in slow if not r["passed"]]
    check("I21", len(failed) == 2, u"慢执行器恰好 2 条失败")
    check("I22", failed[0]["type"] == u"上升时间", u"失败项的 type 是断言名")


# ---------------------------------------------------------------- J 参数读取
def group_params():
    print(u"\nJ 参数读取")
    values = {}

    def getter(name, default=None):
        if name in values:
            return values[name]
        return default

    params = core.ActuatorParams.from_params(getter)
    check_close("J1", params.spec.time_constant, 0.25, 1e-9, u"默认时间常数")
    check_close("J2", params.spec.dead_band_deg, 0.30, 1e-9, u"默认死区")
    check_close("J3", params.test_step_deg, 20.0, 1e-9, u"默认阶跃")
    check_close("J4", params.test_dwell_seconds, 5.0, 1e-9, u"默认保持")
    check_close("J5", params.sim_step, 0.01, 1e-9, u"默认步长")
    check_close("J6", params.rate_publish_hz, 10.0, 1e-9, u"默认频率")

    values["actuator/time_constant"] = "1.0"
    values["actuator/dead_band_deg"] = "2.0"
    values["test/step_deg"] = "15.0"
    values["test/dwell_seconds"] = "4.0"
    values["sim/step"] = "0.02"
    values["rate/publish_hz"] = "5.0"
    params2 = core.ActuatorParams.from_params(getter)
    check_close("J7", params2.spec.time_constant, 1.0, 1e-9, u"字符串参数被转成 float")
    check_close("J8", params2.spec.dead_band_deg, 2.0, 1e-9, u"死区覆盖")
    check_close("J9", params2.test_step_deg, 15.0, 1e-9, u"阶跃覆盖")
    check_close("J10", params2.test_dwell_seconds, 4.0, 1e-9, u"保持覆盖")
    check_close("J11", params2.sim_step, 0.02, 1e-9, u"步长覆盖")
    check_close("J12", params2.rate_publish_hz, 5.0, 1e-9, u"频率覆盖")
    check("J13", len(core.PARAM_KEYS) == 13, u"共 13 个参数键")
    check("J14", len(set(core.PARAM_KEYS)) == 13, u"参数键不重复")
    check("J15", "actuator/time_constant" in core.PARAM_KEYS, u"键名带分组前缀")
    direct = core.ActuatorParams(**{"actuator/time_constant": 2.0})
    check_close("J16", direct.spec.time_constant, 2.0, 1e-9, u"可直接构造")
    check("J17", raises(core.ActuatorError, core.ActuatorParams,
                        **{"actuator/time_constant": 0.0}),
          u"非法参数在构造时报错")


# ---------------------------------------------------------------- K 工程文件
def group_files():
    print(u"\nK 工程文件约定")
    package_xml = os.path.join(PKG, "package.xml")
    cmake = os.path.join(PKG, "CMakeLists.txt")
    launch = os.path.join(PKG, "launch", "main.launch")
    config = os.path.join(PKG, "config", "steering_actuator_lag.yaml")
    readme = os.path.join(PKG, "README.md")
    docs = os.path.join(ROOT, "docs", "ground", "steering_actuator_lag.md")
    core_py = os.path.join(SCRIPTS, "actuator_core.py")
    node_py = os.path.join(SCRIPTS, "actuator_node.py")
    main_py = os.path.join(SCRIPTS, "main.py")

    check("K1", os.path.isfile(package_xml), u"package.xml 存在")
    check("K2", os.path.isfile(cmake), u"CMakeLists.txt 存在")
    check("K3", os.path.isfile(launch), u"launch/main.launch 存在")
    check("K4", os.path.isfile(config), u"config yaml 存在")
    check("K5", os.path.isfile(readme), u"README.md 存在")
    check("K6", os.path.isfile(docs), u"docs/ground 页面存在")
    check("K7", os.path.isfile(main_py), u"scripts/main.py 存在")

    package_text = io.open(package_xml, encoding="utf-8").read()
    check("K8", u"<name>steering_actuator_lag</name>" in package_text,
          u"包名正确")
    check("K9", u"<exec_depend>rospy</exec_depend>" in package_text,
          u"依赖 rospy")
    check("K10", u"<exec_depend>std_msgs</exec_depend>" in package_text,
          u"依赖 std_msgs")
    check("K11", u"<version>1.0.0</version>" in package_text, u"版本 1.0.0")

    cmake_text = io.open(cmake, encoding="utf-8").read()
    for name in (u"scripts/main.py", u"scripts/actuator_core.py",
                 u"scripts/actuator_node.py"):
        check("K12_%s" % name.replace("/", "_"), name in cmake_text,
              u"安装 %s" % name)
    check("K13", u"catkin_install_python" in cmake_text, u"用 catkin_install_python")
    check("K14", u"install(DIRECTORY launch config" in cmake_text,
          u"安装 launch 与 config")
    check("K15", u"find_package(catkin REQUIRED COMPONENTS" in cmake_text,
          u"find_package 带组件")

    launch_text = io.open(launch, encoding="utf-8").read()
    check("K16", u'type="main.py"' in launch_text, u"节点入口是 main.py")
    check("K17", u"$(find steering_actuator_lag)/config/" in launch_text,
          u"从包里读 config")
    check("K18", u'command="load"' in launch_text, u"用 rosparam 载入 yaml")
    check("K19", launch_text.count(u"<arg ") == 13, u"13 个 launch 参数")
    check("K20", launch_text.count(u"doc=") >= 13, u"每个参数都有 doc")
    check("K21", launch_text.count(u'<param name="') == 13, u"13 个覆盖 param")
    check("K22", u"<arg name=\"time_constant\" default=\"0.25\"" in launch_text,
          u"时间常数默认值与 core 一致")
    check("K23", u"<arg name=\"dwell_seconds\" default=\"5.0\"" in launch_text,
          u"保持时长默认值与 core 一致")

    config_text = io.open(config, encoding="utf-8").read()
    for key in (u"time_constant", u"dead_band_deg", u"max_rate_deg",
                u"center_rate_deg", u"max_angle_deg", u"step_deg",
                u"dwell_seconds", u"steady_tolerance_deg",
                u"rise_limit_seconds", u"center_limit_seconds",
                u"center_tolerance_deg", u"step", u"publish_hz"):
        check("K24_%s" % key, key + u":" in config_text, u"yaml 含 %s" % key)

    core_text = io.open(core_py, encoding="utf-8").read()
    check("K25", u"\nimport rospy" not in core_text, u"core 不 import rospy")
    check("K26", u"\nimport numpy" not in core_text, u"core 不依赖 numpy")
    node_text = io.open(node_py, encoding="utf-8").read()
    check("K27", u"import actuator_core as core" in node_text,
          u"节点复用 core")
    check("K28", u"def _log(" in node_text, u"节点有 py2 安全的日志封装")
    check("K29", u'"log" + level' in node_text, u"日志函数名带 log 前缀")
    main_text = io.open(main_py, encoding="utf-8").read()
    check("K30", u"from actuator_node import main" in main_text,
          u"main.py 复用节点逻辑")

    docs_text = io.open(docs, encoding="utf-8").read()
    for name in core.ASSERT_NAMES:
        check("K31_%s" % name, name in docs_text, u"文档提到断言 %s" % name)
    check("K32", u"roslaunch steering_actuator_lag main.launch" in docs_text,
          u"文档给了启动命令")

    readme_text = io.open(readme, encoding="utf-8").read()
    check("K33", u"roslaunch steering_actuator_lag main.launch" in readme_text,
          u"README 给了启动命令")
    check("K34", u"test_steering_actuator_lag_local.py" in readme_text,
          u"README 提到离线测试")
    check("K35", u"大模型" in readme_text, u"README 有大模型声明")


# ---------------------------------------------------------------- 主流程
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


test_spec = _pytest_group(group_spec)
test_helpers = _pytest_group(group_helpers)
test_command = _pytest_group(group_command)
test_simulation = _pytest_group(group_simulation)
test_rise = _pytest_group(group_rise)
test_steady = _pytest_group(group_steady)
test_rate = _pytest_group(group_rate)
test_center = _pytest_group(group_center)
test_checks = _pytest_group(group_checks)
test_params = _pytest_group(group_params)
test_project_files = _pytest_group(group_files)


def main():
    print(u"=" * 60)
    print(u"steering_actuator_lag 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"=" * 60)

    group_spec()
    group_helpers()
    group_command()
    group_simulation()
    group_rise()
    group_steady()
    group_rate()
    group_center()
    group_checks()
    group_params()
    group_files()

    print(u"\n" + u"=" * 60)
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        for item in FAILED:
            print(u"  [失败] %s" % item)
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
