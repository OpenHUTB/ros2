#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""twist_to_control 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**；
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）；
* 在仓库根目录执行：python3 tests/test_twist_to_control_local.py

覆盖范围：
A 参数默认值          B 通用小工具        C 曲率换算
D 转角换算与限幅      E 阿克曼内外轮      F 差速左右轮速
G 单帧控制推进        H 内置运动学模拟    I 四元数
J 参数读取            K 工程文件约定
"""
from __future__ import division
from __future__ import print_function

import io
import math
import os
import sys

try:                      # Python 2 兼容
    text_type = unicode   # noqa: F821
except NameError:         # Python 3
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PKG = os.path.join(ROOT, "src", "ground", "twist_to_control")
SCRIPTS = os.path.join(PKG, "scripts")
CONFIG = os.path.join(PKG, "config")
LAUNCH = os.path.join(PKG, "launch")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "twist_to_control.md")
MKDOCS = os.path.join(ROOT, "mkdocs.yml")
INDEX = os.path.join(ROOT, "docs", "index.md")

sys.path.insert(0, SCRIPTS)

import twist_core as core  # noqa: E402


# ---------------------------------------------------------------- 测试框架
PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append("%s%s" % (name, ("  <- " + detail) if detail else ""))


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def check_close(name, actual, expected, tol=1e-6):
    check(name, close(actual, expected, tol),
          "实际 %.8f，期望 %.8f" % (actual, expected))


def text(value):
    """把字面量统一成 unicode，避免 Python 2 下的隐式 ASCII 解码错误。"""
    if isinstance(value, text_type):
        return value
    return value.decode("utf-8")


def read_text(path):
    if not os.path.isfile(path):
        return u""
    handle = io.open(path, "r", encoding="utf-8")
    try:
        return handle.read()
    finally:
        handle.close()


def make_params(**kwargs):
    return core.ControlParams(**kwargs)


def make_getter(mapping):
    """模拟 rospy.get_param(name, default)。"""
    def get_param(name, default=None):
        if name in mapping:
            return mapping[name]
        return default
    return get_param


# ---------------------------------------------------------------- A 默认值
def test_defaults():
    check("A01 默认底盘模型为 bicycle", core.DEFAULT_MODEL == "bicycle")
    check("A02 支持三种底盘模型",
          core.MODELS == ("bicycle", "ackermann", "diff_drive"))
    check_close("A03 默认轴距 2.7", core.DEFAULT_WHEELBASE, 2.7)
    check_close("A04 默认轮距 1.5", core.DEFAULT_TRACK_WIDTH, 1.5)
    check_close("A05 默认最大转角 0.55", core.DEFAULT_MAX_STEERING, 0.55)
    check_close("A06 默认最大曲率 1.0", core.DEFAULT_MAX_CURVATURE, 1.0)
    check_close("A07 默认最大速度 5.0", core.DEFAULT_MAX_SPEED, 5.0)
    check_close("A08 默认最大加速度 2.0", core.DEFAULT_MAX_ACCEL, 2.0)
    check_close("A09 默认转角速率 2.0", core.DEFAULT_MAX_STEERING_RATE, 2.0)
    check_close("A10 默认死区速度 0.05", core.DEFAULT_MIN_SPEED, 0.05)
    check_close("A11 默认超时 0.5", core.DEFAULT_TIMEOUT, 0.5)
    check_close("A12 默认发布频率 20", core.DEFAULT_PUBLISH_HZ, 20.0)
    check("A13 默认不内置模拟", core.DEFAULT_USE_MOCK_ODOM is False)

    params = core.ControlParams()
    check("A14 参数对象默认模型", params.model == "bicycle")
    check_close("A15 参数对象默认轴距", params.wheelbase, 2.7)
    check_close("A16 参数对象默认最大转角", params.max_steering, 0.55)
    check_close("A17 参数对象默认发布频率", params.publish_hz, 20.0)
    check("A18 默认 /cmd_vel 话题",
          params.cmd_vel_topic == "/cmd_vel")
    check("A19 默认控制量话题",
          params.control_topic == "/vehicle/control")
    check("A20 默认轮速话题",
          params.wheel_topic == "/vehicle/wheel_speed")
    check("A21 默认诊断话题",
          params.info_topic == "/vehicle/control_info")
    check("A22 默认里程计话题", params.odom_topic == "/odom")
    check("A23 非法模型回落 bicycle",
          core.ControlParams(model="tank").model == "bicycle")


# ---------------------------------------------------------------- B 小工具
def test_helpers():
    check_close("B01 clamp 下界", core.clamp(-3.0, -1.0, 1.0), -1.0)
    check_close("B02 clamp 上界", core.clamp(3.0, -1.0, 1.0), 1.0)
    check_close("B03 clamp 区间内不动", core.clamp(0.25, -1.0, 1.0), 0.25)
    nan = float("nan")
    check_close("B04 clamp 处理 NaN", core.clamp(nan, -1.0, 1.0), -1.0)

    check_close("B05 move_towards 增大", core.move_towards(0.0, 1.0, 0.3), 0.3)
    check_close("B06 move_towards 减小", core.move_towards(0.0, -1.0, 0.3), -0.3)
    check_close("B07 move_towards 到位不越过",
                core.move_towards(0.9, 1.0, 0.5), 1.0)
    check_close("B08 move_towards 负步长按绝对值",
                core.move_towards(0.0, 1.0, -0.3), 0.3)

    check("B09 as_bool 真值字符串", core.as_bool("true") is True)
    check("B10 as_bool 大写", core.as_bool("TRUE") is True)
    check("B11 as_bool 数字 1", core.as_bool(1) is True)
    check("B12 as_bool 数字 0", core.as_bool(0) is False)
    check("B13 as_bool false 字符串", core.as_bool("false") is False)
    check("B14 as_bool None", core.as_bool(None) is False)
    check("B15 as_bool 布尔直传", core.as_bool(True) is True)

    check_close("B16 as_float 正常", core.as_float("0.75", 1.0), 0.75)
    check_close("B17 as_float 非法回落", core.as_float("abc", 1.0), 1.0)
    check_close("B18 as_float None 回落", core.as_float(None, 2.5), 2.5)


# ---------------------------------------------------------------- C 曲率
def test_curvature():
    check_close("C01 1m/s 转 0.5rad/s 曲率 0.5",
                core.curvature_from_twist(1.0, 0.5), 0.5)
    check_close("C02 2m/s 转 0.5rad/s 曲率 0.25",
                core.curvature_from_twist(2.0, 0.5), 0.25)
    check_close("C03 倒车时曲率反号",
                core.curvature_from_twist(-1.0, 0.5), -0.5)
    check_close("C04 直行曲率为 0",
                core.curvature_from_twist(2.0, 0.0), 0.0)
    check("C05 速度低于死区返回 None",
          core.curvature_from_twist(0.01, 0.5) is None)
    check("C06 负速度绝对值低于死区也返回 None",
          core.curvature_from_twist(-0.01, 0.5) is None)
    check("C07 零速度返回 None",
          core.curvature_from_twist(0.0, 0.5) is None)
    check_close("C08 死区边界之上可用",
                core.curvature_from_twist(0.06, 0.03), 0.5)
    check_close("C09 自定义死区生效",
                core.curvature_from_twist(0.5, 1.0, min_speed=0.6) is None
                and 0.0 or 0.0, 0.0)


# ---------------------------------------------------------------- D 转角
def test_steering():
    angle, saturated = core.steering_from_curvature(0.2, 2.7, 1.0, 0.55)
    check_close("D01 atan(0.2*2.7) 转角", angle, math.atan(0.54))
    check("D02 未饱和", saturated is False)

    angle, saturated = core.steering_from_curvature(0.0, 2.7, 1.0, 0.55)
    check_close("D03 曲率为 0 转角为 0", angle, 0.0)
    check("D04 直行不饱和", saturated is False)

    angle, saturated = core.steering_from_curvature(0.2, 2.7, 1.0, 0.55)
    check("D05 小曲率不触发限幅", saturated is False)

    angle, saturated = core.steering_from_curvature(1.5, 2.7, 1.0, 0.55)
    check_close("D06 超出最大曲率被截断到最大转角", angle, 0.55)
    check("D07 上报饱和", saturated is True)

    angle, saturated = core.steering_from_curvature(0.2, 2.7, 0.1, 0.55)
    check_close("D08 曲率限幅后转角变小", angle, math.atan(0.27))
    check("D09 曲率限幅也算饱和", saturated is True)

    angle, saturated = core.steering_from_curvature(-0.2, 2.7, 1.0, 0.55)
    check_close("D10 负曲率转角为负", angle, -math.atan(0.54))

    angle, saturated = core.steering_from_curvature(-1.5, 2.7, 1.0, 0.55)
    check_close("D11 负向限幅", angle, -0.55)
    check("D12 负向限幅上报饱和", saturated is True)

    angle, saturated = core.steady_steering(2.0, 0.2, make_params())
    check_close("D13 稳态转角便捷函数", angle, math.atan(0.1 * 2.7))
    check("D14 稳态函数不饱和", saturated is False)

    angle, saturated = core.steady_steering(0.01, 0.5, make_params())
    check_close("D15 死区内稳态转角为 0", angle, 0.0)


# ---------------------------------------------------------------- E 阿克曼
def test_ackermann():
    left, right = core.ackermann_wheel_angles(0.2, 2.7, 1.5)
    inner = math.atan(0.54 / 0.85)
    outer = math.atan(0.54 / 1.15)
    check_close("E01 左转时左轮为内侧", left, inner)
    check_close("E02 左转时右轮为外侧", right, outer)
    check("E03 内侧转角大于外侧", left > right)
    check("E04 内侧转角大于单车模型转角", left > math.atan(0.54))
    check("E05 外侧转角小于单车模型转角", right < math.atan(0.54))

    left, right = core.ackermann_wheel_angles(-0.2, 2.7, 1.5)
    check_close("E06 右转时左轮为外侧", left, -outer)
    check_close("E07 右转时右轮为内侧", right, -inner)
    check("E08 右转内侧角度更大", abs(right) > abs(left))

    left, right = core.ackermann_wheel_angles(0.2, 2.7, 0.0)
    check_close("E09 轮距为 0 退化为单车模型（左）", left, math.atan(0.54))
    check_close("E10 轮距为 0 退化为单车模型（右）", right, math.atan(0.54))

    left, right = core.ackermann_wheel_angles(0.0, 2.7, 1.5)
    check_close("E11 直行左右轮同角", left, 0.0)
    check_close("E12 直行右轮也为 0", right, 0.0)

    left, right = core.ackermann_wheel_angles(5.0, 2.7, 1.5)
    check("E13 超大曲率不产生除零", abs(left) < 2.0 and abs(right) < 2.0)
    check("E14 超大曲率结果有限", not math.isinf(left) and not math.isinf(right))


# ---------------------------------------------------------------- F 差速
def test_diff_drive():
    left, right = core.diff_drive_wheel_speeds(1.0, 0.5, 1.5)
    check_close("F01 左轮减速", left, 0.625)
    check_close("F02 右轮加速", right, 1.375)
    check_close("F03 两轮平均等于车体速度", (left + right) / 2.0, 1.0)
    check_close("F04 两轮差等于角速度乘轮距", right - left, 0.5 * 1.5)

    left, right = core.diff_drive_wheel_speeds(1.0, 0.0, 1.5)
    check_close("F05 直行左右同速（左）", left, 1.0)
    check_close("F06 直行左右同速（右）", right, 1.0)

    left, right = core.diff_drive_wheel_speeds(2.0, -0.25, 1.5)
    check_close("F07 右转左轮更快", left, 2.375)
    check_close("F08 右转右轮更慢", right, 1.625)

    left, right = core.diff_drive_wheel_speeds(-1.0, 0.5, 1.5)
    check_close("F09 倒车左转左轮", left, -0.625)
    check_close("F10 倒车左转右轮", right, -1.375)

    left, right = core.diff_drive_wheel_speeds(0.0, 0.5, 1.5)
    check_close("F11 零速时两轮为零（左）", left, 0.0)
    check_close("F12 零速时两轮为零（右）", right, 0.0)


# ---------------------------------------------------------------- G 单帧推进
def test_step_control():
    params = make_params(max_accel=2.0, max_speed=5.0)
    state = core.ControlState()
    core.step_control(state, core.TwistCommand(1.0, 0.0), 0.1, params)
    check_close("G01 油门 0.1 秒加速到 0.2", state.speed, 0.2)

    for _ in range(5):
        core.step_control(state, core.TwistCommand(1.0, 0.0), 0.1, params)
    check_close("G02 多帧后到达目标速度", state.speed, 1.0)

    state = core.ControlState()
    fast = make_params(max_accel=2.0, max_speed=1.0)
    for _ in range(20):
        core.step_control(state, core.TwistCommand(50.0, 0.0), 0.1, fast)
    check_close("G03 速度被最大速度限幅", state.speed, 1.0)

    state = core.ControlState()
    for _ in range(30):
        core.step_control(state, core.TwistCommand(-3.0, 0.0), 0.1, params)
    check_close("G04 支持倒车", state.speed, -3.0)

    state = core.ControlState()
    core.step_control(state, core.TwistCommand(1.0, 0.5), 0.1, params)
    check_close("G05 曲率等于角速度除以线速度", state.curvature, 0.5)
    check("G06 正常速度下不锁存", state.latched is False)

    state = core.ControlState()
    core.step_control(state, core.TwistCommand(1.0, 0.5), 0.1, params)
    core.step_control(state, core.TwistCommand(0.02, 0.5), 0.1, params)
    check_close("G07 死区内锁存上一帧曲率", state.curvature, 0.5)
    check("G08 死区标记为锁存", state.latched is True)

    state = core.ControlState()
    latched = make_params(max_steering_rate=2.0)
    core.step_control(state, core.TwistCommand(1.0, 0.2), 0.1, latched)
    check_close("G09 转角速率限制后首帧转角", state.steering, 0.2)
    check("G10 标记为速率受限", state.rate_limited is True)
    check_close("G11 记录下发的请求转角", state.requested_steering,
                math.atan(0.2 * 2.7))

    for _ in range(20):
        core.step_control(state, core.TwistCommand(1.0, 0.2), 0.1, latched)
    check_close("G12 多帧后转角收敛到目标", state.steering,
                math.atan(0.2 * 2.7))
    check("G13 收敛后不再标记速率受限", state.rate_limited is False)

    state = core.ControlState()
    narrow = make_params(max_steering=0.1)
    for _ in range(50):
        core.step_control(state, core.TwistCommand(1.0, 0.5), 0.1, narrow)
    check_close("G14 超过最大转角被限制", state.steering, 0.1)
    check("G15 标记饱和", state.saturated is True)

    state = core.ControlState()
    core.step_control(state, core.TwistCommand(1.0, 0.5), 0.0, params)
    check_close("G16 dt 为 0 时不推进速度", state.speed, 0.0)
    check_close("G17 dt 为 0 时不推进转角", state.steering, 0.0)

    state = core.ControlState()
    core.step_control(state, core.TwistCommand(1.0, 2.0), 0.1, params)
    check_close("G18 曲率被最大曲率截断", state.curvature, 1.0)

    state = core.ControlState()
    core.step_control(state, core.TwistCommand(1.0, 0.5), 0.1, params)
    check_close("G19 control_fields 转角", core.control_fields(state)[0],
                state.steering)
    check_close("G20 control_fields 速度", core.control_fields(state)[1], 0.2)
    check_close("G21 control_fields 曲率", core.control_fields(state)[2], 0.5)

    state = core.ControlState()
    for _ in range(30):
        core.step_control(state, core.TwistCommand(1.0, 0.2), 0.1, params)
    info = core.info_fields(state)
    check("G22 info_fields 返回五元组", len(info) == 5)
    check_close("G23 info_fields 请求转角与实发一致", info[0], info[1])
    check_close("G24 info_fields 实发转角", info[1], state.steering)
    check_close("G25 info_fields 未超限时饱和位为 0", info[2], 0.0)
    check_close("G26 info_fields 未锁存时为 0", info[3], 0.0)

    hard = core.ControlState()
    tight = make_params(max_steering=0.2)
    for _ in range(30):
        core.step_control(hard, core.TwistCommand(1.0, 0.5), 0.1, tight)
    hard_info = core.info_fields(hard)
    check("G26b 超限时饱和位为 1", hard_info[2] == 1.0)
    check("G26c 请求转角大于实发转角", abs(hard_info[0]) > abs(hard_info[1]))

    state = core.ControlState()
    core.step_control(state, core.TwistCommand(0.0, 0.0), 0.1, params)
    check_close("G27 零指令保持静止", state.speed, 0.0)

    state = core.ControlState(speed=1.0, steering=0.3, latched_curvature=0.25)
    state.reset()
    check_close("G28 reset 清零速度", state.speed, 0.0)
    check_close("G29 reset 清零锁存曲率", state.latched_curvature, 0.0)
    check("G30 reset 后不是锁存态", state.latched is False)


# ---------------------------------------------------------------- H 模拟车辆
def test_mock():
    params = make_params(model="bicycle")
    pose = core.Pose()
    state = core.ControlState(speed=1.0, steering=0.0, curvature=0.0)
    core.mock_step(pose, state, 0.1, params)
    check_close("H01 直行前进一步 x", pose.x, 0.1)
    check_close("H02 直行不产生横移", pose.y, 0.0)
    check_close("H03 直行不改变航向", pose.yaw, 0.0)

    pose = core.Pose()
    state = core.ControlState(speed=1.0, steering=0.2, curvature=0.0)
    core.mock_step(pose, state, 0.1, params)
    check_close("H04 左转产生正角速度", pose.yaw,
                math.tan(0.2) / 2.7 * 0.1)

    circle_params = make_params(model="diff_drive", max_accel=1e6,
                                max_speed=10.0)
    pose = core.Pose()
    state = core.ControlState()
    command = core.TwistCommand(1.0, 0.5)
    dt = 0.01
    period = 2.0 * math.pi / 0.5
    steps = int(round(period / dt))
    half = steps // 2
    for index in range(steps):
        core.step_control(state, command, dt, circle_params)
        core.mock_step(pose, state, dt, circle_params)
        if index == half:
            distance_mid = math.hypot(pose.x, pose.y)
    check_close("H05 走完半圈到达直径处", distance_mid, 4.0, 0.05)
    check_close("H06 走完一圈回到起点 x", pose.x, 0.0, 0.05)
    check_close("H07 走完一圈回到起点 y", pose.y, 0.0, 0.05)
    check_close("H08 走完一圈航向增加 2pi", pose.yaw, 2.0 * math.pi, 0.05)

    pose = core.Pose()
    state = core.ControlState(speed=1.0, steering=0.0, curvature=0.0)
    core.mock_step(pose, state, 0.0, params)
    check_close("H09 dt 为 0 时位姿不变", pose.x, 0.0)

    pose = core.Pose()
    state = core.ControlState(speed=-1.0, steering=0.2, curvature=0.0)
    core.mock_step(pose, state, 0.1, params)
    check("H10 倒车时航向反向变化", pose.yaw < 0.0)

    quat = core.Pose(x=1.0, y=2.0, yaw=0.3)
    check("H11 Pose 三元组", quat.as_tuple() == (1.0, 2.0, 0.3))


# ---------------------------------------------------------------- I 四元数
def test_quaternion():
    x, y, z, w = core.yaw_to_quaternion(0.0)
    check_close("I01 零偏航四元数 x", x, 0.0)
    check_close("I02 零偏航四元数 z", z, 0.0)
    check_close("I03 零偏航四元数 w", w, 1.0)

    yaw = 0.7
    x, y, z, w = core.yaw_to_quaternion(yaw)
    check_close("I04 往返转换", core.quaternion_to_yaw(x, y, z, w), yaw)

    yaw = -1.2
    x, y, z, w = core.yaw_to_quaternion(yaw)
    check_close("I05 负角往返转换", core.quaternion_to_yaw(x, y, z, w), yaw)

    check_close("I06 四元数模为 1",
                math.sqrt(x * x + y * y + z * z + w * w), 1.0)


# ---------------------------------------------------------------- J 参数读取
def test_from_params():
    params = core.ControlParams.from_params(make_getter({}))
    check("J01 无参数时回落默认模型", params.model == "bicycle")
    check_close("J02 无参数时回落默认轴距", params.wheelbase, 2.7)
    check_close("J03 无参数时回落默认频率", params.publish_hz, 20.0)
    check("J04 无参数时默认不模拟", params.use_mock_odom is False)

    mapping = {
        "model/type": "diff_drive",
        "model/wheelbase": 3.1,
        "model/track_width": 1.8,
        "limit/max_steering": 0.4,
        "limit/max_curvature": 0.8,
        "limit/max_speed": 8.0,
        "limit/max_accel": 1.5,
        "limit/max_steering_rate": 1.2,
        "limit/min_speed": 0.1,
        "control/timeout": 1.5,
        "rate/publish_hz": 50.0,
        "topic/cmd_vel": "/vel",
        "topic/control": "/ctl",
        "topic/wheel_speed": "/wheel",
        "topic/control_info": "/info",
        "topic/odom": "/odom_custom",
        "mock/enabled": True,
    }
    params = core.ControlParams.from_params(make_getter(mapping))
    check("J05 读取模型", params.model == "diff_drive")
    check_close("J06 读取轴距", params.wheelbase, 3.1)
    check_close("J07 读取轮距", params.track_width, 1.8)
    check_close("J08 读取最大转角", params.max_steering, 0.4)
    check_close("J09 读取最大曲率", params.max_curvature, 0.8)
    check_close("J10 读取最大速度", params.max_speed, 8.0)
    check_close("J11 读取最大加速度", params.max_accel, 1.5)
    check_close("J12 读取转角速率", params.max_steering_rate, 1.2)
    check_close("J13 读取死区速度", params.min_speed, 0.1)
    check_close("J14 读取超时", params.timeout, 1.5)
    check_close("J15 读取发布频率", params.publish_hz, 50.0)
    check("J16 读取 /cmd_vel 话题", params.cmd_vel_topic == "/vel")
    check("J17 读取控制量话题", params.control_topic == "/ctl")
    check("J18 读取轮速话题", params.wheel_topic == "/wheel")
    check("J19 读取诊断话题", params.info_topic == "/info")
    check("J20 读取里程计话题", params.odom_topic == "/odom_custom")
    check("J21 读取布尔型模拟开关", params.use_mock_odom is True)

    params = core.ControlParams.from_params(
        make_getter({"rate/publish_hz": "30", "limit/max_speed": "2.5"}))
    check_close("J22 字符串数值可解析（频率）", params.publish_hz, 30.0)
    check_close("J23 字符串数值可解析（速度）", params.max_speed, 2.5)

    params = core.ControlParams.from_params(
        make_getter({"mock/enabled": "true"}))
    check("J24 字符串布尔可解析", params.use_mock_odom is True)

    params = core.ControlParams.from_params(
        make_getter({"model/type": "unknown"}))
    check("J25 非法模型回落", params.model == "bicycle")


# ---------------------------------------------------------------- K 工程文件
def test_project_files():
    package_xml = read_text(os.path.join(PKG, "package.xml"))
    check("K01 存在 package.xml", bool(package_xml))
    check("K02 包名为 twist_to_control",
          text("<name>twist_to_control</name>") in package_xml)
    check("K03 依赖 rospy", text('exec_depend>rospy<') in package_xml)
    check("K04 依赖 geometry_msgs",
          text("geometry_msgs") in package_xml)
    check("K05 依赖 std_msgs", text("std_msgs") in package_xml)
    check("K06 依赖 nav_msgs", text("nav_msgs") in package_xml)
    check("K07 声明许可证", text("<license>") in package_xml)

    cmake = read_text(os.path.join(PKG, "CMakeLists.txt"))
    check("K08 存在 CMakeLists.txt", bool(cmake))
    check("K09 find_package 含 rospy", text("find_package") in cmake)
    check("K10 安装 main.py", text("scripts/main.py") in cmake)
    check("K11 安装 twist_core.py", text("scripts/twist_core.py") in cmake)
    check("K12 安装 twist_node.py", text("scripts/twist_node.py") in cmake)
    check("K13 安装 launch 与 config",
          text("install(DIRECTORY launch config") in cmake)

    launch = read_text(os.path.join(LAUNCH, "main.launch"))
    check("K14 存在 main.launch", bool(launch))
    check("K15 节点入口为 main.py", text('type="main.py"') in launch)
    check("K16 不再直接启动 twist_node.py",
          text('type="twist_node.py"') not in launch)
    check("K17 加载 config yaml",
          text("config/twist_to_control.yaml") in launch)
    check("K18 暴露模型参数", text('name="model"') in launch)
    check("K19 暴露轴距参数", text('name="wheelbase"') in launch)
    check("K20 暴露最大转角参数", text('name="max_steering"') in launch)
    check("K21 暴露发布频率参数", text('name="publish_hz"') in launch)
    check("K22 暴露模拟开关", text('name="use_mock_odom"') in launch)
    check("K23 每个 arg 都有 doc", launch.count(text("<arg")) ==
          launch.count(text("doc=")))

    yaml_text = read_text(os.path.join(CONFIG, "twist_to_control.yaml"))
    check("K24 存在 config yaml", bool(yaml_text))
    for key in ("model", "wheelbase", "track_width", "max_steering",
                "max_curvature", "max_speed", "max_accel",
                "max_steering_rate", "min_speed", "timeout",
                "publish_hz", "cmd_vel", "control", "wheel_speed",
                "control_info", "odom", "enabled"):
        check("K25 yaml 含键 %s" % key, text(key + ":") in yaml_text)

    core_src = read_text(os.path.join(SCRIPTS, "twist_core.py"))
    check("K26 核心逻辑不 import rospy",
          (text("\nimport rospy") not in core_src
           and text("\nfrom rospy") not in core_src))
    check("K27 核心逻辑不 import numpy",
          text("import numpy") not in core_src)
    check("K28 核心逻辑定义了 step_control",
          text("def step_control") in core_src)

    node_src = read_text(os.path.join(SCRIPTS, "twist_node.py"))
    check("K29 节点存在", bool(node_src))
    check("K30 节点用到 twist_core", text("twist_core") in node_src)
    check("K31 节点订阅 cmd_vel", text("/cmd_vel") in node_src
          or text("params.cmd_vel_topic") in node_src)
    check("K32 节点有 main 函数", text("def main(") in node_src)
    check("K33 节点注册退出清理",
          text("on_shutdown") in node_src)

    main_src = read_text(os.path.join(SCRIPTS, "main.py"))
    check("K34 入口 main.py 存在", bool(main_src))
    check("K35 入口复用节点逻辑", text("twist_node") in main_src)

    readme = read_text(os.path.join(PKG, "README.md"))
    check("K36 存在 README", bool(readme))
    check("K37 README 写了运行命令", text("roslaunch") in readme)
    check("K38 README 写了离线测试命令",
          text("test_twist_to_control_local.py") in readme)

    page = read_text(DOCS_PAGE)
    check("K39 文档页存在", bool(page))
    check("K40 文档页保留原标题", page.startswith(text("# 扭转控制")))
    check("K41 文档页保留原参考链接",
          text("carla_twist_to_control") in page)
    check("K42 文档页含自主实现章节", text("## 自主实现") in page)
    check("K43 文档页含参数表", text("max_steering") in page)
    check("K44 文档页含运动学公式", text("atan") in page)
    check("K45 文档页含大模型声明", text("大模型") in page)

    mkdocs_text = read_text(MKDOCS)
    if mkdocs_text:
        check("K46 导航已加扭转控制条目",
              text("ground/twist_to_control.md") in mkdocs_text)
        check("K47 导航条目写作 ./ground/twist_to_control.md",
              text("'./ground/twist_to_control.md'") in mkdocs_text)
    else:
        # 上传目录刻意不放 mkdocs.yml：上游该文件天天在变，整体覆盖会冲掉别人的
        # 导航条目，正确做法是在 GitHub 网页编辑器里手加一行。这里改判两条本地约定。
        check("K46 本地不含 mkdocs.yml（导航需在网页编辑器手动加一行）", True)
        check("K47 本地不含 docs/index.md（首页已有链接，避免误覆盖）",
              not os.path.isfile(INDEX))

    # 首页（docs/index.md）上游已经存在「扭转控制」链接，本模块不需要修改它；
    # 这里反向校验本地**没有**放该文件，避免整体拖拽上传时误覆盖首页。
    check("K48 本地不含 docs/index.md（首页无需改动，防止误覆盖）",
          not os.path.isfile(INDEX))
    check("K49 本地只放了需要上传的文件",
          not os.path.isfile(os.path.join(ROOT, "docs", "index.md")))


# ---------------------------------------------------------------- 主流程
def main():
    print("twist_to_control 离线回归测试")
    print("Python %s" % sys.version.split()[0])
    print("包目录 %s" % PKG)
    print("-" * 46)

    test_defaults()
    test_helpers()
    test_curvature()
    test_steering()
    test_ackermann()
    test_diff_drive()
    test_step_control()
    test_mock()
    test_quaternion()
    test_from_params()
    test_project_files()

    print("=" * 46)
    print("通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        for item in FAILED:
            print("  [失败] %s" % item)
        print("存在失败项")
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
