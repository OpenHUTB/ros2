#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车辆键盘遥操作模块本地测试（不需要 ROS / 仿真器 / 显示器 / pygame）.

分层之后核心逻辑都在 ``scripts/teleop_core.py``（不依赖 pygame 与 rospy），
所以这个测试只用标准库，任何装了 Python 3 的机器都能直接跑：

    python3 tests/test_vehicle_pygame_teleop_local.py

覆盖范围：
A. TeleopParams —— 默认值与原硬编码常量一致、能从参数服务器读并覆盖、布尔容错
B. 控制意图 —— KEY_BINDINGS 完整性与按键→意图映射
C. 运动学 step_vehicle —— 加速/限速、刹车、倒车、手刹、滑行、转向平滑、
   倒车转向反号、位置积分
D. 话题字段映射 twist_fields
E. 工程文件 —— package.xml / CMakeLists / main.launch / config yaml / 文档页
"""
# 与 teleop_core 一致：同时兼容 Python 2.7（Melodic）与 Python 3.x（Noetic）
from __future__ import absolute_import, division, print_function

import io
import math
import os
import sys
import xml.etree.ElementTree as ET

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "vehicle_pygame_teleop")
SCRIPTS = os.path.join(PKG, "scripts")
CONFIG = os.path.join(PKG, "config", "vehicle_pygame_teleop.yaml")
LAUNCH = os.path.join(PKG, "launch", "main.launch")
DOC = os.path.join(ROOT, "docs", "ground", "vehicle_pygame_teleop.md")
sys.path.insert(0, SCRIPTS)

import teleop_core as tc  # noqa: E402


# ---------------------------------------------------------------- 工具
def fake_get_param(values):
    """充当 rospy.get_param(name, default) 的最小替身。"""
    def get_param(name, default=None):
        return values.get(name, default)
    return get_param


def make_params(**overrides):
    """构造测试用参数：默认取 teleop_core 的默认值，可逐项覆盖。"""
    return tc.TeleopParams(**overrides)


def read(path):
    # 用 io.open 而不是内置 open：Python 2 的内置 open 不接受 encoding 参数
    with io.open(path, encoding="utf-8") as fp:
        return fp.read()


try:  # Python 2：文本类型是 unicode，str 是字节串
    _TEXT_TYPES = unicode  # noqa: F821
except NameError:  # Python 3：str 就是文本类型
    _TEXT_TYPES = (str,)


def text(value):
    """统一成文本类型。

    Python 2 下 teleop_core 里的字面量是字节串、而 read() 读进来的是 unicode，
    直接比较会触发隐式的 ASCII 解码，并在非 ASCII 字符（如按键表里的箭头）上报错，
    这里显式转换一次，让 Python 2 与 Python 3 行为一致。
    """
    if isinstance(value, _TEXT_TYPES):
        return value
    return value.decode("utf-8")


# ================================================================= A. 参数
def test_params():
    print("== A. TeleopParams 参数 ==")
    p = tc.TeleopParams()
    # 每一项都等于改造前的硬编码常量，保证默认行为不变
    check(p.max_speed == 1.5, "默认 max_speed = 1.5（原 MAX_SPEED）")
    check(p.max_reverse == 0.8, "默认 max_reverse = 0.8（原 MAX_REVERSE）")
    check(p.accel == 2.2, "默认 accel = 2.2（原 ACCEL）")
    check(p.brake == 4.5, "默认 brake = 4.5（原 BRAKE）")
    check(p.handbrake == 8.0, "默认 handbrake = 8.0（原 HANDBRAKE）")
    check(p.drag == 1.2, "默认 drag = 1.2（原 DRAG）")
    check(p.max_yaw_rate == 1.4, "默认 max_yaw_rate = 1.4（原 MAX_YAW_RATE）")
    check(p.yaw_response == 8.0, "默认 yaw_response = 8.0（原 YAW_RESPONSE）")
    check(p.reverse_steer_invert is True, "默认 reverse_steer_invert = True（原行为）")
    check(p.cmd_vel_topic == "/cmd_vel", "默认 cmd_vel_topic = /cmd_vel")
    check(p.publish_hz == 30.0, "默认 publish_hz = 30.0（原 FPS）")

    # 参数服务器给值 → 覆盖；未给 → 回落默认值
    p2 = tc.TeleopParams.from_params(fake_get_param({
        "vehicle/max_speed": 3.0,
        "vehicle/accel": "1.0",
        "topic/cmd_vel": "/car/cmd_vel",
        "rate/publish_hz": 50,
    }))
    check(p2.max_speed == 3.0, "from_params 覆盖 vehicle/max_speed")
    check(p2.accel == 1.0, "from_params 接受字符串数值（rosparam 兼容）")
    check(p2.cmd_vel_topic == "/car/cmd_vel", "from_params 覆盖 topic/cmd_vel")
    check(p2.publish_hz == 50.0, "from_params 覆盖 rate/publish_hz")
    check(p2.brake == 4.5, "未提供的参数回落默认值（brake）")
    check(p2.max_reverse == 0.8, "未提供的参数回落默认值（max_reverse）")

    # 布尔容错：rosparam 可能给 bool，也可能给字符串
    check(tc.TeleopParams.from_params(
        fake_get_param({"vehicle/reverse_steer_invert": False})
    ).reverse_steer_invert is False, "reverse_steer_invert 接受布尔 False")
    check(tc.TeleopParams.from_params(
        fake_get_param({"vehicle/reverse_steer_invert": "false"})
    ).reverse_steer_invert is False, "reverse_steer_invert 接受字符串 \"false\"")
    check(tc.TeleopParams.from_params(
        fake_get_param({"vehicle/reverse_steer_invert": "true"})
    ).reverse_steer_invert is True, "reverse_steer_invert 接受字符串 \"true\"")

    # 所有参数都从 yaml 里的键名读取
    names = []

    def spy(name, default=None):
        names.append(name)
        return default

    tc.TeleopParams.from_params(spy)
    for key in ("vehicle/max_speed", "vehicle/max_reverse", "vehicle/accel",
                "vehicle/brake", "vehicle/handbrake", "vehicle/drag",
                "vehicle/max_yaw_rate", "vehicle/yaw_response",
                "vehicle/reverse_steer_invert", "topic/cmd_vel", "rate/publish_hz"):
        check(key in names, "from_params 读取 %s" % key)


# ================================================================= B. 控制意图
def test_control_input():
    print("== B. KEY_BINDINGS 与控制意图 ==")
    actions = [a for a, _label, _keys in tc.KEY_BINDINGS]
    check(actions == list(tc.CONTROL_ACTIONS), "KEY_BINDINGS 覆盖全部五个控制分量")
    for action, label, keys in tc.KEY_BINDINGS:
        check(action in tc.KEY_BINDINGS_MAP, "KEY_BINDINGS_MAP 含 %s" % action)
        check(tc.KEY_BINDINGS_MAP[action] == keys, "%s 的键名表与映射一致" % action)
        check(len(keys) >= 1 and len(label) > 0, "%s 有按键与文档写法" % action)

    check(tc.KEY_BINDINGS_MAP["gas"] == ("w", "up"), "油门键 = W / ↑")
    check(tc.KEY_BINDINGS_MAP["brake"] == ("s", "down"), "刹车键 = S / ↓")
    check(tc.KEY_BINDINGS_MAP["left"] == ("a", "left"), "左转键 = A / ←")
    check(tc.KEY_BINDINGS_MAP["right"] == ("d", "right"), "右转键 = D / →")
    check(tc.KEY_BINDINGS_MAP["handbrake"] == ("space",), "手刹键 = 空格")

    c = tc.control_from_predicate(lambda n: n in ("w",))
    check(c.as_tuple() == (True, False, False, False, False), "按下 w → 只有 gas")
    c = tc.control_from_predicate(lambda n: n in ("up",))
    check(c.gas is True and c.brake is False, "按方向键 ↑ 与 w 等价")
    c = tc.control_from_predicate(lambda n: n in ("s", "down"))
    check(c.as_tuple() == (False, True, False, False, False), "按下 s → 只有 brake")
    c = tc.control_from_predicate(lambda n: n in ("a", "space"))
    check(c.as_tuple() == (False, False, True, False, True), "可同时按下多个键")
    c = tc.control_from_predicate(lambda n: False)
    check(c.as_tuple() == (False, False, False, False, False), "无按键 → 全 False")

    c = tc.ControlInput(gas=1, brake=0, left="", right=None, handbrake=[])
    check(c.as_tuple() == (True, False, False, False, False), "ControlInput 归一化为布尔")
    check("gas=True" in repr(tc.ControlInput(gas=True)), "ControlInput 可读的 repr")


# ================================================================= C. 运动学
def test_kinematics():
    print("== C. step_vehicle 运动学 ==")
    p = make_params()

    # C1 油门加速：一帧 = accel * dt
    st = tc.VehicleState()
    st.speed = 0.0
    tc.step_vehicle(st, tc.ControlInput(gas=True), 0.1, p)
    check(close(st.speed, 0.22), "油门 0.1s → 0.22 m/s（accel 2.2）")

    # 重复加速到上限后不再增长
    for _ in range(200):
        tc.step_vehicle(st, tc.ControlInput(gas=True), 0.1, p)
    check(st.speed == 1.5, "持续油门被 max_speed 限速在 1.5 m/s")

    # C2 前进中踩刹车
    st = tc.VehicleState(speed=1.0)
    tc.step_vehicle(st, tc.ControlInput(brake=True), 0.1, p)
    check(close(st.speed, 0.55), "1.0 m/s 刹车 0.1s → 0.55 m/s")

    # C3 静止时按下刹车 → 倒车
    st = tc.VehicleState(speed=0.0)
    tc.step_vehicle(st, tc.ControlInput(brake=True), 0.1, p)
    check(close(st.speed, -0.22), "静止踩刹车 → 倒车 -0.22 m/s")

    # C4 倒车被 max_reverse 限速
    st = tc.VehicleState(speed=-0.7)
    tc.step_vehicle(st, tc.ControlInput(brake=True), 0.1, p)
    check(st.speed == -0.8, "倒车被 max_reverse 限速在 -0.8 m/s")

    # C5 倒车中踩油门：先制动到 0，不直接前进
    st = tc.VehicleState(speed=-0.5)
    tc.step_vehicle(st, tc.ControlInput(gas=True), 0.1, p)
    check(close(st.speed, -0.05), "倒车中踩油门先制动（-0.5 → -0.05）")
    check(st.speed < 0.0, "倒车中踩油门不会瞬间变正速度")

    # C6 手刹急停，且不响应油门
    st = tc.VehicleState(speed=1.0)
    tc.step_vehicle(st, tc.ControlInput(handbrake=True), 0.1, p)
    check(close(st.speed, 0.2), "手刹 0.1s → 1.0 → 0.2 m/s（handbrake 8.0）")
    st = tc.VehicleState(speed=1.0)
    tc.step_vehicle(st, tc.ControlInput(gas=True, handbrake=True), 0.1, p)
    check(close(st.speed, 0.2), "手刹优先于油门")
    st = tc.VehicleState(speed=-1.0)
    for _ in range(20):
        tc.step_vehicle(st, tc.ControlInput(handbrake=True), 0.1, p)
    check(st.speed == 0.0, "倒车时手刹最终停在 0，不会过冲成正速度")
    st = tc.VehicleState(speed=1.0)
    for _ in range(20):
        tc.step_vehicle(st, tc.ControlInput(handbrake=True), 0.1, p)
    check(st.speed == 0.0, "前进时手刹最终停在 0，不会过冲成倒车")

    # C7 无按键滑行阻力
    st = tc.VehicleState(speed=0.5)
    tc.step_vehicle(st, tc.ControlInput(), 0.1, p)
    check(close(st.speed, 0.38), "松手滑行 0.1s → 0.5 → 0.38 m/s（drag 1.2）")
    st = tc.VehicleState(speed=-0.5)
    tc.step_vehicle(st, tc.ControlInput(), 0.1, p)
    check(close(st.speed, -0.38), "倒车滑行向 0 收敛")
    st = tc.VehicleState(speed=0.0)
    tc.step_vehicle(st, tc.ControlInput(), 0.1, p)
    check(st.speed == 0.0, "静止且无输入 → 保持 0（不自行漂移）")

    # C8 转向按 yaw_response 平滑逼近上限
    st = tc.VehicleState(speed=0.0)
    tc.step_vehicle(st, tc.ControlInput(left=True), 0.05, p)
    check(close(st.yaw_rate, 0.4), "左转 0.05s → 角速度 0.4 rad/s（response 8.0）")
    for _ in range(100):
        tc.step_vehicle(st, tc.ControlInput(left=True), 0.05, p)
    check(st.yaw_rate == 1.4, "持续左转收敛到 max_yaw_rate")
    st = tc.VehicleState(speed=0.0)
    for _ in range(100):
        tc.step_vehicle(st, tc.ControlInput(right=True), 0.05, p)
    check(st.yaw_rate == -1.4, "右转角速度为负（与文档一致）")

    # C9 倒车时转向反向（差速底盘特性），可用参数关闭
    st = tc.VehicleState(speed=-0.5)
    tc.step_vehicle(st, tc.ControlInput(left=True), 0.1, p)
    check(st.yaw_rate < 0.0, "倒车中左转 → 角速度为负（转向效果反向）")
    st = tc.VehicleState(speed=-0.5)
    tc.step_vehicle(st, tc.ControlInput(left=True), 0.1,
                    make_params(reverse_steer_invert=False))
    check(st.yaw_rate > 0.0, "关闭 reverse_steer_invert 后倒车左转为正")

    # C10 位置积分：直行
    st = tc.VehicleState(speed=1.0)
    tc.step_vehicle(st, tc.ControlInput(), 0.1, make_params(drag=0.0))
    check(close(st.x, 0.1) and close(st.y, 0.0), "yaw=0 直行 0.1s → x=0.1, y=0")
    check(close(st.yaw, 0.0), "直行不改变航向")

    # C11 位置积分：转向后沿圆弧前进
    st = tc.VehicleState(speed=1.0)
    tc.step_vehicle(st, tc.ControlInput(left=True), 0.1,
                    make_params(drag=0.0, yaw_response=1e6, max_yaw_rate=1.0))
    check(close(st.yaw_rate, 1.0), "大 response 下角速度一帧到上限")
    check(close(st.yaw, 0.1), "航向积分 yaw = yaw_rate * dt")
    check(close(st.x, 0.1 * math.cos(0.1)) and close(st.y, 0.1 * math.sin(0.1)),
          "位置按更新后的航向积分（与改造前一致）")

    # C12 dt=0 不改变任何状态
    st = tc.VehicleState(speed=1.0, yaw=0.3, yaw_rate=0.5)
    tc.step_vehicle(st, tc.ControlInput(gas=True, left=True), 0.0, p)
    check(close(st.x, 0.0) and close(st.y, 0.0) and close(st.yaw, 0.3),
          "dt=0 不推进位置与航向")

    # C13 参数化确实生效：改 max_speed 后限速点跟着变
    st = tc.VehicleState()
    for _ in range(200):
        tc.step_vehicle(st, tc.ControlInput(gas=True), 0.1, make_params(max_speed=4.0))
    check(st.speed == 4.0, "max_speed=4.0 时限速点变为 4.0（参数化生效）")

    # C14 step_vehicle 返回同一个 state 对象（就地更新）
    st = tc.VehicleState()
    check(tc.step_vehicle(st, tc.ControlInput(), 0.1, p) is st, "step_vehicle 就地更新并返回 state")


# ================================================================= D. 话题字段
def test_twist_fields():
    print("== D. twist_fields 话题字段 ==")
    st = tc.VehicleState(speed=1.25, yaw_rate=-0.75)
    linear_x, angular_z = tc.twist_fields(st)
    check(close(linear_x, 1.25), "linear.x = 纵向速度")
    check(close(angular_z, -0.75), "angular.z = 偏航角速度")
    st = tc.VehicleState()
    check(tc.twist_fields(st) == (0.0, 0.0), "静止时为零速度指令")


# ================================================================= E. 工程文件
def test_project_files():
    print("== E. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "vehicle_pygame_teleop", "package.xml 包名正确")
    depends = [d.text.strip() for d in pkg.findall("exec_depend")]
    for name in ("rospy", "geometry_msgs", "python3-pygame"):
        check(name in depends, "package.xml 声明 exec_depend %s" % name)

    cmake = read(os.path.join(PKG, "CMakeLists.txt"))
    for name in ("main.py", "teleop_core.py", "vehicle_pygame_teleop.py"):
        check(name in cmake, "CMakeLists 安装 %s" % name)
    check("install(DIRECTORY launch config" in cmake, "CMakeLists 安装 launch 与 config")

    launch_text = read(LAUNCH)
    launch = ET.parse(LAUNCH).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py（入口符合 main.* 约定）")
    check("vehicle_pygame_teleop.py" not in nodes,
          "main.launch 不再直接启动 vehicle_pygame_teleop.py")
    check("config/vehicle_pygame_teleop.yaml" in launch_text, "main.launch 加载 config yaml")
    args = [a.get("name") for a in launch.findall("arg")]
    for a in ("cmd_vel_topic", "publish_hz", "max_speed", "max_reverse", "accel",
              "brake", "handbrake", "drag", "max_yaw_rate", "yaw_response",
              "reverse_steer_invert"):
        check(a in args, "main.launch 暴露参数 %s" % a)
    check(all(a.get("doc") for a in launch.findall("arg")), "main.launch 每个参数都有 doc")
    params = [p.get("name") for p in launch.findall("param")]
    for name in ("topic/cmd_vel", "rate/publish_hz", "vehicle/max_speed",
                 "vehicle/reverse_steer_invert"):
        check(name in params, "main.launch 把 <arg> 覆盖到 %s" % name)

    yaml_text = read(CONFIG)
    for key in ("max_speed", "max_reverse", "accel", "brake", "handbrake", "drag",
                "max_yaw_rate", "yaw_response", "reverse_steer_invert",
                "publish_hz", "cmd_vel"):
        check(key in yaml_text, "config yaml 含 %s" % key)

    # 分层：核心逻辑不依赖 pygame / ROS，入口是薄壳
    core_src = read(os.path.join(SCRIPTS, "teleop_core.py"))
    check("import pygame" not in core_src, "teleop_core 不依赖 pygame（可离线测试）")
    check("import rospy" not in core_src, "teleop_core 不依赖 rospy（可离线测试）")
    node_src = read(os.path.join(SCRIPTS, "vehicle_pygame_teleop.py"))
    check("import teleop_core" in node_src, "节点复用 teleop_core 的运动学")
    entry_src = read(os.path.join(SCRIPTS, "main.py"))
    check("from vehicle_pygame_teleop import main" in entry_src,
          "main.py 是复用节点逻辑的入口")

    # 文档页存在，且按键表与代码里的 KEY_BINDINGS 一致
    check(os.path.isfile(DOC), "文档页 docs/ground/vehicle_pygame_teleop.md 存在")
    doc_text = read(DOC)
    for _action, label, _keys in tc.KEY_BINDINGS:
        check(text(label) in doc_text, "文档按键表含 %s" % label)


def main():
    test_params()
    test_control_input()
    test_kinematics()
    test_twist_fields()
    test_project_files()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for f in FAIL:
            print("  FAIL: %s" % f)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
