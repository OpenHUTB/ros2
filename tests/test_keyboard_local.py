#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制模块本地测试（桩依赖，不需要 ROS / 仿真器 / 终端）.

A. key_to_cmd 单键映射：W/S/A/D/R/F/Q/E -> 机体系速度分量与符号
B. merge_target 组合键：多键叠加、互斥抵消、未知键忽略
C. KeyTracker 按键保持：自动重复续期、超时判松开
D. VelocitySmoother 平滑：按加速度斜坡逼近、dt 上限、accel<=0 关闭平滑
E. build_twist / step_once：字段映射、组合键下发、松开后减速
F. ESC 转义序列识别：方向键不退出、不产生运动指令
G. 工程文件：package.xml / CMakeLists / main.launch / keyboard.yaml

用法: python3 tests/test_keyboard_local.py
"""
from __future__ import annotations

import os
import sys
import types
import xml.etree.ElementTree as ET

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


def eq(got, want, tol=1e-9):
    """浮点序列比较：加速度斜坡会引入 1e-16 量级的二进制误差."""
    try:
        return len(got) == len(want) and all(abs(a - b) <= tol for a, b in zip(got, want))
    except TypeError:
        return False


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "uav_keyboard_control")
SCRIPTS = os.path.join(PKG, "scripts")
sys.path.insert(0, SCRIPTS)


# ---------------------------------------------------------------- 消息桩
class _Vec3(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist(object):
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


# ---------------------------------------------------------------- rospy 桩
PARAMS = {
    "control/horiz_speed": 2.0, "control/vert_speed": 1.0, "control/yaw_rate": 1.0,
    "control/key_hold_timeout": 0.15,
    "control/accel_horiz": 4.0, "control/accel_vert": 2.0, "control/accel_yaw": 4.0,
    "topic/cmd_vel": "/uav/cmd_vel", "rate/publish_hz": 30.0,
}
PUBS = {}


class _Publisher(object):
    def __init__(self, name, msg_type, queue_size=1):
        self.name = name
        self.msgs = []
        PUBS[name] = self

    def publish(self, msg):
        self.msgs.append(msg)


class _Rate(object):
    def __init__(self, hz):
        self.hz = hz

    def sleep(self):
        pass


def install_stubs():
    m = types.ModuleType("rospy")
    m.get_param = lambda name, default=None: PARAMS.get(name, default)
    m.Publisher = _Publisher
    m.Rate = _Rate
    m.Subscriber = lambda *a, **k: None
    m.init_node = lambda *a, **k: None
    m.is_shutdown = lambda: False
    m.loginfo = lambda *a, **k: None
    m.logwarn = lambda *a, **k: None
    sys.modules["rospy"] = m

    gm = types.ModuleType("geometry_msgs")
    gmm = types.ModuleType("geometry_msgs.msg")
    gmm.Twist = _Twist
    gm.msg = gmm
    sys.modules["geometry_msgs"] = gm
    sys.modules["geometry_msgs.msg"] = gmm

    nm = types.ModuleType("nav_msgs")
    nmm = types.ModuleType("nav_msgs.msg")
    nmm.Odometry = type("Odometry", (object,), {})
    nm.msg = nmm
    sys.modules["nav_msgs"] = nm
    sys.modules["nav_msgs.msg"] = nmm


install_stubs()
import keyboard_control as kc  # noqa: E402


# ================================================================= A. 单键映射
def test_key_mapping():
    print("== A. key_to_cmd 单键映射 ==")
    check(kc.key_to_cmd("w") == (2.0, 0.0, 0.0, 0.0), "W -> 前进 (vx=+2.0)")
    check(kc.key_to_cmd("s") == (-2.0, 0.0, 0.0, 0.0), "S -> 后退 (vx=-2.0)")
    check(kc.key_to_cmd("a") == (0.0, 2.0, 0.0, 0.0), "A -> 左移 (vy=+2.0)")
    check(kc.key_to_cmd("d") == (0.0, -2.0, 0.0, 0.0), "D -> 右移 (vy=-2.0)")
    check(kc.key_to_cmd("r") == (0.0, 0.0, 1.0, 0.0), "R -> 上升 (vz=+1.0)")
    check(kc.key_to_cmd("f") == (0.0, 0.0, -1.0, 0.0), "F -> 下降 (vz=-1.0)")
    check(kc.key_to_cmd("q") == (0.0, 0.0, 0.0, 1.0), "Q -> 右转 (wz=+1.0)")
    check(kc.key_to_cmd("e") == (0.0, 0.0, 0.0, -1.0), "E -> 左转 (wz=-1.0)")

    check(kc.key_to_cmd("") == (0.0, 0.0, 0.0, 0.0), "空键 -> 全零（悬停）")
    check(kc.key_to_cmd(None) == (0.0, 0.0, 0.0, 0.0), "None -> 全零（悬停）")
    check(kc.key_to_cmd("x") is None, "未知键 -> None")
    check(kc.key_to_cmd("\x1b") is None, "ESC 转义 -> None（由 spin 单独处理退出）")

    check(kc.key_to_cmd("W", 3.0, 2.0, 0.5) == (3.0, 0.0, 0.0, 0.0), "速度参数可配置（大写键也识别）")
    check(kc.key_to_cmd("q", 3.0, 2.0, 0.5) == (0.0, 0.0, 0.0, 0.5), "偏航角速度参数可配置")


# ================================================================= B. 组合键
def test_merge_target():
    print("== B. merge_target 组合键 ==")
    check(kc.merge_target(["w", "a"]) == (2.0, 2.0, 0.0, 0.0), "W+A -> 斜向（vx=+2, vy=+2）")
    check(kc.merge_target(["w", "d"]) == (2.0, -2.0, 0.0, 0.0), "W+D -> 斜向（vx=+2, vy=-2）")
    check(kc.merge_target(["w", "q"]) == (2.0, 0.0, 0.0, 1.0), "W+Q -> 边走边右转")
    check(kc.merge_target(["r", "e"]) == (0.0, 0.0, 1.0, -1.0), "R+E -> 上升同时左转")
    check(kc.merge_target(["w", "s"]) == (0.0, 0.0, 0.0, 0.0), "W+S 互斥 -> 抵消为 0")
    check(kc.merge_target(["a", "d", "r"]) == (0.0, 0.0, 1.0, 0.0), "A+D 抵消，R 仍生效")
    check(kc.merge_target(["w", "x"]) == (2.0, 0.0, 0.0, 0.0), "未知键被忽略，不污染其它轴")
    check(kc.merge_target([]) == (0.0, 0.0, 0.0, 0.0), "空集合 -> 悬停（全零）")
    check(kc.merge_target(["W", "A"]) == (2.0, 2.0, 0.0, 0.0), "大写字母也识别")
    check(kc.merge_target(["w", "a"], 3.0, 2.0, 0.5) == (3.0, 3.0, 0.0, 0.0),
          "组合键沿用自定义速度参数")


# ================================================================= C. 按键保持
def test_key_tracker():
    print("== C. KeyTracker 按键保持 ==")
    t = kc.KeyTracker(hold_timeout=0.15)
    t.press("w", 1.00)
    check(t.active(1.05) == ("w",), "按住未超时 -> 仍激活")
    check(t.active(1.20) == (), "超过 hold_timeout 未再收到 -> 判松开")

    t = kc.KeyTracker(0.15)
    t.press("w", 1.00)
    t.press("w", 1.10)
    check(t.active(1.20) == ("w",), "自动重复续期 -> 一直是按住")

    t = kc.KeyTracker(0.5)
    t.press("w", 1.00)
    t.press("a", 1.10)
    check(t.active(1.25) == ("a", "w"), "多键同时按住 -> 集合包含两键")
    check(t.active(1.55) == ("a",), "先松开的键先过期，互不影响")

    t = kc.KeyTracker(0.15)
    t.press("W", 1.00)
    check(t.active(1.01) == ("w",), "按键统一小写保存")
    t.press("x", 1.01)
    check(t.active(1.02) == ("w",), "未知键不进入保持集合")

    t.release_all()
    check(t.active(1.02) == (), "release_all 清空保持状态")


# ================================================================= D. 平滑
def test_smoother():
    print("== D. VelocitySmoother 加速度斜坡 ==")
    s = kc.VelocitySmoother(4.0, 2.0, 4.0)
    check(eq(s.update((2.0, 0.0, 0.0, 0.0), 0.1), (0.4, 0.0, 0.0, 0.0)),
          "首帧 0.1 s：vx = 0 + 4*0.1 = 0.4")
    check(eq(s.update((2.0, 0.0, 0.0, 0.0), 0.1), (0.8, 0.0, 0.0, 0.0)), "第二帧累加到 0.8")
    for _ in range(5):
        s.update((2.0, 0.0, 0.0, 0.0), 0.1)
    check(eq(s.cmd, (2.0, 0.0, 0.0, 0.0)), "斜坡到目标值后不再超调")

    check(eq(s.update((0.0, 0.0, 0.0, 0.0), 0.1), (1.6, 0.0, 0.0, 0.0)),
          "目标归零后按同一加速度减速（2.0-0.4=1.6）")

    s2 = kc.VelocitySmoother(4.0, 2.0, 4.0)
    check(eq(s2.update((2.0, 0.0, 0.0, 0.0), 10.0), (0.4, 0.0, 0.0, 0.0)),
          "dt 被 MAX_LOOP_DT 截断，长阻塞也不会一帧冲顶")

    s3 = kc.VelocitySmoother(0.0, 0.0, 0.0)
    check(eq(s3.update((2.0, 1.0, 1.0, 1.0), 0.001), (2.0, 1.0, 1.0, 1.0)),
          "accel<=0 -> 关闭平滑，瞬时到位")

    s3.reset()
    check(eq(s3.cmd, (0.0, 0.0, 0.0, 0.0)), "reset 清零")


# ================================================================= E. 节点
def test_node():
    print("== E. build_twist / step_once ==")
    node = kc.KeyboardController()
    check(node.horiz == 2.0 and node.vert == 1.0 and node.yaw == 1.0, "从参数读取速度配置")
    check(node.smoother.accel == (4.0, 4.0, 2.0, 4.0), "从参数读取三轴加速度上限")

    t = node.step_once("w", now=0.0, dt=0.1)
    check(isinstance(t, _Twist), "step_once 返回 Twist")
    check(eq((t.linear.x, t.linear.y, t.linear.z, t.angular.z), (0.4, 0.0, 0.0, 0.0)),
          "首帧 W 按斜坡输出 0.4 m/s（不是阶跃 2.0）")

    t = node.step_once("w", now=0.1, dt=0.1)
    check(eq((t.linear.x,), (0.8,)), "持续按住 -> 继续加速")

    t = node.step_once("a", now=0.2, dt=0.1)
    check(eq((t.linear.x, t.linear.y), (1.2, 0.4)), "W+A 组合键 -> 两轴同时加速")

    t = node.step_once(None, now=0.50, dt=0.1)
    check(eq((t.linear.x, t.linear.y), (0.8, 0.0)),
          "两键都超过 hold_timeout 后目标归零，仅剩按加速度回落的余速")

    for i in range(4):
        t = node.step_once(None, now=0.6 + 0.1 * i, dt=0.1)
    check(eq((t.linear.x, t.linear.y, t.linear.z, t.angular.z), (0.0, 0.0, 0.0, 0.0)),
          "松开后斜坡回到全零（悬停）")

    check(PUBS["/uav/cmd_vel"] is not None, "已创建 /uav/cmd_vel 发布器")

    node.stop()
    last = PUBS["/uav/cmd_vel"].msgs[-1]
    check(eq((last.linear.x, last.linear.y, last.linear.z, last.angular.z), (0.0, 0.0, 0.0, 0.0)),
          "stop() 补发一条零速指令")


# ================================================================= F. ESC 序列
def test_escape_sequence():
    print("== F. consume_escape 方向键序列 ==")

    class _Reader(object):
        def __init__(self, chars):
            self.chars = list(chars)
            self.used = []

        def get_key(self, timeout=0.05):
            if not self.chars:
                return None
            ch = self.chars.pop(0)
            self.used.append(ch)
            return ch

    r = _Reader(["[", "A"])
    check(kc.consume_escape(r) is True, "ESC [ A（方向键上）-> 识别为转义序列")

    r = _Reader(["[", "1", ";", "5", "A"])
    check(kc.consume_escape(r) is True and r.used == ["[", "1", ";", "5", "A"],
          "带参数的 CSI 序列被完整吃掉，残字节不会变成运动键")

    r = _Reader([])
    check(kc.consume_escape(r) is False, "单独 ESC（后面没有字节）-> 返回 False，用于退出")

    r = _Reader(["O", "P"])
    check(kc.consume_escape(r) is True, "ESC O P（F1 等 SS3 序列）-> 识别为转义序列")


# ================================================================= G. 工程文件
def test_project_files():
    print("== G. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "uav_keyboard_control", "package.xml 包名正确")

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("main.py", "keyboard_control.py", "skill_macros.py"):
        check(name in cmake, "CMakeLists 安装 %s" % name)
    check("nav_msgs" in cmake, "CMakeLists 声明 nav_msgs 依赖")

    pkg_xml = open(os.path.join(PKG, "package.xml"), encoding="utf-8").read()
    check("nav_msgs" in pkg_xml, "package.xml 声明 nav_msgs 依赖")

    launch_path = os.path.join(PKG, "launch", "main.launch")
    launch = ET.parse(launch_path).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py")
    launch_text = open(launch_path, encoding="utf-8").read()
    check("keyboard.yaml" in launch_text, "main.launch 加载 keyboard.yaml")
    args = [a.get("name") for a in launch.findall("arg")]
    for a in ("horiz_speed", "vert_speed", "yaw_rate", "cmd_vel_topic",
              "key_hold_timeout", "accel_horiz", "accel_vert", "accel_yaw",
              "odom_topic", "takeoff_altitude", "spin_degrees", "forward_distance"):
        check(a in args, "main.launch 暴露参数 %s" % a)
    params = [p.get("name") for p in launch.findall("param")]
    for p in ("control/key_hold_timeout", "control/accel_horiz",
              "control/accel_vert", "control/accel_yaw",
              "topic/odom", "macro/takeoff_altitude", "macro/spin_degrees",
              "macro/forward_distance"):
        check(p in params, "main.launch 下发参数 %s" % p)

    yaml = open(os.path.join(PKG, "config", "keyboard.yaml"), encoding="utf-8").read()
    for key in ("horiz_speed", "vert_speed", "yaw_rate", "publish_hz", "cmd_vel",
                "key_hold_timeout", "accel_horiz", "accel_vert", "accel_yaw",
                "odom", "odom_timeout", "takeoff_altitude", "spin_degrees",
                "forward_distance", "forward_speed", "brake_accel"):
        check(key in yaml, "keyboard.yaml 含 %s" % key)


def main():
    test_key_mapping()
    test_merge_target()
    test_key_tracker()
    test_smoother()
    test_node()
    test_escape_sequence()
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
