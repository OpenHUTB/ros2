#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制模块本地测试（桩依赖，不需要 ROS / 仿真器 / 终端）.

覆盖：
A. key_to_cmd 按键映射与符号、空键 / 未知键 / ESC、参数可配置
B. KeyboardController 参数读取与 Twist 字段映射
C. 状态锁存 + hold_timeout 超时自动悬停
D. 工程文件：package.xml / CMakeLists / main.launch / keyboard.yaml

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


HERE = os.path.dirname(os.path.abspath(__file__))
# 兼容两种目录布局：
#   1) 标准 catkin 扁平包：tests 与 scripts 同级（<pkg>/tests、<pkg>/scripts）；
#   2) 嵌套源码树：包体位于 <root>/src/air/uav_keyboard_control/。
# 优先以含 package.xml 的目录为包根；否则退而定位含 scripts/keyboard_control.py 的目录，
# 保证 A/B/C 段纯函数用例可独立运行。
_C0 = os.path.abspath(os.path.join(HERE, ".."))
_C1 = os.path.abspath(os.path.join(HERE, "..", "src", "air", "uav_keyboard_control"))


def _find_pkg_root():
    for c in (_C0, _C1):
        if os.path.exists(os.path.join(c, "package.xml")):
            return c
    for c in (_C0, _C1):
        if os.path.exists(os.path.join(c, "scripts", "keyboard_control.py")):
            return c
    return _C0


PKG = _find_pkg_root()
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
    "control/forward_speed": 2.0,
    "control/side_speed": 0.5,
    "control/vert_speed": 1.0,
    "control/yaw_rate": 0.4,
    "control/hold_timeout": 0.3,
    "topic/cmd_vel": "/uav/cmd_vel",
    "rate/publish_hz": 30.0,
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


class _FakeTime(object):
    def __init__(self, sec):
        self.sec = sec

    def __sub__(self, other):
        return _FakeDelta(self.sec - other.sec)


class _FakeDelta(object):
    def __init__(self, sec):
        self.sec = sec

    def to_sec(self):
        return self.sec


def install_stubs():
    m = types.ModuleType("rospy")
    m.get_param = lambda name, default=None: PARAMS.get(name, default)
    m.Publisher = _Publisher
    m.Rate = _Rate
    m.init_node = lambda *a, **k: None
    m.is_shutdown = lambda: False
    m.Time = _FakeTime
    m.Time.now = staticmethod(lambda: _FakeTime(0))
    m.loginfo = lambda *a, **k: None
    m.logwarn = lambda *a, **k: None
    sys.modules["rospy"] = m

    gm = types.ModuleType("geometry_msgs")
    gmm = types.ModuleType("geometry_msgs.msg")
    gmm.Twist = _Twist
    gm.msg = gmm
    sys.modules["geometry_msgs"] = gm
    sys.modules["geometry_msgs.msg"] = gmm


install_stubs()
import keyboard_control as kc  # noqa: E402


# ================================================================= A. 按键映射
def test_key_mapping():
    print("== A. key_to_cmd 按键映射 ==")
    check(kc.key_to_cmd("w") == (2.0, 0.0, 0.0, 0.0), "W -> 前进 (vx=+2.0)")
    check(kc.key_to_cmd("s") == (-2.0, 0.0, 0.0, 0.0), "S -> 后退 (vx=-2.0)")
    check(kc.key_to_cmd("a") == (0.0, 0.5, 0.0, 0.0), "A -> 左移 (vy=+0.5)")
    check(kc.key_to_cmd("d") == (0.0, -0.5, 0.0, 0.0), "D -> 右移 (vy=-0.5)")
    check(kc.key_to_cmd("r") == (0.0, 0.0, 1.0, 0.0), "R -> 上升 (vz=+1.0)")
    check(kc.key_to_cmd("f") == (0.0, 0.0, -1.0, 0.0), "F -> 下降 (vz=-1.0)")
    check(kc.key_to_cmd("q") == (0.0, 0.0, 0.0, 0.4), "Q -> 右转 (wz=+0.4)")
    check(kc.key_to_cmd("e") == (0.0, 0.0, 0.0, -0.4), "E -> 左转 (wz=-0.4)")
    check(kc.key_to_cmd("W") == kc.key_to_cmd("w"), "大写键与小写键等价")
    check(kc.key_to_cmd("") == (0.0, 0.0, 0.0, 0.0), "空键 -> 全零（悬停）")
    check(kc.key_to_cmd(None) == (0.0, 0.0, 0.0, 0.0), "None -> 全零（悬停）")
    check(kc.key_to_cmd("x") is None, "未知键 -> None")
    check(kc.key_to_cmd("\x1b") is None, "ESC 转义 -> None（由 spin 单独处理退出）")
    check(kc.key_to_cmd("w", 3.0, 2.0, 1.0, 0.5) == (3.0, 0.0, 0.0, 0.0), "前后速度参数可配置")
    check(kc.key_to_cmd("q", 3.0, 2.0, 1.0, 0.5) == (0.0, 0.0, 0.0, 0.5), "偏航角速度参数可配置")


# ================================================================= B. 节点参数与字段
def test_node_params_and_twist():
    print("== B. 节点参数读取与 Twist 字段映射 ==")
    node = kc.KeyboardController()
    check(node.forward == 2.0 and node.side == 0.5 and node.vert == 1.0 and node.yaw == 0.4,
          "从参数读取速度配置 2.0 / 0.5 / 1.0 / 0.4")
    check(abs(node.hold_timeout - 0.3) < 1e-9, "hold_timeout = 0.3s")
    check("/uav/cmd_vel" in PUBS, "已创建 /uav/cmd_vel 发布器")

    t = node.step_once("w")
    check((t.linear.x, t.linear.y, t.linear.z, t.angular.z) == (2.0, 0.0, 0.0, 0.0),
          "W -> 前进速度分量正确")
    t = node.step_once("a")
    check(t.linear.y == 0.5 and t.linear.x == 0.0 and t.linear.z == 0.0,
          "A 只影响 vy，不串扰其它轴")
    t = node.step_once("x")
    check((t.linear.x, t.linear.y, t.linear.z, t.angular.z) == (0.0, 0.0, 0.0, 0.0),
          "未知键 -> 悬停（全零）")


# ================================================================= C. 锁存与超时
def test_latch_and_timeout():
    print("== C. 状态锁存 + 超时自动悬停 ==")
    node = kc.KeyboardController()
    t0 = _FakeTime(0.0)
    check(node.feed_key("w", now=t0) == (2.0, 0.0, 0.0, 0.0), "按下 W 锁存前进指令")
    check(node.feed_key(None, now=_FakeTime(0.1)) == (2.0, 0.0, 0.0, 0.0),
          "0.1s 内无按键仍保持上一条指令")
    check(node.feed_key(None, now=_FakeTime(0.4)) == (0.0, 0.0, 0.0, 0.0),
          "超过 0.3s 无按键自动清零悬停")

    node.feed_key("w", now=_FakeTime(0.5))
    check(node.feed_key(" ", now=_FakeTime(0.5)) == (0.0, 0.0, 0.0, 0.0),
          "空格立即清零悬停")

    node.feed_key("w", now=_FakeTime(0.6))
    check(node.feed_key("x", now=_FakeTime(0.6)) == (0.0, 0.0, 0.0, 0.0),
          "未知键立即清零悬停")

    check(node.feed_key("s", now=_FakeTime(0.7)) == (-2.0, 0.0, 0.0, 0.0),
          "重新按 S 锁存后退指令")


# ================================================================= D. 工程文件
def test_project_files():
    print("== D. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "uav_keyboard_control", "package.xml 包名正确")

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("main.py", "keyboard_control.py"):
        check(name in cmake, "CMakeLists 安装 %s" % name)

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py")
    launch_text = open(os.path.join(PKG, "launch", "main.launch"), encoding="utf-8").read()
    check("keyboard.yaml" in launch_text, "main.launch 加载 keyboard.yaml")
    args = [a.get("name") for a in launch.findall("arg")]
    check("cmd_vel_topic" in args, "main.launch 暴露 cmd_vel_topic 参数")

    yaml = open(os.path.join(PKG, "config", "keyboard.yaml"), encoding="utf-8").read()
    for key in ("forward_speed", "side_speed", "vert_speed", "yaw_rate",
                "hold_timeout", "publish_hz", "cmd_vel"):
        check(key in yaml, "keyboard.yaml 含 %s" % key)


def main():
    test_key_mapping()
    test_node_params_and_twist()
    test_latch_and_timeout()
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
