#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""turtlesim_teleop 本地测试（桩依赖，不需要 ROS / turtlesim / 终端）.

A. 按键映射：方向键转义序列与 WASD -> 线速度/角速度系数，空格停车，未知键忽略
B. publish_cmd：Twist 字段映射正确
C. 工程文件：package.xml / CMakeLists.txt / main.launch

用法: python3 tests/test_turtlesim_teleop_local.py
"""
from __future__ import annotations

import importlib.util
import os
import sys
import types
import xml.etree.ElementTree as ET

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "chap1", "turtlesim_teleop")


# ---------------------------------------------------------------- 消息桩
class _Vec3(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist(object):
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


# ---------------------------------------------------------------- rospy 桩
rospy = types.ModuleType("rospy")
rospy.loginfo = lambda *a, **k: None
rospy.logerr = lambda *a, **k: None
rospy.init_node = lambda *a, **k: None
rospy.get_param = lambda name, default=None: default
rospy.Publisher = lambda *a, **k: None
rospy.sleep = lambda *a, **k: None
rospy.is_shutdown = lambda: False
sys.modules.setdefault("rospy", rospy)

geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg.Twist = _Twist
geometry_msgs.msg = geometry_msgs_msg
sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

# termios / tty 是 Unix 专有模块，本测试用空实现替代
termios = types.ModuleType("termios")
termios.tcgetattr = lambda fd: []
termios.tcsetattr = lambda fd, when, attrs: None
termios.TCSADRAIN = 1
sys.modules.setdefault("termios", termios)
tty = types.ModuleType("tty")
tty.setcbreak = lambda fd: None
sys.modules.setdefault("tty", tty)

spec = importlib.util.spec_from_file_location(
    "turtlesim_teleop_main", os.path.join(PKG, "main.py"))
teleop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)


# ================================================================= A. 按键映射
def test_key_mapping():
    print("== A. 按键映射 ==")
    cases = {
        "\x1b[A": (1.0, 0.0),    # ↑ 前进
        "\x1b[B": (-1.0, 0.0),   # ↓ 后退
        "\x1b[D": (0.0, 1.0),    # ← 左转
        "\x1b[C": (0.0, -1.0),   # → 右转
        "w": (1.0, 0.0),
        "W": (1.0, 0.0),
        "s": (-1.0, 0.0),
        "a": (0.0, 1.0),
        "d": (0.0, -1.0),
        " ": (0.0, 0.0),         # 空格停车
    }
    for key, expected in cases.items():
        check(teleop.command_for_key(key) == expected,
              "按键 %r -> %r" % (key, expected))

    check(teleop.command_for_key("x") is None, "未知按键返回 None")
    check(teleop.command_for_key(None) is None, "空按键返回 None")


# ================================================================= B. 消息发布
def test_publish_cmd():
    print("== B. 消息发布 ==")
    published = []

    class Pub(object):
        def publish(self, msg):
            published.append((msg.linear.x, msg.angular.z))

    teleop.publish_cmd(Pub(), 2.0, -1.5)
    check(published == [(2.0, -1.5)], "publish_cmd 正确写入 linear.x / angular.z")

    teleop.publish_cmd(Pub(), 0.0, 0.0)
    check(published[-1] == (0.0, 0.0), "停车消息为全零")


# ================================================================= C. 工程文件
def test_project_files():
    print("== C. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "turtlesim_teleop", "package.xml 包名正确")
    deps = [d.text.strip() for d in pkg.findall("depend")]
    for dep in ("rospy", "geometry_msgs", "turtlesim"):
        check(dep in deps, "package.xml 声明依赖 %s" % dep)

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    check("main.py" in cmake, "CMakeLists.txt 安装 main.py")
    check("catkin_install_python" in cmake, "CMakeLists.txt 使用 catkin_install_python")
    check("install(DIRECTORY launch" in cmake, "CMakeLists.txt 安装 launch 目录")

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("turtlesim_node" in nodes, "main.launch 启动 turtlesim_node")
    check("main.py" in nodes, "main.launch 启动 main.py")
    check(os.path.exists(os.path.join(PKG, "README.md")), "README.md 存在")
    check(os.path.exists(os.path.join(ROOT, "docs", "chapter", "chap1",
                                      "turtlesim_teleop.md")), "配套文档存在")
    check(os.path.exists(os.path.join(ROOT, "docs", "chapter", "chap1", "images",
                                      "turtle_teleop.png")), "运行截图存在")


def main():
    test_key_mapping()
    test_publish_cmd()
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
