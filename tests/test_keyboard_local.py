#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制模块本地测试（桩依赖，不需要 ROS / 仿真器 / 终端）.

A. key_to_cmd 按键映射：W/S/A/D/R/F/Q/E -> 机体系速度分量与符号
B. 速度参数可配置；空键/未知键 -> 悬停
C. build_twist / step_once 字段映射
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


install_stubs()
import keyboard_control as kc  # noqa: E402


# ================================================================= A. 按键映射
def test_key_mapping():
    print("== A. key_to_cmd 按键映射 ==")
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


# ================================================================= B. 节点
def test_node():
    print("== B. build_twist / step_once ==")
    node = kc.KeyboardController()
    check(node.horiz == 2.0 and node.vert == 1.0 and node.yaw == 1.0, "从参数读取速度配置")

    t = node.step_once("w")
    check(isinstance(t, _Twist), "step_once 返回 Twist")
    check((t.linear.x, t.linear.y, t.linear.z, t.angular.z) == (2.0, 0.0, 0.0, 0.0),
          "W 按键 -> 前进速度分量正确")
    check(PUBS["/uav/cmd_vel"] is not None, "已创建 /uav/cmd_vel 发布器")

    t0 = node.step_once(None)
    check((t0.linear.x, t0.linear.y, t0.linear.z, t0.angular.z) == (0, 0, 0, 0),
          "无按键 -> 悬停（全零）")

    tx = node.step_once("\x1b")
    check((tx.linear.x, tx.angular.z) == (0.0, 0.0), "未知键 -> 悬停（不崩溃）")

    # 组合键语义：同一次只能有一个方向（纯函数，符号互不污染）
    t_left = node.step_once("a")
    check(t_left.linear.y == 2.0 and t_left.linear.x == 0.0 and t_left.linear.z == 0.0,
          "A 键只影响 vy，不串扰其它轴")


# ================================================================= C. 工程文件
def test_project_files():
    print("== C. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "uav_keyboard_control", "package.xml 包名正确")

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("main.py", "keyboard_control.py"):
        check(name in cmake, "CMakeLists 安装 %s" % name)

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py")
    check("keyboard.yaml" in open(os.path.join(PKG, "launch", "main.launch"), encoding="utf-8").read(),
          "main.launch 加载 keyboard.yaml")
    args = [a.get("name") for a in launch.findall("arg")]
    for a in ("horiz_speed", "vert_speed", "yaw_rate", "cmd_vel_topic"):
        check(a in args, "main.launch 暴露参数 %s" % a)

    yaml = open(os.path.join(PKG, "config", "keyboard.yaml"), encoding="utf-8").read()
    for key in ("horiz_speed", "vert_speed", "yaw_rate", "publish_hz", "cmd_vel"):
        check(key in yaml, "keyboard.yaml 含 %s" % key)


def main():
    test_key_mapping()
    test_node()
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
