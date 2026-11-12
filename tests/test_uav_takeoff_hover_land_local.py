#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_takeoff_hover_land 本地测试（不需要 ROS / 飞控 / 仿真器）.

A. 状态流转：IDLE -> TAKEOFF -> HOVER -> LAND -> DONE
B. 速度限幅与悬停点偏移
C. 工程文件：package.xml / CMakeLists.txt / main.launch / README / 文档 / 图片

用法: python3 tests/test_uav_takeoff_hover_land_local.py
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "uav_takeoff_hover_land")
sys.path.insert(0, PKG)

from flight_state_machine import (  # noqa: E402
    STATE_DONE, STATE_HOVER, STATE_IDLE, STATE_LAND, STATE_TAKEOFF,
    TakeoffHoverLandFSM,
)


# ================================================================= A. 状态流转
def run_flight(fsm, dt=0.05, max_time=60.0):
    position = [0.0, 0.0, 0.0]
    states = []
    commands = []
    hover_target = None
    t = 0.0
    while t < max_time:
        command = fsm.update(position, dt)
        if hover_target is None and command.state == STATE_HOVER:
            hover_target = fsm.hover_target
        states.append(command.state)
        commands.append(command)
        position[0] += command.vx * dt
        position[1] += command.vy * dt
        position[2] += command.vz * dt
        if position[2] < 0.0:
            position[2] = 0.0
        t += dt
        if command.state == STATE_DONE:
            break
    return position, states, commands, hover_target, t


def test_state_flow():
    print("== A. 状态流转 ==")
    fsm = TakeoffHoverLandFSM(takeoff_altitude=2.0, hover_time=5.0,
                              hover_offset=(1.0, 0.5))
    check(fsm.state == STATE_IDLE, "初始状态为 IDLE")

    position, states, commands, hover_target, total = run_flight(fsm)

    check(states[0] == STATE_TAKEOFF, "首帧进入 TAKEOFF")
    check(STATE_HOVER in states, "出现 HOVER")
    check(STATE_LAND in states, "出现 LAND")
    check(states[-1] == STATE_DONE, "最终进入 DONE")
    order = [states[0]]
    for s in states:
        if s != order[-1]:
            order.append(s)
    check(order == [STATE_TAKEOFF, STATE_HOVER, STATE_LAND, STATE_DONE],
          "状态顺序为 TAKEOFF -> HOVER -> LAND -> DONE（实际 %s）" % order)

    takeoff_time = states.index(STATE_HOVER) * 0.05
    land_time = states.index(STATE_LAND) * 0.05
    check(2.0 <= takeoff_time <= 10.0, "起飞到点时间在 2~10 s（%.2f s）" % takeoff_time)
    check(4.5 <= (land_time - takeoff_time) <= 5.5,
          "悬停时长接近 5 s（%.2f s）" % (land_time - takeoff_time))
    check(abs(position[2]) <= 0.06, "落地高度接近 0（%.3f m）" % position[2])
    check(total <= 30.0, "总飞行时间不超过 30 s（%.2f s）" % total)

    check(hover_target is not None
          and abs(hover_target[0] - 1.0) < 1e-6
          and abs(hover_target[1] - 0.5) < 1e-6,
          "悬停点 = 起飞点 + (1.0, 0.5)")

    last = commands[-1]
    check((last.vx, last.vy, last.vz) == (0.0, 0.0, 0.0), "DONE 后速度指令为零")


# ================================================================= B. 限幅
def test_limits():
    print("== B. 速度限幅 ==")
    fsm = TakeoffHoverLandFSM(takeoff_altitude=2.0,
                              max_horizontal_speed=0.8,
                              max_vertical_speed=0.3,
                              kp_horizontal=5.0, kp_vertical=5.0)
    command = fsm.update((10.0, -10.0, 0.0), 0.05)  # 距离目标很远
    check(abs(command.vx) <= 0.8 + 1e-9 and abs(command.vy) <= 0.8 + 1e-9,
          "水平速度被限制在 0.8 m/s 以内（vx=%.2f, vy=%.2f）"
          % (command.vx, command.vy))
    check(abs(command.vz) <= 0.3 + 1e-9,
          "垂直速度被限制在 0.3 m/s 以内（vz=%.2f）" % command.vz)

    down = fsm.update((10.0, -10.0, -1.0), 0.05)
    check(down.vz >= -0.3 - 1e-9, "下降速度同样受限幅约束")


# ================================================================= C. 工程文件
def test_project_files():
    print("== C. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "uav_takeoff_hover_land", "package.xml 包名正确")
    deps = [d.text.strip() for d in pkg.findall("depend")]
    for dep in ("rospy", "geometry_msgs"):
        check(dep in deps, "package.xml 声明依赖 %s" % dep)

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("main.py", "flight_state_machine.py", "offline_sim.py"):
        check(name in cmake, "CMakeLists.txt 安装 %s" % name)

    launch = ET.parse(os.path.join(PKG, "main.launch")).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py")
    args = [a.get("name") for a in launch.findall("arg")]
    for name in ("pose_topic", "cmd_topic"):
        check(name in args, "main.launch 暴露参数 %s" % name)

    for rel in (
        "README.md",
    ):
        check(os.path.exists(os.path.join(PKG, rel)), "文件存在：%s" % rel)
    for rel in (
        "docs/air/uav_takeoff_hover_land.md",
        "docs/img/air/uav_takeoff_hover_land/trajectory_3d.png",
        "docs/img/air/uav_takeoff_hover_land/altitude_state.png",
        "docs/img/air/uav_takeoff_hover_land/velocity.png",
    ):
        check(os.path.exists(os.path.join(ROOT, rel)), "文件存在：%s" % rel)


def main():
    test_state_flow()
    test_limits()
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
