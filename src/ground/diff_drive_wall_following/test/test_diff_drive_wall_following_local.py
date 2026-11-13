#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diff_drive_wall_following 本地测试（不需要 ROS / Gazebo / 激光雷达）.

A. 沿墙控制：距离保持、夹角修正、墙角转向状态切换
B. 限幅与丢墙保护
C. 工程文件：package.xml / CMakeLists.txt / main.launch / README / 文档 / 图片

用法: python3 src/ground/diff_drive_wall_following/test/test_diff_drive_wall_following_local.py
"""
from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(PKG, "..", "..", ".."))
sys.path.insert(0, PKG)

from wall_follower import (  # noqa: E402
    STATE_FOLLOW, STATE_TURN, WallFollower,
)


# ================================================================= A. 沿墙控制
def test_follow():
    print("== A. 沿墙控制 ==")
    follower = WallFollower()

    v, w, state = follower.update(2.0, 0.5, 0.5, 0.5, 2.0, 0.1)
    check(state == STATE_FOLLOW, "正前方无遮挡时为 FOLLOW")
    check(abs(w) < 1e-6, "距离和夹角都无误差时角速度约为 0")
    check(0.0 < v <= 0.4, "正常跟随时有前进速度（%.2f m/s）" % v)

    follower = WallFollower()
    _, w_far, _ = follower.update(2.0, 0.8, 0.8, 0.8, 2.0, 0.1)
    check(w_far < 0.0, "离墙太远时向右转（w<0，%.2f）" % w_far)

    follower = WallFollower()
    _, w_near, _ = follower.update(2.0, 0.3, 0.3, 0.3, 2.0, 0.1)
    check(w_near > 0.0, "离墙太近时向左转（w>0，%.2f）" % w_near)

    follower = WallFollower()
    _, w_angle, _ = follower.update(2.0, 0.5, 0.6, 0.4, 2.0, 0.1)
    check(w_angle < 0.0, "车头偏向外侧时向右修正（w<0，%.2f）" % w_angle)


def test_corner_and_limits():
    print("== B. 墙角转向、限幅与丢墙 ==")
    follower = WallFollower()
    v, w, state = follower.update(0.5, 0.5, 0.5, 0.5, 0.5, 0.1)
    check(state == STATE_TURN, "正前方小于阈值时进入 TURN")
    check(v == 0.0 and w > 0.0, "转角时停车并原地左转")

    v, w, state = follower.update(0.9, 0.5, 0.5, 0.5, 2.0, 0.1)
    check(state == STATE_TURN, "前方仍不够空时保持 TURN")

    v, w, state = follower.update(1.2, 0.5, 0.5, 0.5, 2.0, 0.1)
    check(state == STATE_FOLLOW, "转过墙角后回到 FOLLOW")

    follower = WallFollower(max_angular=0.6)
    _, w, _ = follower.update(2.0, 3.0, 3.0, 3.0, 2.0, 0.1)
    check(abs(w) <= 0.6 + 1e-9, "角速度被限制在 max_angular 以内（%.2f）" % w)

    follower = WallFollower()
    v, w, _ = follower.update(2.0, 5.0, 5.0, 5.0, 2.0, 0.1)
    check(v > 0.0 and w < 0.0, "右侧丢墙时低速右转寻找墙壁")

    follower = WallFollower()
    v, w, _ = follower.update(2.0, 0.5, float("inf"), float("inf"), 2.0, 0.1)
    check(math.isfinite(v) and math.isfinite(w),
          "前右/后右同时无回波时不输出 NaN（v=%s, w=%s）" % (v, w))
    check(abs(w) < 1e-6, "两路缺失时用 side 兜底，夹角误差为 0")

    follower = WallFollower()
    _, w, _ = follower.update(2.0, 0.5, 0.8, float("inf"), 2.0, 0.1)
    check(math.isfinite(w) and w < 0.0,
          "单路缺失时用 side 兜底并保持有限输出（w=%.2f）" % w)

    follower = WallFollower()
    v, w, _ = follower.update(2.0, float("inf"), float("inf"), float("inf"), 2.0, 0.1)
    check(math.isfinite(v) and math.isfinite(w),
          "右侧完全丢墙时输出仍有限（v=%.2f, w=%.2f）" % (v, w))


# ================================================================= C. 工程文件
def test_project_files():
    print("== C. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "diff_drive_wall_following", "package.xml 包名正确")
    deps = [d.text.strip() for d in pkg.findall("depend")]
    for dep in ("rospy", "geometry_msgs", "sensor_msgs"):
        check(dep in deps, "package.xml 声明依赖 %s" % dep)

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("main.py", "wall_follower.py", "offline_sim.py"):
        check(name in cmake, "CMakeLists.txt 安装 %s" % name)

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py")
    args = [a.get("name") for a in launch.findall("arg")]
    for name in ("scan_topic", "cmd_topic"):
        check(name in args, "main.launch 暴露参数 %s" % name)

    check(os.path.exists(os.path.join(PKG, "README.md")), "README.md 存在")
    check(os.path.exists(os.path.join(
        ROOT, ".github", "workflows", "diff_drive_wall_following_tests.yml")),
        "CI workflow 存在")
    for rel in (
        "docs/ground/diff_drive_wall_following.md",
        "docs/img/ground/diff_drive_wall_following/map_trajectory.png",
        "docs/img/ground/diff_drive_wall_following/wall_distance.png",
        "docs/img/ground/diff_drive_wall_following/velocity.png",
    ):
        check(os.path.exists(os.path.join(ROOT, rel)), "文件存在：%s" % rel)


def main():
    test_follow()
    test_corner_and_limits()
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
