#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rov_thruster_allocation_ros1 本地测试（不需要 ROS / 仿真器）.

A. 分配矩阵：形状、秩、横滚不可控
B. 正常分配：合力与期望一致
C. 饱和：clamp / scale 两种策略与限幅
D. 故障重构：故障推进器输出为 0，其余推进器重新分配
E. 工程文件

用法: python3 tests/test_rov_thruster_allocation_ros1_local.py
"""
from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(PKG, "..", "..", ".."))
sys.path.insert(0, PKG)

from allocation import (  # noqa: E402
    THRUSTERS, allocate, build_allocation_matrix, residual_ratio,
)


# ================================================================= A. 分配矩阵
def test_matrix():
    print("== A. 分配矩阵 ==")
    B = build_allocation_matrix()
    check(B.shape == (6, len(THRUSTERS)), "B 形状为 6 x %d" % len(THRUSTERS))
    check(np.linalg.matrix_rank(B) == 5, "B 的秩为 5（横滚不可控）")
    check(np.allclose(B[3, :], 0.0), "横滚行全为 0")
    check(np.allclose(B[2, 4:], [1.0, 1.0]), "两个垂直推进器提供垂向推力")


# ================================================================= B. 正常分配
def test_normal_allocation():
    print("== B. 正常分配 ==")
    for wrench in ([30.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 20.0, 0.0, 3.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 5.0]):
        result = allocate(wrench, max_thrust=40.0)
        check(not result["saturated"], "wrench=%s 未饱和" % wrench[:2])
        check(np.linalg.norm(result["residual"]) < 1e-9,
              "wrench=%s 分配残差接近 0" % wrench[:2])
        check(np.all(np.abs(result["commands"]) <= 40.0 + 1e-9),
              "各推进器推力不超过 40 N")

    result = allocate([0.0, 0.0, 0.0, 2.0, 0.0, 0.0], max_thrust=40.0)
    check(abs(result["achieved"][3]) < 1e-9, "横滚指令无法执行，Mx 实际为 0")


# ================================================================= C. 饱和
def test_saturation():
    print("== C. 饱和处理 ==")
    wrench = [80.0, 10.0, 0.0, 0.0, 0.0, 10.0]
    clamp = allocate(wrench, max_thrust=40.0, mode="clamp")
    scale = allocate(wrench, max_thrust=40.0, mode="scale")

    check(clamp["saturated"] and scale["saturated"], "两种模式都检测到饱和")
    check(np.all(np.abs(clamp["commands"]) <= 40.0 + 1e-9), "clamp 后推力不超过上限")
    check(np.all(np.abs(scale["commands"]) <= 40.0 + 1e-9), "scale 后推力不超过上限")
    check(scale["scale"] < 1.0, "scale 比例小于 1（%.2f）" % scale["scale"])

    check(residual_ratio(clamp, wrench) > residual_ratio(scale, wrench),
          "clamp 残差大于 scale（方向被破坏）")
    check(np.allclose(scale["achieved"], np.array(wrench) * scale["scale"], atol=1e-9),
          "scale 模式下实际 wrench 与期望成正比（保持方向）")


# ================================================================= D. 故障重构
def test_fault():
    print("== D. 故障重构 ==")
    result = allocate([30.0, 0.0, 0.0, 0.0, 0.0, 4.0], max_thrust=40.0, failed=1)
    check(result["commands"][1] == 0.0, "故障推进器 T2 指令为 0")
    check(1 not in result["active"] and len(result["active"]) == len(THRUSTERS) - 1,
          "活动推进器数量少 1")
    check(abs(result["achieved"][0]) > 20.0, "剩余推进器仍能提供主要前进推力")
    check(result["residual"][0] != 0.0, "故障后存在不可完全补偿的残差")


# ================================================================= E. 工程文件
def test_project_files():
    print("== E. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "rov_thruster_allocation_ros1", "package.xml 包名正确")
    check(pkg.get("format") == "3", "package.xml 使用 format=\"3\"")
    deps = [d.text.strip() for d in pkg.findall("depend")]
    for dep in ("rospy", "geometry_msgs", "std_msgs"):
        check(dep in deps, "package.xml 声明依赖 %s" % dep)
    exec_deps = [d.text.strip() for d in pkg.findall("exec_depend")]
    check("python3-numpy" in exec_deps, "package.xml 声明 python3-numpy 运行依赖")

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("main.py", "allocation.py", "offline_sim.py"):
        check(name in cmake, "CMakeLists.txt 安装 %s" % name)

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py")
    args = [a.get("name") for a in launch.findall("arg")]
    for name in ("wrench_topic", "cmd_topic", "failed_thruster"):
        check(name in args, "main.launch 暴露参数 %s" % name)

    check(os.path.exists(os.path.join(PKG, "README.md")), "README.md 存在")
    for rel in (
        "docs/water/rov_thruster_allocation_ros1.md",
        "docs/img/water/rov_thruster_allocation_ros1/thruster_cmds.png",
        "docs/img/water/rov_thruster_allocation_ros1/achieved_vs_desired.png",
        "docs/img/water/rov_thruster_allocation_ros1/saturation_fault.png",
    ):
        check(os.path.exists(os.path.join(ROOT, rel)), "文件存在：%s" % rel)


def main():
    test_matrix()
    test_normal_allocation()
    test_saturation()
    test_fault()
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
