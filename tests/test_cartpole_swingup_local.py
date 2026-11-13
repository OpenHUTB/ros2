#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cartpole_swingup 本地测试（不需要 ROS / Gazebo）.

A. 动力学与能量函数
B. LQR 增益的闭环稳定性
C. 摆起 / 平衡模式切换
D. 能量控制确实能把能量泵上去
E. 工程文件

用法: python3 tests/test_cartpole_swingup_local.py
"""
from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "classical_control", "cartpole_swingup")
sys.path.insert(0, PKG)

from cartpole_model import (  # noqa: E402
    MODE_BALANCE, MODE_SWINGUP, CartPoleController, DEFAULT_PARAMS,
    cartpole_dynamics, lqr_gain, total_energy, upright_energy, wrap_angle,
)


def rk4(state, force, dt):
    k1 = cartpole_dynamics(state, force)
    k2 = cartpole_dynamics([s + dt / 2 * d for s, d in zip(state, k1)], force)
    k3 = cartpole_dynamics([s + dt / 2 * d for s, d in zip(state, k2)], force)
    k4 = cartpole_dynamics([s + dt * d for s, d in zip(state, k3)], force)
    return [s + dt / 6 * (a + 2 * b + 2 * c + d)
            for s, a, b, c, d in zip(state, k1, k2, k3, k4)]


# ================================================================= A. 模型
def test_model():
    print("== A. 动力学与能量 ==")
    check(abs(abs(wrap_angle(3 * math.pi)) - math.pi) < 1e-9,
          "角度归一化到 [-pi, pi]")

    derivative = cartpole_dynamics([0.0, 0.0, 0.0, 0.0], 0.0)
    check(max(abs(v) for v in derivative) < 1e-12, "竖直静止时状态导数为 0")

    check(abs(total_energy([0.0, 0.0, math.pi, 0.0])) < 1e-9,
          "下垂静止时总能量为 0")
    check(abs(total_energy([0.0, 0.0, 0.0, 0.0]) - upright_energy()) < 1e-9,
          "竖直静止时总能量等于目标能量")
    check(abs(upright_energy() - 2 * DEFAULT_PARAMS['mp'] * DEFAULT_PARAMS['g'] * DEFAULT_PARAMS['l']) < 1e-9,
          "目标能量 = 2 m g l")


# ================================================================= B. LQR
def test_lqr():
    print("== B. LQR 闭环稳定性 ==")
    from cartpole_model import linearize_upright
    K = lqr_gain(dt=0.02)
    A, B = linearize_upright()
    Ad = np.eye(4) + A * 0.02
    Bd = B * 0.02
    eigenvalues = np.linalg.eigvals(Ad - Bd.dot(K.reshape(1, -1)))
    check(np.max(np.abs(eigenvalues)) < 1.0,
          "离散闭环特征值都在单位圆内（max=%.3f）" % np.max(np.abs(eigenvalues)))
    check(K.shape == (4,), "LQR 增益为 4 维")


# ================================================================= C. 模式切换
def test_switch():
    print("== C. 摆起 / 平衡模式切换 ==")
    controller = CartPoleController()
    check(controller.mode == MODE_SWINGUP, "初始为 SWINGUP")

    controller.update([0.0, 0.0, 0.2, 0.5])
    check(controller.mode == MODE_BALANCE, "接近竖直且角速度小时切到 BALANCE")

    force = controller.update([0.0, 0.0, 0.3, 0.0])
    check(math.isfinite(force) and abs(force) <= DEFAULT_PARAMS['force_limit'],
          "BALANCE 输出有限且在限幅内")

    controller.update([0.0, 0.0, 1.0, 0.0])
    check(controller.mode == MODE_SWINGUP, "角度过大时退回 SWINGUP")

    lqr_only = CartPoleController(allow_swingup=False)
    lqr_only.update([0.0, 0.0, 3.0, 0.0])
    check(lqr_only.mode == MODE_BALANCE, "allow_swingup=False 时始终为 BALANCE")


# ================================================================= D. 能量泵入
def test_swingup():
    print("== D. 能量摆起 ==")
    controller = CartPoleController()
    state = [0.0, 0.0, math.pi - 0.1, 0.0]
    initial_energy = total_energy(state)
    dt = 0.02
    for _ in range(150):     # 3 s
        force = controller.update(state, dt)
        state = rk4(state, force, dt)

    check(total_energy(state) > initial_energy + 0.2,
          "3 s 内能量明显增加（%.2f -> %.2f J）" % (initial_energy, total_energy(state)))
    check(abs(wrap_angle(state[2])) < math.pi - 0.02, "摆杆已经开始向上运动")


# ================================================================= E. 工程文件
def test_project_files():
    print("== E. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "cartpole_swingup", "package.xml 包名正确")
    deps = [d.text.strip() for d in pkg.findall("depend")] + \
        [d.text.strip() for d in pkg.findall("exec_depend")]
    for dep in ("rospy", "std_msgs", "python3-numpy"):
        check(dep in deps, "package.xml 声明依赖 %s" % dep)

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("main.py", "cartpole_model.py", "offline_sim.py"):
        check(name in cmake, "CMakeLists.txt 安装 %s" % name)

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py")
    args = [a.get("name") for a in launch.findall("arg")]
    for name in ("state_topic", "force_topic"):
        check(name in args, "main.launch 暴露参数 %s" % name)

    check(os.path.exists(os.path.join(PKG, "README.md")), "README.md 存在")
    for rel in (
        "docs/classical_control/cartpole_swingup.md",
        "docs/img/classical_control/cartpole_swingup/swingup_response.png",
        "docs/img/classical_control/cartpole_swingup/phase_portrait.png",
        "docs/img/classical_control/cartpole_swingup/lqr_only_comparison.png",
    ):
        check(os.path.exists(os.path.join(ROOT, rel)), "文件存在：%s" % rel)


def main():
    test_model()
    test_lqr()
    test_switch()
    test_swingup()
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
