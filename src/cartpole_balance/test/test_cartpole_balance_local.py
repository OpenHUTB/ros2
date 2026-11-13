#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cartpole_balance 的离线测试：不需要 ROS、不需要 roscore、不需要 pytest。

用法::

    python3 test/test_cartpole_balance_local.py

脚本会桩掉 rospy / std_msgs（模块 import 需要），然后检查：

1. 动力学：竖直平衡点是定点、重力方向、外力方向、无控制时必然倒下；
2. RK4：步长收敛、随机状态有限；
3. LQR：增益维度/符号、A-BK 特征值全负、闭环 5 s 从 0.2 rad 拉回、力限幅；
4. 节点逻辑：合法状态发布、长度/类型异常消息跳过、无 publisher 不崩。
"""

from __future__ import annotations

import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCRIPTS = os.path.join(ROOT, "scripts")
sys.path.insert(0, SCRIPTS)

import numpy as np  # noqa: E402

PASS, FAIL = 0, 0


def check(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  [PASS] %s" % label)
    else:
        FAIL += 1
        print("  [FAIL] %s" % label)


def _install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda *a, **k: None
    rospy.is_shutdown = lambda: True
    rospy.loginfo = lambda *a, **k: None
    rospy.logwarn_throttle = lambda *a, **k: None

    class _Pub(object):
        def __init__(self, *a, **k):
            self.sent = []

        def publish(self, msg):
            self.sent.append(msg)

    class _Sub(object):
        def __init__(self, *a, **k):
            pass

    class _Rate(object):
        def __init__(self, hz):
            self.hz = hz

        def sleep(self):
            pass

    rospy.Publisher = _Pub
    rospy.Subscriber = _Sub
    rospy.Rate = _Rate

    std_msgs = types.ModuleType("std_msgs")
    msg = types.ModuleType("std_msgs.msg")

    class Float64(object):
        def __init__(self, data=0.0):
            self.data = data

    class Float64MultiArray(object):
        def __init__(self, data=None):
            self.data = list(data or [])

    msg.Float64 = Float64
    msg.Float64MultiArray = Float64MultiArray
    std_msgs.msg = msg

    sys.modules["rospy"] = rospy
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = msg


_install_stubs()

import balance_controller as ctrl  # noqa: E402
import cartpole_sim as sim  # noqa: E402


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


class FakeMsg(object):
    def __init__(self, data):
        self.data = data


class RecordingPub(object):
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def test_dynamics():
    print("== 动力学 ==")
    d = sim.cartpole_dynamics(np.zeros(4), 0.0)
    check(np.allclose(d, 0.0, atol=1e-12), "竖直平衡点是定点")
    d = sim.cartpole_dynamics(np.array([0.0, 0.0, 0.2, 0.0]), 0.0)
    check(d[3] > 0.0, "theta>0 时继续向右倒（重力方向）")
    d = sim.cartpole_dynamics(np.zeros(4), 1.0)
    check(d[1] > 0.0, "向右的力使小车加速向右")
    d = sim.cartpole_dynamics(np.zeros(4), -1.0)
    check(d[1] < 0.0, "向左的力使小车加速向左")

    s = np.array([0.0, 0.0, 0.1, 0.0])
    for _ in range(int(2.0 / sim.DT)):
        s = sim.rk4(s, 0.0, sim.DT)
    check(abs(s[2]) > 0.5, "无控制时 0.1 rad 倾角持续增大（必然倒下）")
    check(np.all(np.isfinite(s)), "无控制仿真结果有限")


def test_rk4():
    print("== RK4 ==")
    s0 = np.array([0.0, 0.0, 0.3, 0.0])
    total = 0.5

    def integrate(dt):
        s = s0.copy()
        for _ in range(int(round(total / dt))):
            s = sim.rk4(s, 0.0, dt)
        return s

    coarse = integrate(0.02)
    fine = integrate(0.01)
    reference = integrate(0.001)
    check(np.linalg.norm(fine - reference) < np.linalg.norm(coarse - reference),
          "步长减半后更接近参考解")

    rng = np.random.RandomState(0)
    ok = True
    for _ in range(50):
        state = rng.normal(0.0, 1.0, size=4)
        d = sim.cartpole_dynamics(state, float(rng.normal(0.0, 10.0)))
        ok = ok and bool(np.all(np.isfinite(d)))
    check(ok, "随机状态的导数有限")


def _linearize(eps=1e-6):
    s0 = np.zeros(4)
    a = np.zeros((4, 4))
    b = np.zeros((4, 1))
    for j in range(4):
        e = np.zeros(4)
        e[j] = eps
        a[:, j] = (sim.cartpole_dynamics(s0 + e, 0.0)
                   - sim.cartpole_dynamics(s0 - e, 0.0)) / (2 * eps)
    b[:, 0] = (sim.cartpole_dynamics(s0, eps)
               - sim.cartpole_dynamics(s0, -eps)) / (2 * eps)
    return a, b


def test_lqr():
    print("== LQR ==")
    k = ctrl.lqr_gain()
    check(k.shape == (4,), "增益维度为 4")
    check(bool(np.all(np.isfinite(k))), "增益有限")
    check(k[2] < 0.0, "K_theta < 0（摆杆右倾时把小车往右拉）")

    controller = ctrl.BalanceController(K=k)
    check(controller.compute_force([0.0, 0.0, 0.1, 0.0]) > 0.0,
          "theta>0 时控制力为正")
    check(controller.compute_force(np.zeros(4)) == 0.0, "零状态零力")

    a, b = _linearize()
    eig = np.linalg.eigvals(a - b @ k[None, :])
    check(bool(np.all(eig.real < -1e-6)), "A-BK 特征值实部全为负")

    clamp = ctrl.BalanceController(K=np.array([-100.0, 0.0, -100.0, 0.0]))
    check(clamp.compute_force([1.0, 0.0, 0.0, 0.0]) == ctrl.F_MAX, "正向限幅")
    check(clamp.compute_force([-1.0, 0.0, 0.0, 0.0]) == -ctrl.F_MAX, "负向限幅")

    s = np.array([0.0, 0.0, 0.2, 0.0])
    for _ in range(int(5.0 / sim.DT)):
        s = sim.rk4(s, controller.compute_force(s), sim.DT)
    check(abs(s[2]) < 0.05, "闭环 5 s 从 0.2 rad 拉回（末角 %.4f）" % s[2])


def test_node_logic():
    print("== 节点逻辑 ==")
    pub = RecordingPub()
    controller = ctrl.BalanceController(K=ctrl.lqr_gain(), pub=pub)

    controller.callback(FakeMsg([0.0, 0.0, 0.1, 0.0]))
    check(len(pub.sent) == 1, "合法状态发布一次")
    check(close(pub.sent[0].data, controller.compute_force([0.0, 0.0, 0.1, 0.0]), 1e-12),
          "发布值与 compute_force 一致")

    pub.sent = []
    controller.callback(FakeMsg([0.0, 0.0, 0.1]))
    controller.callback(FakeMsg(None))
    controller.callback(FakeMsg([]))
    check(pub.sent == [], "长度异常的消息被跳过")

    controller.callback(FakeMsg(["a", 0.0, 0.0, 0.0]))
    check(pub.sent == [], "非数值状态被跳过")

    safe = ctrl.BalanceController(K=ctrl.lqr_gain(), pub=None)
    try:
        safe.callback(FakeMsg([0.0, 0.0, 0.1, 0.0]))
        ok = True
    except Exception:
        ok = False
    check(ok, "无 publisher 时不抛异常")


def main():
    test_dynamics()
    test_rk4()
    test_lqr()
    test_node_logic()
    print("-" * 50)
    print("通过 %d / 失败 %d" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
