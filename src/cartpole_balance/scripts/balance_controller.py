#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""节点1: LQR 控制器 (cartpole_balance)
订阅小车+摆杆状态 /cartpole/state, 计算最优控制力 F, 发布到 /cartpole/cmd。

控制律: 在竖直平衡点线性化系统, 解连续 Riccati 方程得到最优增益 K,
        F = -K·[x, x_dot, theta, theta_dot], 并限幅到 [-F_MAX, F_MAX]。

话题:
  订阅  /cartpole/state   std_msgs/Float64MultiArray   [x, x_dot, theta, theta_dot]
  发布  /cartpole/cmd     std_msgs/Float64             控制力 F (N)

用法: roslaunch cartpole_balance main.launch
"""
import math
import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray

# 物理参数(与 cartpole_sim 保持一致)
M, m, L, g = 1.0, 0.1, 0.5, 9.8
F_MAX = 40.0

# LQR 权重: 位置/速度/摆角/角速度, 控制代价 R=1
Q = np.diag([5.0, 0.1, 100.0, 10.0])
R = np.array([[1.0]])


def lqr_gain():
    """数值线性化动力学 -> 解 Riccati 方程求 K。无 scipy 时回退到硬编码增益。"""
    def dyn(s, F):
        x, xd, th, thd = s
        total = M + m
        th_acc = (g * math.sin(th) - math.cos(th) * (F + m * L * thd * thd * math.sin(th)) / total) \
                 / (L * (4.0 / 3.0 - m * math.cos(th) ** 2 / total))
        x_acc = (F + m * L * (thd * thd * math.sin(th) - th_acc * math.cos(th))) / total
        return np.array([xd, x_acc, thd, th_acc])
    try:
        from scipy.linalg import solve_continuous_are
        s0 = np.zeros(4)
        eps = 1e-6
        A = np.zeros((4, 4))
        B = np.zeros((4, 1))
        for j in range(4):
            e = np.zeros(4)
            e[j] = eps
            A[:, j] = (dyn(s0 + e, 0.0) - dyn(s0 - e, 0.0)) / (2 * eps)
        B[:, 0] = (dyn(s0, eps) - dyn(s0, -eps)) / (2 * eps)
        P = solve_continuous_are(A, B, Q, R)
        return (B.T @ P)[0] / R[0, 0]
    except ImportError:
        return np.array([-2.236, -3.649, -39.736, -10.326])  # 与上式计算一致


def on_state(msg):
    """收到状态 -> 计算并发布控制力"""
    s = np.array(msg.data)                 # [x, x_dot, theta, theta_dot]
    F = float(-(K @ s).item())
    F = max(-F_MAX, min(F_MAX, F))
    pub.publish(Float64(data=F))


if __name__ == "__main__":
    rospy.init_node("balance_controller")
    K = lqr_gain()
    rospy.loginfo("LQR 增益 K = %s", np.round(K, 3).tolist())
    pub = rospy.Publisher("/cartpole/cmd", Float64, queue_size=1)
    rospy.Subscriber("/cartpole/state", Float64MultiArray, on_state)
    rospy.loginfo("balance_controller 已启动: 订阅 /cartpole/state, 发布 /cartpole/cmd")
    rospy.spin()
