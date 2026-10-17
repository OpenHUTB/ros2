#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""节点2: 倒立摆物理仿真 (cartpole_balance)
订阅控制力 /cartpole/cmd, 用 RK4 积分倒立摆动力学, 以 50Hz 发布状态 /cartpole/state。

状态 s = [x, x_dot, theta, theta_dot]
  x     小车水平位置 (m, 向右为正)
  x_dot 小车速度 (m/s)
  theta 摆杆相对竖直的偏角 (rad, 0=竖直, 向右倾倒为正)
  theta_dot 摆杆角速度 (rad/s)

话题:
  订阅  /cartpole/cmd    std_msgs/Float64    控制力 F (N, 向右为正)
  发布  /cartpole/state  std_msgs/Float64MultiArray  [x, x_dot, theta, theta_dot]

用法: roslaunch cartpole_balance main.launch
"""
import math
import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray

# 物理参数
M, m, L, g = 1.0, 0.1, 0.5, 9.8
DT = 0.02          # 仿真步长 s (50Hz)

# 初始状态: 小车在原点, 摆杆偏离竖直 0.2 rad(以便展示"从倾斜被拉回竖直")
state = np.array([0.0, 0.0, 0.2, 0.0])   # [x, x_dot, theta, theta_dot]
force = 0.0


def cartpole_dynamics(s, F):
    """s=[x, x_dot, theta, theta_dot] -> 导数 [xd, x_acc, thd, th_acc]"""
    x, xd, th, thd = s
    total = M + m
    th_acc = (g * math.sin(th) - math.cos(th) * (F + m * L * thd * thd * math.sin(th)) / total) \
             / (L * (4.0 / 3.0 - m * math.cos(th) ** 2 / total))
    x_acc = (F + m * L * (thd * thd * math.sin(th) - th_acc * math.cos(th))) / total
    return np.array([xd, x_acc, thd, th_acc])


def rk4(s, F, dt):
    k1 = cartpole_dynamics(s, F)
    k2 = cartpole_dynamics(s + 0.5 * dt * k1, F)
    k3 = cartpole_dynamics(s + 0.5 * dt * k2, F)
    k4 = cartpole_dynamics(s + dt * k3, F)
    return s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def on_cmd(msg):
    global force
    force = msg.data


if __name__ == "__main__":
    rospy.init_node("cartpole_sim")
    pub = rospy.Publisher("/cartpole/state", Float64MultiArray, queue_size=1)
    rospy.Subscriber("/cartpole/cmd", Float64, on_cmd)
    rate = rospy.Rate(50)
    rospy.loginfo("cartpole_sim 已启动: 50Hz 发布 /cartpole/state")
    while not rospy.is_shutdown():
        state = rk4(state, force, DT)
        msg = Float64MultiArray(data=list(state))
        msg.layout.data_offset = 0
        pub.publish(msg)
        rate.sleep()
