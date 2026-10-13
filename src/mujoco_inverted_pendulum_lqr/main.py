#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import warnings
import numpy as np
import mujoco
import mujoco.viewer
from scipy.linalg import solve_continuous_are
import rospy

warnings.filterwarnings("ignore")

def main():
    rospy.init_node("cartpole_lqr_node")
    rate = rospy.Rate(20) # 降低到20Hz，放慢可视化，方便录GIF

    # =========物理参数=========
    M = 1.0    # 小车质量 kg（与XML模型一致）
    m = 0.5    # 摆杆质量 kg（与XML模型一致）
    l = 0.15   # 转轴到摆杆质心距离 m（杆总长0.3m，几何中点）
    g = 9.81
    # 建模说明：LQR控制器基于【质点摆简化模型】设计，忽略摆杆自身转动惯量；
    # MuJoCo仿真使用均匀圆柱刚体，引擎自动计算摆杆真实转动惯量，二者存在理论偏差。

    # 状态 x = [x, x_dot, theta, theta_dot]
    A = np.array([
        [0, 1, 0, 0],
        [0, 0, -(m*g)/M, 0],
        [0, 0, 0, 1],
        [0, 0, ((M+m)*g)/(M*l), 0]
    ])
    B = np.array([
        [0],
        [1/M],
        [0],
        [-1/(M*l)]
    ])

    # LQR权重
    Q = np.diag([0.2, 0.1, 10, 0.5])
    R = np.array([[2.0]])

    # LQR求解
    P = solve_continuous_are(A, B, Q, R)
    K = (1.0 / R[0,0]) * B.T @ P

    # 闭环稳定性校验，优化打印，只输出实部
    A_cl = A - B @ K
    eigvals = np.linalg.eigvals(A_cl)
    rospy.loginfo(f"闭环特征值实部: {np.real(eigvals)}")
    if np.all(np.real(eigvals) < 0):
        rospy.loginfo("✅ LQR闭环系统稳定")
    else:
        rospy.logwarn("⚠️ LQR闭环不稳定！")

    # 绝对路径加载xml，解决路径问题
    xml_path = os.path.join(os.path.dirname(__file__), "balance.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 增大初始扰动，让扶正过程更长，GIF效果清晰
    data.qpos[0] = 0.0
    data.qpos[1] = 0.12
    data.qvel[:] = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        rospy.loginfo("MuJoCo仿真窗口已启动，按Tab切换广角相机 wide_view")
        while viewer.is_running() and not rospy.is_shutdown():
            x_state = np.array([data.qpos[0], data.qvel[0], data.qpos[1], data.qvel[1]])
            u = -K @ x_state
            u = np.clip(u, -10, 10) # 限幅，防止推力爆炸
            data.ctrl[0] = u[0]  # 取出标量，规避numpy数组形状警告

            mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
