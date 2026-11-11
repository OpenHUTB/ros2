#!/usr/bin/env python3
"""二连杆阻抗控制 ROS 节点（带 MuJoCo viewer + 实时曲线图）。"""
import os, sys
import numpy as np
import rospy
import mujoco
import mujoco.viewer
import matplotlib

# 后端按环境可用性挑：没装 Tk 的机器退到 Qt5Agg / Agg，import 阶段不会直接崩
for _backend in ("TkAgg", "Qt5Agg", "Agg"):
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        continue
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from impedance_controller import ImpedanceController
from output_paths import resolve_model_path, resolve_out_dir

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HERE = os.path.dirname(SCRIPT_DIR)
# 与 headless_test.py 完全相同的稳态统计窗口，避免出现第二套数字
STEADY_START = 5.0
STEADY_END = 10.0
SAMPLE_EVERY = 10      # 每 10 个物理步（2 ms/步）采样一次 = 20 ms
KP = [80.0, 50.0]
BD = [8.0, 5.0]


class ImpedanceArmNode:
    def __init__(self):
        rospy.init_node("impedance_arm_node")
        # 兼容 catkin 源码空间、devel 与 install 三种布局（与离线脚本共用同一个解析函数）
        model_path = resolve_model_path(SCRIPT_DIR, rospy.get_param("~model", None))
        self.model = mujoco.MjModel.from_xml_path(model_path)
        rospy.loginfo("模型: %s", model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)

        self.ctrl = ImpedanceController(Kp=KP, Bd=BD)

        # 解析 x_des 参数：可能是字符串 "0.62 0.0" 或列表
        raw = rospy.get_param("~x_des", "0.62 0.0")
        if isinstance(raw, str):
            self.x_des = np.array([float(v) for v in raw.split()], dtype=float)
        else:
            self.x_des = np.array(raw, dtype=float)

        # 记录数据
        self.t_list = []
        self.x_list = []
        self.fx_list = []
        self.q_list = []
        self.step_count = 0

    def get_contact_force(self):
        fx = 0.0
        for ci in range(self.data.ncon):
            cf = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, ci, cf)
            fx += cf[0]
        return fx

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            rospy.loginfo("阻抗控制启动，目标位置 x_des=(%.3f, %.3f)",
                          self.x_des[0], self.x_des[1])
            rospy.loginfo("3秒后开始动作，请观察初始弯曲姿态...")

            # 阶段1：viewer 打开后先保持 3 秒，让用户看清初始姿态
            start_time = rospy.get_time()
            while viewer.is_running() and not rospy.is_shutdown():
                if rospy.get_time() - start_time > 3.0:
                    break
                viewer.sync()
                rospy.sleep(0.01)

            # 阶段2：开始阻抗控制，慢动作模式（每步间隔 2ms，视觉速度减半）
            rate = rospy.Rate(250)  # 250Hz，比物理 500Hz 慢，视觉上慢动作
            while viewer.is_running() and not rospy.is_shutdown():
                q = self.data.qpos[:2].copy()
                qdot = self.data.qvel[:2].copy()
                tau, F, x = self.ctrl.compute_torque(q, qdot, self.x_des)
                self.data.ctrl[:] = np.clip(tau, -5, 5)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # 记录数据：按步数计数（浮点取模会因时间累积精度漏采/双采）
                if self.step_count % SAMPLE_EVERY == 0:
                    self.t_list.append(self.data.time)
                    self.x_list.append(x.copy())
                    self.fx_list.append(self.get_contact_force())
                    self.q_list.append(q.copy())
                self.step_count += 1

                rate.sleep()

        # viewer 关闭后弹出曲线图
        if len(self.t_list) > 10:
            self.plot_results()

    def plot_results(self):
        x_arr = np.array(self.x_list)
        q_arr = np.array(self.q_list)
        t = np.array(self.t_list)
        fx = np.array(self.fx_list)

        # 与 headless_test.py 用同一统计窗口 [5, 10] s，避免两套数字
        steady = (t >= STEADY_START) & (t <= STEADY_END)
        avg_fx = float(np.mean(fx[steady])) if np.any(steady) else float("nan")
        rospy.loginfo("稳态接触力（t ∈ [%.1f, %.1f] s 均值）= %.2f N", STEADY_START, STEADY_END, avg_fx)

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        axes[0].plot(t, fx, "r-", lw=1.5)
        axes[0].set_ylabel("Contact Force X (N)")
        axes[0].set_title("Impedance Control — Steady Force (t in [%.0f, %.0f] s) = %.1f N"
                          % (STEADY_START, STEADY_END, avg_fx))
        axes[0].grid(alpha=0.3)
        axes[0].axhline(0, color="k", lw=0.5)
        axes[0].axvline(STEADY_START, color="gray", ls=":", lw=0.8)
        axes[0].axvline(STEADY_END, color="gray", ls=":", lw=0.8)

        axes[1].plot(t, x_arr[:, 0], "b-", label="tip x")
        axes[1].plot(t, x_arr[:, 1], "g-", label="tip y")
        axes[1].axhline(0.58, color="gray", ls="--", label="wall surface")
        axes[1].set_ylabel("Tip Position (m)")
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3)

        axes[2].plot(t, q_arr[:, 0], label="q1")
        axes[2].plot(t, q_arr[:, 1], label="q2")
        axes[2].set_ylabel("Joint Angle (rad)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend(fontsize=8)
        axes[2].grid(alpha=0.3)

        fig.tight_layout()
        # 同时落盘一份，方便贴进文档；写不进去（只读/无图形界面）也不影响现场看图
        try:
            out = resolve_out_dir(HERE, rospy.get_param("~out", None))
            path = os.path.join(out, "impedance_node_curves.png")
            fig.savefig(path, dpi=120)
            rospy.loginfo("节点曲线已保存: %s", path)
        except Exception as exc:
            rospy.logwarn("节点曲线落盘失败: %s", exc)
        plt.show(block=True)


if __name__ == "__main__":
    node = ImpedanceArmNode()
    node.run()
