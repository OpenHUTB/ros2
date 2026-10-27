#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""悬崖边缘紧急制动仿真 —— 物理仿真 + 传感 + 执行节点（三节点架构之一）

架构：
  cliff_brake_sim.py   MuJoCo 物理；发布 /ultrasonic_distance；订阅 /brake_cmd 执行制动
  brake_controller.py  订阅 /ultrasonic_distance，滞回决策；发布 /brake_cmd 与 /brake_status
  brake_monitor.py     订阅 /brake_status（真实订阅者/监视）

与旧版的关键差异：
  1. 底盘增加 z 方向平移关节 —— 越过悬崖边缘会真实坠落（旧版 z 被锁死，跌落不可能发生）
  2. 距离基于车头 sensor_site 计算（旧版用车身中心，且 site 定义后从未使用）
  3. 制动力 = -K*vx 经饱和限幅，通过执行器施加；不再直接改写 qvel（数值刹车）
  4. 控制按仿真时间 100Hz 定频；仿真按墙钟节流到 RTF≈1.0（否则 ROS 话题的
     毫秒级传输延迟在高速仿真下会放大成致命的控制滞后）；实时率(RTF)实测打印

注意：本脚本需要图形界面（mujoco.viewer）。无显示环境请只运行 brake_test.py。
"""
import time

import mujoco
import mujoco.viewer
import numpy as np
import rospy
from std_msgs.msg import Float32, Bool

# ---------------- 可调参数（与文档参数表一致） ----------------
CLIFF_X = 2.5        # 悬崖边缘 x 坐标。与 floor 几何严格对应：pos(-1.25) + size(3.75)
CTRL_DT = 0.01       # 控制/发布周期（仿真时间，s），对应 100 Hz
DRIVE_FORCE = 15.0   # 巡航驱动力（N）
KV_BRAKE = 150.0     # 制动力速度增益（N/(m/s)），力与速度反向、成比例
FMAX_BRAKE = 100.0   # 制动力饱和上限（N），须不超过 actuator ctrlrange
TARGET_RTF = 1.0     # 目标实时率：仿真按墙钟节流到 1x（见 run() 内注释）
RTF_PRINT_DT = 5.0   # 实时率打印间隔（仿真时间，s）

XML_MODEL = """
<mujoco>
    <option gravity="0 0 -9.81" timestep="0.002"/>
    <worldbody>
        <light pos="0 0 3"/>
        <!-- 上层平台：x ∈ [-5, 2.5]，悬崖边缘 = -1.25 + 3.75 = 2.5 -->
        <geom name="floor" type="box" size="3.75 2.0 0.1" pos="-1.25 0 -0.1"
              rgba="0.8 0.8 0.8 1" friction="0.05 0.005 0.0001"/>
        <!-- 悬崖下层平台：顶面 z=-1.9，用于承接坠落（对照演示） -->
        <geom type="box" size="5 2 0.1" pos="7.5 0 -2.0"
              rgba="0.2 0.2 0.2 1" friction="0.05 0.005 0.0001"/>
        <body name="chassis" pos="0 0 0.05">
            <geom type="box" size="0.2 0.15 0.05" rgba="0.2 0.6 0.8 1"
                  friction="0.05 0.005 0.0001"/>
            <!-- x：驱动方向；z：竖直方向（跌落自由度，旧版缺失） -->
            <joint name="chassis_x" type="slide" axis="1 0 0" damping="0.5"/>
            <joint name="chassis_z" type="slide" axis="0 0 1" damping="0.5"/>
            <!-- 车头传感器站点：距离计算基于车头 -->
            <site name="sensor_site" pos="0.2 0 0" size="0.02" rgba="1 0 0 1"/>
        </body>
    </worldbody>
    <actuator>
        <motor joint="chassis_x" ctrlrange="-150 150"/>
    </actuator>
</mujoco>
"""


class CliffSimNode:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string(XML_MODEL)
        self.data = mujoco.MjData(self.model)
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "sensor_site")
        self.x_dof = self.model.joint("chassis_x").dofadr[0]

        self.pub_dist = rospy.Publisher('/ultrasonic_distance', Float32, queue_size=10)
        self.brake_cmd = False
        # 订阅决策节点的制动指令 —— 决策与执行分离
        rospy.Subscriber('/brake_cmd', Bool, self._on_brake_cmd, queue_size=10)
        print("仿真节点已启动：物理步长 2ms，控制频率 100Hz，实时节流 RTF=%.1f" % TARGET_RTF)

    def _on_brake_cmd(self, msg):
        self.brake_cmd = msg.data

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.distance = 4.0
            viewer.cam.lookat[:] = [1.5, 0.0, 0.0]

            next_ctrl = 0.0          # 下一次控制时刻（仿真时间）
            next_rtf = RTF_PRINT_DT
            wall_t0 = time.time()

            while viewer.is_running() and not rospy.is_shutdown():
                mujoco.mj_step(self.model, self.data)

                # —— 控制节拍：按仿真时间 100Hz，与墙钟解耦 ——
                if self.data.time >= next_ctrl:
                    next_ctrl += CTRL_DT
                    # 1) 传感：车头 site 到悬崖边缘的距离（真实车头，非车身中心）
                    front_x = self.data.site_xpos[self.site_id][0]
                    self.pub_dist.publish(CLIFF_X - front_x)
                    # 2) 渲染同步
                    viewer.sync()

                # 3) 执行：力饱和 + 速度比例的物理制动（只阻碍运动方向，不倒推）
                vx = self.data.qvel[self.x_dof]
                if self.brake_cmd:
                    self.data.ctrl[0] = float(np.clip(-KV_BRAKE * vx, -FMAX_BRAKE, 0.0))
                else:
                    self.data.ctrl[0] = DRIVE_FORCE

                # 4) 实时率实测输出（不预设"高实时"，以实测为准）
                if self.data.time >= next_rtf:
                    rtf = self.data.time / max(time.time() - wall_t0, 1e-9)
                    print("sim_t=%.1fs  RTF=%.2f  x=%.2f  z=%.2f" % (
                        self.data.time, rtf, self.data.qpos[0], self.data.qpos[1]))
                    next_rtf += RTF_PRINT_DT

                # 5) 实时节流：把仿真限制在 RTF≈1.0。
                #    若放任 RTF 远大于 1（本模型可到 ~28），车辆穿过触发区的
                #    仿真时间窗口被压缩成几毫秒真实时间，与 ROS 话题的毫秒级
                #    传输延迟同量级，制动指令会迟到、车辆冲出边缘坠落；
                #    节流到 1x 后 ROS 延迟折算回仿真时间仅数毫秒，制动可靠。
                #    软件渲染过慢时该 sleep 不生效，RTF 自然低于 1。
                sleep_need = self.data.time / TARGET_RTF - (time.time() - wall_t0)
                if sleep_need > 0:
                    time.sleep(sleep_need)


if __name__ == '__main__':
    rospy.init_node('cliff_brake_sim', anonymous=True)
    try:
        CliffSimNode().run()
    except rospy.ROSInterruptException:
        pass
