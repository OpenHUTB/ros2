#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真节点：MuJoCo 步进 + 分层控制组合，发布关节状态与躯干状态。

话题：
  订阅 /msk_human/manual_cmd (std_msgs/Float32MultiArray, 22 维手动肌肉激活)
  订阅 /msk_human/action_cmd (std_msgs/String, stand|walk|squat|kick|relax|faster|slower)
  发布 /msk_human/joint_states (sensor_msgs/JointState, 13 关节)
  发布 /msk_human/state (std_msgs/Float32MultiArray, [t, x, z, pitch, fallen])

控制回路由仿真节点以 500 Hz 执行（0.002 s 步长）：
  手动激活(键盘) + 姿势反射(动作参考) -> 肌肉动力学 -> MuJoCo 步进
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

from msk_human_core import JOINT_NAMES, Composer, MskHumanSim, N_MUSCLE


class SimNode:
    def __init__(self):
        scene = rospy.get_param("~scene", "flat")
        viewer = bool(rospy.get_param("~viewer", True))
        load_mass = float(rospy.get_param("~load_mass", 10.0))
        self.sim = MskHumanSim(scene, viewer=viewer, load_mass=load_mass)
        self.comp = Composer()
        self.manual = np.zeros(N_MUSCLE)

        self.sub_manual = rospy.Subscriber(
            "/msk_human/manual_cmd", Float32MultiArray, self.on_manual,
            queue_size=1)
        self.sub_action = rospy.Subscriber(
            "/msk_human/action_cmd", String, self.on_action, queue_size=1)
        self.pub_js = rospy.Publisher(
            "/msk_human/joint_states", JointState, queue_size=1)
        self.pub_state = rospy.Publisher(
            "/msk_human/state", Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(500)
        self.js_seq = 0
        rospy.loginfo("msk_human_sim_node: scene=%s viewer=%s", scene, viewer)

    def on_manual(self, msg):
        if len(msg.data) == N_MUSCLE:
            self.manual = np.clip(np.array(msg.data), 0.0, 1.0)

    def on_action(self, msg):
        name = msg.data.strip().lower()
        if name in ("stand", "walk", "squat", "kick", "relax"):
            self.comp.set_action(name)
        elif name == "faster":
            self.comp.nudge_speed(+0.15)
        elif name == "slower":
            self.comp.nudge_speed(-0.15)
        rospy.loginfo("action: %s", name)

    def spin(self):
        dt = 0.002
        js_every = 10  # 500 Hz -> 50 Hz 状态发布
        i = 0
        while not rospy.is_shutdown():
            self.comp.manual = self.manual
            u = self.comp.update(self.sim, dt)
            self.sim.step(u)
            if i % js_every == 0:
                self.publish_state()
            i += 1
            self.rate.sleep()

    def publish_state(self):
        q, qd = self.sim.q(), self.sim.qd()
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.seq = self.js_seq
        js.name = JOINT_NAMES
        js.position = [float(v) for v in q]
        js.velocity = [float(v) for v in qd]
        self.pub_js.publish(js)
        st = Float32MultiArray()
        st.data = [float(self.sim.time), float(self.sim.pelvis_x()),
                   float(self.sim.pelvis_z()), float(self.sim.pitch()),
                   1.0 if self.sim.fallen() else 0.0]
        self.pub_state.publish(st)
        self.js_seq += 1


if __name__ == "__main__":
    rospy.init_node("msk_human_sim_node")
    SimNode().spin()
