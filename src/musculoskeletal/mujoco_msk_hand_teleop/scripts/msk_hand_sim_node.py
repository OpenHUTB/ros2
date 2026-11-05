#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真节点：订阅肌肉激活指令，步进 MuJoCo，发布手部状态。

话题：
  订阅  /msk_hand/muscle_cmd  (std_msgs/Float32MultiArray)  11 维肌肉激活 0~1
  发布  /msk_hand/state       (std_msgs/Float32MultiArray)
        [t, 8 关节角, 3 指尖×3 坐标, 任务指标]
        任务指标：free=目标距离, button=按键深度, ball=球离台面高度
参数：
  ~scene  (str, 默认 free)   场景: free / button / ball
  ~rate   (int, 默认 100)    物理步频 Hz（内部每拍 5 个 2ms 子步）
  ~viewer (bool, 默认 false) 是否打开 MuJoCo 交互查看器
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import rospy
from std_msgs.msg import Float32MultiArray

from msk_hand_core import MskHandSim, N_MUSCLE


class SimNode:
    def __init__(self):
        scene = rospy.get_param("~scene", "free")
        rate = rospy.get_param("~rate", 100)
        use_viewer = rospy.get_param("~viewer", False)
        self.sim = MskHandSim(scene)
        self.sim.reset()
        self.cmd = [0.0] * N_MUSCLE
        self.scene = scene
        self.sub = rospy.Subscriber("/msk_hand/muscle_cmd", Float32MultiArray,
                                    self.on_cmd, queue_size=1)
        self.pub = rospy.Publisher("/msk_hand/state", Float32MultiArray,
                                   queue_size=1)
        self.viewer = None
        if use_viewer:
            try:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.sim.model,
                                                           self.sim.data)
            except Exception as e:  # 无显示环境时降级为无头
                rospy.logwarn("viewer 不可用(%s)，转为无头模式" % e)
        self.rate = rospy.Rate(rate)
        rospy.loginfo("msk_hand_sim_node 就绪: scene=%s rate=%dHz", scene, rate)

    def on_cmd(self, msg):
        data = list(msg.data)[:N_MUSCLE]
        self.cmd = data + [0.0] * (N_MUSCLE - len(data))

    def task_metric(self):
        if self.scene == "button":
            return self.sim.button_depth()
        if self.scene == "ball":
            return self.sim.ball_height()
        return 0.0

    def spin(self):
        last_pub = rospy.Time(0)
        while not rospy.is_shutdown():
            self.sim.step(self.cmd)
            if self.viewer is not None:
                self.viewer.sync()
            # 25Hz 发布状态
            if (rospy.Time.now() - last_pub).to_sec() > 0.04:
                last_pub = rospy.Time.now()
                st = ([self.sim.time] + list(self.sim.joint_positions()) +
                      list(self.sim.tip_positions().ravel()) +
                      [self.task_metric()])
                msg = Float32MultiArray()
                msg.data = [float(v) for v in st]
                self.pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("msk_hand_sim_node")
    SimNode().spin()
