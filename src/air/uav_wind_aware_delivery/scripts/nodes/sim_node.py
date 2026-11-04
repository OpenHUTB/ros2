#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 仿真节点：发布 /uav/state，接收 /uav/cmd 与 /uav/release。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _bootstrap  # noqa: E402

_bootstrap.use_venv()

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray  # noqa: E402

from uav_wind_aware_delivery.sim import DeliverySim, SimConfig  # noqa: E402


class SimNode:
    def __init__(self):
        rospy.init_node("uav_sim")
        wind = np.array([
            rospy.get_param("~wind_x", 3.0),
            rospy.get_param("~wind_y", 1.0),
            0.0,
        ], dtype=float)
        self.sim = DeliverySim(SimConfig(wind=wind, substeps=2))
        self.cmd = None
        self.release_requested = False
        self.state_pub = rospy.Publisher("/uav/state", Float64MultiArray, queue_size=1)
        rospy.Subscriber("/uav/cmd", Float64MultiArray, self.on_cmd, queue_size=1)
        rospy.Subscriber("/uav/release", Bool, self.on_release, queue_size=1)
        self.rate = rospy.Rate(100)
        rospy.loginfo("uav_sim started: wind=(%.2f, %.2f), model=%s",
                      wind[0], wind[1], self.sim.model_path)

    def on_cmd(self, msg):
        if len(msg.data) >= 6:
            self.cmd = (np.array(msg.data[0:3], dtype=float),
                        np.array(msg.data[3:6], dtype=float))

    def on_release(self, msg):
        if msg.data:
            self.release_requested = True

    def spin(self):
        while not rospy.is_shutdown():
            if self.release_requested and not self.sim.released:
                self.sim.release()
                rospy.loginfo("payload released at t=%.2f", self.sim.time)
            if self.cmd is None:
                self.sim.step()
            else:
                self.sim.step(self.cmd[0], self.cmd[1])
            msg = Float64MultiArray()
            msg.data = self.sim.state().as_vector().tolist()
            self.state_pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        SimNode().spin()
    except rospy.ROSInterruptException:
        pass
