#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""几何控制节点：/uav/state + /uav/setpoint -> /uav/cmd。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _bootstrap  # noqa: E402,F401

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from uav_wind_aware_delivery.control import GeometricController  # noqa: E402
from uav_wind_aware_delivery.state import State  # noqa: E402


class ControlNode:
    def __init__(self):
        rospy.init_node("uav_control")
        self.ctrl = GeometricController()
        self.state = None
        self.setpoint = None
        self.cmd_pub = rospy.Publisher("/uav/cmd", Float64MultiArray, queue_size=1)
        rospy.Subscriber("/uav/state", Float64MultiArray, self.on_state, queue_size=1)
        rospy.Subscriber("/uav/setpoint", PoseStamped, self.on_setpoint, queue_size=1)
        self.rate = rospy.Rate(100)

    def on_state(self, msg):
        try:
            self.state = State.from_vector(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "bad state: %s", exc)

    def on_setpoint(self, msg):
        p = msg.pose.position
        self.setpoint = np.array([p.x, p.y, p.z], dtype=float)

    def spin(self):
        while not rospy.is_shutdown():
            if self.state is not None and self.setpoint is not None:
                f, tau = self.ctrl.compute(self.state, self.setpoint, np.zeros(3))
                msg = Float64MultiArray()
                msg.data = np.concatenate([f, tau]).tolist()
                self.cmd_pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        ControlNode().spin()
    except rospy.ROSInterruptException:
        pass
