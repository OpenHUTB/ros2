#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""落点预测节点：NN 预测当前释放的落点，发布 /uav/predicted_impact。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _bootstrap  # noqa: E402

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from geometry_msgs.msg import PointStamped  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from uav_wind_aware_delivery.ballistics import simulate_drop  # noqa: E402
from uav_wind_aware_delivery.nn import ImpactMLP  # noqa: E402
from uav_wind_aware_delivery.state import State  # noqa: E402


class PredictorNode:
    def __init__(self):
        rospy.init_node("uav_predictor")
        path = os.path.join(_bootstrap.PKG_ROOT, "models", "impact_mlp.npz")
        self.model = ImpactMLP.load(path) if os.path.exists(path) else None
        self.state = None
        self.pub = rospy.Publisher("/uav/predicted_impact", PointStamped, queue_size=1)
        rospy.Subscriber("/uav/state", Float64MultiArray, self.on_state, queue_size=1)
        self.rate = rospy.Rate(30)
        rospy.loginfo("predictor loaded=%s", self.model is not None)

    def on_state(self, msg):
        try:
            self.state = State.from_vector(msg.data)
        except Exception:
            self.state = None

    def spin(self):
        while not rospy.is_shutdown():
            if self.state is not None:
                st = self.state
                if st.released:
                    h = float(st.payload_pos[2])
                    vx, vy, vz = st.payload_vel
                    base = st.payload_pos[:2]
                else:
                    h = float(st.pos[2])
                    vx, vy, vz = st.vel
                    base = st.pos[:2]
                if h > 0.05:
                    if self.model is not None:
                        off = self.model.predict(np.array([[h, vx, vy, vz,
                                                            st.wind[0], st.wind[1]]]))[0]
                    else:
                        off = simulate_drop(h, vx, vy, vz, st.wind)
                    pt = PointStamped()
                    pt.header.stamp = rospy.Time.now()
                    pt.header.frame_id = "map"
                    pt.point.x = float(base[0] + off[0])
                    pt.point.y = float(base[1] + off[1])
                    pt.point.z = 0.0
                    self.pub.publish(pt)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        PredictorNode().spin()
    except rospy.ROSInterruptException:
        pass
