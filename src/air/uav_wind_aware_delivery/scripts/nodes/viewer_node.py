#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenCV 可视化节点。"""
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _bootstrap  # noqa: E402

import cv2  # noqa: E402
import rospy  # noqa: E402
from geometry_msgs.msg import PointStamped  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402

from uav_wind_aware_delivery.state import State  # noqa: E402
from uav_wind_aware_delivery.viz import render  # noqa: E402


class ViewerNode:
    def __init__(self):
        rospy.init_node("uav_viewer")
        self.target = (rospy.get_param("~target_x", 5.0), rospy.get_param("~target_y", 2.0))
        self.state = None
        self.predicted = None
        self.phase = "takeoff"
        self.trail = deque(maxlen=300)
        rospy.Subscriber("/uav/state", Float64MultiArray, self.on_state, queue_size=1)
        rospy.Subscriber("/uav/predicted_impact", PointStamped, self.on_pred, queue_size=1)
        rospy.Subscriber("/uav/mission", String, self.on_phase, queue_size=1)
        self.rate = rospy.Rate(30)
        self.window = "UAV Wind-Aware Delivery"
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        # 录制视频时放在终端右侧：默认 860x500 @ (395, 55)
        cv2.resizeWindow(self.window,
                         int(rospy.get_param("~win_w", 860)),
                         int(rospy.get_param("~win_h", 500)))
        cv2.moveWindow(self.window,
                       int(rospy.get_param("~win_x", 395)),
                       int(rospy.get_param("~win_y", 55)))
        rospy.on_shutdown(self.cleanup)

    def on_state(self, msg):
        try:
            self.state = State.from_vector(msg.data)
            self.trail.append((float(self.state.pos[0]), float(self.state.pos[1])))
        except Exception:
            self.state = None

    def on_pred(self, msg):
        self.predicted = (msg.point.x, msg.point.y)

    def on_phase(self, msg):
        self.phase = msg.data

    def cleanup(self):
        cv2.destroyAllWindows()

    def spin(self):
        while not rospy.is_shutdown():
            if self.state is not None:
                img = render(self.state, self.predicted, self.phase, self.trail, self.target)
                cv2.imshow(self.window, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            self.rate.sleep()
        self.cleanup()


if __name__ == "__main__":
    try:
        ViewerNode().spin()
    except rospy.ROSInterruptException:
        pass
