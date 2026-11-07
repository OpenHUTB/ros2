#!/usr/bin/env python3
"""ROS Noetic 封装：无人艇自主靠泊。

发布：
    /usv/scan      sensor_msgs/LaserScan        91 束激光雷达
    /usv/odom      nav_msgs/Odometry            位姿
    /usv/berth     std_msgs/Float32MultiArray   泊位相对偏差 [横向, 艏向, 纵向距离]
    /usv/state     std_msgs/String              approach / align / docked
订阅：
    /usv/cmd_vel   geometry_msgs/Twist          手动模式下的速度指令
"""

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Header, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from sim_core import (DockSim, N_BEAMS, ANGLE_MIN, ANGLE_MAX, MAX_RANGE,  # noqa: E402
                      thrust_from_twist, TOL_Y, TOL_YAW)
from oracle import DockExpert, build_path                                 # noqa: E402
from nn import MLP                                                        # noqa: E402


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "dock_planner.npz"))
        z = np.load(os.path.join(model_dir, "dock_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def act(self, obs):
        e = obs["berth"]
        feat = np.concatenate([obs["scan"],
                               [np.clip(e[0], -3, 3), np.clip(e[1], -1, 1),
                                np.clip(e[2], -20, 20), obs["speed"]]])
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], 0.0, 1.5))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -0.8, 0.8))
        a = 0.5
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


class DockNode(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "nn")
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.seed = rospy.get_param("~seed", 0)
        self.model_dir = rospy.get_param("~model_dir", os.path.join(HERE, "..", "models"))

        self.sim = DockSim(seed=self.seed)
        self.obs = self.sim.reset(seed=self.seed)
        self.expert = DockExpert(build_path(self.sim.start)) if self.mode == "expert" else None
        self.pilot = NNPilot(self.model_dir) if self.mode == "nn" else None

        self.ext_v = 0.0
        self.ext_w = 0.0
        rospy.Subscriber("/usv/cmd_vel", Twist, self.cb_cmd, queue_size=1)

        self.pub_scan = rospy.Publisher("/usv/scan", LaserScan, queue_size=1)
        self.pub_odom = rospy.Publisher("/usv/odom", Odometry, queue_size=1)
        self.pub_berth = rospy.Publisher("/usv/berth", Float32MultiArray, queue_size=1)
        self.pub_state = rospy.Publisher("/usv/state", String, queue_size=1, latch=True)
        rospy.loginfo("Dock node ready: mode=%s rate=%.1fHz", self.mode, self.rate_hz)

    def cb_cmd(self, msg):
        self.ext_v = msg.linear.x
        self.ext_w = msg.angular.z

    def _scan_msg(self, stamp):
        m = LaserScan()
        m.header = Header(stamp=stamp, frame_id="usv/lidar")
        m.angle_min = float(ANGLE_MIN)
        m.angle_max = float(ANGLE_MAX)
        m.angle_increment = float((ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1))
        m.range_min = 0.05
        m.range_max = float(MAX_RANGE)
        m.ranges = [float(r) for r in self.obs["scan"]]
        self.pub_scan.publish(m)

    def _odom_msg(self, stamp):
        x, y, yaw = self.sim.pose()
        m = Odometry()
        m.header = Header(stamp=stamp, frame_id="map")
        m.child_frame_id = "usv/base_link"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        m.pose.pose.orientation.w = float(np.cos(yaw / 2.0))
        self.pub_odom.publish(m)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            e = self.obs["berth"]
            self.pub_berth.publish(Float32MultiArray(data=[float(e[0]), float(e[1]), float(e[2])]))

            if self.sim.docked():
                self.pub_state.publish(String(data="docked"))
            elif e[2] > 9.0:
                self.pub_state.publish(String(data="approach"))
            else:
                self.pub_state.publish(String(data="align"))

            if self.mode == "manual":
                v, w = self.ext_v, self.ext_w
            elif self.mode == "expert":
                v, w = self.expert.act(self.sim.pose(), self.sim.speed(), self.sim.yaw_rate())
            else:
                v, w = self.pilot.act(self.obs)

            tl, tr = thrust_from_twist(v, w)
            self.obs = self.sim.step(tl, tr)

            stamp = rospy.Time.now()
            self._scan_msg(stamp)
            self._odom_msg(stamp)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("usv_dock_node")
    DockNode().spin()
