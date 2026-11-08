#!/usr/bin/env python3
"""ROS Noetic 封装：狭窄通道通行。

发布：
    /gap/scan    sensor_msgs/LaserScan    91 束 270 度激光雷达
    /gap/odom    nav_msgs/Odometry        位姿
    /gap/state   std_msgs/String          approached / crossing / reached
订阅：
    /gap/cmd_vel geometry_msgs/Twist      手动模式速度指令

参数由 launch 从 config/params.yaml 加载到本节点私有命名空间，
未提供时回退到代码默认值。
"""

import os
import sys

import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

HERE = os.path.dirname(os.path.abspath(__file__))


def _package_scripts_dir():
    try:
        pkg = rospkg.RosPack().get_path("gap_nav_nn")
        cand = os.path.join(pkg, "scripts")
        if os.path.isdir(cand):
            return cand
    except Exception:
        pass
    return os.path.abspath(os.path.join(HERE, "..", "scripts"))


SCRIPTS_DIR = _package_scripts_dir()
PKG_DIR = os.path.abspath(os.path.join(SCRIPTS_DIR, ".."))
sys.path.insert(0, SCRIPTS_DIR)

from sim_core import (GapSim, N_BEAMS, ANGLE_MIN, ANGLE_MAX, MAX_RANGE,  # noqa: E402
                      MIN_PASSABLE, thrust_from_twist)
from oracle import build_path, GapExpert, DEFAULTS                      # noqa: E402
from pilot import NNPilot, features                                     # noqa: E402


class GapNode(object):
    def __init__(self):
        # —— 参数（config/params.yaml -> 节点私有命名空间）——
        self.mode = rospy.get_param("~mode", "nn")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.seed = int(rospy.get_param("~seed", 0))
        self.min_passable = float(rospy.get_param("~min_passable", MIN_PASSABLE))
        self.v_max = float(rospy.get_param("~v_max", DEFAULTS["v_max"]))
        self.lookahead = float(rospy.get_param("~lookahead", DEFAULTS["lookahead"]))
        self.model_dir = rospy.get_param("~model_dir", os.path.join(PKG_DIR, "models"))

        self.sim = GapSim(seed=self.seed)
        rng = np.random.default_rng(self.seed)
        self.obs = self.sim.reset(None, None, rng)
        self.path, self.gap = build_path(self.sim.start, self.sim.goal,
                                         self.sim.gaps, self.min_passable)
        self.expert = (GapExpert(self.path, v_max=self.v_max, lookahead=self.lookahead)
                       if self.mode == "expert" else None)
        self.pilot = NNPilot(self.model_dir) if self.mode == "nn" else None

        self.ext_v = 0.0
        self.ext_w = 0.0
        rospy.Subscriber("/gap/cmd_vel", Twist, self.cb_cmd, queue_size=1)

        self.pub_scan = rospy.Publisher("/gap/scan", LaserScan, queue_size=1)
        self.pub_odom = rospy.Publisher("/gap/odom", Odometry, queue_size=1)
        self.pub_state = rospy.Publisher("/gap/state", String, queue_size=1, latch=True)
        rospy.loginfo("gap nav node ready: mode=%s rate=%.1fHz min_passable=%.2f "
                      "v_max=%.2f lookahead=%.2f gap_width=%.2f",
                      self.mode, self.rate_hz, self.min_passable, self.v_max,
                      self.lookahead, self.gap["width"] if self.gap else 0.0)

    def cb_cmd(self, msg):
        self.ext_v = msg.linear.x
        self.ext_w = msg.angular.z

    def _scan_msg(self, stamp):
        m = LaserScan()
        m.header = Header(stamp=stamp, frame_id="gap/lidar")
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
        m.child_frame_id = "gap/base_link"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        m.pose.pose.orientation.w = float(np.cos(yaw / 2.0))
        self.pub_odom.publish(m)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.sim.reached():
                self.pub_state.publish(String(data="reached"))
            elif self.sim.crossed():
                self.pub_state.publish(String(data="crossing"))
            else:
                self.pub_state.publish(String(data="approached"))

            if self.mode == "manual":
                v, w = self.ext_v, self.ext_w
            elif self.mode == "expert":
                v, w = self.expert.act(self.sim.pose(), self.sim.speed(),
                                       self.sim.yaw_rate(), self.obs["scan"])
            else:
                v, w = self.pilot.act(features(self.sim, self.obs))

            tl, tr = thrust_from_twist(v, w)
            self.obs = self.sim.step(tl, tr)

            stamp = rospy.Time.now()
            self._scan_msg(stamp)
            self._odom_msg(stamp)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gap_nav_node")
    GapNode().spin()
