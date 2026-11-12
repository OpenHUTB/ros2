#!/usr/bin/env python3
"""ROS Noetic 封装：动态障碍物避让。

发布：
    /dyn/scan      sensor_msgs/LaserScan    91 束 360° 激光雷达
    /dyn/odom      nav_msgs/Odometry        位姿
    /dyn/state     std_msgs/String          yield / cruising / reached
订阅：
    /dyn/cmd_vel   geometry_msgs/Twist      手动模式速度指令
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
        pkg = rospkg.RosPack().get_path("dyn_avoid_nn")
        cand = os.path.join(pkg, "scripts")
        if os.path.isdir(cand):
            return cand
    except Exception:
        pass
    return os.path.abspath(os.path.join(HERE, "..", "scripts"))


SCRIPTS_DIR = _package_scripts_dir()
PKG_DIR = os.path.abspath(os.path.join(SCRIPTS_DIR, ".."))
sys.path.insert(0, SCRIPTS_DIR)

from sim_core import (DynSim, N_BEAMS, ANGLE_MIN, ANGLE_MAX, MAX_RANGE,  # noqa: E402
                      GOAL, thrust_from_twist)
from oracle import build_path, DynExpert                                 # noqa: E402
from nn import MLP                                                       # noqa: E402


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "dyn_planner.npz"))
        z = np.load(os.path.join(model_dir, "dyn_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def act(self, sim, obs, prev):
        feat = np.concatenate([obs["scan"], prev,
                               [sim.goal_bearing(), min(sim.goal_distance(), 20.0), obs["speed"]]])
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], -0.3, 1.0))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2))
        a = 0.5
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


class DynNode(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "nn")
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.seed = rospy.get_param("~seed", 0)
        self.model_dir = rospy.get_param("~model_dir", os.path.join(PKG_DIR, "models"))

        self.sim = DynSim(seed=self.seed)
        rng = np.random.default_rng(self.seed)
        self.obs = self.sim.reset(None, rng)
        self.path = build_path(self.sim.obstacles, self.sim.pose()[:2], GOAL)
        self.expert = DynExpert(self.path) if self.mode == "expert" else None
        self.pilot = NNPilot(self.model_dir) if self.mode == "nn" else None
        self.prev = self.obs["scan"].copy()

        self.ext_v = 0.0
        self.ext_w = 0.0
        rospy.Subscriber("/dyn/cmd_vel", Twist, self.cb_cmd, queue_size=1)

        self.pub_scan = rospy.Publisher("/dyn/scan", LaserScan, queue_size=1)
        self.pub_odom = rospy.Publisher("/dyn/odom", Odometry, queue_size=1)
        self.pub_state = rospy.Publisher("/dyn/state", String, queue_size=1, latch=True)
        rospy.loginfo("dyn avoid node ready: mode=%s rate=%.1fHz", self.mode, self.rate_hz)

    def cb_cmd(self, msg):
        self.ext_v = msg.linear.x
        self.ext_w = msg.angular.z

    def _scan_msg(self, stamp):
        m = LaserScan()
        m.header = Header(stamp=stamp, frame_id="dyn/lidar")
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
        m.child_frame_id = "dyn/base_link"
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
            elif self.mode == "nn":
                self.pub_state.publish(String(data="cruising"))
            else:
                self.pub_state.publish(String(data="cruising"))

            if self.mode == "manual":
                v, w = self.ext_v, self.ext_w
            elif self.mode == "expert":
                v, w = self.expert.act(self.sim.pose(), self.sim.speed(), self.sim.yaw_rate(),
                                       self.obs["movers"], self.obs["scan"])
            else:
                v, w = self.pilot.act(self.sim, self.obs, self.prev)

            self.prev = self.obs["scan"].copy()
            tl, tr = thrust_from_twist(v, w)
            self.obs = self.sim.step(tl, tr)

            stamp = rospy.Time.now()
            self._scan_msg(stamp)
            self._odom_msg(stamp)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("dyn_avoid_node")
    DynNode().spin()
