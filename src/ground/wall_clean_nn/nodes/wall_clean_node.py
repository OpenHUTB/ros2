#!/usr/bin/env python3
"""ROS Noetic 封装：室内清扫车沿墙清扫。

发布：
    /cleaner/scan      sensor_msgs/LaserScan       61 束激光雷达
    /cleaner/odom      nav_msgs/Odometry           位姿
    /cleaner/coverage  std_msgs/Float32            沿墙覆盖率
    /cleaner/state     std_msgs/String             approach / following / done
订阅：
    /cleaner/cmd_vel   geometry_msgs/Twist         手动模式速度指令
"""

import os
import sys

import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Header, String

HERE = os.path.dirname(os.path.abspath(__file__))


def _package_scripts_dir():
    """安装空间用 rospkg 定位；源码空间回退到 ../scripts。"""
    try:
        pkg = rospkg.RosPack().get_path("wall_clean_nn")
        cand = os.path.join(pkg, "scripts")
        if os.path.isdir(cand):
            return cand
    except Exception:
        pass
    return os.path.abspath(os.path.join(HERE, "..", "scripts"))


SCRIPTS_DIR = _package_scripts_dir()
PKG_DIR = os.path.abspath(os.path.join(SCRIPTS_DIR, ".."))
sys.path.insert(0, SCRIPTS_DIR)

from sim_core import (CoverSim, N_BEAMS, ANGLE_MIN, ANGLE_MAX, MAX_RANGE,  # noqa: E402
                      thrust_from_twist)
from oracle import build_wall_path, WallExpert, sample_start              # noqa: E402
from nn import MLP                                                        # noqa: E402


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "wall_planner.npz"))
        z = np.load(os.path.join(model_dir, "wall_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.v = self.w = None

    def act(self, sim, obs):
        cov = sim.local_coverage(size=11).reshape(-1)
        feat = np.concatenate([obs["scan"], cov, [obs["speed"]]])
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], -0.3, 1.0))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2))
        a = 0.5
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w


class CleanNode(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "nn")
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.seed = rospy.get_param("~seed", 0)
        self.model_dir = rospy.get_param("~model_dir", os.path.join(PKG_DIR, "models"))

        self.sim = CoverSim(seed=self.seed)
        rng = np.random.default_rng(self.seed)
        start = sample_start(rng, self.sim.obstacles)
        self.obs = self.sim.reset(start)
        self.path, _ = build_wall_path(self.sim.obstacles, start[:2])
        self.expert = WallExpert(self.path) if self.mode == "expert" else None
        self.pilot = NNPilot(self.model_dir) if self.mode == "nn" else None

        self.ext_v = 0.0
        self.ext_w = 0.0
        rospy.Subscriber("/cleaner/cmd_vel", Twist, self.cb_cmd, queue_size=1)

        self.pub_scan = rospy.Publisher("/cleaner/scan", LaserScan, queue_size=1)
        self.pub_odom = rospy.Publisher("/cleaner/odom", Odometry, queue_size=1)
        self.pub_cov = rospy.Publisher("/cleaner/coverage", Float32, queue_size=1)
        self.pub_state = rospy.Publisher("/cleaner/state", String, queue_size=1, latch=True)
        rospy.loginfo("clean node ready: mode=%s rate=%.1fHz", self.mode, self.rate_hz)

    def cb_cmd(self, msg):
        self.ext_v = msg.linear.x
        self.ext_w = msg.angular.z

    def _scan_msg(self, stamp):
        m = LaserScan()
        m.header = Header(stamp=stamp, frame_id="cleaner/lidar")
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
        m.child_frame_id = "cleaner/base_link"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        m.pose.pose.orientation.w = float(np.cos(yaw / 2.0))
        self.pub_odom.publish(m)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            cov = self.sim.wall_coverage()
            self.pub_cov.publish(Float32(data=float(cov)))
            if cov > 0.94:
                self.pub_state.publish(String(data="done"))
            elif cov < 0.05:
                self.pub_state.publish(String(data="approach"))
            else:
                self.pub_state.publish(String(data="following"))

            if self.mode == "manual":
                v, w = self.ext_v, self.ext_w
            elif self.mode == "expert":
                v, w = self.expert.act(self.sim.pose(), self.sim.speed(),
                                       self.sim.yaw_rate(), self.obs["scan"])
            else:
                v, w = self.pilot.act(self.sim, self.obs)

            tl, tr = thrust_from_twist(v, w)
            self.obs = self.sim.step(tl, tr)

            stamp = rospy.Time.now()
            self._scan_msg(stamp)
            self._odom_msg(stamp)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("wall_clean_node")
    CleanNode().spin()
