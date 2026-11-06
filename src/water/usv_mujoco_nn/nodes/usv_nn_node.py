#!/usr/bin/env python3
"""ROS Noetic 封装节点：把 MuJoCo 水面无人艇接入 ROS 话题。

发布：
    /usv/scan      sensor_msgs/LaserScan     二维激光雷达（270°，91 束）
    /usv/odom      nav_msgs/Odometry         位姿与速度
    /usv/map       nav_msgs/OccupancyGrid    在线占据栅格地图
订阅：
    /usv/cmd_vel   geometry_msgs/Twist       外部速度指令（manual 模式下生效）
"""

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from sim_core import (USVSim, N_BEAMS, ANGLE_MIN, ANGLE_MAX, MAX_RANGE,  # noqa: E402
                      thrust_from_twist)
from occupancy import OccupancyMap                                        # noqa: E402
from oracle import GridPlanner                                            # noqa: E402
from policy import NNPolicy                                              # noqa: E402

CROP = 21


class NNPilot(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "planner_mlp.npz"))
        z = np.load(os.path.join(model_dir, "norm_stats.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]

    def act(self, feat):
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        return (float(np.clip(yn[0] * self.ys[0] + self.ym[0], 0.0, 2.0)),
                float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2)))


class USVNode(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "nn")
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.model_dir = rospy.get_param("~model_dir", os.path.join(HERE, "..", "models"))
        self.seed = rospy.get_param("~seed", 7)

        self.sim = USVSim(seed=self.seed)
        self.occ = OccupancyMap()
        self.obs = self.sim.reset(randomize=(self.mode != "manual"))

        self.pilot = None
        if self.mode == "nn":
            self.pilot = NNPolicy(self.model_dir)
        self.path = None
        if self.mode == "expert":
            self.path = GridPlanner(self.sim.obstacles).plan(self.sim.start[:2], self.sim.goal)

        self.ext_v = 0.0
        self.ext_w = 0.0
        rospy.Subscriber("/usv/cmd_vel", Twist, self.cb_cmd, queue_size=1)

        self.pub_scan = rospy.Publisher("/usv/scan", LaserScan, queue_size=1)
        self.pub_odom = rospy.Publisher("/usv/odom", Odometry, queue_size=1)
        self.pub_map = rospy.Publisher("/usv/map", OccupancyGrid, queue_size=1, latch=True)

        rospy.loginfo("USV node started: mode=%s rate=%.1fHz", self.mode, self.rate_hz)

    def cb_cmd(self, msg):
        self.ext_v = msg.linear.x
        self.ext_w = msg.angular.z

    def _publish_scan(self, stamp):
        msg = LaserScan()
        msg.header = Header(stamp=stamp, frame_id="usv/lidar")
        msg.angle_min = float(ANGLE_MIN)
        msg.angle_max = float(ANGLE_MAX)
        msg.angle_increment = float((ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1))
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.rate_hz
        msg.range_min = 0.05
        msg.range_max = float(MAX_RANGE)
        msg.ranges = [float(r) for r in self.obs["scan"]]
        self.pub_scan.publish(msg)

    def _publish_odom(self, stamp):
        x, y, yaw = self.sim.pose()
        msg = Odometry()
        msg.header = Header(stamp=stamp, frame_id="map")
        msg.child_frame_id = "usv/base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # 绕 z 轴旋转的直接四元数形式，省去对 tf 的依赖
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        msg.pose.pose.orientation.w = float(np.cos(yaw / 2.0))
        self.pub_odom.publish(msg)

    def _publish_map(self):
        prob = self.occ.probability()
        grid = np.where(prob > 0.65, 100, np.where(prob < 0.35, 0, -1)).astype(np.int8)
        msg = OccupancyGrid()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        msg.info.resolution = self.occ.res
        msg.info.width = self.occ.n
        msg.info.height = self.occ.n
        msg.info.origin.position.x = -self.occ.half
        msg.info.origin.position.y = -self.occ.half
        msg.data = grid.reshape(-1).tolist()
        self.pub_map.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        step = 0
        while not rospy.is_shutdown():
            pose = self.sim.pose()
            self.occ.update(pose, self.obs["scan"], ANGLE_MIN, ANGLE_MAX, MAX_RANGE)

            if self.mode == "manual":
                v, w = self.ext_v, self.ext_w
            elif self.mode == "expert":
                v, w = GridPlanner(self.sim.obstacles).pure_pursuit(pose, self.path)
            else:
                feat = np.concatenate([self.occ.crop(pose, CROP).reshape(-1),
                                       [self.obs["goal_body"][0],
                                        self.obs["goal_body"][1], self.obs["goal_dist"]]])
                v, w = self.pilot.act(feat, self.obs["scan"])

            tl, tr = thrust_from_twist(v, w)
            self.obs = self.sim.step(tl, tr)

            stamp = rospy.Time.now()
            self._publish_scan(stamp)
            self._publish_odom(stamp)
            if step % int(self.rate_hz) == 0:
                self._publish_map()
            step += 1
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("usv_nn_node")
    USVNode().spin()
