#!/usr/bin/env python3
"""建图 + 导航节点：激光 SLAM -> 代价地图 -> A* -> 纯跟踪 -> /cmd_vel。

订阅：/scan, /humanoid/odom
发布：/map, /plan, /humanoid/slam_pose, /nav_status

本节点与 sim_node 完全解耦：只吃话题、只发速度，因此同一份代码可以直接
接到真实机器人或别的仿真器上。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from humanoid_mujoco_slam_nn_nav.mapping import OccupancyGrid as Grid, Slam  # noqa: E402
from humanoid_mujoco_slam_nn_nav.planning import CostMap, PurePursuit, astar, simplify  # noqa: E402

from geometry_msgs.msg import Twist  # noqa: E402


class NavNode:
    def __init__(self):
        self.goal = (float(rospy.get_param("~goal_x", 2.4)),
                     float(rospy.get_param("~goal_y", 0.0)))
        self.res = float(rospy.get_param("~resolution", 0.05))
        self.inflation = float(rospy.get_param("~inflation", 0.30))
        self.replan_period = float(rospy.get_param("~replan_period", 0.8))
        self.grid = Grid(size_x=10.5, size_y=7.5, res=self.res)
        self.slam = Slam(self.grid)
        self.pp = PurePursuit()
        self.path = []
        self.path_idx = 0
        self.last_plan = rospy.Time(0)
        self.last_odom = None
        self.odom = (0.0, 0.0, 0.0)
        self.pose = (0.0, 0.0, 0.0)
        self.done = False

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.pub_path = rospy.Publisher("/plan", Path, queue_size=1, latch=True)
        self.pub_pose = rospy.Publisher("/humanoid/slam_pose", PoseStamped, queue_size=1)
        self.pub_status = rospy.Publisher("/nav_status", String, queue_size=1)

        rospy.Subscriber("/scan", LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber("/humanoid/odom", Odometry, self._on_odom, queue_size=1)
        rospy.loginfo("导航节点启动，目标 (%.2f, %.2f)" % self.goal)

    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     _yaw(msg.pose.pose.orientation))

    def _on_scan(self, msg: LaserScan):
        if self.last_odom is None:
            return
        if self.slam.pose[0] == 0.0 and self.slam.pose[1] == 0.0:
            self.slam.reset(self.odom)
            self.last_odom = self.odom
        dx = self.odom[0] - self.last_odom[0]
        dy = self.odom[1] - self.last_odom[1]
        dyaw = math.atan2(math.sin(self.odom[2] - self.last_odom[2]),
                          math.cos(self.odom[2] - self.last_odom[2]))
        self.last_odom = self.odom
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        self.pose = self.slam.update((dx, dy, dyaw), ranges, angles, msg.range_max)
        self._publish_pose()
        self._maybe_plan()
        self._publish_map()
        self._control()

    # ------------------------------------------------------------------
    def _maybe_plan(self):
        now = rospy.Time.now()
        if (now - self.last_plan).to_sec() < self.replan_period and self.path:
            return
        self.last_plan = now
        for inflation in (self.inflation, 0.30, 0.22, 0.16):
            cm = CostMap(self.grid, inflation=inflation)
            path = astar(cm, (self.pose[0], self.pose[1]), self.goal)
            if path:
                self.path = simplify(path)
                self.path_idx = 0
                self._publish_path()
                return
        self.path = []

    def _control(self):
        if self.done:
            return
        dist = math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])
        tw = Twist()
        if dist < 0.35:
            self.done = True
            self.pub_status.publish(String(data="done"))
            self.pub_cmd.publish(tw)
            return
        if not self.path:
            self.pub_status.publish(String(data="blocked"))
            return
        v, w, self.path_idx = self.pp.control(self.pose, self.path, self.path_idx)
        tw.linear.x = v
        tw.angular.z = w
        self.pub_cmd.publish(tw)
        self.pub_status.publish(String(data="walk v=%.2f w=%+.2f" % (v, w)))

    # ------------------------------------------------------------------
    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = float(self.pose[0]), float(self.pose[1])
        q = _quat(self.pose[2])
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.pub_pose.publish(msg)

    def _publish_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for x, y in self.path:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            path.poses.append(p)
        self.pub_path.publish(path)

    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid.res
        msg.info.width = self.grid.nx
        msg.info.height = self.grid.ny
        msg.info.origin.position.x = float(self.grid.origin[0])
        msg.info.origin.position.y = float(self.grid.origin[1])
        p = self.grid.prob()
        unknown = np.abs(self.grid.logodds) < 0.2
        data = np.where(unknown, -1, (p * 100)).astype(np.int8)
        msg.data = data.ravel().tolist()
        self.pub_map.publish(msg)


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


if __name__ == "__main__":
    rospy.init_node("humanoid_nav")
    NavNode()
    rospy.spin()
