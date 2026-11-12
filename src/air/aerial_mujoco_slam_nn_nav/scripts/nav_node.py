#!/usr/bin/env python3
"""无人机建图与导航节点：/scan + /drone/odom -> SLAM -> A* -> /cmd_vel。"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aerial_mujoco_slam_nn_nav.mapping import OccupancyGrid as Grid, Slam      # noqa: E402
from aerial_mujoco_slam_nn_nav.planning import CostMap, astar, simplify        # noqa: E402


class NavNode:
    def __init__(self):
        self.goal = (float(rospy.get_param("~goal_x", 2.6)), float(rospy.get_param("~goal_y", 0.0)))
        self.inflation = float(rospy.get_param("~inflation", 0.45))
        self.v_max = float(rospy.get_param("~v_max", 0.65))
        self.altitude = float(rospy.get_param("~altitude", 1.2))
        self.grid = Grid(size_x=8.0, size_y=6.0, res=0.05)
        self.slam = Slam(self.grid)
        self.path = []
        self.idx = 0
        self.pose = (0.0, 0.0, 0.0)
        self.odom = None
        self.last_odom = None
        self.last_plan = rospy.Time(0)
        self.done = False

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_alt = rospy.Publisher("/drone/altitude", Float64, queue_size=1)
        self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.pub_path = rospy.Publisher("/plan", Path, queue_size=1, latch=True)
        self.pub_pose = rospy.Publisher("/drone/slam_pose", PoseStamped, queue_size=1)
        self.pub_status = rospy.Publisher("/nav_status", String, queue_size=1)

        rospy.Subscriber("/scan", LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber("/drone/odom", Odometry, self._on_odom, queue_size=1)
        rospy.loginfo("无人机导航启动，目标 (%.2f, %.2f)，高度 %.1f m", *self.goal, self.altitude)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_scan(self, msg):
        if self.odom is None:
            return
        if self.last_odom is None:
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

    def _maybe_plan(self):
        now = rospy.Time.now()
        if (now - self.last_plan).to_sec() < 0.8 and self.path:
            return
        self.last_plan = now
        for inf in (self.inflation, 0.30, 0.22):
            cm = CostMap(self.grid, inflation=inf)
            p = astar(cm, (self.pose[0], self.pose[1]), self.goal)
            if p:
                self.path = simplify(p)
                self.idx = 0
                self._publish_path()
                return
        self.path = []

    def _control(self):
        tw = Twist()
        alt = Float64(data=self.altitude)
        self.pub_alt.publish(alt)
        if self.done:
            return
        dist = math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])
        if dist < 0.4:
            self.done = True
            self.pub_cmd.publish(tw)
            self.pub_status.publish(String(data="done"))
            return
        if not self.path:
            self.pub_status.publish(String(data="blocked"))
            return
        # 前视点
        look = 0.8
        best = self.idx
        for i in range(self.idx, len(self.path)):
            if math.hypot(self.path[i][0] - self.pose[0], self.path[i][1] - self.pose[1]) >= look:
                best = i
                break
        self.idx = best
        tx, ty = self.path[best]
        dx, dy = tx - self.pose[0], ty - self.pose[1]
        d = math.hypot(dx, dy) + 1e-9
        speed = min(self.v_max, 1.6 * d)
        wx, wy = speed * dx / d, speed * dy / d
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        tw.linear.x = c * wx + s * wy
        tw.linear.y = -s * wx + c * wy
        tw.angular.z = 0.6 * math.atan2(math.sin(math.atan2(wy, wx) - self.pose[2]),
                                        math.cos(math.atan2(wy, wx) - self.pose[2]))
        self.pub_cmd.publish(tw)
        self.pub_status.publish(String(data=f"fly v=({tw.linear.x:+.2f},{tw.linear.y:+.2f})"))

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = float(self.pose[0]), float(self.pose[1])
        msg.pose.position.z = self.altitude
        msg.pose.orientation.z = math.sin(self.pose[2] / 2)
        msg.pose.orientation.w = math.cos(self.pose[2] / 2)
        self.pub_pose.publish(msg)

    def _publish_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for x, y in self.path:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.position.z = self.altitude
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
        unknown = np.abs(self.grid.logodds) < 0.2
        msg.data = np.where(unknown, -1, self.grid.prob() * 100).astype(np.int8).ravel().tolist()
        self.pub_map.publish(msg)


if __name__ == "__main__":
    rospy.init_node("drone_nav")
    NavNode()
    rospy.spin()
