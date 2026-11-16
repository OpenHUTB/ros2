#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地仿真后端（不依赖 OpenHUTB）。

用途：在模拟器尚未就绪时验证整条 ROS 链路，也让评审者不必安装 OpenHUTB 就能复现。
它与 airsim_bridge 提供**完全相同的话题与服务**，因此上层算法无需改动：

    /uav/odom  /uav/lidar/scan  /uav/cmd_vel  /uav/takeoff  /uav/land

场景为 10 m × 10 m 的封闭场地，内含两根立柱与一个矩形障碍；
激光为前向 180°、1° 分辨率，直接对几何体做步进求交。

参数
    ~world        默认 auto（10×10 场地 + 3 个障碍）
    ~rate         主循环 20 Hz
    ~lidar_bins   180
    ~range_max    40 m
"""

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


# 障碍：矩形 (xmin, xmax, ymin, ymax) 与 圆柱 (cx, cy, r)
BOXES = [
    (-5.0, 5.0, 4.85, 5.15),    # 北墙
    (-5.0, 5.0, -5.15, -4.85),  # 南墙
    (4.85, 5.15, -5.0, 5.0),    # 东墙
    (-5.15, -4.85, -5.0, 5.0),  # 西墙
    (1.2, 2.2, -0.5, 0.5),      # 中部矩形障碍
]
CIRCLES = [
    (-2.0, 2.0, 0.4),
    (2.5, -2.5, 0.4),
]


def point_blocked(x, y):
    for (x0, x1, y0, y1) in BOXES:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    for (cx, cy, r) in CIRCLES:
        if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
            return True
    return False


class MockSim(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.bins = int(rospy.get_param("~lidar_bins", 180))
        self.range_max = float(rospy.get_param("~range_max", 40.0))
        self.range_min = 0.3
        self.z = 2.0
        self.tau = 0.35
        self.max_speed = 3.0

        self.x = float(rospy.get_param("~init_x", 0.0))
        self.y = float(rospy.get_param("~init_y", -3.0))
        self.yaw = float(rospy.get_param("~init_yaw", 1.5708))
        self.vx = self.vy = self.wz = 0.0
        self.cmd = Twist()
        self.flying = bool(rospy.get_param("~auto_takeoff", True))
        self.last = rospy.Time.now()

        self.pub_odom = rospy.Publisher("/uav/odom", Odometry, queue_size=1)
        self.pub_scan = rospy.Publisher("/uav/lidar/scan", LaserScan, queue_size=1)
        self.tf_br = TransformBroadcaster()
        rospy.Subscriber("/uav/cmd_vel", Twist, self.on_cmd, queue_size=1)
        rospy.Service("/uav/takeoff", Trigger, self.on_takeoff)
        rospy.Service("/uav/land", Trigger, self.on_land)
        rospy.loginfo("本地仿真后端已启动（10×10 m 场地，3 个障碍）")

    def on_cmd(self, msg):
        self.cmd = msg

    def on_takeoff(self, _req):
        self.flying = True
        self.z = 2.0
        return TriggerResponse(success=True, message="已起飞（本地仿真）")

    def on_land(self, _req):
        self.flying = False
        self.vx = self.vy = self.wz = 0.0
        return TriggerResponse(success=True, message="已降落（本地仿真）")

    def step(self, dt):
        if not self.flying:
            self.vx = self.vy = self.wz = 0.0
            return
        ax = (self.cmd.linear.x - self.vx) / self.tau
        ay = (self.cmd.linear.y - self.vy) / self.tau
        aw = (self.cmd.angular.z - self.wz) / self.tau
        self.vx = max(-self.max_speed, min(self.max_speed, self.vx + ax * dt))
        self.vy = max(-self.max_speed, min(self.max_speed, self.vy + ay * dt))
        self.wz = self.wz + aw * dt

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        nx = self.x + (self.vx * cy - self.vy * sy) * dt
        ny = self.y + (self.vx * sy + self.vy * cy) * dt
        if not point_blocked(nx, ny):
            self.x, self.y = nx, ny
        else:
            self.vx = self.vy = 0.0
        self.yaw += self.wz * dt

    def publish_odom(self, now):
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

    def publish_scan(self, now):
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = (scan.angle_max - scan.angle_min) / max(self.bins - 1, 1)
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        step = 0.1
        ranges = []
        for i in range(self.bins):
            a = self.yaw + scan.angle_min + i * scan.angle_increment
            ca, sa = math.cos(a), math.sin(a)
            r = self.range_min
            hit = self.range_max
            while r < self.range_max:
                if point_blocked(self.x + ca * r, self.y + sa * r):
                    hit = r
                    break
                r += step
            ranges.append(float(hit))
        scan.ranges = ranges
        self.pub_scan.publish(scan)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1e-4, (now - self.last).to_sec())
            self.last = now
            self.step(min(dt, 0.1))
            self.publish_odom(now)
            self.publish_scan(now)
            rate.sleep()


def main():
    rospy.init_node("uav_mock_sim")
    MockSim().spin()


if __name__ == "__main__":
    main()
