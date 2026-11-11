#!/usr/bin/env python3
"""Publish camera frustum + flight trail as RViz MarkerArray."""
from __future__ import annotations

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frustum import edge_points, world_corners  # noqa: E402


class FrustumNode(object):
    def __init__(self):
        self.near = float(rospy.get_param("~near", 0.5))
        self.far = float(rospy.get_param("~far", 8.0))
        self.max_trail = int(rospy.get_param("~max_trail", 300))
        self.pose = None
        self.caminfo = None
        self.trail = []

        self.pub = rospy.Publisher("/uav/camera_frustum", MarkerArray, queue_size=1)
        rospy.Subscriber("/uav/odom", Odometry, self.on_odom, queue_size=5)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.on_info, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.publish)
        rospy.loginfo("frustum_viz: near=%.2f far=%.2f", self.near, self.far)

    def on_odom(self, msg):
        self.pose = msg
        self.trail.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                           msg.pose.pose.position.z))
        self.trail = self.trail[-self.max_trail:]

    def on_info(self, msg):
        self.caminfo = msg

    def publish(self, _event=None):
        if self.pose is None:
            return
        p = self.pose.pose.pose.position
        q = self.pose.pose.pose.orientation
        width, height = 640, 480
        fx = fy = 320.0
        cx, cy = 320.0, 240.0
        if self.caminfo is not None:
            width, height = self.caminfo.width, self.caminfo.height
            fx, fy = self.caminfo.K[0], self.caminfo.K[4]
            cx, cy = self.caminfo.K[2], self.caminfo.K[5]
        corners = world_corners(
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
            width, height, fx, fy, cx, cy, self.near, self.far)
        edges = edge_points(corners)

        array = MarkerArray()
        frustum = Marker()
        frustum.header.frame_id = "world"
        frustum.header.stamp = rospy.Time.now()
        frustum.ns = "frustum"
        frustum.id = 0
        frustum.type = Marker.LINE_LIST
        frustum.action = Marker.ADD
        frustum.scale.x = 0.03
        frustum.color.r = 0.1
        frustum.color.g = 0.9
        frustum.color.b = 0.3
        frustum.color.a = 0.9
        frustum.points = [Point(float(x), float(y), float(z)) for x, y, z in edges]
        array.markers.append(frustum)

        trail = Marker()
        trail.header.frame_id = "world"
        trail.header.stamp = rospy.Time.now()
        trail.ns = "trail"
        trail.id = 1
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.scale.x = 0.04
        trail.color.r = 1.0
        trail.color.g = 0.6
        trail.color.b = 0.1
        trail.color.a = 0.9
        trail.points = [Point(float(x), float(y), float(z)) for x, y, z in self.trail]
        array.markers.append(trail)

        drone = Marker()
        drone.header.frame_id = "world"
        drone.header.stamp = rospy.Time.now()
        drone.ns = "drone"
        drone.id = 2
        drone.type = Marker.SPHERE
        drone.action = Marker.ADD
        drone.pose.position = p
        drone.pose.orientation = q
        drone.scale.x = drone.scale.y = drone.scale.z = 0.25
        drone.color.r = 0.9
        drone.color.g = 0.2
        drone.color.b = 0.2
        drone.color.a = 1.0
        array.markers.append(drone)

        self.pub.publish(array)


def main():
    rospy.init_node("uav_camera_frustum_viz", anonymous=False)
    FrustumNode()
    rospy.spin()


if __name__ == "__main__":
    main()
