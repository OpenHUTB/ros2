#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SLAM 建图演示脚本：实时打印地图尺寸和无人机位置。"""
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry

map_info = "等待地图..."
odom_info = "等待里程计..."

def map_cb(msg):
    global map_info
    w = msg.info.width
    h = msg.info.height
    occupied = sum(1 for v in msg.data if v > 0)
    map_info = f"地图: {w}x{h}  障碍物: {occupied}"

def odom_cb(msg):
    global odom_info
    p = msg.pose.pose.position
    odom_info = f"无人机: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}"

rospy.init_node("demo", anonymous=True)
rospy.Subscriber("/map", OccupancyGrid, map_cb, queue_size=1)
rospy.Subscriber("/uav/odom", Odometry, odom_cb, queue_size=1)

print("=" * 50)
print("  SLAM 建图 + 自主避障导航 演示")
print("=" * 50)
print()

rate = rospy.Rate(2)
while not rospy.is_shutdown():
    print(f"\r{map_info}  |  {odom_info}        ", end="")
    rate.sleep()
