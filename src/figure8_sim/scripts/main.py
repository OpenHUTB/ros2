#!/usr/bin/env python3
import os
import sys

# 安装空间里 main.py 与 figure8.py 会装到同一个目录（lib/figure8_sim/），
# 这里显式把脚本目录加入 sys.path，保证 install 之后也能 import。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from visualization_msgs.msg import Marker

from figure8 import position, quaternion

RATE_HZ = 20.0
MARKER_Z = 0.2
SCALE = (0.8, 0.4, 0.3)
COLOR = (0.0, 0.5, 1.0, 1.0)


def build_marker(t):
    """按轨迹参数 ``t`` 生成一帧 Marker（位置 + 切向航向）。"""
    x, y = position(t)
    qx, qy, qz, qw = quaternion(t)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = MARKER_Z
    marker.pose.orientation.x = qx
    marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz
    marker.pose.orientation.w = qw
    marker.scale.x, marker.scale.y, marker.scale.z = SCALE
    (marker.color.r, marker.color.g,
     marker.color.b, marker.color.a) = COLOR
    return marker


def draw_figure8(rate_hz=RATE_HZ):
    rospy.init_node('figure8_sim')
    pub = rospy.Publisher('/car_visual', Marker, queue_size=10)
    rate = rospy.Rate(rate_hz)
    # 用真实时间推进轨迹参数：循环快慢不再影响小车速度
    start = rospy.Time.now()

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start).to_sec()
        pub.publish(build_marker(t))
        rate.sleep()


if __name__ == '__main__':
    try:
        draw_figure8()
    except rospy.ROSInterruptException:
        pass
