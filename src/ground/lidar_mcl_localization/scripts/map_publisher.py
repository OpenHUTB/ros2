#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 PGM 地图发布成 nav_msgs/OccupancyGrid（等价于 map_server 的最小实现）。

为什么自己发而不是用 map_server：
    虚拟机里不一定装了 `ros-noetic-map-server`（本模块的其他部分也不需要它）。
    这个节点只做一件事：读 PGM + YAML → 填 OccupancyGrid → **latched** 发布一次。
    有了它，rviz 的 Map 显示才能看到地图，粒子云和激光也才有参照。

发布：`/map`（latched，transient_local）
参数：`~map_file`（默认包内 maps/mcl_room.pgm）
"""

import os
import sys

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

try:
    from lidar_mcl_localization.map_model import OccupancyMap, to_occupancy_values
except ImportError:                                          # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from lidar_mcl_localization.map_model import OccupancyMap, to_occupancy_values


def main():
    rospy.init_node("mcl_map_publisher")
    map_file = rospy.get_param("~map_file", "")
    if not map_file:
        map_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "maps", "mcl_room.pgm")

    occ = OccupancyMap.from_files(map_file)
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.resolution = float(occ.resolution)
    msg.info.width = int(occ.width)
    msg.info.height = int(occ.height)
    msg.info.origin.position.x = float(occ.origin[0])
    msg.info.origin.position.y = float(occ.origin[1])
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = to_occupancy_values(occ.grid).tolist()

    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rospy.loginfo("已发布 /map：%dx%d，%.3f m/px，原点 (%.2f, %.2f)",
                  occ.width, occ.height, occ.resolution, occ.origin[0], occ.origin[1])
    rospy.spin()


if __name__ == "__main__":
    main()
