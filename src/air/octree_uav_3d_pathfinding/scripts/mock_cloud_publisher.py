#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线点云模拟发布器（**不需要模拟器**）

用途：把"点云 → 八叉树 → RViz"这半条链路单独拎出来验证。
      排障时用它就能分清问题出在：
        · airsim_bridge 取数/坐标转换（用本脚本时不参与，直接排除）
        · pointcloud_to_octomap 建图（本脚本照常考验它）
        · RViz 配置与 Fixed Frame（本脚本照常考验它）

发布一个 20 x 20 x 6 m 的封闭房间（地面 + 四壁）+ 房间内一个 2 m 立方体障碍，
以 /cloud_in 话题发出，并广播 world -> lidar_link 的静态 TF（原点在房间中心）。

用法：
    rosrun octree_uav_3d_pathfinding mock_cloud_publisher.py
    # 或者直接跑：
    python3 scripts/mock_cloud_publisher.py

然后另开终端：
    rosrun octree_uav_3d_pathfinding pointcloud_to_octomap
    rosrun rviz rviz -d $(rospack find octree_uav_3d_pathfinding)/rviz/default.rviz

**期望结果**：RViz 里出现一个彩色的空心立方体房间（地面蓝、墙顶红），
中间有一个悬空的彩色方块。看不到就说明问题在建图节点或 RViz，与模拟器无关。
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


ROOM_L = 20.0     # 房间长（x）
ROOM_W = 20.0     # 房间宽（y）
ROOM_H = 6.0      # 房间高（z）
STEP = 0.4        # 采样间距（m）
BOX_SIDE = 2.0    # 障碍立方体边长（m）
BOX_OFFSET = 5.0  # 立方体离房间中心的水平距离（m）


def build_scene():
    """生成房间表面 + 中央障碍的采样点（世界系坐标）。"""
    hx, hy = ROOM_L / 2.0, ROOM_W / 2.0
    z0 = -ROOM_H / 2.0
    z1 = z0 + ROOM_H
    pts = []

    def grid(u0, u1, v0, v1, fn):
        u = u0
        while u <= u1 + 1e-6:
            v = v0
            while v <= v1 + 1e-6:
                pts.append(fn(u, v))
                v += STEP
            u += STEP

    # 地面
    grid(-hx, hx, -hy, hy, lambda x, y: (x, y, z0))
    # 四面墙
    grid(-hx, hx, z0, z1, lambda x, z: (x, -hy, z))
    grid(-hx, hx, z0, z1, lambda x, z: (x, hy, z))
    grid(-hy, hy, z0, z1, lambda y, z: (-hx, y, z))
    grid(-hy, hy, z0, z1, lambda y, z: (hx, y, z))

    # 悬空障碍立方体（只取四个侧面，模拟被激光扫到的表面）
    b = BOX_SIDE / 2.0
    cx, cy, cz = BOX_OFFSET, 0.0, 0.0
    grid(cx - b, cx + b, cz - b, cz + b, lambda x, z: (x, cy - b, z))
    grid(cx - b, cx + b, cz - b, cz + b, lambda x, z: (x, cy + b, z))
    grid(cy - b, cy + b, cz - b, cz + b, lambda y, z: (cx - b, y, z))
    grid(cy - b, cy + b, cz - b, cz + b, lambda y, z: (cx + b, y, z))

    return pts


class MockCloudPublisher(object):
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'lidar_link')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.rate = rospy.get_param('~rate', 10.0)

        self.points = build_scene()
        rospy.loginfo('离线点云模拟器：生成 %d 个点（房间 %.0fx%.0fx%.0f m + %.0f m 立方体）',
                      len(self.points), ROOM_L, ROOM_W, ROOM_H, BOX_SIDE)

        self.pub = rospy.Publisher('/cloud_in', PointCloud2, queue_size=1)

        # world -> lidar_link 静态 TF，原点放在房间中心（射线从这里射出去）
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.frame_id
        t.transform.rotation.w = 1.0
        self.static = tf2_ros.StaticTransformBroadcaster()
        self.static.sendTransform(t)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.publish_cb)
        rospy.loginfo('已开始发布 /cloud_in（frame_id=%s，%.0f Hz）', self.frame_id, self.rate)
        rospy.loginfo('下一步：另开终端跑 pointcloud_to_octomap，再看 RViz')

    def publish_cb(self, _event):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, self.points))


def main():
    rospy.init_node('mock_cloud_publisher')
    MockCloudPublisher()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
