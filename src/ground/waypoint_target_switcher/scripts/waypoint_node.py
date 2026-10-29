#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""waypoint_target_switcher 节点：发布路径点与当前目标点，并支持动态切换目标点。

发布的话题
    ``/waypoints``                geometry_msgs/PoseArray   全部路径点（latched）
    ``/waypoint/target``          geometry_msgs/PoseStamped 当前目标点（按频率发布）
    ``/waypoint/current_index``   std_msgs/Int32            当前目标点下标（变化时才发）
    ``/odom``                     nav_msgs/Odometry         仅在 mock/enabled 时由本节点模拟发布

订阅的话题
    ``/waypoint/set_index``       std_msgs/Int32   切换到第 N 个路径点（1 起算）
    ``/waypoint/next``            std_msgs/Empty   手动前进一个
    ``/waypoint/snap_nearest``    std_msgs/Empty   吸附到离当前位置最近的路径点
    ``/odom``                     nav_msgs/Odometry  真实/仿真位姿（mock/enabled 为 false 时使用）

参数见 ``config/waypoint_target_switcher.yaml``；每一项默认值都写在
``waypoint_core.PublisherParams`` 里，所以不传参数也能直接跑。

分层：本文件只做 ROS 收发与日志；解析、推进、到达判定、朝向换算都在
``waypoint_core``（不依赖 rospy），因此核心逻辑可以做离线回归测试。
"""

from __future__ import division, print_function

import io
import math
import os
import sys

# 兼容 devel / install 两种布局：catkin 的 relay 脚本有时会把 __file__ 指到
# lib/<pkg>/ 下，因此除 abspath 外再补一次 realpath（会解析软链）。
for _path in (os.path.abspath(__file__), os.path.realpath(__file__)):
    _dir = os.path.dirname(_path)
    if _dir not in sys.path:
        sys.path.insert(0, _dir)

import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32

import waypoint_core as core


class WaypointPublisherNode(object):
    """把「路线 + 目标点游标」包装成 ROS 节点。"""

    def __init__(self):
        rospy.init_node("waypoint_target_switcher", anonymous=False)
        self.params = core.PublisherParams.from_params(rospy.get_param)

        self.points = self._load_route()
        if not self.points:
            raise RuntimeError("没有可用的路径点，节点退出")
        for problem in core.validate_waypoints(self.points):
            rospy.logwarn("路线检查：%s", problem)

        self.cursor = core.TargetCursor(len(self.points), self.params.mode,
                                        self.params.start_index)
        # 模拟车辆从第一个路径点出发；真实模式等 /odom 覆盖
        first = self.points[0]
        self.pose = (first.x, first.y, first.yaw)
        self.armed = self.params.autostart
        self._published_index = None

        self.pub_waypoints = rospy.Publisher(self.params.waypoints_topic, PoseArray,
                                             queue_size=1, latch=True)
        self.pub_target = rospy.Publisher(self.params.target_topic, PoseStamped,
                                          queue_size=10)
        self.pub_index = rospy.Publisher(self.params.index_topic, Int32,
                                         queue_size=10, latch=True)
        self.pub_odom = None
        if self.params.use_mock_odom:
            self.pub_odom = rospy.Publisher(self.params.odom_topic, Odometry, queue_size=10)
        else:
            rospy.Subscriber(self.params.odom_topic, Odometry, self._on_odom, queue_size=10)

        rospy.Subscriber(self.params.set_index_topic, Int32, self._on_set_index, queue_size=10)
        rospy.Subscriber("/waypoint/next", Empty, self._on_next, queue_size=10)
        rospy.Subscriber("/waypoint/snap_nearest", Empty, self._on_snap_nearest, queue_size=10)

        self._log_startup()

    # ------------------------------------------------------------ 启动
    def _load_route(self):
        """读取路径点：给了 route/file 就读文件，否则用内置示例路线。"""
        path = self.params.route_file
        if not path:
            points = core.demo_route()
            rospy.loginfo("未指定 route/file，使用内置示例路线（%d 个点）", len(points))
            return points
        path = os.path.expanduser(path)
        try:
            with io.open(path, encoding="utf-8") as handle:
                text = handle.read()
        except (IOError, OSError) as exc:  # 文件不存在 / 无权限都走这里，报清楚路径
            raise RuntimeError("读取路径点文件失败：%s（%s）" % (path, exc))
        points = core.parse_route_text(text, source=path)
        rospy.loginfo("从 %s 载入 %d 个路径点", path, len(points))
        return points

    def _log_startup(self):
        rospy.loginfo("路径点发布器已启动：%d 个路径点，模式=%s，到达阈值=%.2fm，频率=%.1fHz",
                      len(self.points), self.params.mode,
                      self.params.target_tolerance, self.params.publish_hz)
        rospy.loginfo("话题：%s（全部点）｜%s（当前目标）｜%s（下标）｜切换用 %s",
                      self.params.waypoints_topic, self.params.target_topic,
                      self.params.index_topic, self.params.set_index_topic)
        if self.params.use_mock_odom:
            rospy.loginfo("内置模拟车辆已启用：%s 由本节点发布，速度 %.1f m/s",
                          self.params.odom_topic, self.params.mock_speed)
        if not self.armed:
            rospy.loginfo("autostart=false：等待 %s 或 /waypoint/next 后再开始",
                          self.params.set_index_topic)

    def start(self):
        """发布一次性话题并启动定时器。"""
        self._publish_waypoints()
        self._publish_target()
        period = 1.0 / max(self.params.publish_hz, 1e-6)
        rospy.Timer(rospy.Duration(period), self._tick)

    # ------------------------------------------------------------ 周期任务
    def _tick(self, _event):
        if not self.armed:
            self._publish_target()
            return
        self._step_mock_vehicle()
        target = self.points[self.cursor.current()]
        if core.reached(self.pose, target, self.params.target_tolerance):
            old = self.cursor.current()
            new = self.cursor.advance()
            if new != old:
                rospy.loginfo("到达第 %d 个路径点 → 目标切换为第 %d 个", old + 1, new + 1)
            elif self.cursor.is_finished() and not getattr(self, "_finished_logged", False):
                rospy.loginfo("已到达最后一个路径点，路线结束（可发 %s 重新指定）",
                              self.params.set_index_topic)
                self._finished_logged = True
        self._publish_target()

    def _step_mock_vehicle(self):
        """内置模拟车辆：朝当前目标点直线推进，并发布 /odom。"""
        if not self.params.use_mock_odom:
            return
        target = self.points[self.cursor.current()]
        step = self.params.mock_speed / max(self.params.publish_hz, 1e-6)
        self.pose = core.step_towards(self.pose, target, step)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.params.frame_id
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.position.z = 0.0
        qz, qw = core.yaw_to_quaternion(self.pose[2])
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)

    # ------------------------------------------------------------ 发布
    def _publish_waypoints(self):
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.params.frame_id
        for wp in self.points:
            pose = Pose()
            pose.position.x = wp.x
            pose.position.y = wp.y
            pose.position.z = 0.0
            qz, qw = core.yaw_to_quaternion(wp.yaw)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = qz
            pose.orientation.w = qw
            msg.poses.append(pose)
        self.pub_waypoints.publish(msg)

    def _publish_target(self):
        index = self.cursor.current()
        wp = self.points[index]
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.params.frame_id
        msg.pose.position.x = wp.x
        msg.pose.position.y = wp.y
        msg.pose.position.z = 0.0
        qz, qw = core.yaw_to_quaternion(wp.yaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_target.publish(msg)
        if index != self._published_index:
            self._published_index = index
            self.pub_index.publish(Int32(data=index))

    # ------------------------------------------------------------ 回调
    def _on_set_index(self, msg):
        if self.cursor.set_index(msg.data):
            self.armed = True
            self._finished_logged = False
            wp = self.points[self.cursor.current()]
            rospy.loginfo("收到切换指令：目标点改为第 %d 个（x=%.2f, y=%.2f）",
                          self.cursor.current() + 1, wp.x, wp.y)
            self._publish_target()
        else:
            rospy.logwarn("切换指令的下标 %r 越界（有效范围 1~%d），已忽略",
                          msg.data, len(self.points))

    def _on_next(self, _msg):
        old = self.cursor.current()
        new = self.cursor.force_next()
        self.armed = True
        self._finished_logged = False
        rospy.loginfo("手动前进：第 %d 个 → 第 %d 个", old + 1, new + 1)
        self._publish_target()

    def _on_snap_nearest(self, _msg):
        if self.cursor.jump_to_nearest(self.pose, self.points):
            self.armed = True
            self._finished_logged = False
            rospy.loginfo("按最近距离吸附：目标点改为第 %d 个", self.cursor.current() + 1)
            self._publish_target()
        else:
            rospy.logwarn("吸附失败：路线为空")

    def _on_odom(self, msg):
        pose = msg.pose.pose
        yaw = core.quaternion_to_yaw(pose.orientation.x, pose.orientation.y,
                                     pose.orientation.z, pose.orientation.w)
        self.pose = (pose.position.x, pose.position.y, yaw)


def main():
    node = WaypointPublisherNode()
    node.start()
    rospy.spin()


if __name__ == "__main__":
    main()
