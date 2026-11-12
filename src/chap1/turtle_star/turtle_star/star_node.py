#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小海龟画五角星节点（ROS 2 版，绝对位姿反馈）。

文档 `docs/chapter/chap1/turtle_star.md` 原版是 ROS 1 + rospy + 定时开环。
本模块保留「5 次直行 L + 左转 144°」的算法，但判据改成**绝对位姿反馈**：

* 直行：从本边起点算直线距离，达到 ``side_length`` 就停；
* 转向：目标航向 = 本边起始航向 + 144°，误差进入 0.05 rad 或穿过目标就停。

这样即使 turtlesim 的位姿流被调度拖慢、丢几帧，也不会像定时开环那样越走越偏。
每条边开始前调用 ``/turtle1/set_pen`` 换颜色。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from turtle_star.star import TURN_ANGLE, palette, wrap_to_pi

EDGES = 5


class StarDrawer(Node):
    FORWARD, TURN, DONE = range(3)

    def __init__(self):
        super().__init__('turtle_star_node')
        self.turtle = self.declare_parameter('turtle', 'turtle1').value
        self.side_length = float(self.declare_parameter('side_length', 1.0).value)
        self.linear_speed = float(
            self.declare_parameter('linear_speed', 1.0).value)
        self.angular_speed = float(
            self.declare_parameter('angular_speed', 1.0).value)
        self.pen_width = int(self.declare_parameter('pen_width', 3).value)
        self.tick = float(self.declare_parameter('tick', 0.02).value)
        self.heading_tolerance = float(
            self.declare_parameter('heading_tolerance', 0.01).value)

        self.colors = palette(EDGES)
        self.state = self.FORWARD
        self.edge = 0
        self.pose = None
        self.edge_start = None
        self.edge_heading = 0.0
        self.turn_target = 0.0
        self._prev_err = None

        self.pub = self.create_publisher(Twist, f'/{self.turtle}/cmd_vel', 10)
        self.create_subscription(Pose, f'/{self.turtle}/pose', self.on_pose, 200)
        self.pen = self.create_client(SetPen, f'/{self.turtle}/set_pen')
        self.timer = self.create_timer(self.tick, self.on_timer)
        self._pen_sent = False
        self.get_logger().info(
            'star: %d edges, side=%.2f m, v=%.2f, w=%.2f'
            % (EDGES, self.side_length, self.linear_speed, self.angular_speed))

    # ------------------------------------------------------------------
    def on_pose(self, msg):
        self.pose = msg
        if self.state == self.FORWARD and self.edge_start is None:
            self._begin_edge()

    def _begin_edge(self):
        self.edge_start = (self.pose.x, self.pose.y)
        self.edge_heading = self.pose.theta
        self._pen_sent = False

    def _publish(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def _set_pen(self, color):
        if not self.pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('set_pen service not available')
            return
        req = SetPen.Request()
        req.r, req.g, req.b = color
        req.width = self.pen_width
        req.off = 0
        self.pen.call_async(req)

    def on_timer(self):
        if self.state == self.DONE or self.pose is None:
            return
        if self.edge_start is None:
            self._begin_edge()
        if not self._pen_sent:
            self._set_pen(self.colors[self.edge])
            self._pen_sent = True
            self.get_logger().info(
                'edge %d color=%s' % (self.edge + 1, self.colors[self.edge]))

        if self.state == self.FORWARD:
            traveled = math.hypot(self.pose.x - self.edge_start[0],
                                  self.pose.y - self.edge_start[1])
            if traveled >= self.side_length:
                self._publish()
                self.turn_target = wrap_to_pi(self.edge_heading + TURN_ANGLE)
                self._prev_err = None
                self.state = self.TURN
            else:
                self._publish(linear=self.linear_speed)
        elif self.state == self.TURN:
            err = wrap_to_pi(self.turn_target - self.pose.theta)
            reached = abs(err) <= self.heading_tolerance
            crossed = (self._prev_err is not None
                       and self._prev_err > 0.0 >= err)
            self._prev_err = err
            if reached or crossed:
                self._publish()
                self.edge += 1
                if self.edge >= EDGES:
                    self.state = self.DONE
                    self.timer.cancel()
                    self.get_logger().info(
                        'finished: %d edges, final theta=%.2f rad'
                        % (EDGES, self.pose.theta))
                else:
                    self.edge_start = None
                    self.state = self.FORWARD
            else:
                self._publish(angular=self.angular_speed)

    def stop(self):
        """发一帧零速度，避免节点退出后 turtlesim 继续执行最后的速度指令。"""
        if rclpy.ok():
            try:
                self._publish()
            except Exception:
                pass


def main(argv=None):
    rclpy.init(args=argv)
    node = StarDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
