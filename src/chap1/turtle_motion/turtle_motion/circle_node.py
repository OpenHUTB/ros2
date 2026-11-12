#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小海龟画圆节点（ROS 2 版）。

文档 `docs/chapter/chap1/turtle_circle.md` 原版是 ROS 1 + rospy；本模块按同样的
话题与算法移植到 ROS 2：

* 发布 ``/<turtle>/cmd_vel``（geometry_msgs/Twist）；
* 订阅 ``/<turtle>/pose``（turtlesim/Pose）；
* 累计转过的角度，达到 2π 后发零速度并退出。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose

from turtle_motion.circle import FULL_TURN, TurnAccumulator, command_for_circle


class CircleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_circle_node')
        self.turtle = self.declare_parameter('turtle', 'turtle1').value
        self.radius = float(self.declare_parameter('radius', 2.0).value)
        self.linear_speed = float(
            self.declare_parameter('linear_speed', 2.0).value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 20.0).value)

        self.v, self.omega = command_for_circle(self.radius, self.linear_speed)
        self.accumulator = TurnAccumulator(FULL_TURN)
        self.finished = False
        self.last_pose = None

        self.pub = self.create_publisher(Twist, f'/{self.turtle}/cmd_vel', 10)
        self.sub = self.create_subscription(
            Pose, f'/{self.turtle}/pose', self.on_pose, 200)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish)
        self.get_logger().info(
            'circle: radius=%.2f m, v=%.2f m/s, omega=%.3f rad/s'
            % (self.radius, self.v, self.omega))

    def on_pose(self, msg):
        self.last_pose = msg
        self.accumulator.update(msg.theta)
        # 完成判定放在位姿回调里：定时器被高频位姿挤占时也能及时停车
        if not self.finished and self.accumulator.done:
            self._finish()

    def _finish(self):
        self.finished = True
        self.pub.publish(Twist())
        self.get_logger().info('finished: turned %.3f rad' % self.accumulator.total)
        self.timer.cancel()

    def publish(self):
        twist = Twist()
        if not self.finished:
            twist.linear.x = self.v
            twist.angular.z = self.omega
        self.pub.publish(twist)



    def stop(self):
        """发一帧零速度，避免节点退出后 turtlesim 继续执行最后的速度指令。"""
        if rclpy.ok():
            try:
                self.pub.publish(Twist())
            except Exception:
                pass

def main(argv=None):
    rclpy.init(args=argv)
    node = CircleDrawer()
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
