#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""twist_to_control 的 ROS 节点。

职责很薄：订阅 /cmd_vel，按固定频率推进控制状态，把结果发布出去。
所有换算、限幅、积分都在 twist_core.py 里（不依赖 rospy，可离线测试）。
"""
from __future__ import division
from __future__ import print_function

import math
import os
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import twist_core as core  # noqa: E402


class TwistToControlNode(object):
    """速度指令转车辆控制量。"""

    def __init__(self):
        self.params = core.ControlParams.from_params(rospy.get_param)
        self.state = core.ControlState()
        self.pose = core.Pose()
        self.command = core.TwistCommand()

        self.rate_hz = self.params.publish_hz
        if self.rate_hz <= 0.0:
            self.rate_hz = core.DEFAULT_PUBLISH_HZ
        self.dt = 1.0 / self.rate_hz
        self.last_command_stamp = None

        self.pub_control = rospy.Publisher(
            self.params.control_topic, Float64MultiArray, queue_size=10)
        self.pub_info = rospy.Publisher(
            self.params.info_topic, Float64MultiArray, queue_size=10)

        self.pub_wheel = None
        if self.params.model == "diff_drive":
            self.pub_wheel = rospy.Publisher(
                self.params.wheel_topic, Float64MultiArray, queue_size=10)

        self.pub_odom = None
        if self.params.use_mock_odom:
            self.pub_odom = rospy.Publisher(
                self.params.odom_topic, Odometry, queue_size=20)

        self.sub_cmd = rospy.Subscriber(
            self.params.cmd_vel_topic, Twist, self.on_cmd_vel, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.on_timer)
        rospy.on_shutdown(self.on_ros_shutdown)

        rospy.loginfo(
            "扭转控制节点已启动：模型=%s 轴距=%.2fm 轮距=%.2fm "
            "最大转角=%.2frad 最大曲率=%.2f/m 频率=%.1fHz",
            self.params.model, self.params.wheelbase, self.params.track_width,
            self.params.max_steering, self.params.max_curvature, self.rate_hz)
        rospy.loginfo(
            "订阅 %s，发布 %s / %s%s%s",
            self.params.cmd_vel_topic, self.params.control_topic,
            self.params.info_topic,
            (" / " + self.params.wheel_topic) if self.pub_wheel else "",
            (" / " + self.params.odom_topic + "（内置模拟）")
            if self.pub_odom else "")

    # ------------------------------------------------------------ 回调
    def on_cmd_vel(self, msg):
        self.command = core.TwistCommand(msg.linear.x, msg.angular.z)
        self.last_command_stamp = rospy.get_time()

    def on_timer(self, event):
        # Ctrl+C 之后 rospy 会先注销话题，定时器可能还被触发最后一次；
        # 那时 publish() 会抛 ROSException("publish() to a closed topic")。
        # 这是退出过程中的正常现象，不该把调用栈打给用户看。
        if rospy.is_shutdown():
            return
        try:
            self.apply_watchdog()
            self.state = core.step_control(self.state, self.command, self.dt,
                                           self.params)
            self.publish_control()
            self.publish_info()
            if self.pub_wheel is not None:
                self.publish_wheel()
            if self.pub_odom is not None:
                core.mock_step(self.pose, self.state, self.dt, self.params)
                self.publish_odom()
        except rospy.ROSException:
            return

    def apply_watchdog(self):
        """/cmd_vel 断流时按零速度处理，避免车辆失控。"""
        if self.last_command_stamp is None:
            return
        silent = rospy.get_time() - self.last_command_stamp
        if silent > self.params.timeout and not self.command.is_zero():
            rospy.logwarn_throttle(
                2.0, "%.1f 秒未收到 %s，按零速度处理",
                silent, self.params.cmd_vel_topic)
            self.command = core.TwistCommand()

    # ------------------------------------------------------------ 发布
    def publish_control(self):
        message = Float64MultiArray()
        message.data = [float(value) for value in
                        core.control_fields(self.state)]
        self.pub_control.publish(message)

    def publish_info(self):
        message = Float64MultiArray()
        message.data = [float(value) for value in core.info_fields(self.state)]
        self.pub_info.publish(message)

    def publish_wheel(self):
        left, right = core.diff_drive_wheel_speeds(
            self.state.speed, self.state.curvature, self.params.track_width)
        message = Float64MultiArray()
        message.data = [float(left), float(right)]
        self.pub_wheel.publish(message)

    def publish_odom(self):
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self.pose.x
        message.pose.pose.position.y = self.pose.y
        quat = core.yaw_to_quaternion(self.pose.yaw)
        message.pose.pose.orientation.x = quat[0]
        message.pose.pose.orientation.y = quat[1]
        message.pose.pose.orientation.z = quat[2]
        message.pose.pose.orientation.w = quat[3]
        message.twist.twist.linear.x = self.state.speed
        message.twist.twist.angular.z = self.actual_yaw_rate()
        self.pub_odom.publish(message)

    def actual_yaw_rate(self):
        """实际角速度：用实发转角算，转角被速率限制时会小于请求值。"""
        if self.params.model == "diff_drive":
            return self.state.speed * self.state.curvature
        if abs(self.params.wheelbase) < 1e-9:
            return 0.0
        return (self.state.speed * math.tan(self.state.steering)
                / self.params.wheelbase)

    def publish_zero(self):
        """退出前发一帧零控制量，保证车辆停下。"""
        self.command = core.TwistCommand()
        self.state.reset()
        try:
            self.publish_control()
            self.publish_info()
            if self.pub_wheel is not None:
                self.publish_wheel()
        except rospy.ROSException:
            return          # 话题已关闭，说明进程已在退出，无需再发
        rospy.loginfo("已发布零控制量")

    def on_ros_shutdown(self):
        try:
            self.publish_zero()
        except Exception as error:      # noqa: BLE001
            print("退出清理时出错：%s" % (error,))


def main():
    rospy.init_node("twist_to_control", anonymous=False)
    node = TwistToControlNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    node.publish_zero()


if __name__ == "__main__":
    main()
