#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 小海龟画正方形节点（闭环反馈控制版）：turtle_square_node
# 思路：直走一条边 -> 停 0.3 秒 -> 左转 90 度 -> 停 0.3 秒，循环 4 次就是正方形
#
# 与开环版本的区别：订阅 /turtle1/pose 实时获取海龟位姿 (x, y, theta)，
# 用反馈信息判断"走够了没有 / 转够了没有"，距离和角度一到位就立刻停下，
# 不再依赖固定时长，从根上消除距离与角度的累计误差。

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class SquareDrawer:
    """闭环反馈控制：小海龟精准绘制边长 2 m 的正方形"""

    def __init__(self):
        # 初始化节点
        rospy.init_node('turtle_square_node')
        # 发布者：把速度指令发到 /turtle1/cmd_vel 话题
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # 订阅者：实时接收 /turtle1/pose 话题上的位姿
        self.pose = None
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        # 50 Hz 闭环控制频率：高频采样位姿并重发指令，防止 0.5s 看门狗刹车
        self.rate = rospy.Rate(50)

        # 运动参数
        self.side_length = 2.0            # 正方形边长 2 m
        self.linear_speed = 1.0           # 线速度 1.0 m/s
        self.angular_speed = 1.0          # 角速度 1.0 rad/s
        self.turn_angle = math.pi / 2     # 左转 90 度

    def pose_callback(self, msg):
        """位姿回调：把最新位姿存下来，控制循环里随时读取"""
        self.pose = msg

    def publish_velocity(self, linear=0.0, angular=0.0):
        """组装并发布一条速度指令"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def wait_for_pose(self):
        """等第一帧位姿消息到来，保证反馈数据可用"""
        while self.pose is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(1.0, "等待 /turtle1/pose 位姿数据...")
            self.rate.sleep()

    def pause(self, duration=0.3):
        """持续发布空 Twist 停顿 0.3 秒，把上一次动作的误差彻底清零"""
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.publish_velocity()
            self.rate.sleep()

    def move_straight(self, distance):
        """闭环直行：从当前位姿出发，欧氏距离走满 distance 就精准停下"""
        start_x = self.pose.x
        start_y = self.pose.y
        moved = 0.0
        while not rospy.is_shutdown():
            # 实时计算已移动的欧氏距离
            moved = math.hypot(self.pose.x - start_x, self.pose.y - start_y)
            if moved >= distance:
                break
            # 没走够就继续前进，50 Hz 持续发布防看门狗刹车
            self.publish_velocity(linear=self.linear_speed)
            self.rate.sleep()
        self.publish_velocity()   # 到位立刻发零速度停车
        rospy.loginfo("直行完成，实际走了 %.3f 米（目标 %.2f 米）", moved, distance)

    def turn(self, angle):
        """闭环转向：实时累计朝向角差（±π 跨界归一化），转满 angle 就精准停下"""
        total_turned = 0.0
        prev_theta = self.pose.theta
        while not rospy.is_shutdown():
            # 本周期转过的角度差，先归一化到 [-pi, pi] 再累加，防 ±π 跨界跳变
            delta = self.normalize_angle(self.pose.theta - prev_theta)
            total_turned += delta
            prev_theta = self.pose.theta
            if total_turned >= angle:
                break
            # 没转够就继续左转
            self.publish_velocity(angular=self.angular_speed)
            self.rate.sleep()
        self.publish_velocity()   # 到位立刻发零速度停车
        rospy.loginfo("转弯完成，实际转了 %.3f 弧度（目标 %.3f 弧度）",
                      total_turned, angle)

    @staticmethod
    def normalize_angle(a):
        """把角度差归一化到 [-pi, pi]，处理 ±π 附近的跨界跳变"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def run(self):
        # 等位姿数据就绪，然后停一下让海龟初始状态稳定
        self.wait_for_pose()
        self.pause()

        rospy.loginfo("开始画正方形，边长 %.1f 米（闭环反馈控制）", self.side_length)
        for i in range(4):
            # 闭环直行一条边
            rospy.loginfo("正在画第 %d 条边", i + 1)
            self.move_straight(self.side_length)
            self.pause()

            # 闭环左转 90 度
            rospy.loginfo("正在转第 %d 个角", i + 1)
            self.turn(self.turn_angle)
            self.pause()

        rospy.loginfo("正方形画完啦！")


if __name__ == '__main__':
    node = SquareDrawer()
    try:
        node.run()
    except rospy.ROSInterruptException:
        # 按 Ctrl+C 或者节点被关掉的时候走到这里
        rospy.loginfo("节点被中断，让海龟停下来")
    finally:
        # 不管正常画完还是中间出错，最后都再发一次 0 速度，保险一点
        if not rospy.is_shutdown():
            node.publish_velocity()
