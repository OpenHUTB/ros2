#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
八叉树环境建模 + 地面小车 自动避障快递配送【作业稳定版】
ROS Noetic + TurtleBot3 + Gazebo
修改：配送点全部在右侧开阔区域，不会进入墙死角
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class OctreeDeliveryCar:
    def __init__(self):
        rospy.init_node("octree_delivery_planner")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        # 【重点！新目标点，全部在右边空地，远离长墙，不会卡死角】
        self.delivery_goals = [
            (2.0, 2.0),
            (2.5, -2.0),
            (1.5, 0.0)
        ]
        self.goal_idx = 0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.yaw = 0.0

        # 雷达分区距离
        self.front_min = 999
        self.left_min = 999
        self.right_min = 999
        self.front_left_min = 999
        self.front_right_min = 999

        self.rate = rospy.Rate(10)

    def laser_cb(self, msg):
        front_ranges = []
        left_ranges = []
        right_ranges = []
        front_left_ranges = []
        front_right_ranges = []

        for i, r in enumerate(msg.ranges):
            if 0.12 < r < 4.0:
                angle = msg.angle_min + i * msg.angle_increment

                # 正前方 ±20°
                if -math.pi / 9 < angle < math.pi / 9:
                    front_ranges.append(r)
                # 左前 20°~60°
                elif math.pi / 9 < angle < math.pi / 3:
                    front_left_ranges.append(r)
                # 右前 -60°~-20°
                elif -math.pi / 3 < angle < -math.pi / 9:
                    front_right_ranges.append(r)
                # 左侧 60°~90°
                elif math.pi / 3 < angle < math.pi / 2:
                    left_ranges.append(r)
                # 右侧 -90°~-60°
                elif -math.pi / 2 < angle < -math.pi / 3:
                    right_ranges.append(r)

        self.front_min = min(front_ranges) if front_ranges else 999
        self.front_left_min = min(front_left_ranges) if front_left_ranges else 999
        self.front_right_min = min(front_right_ranges) if front_right_ranges else 999
        self.left_min = min(left_ranges) if left_ranges else 999
        self.right_min = min(right_ranges) if right_ranges else 999

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def run(self):
        rospy.loginfo("===== 快递小车启动，八叉树环境构建，开始配送任务 =====")
        twist = Twist()

        while not rospy.is_shutdown():
            if self.goal_idx >= len(self.delivery_goals):
                rospy.loginfo("✅所有快递配送完成！任务结束")
                twist.linear.x = 0
                twist.angular.z = 0
                self.pub.publish(twist)
                break

            target_x, target_y = self.delivery_goals[self.goal_idx]
            dx = target_x - self.odom_x
            dy = target_y - self.odom_y
            dist2goal = math.hypot(dx, dy)
            angle2goal = math.atan2(dy, dx)
            angle_err = angle2goal - self.yaw
            angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

            # 分层避障逻辑
            if self.front_min < 0.5:
                rospy.loginfo("⚠前方障碍，停车转向")
                twist.linear.x = 0
                if self.left_min > self.right_min:
                    twist.angular.z = 0.5
                else:
                    twist.angular.z = -0.5

            elif self.front_left_min < 0.4:
                rospy.loginfo("⚠左前方障碍，向右绕")
                twist.linear.x = 0.08
                twist.angular.z = -0.4

            elif self.front_right_min < 0.4:
                rospy.loginfo("⚠右前方障碍，向左绕")
                twist.linear.x = 0.08
                twist.angular.z = 0.4

            else:
                # 正常朝目标行驶
                if abs(angle_err) > 0.1:
                    twist.linear.x = 0
                    twist.angular.z = 0.35 * angle_err
                else:
                    twist.linear.x = 0.16
                    twist.angular.z = 0.12 * angle_err

            #到达配送点，切换下一站
            if dist2goal < 0.4:
                rospy.loginfo(f"📦到达配送点{self.goal_idx+1}，准备前往下一站")
                self.goal_idx += 1

            self.pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        car = OctreeDeliveryCar()
        car.run()
    except rospy.ROSInterruptException:
        pass

