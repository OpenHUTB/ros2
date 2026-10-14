#!/usr/bin/env python3
"""turtlebot_multi_goal_nav：多目标点顺序导航。

地面移动机器人（TurtleBot3 等）依次访问多个目标点：
  订阅 /odom（nav_msgs/Odometry）获取位姿、/scan（sensor_msgs/LaserScan）检测正前方障碍，
  发布 /cmd_vel（geometry_msgs/Twist）驱动底盘。
入口为 main.py，可通过 roslaunch / rosrun 启动。

作业原始代码：多目标点顺序导航，到达目标点阈值内即切换到下一个目标点；
正前方障碍过近时停车并原地右转避障（内嵌于同一导航节点，无独立激光避障节点）。
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class MultiGoalNav:
    """多目标点顺序导航控制器。"""

    def __init__(self):
        rospy.init_node("multi_goal_nav_node")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # ========== 室外场景目标点 ==========
        self.goals = [
            (1.5, 0.0),
            (1.5, 3.0),
            (-2.5, 3.0),
            (-2.5, -1.5),
            (0.0, 0.0),
        ]
        self.goal_index = 0

        # 小车参数
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.min_dist = 999.0
        self.safe_dist = 0.4
        self.goal_tol = 0.2

        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def laser_callback(self, msg):
        front_scan = msg.ranges[340:360] + msg.ranges[0:20]
        valid = [d for d in front_scan if 0.1 < d < 10.0]
        if valid:
            self.min_dist = min(valid)
        else:
            self.min_dist = 999

    def run(self):
        vel_msg = Twist()
        rospy.loginfo("开始多目标导航！")
        while not rospy.is_shutdown():
            if self.goal_index >= len(self.goals):
                rospy.loginfo("全部目标点跑完，停车！")
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.vel_pub.publish(vel_msg)
                break

            target_x, target_y = self.goals[self.goal_index]
            dx = target_x - self.x
            dy = target_y - self.y
            dist_to_goal = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_err = angle_to_goal - self.yaw
            angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

            # 内嵌简单前方避障
            if self.min_dist < self.safe_dist:
                rospy.loginfo("前方障碍物，停止前进，原地右转")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.4
            else:
                if dist_to_goal < self.goal_tol:
                    rospy.loginfo("到达目标 %d", self.goal_index + 1)
                    self.goal_index += 1
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                else:
                    if abs(angle_err) > 0.15:
                        # 对准目标方向（原地转向）
                        vel_msg.linear.x = 0.0
                        vel_msg.angular.z = 0.3 * angle_err
                    else:
                        # 直行驶向目标点
                        vel_msg.linear.x = 0.2
                        vel_msg.angular.z = 0.1 * angle_err

            self.vel_pub.publish(vel_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        nav = MultiGoalNav()
        nav.run()
    except rospy.ROSInterruptException:
        pass
