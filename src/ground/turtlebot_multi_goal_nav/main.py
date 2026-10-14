#!/usr/bin/env python3
"""turtlebot_multi_goal_nav：多目标点顺序导航。

地面移动机器人（TurtleBot3 等）依次访问多个目标点：
  订阅 /odom（nav_msgs/Odometry）获取位姿，发布 /cmd_vel（geometry_msgs/Twist）
  驱动底盘。入口为 main.py，可通过 roslaunch / rosrun 启动。

作业原始代码：多目标点顺序导航，到达目标点阈值内即切换到下一个目标点。
本模块不含激光避障节点。
"""

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class MultiGoalNav:
    """多目标点顺序导航控制器。"""

    def __init__(self):
        rospy.init_node("multi_goal_nav_node")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

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
        self.goal_tol = 0.2

        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

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
