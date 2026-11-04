#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

current_pose = None

def pose_callback(msg):
    global current_pose
    current_pose = msg

class TurtleDrawer:
    def __init__(self):
        rospy.init_node('draw_heart', anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
        self.rate = rospy.Rate(10)
        time.sleep(2)

    def move_to(self, target_x, target_y):
        """让海龟平滑移动到目标点"""
        global current_pose
        while not rospy.is_shutdown():
            if current_pose is None:
                continue
            
            dx = target_x - current_pose.x
            dy = target_y - current_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            # 距离小于 0.15 就算到达
            if distance < 0.15:
                self.stop()
                break

            target_theta = math.atan2(dy, dx)
            angle_diff = target_theta - current_pose.theta
            
            # 角度归一化到 [-pi, pi]
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            move_cmd = Twist()
            # 降低线速度，防止冲过头
            move_cmd.linear.x = min(1.0, 0.8 * distance)
            # 提高转向灵敏度
            move_cmd.angular.z = 4.0 * angle_diff 

            self.pub.publish(move_cmd)
            self.rate.sleep()

    def stop(self):
        """停止海龟"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.pub.publish(move_cmd)
        time.sleep(0.5)

    def draw_heart(self):
        rospy.loginfo("开始绘制爱心...")
        
        # 爱心参数方程，生成一系列离散的坐标点
        points = []
        # t 从 0 到 2*pi 循环，生成 60 个点
        for i in range(61):
            t = i * (2 * math.pi / 60)
            
            # 爱心参数方程
            x = 16 * math.sin(t)**3
            y = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
            
            # 缩放和位移，使其适应小海龟 11x11 的边界
            # 参数方程原本的坐标值大概在 -16 到 16 之间
            # 我们将其缩放0.25倍，然后平移中心到 (5.5, 5.5)
            scaled_x = x * 0.25 + 5.5
            scaled_y = y * 0.25 + 5.5
            
            # 边界安全检查（防止因为误差冲出边界）
            scaled_x = max(0.5, min(10.5, scaled_x))
            scaled_y = max(0.5, min(10.5, scaled_y))
            
            points.append((scaled_x, scaled_y))

        # 让海龟依次走到这些点
        for point in points:
            rospy.loginfo(f"移动到: {point}")
            self.move_to(point[0], point[1])
            time.sleep(0.05)

        rospy.loginfo("爱心绘制完成！")

if __name__ == '__main__':
    try:
        drawer = TurtleDrawer()
        drawer.draw_heart()
    except rospy.ROSInterruptException:
        pass