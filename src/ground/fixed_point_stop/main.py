#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class FixedPointStop:
    """定点停车：小车从起点驶向指定目标点，到达后自动停车。
    订阅 /odom 获取位置与朝向、/scan 做前方避障，发布 /cmd_vel。"""

    def __init__(self):
        rospy.init_node("fixed_point_stop_node")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # 目标点（米，可通过 launch 参数配置）
        self.target_x = rospy.get_param("~target_x", 1.5)
        self.target_y = rospy.get_param("~target_y", 1.5)
        self.goal_tol = rospy.get_param("~goal_tol", 0.1)  # 停车容差
        self.safe_dist = 0.4                               # 前方避障距离
        self.angle_tol = 0.15                              # 转向容差
        self.cruise_v = 0.2                                # 直行速度
        self.turn_gain = 0.3                               # 转向增益

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.front_dist = 999.0
        self.rate = rospy.Rate(10)

    # ---------------- 回调 ----------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def laser_callback(self, msg):
        front = list(msg.ranges[0:20]) + list(msg.ranges[340:360])
        valid = [d for d in front if 0.1 < d < 10.0]
        self.front_dist = min(valid) if valid else 999.0

    # ---------------- 工具 ----------------
    def publish(self, vx, wz):
        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        self.vel_pub.publish(m)

    # ---------------- 主循环 ----------------
    def run(self):
        rospy.loginfo("定点停车启动，目标点 (%.2f, %.2f)，容差 %.2f m",
                      self.target_x, self.target_y, self.goal_tol)
        while not rospy.is_shutdown():
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            dist = math.hypot(dx, dy)
            angle_to_target = math.atan2(dy, dx)
            angle_err = math.atan2(math.sin(angle_to_target - self.yaw),
                                   math.cos(angle_to_target - self.yaw))

            if dist < self.goal_tol:
                rospy.loginfo("已到达目标点，停车！实际位置 (%.2f, %.2f)，误差 %.3f m",
                              self.x, self.y, dist)
                self.publish(0.0, 0.0)
                break

            # 前方避障
            if self.front_dist < self.safe_dist:
                rospy.loginfo("前方障碍，停车右转")
                self.publish(0.0, 0.4)
            elif abs(angle_err) > self.angle_tol:
                # 先转向对准目标方向
                self.publish(0.0, self.turn_gain * angle_err)
            else:
                # 直行逼近目标点
                self.publish(self.cruise_v, 0.1 * angle_err)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = FixedPointStop()
        node.run()
    except rospy.ROSInterruptException:
        pass
