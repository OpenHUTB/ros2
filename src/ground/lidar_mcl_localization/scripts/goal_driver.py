#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 MCL 估出来的位姿闭环行驶到目标点。

这一节对应作业要求里的"运动控制"：定位不是目的，**用定位结果去控制**才是。
控制器故意写得很朴素——到点就转、对齐再走（rotate-then-drive），
因为重点在"位姿来自 /mcl_pose 而不是真值"。

用法：
    rosrun lidar_mcl_localization goal_driver.py _goal_x:=2.0 _goal_y:=2.5
    # 走完一整圈（四个角），验证定位误差不会让机器人跑偏：
    rosrun lidar_mcl_localization goal_driver.py _waypoints:="[(-3.5,0),(1.0,2.5),(2.0,-2.0),(-3.5,0)]"
"""

import math
import sys

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32MultiArray

try:
    from lidar_mcl_localization.geometry import wrap_angle
except ImportError:                                          # pragma: no cover
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from lidar_mcl_localization.geometry import wrap_angle


class GoalDriver(object):

    def __init__(self):
        self.goal_x = float(rospy.get_param("~goal_x", 1.0))
        self.goal_y = float(rospy.get_param("~goal_y", 2.0))
        self.waypoints = rospy.get_param("~waypoints", None)
        self.linear = float(rospy.get_param("~linear_speed", 0.35))
        self.angular = float(rospy.get_param("~angular_speed", 0.80))
        self.pos_tol = float(rospy.get_param("~pos_tolerance", 0.15))
        self.yaw_tol = float(rospy.get_param("~yaw_tolerance", 0.10))
        self.max_particles_std = float(rospy.get_param("~max_pose_std", 0.60))

        self.pose = None
        self.pose_std = 0.0
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/mcl_pose", PoseWithCovarianceStamped, self.on_pose, queue_size=10)
        rospy.Subscriber("/mcl_diagnostics", Float32MultiArray, self.on_diag, queue_size=10)

    def on_pose(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_diag(self, msg):
        if len(msg.data) >= 3:
            self.pose_std = float(msg.data[1])

    def wait_pose(self, timeout=20.0):
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.pose is not None:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rate.sleep()
        return False

    def stop(self):
        self.pub.publish(Twist())

    def drive_to(self, gx, gy, budget=90.0):
        """旋转对齐 → 直行 → 到点停车。"""
        rate = rospy.Rate(10)
        start = rospy.get_time()

        cmd = Twist()
        while not rospy.is_shutdown():
            if self.pose is None:
                rate.sleep()
                continue
            x, y, th = self.pose
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < self.pos_tol:
                self.stop()
                return True
            if rospy.get_time() - start > budget:
                self.stop()
                rospy.logwarn("到 %.2f,%.2f 超时（剩余 %.2f m）", gx, gy, dist)
                return False

            # 定位不确定度太大就先停车等待（这是定位闭环该有的保护）
            if self.pose_std > self.max_particles_std:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
                rate.sleep()
                continue

            heading_err = wrap_angle(math.atan2(dy, dx) - th)
            if abs(heading_err) > 0.35:
                # 先原地转
                cmd.linear.x = 0.0
                cmd.angular.z = math.copysign(self.angular, heading_err)
            else:
                cmd.linear.x = max(0.12, self.linear * min(1.0, dist / 0.5))
                cmd.angular.z = 1.2 * heading_err
            self.pub.publish(cmd)
            rate.sleep()
        self.stop()
        return False

    def run(self):
        rospy.loginfo("等待 /mcl_pose ...")
        if not self.wait_pose():
            rospy.logerr("没收到 /mcl_pose，先启动 mcl_node？")
            return 1

        if self.waypoints:
            targets = [tuple(float(v) for v in p) for p in self.waypoints]
        else:
            targets = [(self.goal_x, self.goal_y)]

        for i, (gx, gy) in enumerate(targets):
            rospy.loginfo("航点 %d/%d → (%.2f, %.2f)", i + 1, len(targets), gx, gy)
            self.drive_to(gx, gy)
        self.stop()
        rospy.loginfo("全部航点完成")
        return 0


def main():
    rospy.init_node("goal_driver")
    return GoalDriver().run()


if __name__ == "__main__":
    sys.exit(main())
