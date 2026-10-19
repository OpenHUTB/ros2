#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""决策节点：滞回阈值紧急制动控制器（三节点架构之二）

订阅 /ultrasonic_distance；
  车头距离 < BRAKE_DIST   → 进入制动（触发）
  车头距离 > RELEASE_DIST → 解除制动（滞回带，防止在阈值边界来回抖振）

发布 /brake_cmd（给仿真执行节点）与 /brake_status（给监视节点）。
两个话题均按控制帧持续发布，订阅者任意时刻加入都能取到当前状态。
"""
import rospy
from std_msgs.msg import Float32, Bool

BRAKE_DIST = 0.3     # 触发阈值（m）
RELEASE_DIST = 0.5   # 解除阈值（m），必须大于触发阈值以构成滞回


class BrakeController:
    def __init__(self):
        self.braking = False
        self.pub_cmd = rospy.Publisher('/brake_cmd', Bool, queue_size=10)
        self.pub_status = rospy.Publisher('/brake_status', Bool, queue_size=10)
        rospy.Subscriber('/ultrasonic_distance', Float32, self._on_dist, queue_size=10)
        print("决策节点已启动：触发 %.2fm / 解除 %.2fm（滞回）" % (BRAKE_DIST, RELEASE_DIST))

    def _on_dist(self, msg):
        d = msg.data
        if not self.braking and d < BRAKE_DIST:
            self.braking = True
            rospy.logwarn("警告！车头距悬崖 %.2fm，触发紧急制动" % d)
        elif self.braking and d > RELEASE_DIST:
            self.braking = False
            rospy.loginfo("距离恢复至 %.2fm，解除制动" % d)
        self.pub_cmd.publish(self.braking)
        self.pub_status.publish(self.braking)


if __name__ == '__main__':
    rospy.init_node('brake_controller', anonymous=True)
    BrakeController()
    rospy.spin()
