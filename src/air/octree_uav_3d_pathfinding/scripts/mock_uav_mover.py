#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证用的"假载具"（P6）。

## 为什么需要它

`path_follower` 靠位姿反馈判断"该追哪个航点"，而位姿原本由 `airsim_bridge`
从 AirSim 取来、发布成 `/ground_truth/odom`。用 `bridge:=false` 离线跑时没有这个
话题，跟踪节点一步都不会动。

本节点补上这一环：订阅 `/uav/goal_cmd`，按固定速度直线飞向目标，把位置发布成
`/ground_truth/odom` 与 `world → base_link` 的 TF。

## 它是什么、不是什么

* **是**：一个只有运动学的运动学桩 —— 匀速直线、到达即停、无加速度、无姿态变化。
* **不是**：动力学模型、也不是碰撞检测。它不会因为撞到障碍而停下，
  所以**不能**用它验证"路径是否安全"，只能验证**跟踪逻辑**是否走完全程。

因此它只用于离线跑通链路与记录 `reached / duration / max_dev` 这类跟踪指标，
真实飞行仍以 AirSim 为准。

## 跑法

    rosrun octree_uav_3d_pathfinding mock_uav_mover.py

参数（私有，可用 `_name:=value` 覆盖）：
    ~speed          3.0        飞行速度（m/s），与桥接节点的 goal_velocity 对齐
    ~rate           20.0       位姿发布频率（Hz），与桥接节点对齐
    ~goal_topic     /uav/goal_cmd
    ~odom_topic     /ground_truth/odom
    ~world_frame    world
    ~body_frame     base_link
    ~start_x/y/z    0/0/0      初始位置（ROS 世界系 ENU）
"""

import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class MockUavMover(object):
    def __init__(self):
        self.speed = rospy.get_param('~speed', 3.0)
        self.rate = rospy.get_param('~rate', 20.0)
        self.goal_topic = rospy.get_param('~goal_topic', '/uav/goal_cmd')
        self.odom_topic = rospy.get_param('~odom_topic', '/ground_truth/odom')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.body_frame = rospy.get_param('~body_frame', 'base_link')

        self.pos = [
            rospy.get_param('~start_x', 0.0),
            rospy.get_param('~start_y', 0.0),
            rospy.get_param('~start_z', 0.0),
        ]
        self.target = list(self.pos)      # 初始目标 = 当前位置，静止
        self.vel = [0.0, 0.0, 0.0]
        self.lock = __import__('threading').Lock()

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()
        rospy.Subscriber(self.goal_topic, Point, self.goal_cb, queue_size=1)

        rospy.loginfo('mock_uav_mover started (离线运动学桩)')
        rospy.loginfo('  goal topic : %s', self.goal_topic)
        rospy.loginfo('  odom topic : %s (%.0f Hz)', self.odom_topic, self.rate)
        rospy.loginfo('  speed      : %.1f m/s（匀速直线，无加速度）', self.speed)
        rospy.loginfo('  start      : (%.2f, %.2f, %.2f)',
                      self.pos[0], self.pos[1], self.pos[2])
        rospy.loginfo('  ⚠ 只有运动学：不做碰撞判定，不能用来验证路径安全')

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)

    def goal_cb(self, msg):
        with self.lock:
            self.target = [msg.x, msg.y, msg.z]
        rospy.loginfo('收到航点 (%.2f, %.2f, %.2f)', msg.x, msg.y, msg.z)

    def tick(self, _event):
        dt = 1.0 / self.rate
        with self.lock:
            dx = self.target[0] - self.pos[0]
            dy = self.target[1] - self.pos[1]
            dz = self.target[2] - self.pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            step = self.speed * dt

            if dist <= step or dist < 1e-6:
                # 到达：直接落到目标并停住
                self.pos = list(self.target)
                self.vel = [0.0, 0.0, 0.0]
            else:
                k = step / dist
                self.vel = [dx / dist * self.speed,
                            dy / dist * self.speed,
                            dz / dist * self.speed]
                self.pos[0] += dx * k
                self.pos[1] += dy * k
                self.pos[2] += dz * k

            stamp = rospy.Time.now()
            px, py, pz = self.pos
            vx, vy, vz = self.vel

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.w = 1.0     # 不模拟姿态：始终保持单位四元数
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.body_frame
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node('mock_uav_mover')
    MockUavMover()
    rospy.spin()


if __name__ == '__main__':
    main()
