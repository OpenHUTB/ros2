#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务 4：图像端到端 DQN 部署节点

订阅 /drone/image_raw（sensor_msgs/Image, bgr8）与 /drone/state（NED），
把图像缩到 84x84 灰度后与 7 维辅助量拼成状态，交给卷积 DQN 输出动作，
发布到 /drone/cmd_vel。整条链路不经过雷达特征，属于纯视觉端到端控制。
"""

import math
import os
import sys

import cv2
import numpy as np
import rospkg
import rospy
import torch
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)


def _package_path():
    try:
        return rospkg.RosPack().get_path('uav_lidar_dqn_navigation')
    except Exception:
        return os.path.dirname(_SCRIPT_DIR)

from dqn_agent import ImageDQNAgent  # noqa: E402
from dqn_network import ACTIONS, ACTION_NAMES, NUM_ACTIONS  # noqa: E402
from lidar_features import goal_state, velocity_state  # noqa: E402


class DQNE2ENode(object):
    IMG_SIZE = 84

    def __init__(self):
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.model_path = rospy.get_param(
            '~model_path', os.path.join(_package_path(), 'models', 'dqn_e2e.pt'))
        self.goal = (rospy.get_param('~goal_x', 15.0),
                     rospy.get_param('~goal_y', 10.0),
                     rospy.get_param('~goal_z', -4.0))
        self.goal_tolerance = float(rospy.get_param('~goal_tolerance', 1.5))
        self.reached_goal = False
        self.state = None
        self.image = None
        self.last_action = 0

        self.agent = ImageDQNAgent(action_dim=NUM_ACTIONS, extra_dim=7)
        if os.path.exists(self.model_path):
            self.agent.load(self.model_path)
            rospy.loginfo('已加载端到端 DQN 模型 %s', self.model_path)
        else:
            rospy.logwarn('模型不存在，将输出悬停: %s', self.model_path)

        self.cmd_pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.act_pub = rospy.Publisher('/drone/dqn_action', Int32, queue_size=5)
        rospy.Subscriber('/drone/image_raw', Image, self._on_image)
        rospy.Subscriber('/drone/state', Float32MultiArray, self._on_state)

    def _on_state(self, msg):
        if len(msg.data) >= 7:
            self.state = list(msg.data[:7])

    def _on_image(self, msg):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = buf.reshape(msg.height, msg.width, 3)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, (self.IMG_SIZE, self.IMG_SIZE),
                              interpolation=cv2.INTER_AREA)
            self.image = gray.astype(np.float32) / 255.0
        except Exception as exc:  # pragma: no cover
            rospy.logwarn_throttle(5.0, '图像解码失败: %s', exc)

    def _state_pair(self):
        x, y, z, yaw, vx, vy, vz = self.state
        extra = np.concatenate([
            goal_state((x, y, z), self.goal, yaw),
            velocity_state((vx, vy, vz), yaw),
        ]).astype(np.float32)
        return self.image, extra

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.state is None or self.image is None:
                rate.sleep()
                continue

            distance = math.hypot(self.goal[0] - self.state[0],
                                  self.goal[1] - self.state[1])
            if not self.reached_goal and distance <= self.goal_tolerance:
                self.reached_goal = True
                rospy.loginfo('端到端控制到达目标：水平误差 %.2f m，保持悬停', distance)

            action = 0
            if not self.reached_goal and os.path.exists(self.model_path):
                action = self.agent.act(self._state_pair(), epsilon=0.0)

            vx, yaw_rate = ACTIONS[action]
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.linear.z = 0.0
            cmd.angular.z = float(yaw_rate)
            self.cmd_pub.publish(cmd)
            self.act_pub.publish(Int32(data=int(action)))
            self.last_action = int(action)

            if rospy.get_param('~verbose', False):
                rospy.loginfo_throttle(1.0, '端到端动作 %s', ACTION_NAMES[action])
            rate.sleep()


def main():
    rospy.init_node('dqn_e2e')
    DQNE2ENode().run()


if __name__ == '__main__':
    main()
