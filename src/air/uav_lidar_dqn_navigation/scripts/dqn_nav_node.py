#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务 2：DQN 导航策略部署节点

订阅：
  /drone/state           [x, y, z, yaw, vx, vy, vz]（NED）
  /drone/lidar/sectors   24 维扇区最近距离（已归一化）
  /drone/goal            PointStamped，NED 坐标（可选，优先于参数）

发布：
  /drone/cmd_vel         geometry_msgs/Twist（ROS 语义，偏航正方向为左）
  /drone/dqn_action      std_msgs/Int32，当前动作编号

策略每 0.1 s 推理一次；若最近障碍距离低于安全阈值则强制悬停，
避免策略在训练分布外做出危险动作。
"""

import math
import os
import sys

import numpy as np
import rospkg
import rospy
import torch
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float32MultiArray, Int32

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)


def _package_path():
    """rosrun 时脚本位于 devel/lib 下，模型要按包目录取，不能按脚本位置。"""
    try:
        return rospkg.RosPack().get_path('uav_lidar_dqn_navigation')
    except Exception:
        return os.path.dirname(_SCRIPT_DIR)

from dqn_agent import DQNAgent  # noqa: E402
from dqn_network import ACTIONS, ACTION_NAMES, NUM_ACTIONS  # noqa: E402
from lidar_features import goal_state, velocity_state  # noqa: E402
from safety_shield import front_distance_m, recovery_turn, select_action  # noqa: E402


class DQNNavNode(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.model_path = rospy.get_param(
            '~model_path', os.path.join(_package_path(), 'models', 'dqn_nav.pt'))
        # 硬兜底：贴到 0.35 m 才强制悬停；1.0 m 内由安全层屏蔽前向动作
        self.safety_dist = float(rospy.get_param('~safety_dist', 0.35))
        # 建图时降低速度：hector 的扫描匹配假设帧间位移很小
        self.speed_scale = float(rospy.get_param('~speed_scale', 1.0))
        self.goal = (rospy.get_param('~goal_x', 15.0),
                     rospy.get_param('~goal_y', 10.0),
                     rospy.get_param('~goal_z', -4.0))

        self.state = None
        self.sectors = None
        self.last_action = 0
        self.recovery_forward_ticks = 0
        self.stagnant_ticks = 0
        self.last_position = None

        self.agent = DQNAgent(state_dim=32, action_dim=NUM_ACTIONS)
        self.model_loaded = os.path.exists(self.model_path)
        if self.model_loaded:
            self.agent.load(self.model_path)
            rospy.loginfo('已加载 DQN 策略 %s', self.model_path)
        else:
            rospy.logwarn('模型不存在，将输出悬停动作: %s', self.model_path)

        self.cmd_pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.act_pub = rospy.Publisher('/drone/dqn_action', Int32, queue_size=5)
        rospy.Subscriber('/drone/state', Float32MultiArray, self._on_state)
        rospy.Subscriber('/drone/lidar/sectors', Float32MultiArray, self._on_sectors)
        rospy.Subscriber('/drone/goal', PointStamped, self._on_goal)

    def _on_state(self, msg):
        if len(msg.data) >= 7:
            self.state = list(msg.data[:7])

    def _on_sectors(self, msg):
        arr = np.array(msg.data, dtype=np.float32)
        if arr.size == 24:
            self.sectors = arr

    def _on_goal(self, msg):
        self.goal = (msg.point.x, msg.point.y, msg.point.z)
        rospy.loginfo('目标点更新为 NED (%.2f, %.2f, %.2f)', *self.goal)

    def _build_state(self):
        x, y, z, yaw, vx, vy, vz = self.state
        goal = goal_state((x, y, z), self.goal, yaw)
        vel = velocity_state((vx, vy, vz), yaw)
        return np.concatenate([
            self.sectors, goal, vel,
            np.array([self.last_action / float(NUM_ACTIONS)], dtype=np.float32),
        ]).astype(np.float32)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.state is None or self.sectors is None:
                rate.sleep()
                continue

            state = self._build_state()
            action = 0
            if self.model_loaded:
                # 安全层先把会撞的动作屏蔽，再在剩余动作里取 Q 最大
                action = select_action(self.agent.q_values(state), self.sectors)

            # Only the forward field can stop forward flight. A wall at the
            # side must not freeze a clear forward path.
            nearest = front_distance_m(self.sectors)
            position = np.asarray(self.state[:2], dtype=np.float32)
            if self.last_position is not None and np.linalg.norm(
                    position - self.last_position) < 0.01:
                self.stagnant_ticks += 1
            else:
                self.stagnant_ticks = 0
            self.last_position = position

            goal_distance = math.hypot(float(self.state[0]) - float(self.goal[0]),
                                       float(self.state[1]) - float(self.goal[1]))
            if goal_distance <= 1.5:
                action = 0
                self.recovery_forward_ticks = 0
            elif self.model_loaded and nearest < max(1.0, self.safety_dist):
                # A masked forward action alone can leave the vehicle hovering
                # forever. Rotate toward open space, then advance once clear.
                action = recovery_turn(self.sectors)
                self.recovery_forward_ticks = 10
            elif self.model_loaded and self.recovery_forward_ticks:
                action = 1  # forward, rechecked against the live scan each tick
                self.recovery_forward_ticks -= 1
            elif (self.model_loaded and
                  abs(float(self.state[2]) - float(self.goal[2])) < 1.0 and
                  self.stagnant_ticks > int(2.0 * self.rate_hz)):
                # Break a policy that keeps choosing hover/turn on a clear path.
                action = 1
                self.stagnant_ticks = 0

            vx, yaw_rate = ACTIONS[action]
            cmd = Twist()
            cmd.linear.x = float(vx) * self.speed_scale
            cmd.linear.z = 0.0
            cmd.angular.z = float(yaw_rate)
            self.cmd_pub.publish(cmd)
            self.act_pub.publish(Int32(data=int(action)))
            self.last_action = int(action)

            if rospy.get_param('~verbose', False):
                rospy.loginfo_throttle(1.0, '动作 %s | 最近障碍 %.2f m',
                                       ACTION_NAMES[action], nearest)
            rate.sleep()


def main():
    rospy.init_node('dqn_nav')
    DQNNavNode().run()


if __name__ == '__main__':
    main()
