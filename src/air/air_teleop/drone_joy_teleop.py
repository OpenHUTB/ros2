#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 手柄发布节点：drone_joy_teleop（真机轴向校准版）
# 作用：订阅 ROS 标准手柄话题 /joy（sensor_msgs/Joy），把摇杆轴值映射为
#       AirSim 机体坐标系下的速度指令（geometry_msgs/Twist），并按 20 Hz
#       发布到 /drone/cmd_vel。与键盘节点（drone_ros_teleop.py）一样，本节点
#       只负责「手柄 -> ROS 消息」的转换，不直接接触仿真器。
#
# 轴序与符号已按「飞智冰原狼 4（Xbox 协议）+ Linux joy_node + AirSim 飞控底座」
# 硬件在环（HIL）实测标定，关键事实如下：
#   1. Linux Xbox 驱动轴序：
#        axes[0] 左摇杆左右（向左推 +1.0）
#        axes[1] 左摇杆前后（向前推 +1.0）
#        axes[3] 右摇杆左右（向左推 +1.0）
#        axes[5] 右摇杆前后（向前推 +1.0）
#      注意：axes[4] 是 RT 扳机，不是摇杆轴！
#   2. AirSim 飞控符号对齐（缩放系数因此含正负号）：
#        前进 +vx：推前 axes[1] -> +2.5 m/s            （scale_vx = +2.5）
#        偏航左转：左推 axes[0] -> -1.0 rad/s 才向左   （scale_yaw = -1.0）
#        向左侧移：左推 axes[3] -> -1.5 m/s 才向左     （scale_vy = -1.5）
#        垂直爬升：推前 axes[5] -> +1.5 m/s 才上升     （scale_vz = +1.5）
#
# 以上轴索引与缩放系数均为 ROS 私有参数（默认值即标定值），可在 launch/命令行
# 中按实际手柄或驱动微调，无需改动源码。

import argparse

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class DroneJoyTeleop:
    """订阅 /joy 并按 20 Hz 发布 /drone/cmd_vel 的手柄连续遥控节点"""

    # ---- 默认轴索引（飞智冰原狼 4 / Xbox 协议 + joy_node 真机标定）----
    DEFAULT_AXIS = {
        'vx': 1,       # 左摇杆前后（前推 +1.0）-> 前进/后退
        'yaw': 0,      # 左摇杆左右（左推 +1.0）-> 左/右偏航
        'vy': 3,       # 右摇杆左右（左推 +1.0）-> 左/右平移
        'vz': 5,       # 右摇杆前后（前推 +1.0）-> 上升/下降（axes[4] 是 RT 扳机）
    }

    # ---- 默认缩放系数（含符号，直接乘轴值即得速度指令）----
    DEFAULT_SCALE = {
        'vx': 2.5,     # 前推 +1 -> +2.5 m/s 前进
        'yaw': -1.0,   # 左推 +1 -> -1.0 rad/s 向左偏航
        'vy': -1.5,    # 左推 +1 -> -1.5 m/s 向左平移
        'vz': 1.5,     # 前推 +1 -> +1.5 m/s 垂直爬升
    }

    DEFAULT_DEADZONE = 0.1

    def __init__(self, rate=20.0):
        self.rate = max(0.1, rate)          # 防止 0 频率导致除零

        # ---- 从参数服务器读取配置，默认值即真机标定结果 ----
        self.axis = {k: rospy.get_param('~axis_%s' % k, v)
                     for k, v in self.DEFAULT_AXIS.items()}
        self.scale = {k: rospy.get_param('~scale_%s' % k, v)
                      for k, v in self.DEFAULT_SCALE.items()}
        self.deadzone = rospy.get_param('~deadzone', self.DEFAULT_DEADZONE)

        # 当前速度指令：手柄回调更新、20 Hz 定时器统一发布。rospy 单线程顺序
        # 处理订阅与定时器回调，无需加锁；初始为全零（原地悬停）。
        self.cmd = Twist()

        self.pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.publish_cb)

        self.log_config()

    # ---------------- 摇杆解析 ----------------

    def _deadzoned(self, value):
        """对单个轴值施加死区：绝对值小于死区则归零，防止回中漂移"""
        return 0.0 if abs(value) < self.deadzone else value

    def _scaled(self, axes, name):
        """读取指定轴值，施加死区后乘以缩放系数（缩放系数本身含符号）"""
        return self._deadzoned(axes[self.axis[name]]) * self.scale[name]

    def joy_cb(self, msg):
        """摇杆回调：先做数组边界检查，再把 4 个轴映射为速度指令并缓存"""
        # 边界保护：只有轴数组足够长（能容纳所有已配置轴索引）才映射，
        # 防止低维手柄/驱动报告较少轴时索引越界导致节点闪退。
        max_axis = max(self.axis.values())
        if len(msg.axes) <= max_axis:
            rospy.logwarn_throttle(
                5.0, 'joy 消息仅 %d 个轴，低于所需最高索引 %d，忽略本帧',
                len(msg.axes), max_axis)
            return

        self.cmd.linear.x = self._scaled(msg.axes, 'vx')
        self.cmd.linear.y = self._scaled(msg.axes, 'vy')
        self.cmd.linear.z = self._scaled(msg.axes, 'vz')
        self.cmd.angular.z = self._scaled(msg.axes, 'yaw')

    # ---------------- 消息发布 ----------------

    def publish_cb(self, _event):
        """定时回调：按 rate 持续发布当前速度指令（松杆时即为全零悬停）"""
        self.pub.publish(self.cmd)

    # ---------------- 配置说明打印 ----------------

    def log_config(self):
        """启动时打印轴映射、缩放系数与死区等控制说明"""
        rospy.loginfo('手柄连续遥控节点已启动：订阅 /joy -> 发布 /drone/cmd_vel'
                      '（%.1f Hz）', self.rate)
        rospy.loginfo('左摇杆前后  axis[%d] -> vx    scale=%+.1f m/s',
                      self.axis['vx'], self.scale['vx'])
        rospy.loginfo('左摇杆左右  axis[%d] -> yaw   scale=%+.1f rad/s',
                      self.axis['yaw'], self.scale['yaw'])
        rospy.loginfo('右摇杆左右  axis[%d] -> vy    scale=%+.1f m/s',
                      self.axis['vy'], self.scale['vy'])
        rospy.loginfo('右摇杆前后  axis[%d] -> vz    scale=%+.1f m/s',
                      self.axis['vz'], self.scale['vz'])
        rospy.loginfo('死区 %.2f：|轴值| 小于死区归零，防止摇杆回中漂移',
                      self.deadzone)


def main():
    parser = argparse.ArgumentParser(
        description='AirSim 无人机手柄发布节点：/joy -> /drone/cmd_vel')
    parser.add_argument('--rate', type=float, default=None,
                        help='发布频率（Hz），默认 20')
    args, _ = parser.parse_known_args()    # 兼容 ROS 参数重映射传入的 _xxx:=yyy

    rospy.init_node('drone_joy_teleop')
    rate = args.rate if args.rate is not None else rospy.get_param('~rate', 20.0)

    DroneJoyTeleop(rate=rate)
    rospy.spin()


if __name__ == '__main__':
    main()
