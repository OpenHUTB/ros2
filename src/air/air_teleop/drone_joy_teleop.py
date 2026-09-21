#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 手柄发布节点：drone_joy_teleop
# 作用：订阅 ROS 标准手柄话题 /joy（sensor_msgs/Joy），把摇杆轴值按标准 Xbox
#       手感映射为 NED 机体坐标系下的速度指令（geometry_msgs/Twist），并按
#       20 Hz 发布到 /drone/cmd_vel。与键盘节点（drone_ros_teleop.py）一样，
#       本节点只负责「手柄 -> ROS 消息」的转换，不直接接触仿真器。
#
# 轴映射（joy 包 / SDL 惯例，见 wiki.ros.org/joy）：
#       左摇杆前后  axes[1] -> vx（前进/后退，上限 2.5 m/s）
#       左摇杆左右  axes[0] -> yaw_rate（左/右偏航，上限 1.0 rad/s）
#       右摇杆前后  axes[4] -> vz（上升/下降，上限 1.5 m/s）
#       右摇杆左右  axes[3] -> vy（左/右平移，上限 1.5 m/s）
#   部分手柄不报扳机轴（共 4 轴），右摇杆回退为 axes[3]/axes[2]。
#
# 坐标系：与桥接节点（drone_ros_node）约定一致，消息按 NED 机体坐标系解释：
#        x 前、y 右、z 指向地面；linear.z < 0 上升、angular.z > 0 右偏航。
#
# 符号约定（SDL/joy 惯例）：左摇杆前推为 -1、右推为 +1、下推为 +1。因此
#       左摇杆前推得到负值，需取负号才让「前推 = 前进（vx 为正）」；其余三轴
#       与 NED 方向同号，无需取反。若你的手柄/驱动前推报正，把 INVERT_VX 改
#       为 False 即可。

import argparse

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class DroneJoyTeleop:
    """订阅 /joy 并按 20 Hz 发布 /drone/cmd_vel 的手柄遥控节点"""

    # ---- 速度上限（NED 机体坐标系）----
    MAX_VX = 2.5                            # 前后线速度上限（m/s）
    MAX_VY = 1.5                            # 横向平移线速度上限（m/s）
    MAX_VZ = 1.5                            # 垂直升降线速度上限（m/s）
    MAX_YAW = 1.0                           # 偏航角速度上限（rad/s）
    DEADZONE = 0.1                          # 摇杆死区：|轴值| 小于该值置 0

    # ---- 标准 Xbox 轴索引（joy 包 / SDL 惯例）----
    AXIS_VX = 1                             # 左摇杆前后（vx）
    AXIS_YAW = 0                            # 左摇杆左右（偏航）
    AXIS_VZ = 4                             # 右摇杆前后（vz），无扳机轴时回退到 3
    AXIS_VY = 3                             # 右摇杆左右（vy），无扳机轴时回退到 2

    # ---- 左摇杆前后轴的符号：SDL 惯例前推为 -1，取负号让前推 = 前进 ----
    INVERT_VX = True

    def __init__(self, rate=20.0):
        self.rate = max(0.1, rate)          # 防止 0 频率导致除零
        self.deadzone = rospy.get_param('~deadzone', self.DEADZONE)

        # 当前速度指令：手柄回调更新、20 Hz 定时器统一发布。rospy 单线程顺序
        # 处理订阅与定时器回调，无需加锁；初始为全零（原地悬停）。
        self.cmd = Twist()

        self.pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.publish_cb)

    # ---------------- 摇杆解析 ----------------

    @staticmethod
    def _axis(axes, index):
        """按索引安全读取轴值，越界返回 0.0"""
        return axes[index] if 0 <= index < len(axes) else 0.0

    def _apply_deadzone(self, value):
        """对单个轴值施加死区：绝对值小于死区则归零，防止摇杆虚位漂移"""
        return 0.0 if abs(value) < self.deadzone else value

    def joy_cb(self, msg):
        """摇杆回调：把 4 个轴映射为 NED 速度指令并缓存，由定时器统一发布"""
        axes = msg.axes
        # 右摇杆索引随轴数量自适应：标准 Xbox（含扳机，>=5 轴）用 4/3，
        # 无扳机轴的手柄（<=4 轴）回退为 3/2。
        if len(axes) >= 5:
            idx_vz, idx_vy = self.AXIS_VZ, self.AXIS_VY
        else:
            idx_vz, idx_vy = 3, 2

        vx = self._apply_deadzone(self._axis(axes, self.AXIS_VX))
        yaw = self._apply_deadzone(self._axis(axes, self.AXIS_YAW))
        vz = self._apply_deadzone(self._axis(axes, idx_vz))
        vy = self._apply_deadzone(self._axis(axes, idx_vy))

        sign = -1.0 if self.INVERT_VX else 1.0
        self.cmd.linear.x = sign * vx * self.MAX_VX     # 前推(负)取反 -> 前进
        self.cmd.linear.y = vy * self.MAX_VY            # 右推(正) -> 右移
        self.cmd.linear.z = vz * self.MAX_VZ            # 前推(负) -> 负 z = 上升
        self.cmd.angular.z = yaw * self.MAX_YAW         # 右推(正) -> 右偏航

    # ---------------- 消息发布 ----------------

    def publish_cb(self, _event):
        """定时回调：按 rate 持续发布当前速度指令（松杆时即为全零悬停）"""
        self.pub.publish(self.cmd)

    # ---------------- 启动提示 ----------------

    def print_help(self):
        """启动时打印轴映射与速度上限对照表"""
        print('=' * 60)
        print('        AirSim 无人机手柄遥控（/joy -> /drone/cmd_vel）')
        print('=' * 60)
        print('  左摇杆前后  (axes[%d])  前进/后退   vx = ±%.1f m/s'
              % (self.AXIS_VX, self.MAX_VX))
        print('  左摇杆左右  (axes[%d])  左/右偏航   yaw = ±%.1f rad/s'
              % (self.AXIS_YAW, self.MAX_YAW))
        print('  右摇杆前后  (axes[%d])  上升/下降   vz = ±%.1f m/s'
              % (self.AXIS_VZ, self.MAX_VZ))
        print('  右摇杆左右  (axes[%d])  左/右平移   vy = ±%.1f m/s'
              % (self.AXIS_VY, self.MAX_VY))
        print('-' * 60)
        print('  死区 %.2f：|轴值| 小于死区视为 0，防止摇杆虚位漂移' % self.deadzone)
        print('  松杆时持续发布全零指令，无人机原地悬停')
        print('  无扳机轴的 4 轴手柄，右摇杆自动回退为 axes[3]/axes[2]')
        print('=' * 60)


def main():
    parser = argparse.ArgumentParser(
        description='AirSim 无人机手柄发布节点：/joy -> /drone/cmd_vel')
    parser.add_argument('--rate', type=float, default=None,
                        help='发布频率（Hz），默认 20')
    args, _ = parser.parse_known_args()        # 兼容 ROS 参数重映射传入的 _xxx:=yyy

    rospy.init_node('drone_joy_teleop')
    rate = args.rate if args.rate is not None else rospy.get_param('~rate', 20.0)

    teleop = DroneJoyTeleop(rate=rate)
    teleop.print_help()
    rospy.loginfo('手柄节点已启动：订阅 /joy，按 %.1f Hz 发布 /drone/cmd_vel',
                  teleop.rate)
    rospy.spin()


if __name__ == '__main__':
    main()
