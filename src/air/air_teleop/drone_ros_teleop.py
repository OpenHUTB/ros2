#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 键盘发布节点：drone_ros_teleop
# 作用：用 pynput 监听键盘按键，把当前按住的按键组合翻译成 NED 机体坐标系下的
#       速度指令（geometry_msgs/Twist），并按 10 Hz 发布到 /drone/cmd_vel。
#       该节点不直接接触仿真器，只负责“键盘 -> ROS 消息”的转换。
#
# 坐标系：与桥接节点（drone_ros_node）约定一致，消息按 NED 机体坐标系解释：
#         x 前、y 右、z 指向地面；linear.z < 0 上升、angular.z > 0 右偏航。
#
# 按键松开后对应轴速度立即归零（每个周期都按当前按键集合重新计算）；
# 无任何按键时持续发布全零指令，表示“原地保持”。

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard as pynput_keyboard


class DroneTeleop:
    """监听键盘并按 10 Hz 发布 /drone/cmd_vel 的 ROS 节点"""

    # ---- 标称速度：按键对照表与速度计算共用同一份参数 ----
    LINEAR_SPEED = 3.0                            # 水平标称线速度 3.0 m/s
    VERTICAL_SPEED = 2.0                          # 垂直标称速度 2.0 m/s
    YAW_RATE_DEG = 30.0                           # 偏航角速度 30 度/秒

    # ---- 按键映射表：内部键名 -> 功能说明 ----
    KEYMAP = {
        'w': '前进', 's': '后退',
        'a': '左移', 'd': '右移',
        'space': '上升', 'down': '下降',          # down 即 Ctrl+P
        'j': '左偏航', 'l': '右偏航',
    }

    def __init__(self, rate=10.0):
        self.rate = max(0.1, rate)                # 防止 0 频率导致除零
        self.yaw_rate = math.radians(self.YAW_RATE_DEG)   # 偏航角速度，弧度/秒

        # 按键状态：pynput 监听线程写入、ROS 定时器线程读取
        self.lock = threading.Lock()
        self.pressed = set()                      # 当前按住的内部键名集合
        self.ctrl_down = False                    # 左/右 Ctrl 是否按下（组合键 Ctrl+P 用）
        self.exit_requested = False               # 是否收到 ESC
        self.stopped = False                      # 安全收尾是否已完成

        self.pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.publish_cb)
        self.listener = pynput_keyboard.Listener(on_press=self.on_press,
                                                 on_release=self.on_release)
        rospy.on_shutdown(self.stop)

    # ---------------- 按键监听 ----------------

    def key_name(self, key):
        """把 pynput 的 Key 归一化成内部键名；无法识别的按键返回 None"""
        if key == pynput_keyboard.Key.space:
            return 'space'
        if key == pynput_keyboard.Key.esc:
            return 'esc'
        if hasattr(key, 'char') and key.char:
            ch = key.char.lower()
            if ch == 'p' and self.ctrl_down:      # Ctrl+P：终端无法单独捕获组合键，这里用状态判断
                return 'down'
            return ch
        return None

    def on_press(self, key):
        """按键按下：记录键名；ESC 只置退出标志，由定时器线程执行安全退出"""
        if key in (pynput_keyboard.Key.ctrl_l, pynput_keyboard.Key.ctrl_r):
            self.ctrl_down = True
            return
        name = self.key_name(key)
        if name is None:
            return
        with self.lock:
            if name == 'esc':
                self.exit_requested = True
            else:
                self.pressed.add(name)

    def on_release(self, key):
        """按键松开：从按住集合中移除对应键名"""
        if key in (pynput_keyboard.Key.ctrl_l, pynput_keyboard.Key.ctrl_r):
            self.ctrl_down = False
            return
        name = self.key_name(key)
        if name is None:
            return
        with self.lock:
            self.pressed.discard(name)

    # ---------------- 消息发布 ----------------

    def build_twist(self, pressed):
        """把当前按住的按键集合翻译成 NED 机体坐标系下的 Twist 指令

        linear.x 前、linear.y 右、linear.z 下（负值上升）；
        angular.z 偏航角速度弧度/秒，正值右偏航（顺时针，从上方看）。
        """
        msg = Twist()
        if 'w' in pressed:                        # 前进
            msg.linear.x += self.LINEAR_SPEED
        if 's' in pressed:                        # 后退
            msg.linear.x -= self.LINEAR_SPEED
        if 'd' in pressed:                        # 右移（NED y 轴向右）
            msg.linear.y += self.LINEAR_SPEED
        if 'a' in pressed:                        # 左移
            msg.linear.y -= self.LINEAR_SPEED
        if 'space' in pressed:                    # 上升（NED z 轴向下，上升为负）
            msg.linear.z -= self.VERTICAL_SPEED
        if 'down' in pressed:                     # 下降
            msg.linear.z += self.VERTICAL_SPEED
        if 'j' in pressed:                        # 左偏航（逆时针，负角速度）
            msg.angular.z -= self.yaw_rate
        if 'l' in pressed:                        # 右偏航（顺时针，正角速度）
            msg.angular.z += self.yaw_rate
        return msg

    def publish_cb(self, _event):
        """定时回调：发布当前按键状态对应的 Twist；收到 ESC 后触发安全退出"""
        if self.exit_requested:
            # 不在 pynput 监听线程中直接 shutdown（on_shutdown 会 join 监听线程自身）
            rospy.signal_shutdown('收到 ESC 按键，退出键盘遥控')
            return
        with self.lock:
            pressed = set(self.pressed)
        self.pub.publish(self.build_twist(pressed))

    # ---------------- 生命周期 ----------------

    @staticmethod
    def print_help():
        """启动时打印按键对照表"""
        print('=' * 60)
        print('            AirSim 无人机键盘遥控（ROS 消息解耦）')
        print('=' * 60)
        print('  W / S      前进 / 后退         (±%.1f m/s 机体 x 轴)' % DroneTeleop.LINEAR_SPEED)
        print('  A / D      左移 / 右移         (±%.1f m/s 机体 y 轴)' % DroneTeleop.LINEAR_SPEED)
        print('  Space      垂直上升           (NED linear.z = -%.1f m/s)' % DroneTeleop.VERTICAL_SPEED)
        print('  Ctrl+P     垂直下降           (NED linear.z = +%.1f m/s)' % DroneTeleop.VERTICAL_SPEED)
        print('  J / L      原地左偏航 / 右偏航 (±%.0f 度/秒)' % DroneTeleop.YAW_RATE_DEG)
        print('  ESC        退出键盘遥控（桥接节点 0.5 s 超时后自动悬停）')
        print('-' * 60)
        print('  按键松开后对应轴速度自动归零；桥接节点收到第一条指令后自动起飞')
        print('=' * 60)

    def start(self):
        """启动监听并进入 ROS 循环"""
        self.print_help()
        self.listener.start()
        rospy.loginfo('键盘节点已启动：按 %.1f Hz 发布 /drone/cmd_vel', self.rate)
        rospy.spin()

    def stop(self):
        """安全收尾：停止按键监听（可能被 rospy.on_shutdown 与 finally 各调用一次）"""
        if self.stopped:
            return
        self.stopped = True
        if self.listener.running:
            self.listener.stop()
        rospy.loginfo('键盘节点已退出，停止发布速度指令')


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='AirSim 无人机键盘发布节点：键盘 -> /drone/cmd_vel')
    parser.add_argument('--rate', type=float, default=None, help='发布频率（Hz），默认 10')
    args, _ = parser.parse_known_args()               # 兼容 ROS 参数重映射传入的 _xxx:=yyy

    rospy.init_node('drone_ros_teleop')
    rate = args.rate if args.rate is not None else rospy.get_param('~rate', 10.0)

    teleop = DroneTeleop(rate=rate)
    try:
        teleop.start()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()


if __name__ == '__main__':
    main()
