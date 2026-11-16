#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1：键盘遥控（ROS 侧节点，不直接接触模拟器）。

通过 /uav/cmd_vel 下发速度，由 airsim_bridge 转成 OpenHUTB 的 NED 指令。

按键
    w/s 前后   a/d 左右   r/f 升降   q/e 偏航
    t 起飞     g 降落     空格 悬停    Ctrl+C 退出
"""

import sys

import rospy
import termios
import tty
import select
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

HELP = """
-----------------------------------------
 w/s 前后    a/d 左右    r/f 升降
 q/e 偏航    t 起飞      g 降落
 空格 悬停   Ctrl+C 退出
-----------------------------------------
"""

KEYMAP = {
    "w": ("linear", "x", 1.0), "s": ("linear", "x", -1.0),
    "a": ("linear", "y", 1.0), "d": ("linear", "y", -1.0),
    "r": ("linear", "z", 1.0), "f": ("linear", "z", -1.0),
    "q": ("angular", "z", 1.0), "e": ("angular", "z", -1.0),
}


def get_key(timeout):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if not dr:
        return ""
    return sys.stdin.read(1)


def main():
    rospy.init_node("uav_keyboard_control")
    speed = float(rospy.get_param("~speed", 2.0))
    yaw_rate = float(rospy.get_param("~yaw_rate", 0.6))
    rate_hz = float(rospy.get_param("~rate", 20.0))

    pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(rate_hz)

    if not sys.stdin.isatty():
        rospy.logwarn("stdin 不是终端，键盘读取会失效；请在交互式终端中运行")
        return

    takeoff = rospy.ServiceProxy("/uav/takeoff", Trigger)
    land = rospy.ServiceProxy("/uav/land", Trigger)

    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    cmd = Twist()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            key = get_key(1.0 / rate_hz)
            if key:
                if key == " ":
                    cmd = Twist()
                    rospy.loginfo("悬停")
                elif key == "t":
                    try:
                        takeoff()
                    except Exception as exc:  # noqa: BLE001
                        rospy.logwarn("起飞服务不可用: %s", exc)
                elif key == "g":
                    try:
                        land()
                    except Exception as exc:  # noqa: BLE001
                        rospy.logwarn("降落服务不可用: %s", exc)
                elif key in KEYMAP:
                    axis, comp, sign = KEYMAP[key]
                    cmd = Twist()
                    value = sign * (yaw_rate if axis == "angular" else speed)
                    vec = getattr(cmd, axis)
                    setattr(vec, comp, value)
            pub.publish(cmd)
            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        pub.publish(Twist())
        print("\n已退出，速度归零")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
