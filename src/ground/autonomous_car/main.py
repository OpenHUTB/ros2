#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
控制你的小车!
---------------------------
移动:
   w
a  s  d
   x
w/x : 增加/减少线速度
a/d : 增加/减少角速度
空格/S : 强制停止
CTRL-C 退出
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('hutb_keyboard_control')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    speed = 0.2
    turn = 1.0
    x = 0; th = 0
    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' : x = speed; th = 0
            elif key == 'x' : x = -speed; th = 0
            elif key == 'a' : x = 0; th = turn
            elif key == 'd' : x = 0; th = -turn
            elif key == ' ' or key == 's' : x = 0; th = 0
            elif key == '\x03': break
            twist = Twist()
            twist.linear.x = x; twist.angular.z = th
            pub.publish(twist)
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)