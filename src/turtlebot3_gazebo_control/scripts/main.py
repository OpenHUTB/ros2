#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# 键盘控制说明
msg = """
Control Your Turtlebot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

# 按键与速度映射表 (按键: [线速度系数, 角速度系数])
moveBindings = {
    'w': (1, 0),   # 前进
    'x': (-1, 0),  # 后退
    'a': (0, 1),   # 左转
    'd': (0, -1),  # 右转
    's': (0, 0),   # 停止
    ' ': (0, 0)    # 空格停止
}

def getKey():
    """读取终端单个按键输入"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    """打印当前速度信息"""
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    # 保存终端设置，程序退出时需要还原
    settings = termios.tcgetattr(sys.stdin)
    
    # 初始化 ROS 节点
    rospy.init_node('turtlebot3_keyboard_control')
    # 发布 /cmd_vel 话题，消息类型为 Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # 速度参数
    speed = 0.2
    turn = 1.0
    
    x = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        
        # 主循环
        while not rospy.is_shutdown():
            key = getKey()
            
            # 根据按键更新速度系数
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            else:
                x = 0
                th = 0
                # 如果按下 Ctrl+C，退出循环
                if (key == '\x03'):
                    break

            # 创建并发布 Twist 消息
            twist = Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # 程序退出时，发布停止指令，避免小车继续移动
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)