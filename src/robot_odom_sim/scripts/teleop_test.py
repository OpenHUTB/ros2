#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, termios, tty, select

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def keyboard_control():
    rospy.init_node('keyboard_control')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    twist = Twist()
    print("使用 W/A/S/D 控制机器人：W前进 S后退 A左转 D右转（按 Ctrl+C 退出）")

    while not rospy.is_shutdown():
        key = get_key()
        
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if key == 'w':
            twist.linear.x = 0.5
        elif key == 's':
            twist.linear.x = -0.5
        elif key == 'a':
            twist.angular.z = 1.0
        elif key == 'd':
            twist.angular.z = -1.0
        elif key == '\x03':
            break

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass