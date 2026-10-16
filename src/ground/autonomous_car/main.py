#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # 非阻塞，超时0.1秒
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    rospy.init_node('hutb_keyboard_control')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10) # 10Hz 发布频率
    speed = 0.2
    turn = 1.0
    x = 0; th = 0
    print("控制你的小车! (W/S/A/D/空格, CTRL-C退出)")
    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w': x = speed; th = 0
            elif key == 'x': x = -speed; th = 0
            elif key == 'a': x = 0; th = turn
            elif key == 'd': x = 0; th = -turn
            elif key == ' ' or key == 's': x = 0; th = 0
            elif key == '\x03': break
            
            twist = Twist()
            twist.linear.x = x
            twist.angular.z = th
            pub.publish(twist) # 每次循环都发布，实现10Hz持续发布
            rate.sleep()
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)