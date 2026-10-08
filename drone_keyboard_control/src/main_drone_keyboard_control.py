@@ -0,0 +1,56 @@
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

CONTROL_TOPIC = "/cmd_vel"
LINEAR_SPEED = 3.0
ANGULAR_SPEED = 1.0

def get_key():
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSANOW, old)
    return key

def run():
    rospy.init_node("drone_keyboard_control")
    pub = rospy.Publisher(CONTROL_TOPIC, Twist, queue_size=10)
    cmd = Twist()
    rate = rospy.Rate(30)
    
    rospy.loginfo("键盘控制启动 | w前进 s后退 a左转 d右转 q上升 e下降  按Ctrl+C退出")
    
    while not rospy.is_shutdown():
        key = get_key()
        if key == 'w':
            cmd.linear.x = LINEAR_SPEED
        elif key == 's':
            cmd.linear.x = -LINEAR_SPEED
        elif key == 'a':
            cmd.angular.z = ANGULAR_SPEED
        elif key == 'd':
            cmd.angular.z = -ANGULAR_SPEED
        elif key == 'q':
            cmd.linear.z = LINEAR_SPEED
        elif key == 'e':
            cmd.linear.z = -LINEAR_SPEED
        elif key == ' ':
            cmd = Twist()
        elif key == '\x03':
            break
        
        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
