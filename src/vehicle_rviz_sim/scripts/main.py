#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
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
    rospy.init_node('vehicle_rviz_sim', anonymous=True)
    pub = rospy.Publisher('/vehicle_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)

    x, y, theta = 0.0, 0.0, 0.0
    print("使用 W/A/S/D 控制车辆：W前进 S后退 A左转 D右转")

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'w':
            x += 0.2
        elif key == 's':
            x -= 0.2
        elif key == 'a':
            theta += 0.2
        elif key == 'd':
            theta -= 0.2
        elif key == '\x03':
            break

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.pose.orientation.z = theta
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.5
        marker.scale.y = 0.8
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.a = 1.0
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass
