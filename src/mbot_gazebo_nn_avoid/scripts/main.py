#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SmartAvoid:
    def __init__(self):
        rospy.init_node('smart_avoid_node')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        # 提取正前方、左前方、右前方的距离
        try:
            front = min(min(msg.ranges[0:15] + msg.ranges[345:360]), 3.5)
            left = min(min(msg.ranges[15:60]), 3.5)
            right = min(min(msg.ranges[300:345]), 3.5)
        except:
            front, left, right = 3.5, 3.5, 3.5

        twist = Twist()

        # 智能避障逻辑：只要前面有东西，就立马朝宽敞的一边打方向
        if front < 0.8:
            twist.linear.x = 0.05
            if left > right:
                twist.angular.z = 1.0  # 左边宽敞往左转
            else:
                twist.angular.z = -1.0 # 右边宽敞往右转
        else:
            twist.linear.x = 0.3
            twist.angular.z = 0.0

        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        node = SmartAvoid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
