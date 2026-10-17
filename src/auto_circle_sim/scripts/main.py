#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
import math

def auto_circle():
    rospy.init_node('auto_circle_sim', anonymous=True)
    pub = rospy.Publisher('/vehicle_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)

    x, y, theta = 0.0, 0.0, 0.0
    linear_speed = 0.5
    angular_speed = 0.5

    while not rospy.is_shutdown():
        theta += angular_speed * 0.1
        x += linear_speed * math.cos(theta) * 0.1
        y += linear_speed * math.sin(theta) * 0.1

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
        marker.pose.orientation.z = math.sin(theta / 2)
        marker.pose.orientation.w = math.cos(theta / 2)
        marker.scale.x = 1.5
        marker.scale.y = 0.8
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.a = 1.0
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        auto_circle()
    except rospy.ROSInterruptException:
        pass
