#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WalkerState:
    def execute(self, node):
        raise NotImplementedError

class ForwardState(WalkerState):
    def __init__(self):
        self.linear_speed = 0.5
        self.min_distance = 0.8

    def execute(self, node):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        if node.obstacle_detected:
            node.current_state = node.rotating_state
            rospy.loginfo("Obstacle detected! Switching to rotation.")
        else:
            node.cmd_pub.publish(cmd)

class RotatingState(WalkerState):
    def __init__(self):
        self.angular_speed = 0.3
        self.rotation_direction = 1.0

    def execute(self, node):
        cmd = Twist()
        cmd.angular.z = self.angular_speed * self.rotation_direction
        if not node.obstacle_detected:
            node.current_state = node.forward_state
            node.rotation_direction *= -1
            rospy.loginfo("Path clear! Switching to forward.")
        else:
            node.cmd_pub.publish(cmd)

class WalkerNode:
    def __init__(self):
        rospy.init_node('walker_node', anonymous=True)
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.3)
        self.min_distance = rospy.get_param('~min_distance', 0.8)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.forward_state = ForwardState()
        self.rotating_state = RotatingState()
        self.current_state = self.forward_state
        self.obstacle_detected = False

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def scan_callback(self, msg):
        front_ranges = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i, r in enumerate(msg.ranges):
            angle = angle_min + i * angle_increment
            if -math.pi/4 <= angle <= math.pi/4:
                if not math.isnan(r) and not math.isinf(r):
                    front_ranges.append(r)
        if front_ranges:
            self.obstacle_detected = min(front_ranges) < self.min_distance
        else:
            self.obstacle_detected = False

    def control_loop(self, event):
        self.current_state.execute(self)

if __name__ == '__main__':
    try:
        walker = WalkerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
