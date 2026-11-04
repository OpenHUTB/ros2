#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist

class SwarmControlNode(Node):
    def __init__(self):
        super().__init__("swarm_control_node")
        self.target_sub = self.create_subscription(
            PointStamped, "/leader_target", self.target_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.kp_ang = 0.003
        self.kp_lin = 0.0015
        self.img_center = 320

    def target_callback(self, msg):
        twist = Twist()
        cx = msg.point.x
        area = msg.point.z
        if area > 10:
            err = cx - self.img_center
            twist.angular.z = -self.kp_ang * err
            twist.linear.x = 0.2 - abs(err)*self.kp_lin
            if twist.linear.x < 0.05:
                twist.linear.x = 0.05
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = SwarmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
