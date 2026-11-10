"""订阅 /odom，发 odom->base_link TF。"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfNode(Node):
    def __init__(self):
        super().__init__("ugv_odom_tf")
        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._cb, 10)

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main():
    rclpy.init()
    n = OdomTfNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
