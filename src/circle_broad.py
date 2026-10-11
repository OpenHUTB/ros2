import math
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class MyBroad(Node):
    def __init__(self):
        super().__init__("circle_broad")
        self.br = tf2_ros.TransformBroadcaster(self)
        self.t0 = self.get_clock().now().seconds_nanoseconds()[0]
        self.timer = self.create_timer(0.05, self.cb)

    def cb(self):
        now = self.get_clock().now()
        t = now.seconds_nanoseconds()[0] - self.t0
        ang = 0.5 * t
        x = 1.0 * math.cos(ang)
        y = 1.0 * math.sin(ang)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "circle_obj"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.br.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    n = MyBroad()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
