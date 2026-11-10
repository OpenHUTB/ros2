"""仿真节点：差速车动力学 + RGB-D 相机 + 里程计 + /cmd_vel 接口。

订阅 /ugv/cmd_vel（键盘遥控和导航节点都往这里发），按固定周期推进仿真，
发布 /ugv/odom、/ugv/camera/image_raw、/ugv/camera/depth、/ugv/ground_truth。
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

from . import camera, params as P
from .kinematics import DiffDrive, twist_to_wheels
from .world import make_scene


class SimNode(Node):
    def __init__(self):
        super().__init__('ugv_sim')

        self.declare_parameter('scene', 'corridor')
        self.declare_parameter('seed', 0)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('publish_tf', True)

        self.scene = str(self.get_parameter('scene').value)
        self.seed = int(self.get_parameter('seed').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.world = make_scene(self.scene, self.seed)
        self.drive = DiffDrive(self.world.start)
        self.rng = np.random.default_rng(self.seed * 31 + 7)
        self.night = self.scene == 'night'
        self.v = 0.0
        self.w = 0.0
        self.steps = 0

        qos = 10
        self.pub_odom = self.create_publisher(Odometry, '/ugv/odom', qos)
        self.pub_img = self.create_publisher(Image, '/ugv/camera/image_raw', qos)
        self.pub_depth = self.create_publisher(Image, '/ugv/camera/depth', qos)
        self.pub_gt = self.create_publisher(PoseStamped, '/ugv/ground_truth', qos)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', qos)

        self.create_subscription(Twist, '/ugv/cmd_vel', self.on_cmd, qos)
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            '仿真启动：scene=%s seed=%d，%d 个路标，起 (%.1f, %.1f) → 终 (%.1f, %.1f)'
            % (self.scene, self.seed, len(self.world.landmarks),
               self.world.start[0], self.world.start[1],
               self.world.goal[0], self.world.goal[1]))

    # ------------------------------------------------------------------
    def on_cmd(self, msg):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def tick(self):
        dt = 1.0 / self.rate
        self.drive.step(self.v, self.w, dt, self.rng,
                        slip_scale=1.15 if self.night else 1.0)
        self.world.step(dt)
        self.steps += 1

        rgb, depth = camera.render(self.world, self.drive.pose, self.rng,
                                   night=self.night)
        stamp = self._stamp()

        self._pub_image(rgb, stamp)
        self._pub_depth(depth, stamp)
        self._pub_odom(stamp)
        self._pub_gt(stamp)
        if self.publish_tf:
            self._pub_tf(stamp)

    # ------------------------------------------------------------------
    def _pub_image(self, rgb, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.height = int(rgb.shape[0])
        msg.width = int(rgb.shape[1])
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = int(rgb.shape[1] * 3)
        msg.data = np.ascontiguousarray(rgb).tobytes()
        self.pub_img.publish(msg)

    def _pub_depth(self, depth, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.height = int(depth.shape[0])
        msg.width = int(depth.shape[1])
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = int(depth.shape[1] * 4)
        msg.data = np.ascontiguousarray(depth, dtype=np.float32).tobytes()
        self.pub_depth.publish(msg)

    def _pub_odom(self, stamp):
        x, y, th = self.drive.odom
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = float(np.sin(th / 2.0))
        msg.pose.pose.orientation.w = float(np.cos(th / 2.0))
        v_l, v_r = twist_to_wheels(self.v, self.w)
        msg.twist.twist.linear.x = float((v_l + v_r) / 2.0)
        msg.twist.twist.angular.z = float(self.w)
        self.pub_odom.publish(msg)

    def _pub_gt(self, stamp):
        x, y, th = self.drive.pose
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = float(np.sin(th / 2.0))
        msg.pose.orientation.w = float(np.cos(th / 2.0))
        self.pub_gt.publish(msg)

    def _pub_tf(self, stamp):
        tfm = TFMessage()
        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = 'odom'
        tr.child_frame_id = 'base_link'
        x, y, th = self.drive.odom
        tr.transform.translation.x = float(x)
        tr.transform.translation.y = float(y)
        tr.transform.translation.z = 0.0
        tr.transform.rotation.z = float(np.sin(th / 2.0))
        tr.transform.rotation.w = float(np.cos(th / 2.0))
        tfm.transforms = [tr]
        self.pub_tf.publish(tfm)


def main(argv=None):
    rclpy.init(args=argv)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
