import math
import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from forklift_slam_nav.simulation_math import advance_planar_pose, yaw_from_freejoint


class MujocoRobot(Node):
    def __init__(self):
        super().__init__('mujoco_planar_robot')
        model_path = get_package_share_directory('forklift_slam_nav') + '/worlds/planar_forklift.xml'
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'laser')
        self.qpos = self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'base_free')]
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_pub = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
        t.transform.translation.x = 0.1
        t.transform.translation.z = 0.04
        t.transform.rotation.w = 1.0
        static.sendTransform(t)
        self.timer = self.create_timer(0.05, self.tick)

    def command(self, msg):
        self.speed = max(-0.2, min(0.35, msg.linear.x))
        self.yaw_rate = max(-1.2, min(1.2, msg.angular.z))

    def tick(self):
        dt = 0.05
        yaw = yaw_from_freejoint(self.data.qpos, self.qpos)
        x, y, yaw = advance_planar_pose(
            self.data.qpos[self.qpos],
            self.data.qpos[self.qpos + 1],
            yaw,
            self.speed,
            self.yaw_rate,
            dt,
        )
        self.data.qpos[self.qpos] = x
        self.data.qpos[self.qpos + 1] = y
        self.data.qpos[self.qpos + 2] = 0.22
        self.data.qpos[self.qpos + 3:self.qpos + 7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        self.data.qvel[self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'base_free')]:] = 0
        mujoco.mj_forward(self.model, self.data)
        stamp = self.get_clock().now().to_msg()
        x, y = self.data.qpos[self.qpos:self.qpos + 2]
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.translation.z = float(self.data.qpos[self.qpos + 2])
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'odom', 'base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = float(x), float(y)
        od.pose.pose.position.z = tf.transform.translation.z
        od.pose.pose.orientation = tf.transform.rotation
        od.twist.twist.linear.x, od.twist.twist.angular.z = self.speed, self.yaw_rate
        self.odom_pub.publish(od)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser'
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 360
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time, scan.time_increment = 0.08, 8.0, 0.05, 0.0
        origin = self.data.site_xpos[self.site_id].copy()
        yaw_mat = self.data.site_xmat[self.site_id].reshape(3, 3)
        ranges = []
        for angle in scan.angle_min + np.arange(360) * scan.angle_increment:
            local = np.array([math.cos(angle), math.sin(angle), 0.0])
            direction = yaw_mat @ local
            dist = mujoco.mj_ray(self.model, self.data, origin, direction, None, 1, self.body_id, None)
            ranges.append(float(dist) if dist >= 0 else float('inf'))
        scan.ranges = ranges
        scan.intensities = [0.0] * 360
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
