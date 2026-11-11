"""视觉 SLAM 节点：RGB-D → 路标检测 → EKF-SLAM → 占据栅格。

发布 /ugv/map（OccupancyGrid）、/ugv/slam_pose、/ugv/landmarks，
以及 map→odom 的 TF 修正。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import (Pose, PoseArray, PoseWithCovarianceStamped,
                               TransformStamped)
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

from . import detector, params as P
from .gridmap import GridMap
from .kinematics import wrap
from .slam import EKFSLAM


class SlamNode(Node):
    def __init__(self):
        super().__init__('ugv_slam')

        self.declare_parameter('map_rate_hz', 2.0)
        self.declare_parameter('occ_threshold', 0.62)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('start_x', 2.0)
        self.declare_parameter('start_y', 2.0)
        self.declare_parameter('start_yaw', 0.0)

        self.map_rate = float(self.get_parameter('map_rate_hz').value)
        self.occ_th = float(self.get_parameter('occ_threshold').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        start = (float(self.get_parameter('start_x').value),
                 float(self.get_parameter('start_y').value),
                 float(self.get_parameter('start_yaw').value))

        self.slam = EKFSLAM(start)
        self.grid = GridMap()
        self._rgb = None
        self._depth = None
        self._odom = None
        self._odom_done = None
        self._stamp = None
        self.n_frames = 0
        self.n_obs = 0

        qos = 10
        self.create_subscription(Image, '/ugv/camera/image_raw', self.on_image, qos)
        self.create_subscription(Image, '/ugv/camera/depth', self.on_depth, qos)
        self.create_subscription(Odometry, '/ugv/odom', self.on_odom, qos)

        self.pub_map = self.create_publisher(OccupancyGrid, '/ugv/map', qos)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/ugv/slam_pose', qos)
        self.pub_lm = self.create_publisher(PoseArray, '/ugv/landmarks', qos)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', qos)
        self.create_timer(1.0 / self.map_rate, self.publish_map)

        self.get_logger().info('SLAM 节点启动，初始位姿 (%.2f, %.2f, %.2f)' % start)

    # ------------------------------------------------------------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        self._odom = np.array([p.x, p.y, yaw])

    def on_depth(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if arr.size != msg.height * msg.width:
            return
        self._depth = arr.reshape(msg.height, msg.width)

    def on_image(self, msg):
        if self._depth is None or self._odom is None:
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if arr.size != msg.height * msg.width * 3:
            return
        rgb = arr.reshape(msg.height, msg.width, 3)
        depth = self._depth
        if depth.shape[:2] != rgb.shape[:2]:
            return

        # 里程计增量（车体系）
        if self._odom_done is None:
            dxy = np.zeros(3)
        else:
            dx = self._odom[0] - self._odom_done[0]
            dy = self._odom[1] - self._odom_done[1]
            c, s = math.cos(self._odom_done[2]), math.sin(self._odom_done[2])
            dxy = np.array([c * dx + s * dy, -s * dx + c * dy,
                            wrap(self._odom[2] - self._odom_done[2])])
        self._odom_done = self._odom.copy()

        self.slam.predict(float(dxy[0]), float(dxy[1]), float(dxy[2]))
        lms, wall = detector.detect(rgb, depth)
        self.slam.update(lms)
        self.grid.update(self.slam.pose, wall)
        for mx, my, _lid in self.slam.landmark_xy():
            self.grid.mark_obstacle(mx, my)

        self.n_frames += 1
        self.n_obs += len(lms)
        # 直接在图像回调里打日志：放 2Hz 的 publish_map 定时器里会被
        # “10Hz 帧 × 2Hz 定时器”的采样错位漏掉（n_frames 每 0.5s 恰好 +5，
        # 定时器只能看到 4 个固定余数，大概率永远撞不上 %20==0）
        if self.n_frames % 100 == 0:
            self.get_logger().info(
                '帧 %d：观测 %d 次，路标 %d 个，已探索 %.0f%%'
                % (self.n_frames, self.n_obs, len(self.slam.ids),
                   100.0 * self.grid.coverage()))
        self._stamp = msg.header.stamp
        self.publish_pose()

    # ------------------------------------------------------------------
    def publish_pose(self):
        pose = self.slam.pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._stamp if self._stamp is not None else self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        msg.pose.pose.orientation.z = float(math.sin(pose[2] / 2.0))
        msg.pose.pose.orientation.w = float(math.cos(pose[2] / 2.0))
        cov = np.zeros(36)
        cov[0] = float(self.slam.S[0, 0])
        cov[1] = float(self.slam.S[0, 1])
        cov[6] = float(self.slam.S[1, 0])
        cov[7] = float(self.slam.S[1, 1])
        cov[35] = float(self.slam.S[2, 2])
        msg.pose.covariance = cov.tolist()
        self.pub_pose.publish(msg)

        if self.publish_tf and self._odom is not None:
            tfm = TFMessage()
            tr = TransformStamped()
            tr.header.stamp = msg.header.stamp
            tr.header.frame_id = 'map'
            tr.child_frame_id = 'odom'
            # T_map_odom = T_map_base * inv(T_odom_base)
            dx = pose[0] - self._odom[0]
            dy = pose[1] - self._odom[1]
            c, s = math.cos(-self._odom[2]), math.sin(-self._odom[2])
            ox = c * dx - s * dy
            oy = s * dx + c * dy
            oth = wrap(pose[2] - self._odom[2])
            tr.transform.translation.x = float(ox)
            tr.transform.translation.y = float(oy)
            tr.transform.rotation.z = float(math.sin(oth / 2.0))
            tr.transform.rotation.w = float(math.cos(oth / 2.0))
            tfm.transforms = [tr]
            self.pub_tf.publish(tfm)

    def publish_map(self):
        prob = self.grid.prob()
        unknown = np.abs(self.grid.lo) < 1e-6
        data = np.where(unknown, -1, np.where(prob > self.occ_th, 100, 0)).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(P.GRID_RES)
        msg.info.width = int(P.GW)
        msg.info.height = int(P.GH)
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.ravel().tolist()
        self.pub_map.publish(msg)

        lms = PoseArray()
        lms.header.stamp = msg.header.stamp
        lms.header.frame_id = 'map'
        for mx, my, _lid in self.slam.landmark_xy():
            p = Pose()
            p.position.x = float(mx)
            p.position.y = float(my)
            p.orientation.w = 1.0
            lms.poses.append(p)
        self.pub_lm.publish(lms)


def main(argv=None):
    rclpy.init(args=argv)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
