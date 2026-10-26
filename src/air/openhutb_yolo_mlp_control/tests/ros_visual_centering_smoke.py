"""Real ROS messages with synthetic inputs. Does not connect to AirSim."""
import json
import math
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from openhutb_yolo_mlp_control.visual_centering_node import VisualCenteringNode


class RosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.controller = VisualCenteringNode(parameter_overrides=[
            Parameter('dry_run', value=self._testMethodName != 'test_active_output')])
        self.peer = Node('centering_test_peer')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.controller)
        self.executor.add_node(self.peer)
        self.images = self.peer.create_publisher(Image, '/openhutb/camera/rgb', qos_profile_sensor_data)
        self.odom = self.peer.create_publisher(Odometry, '/openhutb/odom', qos_profile_sensor_data)
        self.detections = self.peer.create_publisher(Detection2DArray, '/openhutb/detections', 1)
        self.statuses, self.commands, self.active = [], [], []
        self.subs = [
            self.peer.create_subscription(String, '/openhutb/visual_centering/status',
                lambda m: self.statuses.append(json.loads(m.data)), 1),
            self.peer.create_subscription(Twist, '/openhutb/visual_centering/preview_cmd_vel',
                self.commands.append, 1),
            self.peer.create_subscription(Twist, '/openhutb/cmd_vel', self.active.append, 1),
        ]
        self.spin(0.2)

    def tearDown(self):
        self.controller.destroy_node()
        self.peer.destroy_node()
        self.executor.shutdown()

    def spin(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.01)

    def wait_for(self, predicate, timeout=1.0):
        until = time.monotonic()+timeout
        while not predicate() and time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(predicate())

    def frame(self, x=480, empty=False, yaw=0, bad_odom=False, old=False,
              detection_age=0.0, unmatched=False):
        stamp = self.peer.get_clock().now().to_msg()
        odom_stamp = self.peer.get_clock().now().to_msg()
        if old:
            stamp.sec -= 5
        if detection_age:
            ns = stamp.sec*1_000_000_000+stamp.nanosec-int(detection_age*1e9)
            stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'camera'
        image.width, image.height = 640, 480
        self.images.publish(image)
        odom = Odometry()
        odom.header.stamp = odom_stamp
        odom.header.frame_id = 'airsim_ned'
        odom.pose.pose.position.z = -3.0
        odom.pose.pose.orientation.z = math.sin(yaw/2)
        odom.pose.pose.orientation.w = math.cos(yaw/2)
        if bad_odom:
            odom.pose.pose.position.x = float('nan')
        self.odom.publish(odom)
        self.spin(0.04)
        detections = Detection2DArray()
        detections.header = image.header
        if unmatched:
            detections.header.frame_id = 'other_camera'
        if not empty:
            detection = Detection2D()
            center = getattr(detection.bbox.center, 'position', detection.bbox.center)
            center.x, center.y = float(x), 240.0
            detection.bbox.size_x, detection.bbox.size_y = 80.0, 100.0
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = 0.9
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        self.detections.publish(detections)
        self.last_detection = detections
        self.spin(0.08)

    def test_preview_does_not_publish_flight_commands(self):
        self.frame()
        self.wait_for(lambda: self.statuses and self.statuses[-1]['status'] == 'tracking'
                      and self.commands and self.commands[-1].linear.y > 0)
        self.assertEqual(self.statuses[-1]['status'], 'tracking')
        self.assertGreater(self.commands[-1].linear.y, 0)
        self.assertEqual(self.active, [])

    def test_active_output(self):
        self.frame()
        self.wait_for(lambda: self.active and self.active[-1].linear.y > 0)
        self.assertGreater(self.active[-1].linear.y, 0)
        self.assertEqual(self.commands, [])

    def test_empty_frame_stops(self):
        self.frame()
        self.wait_for(lambda: self.commands and self.commands[-1].linear.y > 0)
        self.frame(empty=True)
        self.wait_for(lambda: self.statuses[-1]['status'] == 'target_lost'
                      and self.commands[-1].linear.y == 0)
        self.assertEqual(self.statuses[-1]['status'], 'target_lost')
        self.assertEqual(self.commands[-1].linear.y, 0)

    def test_stale_odom_stops(self):
        self.frame()
        self.spin(0.6)
        self.wait_for(lambda: self.statuses[-1]['status'] == 'odom_stale'
                      and self.commands[-1].linear.y == 0)
        self.assertEqual(self.statuses[-1]['status'], 'odom_stale')
        self.assertEqual(self.commands[-1].linear.y, 0)

    def test_old_capture_rejected(self):
        self.frame(old=True)
        self.assertEqual(self.commands[-1].linear.y, 0)

    def test_unmatched_camera_rejected(self):
        self.frame(unmatched=True)
        self.assertEqual(self.commands[-1].linear.y, 0)

    def test_duplicate_frame_stops_instead_of_refreshing(self):
        self.frame()
        self.wait_for(lambda: self.commands and self.commands[-1].linear.y > 0)
        self.detections.publish(self.last_detection)
        self.wait_for(lambda: self.statuses[-1]['status'] == 'target_lost'
                      and self.commands[-1].linear.y == 0)

    def test_detection_expires_while_odom_continues(self):
        self.frame()
        for _ in range(12):
            odom = Odometry()
            odom.header.frame_id = 'airsim_ned'
            odom.header.stamp = self.peer.get_clock().now().to_msg()
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.z = -3.0
            self.odom.publish(odom)
            self.spin(0.08)
        self.wait_for(lambda: self.statuses[-1]['status'] == 'target_lost'
                      and self.commands[-1].linear.y == 0)

    def test_nan_odom_rejected(self):
        self.frame(bad_odom=True)
        self.assertEqual(self.statuses[-1]['status'], 'odom_stale')

    def test_yaw_rotation(self):
        self.frame(yaw=math.pi/2)
        self.wait_for(lambda: self.commands and self.commands[-1].linear.x < 0)
        self.assertLess(self.commands[-1].linear.x, 0)
        self.assertAlmostEqual(self.commands[-1].linear.y, 0, places=6)


if __name__ == '__main__':
    unittest.main()
