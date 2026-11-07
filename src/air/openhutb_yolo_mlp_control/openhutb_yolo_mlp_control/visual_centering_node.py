"""YOLO detections to bounded horizontal centering commands."""
from collections import OrderedDict
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from .visual_centering import Box, CenteringController, Settings


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class VisualCenteringNode(Node):
    def __init__(self, **kwargs):
        super().__init__('visual_centering', **kwargs)
        defaults = vars(Settings())
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.controller = CenteringController(Settings(**{
            name: self.get_parameter(name).value for name in defaults}))
        for name, value in {
            'dry_run': True, 'control_dt': 0.05,
            'image_topic': '/openhutb/camera/rgb',
            'detections_topic': '/openhutb/detections',
            'odom_topic': '/openhutb/odom',
            'cmd_vel_topic': '/openhutb/cmd_vel',
            'preview_topic': '/openhutb/visual_centering/preview_cmd_vel',
            'status_topic': '/openhutb/visual_centering/status',
        }.items():
            self.declare_parameter(name, value)
        self.dry_run = self.get_parameter('dry_run').value
        dt = self.get_parameter('control_dt').value
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('control_dt must be finite and positive')
        self.dimensions = OrderedDict()
        self.last_detection_stamp = -1
        self.last_odom_stamp = -1
        self.last_status = None
        topic = lambda name: self.get_parameter(name).value
        self.cmd_pub = self.create_publisher(Twist, topic('preview_topic') if
                                             self.dry_run else topic('cmd_vel_topic'), 1)
        self.status_pub = self.create_publisher(String, topic('status_topic'), 1)
        self.subscriptions_ = [
            self.create_subscription(Image, topic('image_topic'), self.image_callback,
                                     qos_profile_sensor_data),
            self.create_subscription(Odometry, topic('odom_topic'), self.odom_callback,
                                     qos_profile_sensor_data),
            self.create_subscription(Detection2DArray, topic('detections_topic'),
                                     self.detection_callback, 1),
        ]
        self.timer = self.create_timer(dt, self.control_step)
        self.get_logger().info(f'Visual centering ready; dry_run={self.dry_run}')

    def fresh(self, stamp, timeout):
        age = (self.get_clock().now().nanoseconds - stamp_ns(stamp)) / 1e9
        return stamp_ns(stamp) > 0 and 0 <= age <= timeout

    def image_callback(self, msg):
        key = (stamp_ns(msg.header.stamp), msg.header.frame_id)
        self.dimensions[key] = (msg.width, msg.height)
        while len(self.dimensions) > 30:
            self.dimensions.popitem(last=False)

    def odom_callback(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if not self.fresh(msg.header.stamp, self.controller.settings.odom_timeout):
            self.controller.pose = None
            return
        if stamp <= self.last_odom_stamp:
            self.controller.pose = None
            return
        self.last_odom_stamp = stamp
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        values = (q.x, q.y, q.z, q.w, p.x, p.y, p.z)
        norm = math.sqrt(sum(v*v for v in values[:4]))
        if (msg.header.frame_id != 'airsim_ned' or
                not all(math.isfinite(v) for v in values) or abs(norm-1) > 0.05):
            self.controller.pose = None
            return
        x, y, z, w = [v/norm for v in values[:4]]
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1, min(1, 2*(w*y-z*x))))
        if max(abs(roll), abs(pitch)) > math.radians(15):
            self.controller.pose = None
            return
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        self.controller.update_pose(p.x, p.y, p.z, yaw, time.monotonic()-age)

    def detection_callback(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        key = (stamp, msg.header.frame_id)
        dims = self.dimensions.get(key)
        self.controller.clear_detection()
        if (dims is None or stamp <= self.last_detection_stamp or
                not self.fresh(msg.header.stamp, self.controller.settings.detection_timeout)):
            return
        self.last_detection_stamp = stamp
        boxes = []
        for detection in msg.detections:
            center = detection.bbox.center
            center = getattr(center, 'position', center)
            for result in detection.results:
                h = result.hypothesis
                boxes.append(Box(center.x, center.y, detection.bbox.size_x,
                                 detection.bbox.size_y, h.score, h.class_id))
        # Use capture age as well as monotonic arrival time; delayed inference
        # must not gain another complete timeout window after delivery.
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        self.controller.update_detections(boxes, *dims, time.monotonic()-age)

    def control_step(self):
        command, status, error = self.controller.step(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = command
        self.cmd_pub.publish(msg)
        report = String()
        report.data = json.dumps({'status': status, 'error_x': error,
                                  'velocity_ned': command, 'dry_run': self.dry_run})
        self.status_pub.publish(report)
        if status != self.last_status:
            if status == 'displacement_limit':
                self.get_logger().warning(
                    f'Visual centering stopped: displacement_limit '
                    f'({self.controller.settings.max_displacement:.1f} m). '
                    'Waiting cannot resume movement; reposition or restart with '
                    'a suitable max_displacement for the simulation scene.')
            else:
                self.get_logger().info(f'Visual centering: {status}')
            self.last_status = status

    def destroy_node(self):
        if rclpy.ok(context=self.context):
            self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualCenteringNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
